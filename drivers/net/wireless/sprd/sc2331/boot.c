// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mmc/sdio_func.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#include "boot.h"
#include "protocol.h"

#define SC2331_FDL_NAME "sprd/sc2331-fdl.bin"
#define SC2331_FDL_ADDRESS 0x80000000
#define SC2331_FDL_SIZE 8192

struct sc2331_boot {
	struct serdev_device *serdev;
	struct regulator *supply;
	struct clk *clock;
	struct gpio_desc *reset;
	struct gpio_desc *host_wake;
	struct gpio_desc *device_wake;
	unsigned long wake_levels;
	struct completion received;
	struct completion ready;
	bool ready_armed;
	spinlock_t lock; /* RX */
	struct sc2331_uart_stream rx;
	int rx_status;
	bool powered;
	bool clocked;
	u8 tx[2062];
};

static irqreturn_t sc2331_boot_wake(int irq, void *data)
{
	struct sc2331_boot *boot = data;
	unsigned long flags;
	int level = gpiod_get_value_cansleep(boot->host_wake);

	if (level < 0)
		return IRQ_NONE;
	if (!(boot->wake_levels & BIT(level))) {
		boot->wake_levels |= BIT(level);
		dev_info(&boot->serdev->dev, "Host wake IRQ level: %d\n", level);
	}
	spin_lock_irqsave(&boot->lock, flags);
	if (!level && boot->ready_armed) {
		boot->ready_armed = false;
		complete(&boot->ready);
	}
	spin_unlock_irqrestore(&boot->lock, flags);
	return IRQ_HANDLED;
}

void sc2331_boot_arm(struct sc2331_boot *boot)
{
	unsigned long flags;

	spin_lock_irqsave(&boot->lock, flags);
	reinit_completion(&boot->ready);
	boot->ready_armed = true;
	spin_unlock_irqrestore(&boot->lock, flags);
}

int sc2331_boot_wait_ready(struct sc2331_boot *boot)
{
	return wait_for_completion_timeout(&boot->ready, msecs_to_jiffies(2000)) ?
		0 : -ETIMEDOUT;
}

void sc2331_boot_sleep_device(struct sc2331_boot *boot)
{
	if (boot->device_wake)
		gpiod_set_value_cansleep(boot->device_wake, 0);
}

int sc2331_boot_wake_device(struct sc2331_boot *boot)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(100);
	int level, ret = -ETIMEDOUT;

	if (!boot->device_wake || !boot->host_wake)
		return -ENODEV;
	gpiod_set_value_cansleep(boot->device_wake, 1);
	do {
		level = gpiod_get_value_cansleep(boot->host_wake);
		if (level > 0) {
			sc2331_boot_sleep_device(boot);
			return 0;
		}
		if (level < 0) {
			ret = level;
			break;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	sc2331_boot_sleep_device(boot);
	return ret;
}

static size_t sc2331_boot_receive(struct serdev_device *serdev,
				  const u8 *data, size_t count)
{
	struct sc2331_boot *boot = serdev_device_get_drvdata(serdev);
	unsigned long flags;
	size_t i;
	int ret;

	spin_lock_irqsave(&boot->lock, flags);
	for (i = 0; i < count && !boot->rx_status; i++) {
		ret = sc2331_uart_feed(&boot->rx, data[i]);
		if (ret) {
			boot->rx_status = ret;
			complete(&boot->received);
		}
	}
	spin_unlock_irqrestore(&boot->lock, flags);
	return count;
}

static const struct serdev_device_ops sc2331_boot_ops = {
	.receive_buf = sc2331_boot_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static void sc2331_boot_rx_reset(struct sc2331_boot *boot)
{
	unsigned long flags;

	spin_lock_irqsave(&boot->lock, flags);
	boot->rx.len = 0;
	boot->rx_status = 0;
	reinit_completion(&boot->received);
	spin_unlock_irqrestore(&boot->lock, flags);
}

static int sc2331_boot_write(struct sc2331_boot *boot, const u8 *data,
			     size_t len, unsigned long timeout)
{
	int ret;

	ret = serdev_device_write(boot->serdev, data, len, timeout);
	return ret < 0 ? ret : ret == len ? 0 : -ETIMEDOUT;
}

static int sc2331_boot_response(struct sc2331_boot *boot, unsigned long timeout)
{
	if (!wait_for_completion_timeout(&boot->received, timeout))
		return -ETIMEDOUT;
	return boot->rx_status < 0 ? boot->rx_status : 0;
}

static int sc2331_boot_handshake(struct sc2331_boot *boot)
{
	const u8 hello[] = { 0x7e, 0x7e, 0x7e };
	unsigned long deadline = jiffies + msecs_to_jiffies(3000);
	unsigned long now;
	int ret;

	sc2331_boot_rx_reset(boot);
	while (time_before(now = jiffies, deadline)) {
		ret = sc2331_boot_write(boot, hello, sizeof(hello), deadline - now);
		if (ret)
			return ret;
		usleep_range(300, 600);
		now = jiffies;
		if (time_after_eq(now, deadline))
			return -ETIMEDOUT;
		ret = sc2331_boot_write(boot, hello, sizeof(hello), deadline - now);
		if (ret)
			return ret;
		now = jiffies;
		if (time_after_eq(now, deadline))
			return -ETIMEDOUT;
		ret = sc2331_boot_response(boot, min(msecs_to_jiffies(20), deadline - now));
		if (ret != -ETIMEDOUT) {
			if (!ret && boot->rx.len > 64)
				return -EMSGSIZE;
			if (!ret)
				dev_info(&boot->serdev->dev, "Boot ROM version: %*ph\n",
					 (int)boot->rx.len, boot->rx.data);
			return ret;
		}
	}
	return -ETIMEDOUT;
}

static int sc2331_boot_command(struct sc2331_boot *boot, u16 command,
			       const u8 *data, size_t len)
{
	struct sc2331_boot_frame frame;
	ssize_t encoded;
	int ret;

	encoded = sc2331_encode_uart(boot->tx, sizeof(boot->tx), command, data, len);
	if (encoded < 0)
		return encoded;
	sc2331_boot_rx_reset(boot);
	ret = sc2331_boot_write(boot, boot->tx, encoded, HZ);
	if (ret)
		return ret;
	ret = sc2331_boot_response(boot, msecs_to_jiffies(500));
	if (ret)
		return ret;
	ret = sc2331_decode_uart(boot->rx.data, boot->rx.len, &frame);
	if (!ret && (frame.command != 0x80 || frame.len))
		ret = -EPROTO;
	if (ret)
		dev_err(&boot->serdev->dev, "Invalid reply to command %u: %*ph\n",
			command, (int)boot->rx.len, boot->rx.data);
	return ret;
}

static void sc2331_boot_power_off(void *data)
{
	struct sc2331_boot *boot = data;
	int ret;

	sc2331_boot_sleep_device(boot);
	gpiod_set_value_cansleep(boot->reset, 1);
	if (boot->clocked) {
		clk_disable_unprepare(boot->clock);
		boot->clocked = false;
	}
	if (boot->powered) {
		ret = regulator_disable(boot->supply);
		if (ret)
			dev_err(&boot->serdev->dev, "Supply disable failed: %d\n", ret);
		else
			boot->powered = false;
	}
}

static int sc2331_boot_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	const struct firmware *firmware;
	struct sc2331_boot *boot;
	u8 *image, payload[8];
	unsigned int offset;
	int ret;

	boot = devm_kzalloc(dev, sizeof(*boot), GFP_KERNEL);
	if (!boot)
		return -ENOMEM;
	boot->serdev = serdev;
	spin_lock_init(&boot->lock);
	init_completion(&boot->received);
	init_completion(&boot->ready);
	serdev_device_set_drvdata(serdev, boot);
	serdev_device_set_client_ops(serdev, &sc2331_boot_ops);
	boot->supply = devm_regulator_get(dev, "vddcon");
	if (IS_ERR(boot->supply))
		return dev_err_probe(dev, PTR_ERR(boot->supply), "Missing connectivity supply\n");
	boot->clock = devm_clk_get(dev, "ext_clock");
	if (IS_ERR(boot->clock))
		return dev_err_probe(dev, PTR_ERR(boot->clock), "Missing reference clock\n");
	boot->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(boot->reset))
		return dev_err_probe(dev, PTR_ERR(boot->reset), "Missing reset\n");
	boot->host_wake = devm_gpiod_get_optional(dev, "host-wake", GPIOD_IN);
	if (IS_ERR(boot->host_wake))
		return dev_err_probe(dev, PTR_ERR(boot->host_wake), "Invalid host wake\n");
	boot->device_wake = devm_gpiod_get_optional(dev, "device-wake", GPIOD_OUT_LOW);
	if (IS_ERR(boot->device_wake))
		return dev_err_probe(dev, PTR_ERR(boot->device_wake), "Invalid device wake\n");
	if (boot->host_wake) {
		ret = gpiod_to_irq(boot->host_wake);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Missing host wake IRQ\n");
		ret = devm_request_threaded_irq(dev, ret, NULL, sc2331_boot_wake,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				dev_name(dev), boot);
		if (ret)
			return dev_err_probe(dev, ret, "Cannot request host wake IRQ\n");
	}
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return ret;
	if (serdev_device_set_baudrate(serdev, 115200) != 115200)
		return -EINVAL;
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return ret;
	ret = request_firmware(&firmware, SC2331_FDL_NAME, dev);
	if (ret)
		return ret;
	if (!firmware->size || firmware->size > SC2331_FDL_SIZE) {
		ret = -EINVAL;
		goto release;
	}
	image = kmalloc(SC2331_FDL_SIZE, GFP_KERNEL);
	if (!image) {
		ret = -ENOMEM;
		goto release;
	}
	memset(image, 0xff, SC2331_FDL_SIZE);
	memcpy(image, firmware->data, firmware->size);
	ret = devm_add_action_or_reset(dev, sc2331_boot_power_off, boot);
	if (ret)
		goto free;
	ret = clk_set_rate(boot->clock, 32768);
	if (ret)
		goto free;
	ret = regulator_enable(boot->supply);
	if (ret)
		goto free;
	boot->powered = true;
	ret = clk_prepare_enable(boot->clock);
	if (ret)
		goto free;
	boot->clocked = true;
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(boot->reset, 0);
	usleep_range(1000, 2000);
	ret = sc2331_boot_handshake(boot);
	if (ret)
		goto free;
	ret = sc2331_boot_command(boot, 0, NULL, 0);
	if (ret)
		goto free;
	put_unaligned_be32(SC2331_FDL_ADDRESS, payload);
	put_unaligned_be32(SC2331_FDL_SIZE, payload + 4);
	ret = sc2331_boot_command(boot, 1, payload, sizeof(payload));
	if (ret)
		goto free;
	for (offset = 0; offset < SC2331_FDL_SIZE; offset += 1024) {
		ret = sc2331_boot_command(boot, 2, image + offset, 1024);
		if (ret)
			goto free;
	}
	ret = sc2331_boot_command(boot, 3, NULL, 0);
	if (ret)
		goto free;
	ret = sc2331_boot_command(boot, 4, payload, 4);
	if (!ret)
		dev_info(dev, "SC2331 UART FDL ready\n");
free:
	kfree(image);
release:
	release_firmware(firmware);
	return ret ? dev_err_probe(dev, ret, "UART bootstrap failed\n") : 0;
}

static void sc2331_boot_shutdown(struct serdev_device *serdev)
{
	sc2331_boot_power_off(serdev_device_get_drvdata(serdev));
}

static const struct of_device_id sc2331_boot_match[] = {
	{ .compatible = "sprd,sc2331" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2331_boot_match);

static struct serdev_device_driver sc2331_boot_driver = {
	.driver = {
		.name = "sc2331-boot",
		.of_match_table = sc2331_boot_match,
	},
	.probe = sc2331_boot_probe,
	.shutdown = sc2331_boot_shutdown,
};

struct sc2331_boot *sc2331_boot_get(struct device *dev)
{
	struct sc2331_boot *boot = ERR_PTR(-EPROBE_DEFER);
	struct device_node *node;
	struct device *supplier;

	node = of_parse_phandle(dev->of_node, "sprd,boot-controller", 0);
	if (!node)
		return ERR_PTR(-EINVAL);
	supplier = driver_find_device_by_of_node(&sc2331_boot_driver.driver, node);
	of_node_put(node);
	if (!supplier)
		return boot;
	device_lock(supplier);
	if (device_is_bound(supplier)) {
		boot = dev_get_drvdata(supplier);
		if (!boot->host_wake)
			boot = ERR_PTR(-ENODEV);
		else if (!device_link_add(dev, supplier, DL_FLAG_AUTOREMOVE_CONSUMER))
			boot = ERR_PTR(-ENOMEM);
	}
	device_unlock(supplier);
	put_device(supplier);
	return boot;
}

static int __init sc2331_init(void)
{
	int ret;

	ret = serdev_device_driver_register(&sc2331_boot_driver);
	if (ret)
		return ret;
	ret = sdio_register_driver(&sc2331_sdio_driver);
	if (ret)
		serdev_device_driver_unregister(&sc2331_boot_driver);
	return ret;
}
module_init(sc2331_init);

static void __exit sc2331_exit(void)
{
	sdio_unregister_driver(&sc2331_sdio_driver);
	serdev_device_driver_unregister(&sc2331_boot_driver);
}
module_exit(sc2331_exit);

MODULE_FIRMWARE(SC2331_FDL_NAME);
MODULE_DESCRIPTION("SC2331 connectivity UART bootstrap");
MODULE_LICENSE("GPL");
