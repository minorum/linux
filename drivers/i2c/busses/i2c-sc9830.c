// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#define REG_CTL		0x00
#define REG_CMD		0x04
#define REG_CLKD0	0x08
#define REG_CLKD1	0x0c
#define REG_RESET	0x10
#define REG_BUF_CTL	0x18
#define REG_START_STOP	0x1c
#define CTL_INT		BIT(0)
#define CTL_IE		BIT(3)
#define CTL_ENABLE	BIT(4)
#define CTL_NOACK_IE	BIT(7)
#define CTL_DUTY	BIT(11)
#define CMD_INT_ACK	BIT(0)
#define CMD_NACK	BIT(1)
#define CMD_WRITE	BIT(2)
#define CMD_READ	BIT(3)
#define CMD_STOP	BIT(4)
#define CMD_START	BIT(5)
#define CMD_ACK		BIT(6)
#define CMD_BUSY	BIT(7)
#define CMD_DATA	GENMASK(15, 8)

struct sc9830_i2c {
	struct i2c_adapter adapter;
	struct completion done;
	struct clk *clk;
	struct clk *enable;
	void __iomem *base;
	unsigned long deadline;
	u32 frequency;
	u32 result;
	int error;
	int irq;
	bool active;
	bool suspended;
};

static int sc9830_i2c_divider(u64 rate, u32 frequency, u32 *divider)
{
	u64 quotient;

	if (!frequency || rate < 4ULL * frequency)
		return -EINVAL;

	quotient = DIV_ROUND_UP_ULL(rate, 4ULL * frequency);
	if (quotient - 1 > U32_MAX)
		return -ERANGE;

	*divider = quotient - 1;
	return 0;
}

static u32 sc9830_i2c_data_command(bool read, u8 byte, bool last, bool stop)
{
	u32 command = read ? CMD_READ : CMD_WRITE | (u32)byte << 8;

	if (read && last)
		command |= CMD_NACK;
	if (last && stop)
		command |= CMD_STOP;

	return command;
}

static void sc9830_i2c_disable(struct sc9830_i2c *i2c)
{
	u32 control = readl(i2c->base + REG_CTL);

	writel(control & ~(CTL_ENABLE | CTL_IE | CTL_NOACK_IE),
	       i2c->base + REG_CTL);
}

static int sc9830_i2c_setup(struct sc9830_i2c *i2c)
{
	u32 control, divider, version;
	unsigned long rate = clk_get_rate(i2c->clk);
	int ret;

	if (rate != 26000000)
		return -EINVAL;
	ret = sc9830_i2c_divider(rate, i2c->frequency, &divider);
	if (ret)
		return ret;

	control = readl(i2c->base + REG_CTL);
	version = control >> 16;
	control &= ~(CTL_ENABLE | CTL_IE | CTL_NOACK_IE | CTL_DUTY);
	writel(control, i2c->base + REG_CTL);
	writel(0, i2c->base + REG_BUF_CTL);
	writel(divider & 0xffff, i2c->base + REG_CLKD0);
	writel(divider >> 16, i2c->base + REG_CLKD1);
	if (version >> 8 >= 7)
		writel(i2c->frequency == 400000 ? 0x10 : 0x75,
		       i2c->base + REG_START_STOP);
	writel(CMD_INT_ACK, i2c->base + REG_CMD);
	writel(control | CTL_ENABLE | CTL_IE, i2c->base + REG_CTL);
	return 0;
}

static irqreturn_t sc9830_i2c_irq(int irq, void *data)
{
	struct sc9830_i2c *i2c = data;
	u32 command, status;
	int ret;

	if (!(readl(i2c->base + REG_CTL) & CTL_INT))
		return IRQ_NONE;

	ret = readl_poll_timeout(i2c->base + REG_CMD, command,
				 !(command & CMD_BUSY), 10, 5000);
	writel((command & CMD_DATA) | CMD_INT_ACK, i2c->base + REG_CMD);
	if (!ret) {
		ret = readl_poll_timeout(i2c->base + REG_CMD, status,
				   !(status & CMD_ACK), 10, 5000);
		command = (command & CMD_DATA) | (status & CMD_ACK);
	}
	if (READ_ONCE(i2c->active)) {
		i2c->result = command;
		i2c->error = ret;
		WRITE_ONCE(i2c->active, false);
		complete(&i2c->done);
	}
	return IRQ_HANDLED;
}

static int sc9830_i2c_command(struct sc9830_i2c *i2c, u32 command,
			      bool address, u8 *byte)
{
	unsigned long remaining, now = jiffies;

	if (time_after_eq(now, i2c->deadline))
		return -ETIMEDOUT;
	remaining = i2c->deadline - now;
	reinit_completion(&i2c->done);
	WRITE_ONCE(i2c->active, true);
	writel(command, i2c->base + REG_CMD);
	if (!wait_for_completion_timeout(&i2c->done, remaining))
		return -ETIMEDOUT;
	if (i2c->error)
		return i2c->error;
	if (i2c->result & CMD_ACK)
		return address ? -ENXIO : -EREMOTEIO;
	if (byte)
		*byte = i2c->result >> 8;
	return 0;
}

static int sc9830_i2c_xfer(struct i2c_adapter *adapter,
			   struct i2c_msg *messages, int count)
{
	struct sc9830_i2c *i2c = i2c_get_adapdata(adapter);
	int ret, index, byte;

	if (i2c->suspended)
		return -ESHUTDOWN;
	for (index = 0; index < count; index++) {
		if (messages[index].addr > 0x7f || !messages[index].len ||
		    messages[index].flags & ~(I2C_M_RD | I2C_M_STOP | I2C_M_DMA_SAFE))
			return -EOPNOTSUPP;
	}
	ret = clk_prepare_enable(i2c->enable);
	if (ret)
		return ret;
	ret = sc9830_i2c_setup(i2c);
	if (ret)
		goto disable_clock;
	i2c->deadline = jiffies + adapter->timeout;
	enable_irq(i2c->irq);
	for (index = 0; index < count; index++) {
		struct i2c_msg *message = &messages[index];
		bool read = message->flags & I2C_M_RD;
		bool stop = index == count - 1 || message->flags & I2C_M_STOP;
		u32 address = (message->addr << 1) | read;

		ret = sc9830_i2c_command(i2c, address << 8 | CMD_START | CMD_WRITE,
					 true, NULL);
		if (ret)
			break;
		for (byte = 0; byte < message->len; byte++) {
			u32 command = sc9830_i2c_data_command(read, read ? 0 : message->buf[byte],
							    byte == message->len - 1, stop);

			ret = sc9830_i2c_command(i2c, command, false,
						 read ? &message->buf[byte] : NULL);
			if (ret)
				break;
		}
		if (ret)
			break;
	}
	disable_irq(i2c->irq);
	WRITE_ONCE(i2c->active, false);
	sc9830_i2c_disable(i2c);
	if (ret) {
		writel(BIT(0), i2c->base + REG_RESET);
		writel(0, i2c->base + REG_RESET);
	}
disable_clock:
	clk_disable_unprepare(i2c->enable);
	return ret ?: count;
}

static u32 sc9830_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm sc9830_i2c_algorithm = {
	.xfer = sc9830_i2c_xfer,
	.functionality = sc9830_i2c_functionality,
};

static const struct i2c_adapter_quirks sc9830_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN,
};

static int sc9830_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc9830_i2c *i2c;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;
	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);
	i2c->clk = devm_clk_get(dev, "i2c");
	if (IS_ERR(i2c->clk))
		return dev_err_probe(dev, PTR_ERR(i2c->clk), "Missing input clock\n");
	i2c->enable = devm_clk_get(dev, "enable");
	if (IS_ERR(i2c->enable))
		return dev_err_probe(dev, PTR_ERR(i2c->enable), "Missing gate clock\n");
	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0)
		return i2c->irq;
	i2c->frequency = 100000;
	if (device_property_present(dev, "clock-frequency")) {
		ret = device_property_read_u32(dev, "clock-frequency", &i2c->frequency);
		if (ret)
			return ret;
	}
	if (i2c->frequency != 100000 && i2c->frequency != 400000)
		return -EINVAL;
	init_completion(&i2c->done);
	ret = clk_prepare_enable(i2c->enable);
	if (ret)
		return ret;
	sc9830_i2c_disable(i2c);
	writel(CMD_INT_ACK, i2c->base + REG_CMD);
	ret = devm_request_threaded_irq(dev, i2c->irq, NULL, sc9830_i2c_irq,
					IRQF_ONESHOT, dev_name(dev), i2c);
	if (!ret)
		disable_irq(i2c->irq);
	clk_disable_unprepare(i2c->enable);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot request interrupt\n");
	i2c->adapter.owner = THIS_MODULE;
	i2c->adapter.algo = &sc9830_i2c_algorithm;
	i2c->adapter.quirks = &sc9830_i2c_quirks;
	i2c->adapter.dev.parent = dev;
	i2c->adapter.dev.of_node = dev->of_node;
	i2c->adapter.timeout = HZ;
	strscpy(i2c->adapter.name, "SC9830 I2C", sizeof(i2c->adapter.name));
	i2c_set_adapdata(&i2c->adapter, i2c);
	platform_set_drvdata(pdev, i2c);
	return devm_i2c_add_adapter(dev, &i2c->adapter);
}

static int sc9830_i2c_suspend(struct device *dev)
{
	struct sc9830_i2c *i2c = dev_get_drvdata(dev);

	i2c_lock_bus(&i2c->adapter, I2C_LOCK_ROOT_ADAPTER);
	i2c->suspended = true;
	i2c_unlock_bus(&i2c->adapter, I2C_LOCK_ROOT_ADAPTER);
	return 0;
}

static int sc9830_i2c_resume(struct device *dev)
{
	struct sc9830_i2c *i2c = dev_get_drvdata(dev);

	i2c_lock_bus(&i2c->adapter, I2C_LOCK_ROOT_ADAPTER);
	i2c->suspended = false;
	i2c_unlock_bus(&i2c->adapter, I2C_LOCK_ROOT_ADAPTER);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc9830_i2c_pm_ops, sc9830_i2c_suspend,
			      sc9830_i2c_resume);

static const struct of_device_id sc9830_i2c_match[] = {
	{ .compatible = "sprd,sc9830-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_i2c_match);

static struct platform_driver sc9830_i2c_driver = {
	.probe = sc9830_i2c_probe,
	.driver = {
		.name = "sc9830-i2c",
		.of_match_table = sc9830_i2c_match,
		.pm = pm_sleep_ptr(&sc9830_i2c_pm_ops),
	},
};
module_platform_driver(sc9830_i2c_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 I2C controller");
MODULE_LICENSE("GPL");
