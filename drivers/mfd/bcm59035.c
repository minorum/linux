// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define BCM59035_HOSTACT		0x51
#define BCM59035_HOSTDICOFF	0x04

static int bcm59035_reg_read(void *context, unsigned int reg,
			     unsigned int *value)
{
	struct i2c_client *client = context;
	u8 address = reg, data;
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = I2C_M_STOP,
		  .len = 1, .buf = &address },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = &data },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	*value = data;
	return 0;
}

static int bcm59035_reg_write(void *context, unsigned int reg,
			      unsigned int value)
{
	struct i2c_client *client = context;
	u8 bytes[] = { reg, value };
	struct i2c_msg msg = {
		.addr = client->addr, .len = sizeof(bytes), .buf = bytes,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == 1 ? 0 : ret < 0 ? ret : -EIO;
}

static bool bcm59035_precious_reg(struct device *dev, unsigned int reg)
{
	return reg >= 0x01 && reg <= 0x0a;
}

static bool bcm59035_readable_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0x01 && reg <= 0x14) || reg == 0x1b ||
	       reg == BCM59035_HOSTACT || reg == 0xb5;
}

static bool bcm59035_writeable_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0x0b && reg <= 0x14) || reg == 0x1b ||
	       reg == BCM59035_HOSTACT;
}

static const struct regmap_config bcm59035_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
	.reg_read = bcm59035_reg_read,
	.reg_write = bcm59035_reg_write,
	.precious_reg = bcm59035_precious_reg,
	.readable_reg = bcm59035_readable_reg,
	.writeable_reg = bcm59035_writeable_reg,
};

#define BCM59035_IRQ_BANKS	10
#define BCM59035_IRQ_COUNT	74
#define BCM59035_IRQ_DRAIN_ROUNDS	4
#define BCM59035_IRQ_RETRY_MS	100
#define BCM59035_IRQ_STATUS	0x01
#define BCM59035_IRQ_MASK		0x0b

static const u8 bcm59035_irq_valid[BCM59035_IRQ_BANKS] = {
	0xdf, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0xff, 0x8f, 0x0f, 0x03,
};

struct bcm59035_irq {
	struct device *dev;
	struct regmap *regmap;
	struct irq_domain *domain;
	struct mutex lock; /* Mask state */
	struct mutex drain_lock; /* IRQ dispatch */
	struct delayed_work retry;
	u8 mask[BCM59035_IRQ_BANKS];
	unsigned long dirty;
	int parent_irq;
	bool stopping;
};

static void bcm59035_irq_schedule_retry(struct bcm59035_irq *irq)
{
	if (!irq->stopping)
		mod_delayed_work(system_wq, &irq->retry,
				 msecs_to_jiffies(BCM59035_IRQ_RETRY_MS));
}

static int bcm59035_irq_sync(struct bcm59035_irq *irq)
{
	unsigned int bank;
	int ret;

	for_each_set_bit(bank, &irq->dirty, BCM59035_IRQ_BANKS) {
		ret = regmap_update_bits(irq->regmap, BCM59035_IRQ_MASK + bank,
					 bcm59035_irq_valid[bank], irq->mask[bank]);
		if (ret) {
			dev_err_ratelimited(irq->dev, "IRQ mask write failed: %d\n", ret);
			return ret;
		}
		irq->dirty &= ~BIT(bank);
	}
	return 0;
}

static void bcm59035_irq_bus_lock(struct irq_data *data)
{
	struct bcm59035_irq *irq = irq_data_get_irq_chip_data(data);

	mutex_lock(&irq->lock);
}

static void bcm59035_irq_bus_sync_unlock(struct irq_data *data)
{
	struct bcm59035_irq *irq = irq_data_get_irq_chip_data(data);

	if (bcm59035_irq_sync(irq))
		bcm59035_irq_schedule_retry(irq);
	mutex_unlock(&irq->lock);
}

static void bcm59035_irq_mask(struct irq_data *data)
{
	struct bcm59035_irq *irq = irq_data_get_irq_chip_data(data);
	unsigned int bank = data->hwirq / 8;

	irq->mask[bank] |= BIT(data->hwirq % 8);
	irq->dirty |= BIT(bank);
}

static void bcm59035_irq_unmask(struct irq_data *data)
{
	struct bcm59035_irq *irq = irq_data_get_irq_chip_data(data);
	unsigned int bank = data->hwirq / 8;

	irq->mask[bank] &= ~BIT(data->hwirq % 8);
	irq->dirty |= BIT(bank);
}

static void bcm59035_irq_process(struct bcm59035_irq *irq)
{
	unsigned int round, bank, bit, status, virq;
	bool active, retry = false;
	int ret;

	mutex_lock(&irq->drain_lock);
	for (round = 0; round < BCM59035_IRQ_DRAIN_ROUNDS; round++) {
		u8 pending[BCM59035_IRQ_BANKS] = {};

		active = false;
		mutex_lock(&irq->lock);
		if (irq->stopping) {
			mutex_unlock(&irq->lock);
			break;
		}
		ret = bcm59035_irq_sync(irq);
		if (!ret) {
			for (bank = 0; bank < BCM59035_IRQ_BANKS; bank++) {
				ret = regmap_read(irq->regmap, BCM59035_IRQ_STATUS + bank,
						  &status);
				if (ret) {
					dev_err_ratelimited(irq->dev,
							    "IRQ status read failed: %d\n", ret);
					break;
				}
				active |= !!status;
				pending[bank] = status & ~irq->mask[bank] &
						bcm59035_irq_valid[bank];
			}
		}
		mutex_unlock(&irq->lock);

		for (bank = 0; bank < BCM59035_IRQ_BANKS; bank++) {
			for (bit = 0; bit < 8; bit++) {
				if (!(pending[bank] & BIT(bit)))
					continue;
				virq = irq_find_mapping(irq->domain, bank * 8 + bit);
				if (virq)
					handle_nested_irq(virq);
			}
		}
		if (ret) {
			retry = true;
			break;
		}
		if (!active)
			break;
	}
	if (retry || round == BCM59035_IRQ_DRAIN_ROUNDS) {
		mutex_lock(&irq->lock);
		bcm59035_irq_schedule_retry(irq);
		mutex_unlock(&irq->lock);
	}
	mutex_unlock(&irq->drain_lock);
}

static irqreturn_t bcm59035_irq_thread(int parent_irq, void *data)
{
	bcm59035_irq_process(data);
	return IRQ_HANDLED;
}

static void bcm59035_irq_retry_work(struct work_struct *work)
{
	struct bcm59035_irq *irq = container_of(to_delayed_work(work),
					      struct bcm59035_irq, retry);

	bcm59035_irq_process(irq);
}

static void bcm59035_irq_quiesce(void *data)
{
	struct bcm59035_irq *irq = data;
	unsigned int bank;

	mutex_lock(&irq->lock);
	if (irq->stopping) {
		mutex_unlock(&irq->lock);
		return;
	}
	irq->stopping = true;
	mutex_unlock(&irq->lock);
	disable_irq(irq->parent_irq);
	cancel_delayed_work_sync(&irq->retry);
	mutex_lock(&irq->lock);
	for (bank = 0; bank < BCM59035_IRQ_BANKS; bank++)
		irq->mask[bank] = bcm59035_irq_valid[bank];
	irq->dirty = BIT(BCM59035_IRQ_BANKS) - 1;
	bcm59035_irq_sync(irq);
	mutex_unlock(&irq->lock);
}

static const struct irq_chip bcm59035_irq_chip = {
	.name = "bcm59035",
	.irq_bus_lock = bcm59035_irq_bus_lock,
	.irq_bus_sync_unlock = bcm59035_irq_bus_sync_unlock,
	.irq_mask = bcm59035_irq_mask,
	.irq_unmask = bcm59035_irq_unmask,
};

static int bcm59035_irq_map(struct irq_domain *domain, unsigned int virq,
			    irq_hw_number_t hwirq)
{
	if (hwirq >= BCM59035_IRQ_COUNT ||
	    !(bcm59035_irq_valid[hwirq / 8] & BIT(hwirq % 8)))
		return -EINVAL;
	irq_set_chip_data(virq, domain->host_data);
	irq_set_chip_and_handler(virq, &bcm59035_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, 1);
	irq_set_noprobe(virq);
	return 0;
}

static const struct irq_domain_ops bcm59035_irq_domain_ops = {
	.map = bcm59035_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static void bcm59035_irq_remove_domain(void *data)
{
	struct bcm59035_irq *irq = data;
	unsigned int hwirq, virq;

	for (hwirq = 0; hwirq < BCM59035_IRQ_COUNT; hwirq++) {
		virq = irq_find_mapping(irq->domain, hwirq);
		if (virq)
			irq_dispose_mapping(virq);
	}
	irq_domain_remove(irq->domain);
}

static int bcm59035_irq_init(struct i2c_client *client, struct regmap *regmap,
			     struct irq_domain **domain)
{
	struct device *dev = &client->dev;
	struct bcm59035_irq *irq;
	unsigned int bank, status;
	int ret;

	if (client->irq <= 0)
		return -EINVAL;
	irq = devm_kzalloc(dev, sizeof(*irq), GFP_KERNEL);
	if (!irq)
		return -ENOMEM;
	irq->dev = dev;
	irq->regmap = regmap;
	irq->parent_irq = client->irq;
	mutex_init(&irq->lock);
	mutex_init(&irq->drain_lock);
	INIT_DELAYED_WORK(&irq->retry, bcm59035_irq_retry_work);
	memcpy(irq->mask, bcm59035_irq_valid, sizeof(irq->mask));
	irq->dirty = BIT(BCM59035_IRQ_BANKS) - 1;
	mutex_lock(&irq->lock);
	ret = bcm59035_irq_sync(irq);
	for (bank = 0; !ret && bank < BCM59035_IRQ_BANKS; bank++)
		ret = regmap_read(regmap, BCM59035_IRQ_STATUS + bank, &status);
	mutex_unlock(&irq->lock);
	if (ret)
		return ret;
	irq->domain = irq_domain_create_linear(dev_fwnode(dev), BCM59035_IRQ_COUNT,
					       &bcm59035_irq_domain_ops, irq);
	if (!irq->domain)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, bcm59035_irq_remove_domain, irq);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(dev, client->irq, NULL, bcm59035_irq_thread,
					IRQF_ONESHOT, dev_name(dev), irq);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, bcm59035_irq_quiesce, irq);
	if (ret)
		return ret;
	i2c_set_clientdata(client, irq);
	*domain = irq->domain;
	return 0;
}

static const struct mfd_cell bcm59035_cells[] = {
	{ .name = "bcm59035-regulator" },
};

static int bcm59035_power_off_prepare(struct sys_off_data *data)
{
	struct regmap *regmap = data->cb_data;
	int ret;

	ret = regmap_write(regmap, BCM59035_HOSTACT, BCM59035_HOSTDICOFF);
	if (ret)
		dev_err(data->dev, "power-off request failed: %d\n", ret);

	return notifier_from_errno(ret);
}

static int bcm59035_power_off(struct sys_off_data *data)
{
	mdelay(1000);
	dev_emerg(data->dev, "power-off request did not stop execution\n");
	return NOTIFY_DONE;
}

static int bcm59035_register_power_off(struct device *dev, struct regmap *regmap)
{
	int ret;

	if (!device_property_read_bool(dev, "system-power-controller"))
		return 0;

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
					    SYS_OFF_PRIO_DEFAULT,
					    bcm59035_power_off_prepare, regmap);
	if (ret)
		return ret;

	return devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
					   SYS_OFF_PRIO_DEFAULT,
					   bcm59035_power_off, NULL);
}

static int bcm59035_i2c_probe(struct i2c_client *client)
{
	struct regmap *regmap;
	struct irq_domain *domain;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	regmap = devm_regmap_init(&client->dev, NULL, client,
				  &bcm59035_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&client->dev, PTR_ERR(regmap),
				     "failed to initialize regmap\n");

	ret = bcm59035_irq_init(client, regmap, &domain);
	if (ret)
		return dev_err_probe(&client->dev, ret, "failed to initialize IRQs\n");

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   bcm59035_cells, ARRAY_SIZE(bcm59035_cells),
				   NULL, 0, domain);
	if (ret)
		return ret;

	ret = bcm59035_register_power_off(&client->dev, regmap);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register power-off handlers\n");

	return 0;
}

static void bcm59035_i2c_shutdown(struct i2c_client *client)
{
	bcm59035_irq_quiesce(i2c_get_clientdata(client));
}

static const struct of_device_id bcm59035_match[] = {
	{ .compatible = "brcm,bcm59035" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm59035_match);

static struct i2c_driver bcm59035_driver = {
	.probe = bcm59035_i2c_probe,
	.shutdown = bcm59035_i2c_shutdown,
	.driver = {
		.name = "bcm59035",
		.of_match_table = bcm59035_match,
	},
};
module_i2c_driver(bcm59035_driver);

MODULE_DESCRIPTION("Broadcom BCM59035 power management IC");
MODULE_LICENSE("GPL");
