// SPDX-License-Identifier: GPL-2.0-only
#include <linux/i2c.h>
#include <linux/devm-helpers.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define SM5701_STATUS1		0x06
#define SM5701_VBUSOK		BIT(2)

struct sm5701 {
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll;
	bool online;
};

static bool sm5701_readable_reg(struct device *dev, unsigned int reg)
{
	return reg == SM5701_STATUS1;
}

static bool sm5701_writeable_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct regmap_config sm5701_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5701_STATUS1,
	.cache_type = REGCACHE_NONE,
	.readable_reg = sm5701_readable_reg,
	.writeable_reg = sm5701_writeable_reg,
};

static int sm5701_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sm5701 *data = power_supply_get_drvdata(psy);
	unsigned int status;
	int ret;

	if (prop != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	ret = regmap_read(data->regmap, SM5701_STATUS1, &status);
	if (ret)
		return ret;

	val->intval = !!(status & SM5701_VBUSOK);

	return 0;
}

static const enum power_supply_property sm5701_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sm5701_desc = {
	.name = "sm5701-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5701_properties,
	.num_properties = ARRAY_SIZE(sm5701_properties),
	.get_property = sm5701_get_property,
};

static void sm5701_poll(struct work_struct *work)
{
	struct sm5701 *data = container_of(to_delayed_work(work), struct sm5701, poll);
	unsigned int status;
	bool online;

	if (!regmap_read(data->regmap, SM5701_STATUS1, &status)) {
		online = !!(status & SM5701_VBUSOK);
		if (online != data->online) {
			data->online = online;
			power_supply_changed(data->psy);
		}
	}
	schedule_delayed_work(&data->poll, msecs_to_jiffies(2000));
}

static int sm5701_suspend(struct device *dev)
{
	struct sm5701 *data = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&data->poll);
	return 0;
}

static int sm5701_resume(struct device *dev)
{
	struct sm5701 *data = dev_get_drvdata(dev);

	schedule_delayed_work(&data->poll, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sm5701_pm_ops, sm5701_suspend, sm5701_resume);

static int sm5701_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct device *dev = &client->dev;
	struct power_supply *psy;
	struct regmap *regmap;
	struct sm5701 *data;
	unsigned int status;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &sm5701_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to create regmap\n");

	ret = regmap_read(regmap, SM5701_STATUS1, &status);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read input status\n");

	data->regmap = regmap;
	data->online = !!(status & SM5701_VBUSOK);
	i2c_set_clientdata(client, data);
	cfg.drv_data = data;
	cfg.fwnode = dev_fwnode(dev);
	psy = devm_power_supply_register(dev, &sm5701_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "Failed to register supply\n");
	data->psy = psy;
	ret = devm_delayed_work_autocancel(dev, &data->poll, sm5701_poll);
	if (ret)
		return ret;
	schedule_delayed_work(&data->poll, msecs_to_jiffies(2000));

	return 0;
}

static const struct of_device_id sm5701_of_match[] = {
	{ .compatible = "siliconmitus,sm5701" },
	{}
};
MODULE_DEVICE_TABLE(of, sm5701_of_match);

static struct i2c_driver sm5701_driver = {
	.driver = {
		.name = "sm5701",
		.of_match_table = sm5701_of_match,
		.pm = pm_sleep_ptr(&sm5701_pm_ops),
	},
	.probe = sm5701_probe,
};
module_i2c_driver(sm5701_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5701 input supply monitor");
MODULE_AUTHOR("minorum <mit.mis@gmail.com>");
MODULE_LICENSE("GPL");
