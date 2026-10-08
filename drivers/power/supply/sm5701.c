// SPDX-License-Identifier: GPL-2.0-only
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define SM5701_STATUS1		0x06
#define SM5701_VBUSOK		BIT(2)

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
	struct regmap *regmap = power_supply_get_drvdata(psy);
	unsigned int status;
	int ret;

	if (prop != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	ret = regmap_read(regmap, SM5701_STATUS1, &status);
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

static int sm5701_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct device *dev = &client->dev;
	struct power_supply *psy;
	struct regmap *regmap;
	unsigned int status;
	int ret;

	regmap = devm_regmap_init_i2c(client, &sm5701_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to create regmap\n");

	ret = regmap_read(regmap, SM5701_STATUS1, &status);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read input status\n");

	cfg.drv_data = regmap;
	cfg.fwnode = dev_fwnode(dev);
	psy = devm_power_supply_register(dev, &sm5701_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "Failed to register supply\n");

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
	},
	.probe = sm5701_probe,
};
module_i2c_driver(sm5701_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5701 input supply monitor");
MODULE_AUTHOR("minorum <mit.mis@gmail.com>");
MODULE_LICENSE("GPL");
