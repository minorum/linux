// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define MT6329_VUSB 0x9b
#define MT6329_VMC  0xa7

static bool mt6329_readable(struct device *dev, unsigned int reg)
{
	return reg == 0 || reg == 1 || reg == 0x21 || reg == 0x2e || reg == 0x30 ||
	       reg == 0x33 || reg == 0x34 ||
	       reg == MT6329_VUSB || reg == MT6329_VMC ||
	       reg == 0x38 || reg == 0xe8 || reg == 0xe9 ||
	       reg == 0xdc || reg == 0xdd || reg == 0xe0 || reg == 0xe1;
}

static bool mt6329_writeable(struct device *dev, unsigned int reg)
{
	return reg == 0x21 || reg == 0x2e || reg == 0x30 ||
	       reg == 0x33 || reg == 0x34 ||
	       reg == MT6329_VUSB || reg == MT6329_VMC ||
	       reg == 0x38 || reg == 0xe8 || reg == 0xe9;
}

static const struct regmap_config mt6329_regmap_config = {
	.name = "power",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xe9,
	.cache_type = REGCACHE_NONE,
	.readable_reg = mt6329_readable,
	.writeable_reg = mt6329_writeable,
};

static bool mt6329_bank1_readable(struct device *dev, unsigned int reg)
{
	return reg == 0x92 || reg == 0x93;
}

static bool mt6329_bank1_writeable(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct regmap_config mt6329_bank1_regmap_config = {
	.name = "bank1",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x93,
	.cache_type = REGCACHE_NONE,
	.readable_reg = mt6329_bank1_readable,
	.writeable_reg = mt6329_bank1_writeable,
};

static const struct mfd_cell mt6329_cells[] = {
	{ .name = "mt6329-regulator" },
	{
		.name = "mt6329-auxadc",
		.of_compatible = "mediatek,mt6329-auxadc",
	},
};

static int mt6329_disable_charger(struct regmap *regmap)
{
	static const struct {
		unsigned int reg, mask;
	} fields[] = {
		{ 0x21, BIT(4) | BIT(3) },
		{ 0x2e, BIT(4) },
		{ 0x30, BIT(1) | BIT(0) },
	};
	unsigned int i, value;
	int ret;

	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		ret = regmap_clear_bits(regmap, fields[i].reg, fields[i].mask);
		if (ret)
			return ret;
		ret = regmap_read(regmap, fields[i].reg, &value);
		if (ret)
			return ret;
		if (value & fields[i].mask)
			return -EIO;
	}
	return 0;
}

static int mt6329_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct i2c_client *bank1;
	struct clk *clock;
	struct regmap *regmap;
	unsigned int low, high;
	int ret;

	regmap = devm_regmap_init_i2c(i2c, &mt6329_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	ret = regmap_read(regmap, 0, &low);
	if (ret)
		return ret;
	ret = regmap_read(regmap, 1, &high);
	if (ret)
		return ret;
	if ((high << 8 | low) != 0x1029)
		return -ENODEV;

	if (device_property_read_bool(dev, "mediatek,external-charger")) {
		ret = mt6329_disable_charger(regmap);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to disable internal charger\n");
	}
	clock = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clock))
		return dev_err_probe(dev, PTR_ERR(clock), "Failed to enable reference clock\n");

	bank1 = devm_i2c_new_dummy_device(dev, i2c->adapter, 0x61);
	if (IS_ERR(bank1))
		return PTR_ERR(bank1);
	regmap = devm_regmap_init_i2c(bank1, &mt6329_bank1_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	ret = regmap_attach_dev(dev, regmap, &mt6329_bank1_regmap_config);
	if (ret)
		return ret;

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, mt6329_cells,
				    ARRAY_SIZE(mt6329_cells), NULL, 0, NULL);
}

static const struct of_device_id mt6329_of_match[] = {
	{ .compatible = "mediatek,mt6329" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6329_of_match);

static struct i2c_driver mt6329_driver = {
	.probe = mt6329_probe,
	.driver = {
		.name = "mt6329",
		.of_match_table = mt6329_of_match,
	},
};
module_i2c_driver(mt6329_driver);

MODULE_DESCRIPTION("MediaTek MT6329 PMIC driver");
MODULE_LICENSE("GPL");
