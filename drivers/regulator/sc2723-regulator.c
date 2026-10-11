// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

struct sc2723_regulator {
	struct regulator_desc desc;
	unsigned int selector;
};

static int sc2723_rf_selector(unsigned int code, unsigned int bits,
			      int target, int minimum, int step)
{
	int delta;

	if (code >= BIT(bits))
		return -EINVAL;
	delta = (int)code - (int)BIT(bits - 1);
	return DIV_ROUND_UP(target - minimum + delta * step, step);
}

static int sc2723_rf_is_enabled(struct regulator_dev *rdev)
{
	struct sc2723_regulator *regulator = rdev_get_drvdata(rdev);
	unsigned int selector;
	int ret;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret <= 0)
		return ret;
	ret = regmap_read(rdev->regmap, rdev->desc->vsel_reg, &selector);
	if (ret)
		return ret;
	if ((selector & rdev->desc->vsel_mask) != regulator->selector)
		return -EINVAL;
	return 1;
}

static int sc2723_rf_enable(struct regulator_dev *rdev)
{
	struct sc2723_regulator *regulator = rdev_get_drvdata(rdev);
	int ret, cleanup;

	ret = sc2723_rf_is_enabled(rdev);
	if (ret)
		return ret < 0 ? ret : 0;
	ret = regmap_update_bits(rdev->regmap, rdev->desc->vsel_reg,
				 rdev->desc->vsel_mask, regulator->selector);
	if (ret)
		return ret;
	ret = regulator_enable_regmap(rdev);
	if (ret) {
		cleanup = regulator_disable_regmap(rdev);
		if (cleanup)
			dev_err(&rdev->dev, "RF disable failed: %d\n", cleanup);
	}
	return ret;
}

static const struct regulator_ops sc2723_rf_ops = {
	.enable = sc2723_rf_enable,
	.disable = regulator_disable_regmap,
	.is_enabled = sc2723_rf_is_enabled,
};

static const struct regulator_desc sc2723_rf_descs[] = {
	{
		.name = "vddcon",
		.of_match = "vddcon",
		.ops = &sc2723_rf_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 1600000,
		.enable_reg = 0x814,
		.enable_mask = BIT(13),
		.enable_is_inverted = true,
		.vsel_reg = 0xa18,
		.vsel_mask = GENMASK(9, 0),
	},
	{
		.name = "vddwifipa",
		.of_match = "vddwifipa",
		.id = 1,
		.ops = &sc2723_rf_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 3300000,
		.enable_reg = 0x814,
		.enable_mask = BIT(11),
		.enable_is_inverted = true,
		.vsel_reg = 0x818,
		.vsel_mask = GENMASK(7, 0),
	},
};

static int sc2723_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { .dev = dev };
	struct sc2723_regulator *regulators;
	struct regulator_dev *rdev;
	u32 code, power, selector;
	u8 state;
	int i, ret;

	config.regmap = dev_get_regmap(dev->parent, NULL);
	if (!config.regmap)
		return -ENODEV;
	ret = nvmem_cell_read_u8(dev, "calibration_state", &state);
	if (ret)
		return dev_err_probe(dev, ret, "Calibration state unavailable\n");
	if (state)
		return dev_err_probe(dev, -EINVAL, "Factory calibration invalid\n");
	regulators = devm_kcalloc(dev, ARRAY_SIZE(sc2723_rf_descs),
				  sizeof(*regulators), GFP_KERNEL);
	if (!regulators)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(sc2723_rf_descs); i++) {
		regulators[i].desc = sc2723_rf_descs[i];
		ret = nvmem_cell_read_variable_le_u32(dev, sc2723_rf_descs[i].name,
						      &code);
		if (ret)
			return dev_err_probe(dev, ret, "RF calibration unavailable\n");
		ret = sc2723_rf_selector(code, i ? 6 : 7,
					 sc2723_rf_descs[i].fixed_uV,
					 i ? 1200000 : 600000, i ? 10000 : 3125);
		if (ret < 0 || ret > sc2723_rf_descs[i].vsel_mask)
			return dev_err_probe(dev, -EINVAL, "RF calibration out of range\n");
		regulators[i].selector = ret;
	}
	ret = regmap_read(config.regmap, 0x814, &power);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(sc2723_rf_descs); i++) {
		if (power & sc2723_rf_descs[i].enable_mask)
			continue;
		ret = regmap_read(config.regmap, sc2723_rf_descs[i].vsel_reg, &selector);
		if (ret)
			return ret;
		if ((selector & sc2723_rf_descs[i].vsel_mask) != regulators[i].selector)
			return dev_err_probe(dev, -EINVAL, "RF voltage already differs\n");
	}
	for (i = 0; i < ARRAY_SIZE(sc2723_rf_descs); i++) {
		config.driver_data = &regulators[i];
		rdev = devm_regulator_register(dev, &regulators[i].desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "RF registration failed\n");
	}
	return 0;
}

static const struct of_device_id sc2723_regulator_match[] = {
	{ .compatible = "sprd,sc2723-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2723_regulator_match);

static struct platform_driver sc2723_regulator_driver = {
	.driver = {
		.name = "sc2723-regulator",
		.of_match_table = sc2723_regulator_match,
	},
	.probe = sc2723_regulator_probe,
};
module_platform_driver(sc2723_regulator_driver);

MODULE_DESCRIPTION("SC2723 calibrated connectivity regulators");
MODULE_LICENSE("GPL");
