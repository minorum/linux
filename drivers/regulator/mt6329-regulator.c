// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MT6329_VUSB 0x9b
#define MT6329_VMC  0xa7

static const unsigned int mt6329_vmc_voltages[] = {
	1300000, 1500000, 1800000, 2500000, 2800000, 3000000, 3300000, 3300000,
};

static int mt6329_check(struct regmap *map)
{
	const unsigned int controls[] = { MT6329_VUSB, MT6329_VMC };
	unsigned int low, high, value, i;
	int ret;

	ret = regmap_read(map, 0, &low);
	if (ret)
		return ret;
	ret = regmap_read(map, 1, &high);
	if (ret)
		return ret;
	if ((high << 8 | low) != 0x1029)
		return -ENODEV;
	for (i = 0; i < ARRAY_SIZE(controls); i++) {
		ret = regmap_read(map, controls[i], &value);
		if (ret)
			return ret;
		if (value & BIT(1))
			return -EOPNOTSUPP;
	}
	return 0;
}

static int mt6329_get_status(struct regulator_dev *rdev)
{
	unsigned int value;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &value);
	if (ret)
		return ret;
	return value & BIT(7) ? REGULATOR_STATUS_ON : REGULATOR_STATUS_OFF;
}

static const struct regulator_ops mt6329_vusb_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_status = mt6329_get_status,
};

static const struct regulator_ops mt6329_vmc_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_status = mt6329_get_status,
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_desc mt6329_regulators[] = {
	{
		.name = "vusb",
		.of_match = "vusb",
		.regulators_node = "regulators",
		.id = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &mt6329_vusb_ops,
		.n_voltages = 1,
		.fixed_uV = 3300000,
		.enable_reg = MT6329_VUSB,
		.enable_mask = BIT(0),
		.enable_time = 10000,
	}, {
		.name = "vmc",
		.of_match = "vmc",
		.regulators_node = "regulators",
		.id = 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &mt6329_vmc_ops,
		.n_voltages = ARRAY_SIZE(mt6329_vmc_voltages),
		.volt_table = mt6329_vmc_voltages,
		.vsel_reg = MT6329_VMC,
		.vsel_mask = 0x70,
		.enable_reg = MT6329_VMC,
		.enable_mask = BIT(0),
	},
};

static int mt6329_probe(struct platform_device *pdev)
{
	struct regulator_config config = { .dev = pdev->dev.parent };
	struct regulator_dev *rdev;
	struct regmap *map;
	unsigned int i;
	int ret;

	map = dev_get_regmap(pdev->dev.parent, "power");
	if (!map)
		return -ENODEV;
	ret = mt6329_check(map);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Unsupported PMIC identity or control mode\n");
	config.regmap = map;
	for (i = 0; i < ARRAY_SIZE(mt6329_regulators); i++) {
		rdev = devm_regulator_register(&pdev->dev, &mt6329_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
					     "Cannot register regulator\n");
	}
	return 0;
}

static const struct platform_device_id mt6329_ids[] = {
	{ "mt6329-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, mt6329_ids);

static struct platform_driver mt6329_driver = {
	.driver = {
		.name = "mt6329-regulator",
	},
	.probe = mt6329_probe,
	.id_table = mt6329_ids,
};
module_platform_driver(mt6329_driver);

MODULE_DESCRIPTION("MediaTek MT6329 VUSB and VMC regulators");
MODULE_LICENSE("GPL");
