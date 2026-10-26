// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define BCM59035_H2	0x1b
#define BCM59035_HOSTACT	0x51
#define BCM59035_HCLDO	0xb5

struct bcm59035 {
	struct regmap *regmap;
	struct regulator_desc desc;
	bool failed;
};

static const int bcm59035_hc2_voltages[] = {
	2500000, 2600000, 2700000, 2800000,
	2900000, 3200000, 3100000, 3000000,
};

static int bcm59035_read(struct bcm59035 *pmu, u8 reg)
{
	unsigned int value;
	int ret;

	if (pmu->failed)
		return -ESHUTDOWN;
	ret = regmap_read(pmu->regmap, reg, &value);
	if (!ret)
		return value;
	pmu->failed = true;
	return ret;
}

static int bcm59035_is_enabled(struct regulator_dev *rdev)
{
	struct bcm59035 *pmu = rdev_get_drvdata(rdev);
	int ret;

	ret = bcm59035_read(pmu, BCM59035_HOSTACT);
	if (ret < 0)
		return ret;
	if (ret)
		return -EOPNOTSUPP;
	ret = bcm59035_read(pmu, BCM59035_H2);
	if (ret < 0)
		return ret;
	switch (ret) {
	case 0x00:
	case 0x11:
		return 1;
	case 0xaa:
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int bcm59035_enable(struct regulator_dev *rdev)
{
	struct bcm59035 *pmu = rdev_get_drvdata(rdev);
	int ret;

	ret = bcm59035_is_enabled(rdev);
	if (ret)
		return ret < 0 ? ret : 0;
	dev_dbg(rdev_get_dev(rdev), "HCLDO2 mode 0xaa observed\n");
	ret = regmap_write(pmu->regmap, BCM59035_H2, 0x11);
	if (ret) {
		pmu->failed = true;
		return ret;
	}
	dev_dbg(rdev_get_dev(rdev), "HCLDO2 mode 0x11 written\n");
	msleep(1000);
	ret = bcm59035_read(pmu, BCM59035_H2);
	if (ret < 0)
		return ret;
	if (ret != 0x11) {
		pmu->failed = true;
		return -EIO;
	}
	dev_dbg(rdev_get_dev(rdev), "HCLDO2 mode 0x11 verified\n");
	return 0;
}

static int bcm59035_get_voltage(struct regulator_dev *rdev)
{
	struct bcm59035 *pmu = rdev_get_drvdata(rdev);
	int ret;

	ret = bcm59035_read(pmu, BCM59035_HCLDO);
	if (ret < 0)
		return ret;
	ret = bcm59035_hc2_voltages[(ret >> 4) & 7];
	if (ret != pmu->desc.min_uV) {
		pmu->failed = true;
		return -EIO;
	}
	return ret;
}

static const struct regulator_ops bcm59035_ops = {
	.enable = bcm59035_enable,
	.is_enabled = bcm59035_is_enabled,
	.get_voltage = bcm59035_get_voltage,
	.list_voltage = regulator_list_voltage_linear,
};

static int bcm59035_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { .dev = dev->parent };
	struct regulator_dev *rdev;
	struct bcm59035 *pmu;
	int ret;

	pmu = devm_kzalloc(dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;
	pmu->regmap = dev_get_regmap(dev->parent, NULL);
	if (!pmu->regmap)
		return -ENODEV;
	ret = bcm59035_read(pmu, BCM59035_HOSTACT);
	if (ret < 0)
		return ret;
	if (ret)
		return dev_err_probe(dev, -EOPNOTSUPP, "unsupported HOSTACT\n");
	ret = bcm59035_read(pmu, BCM59035_HCLDO);
	if (ret < 0)
		return ret;
	pmu->desc.name = "bcm59035-hcldo2";
	pmu->desc.of_match = "hcldo2";
	pmu->desc.regulators_node = "regulators";
	pmu->desc.owner = THIS_MODULE;
	pmu->desc.type = REGULATOR_VOLTAGE;
	pmu->desc.ops = &bcm59035_ops;
	pmu->desc.n_voltages = 1;
	pmu->desc.min_uV = bcm59035_hc2_voltages[(ret >> 4) & 7];
	config.driver_data = pmu;
	rdev = devm_regulator_register(dev, &pmu->desc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);
	dev_info(dev, "HCLDO2 retains %u uV\n", pmu->desc.min_uV);
	return 0;
}

static struct platform_driver bcm59035_driver = {
	.probe = bcm59035_probe,
	.driver = {
		.name = "bcm59035-regulator",
	},
};
module_platform_driver(bcm59035_driver);

MODULE_ALIAS("platform:bcm59035-regulator");
MODULE_DESCRIPTION("Broadcom BCM59035 retained-voltage HCLDO2");
MODULE_LICENSE("GPL");
