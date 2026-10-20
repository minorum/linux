// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>

#define BCM59035_H2	0x1b
#define BCM59035_HOSTACT	0x51
#define BCM59035_HCLDO	0xb5

struct bcm59035 {
	struct i2c_client *client;
	struct regulator_desc desc;
	bool failed;
};

static const int bcm59035_hc2_voltages[] = {
	2500000, 2600000, 2700000, 2800000,
	2900000, 3200000, 3100000, 3000000,
};

static int bcm59035_read(struct bcm59035 *pmu, u8 reg)
{
	u8 value;
	struct i2c_msg msgs[] = {
		{ .addr = pmu->client->addr, .flags = I2C_M_STOP,
		  .len = 1, .buf = &reg },
		{ .addr = pmu->client->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = &value },
	};
	int ret;

	if (pmu->failed)
		return -ESHUTDOWN;
	ret = i2c_transfer(pmu->client->adapter, msgs, 2);
	if (ret == 2)
		return value;
	pmu->failed = true;
	return ret < 0 ? ret : -EIO;
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
	u8 bytes[] = { BCM59035_H2, 0x11 };
	struct i2c_msg msg = {
		.addr = pmu->client->addr, .len = sizeof(bytes), .buf = bytes,
	};
	int ret;

	ret = bcm59035_is_enabled(rdev);
	if (ret)
		return ret < 0 ? ret : 0;
	ret = i2c_transfer(pmu->client->adapter, &msg, 1);
	if (ret != 1) {
		pmu->failed = true;
		return ret < 0 ? ret : -EIO;
	}
	msleep(1000);
	ret = bcm59035_read(pmu, BCM59035_H2);
	if (ret < 0)
		return ret;
	if (ret != 0x11) {
		pmu->failed = true;
		return -EIO;
	}
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

static int bcm59035_probe(struct i2c_client *client)
{
	struct regulator_config config = { .dev = &client->dev };
	struct regulator_dev *rdev;
	struct bcm59035 *pmu;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	pmu = devm_kzalloc(&client->dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;
	pmu->client = client;
	ret = bcm59035_read(pmu, BCM59035_HOSTACT);
	if (ret < 0)
		return ret;
	if (ret)
		return dev_err_probe(&client->dev, -EOPNOTSUPP, "unsupported HOSTACT\n");
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
	rdev = devm_regulator_register(&client->dev, &pmu->desc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);
	dev_info(&client->dev, "HCLDO2 retains %u uV\n", pmu->desc.min_uV);
	return 0;
}

static const struct of_device_id bcm59035_match[] = {
	{ .compatible = "brcm,bcm59035" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm59035_match);

static struct i2c_driver bcm59035_driver = {
	.probe = bcm59035_probe,
	.driver = {
		.name = "bcm59035-regulator",
		.of_match_table = bcm59035_match,
	},
};
module_i2c_driver(bcm59035_driver);

MODULE_DESCRIPTION("Broadcom BCM59035 retained-voltage HCLDO2");
MODULE_LICENSE("GPL");
