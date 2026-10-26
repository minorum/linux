// SPDX-License-Identifier: GPL-2.0-only

#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

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

static const struct regmap_config bcm59035_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
	.reg_read = bcm59035_reg_read,
	.reg_write = bcm59035_reg_write,
	.precious_reg = bcm59035_precious_reg,
};

static const struct mfd_cell bcm59035_cells[] = {
	{ .name = "bcm59035-regulator" },
};

static int bcm59035_i2c_probe(struct i2c_client *client)
{
	struct regmap *regmap;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	regmap = devm_regmap_init(&client->dev, NULL, client,
				  &bcm59035_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&client->dev, PTR_ERR(regmap),
				     "failed to initialize regmap\n");

	return devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				    bcm59035_cells, ARRAY_SIZE(bcm59035_cells),
				    NULL, 0, NULL);
}

static const struct of_device_id bcm59035_match[] = {
	{ .compatible = "brcm,bcm59035" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm59035_match);

static struct i2c_driver bcm59035_driver = {
	.probe = bcm59035_i2c_probe,
	.driver = {
		.name = "bcm59035",
		.of_match_table = bcm59035_match,
	},
};
module_i2c_driver(bcm59035_driver);

MODULE_DESCRIPTION("Broadcom BCM59035 power management IC");
MODULE_LICENSE("GPL");
