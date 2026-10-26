// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

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
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	regmap = devm_regmap_init(&client->dev, NULL, client,
				  &bcm59035_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&client->dev, PTR_ERR(regmap),
				     "failed to initialize regmap\n");

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   bcm59035_cells, ARRAY_SIZE(bcm59035_cells),
				   NULL, 0, NULL);
	if (ret)
		return ret;

	ret = bcm59035_register_power_off(&client->dev, regmap);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register power-off handlers\n");

	return 0;
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
