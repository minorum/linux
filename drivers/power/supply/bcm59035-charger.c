// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define BCM59035_ENV1 0x4c
#define BCM59035_MBCCTRL7 0x3c

struct bcm59035_charger {
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll;
	int last_online;
};

static int bcm59035_charger_read_online(struct bcm59035_charger *charger)
{
	unsigned int env1;
	int ret;

	ret = regmap_read(charger->regmap, BCM59035_ENV1, &env1);
	if (ret)
		return ret;
	return !!(env1 & 0x06);
}

static int bcm59035_charger_read_profile(struct bcm59035_charger *charger, bool voltage)
{
	static const unsigned int regs[] = { 0x37, 0x3e, 0x4c, 0x4f, 0xc9, 0xca };
	unsigned int state[ARRAY_SIZE(regs)], setting, after, reg, i;
	bool wall, usb, changed = false;
	int ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = regmap_read(charger->regmap, regs[i], &state[i]);
		if (ret)
			return ret;
	}
	if (!(state[0] & 0x40) || (state[1] & 2) || (state[5] & 0x10))
		return -ENODATA;
	usb = !!(state[2] & 4);
	wall = !!(state[2] & 2);
	if (state[1] & 8)
		usb &= !!(state[3] & 8);
	else
		wall &= !usb;
	if ((!wall && !usb) || (wall && !usb && !(state[3] & 8) && (state[4] & 0x80)))
		return -ENODATA;
	reg = wall ? 0x38 : 0x3a;
	if (!voltage)
		reg++;
	ret = regmap_read(charger->regmap, reg, &setting);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = regmap_read(charger->regmap, regs[i], &after);
		if (ret)
			return ret;
		changed |= after != state[i];
	}
	if (changed)
		return -EAGAIN;
	if (!voltage) {
		setting &= 0x1f;
		return setting < 16 ? 50000 + setting * 10000 :
				     250000 + (setting - 16) * 50000;
	}
	setting &= 0x0f;
	if (setting < 7)
		return 4200000 - setting * 50000;
	if (setting < 12)
		return 3200000 + (setting - 7) * 100000;
	return -EINVAL;
}

static int bcm59035_charger_get_property(struct power_supply *psy,
					 enum power_supply_property property,
					 union power_supply_propval *value)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	unsigned int setting;
	int online, ret;

	if (property == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT ||
	    property == POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE) {
		bool voltage = property == POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE;

		ret = bcm59035_charger_read_profile(charger, voltage);
		if (ret < 0)
			return ret;
		value->intval = ret;
		return 0;
	}
	if (property == POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT) {
		ret = regmap_read(charger->regmap, BCM59035_MBCCTRL7, &setting);
		if (ret)
			return ret;
		value->intval = 50000 + (setting & 0x0f) * 10000;
		return 0;
	}

	if (property != POWER_SUPPLY_PROP_ONLINE &&
	    property != POWER_SUPPLY_PROP_STATUS)
		return -EINVAL;
	online = bcm59035_charger_read_online(charger);
	if (online < 0)
		return online;
	if (property == POWER_SUPPLY_PROP_ONLINE)
		value->intval = online;
	else
		value->intval = online ? POWER_SUPPLY_STATUS_UNKNOWN :
					POWER_SUPPLY_STATUS_DISCHARGING;
	return 0;
}

static const enum power_supply_property bcm59035_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static const struct power_supply_desc bcm59035_charger_desc = {
	.name = "bcm59035-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = bcm59035_charger_properties,
	.num_properties = ARRAY_SIZE(bcm59035_charger_properties),
	.get_property = bcm59035_charger_get_property,
};

static void bcm59035_charger_poll(struct work_struct *work)
{
	struct bcm59035_charger *charger = container_of(to_delayed_work(work),
						      struct bcm59035_charger, poll);
	int online = bcm59035_charger_read_online(charger);

	if (online != charger->last_online) {
		charger->last_online = online;
		power_supply_changed(charger->psy);
	}
	queue_delayed_work(system_freezable_wq, &charger->poll, 2 * HZ);
}

static void bcm59035_charger_stop(void *data)
{
	struct bcm59035_charger *charger = data;

	cancel_delayed_work_sync(&charger->poll);
}

static int bcm59035_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct bcm59035_charger *charger;
	int ret;

	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;
	charger->regmap = dev_get_regmap(dev->parent, NULL);
	if (!charger->regmap)
		return -ENODEV;
	ret = bcm59035_charger_read_online(charger);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read charger presence\n");
	charger->last_online = ret;
	config.drv_data = charger;
	config.fwnode = dev_fwnode(dev);
	charger->psy = devm_power_supply_register(dev, &bcm59035_charger_desc, &config);
	if (IS_ERR(charger->psy))
		return PTR_ERR(charger->psy);
	INIT_DELAYED_WORK(&charger->poll, bcm59035_charger_poll);
	ret = devm_add_action_or_reset(dev, bcm59035_charger_stop, charger);
	if (ret)
		return ret;
	queue_delayed_work(system_freezable_wq, &charger->poll, 2 * HZ);
	return 0;
}

static const struct of_device_id bcm59035_charger_of_match[] = {
	{ .compatible = "brcm,bcm59035-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm59035_charger_of_match);

static struct platform_driver bcm59035_charger_driver = {
	.driver = {
		.name = "bcm59035-charger",
		.of_match_table = bcm59035_charger_of_match,
	},
	.probe = bcm59035_charger_probe,
};
module_platform_driver(bcm59035_charger_driver);

MODULE_DESCRIPTION("Broadcom BCM59035 charger presence monitor");
MODULE_LICENSE("GPL");
