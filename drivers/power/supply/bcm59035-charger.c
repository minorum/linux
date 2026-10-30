// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define BCM59035_ENV1 0x4c

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

static int bcm59035_charger_get_property(struct power_supply *psy,
					 enum power_supply_property property,
					 union power_supply_propval *value)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	int online;

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
