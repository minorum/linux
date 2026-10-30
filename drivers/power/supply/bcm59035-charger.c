// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/mfd/bcm59035.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define BCM59035_ENV1 0x4c
#define BCM59035_MBCCTRL7 0x3c
#define BCM59035_REMOVE_MASK 0x22
#define BCM59035_VOLTAGE_ERROR_MASK 0x44
#define BCM59035_TIMER_ERROR 0x80
#define BCM59035_SNAPSHOT_ERROR 0x100
#define BCM59035_FAULT_MASK 0x1c4

struct bcm59035_charger {
	struct regmap *regmap;
	struct device *parent;
	struct power_supply *psy;
	struct delayed_work poll;
	struct mutex lock; /* Control and readback */
	int last_online;
	int inhibit_error;
	int battery_health;
	atomic_t events;
	unsigned int faults;
};

struct bcm59035_battery_check {
	struct fwnode_handle *battery;
	int health;
	bool found;
};

static int bcm59035_charger_check_battery(struct power_supply *psy, void *data)
{
	struct bcm59035_battery_check *check = data;
	struct fwnode_handle *node, *battery;
	union power_supply_propval value;
	bool match;
	int ret;

	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;
	node = dev_fwnode(&psy->dev);
	if (!node && psy->dev.parent)
		node = dev_fwnode(psy->dev.parent);
	battery = fwnode_find_reference(node, "monitored-battery", 0);
	if (IS_ERR(battery))
		return 0;
	match = battery == check->battery;
	fwnode_handle_put(battery);
	if (!match)
		return 0;
	if (check->found)
		return -EEXIST;
	check->found = true;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_HEALTH, &value);
	if (ret)
		return ret;
	check->health = value.intval;
	return 0;
}

static int bcm59035_charger_read_battery_health(struct bcm59035_charger *charger)
{
	struct bcm59035_battery_check check = {};
	int ret;

	check.battery = fwnode_find_reference(dev_fwnode(&charger->psy->dev),
					      "monitored-battery", 0);
	if (IS_ERR(check.battery))
		return PTR_ERR(check.battery);
	ret = power_supply_for_each_psy(&check, bcm59035_charger_check_battery);
	fwnode_handle_put(check.battery);
	if (ret)
		return ret;
	return check.found ? check.health : -ENODEV;
}

static irqreturn_t bcm59035_charger_irq(int irq, void *data)
{
	struct bcm59035_charger *charger = data;
	int pending = bcm59035_irq_get_pending(charger->parent, 1);
	unsigned int faults = READ_ONCE(charger->faults);

	if (pending >= 0 && (pending & BCM59035_REMOVE_MASK))
		faults = 0;
	faults |= pending < 0 ? BCM59035_SNAPSHOT_ERROR : pending & BCM59035_FAULT_MASK;
	WRITE_ONCE(charger->faults, faults);
	atomic_or(pending < 0 ? BCM59035_SNAPSHOT_ERROR : pending, &charger->events);
	mod_delayed_work(system_freezable_wq, &charger->poll, 0);
	return IRQ_HANDLED;
}

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

static int bcm59035_charger_read_behaviour(struct bcm59035_charger *charger)
{
	unsigned int wall, usb;
	int ret;

	ret = regmap_read(charger->regmap, 0x37, &wall);
	if (ret)
		return ret;
	ret = regmap_read(charger->regmap, 0x3d, &usb);
	if (ret)
		return ret;
	if ((wall & 0xc0) || (usb & 0x40))
		return -ENODATA;
	return POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
}

static int bcm59035_charger_inhibit(struct bcm59035_charger *charger)
{
	int ret, usb_ret;

	ret = regmap_update_bits(charger->regmap, 0x37, 0xc0, 0);
	usb_ret = regmap_update_bits(charger->regmap, 0x3d, 0x40, 0);
	if (ret)
		return ret;
	if (usb_ret)
		return usb_ret;
	ret = bcm59035_charger_read_behaviour(charger);
	return ret == -ENODATA ? -EIO : ret < 0 ? ret : 0;
}

static int bcm59035_charger_read_property(struct power_supply *psy,
					  enum power_supply_property property,
					  union power_supply_propval *value)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	unsigned int setting;
	int online, ret;

	if (property == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR) {
		ret = bcm59035_charger_read_behaviour(charger);
		if (ret < 0)
			return ret;
		value->intval = ret;
		return 0;
	}
	if (property == POWER_SUPPLY_PROP_HEALTH) {
		unsigned int faults = READ_ONCE(charger->faults);

		if (charger->inhibit_error || charger->battery_health < 0 ||
		    (faults & BCM59035_SNAPSHOT_ERROR) ||
		    ((faults & BCM59035_VOLTAGE_ERROR_MASK) && (faults & BCM59035_TIMER_ERROR)))
			value->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (faults & BCM59035_VOLTAGE_ERROR_MASK)
			value->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (faults & BCM59035_TIMER_ERROR)
			value->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		else if (charger->battery_health != POWER_SUPPLY_HEALTH_GOOD)
			value->intval = charger->battery_health;
		else
			value->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		return 0;
	}
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

static int bcm59035_charger_get_property(struct power_supply *psy,
					 enum power_supply_property property,
					 union power_supply_propval *value)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&charger->lock);
	ret = bcm59035_charger_read_property(psy, property, value);
	mutex_unlock(&charger->lock);
	return ret;
}

static int bcm59035_charger_set_property(struct power_supply *psy,
					 enum power_supply_property property,
					 const union power_supply_propval *value)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	int ret;

	if (property != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR ||
	    value->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
		return -EINVAL;
	mutex_lock(&charger->lock);
	ret = bcm59035_charger_inhibit(charger);
	charger->inhibit_error = ret;
	mutex_unlock(&charger->lock);
	power_supply_changed(psy);
	return ret;
}

static int bcm59035_charger_property_is_writeable(struct power_supply *psy,
						  enum power_supply_property property)
{
	return property == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static const enum power_supply_property bcm59035_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static const struct power_supply_desc bcm59035_charger_desc = {
	.name = "bcm59035-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = bcm59035_charger_properties,
	.num_properties = ARRAY_SIZE(bcm59035_charger_properties),
	.get_property = bcm59035_charger_get_property,
	.set_property = bcm59035_charger_set_property,
	.property_is_writeable = bcm59035_charger_property_is_writeable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static void bcm59035_charger_poll(struct work_struct *work)
{
	struct bcm59035_charger *charger = container_of(to_delayed_work(work),
						      struct bcm59035_charger, poll);
	unsigned int events = atomic_xchg(&charger->events, 0);
	int online = bcm59035_charger_read_online(charger);
	int health = bcm59035_charger_read_battery_health(charger);
	bool changed = false;

	mutex_lock(&charger->lock);
	changed = health != charger->battery_health;
	charger->battery_health = health;
	if (online <= 0 || (events & (BCM59035_REMOVE_MASK | BCM59035_FAULT_MASK)) ||
	    charger->inhibit_error || health != POWER_SUPPLY_HEALTH_GOOD) {
		int ret = bcm59035_charger_inhibit(charger);

		changed |= ret != charger->inhibit_error;
		charger->inhibit_error = ret;
	}
	mutex_unlock(&charger->lock);
	if (online != charger->last_online || events || changed) {
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
	static const char * const irq_names[] = {
		"charger-insert", "charger-remove", "charger-error", "charge-complete",
		"usb-insert", "usb-remove", "usb-error", "charge-error",
	};
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct bcm59035_charger *charger;
	unsigned int i;
	int ret, irq;

	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;
	mutex_init(&charger->lock);
	charger->parent = dev->parent;
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
	for (i = 0; i < ARRAY_SIZE(irq_names); i++) {
		irq = platform_get_irq_byname(pdev, irq_names[i]);
		if (irq < 0)
			return dev_err_probe(dev, irq, "No charger IRQ\n");
		ret = devm_request_threaded_irq(dev, irq, NULL, bcm59035_charger_irq,
						IRQF_ONESHOT, dev_name(dev), charger);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to request charger IRQ\n");
	}
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

MODULE_DESCRIPTION("Broadcom BCM59035 charger");
MODULE_LICENSE("GPL");
