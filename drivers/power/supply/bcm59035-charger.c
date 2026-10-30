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

#include "bcm59035-charger.h"

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
	bool inhibited;
	int battery_health;
	int max_current;
	int max_voltage;
	int term_current;
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
	if (!voltage)
		return bcm59035_current_ua(setting);
	return bcm59035_voltage_uv(setting);
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

static int bcm59035_charger_read_budget(struct bcm59035_charger *charger)
{
	union power_supply_propval value;
	int ret;

	ret = power_supply_get_property_from_supplier(charger->psy,
						      POWER_SUPPLY_PROP_ONLINE, &value);
	if (ret)
		return ret;
	if (value.intval <= 0)
		return -ENODEV;
	ret = power_supply_get_property_from_supplier(charger->psy,
						      POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	if (ret)
		return ret;
	return value.intval < 0 ? -EINVAL : value.intval;
}

static bool bcm59035_charger_profile_within_budget(struct bcm59035_charger *charger,
						   int budget)
{
	int ret;

	if (budget <= 0)
		return false;
	ret = bcm59035_charger_read_behaviour(charger);
	if (ret >= 0)
		return true;
	if (ret != -ENODATA)
		return false;
	ret = bcm59035_charger_read_profile(charger, false);
	return ret > 0 && ret <= budget;
}

static int bcm59035_charger_inhibit(struct bcm59035_charger *charger)
{
	int ret, usb_ret;

	charger->inhibited = false;
	ret = regmap_update_bits(charger->regmap, 0x37, 0xc0, 0);
	usb_ret = regmap_update_bits(charger->regmap, 0x3d, 0x40, 0);
	if (ret)
		return ret;
	if (usb_ret)
		return usb_ret;
	ret = bcm59035_charger_read_behaviour(charger);
	charger->inhibited = ret >= 0;
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
		value->intval = bcm59035_eoc_ua(setting);
		return 0;
	}

	if (property != POWER_SUPPLY_PROP_ONLINE &&
	    property != POWER_SUPPLY_PROP_STATUS)
		return -EINVAL;
	online = bcm59035_charger_read_online(charger);
	if (online < 0)
		return online;
	if (property == POWER_SUPPLY_PROP_ONLINE) {
		value->intval = online;
	} else {
		if (online && charger->inhibited) {
			ret = bcm59035_charger_read_behaviour(charger);
			if (ret >= 0) {
				value->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				return 0;
			}
			charger->inhibited = false;
			charger->inhibit_error = ret == -ENODATA ? -EIO : ret;
			if (ret != -ENODATA)
				return ret;
		}
		value->intval = online ? POWER_SUPPLY_STATUS_UNKNOWN :
					POWER_SUPPLY_STATUS_DISCHARGING;
	}
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

static int bcm59035_charger_set_profile(struct bcm59035_charger *charger,
					enum power_supply_property property, int value)
{
	unsigned int reg, mask, setting;
	int code, limit, count, i, ret;

	switch (property) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		reg = 0x39;
		mask = 0x1f;
		count = 2;
		limit = charger->max_current;
		code = bcm59035_current_reg(value);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		reg = 0x38;
		mask = 0x0f;
		count = 2;
		limit = charger->max_voltage;
		code = bcm59035_voltage_reg(value);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		reg = 0x3c;
		mask = 0x0f;
		count = 1;
		limit = charger->term_current;
		code = bcm59035_eoc_reg(value);
		break;
	default:
		return -EINVAL;
	}
	if (limit <= 0)
		return -ENODATA;
	if (code < 0 || value > limit)
		return -EINVAL;
	ret = bcm59035_charger_read_behaviour(charger);
	if (ret < 0)
		return ret == -ENODATA ? -EBUSY : ret;
	for (i = 0; i < count; i++, reg += 2) {
		ret = regmap_update_bits(charger->regmap, reg, mask, code);
		if (ret)
			return ret;
		ret = regmap_read(charger->regmap, reg, &setting);
		if (ret)
			return ret;
		if ((setting & mask) != (unsigned int)code)
			return -EIO;
	}
	return 0;
}

static int bcm59035_charger_set_property(struct power_supply *psy,
					 enum power_supply_property property,
					 const union power_supply_propval *value)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	int ret;

	if (property == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR &&
	    value->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
		return -EINVAL;
	mutex_lock(&charger->lock);
	if (property == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR) {
		ret = bcm59035_charger_inhibit(charger);
		charger->inhibit_error = ret;
	} else {
		ret = bcm59035_charger_set_profile(charger, property, value->intval);
	}
	mutex_unlock(&charger->lock);
	power_supply_changed(psy);
	return ret;
}

static int bcm59035_charger_property_is_writeable(struct power_supply *psy,
						  enum power_supply_property property)
{
	return property == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR ||
	       property == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT ||
	       property == POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE ||
	       property == POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT;
}

static int bcm59035_charger_init(struct power_supply *psy)
{
	struct bcm59035_charger *charger = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *info;
	int ret;

	ret = power_supply_get_battery_info(psy, &info);
	if (ret)
		return ret == -ENOENT || ret == -ENODEV ? 0 : ret;
	charger->max_current = info->constant_charge_current_max_ua;
	charger->max_voltage = info->constant_charge_voltage_max_uv;
	charger->term_current = info->charge_term_current_ua;
	power_supply_put_battery_info(psy, info);
	return 0;
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
	.init = bcm59035_charger_init,
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
	int budget = bcm59035_charger_read_budget(charger);
	bool inhibited, lost_inhibit, changed;

	mutex_lock(&charger->lock);
	inhibited = charger->inhibited;
	lost_inhibit = inhibited && bcm59035_charger_read_behaviour(charger) < 0;
	changed = health != charger->battery_health;
	charger->battery_health = health;
	if (online <= 0 || (events & (BCM59035_REMOVE_MASK | BCM59035_FAULT_MASK)) ||
	    charger->inhibit_error || lost_inhibit || health != POWER_SUPPLY_HEALTH_GOOD ||
	    !bcm59035_charger_profile_within_budget(charger, budget)) {
		int ret = bcm59035_charger_inhibit(charger);

		changed |= ret != charger->inhibit_error;
		charger->inhibit_error = ret;
	}
	changed |= inhibited != charger->inhibited || lost_inhibit;
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
