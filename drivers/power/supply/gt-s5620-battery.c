// SPDX-License-Identifier: GPL-2.0-only
#include <linux/iio/consumer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "gt-s5620-battery.h"

struct gt_s5620_battery {
	struct iio_channel *voltage;
	struct iio_channel *temperature;
	struct power_supply *psy;
	struct power_supply *source;
	struct mutex lock; /* Gauge state */
	struct delayed_work poll;
	struct gt_s5620_gauge gauge;
	u64 updated;
	bool sampled;
	int status;
};

static int gt_s5620_read_calibration(struct device *dev, const char *name,
				     u16 *result)
{
	struct nvmem_cell *cell;
	size_t length;
	void *data;
	int ret = 0;

	cell = devm_nvmem_cell_get(dev, name);
	if (IS_ERR(cell))
		return PTR_ERR(cell);
	data = nvmem_cell_read(cell, &length);
	if (IS_ERR(data))
		return PTR_ERR(data);
	if (length != sizeof(*result))
		ret = -EINVAL;
	else
		*result = get_unaligned_le16(data);
	kfree(data);
	return ret;
}

static int gt_s5620_read_raw(struct iio_channel *channel, int *result)
{
	int raw, ret;

	ret = iio_read_channel_raw(channel, &raw);
	if (ret < 0)
		return ret;
	if (raw < 0 || raw > 255)
		return -ERANGE;
	*result = raw;
	return 0;
}

static void gt_s5620_poll(struct work_struct *work)
{
	struct gt_s5620_battery *battery = container_of(to_delayed_work(work),
						      struct gt_s5620_battery, poll);
	struct gt_s5620_gauge old;
	union power_supply_propval value;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int mode = -ENODATA, raw = 0, ret, error;
	bool changed;

	if (battery->source) {
		ret = power_supply_get_property(battery->source,
						POWER_SUPPLY_PROP_STATUS, &value);
		if (ret) {
			mode = ret;
		} else if (value.intval >= POWER_SUPPLY_STATUS_CHARGING &&
			   value.intval <= POWER_SUPPLY_STATUS_FULL) {
			status = value.intval;
			mode = status == POWER_SUPPLY_STATUS_CHARGING;
		}
	}
	error = gt_s5620_read_raw(battery->voltage, &raw);
	mutex_lock(&battery->lock);
	old = battery->gauge;
	gt_s5620_gauge_update(&battery->gauge, error, raw, mode);
	changed = !battery->sampled || battery->status != status ||
		old.average != battery->gauge.average ||
		old.average_error != battery->gauge.average_error ||
		old.capacity_error != battery->gauge.capacity_error ||
		old.level != battery->gauge.level;
	battery->status = status;
	battery->updated = ktime_get_boottime_ns();
	battery->sampled = true;
	mutex_unlock(&battery->lock);
	if (changed)
		power_supply_changed(battery->psy);
	queue_delayed_work(system_freezable_wq, &battery->poll, HZ);
}

static void gt_s5620_cancel_poll(void *data)
{
	struct gt_s5620_battery *battery = data;

	cancel_delayed_work_sync(&battery->poll);
}

static int gt_s5620_get_property(struct power_supply *psy,
				 enum power_supply_property property,
				 union power_supply_propval *value)
{
	struct gt_s5620_battery *battery = power_supply_get_drvdata(psy);
	int raw, ret;

	switch (property) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = gt_s5620_read_raw(battery->voltage, &raw);
		if (ret)
			return ret;
		ret = gt_s5620_voltage_raw(raw, battery->gauge.calibration[0],
					   battery->gauge.calibration[1]);
		if (ret < 0)
			return ret;
		value->intval = ret * 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = gt_s5620_read_raw(battery->temperature, &raw);
		if (ret)
			return ret;
		return gt_s5620_temperature(raw, &value->intval);
	default:
		break;
	}
	mutex_lock(&battery->lock);
	ret = 0;
	if (!gt_s5620_cache_fresh(battery->sampled, battery->updated,
				  ktime_get_boottime_ns())) {
		if (property == POWER_SUPPLY_PROP_STATUS)
			value->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else
			ret = -EAGAIN;
		goto out;
	}
	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		value->intval = battery->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = battery->gauge.average_error;
		if (!ret)
			value->intval = battery->gauge.average * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = battery->gauge.capacity_error;
		if (!ret)
			value->intval = battery->gauge.level == 6 ? 0 : battery->gauge.level * 20;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = battery->gauge.capacity_error;
		if (ret)
			break;
		if (battery->gauge.level == 6)
			value->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else if (battery->gauge.level <= 1)
			value->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (battery->gauge.level <= 3)
			value->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (battery->gauge.level == 4)
			value->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else
			value->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	mutex_unlock(&battery->lock);
	return ret;
}

static const enum power_supply_property gt_s5620_properties[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static const struct power_supply_desc gt_s5620_supply = {
	.name = "gt-s5620-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = gt_s5620_properties,
	.num_properties = ARRAY_SIZE(gt_s5620_properties),
	.get_property = gt_s5620_get_property,
};

static int gt_s5620_battery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct gt_s5620_battery *battery;
	struct power_supply *psy;
	u16 low, high;
	int ret;

	battery = devm_kzalloc(dev, sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -ENOMEM;
	battery->voltage = devm_iio_channel_get(dev, "voltage");
	if (IS_ERR(battery->voltage))
		return dev_err_probe(dev, PTR_ERR(battery->voltage), "No voltage channel\n");
	battery->temperature = devm_iio_channel_get(dev, "temperature");
	if (IS_ERR(battery->temperature))
		return dev_err_probe(dev, PTR_ERR(battery->temperature),
				     "No temperature channel\n");
	ret = gt_s5620_read_calibration(dev, "voltage-3v3", &low);
	if (ret)
		return dev_err_probe(dev, ret, "No 3.3 V calibration\n");
	ret = gt_s5620_read_calibration(dev, "voltage-4v2", &high);
	if (ret)
		return dev_err_probe(dev, ret, "No 4.2 V calibration\n");
	ret = gt_s5620_gauge_init(&battery->gauge, low, high);
	if (ret)
		return dev_err_probe(dev, ret, "Invalid voltage calibration\n");
	if (device_property_present(dev, "power-supplies")) {
		battery->source = devm_power_supply_get_by_reference(dev, "power-supplies");
		if (IS_ERR(battery->source))
			return dev_err_probe(dev, PTR_ERR(battery->source),
					     "No charger reference\n");
		if (!battery->source)
			return -EPROBE_DEFER;
	}
	mutex_init(&battery->lock);
	config.fwnode = dev_fwnode(dev);
	config.drv_data = battery;
	psy = devm_power_supply_register(dev, &gt_s5620_supply, &config);
	if (IS_ERR(psy))
		return PTR_ERR(psy);
	battery->psy = psy;
	INIT_DELAYED_WORK(&battery->poll, gt_s5620_poll);
	ret = devm_add_action_or_reset(dev, gt_s5620_cancel_poll, battery);
	if (ret)
		return ret;
	queue_delayed_work(system_freezable_wq, &battery->poll, 0);
	return 0;
}

static const struct of_device_id gt_s5620_battery_match[] = {
	{ .compatible = "samsung,gt-s5620-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, gt_s5620_battery_match);

static struct platform_driver gt_s5620_battery_driver = {
	.probe = gt_s5620_battery_probe,
	.driver = {
		.name = "gt-s5620-battery",
		.of_match_table = gt_s5620_battery_match,
	},
};
module_platform_driver(gt_s5620_battery_driver);

MODULE_DESCRIPTION("Samsung GT-S5620 battery monitor");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
