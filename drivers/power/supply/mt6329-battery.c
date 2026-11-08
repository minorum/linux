// SPDX-License-Identifier: GPL-2.0-only
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#define MT6329_FGADC_CLOCK	0x15
#define MT6329_FGADC_CLOCK_PDN	BIT(4)
#define MT6329_FGADC_MODE		0x69
#define MT6329_FGADC_CONTROL	0x6a
#define MT6329_FGADC_CURRENT_H	0x78
#define MT6329_FGADC_CURRENT_L	0x79
#define MT6329_FGADC_READY	BIT(2)

struct mt6329_fgadc {
	struct regmap *regmap;
	struct regmap *power;
	struct mutex lock; /* Current sampling */
	u32 shunt_uohm;
	u32 calibration_percent;
	bool restore_pending;
	struct power_supply *battery;
	struct iio_channel *voltage;
	struct iio_channel *temperature;
	struct delayed_work work;
	int status;
	int charge;
	int capacity;
	bool soc_valid;
	u16 soc_count;
	u64 soc_time_ms;
	s64 soc_delta_count;
	int soc_origin_uah;
};

static int mt6329_fgadc_init(struct mt6329_fgadc *adc)
{
	unsigned int value;
	int ret;

	ret = regmap_read(adc->regmap, MT6329_FGADC_CONTROL, &value);
	if (ret)
		return ret;
	if (value)
		return -EBUSY;
	ret = regmap_read_poll_timeout(adc->power, 0xc8, value,
				       (value & (BIT(7) | BIT(4) | BIT(0))) ==
				       (BIT(7) | BIT(0)), 1000, 50000);
	if (ret)
		return ret;
	ret = regmap_clear_bits(adc->regmap, MT6329_FGADC_CLOCK,
				MT6329_FGADC_CLOCK_PDN);
	if (ret)
		return ret;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CLOCK, &value);
	if (ret)
		return ret;
	if (value & MT6329_FGADC_CLOCK_PDN)
		return -EIO;
	ret = regmap_read(adc->regmap, MT6329_FGADC_MODE, &value);
	if (ret)
		return ret;
	if (value != 0x29) {
		ret = regmap_write(adc->regmap, MT6329_FGADC_MODE, 0x28);
		if (ret)
			return ret;
		ret = regmap_write(adc->regmap, MT6329_FGADC_MODE, 0x29);
		if (ret)
			return ret;
	}
	ret = regmap_read(adc->regmap, MT6329_FGADC_MODE, &value);
	if (ret)
		return ret;
	if (value != 0x29)
		return -EIO;
	fsleep(50000);

	return 0;
}

static int mt6329_fgadc_convert(struct mt6329_fgadc *adc, unsigned int raw,
				int *value)
{
	unsigned int magnitude = raw < 0x8000 ? raw : 0xffff - raw;
	u64 scaled_current;

	if (!adc->shunt_uohm || !adc->calibration_percent)
		return -EINVAL;

	scaled_current = div_u64((u64)magnitude * 158122, 100000);
	scaled_current = div_u64(scaled_current * 20000, adc->shunt_uohm);
	scaled_current = div_u64(scaled_current * adc->calibration_percent, 100);
	scaled_current = div_u64(scaled_current, 10);
	if (scaled_current > INT_MAX / 1000)
		return -ERANGE;
	*value = raw < 0x8000 ? (int)scaled_current : -(int)scaled_current;

	return 0;
}

static int mt6329_fgadc_restore(struct mt6329_fgadc *adc)
{
	unsigned int control;
	int ret;

	if (!adc->restore_pending)
		return 0;
	ret = regmap_write(adc->regmap, MT6329_FGADC_CONTROL, 0x08);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(adc->regmap, MT6329_FGADC_CONTROL,
				       control, !(control & MT6329_FGADC_READY),
				       1000, 50000);
	if (ret)
		return ret;
	ret = regmap_write(adc->regmap, MT6329_FGADC_CONTROL, 0);
	if (ret)
		return ret;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CONTROL, &control);
	if (ret)
		return ret;
	if (control)
		return -EIO;
	adc->restore_pending = false;

	return 0;
}

static int mt6329_fgadc_charge(struct mt6329_fgadc *adc, const u8 *car,
			       int *value)
{
	unsigned int code = ((car[3] >> 6) | (car[2] << 2) |
			     (car[1] << 10)) & 0xffff;
	s64 charge;

	if (!adc->shunt_uohm || !adc->calibration_percent)
		return -EINVAL;
	charge = code == 0 || code == 0xffff ? 0 :
		 car[0] & BIT(3) ? (int)code - 0xffff : (int)code;
	charge = div_s64(charge * 35986, 10);
	charge = div_s64(charge + 5, 10000);
	charge = div64_s64(charge * 20000, adc->shunt_uohm);
	charge = div_s64(charge * adc->calibration_percent, 100);
	if (charge > INT_MAX / 1000 || charge < INT_MIN / 1000)
		return -ERANGE;
	*value = charge * 1000;

	return 0;
}

static int mt6329_fgadc_sample(struct mt6329_fgadc *adc, int *value, u8 *car)
{
	unsigned int control, low, high, i;
	int ret, cleanup;

	ret = mt6329_fgadc_restore(adc);
	if (ret)
		return ret;
	ret = regmap_read(adc->power, 0xc8, &control);
	if (ret)
		return ret;
	if ((control & (BIT(7) | BIT(4) | BIT(0))) != (BIT(7) | BIT(0)))
		return -EOPNOTSUPP;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CLOCK, &control);
	if (ret)
		return ret;
	if (control & MT6329_FGADC_CLOCK_PDN)
		return -EOPNOTSUPP;
	ret = regmap_read(adc->regmap, MT6329_FGADC_MODE, &control);
	if (ret)
		return ret;
	if (control != 0x29)
		return -EOPNOTSUPP;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CONTROL, &control);
	if (ret)
		return ret;
	if (control)
		return -EBUSY;

	adc->restore_pending = true;
	ret = regmap_write(adc->regmap, MT6329_FGADC_CONTROL, 0x02);
	if (ret)
		goto restore;
	ret = regmap_read_poll_timeout(adc->regmap, MT6329_FGADC_CONTROL,
				       control, control & MT6329_FGADC_READY,
				       1000, 50000);
	if (ret)
		goto restore;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CURRENT_L, &low);
	if (ret)
		goto restore;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CURRENT_H, &high);
	if (ret || !car)
		goto restore;
	for (i = 0; i < 4; i++) {
		ret = regmap_read(adc->regmap, 0x6b + i, &control);
		if (ret)
			goto restore;
		car[i] = control;
	}
restore:
	cleanup = mt6329_fgadc_restore(adc);
	if (ret || cleanup)
		return ret ?: cleanup;
	*value = high << 8 | low;

	return 0;
}

static int mt6329_fgadc_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct mt6329_fgadc *adc = iio_priv(indio_dev);
	int raw, ret;

	if (mask != IIO_CHAN_INFO_PROCESSED)
		return -EINVAL;
	guard(mutex)(&adc->lock);
	ret = mt6329_fgadc_sample(adc, &raw, NULL);
	if (ret)
		return ret;
	ret = mt6329_fgadc_convert(adc, raw, val);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static void mt6329_fgadc_cleanup(void *data)
{
	struct mt6329_fgadc *adc = data;
	int ret;

	guard(mutex)(&adc->lock);
	ret = mt6329_fgadc_restore(adc);
	if (ret)
		dev_err(regmap_get_device(adc->regmap),
			"Failed to release current latch: %d\n", ret);
}

static const struct iio_chan_spec mt6329_fgadc_channel = {
	.type = IIO_CURRENT,
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
};

static const struct iio_info mt6329_fgadc_info = {
	.read_raw = mt6329_fgadc_read_raw,
};

static int mt6329_fgadc_delta_charge(struct mt6329_fgadc *adc, s64 counts,
				     s64 *charge)
{
	u64 magnitude = counts < 0 ? -(u64)counts : (u64)counts;
	u64 value;

	if (!adc->shunt_uohm || !adc->calibration_percent)
		return -EINVAL;
	value = mul_u64_u64_div_u64(magnitude,
				    35986ULL * 20000 * adc->calibration_percent,
				    10000ULL * adc->shunt_uohm);
	if (value > S64_MAX)
		return -ERANGE;
	*charge = counts < 0 ? -(s64)value : (s64)value;

	return 0;
}

static int mt6329_battery_track(struct mt6329_fgadc *adc, const u8 *car,
				u64 now_ms, int full_uah, int seed, int *charge)
{
	u16 count = (((car[3] >> 6) | (car[2] << 2) | (car[1] << 10)) +
		     !!(car[0] & BIT(3))) & 0xffff;
	s64 quantum, half_wrap, delta, total, value;
	u64 possible;
	int maximum, ret;

	if (full_uah <= 0)
		return -ENODATA;
	if (!adc->soc_valid) {
		if (seed < 0)
			return -EAGAIN;
		if (seed > 100)
			return -EINVAL;
		adc->soc_origin_uah = div_s64((s64)full_uah * seed, 100);
		adc->soc_delta_count = 0;
		adc->soc_count = count;
		adc->soc_time_ms = now_ms;
		adc->soc_valid = true;
		*charge = adc->soc_origin_uah;
		return 0;
	}
	ret = mt6329_fgadc_delta_charge(adc, 2, &quantum);
	if (ret)
		return ret;
	ret = mt6329_fgadc_delta_charge(adc, 32768, &half_wrap);
	if (ret)
		return ret;
	ret = mt6329_fgadc_convert(adc, 0x7fff, &maximum);
	if (ret)
		return ret;
	if (now_ms < adc->soc_time_ms || !quantum || !maximum)
		goto discontinuity;
	possible = mul_u64_u64_div_u64((u64)maximum * 1000,
				       now_ms - adc->soc_time_ms, 3600000);
	if (possible + quantum >= (u64)half_wrap)
		goto discontinuity;
	ret = mt6329_fgadc_delta_charge(adc, (s16)(count - adc->soc_count), &delta);
	if (ret)
		return ret;
	if ((delta < 0 ? -delta : delta) > possible + quantum)
		goto discontinuity;
	if (check_add_overflow(adc->soc_delta_count, (s64)(s16)(count - adc->soc_count), &total))
		goto discontinuity;
	ret = mt6329_fgadc_delta_charge(adc, total, &value);
	if (ret)
		return ret;
	if (check_add_overflow(value, (s64)adc->soc_origin_uah, &value))
		goto discontinuity;
	*charge = clamp_val(value, 0, full_uah);
	adc->soc_delta_count = total;
	adc->soc_count = count;
	adc->soc_time_ms = now_ms;
	if (value <= 0 || value >= full_uah) {
		adc->soc_origin_uah = *charge;
		adc->soc_delta_count = 0;
	}

	return 0;

discontinuity:
	adc->soc_valid = false;
	return -EAGAIN;
}

static int mt6329_battery_capacity(struct power_supply *psy,
				   enum power_supply_property prop, int *value)
{
	struct power_supply_battery_info *info = psy->battery_info;
	struct mt6329_fgadc *adc = power_supply_get_drvdata(psy);
	int raw, current_ma, voltage, temp, resistance, seed, charge, ret;
	s64 ocv, estimate;
	u8 car[4];
	u64 now;

	if (!info || info->charge_full_design_uah <= 0)
		return -ENODATA;
	guard(mutex)(&adc->lock);
	ret = mt6329_fgadc_sample(adc, &raw, car);
	if (ret)
		return ret;
	now = div_u64(ktime_get_boottime_ns(), NSEC_PER_MSEC);
	ret = mt6329_battery_track(adc, car, now, info->charge_full_design_uah, -1, &charge);
	if (ret == -EAGAIN) {
		ret = mt6329_fgadc_convert(adc, raw, &current_ma);
		if (ret)
			return ret;
		ret = iio_read_channel_processed(adc->voltage, &voltage);
		if (ret < 0)
			return ret;
		ret = iio_read_channel_processed(adc->temperature, &temp);
		if (ret < 0)
			return ret;
		if (voltage <= 0 || voltage > INT_MAX / 1000)
			return -ERANGE;
		voltage *= 1000;
		ocv = voltage;
		/* shortcut: FET loss; charger-path model pending */
		for (int i = 0; i < 5; i++) {
			resistance = power_supply_vbat2ri(info, ocv, current_ma > 0);
			if (resistance <= 0)
				return -ENODATA;
			estimate = voltage - div_s64((s64)current_ma *
						   ((s64)resistance + adc->shunt_uohm), 1000);
			if (estimate <= 0 || estimate > INT_MAX)
				return -ERANGE;
			if (estimate == ocv)
				break;
			ocv = estimate;
		}
		seed = power_supply_batinfo_ocv2cap(info, ocv, temp / 1000);
		if (seed < 0)
			return -ENODATA;
		ret = mt6329_battery_track(adc, car, now, info->charge_full_design_uah,
					   seed, &charge);
	}
	if (ret)
		return ret;
	if (prop == POWER_SUPPLY_PROP_CHARGE_NOW)
		*value = charge;
	else
		*value = div_s64((s64)charge * 100 + info->charge_full_design_uah / 2,
				 info->charge_full_design_uah);

	return 0;
}

static int mt6329_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct mt6329_fgadc *adc = power_supply_get_drvdata(psy);
	u8 car[4];
	int raw, current_ma, ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		return mt6329_battery_capacity(psy, prop, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = iio_read_channel_processed(adc->voltage, &raw);
		if (ret < 0)
			return ret;
		if (raw > INT_MAX / 1000 || raw < 0)
			return -ERANGE;
		val->intval = raw * 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = iio_read_channel_processed(adc->temperature, &raw);
		if (ret < 0)
			return ret;
		val->intval = raw / 100;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		scoped_guard(mutex, &adc->lock) {
			ret = mt6329_fgadc_sample(adc, &raw,
						  prop == POWER_SUPPLY_PROP_CHARGE_COUNTER ?
						  car : NULL);
			if (ret)
				return ret;
		}
		if (prop == POWER_SUPPLY_PROP_CHARGE_COUNTER)
			return mt6329_fgadc_charge(adc, car, &val->intval);
		ret = mt6329_fgadc_convert(adc, raw, &current_ma);
		if (ret)
			return ret;
		if (prop == POWER_SUPPLY_PROP_CURRENT_NOW) {
			val->intval = current_ma * 1000;
			return 0;
		}
		if (current_ma) {
			val->intval = current_ma > 0 ? POWER_SUPPLY_STATUS_CHARGING :
						    POWER_SUPPLY_STATUS_DISCHARGING;
			return 0;
		}
		return power_supply_get_property_from_supplier(psy, prop, val);
	default:
		return -EINVAL;
	}
}

static void mt6329_battery_changed(struct power_supply *psy)
{
	struct mt6329_fgadc *adc = power_supply_get_drvdata(psy);

	mod_delayed_work(system_wq, &adc->work, 0);
}

static void mt6329_battery_work(struct work_struct *work)
{
	struct mt6329_fgadc *adc = container_of(to_delayed_work(work),
						struct mt6329_fgadc, work);
	union power_supply_propval status, charge, capacity;
	bool changed;
	int ret;

	ret = mt6329_battery_get_property(adc->battery, POWER_SUPPLY_PROP_STATUS, &status);
	if (ret)
		status.intval = POWER_SUPPLY_STATUS_UNKNOWN;
	changed = adc->status != status.intval;
	adc->status = status.intval;
	ret = mt6329_battery_get_property(adc->battery, POWER_SUPPLY_PROP_CHARGE_COUNTER, &charge);
	if (!ret) {
		changed |= adc->charge != charge.intval;
		adc->charge = charge.intval;
	}
	ret = mt6329_battery_get_property(adc->battery, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (!ret) {
		changed |= adc->capacity != capacity.intval;
		adc->capacity = capacity.intval;
	}
	if (changed)
		power_supply_changed(adc->battery);
	schedule_delayed_work(&adc->work, msecs_to_jiffies(5000));
}

static void mt6329_battery_stop(void *data)
{
	struct mt6329_fgadc *adc = data;

	disable_delayed_work_sync(&adc->work);
}

static const enum power_supply_property mt6329_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc mt6329_battery_desc = {
	.name = "mt6329-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = mt6329_battery_props,
	.num_properties = ARRAY_SIZE(mt6329_battery_props),
	.get_property = mt6329_battery_get_property,
	.external_power_changed = mt6329_battery_changed,
};

static int mt6329_fgadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6329_fgadc *adc;
	struct iio_dev *indio_dev;
	struct power_supply_config config = {};
	int ret, maximum;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->power = dev_get_regmap(dev->parent, "power");
	adc->regmap = dev_get_regmap(dev->parent, "bank1");
	if (!adc->power || !adc->regmap)
		return -ENODEV;
	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				       &adc->shunt_uohm);
	if (ret)
		return ret;
	ret = device_property_read_u32(dev, "mediatek,current-calibration-percent",
				       &adc->calibration_percent);
	if (ret)
		return ret;
	ret = mt6329_fgadc_convert(adc, 0x7fff, &maximum);
	if (ret)
		return ret;
	ret = devm_regulator_get_enable_read_voltage(dev, "va2");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to enable VA2\n");
	if (ret != 2500000)
		return dev_err_probe(dev, -EOPNOTSUPP, "Unsupported VA2 voltage\n");
	mutex_init(&adc->lock);
	ret = devm_add_action_or_reset(dev, mt6329_fgadc_cleanup, adc);
	if (ret)
		return ret;
	ret = mt6329_fgadc_init(adc);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to start converter\n");

	indio_dev->name = "mt6329-fgadc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mt6329_fgadc_info;
	indio_dev->channels = &mt6329_fgadc_channel;
	indio_dev->num_channels = 1;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, adc);
	if (!device_property_present(dev, "monitored-battery"))
		return 0;
	adc->voltage = devm_iio_channel_get(dev, "voltage");
	if (IS_ERR(adc->voltage))
		return dev_err_probe(dev, PTR_ERR(adc->voltage), "Failed to get voltage channel\n");
	adc->temperature = devm_iio_channel_get(dev, "temperature");
	if (IS_ERR(adc->temperature))
		return dev_err_probe(dev, PTR_ERR(adc->temperature),
				     "Failed to get temperature channel\n");
	config.fwnode = dev_fwnode(dev);
	config.drv_data = adc;
	INIT_DELAYED_WORK(&adc->work, mt6329_battery_work);
	disable_delayed_work_sync(&adc->work);
	adc->battery = devm_power_supply_register(dev, &mt6329_battery_desc, &config);
	if (IS_ERR(adc->battery))
		return dev_err_probe(dev, PTR_ERR(adc->battery), "Failed to register battery\n");
	ret = devm_add_action_or_reset(dev, mt6329_battery_stop, adc);
	if (ret)
		return ret;
	enable_delayed_work(&adc->work);
	schedule_delayed_work(&adc->work, 0);

	return 0;
}

static int mt6329_battery_suspend(struct device *dev)
{
	struct mt6329_fgadc *adc = dev_get_drvdata(dev);

	if (adc->battery)
		disable_delayed_work_sync(&adc->work);
	return 0;
}

static int mt6329_battery_resume(struct device *dev)
{
	struct mt6329_fgadc *adc = dev_get_drvdata(dev);

	if (adc->battery) {
		enable_delayed_work(&adc->work);
		schedule_delayed_work(&adc->work, 0);
	}
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6329_battery_pm_ops,
				mt6329_battery_suspend, mt6329_battery_resume);

static const struct of_device_id mt6329_fgadc_of_match[] = {
	{ .compatible = "mediatek,mt6329-fgadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6329_fgadc_of_match);

static struct platform_driver mt6329_fgadc_driver = {
	.probe = mt6329_fgadc_probe,
	.driver = {
		.name = "mt6329-fgadc",
		.of_match_table = mt6329_fgadc_of_match,
		.pm = pm_sleep_ptr(&mt6329_battery_pm_ops),
	},
};
module_platform_driver(mt6329_fgadc_driver);

MODULE_DESCRIPTION("MediaTek MT6329 fuel gauge ADC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
