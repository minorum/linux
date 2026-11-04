// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/limits.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define NCP1851_STATUS		0x00
#define NCP1851_CTRL1		0x01
#define NCP1851_CTRL2		0x02
#define NCP1851_CH2_INT		0x05
#define NCP1851_VIN		0x07
#define NCP1851_VBAT		0x08
#define NCP1851_TEMP		0x09
#define NCP1851_VREG		0x0e
#define NCP1851_IBAT		0x0f
#define NCP1851_MISC		0x10
#define NCP1851_MAX_REG		0x12
#define NCP1851_CHGEN		BIT(6)
#define NCP1851_RESTART		BIT(1)
#define NCP1851_TIMER_FAULTS	0x0e
#define NCP1851_INPUT_READY	0x0c
#define NCP1851_POLL_MS		10000

static const struct linear_range ncp1851_vreg =
	LINEAR_RANGE(3300000, 0, 48, 25000);
static const struct linear_range ncp1851_ichg =
	LINEAR_RANGE(400000, 0, 12, 100000);
static const struct linear_range ncp1851_ieoc =
	LINEAR_RANGE(100000, 0, 7, 25000);
static const struct linear_range ncp1851_iweak =
	LINEAR_RANGE(0, 0, 3, 100000);
static const struct linear_range ncp1851_vsys =
	LINEAR_RANGE(3100000, 0, 5, 100000);
static const int ncp1851_input_limits[] = { 100000, 500000, 900000, 1500000 };

struct ncp1851 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct iio_channel *voltage, *temperature;
	struct mutex lock; /* Registers and policy */
	struct delayed_work work;
	u8 regs[NCP1851_MAX_REG + 1];
	unsigned int timer_fault;
	unsigned int vreg, ichg, ieoc, iweak, vsys;
	int temp_mc, voltage_uv, temp_min_mc, temp_max_mc, overvoltage_uv;
	int input_max, input_budget;
	int health, status, error;
	bool power_path, inhibit, stopped, online, cycle_pending, fault_latched;
};

static int ncp1851_encode(const struct linear_range *range, unsigned int value,
			  unsigned int *code)
{
	bool found;

	if (value < range->min || value > linear_range_get_max_value(range))
		return -EINVAL;

	return linear_range_get_selector_low(range, value, code, &found);
}

static int ncp1851_input_code(int ua)
{
	int i;

	for (i = ARRAY_SIZE(ncp1851_input_limits) - 1; i >= 0; i--)
		if (ua >= ncp1851_input_limits[i])
			return i;
	return -EINVAL;
}

static int ncp1851_ctrl1(struct ncp1851 *chip, bool enabled, bool restart)
{
	unsigned int old, value;
	int ret;

	ret = regmap_read(chip->regmap, NCP1851_CTRL1, &old);
	if (ret)
		return ret;
	value = (old & 0x0c) | BIT(0);
	if (enabled)
		value |= NCP1851_CHGEN;
	if (restart)
		value |= NCP1851_RESTART;
	if (value == old && !restart)
		return 0;
	return regmap_write(chip->regmap, NCP1851_CTRL1, value);
}

static int ncp1851_kick(struct ncp1851 *chip)
{
	return regmap_write_bits(chip->regmap, NCP1851_CTRL2, 0xc0, 0);
}

static int ncp1851_inhibit(struct ncp1851 *chip)
{
	unsigned int value;
	int ret;

	ret = ncp1851_ctrl1(chip, false, false);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap, NCP1851_CTRL1, &value);
	if (ret)
		return ret;
	if (value & NCP1851_CHGEN)
		return -EIO;
	return regmap_update_bits(chip->regmap, NCP1851_CTRL2, BIT(7), BIT(7));
}

static int ncp1851_read_regs(struct ncp1851 *chip)
{
	static const u8 addresses[] = { 0, 1, 2, 5, 7, 8, 9, 14, 15, 16 };
	u8 regs[NCP1851_MAX_REG + 1] = {};
	unsigned int value;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(addresses); i++) {
		ret = regmap_read(chip->regmap, addresses[i], &value);
		if (ret)
			return ret;
		regs[addresses[i]] = value;
		if (addresses[i] == NCP1851_CH2_INT)
			chip->timer_fault |= value & NCP1851_TIMER_FAULTS;
	}
	memcpy(chip->regs, regs, sizeof(regs));
	if ((regs[NCP1851_STATUS] >> 4) == 11 && (regs[NCP1851_CTRL1] & NCP1851_CHGEN))
		chip->fault_latched = true;
	return 0;
}

static int ncp1851_status(unsigned int state, bool online, bool enabled)
{
	if (!online)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	if (!enabled)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	switch (state) {
	case 2:
	case 3:
	case 4:
	case 5:
	case 9:
	case 10:
		return POWER_SUPPLY_STATUS_CHARGING;
	case 6:
		return POWER_SUPPLY_STATUS_FULL;
	default:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static int ncp1851_health(struct ncp1851 *chip)
{
	if (chip->temp_mc <= chip->temp_min_mc)
		return POWER_SUPPLY_HEALTH_COLD;
	if (chip->temp_mc >= chip->temp_max_mc || (chip->regs[NCP1851_TEMP] & BIT(3)))
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if ((chip->regs[NCP1851_VIN] & 0xa0) ||
	    (chip->regs[NCP1851_VBAT] & BIT(6)) ||
	    chip->voltage_uv >= chip->overvoltage_uv)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	if (chip->timer_fault & BIT(3))
		return POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
	if (chip->timer_fault & BIT(1))
		return POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	if (chip->fault_latched || (chip->timer_fault & BIT(2)) ||
	    ((chip->regs[NCP1851_STATUS] >> 4) == 11 &&
	     (chip->regs[NCP1851_CTRL1] & NCP1851_CHGEN)))
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int ncp1851_configure(struct ncp1851 *chip, int input)
{
	int ret;

	ret = ncp1851_ctrl1(chip, false, false);
	if (ret)
		return ret;
	ret = regmap_update_bits(chip->regmap, NCP1851_MISC, 0x7f,
				 (chip->iweak << 5) | (chip->vsys << 2) | input);
	if (ret)
		return ret;
	ret = regmap_update_bits(chip->regmap, NCP1851_VREG, 0x3f, chip->vreg);
	if (ret)
		return ret;
	ret = regmap_update_bits(chip->regmap, NCP1851_IBAT, 0x7f,
				 (chip->ieoc << 4) | chip->ichg);
	if (ret)
		return ret;
	return regmap_update_bits(chip->regmap, NCP1851_CTRL2, 0xef,
				 (chip->power_path ? BIT(5) : 0) | BIT(1));
}

static bool ncp1851_configured(struct ncp1851 *chip, int input)
{
	return (chip->regs[NCP1851_VREG] & 0x3f) == chip->vreg &&
	       (chip->regs[NCP1851_IBAT] & 0x7f) == ((chip->ieoc << 4) | chip->ichg) &&
	       (chip->regs[NCP1851_MISC] & 0x7f) ==
		((chip->iweak << 5) | (chip->vsys << 2) | input) &&
	       (chip->regs[NCP1851_CTRL2] & 0xef) ==
		((chip->power_path ? BIT(5) : 0) | BIT(1)) &&
	       !(chip->regs[NCP1851_CTRL1] & 0xb0);
}

static int ncp1851_verify_config(struct ncp1851 *chip, int input, bool enabled)
{
	int ret;

	ret = ncp1851_read_regs(chip);
	if (ret)
		return ret;
	if (!ncp1851_configured(chip, input) ||
	    !!(chip->regs[NCP1851_CTRL1] & NCP1851_CHGEN) != enabled)
		return -EIO;
	return 0;
}

static int ncp1851_update(struct ncp1851 *chip)
{
	int ret, mv, input, health;
	bool online, restart;

	ret = ncp1851_read_regs(chip);
	if (ret)
		goto fail;
	online = (chip->regs[NCP1851_VIN] & NCP1851_INPUT_READY) == NCP1851_INPUT_READY &&
		 !(chip->regs[NCP1851_CTRL1] & BIT(5));
	if (!(chip->regs[NCP1851_VIN] & BIT(3))) {
		chip->timer_fault = 0;
		chip->fault_latched = false;
		chip->cycle_pending = true;
		chip->input_budget = min(chip->input_budget, 100000);
	}
	chip->online = online;
	input = ncp1851_input_code(min(chip->input_budget, chip->input_max));
	if (!online || chip->inhibit || input < 0 ||
	    chip->health != POWER_SUPPLY_HEALTH_GOOD ||
	    ncp1851_health(chip) != POWER_SUPPLY_HEALTH_GOOD ||
	    !ncp1851_configured(chip, input)) {
		ret = ncp1851_inhibit(chip);
		if (ret)
			goto fail;
	}
	ret = iio_read_channel_processed(chip->voltage, &mv);
	if (ret < 0)
		goto fail;
	if (mv <= 0 || mv > INT_MAX / 1000) {
		ret = -ERANGE;
		goto fail;
	}
	chip->voltage_uv = mv * 1000;
	ret = iio_read_channel_processed(chip->temperature, &chip->temp_mc);
	if (ret < 0)
		goto fail;
	health = ncp1851_health(chip);
	chip->health = health;
	if (!online || chip->inhibit || input < 0 || health != POWER_SUPPLY_HEALTH_GOOD) {
		ret = ncp1851_inhibit(chip);
		if (ret)
			goto fail;
		chip->status = ncp1851_status(0, online, false);
		chip->error = 0;
		return 0;
	}
	if (!ncp1851_configured(chip, input)) {
		ret = ncp1851_configure(chip, input);
		if (ret)
			goto fail;
		ret = ncp1851_verify_config(chip, input, false);
		if (ret)
			goto fail;
		if (ncp1851_health(chip) != POWER_SUPPLY_HEALTH_GOOD)
			goto check_health;
	}
	restart = chip->cycle_pending;
	chip->cycle_pending = false;
	ret = ncp1851_ctrl1(chip, true, restart);
	if (ret)
		goto fail;
	ret = ncp1851_kick(chip);
	if (ret)
		goto fail;
	ret = ncp1851_verify_config(chip, input, true);
	if (ret)
		goto fail;
check_health:
	chip->health = ncp1851_health(chip);
	if (chip->health != POWER_SUPPLY_HEALTH_GOOD) {
		ret = ncp1851_inhibit(chip);
		if (ret)
			goto fail;
	}
	chip->status = ncp1851_status(chip->regs[NCP1851_STATUS] >> 4, online,
				      chip->health == POWER_SUPPLY_HEALTH_GOOD);
	chip->error = 0;
	return 0;
fail:
	chip->error = ret;
	chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	if (ncp1851_inhibit(chip))
		dev_err_ratelimited(chip->dev, "Cannot inhibit charging\n");
	return ret;
}

static void ncp1851_work(struct work_struct *work)
{
	struct ncp1851 *chip = container_of(to_delayed_work(work), struct ncp1851, work);

	mutex_lock(&chip->lock);
	if (!chip->stopped) {
		ncp1851_update(chip);
		schedule_delayed_work(&chip->work, msecs_to_jiffies(NCP1851_POLL_MS));
	}
	mutex_unlock(&chip->lock);
	power_supply_changed(chip->psy);
}

static enum power_supply_property ncp1851_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int ncp1851_get_property(struct power_supply *psy, enum power_supply_property prop,
				union power_supply_propval *val)
{
	struct ncp1851 *chip = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chip->lock);
	if (chip->error && prop != POWER_SUPPLY_PROP_STATUS &&
	    prop != POWER_SUPPLY_PROP_HEALTH) {
		ret = chip->error;
		goto out;
	}
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if ((chip->regs[NCP1851_CTRL2] & 6) != 2) {
			ret = -ENODATA;
			break;
		}
		val->intval = ncp1851_input_limits[chip->regs[NCP1851_MISC] & 3];
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = ncp1851_ichg.min + chip->ichg * ncp1851_ichg.step;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = ncp1851_vreg.min + chip->vreg * ncp1851_vreg.step;
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		val->intval = chip->iweak * ncp1851_iweak.step;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = ncp1851_ieoc.min + chip->ieoc * ncp1851_ieoc.step;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chip->inhibit ? POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->voltage_uv;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->temp_mc / 100;
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	mutex_unlock(&chip->lock);
	return ret;
}

static int ncp1851_set_property(struct power_supply *psy, enum power_supply_property prop,
				const union power_supply_propval *val)
{
	struct ncp1851 *chip = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&chip->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0) {
			ret = -EINVAL;
			goto out;
		}
		chip->input_budget = min(val->intval, chip->input_max);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			ret = -EINVAL;
			goto out;
		}
		chip->inhibit = val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}
	ret = chip->stopped ? 0 : ncp1851_update(chip);
out:
	mutex_unlock(&chip->lock);
	if (!ret)
		power_supply_changed(psy);
	return ret;
}

static int ncp1851_writeable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static const struct power_supply_desc ncp1851_desc = {
	.name = "ncp1851-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ncp1851_properties,
	.num_properties = ARRAY_SIZE(ncp1851_properties),
	.get_property = ncp1851_get_property,
	.set_property = ncp1851_set_property,
	.property_is_writeable = ncp1851_writeable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static bool ncp1851_precious(struct device *dev, unsigned int reg)
{
	return reg >= 3 && reg <= 6;
}

static const struct regmap_config ncp1851_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = NCP1851_MAX_REG,
	.precious_reg = ncp1851_precious,
};

static void ncp1851_stop(void *data)
{
	struct ncp1851 *chip = data;
	int ret;

	mutex_lock(&chip->lock);
	chip->stopped = true;
	mutex_unlock(&chip->lock);
	cancel_delayed_work_sync(&chip->work);
	mutex_lock(&chip->lock);
	ret = ncp1851_inhibit(chip);
	chip->error = ret;
	chip->status = ret ? POWER_SUPPLY_STATUS_UNKNOWN : POWER_SUPPLY_STATUS_NOT_CHARGING;
	mutex_unlock(&chip->lock);
	if (ret)
		dev_err(chip->dev, "Cannot stop charger: %d\n", ret);
}

static int ncp1851_battery_info(struct ncp1851 *chip)
{
	struct power_supply_battery_info *info;
	int ret;

	ret = power_supply_get_battery_info(chip->psy, &info);
	if (ret)
		return ret;
	ret = -EINVAL;
	if (info->constant_charge_current_max_ua <= 0 ||
	    info->constant_charge_voltage_max_uv <= 0 || info->precharge_current_ua != 100000 ||
	    info->charge_term_current_ua <= 0 ||
	    info->overvoltage_limit_uv <= info->constant_charge_voltage_max_uv ||
	    info->temp_alert_min <= INT_MIN / 1000 ||
	    info->temp_alert_max >= INT_MAX / 1000 ||
	    info->temp_alert_min >= info->temp_alert_max)
		goto out;
	ret = ncp1851_encode(&ncp1851_vreg, info->constant_charge_voltage_max_uv, &chip->vreg);
	if (ret)
		goto out;
	ret = ncp1851_encode(&ncp1851_ichg, info->constant_charge_current_max_ua, &chip->ichg);
	if (ret)
		goto out;
	ret = ncp1851_encode(&ncp1851_ieoc, info->charge_term_current_ua, &chip->ieoc);
	if (ret)
		goto out;
	ret = ncp1851_encode(&ncp1851_iweak, info->precharge_current_ua, &chip->iweak);
	if (ret)
		goto out;
	if (!chip->iweak) {
		ret = -EINVAL;
		goto out;
	}
	if (chip->power_path &&
	    ncp1851_vsys.min + chip->vsys * ncp1851_vsys.step >=
	    ncp1851_vreg.min + chip->vreg * ncp1851_vreg.step) {
		ret = -EINVAL;
		goto out;
	}
	chip->temp_min_mc = info->temp_alert_min * 1000;
	chip->temp_max_mc = info->temp_alert_max * 1000;
	chip->overvoltage_uv = info->overvoltage_limit_uv;
out:
	power_supply_put_battery_info(chip->psy, info);
	return ret;
}

static int ncp1851_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config config = {};
	struct ncp1851 *chip;
	u32 value;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;
	mutex_init(&chip->lock);
	INIT_DELAYED_WORK(&chip->work, ncp1851_work);
	i2c_set_clientdata(client, chip);
	chip->regmap = devm_regmap_init_i2c(client, &ncp1851_regmap);
	if (IS_ERR(chip->regmap))
		return PTR_ERR(chip->regmap);
	chip->voltage = devm_iio_channel_get(dev, "voltage");
	if (IS_ERR(chip->voltage))
		return dev_err_probe(dev, PTR_ERR(chip->voltage), "No battery voltage\n");
	chip->temperature = devm_iio_channel_get(dev, "temperature");
	if (IS_ERR(chip->temperature))
		return dev_err_probe(dev, PTR_ERR(chip->temperature), "No battery temperature\n");
	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms", &value);
	if (ret || value != 68000)
		return dev_err_probe(dev, -EINVAL, "Unsupported sense resistor\n");
	ret = device_property_read_u32(dev, "input-current-limit-microamp", &value);
	if (ret || value > 1500000 || ncp1851_input_code(value) < 0 ||
	    ncp1851_input_limits[ncp1851_input_code(value)] != value)
		return dev_err_probe(dev, -EINVAL, "Invalid input current limit\n");
	chip->input_max = value;
	chip->input_budget = 100000;
	chip->power_path = device_property_read_bool(dev, "onnn,power-path");
	if (chip->power_path) {
		ret = device_property_read_u32(dev, "onnn,system-voltage-microvolt", &value);
		if (ret)
			return ret;
		ret = ncp1851_encode(&ncp1851_vsys, value, &chip->vsys);
		if (ret)
			return ret;
	}
	chip->stopped = true;
	chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	chip->error = -EAGAIN;
	config.drv_data = chip;
	config.fwnode = dev_fwnode(dev);
	chip->psy = devm_power_supply_register(dev, &ncp1851_desc, &config);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy), "Cannot register charger\n");
	ret = ncp1851_battery_info(chip);
	if (ret)
		return dev_err_probe(dev, ret, "Unsupported battery limits\n");
	ret = devm_add_action_or_reset(dev, ncp1851_stop, chip);
	if (ret)
		return ret;
	mutex_lock(&chip->lock);
	chip->stopped = false;
	mutex_unlock(&chip->lock);
	schedule_delayed_work(&chip->work, 0);
	return 0;
}

static int ncp1851_suspend(struct device *dev)
{
	struct ncp1851 *chip = dev_get_drvdata(dev);
	int ret;

	ncp1851_stop(chip);
	mutex_lock(&chip->lock);
	ret = chip->error;
	if (ret) {
		chip->stopped = false;
		schedule_delayed_work(&chip->work, 0);
	}
	mutex_unlock(&chip->lock);
	return ret;
}

static int ncp1851_resume(struct device *dev)
{
	struct ncp1851 *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	chip->stopped = false;
	mutex_unlock(&chip->lock);
	schedule_delayed_work(&chip->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ncp1851_pm, ncp1851_suspend, ncp1851_resume);

static void ncp1851_shutdown(struct i2c_client *client)
{
	ncp1851_stop(i2c_get_clientdata(client));
}

static const struct of_device_id ncp1851_of_match[] = {
	{ .compatible = "onnn,ncp1851" },
	{ .compatible = "onnn,ncp1851a" },
	{}
};
MODULE_DEVICE_TABLE(of, ncp1851_of_match);

static struct i2c_driver ncp1851_driver = {
	.driver = {
		.name = "ncp1851-charger",
		.of_match_table = ncp1851_of_match,
		.pm = pm_sleep_ptr(&ncp1851_pm),
	},
	.probe = ncp1851_probe,
	.shutdown = ncp1851_shutdown,
};
module_i2c_driver(ncp1851_driver);

MODULE_DESCRIPTION("ON Semiconductor NCP1851 battery charger");
MODULE_LICENSE("GPL");
