// SPDX-License-Identifier: GPL-2.0-only
#include <linux/i2c.h>
#include <linux/devm-helpers.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define SM5701_INT1		0x00
#define SM5701_INT3		0x02
#define SM5701_STATUS1		0x06
#define SM5701_STATUS2		0x07
#define SM5701_VBUSOK		BIT(2)
#define SM5701_VBUSUVLO		BIT(3)
#define SM5701_VBUSOVP		BIT(4)
#define SM5701_NOBAT		BIT(7)
#define SM5701_TOPOFF		BIT(0)
#define SM5701_DONE		BIT(1)
#define SM5701_CHGON		BIT(3)

struct sm5701 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll;
	bool online;
	bool present;
	int status;
	int health;
	int irq;
	struct led_classdev torch;
	/* Torch transitions. */
	struct mutex torch_lock;
	unsigned int saved_mode;
	unsigned int saved_boost;
	unsigned int saved_current;
	unsigned int saved_timer;
	bool torch_active;
	bool torch_fault;
};

static bool sm5701_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= SM5701_INT3 ||
	       (reg >= SM5701_STATUS1 && reg <= 0x15);
}

static bool sm5701_precious_reg(struct device *dev, unsigned int reg)
{
	return reg <= SM5701_INT3;
}

static bool sm5701_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg == 0x09 || reg == 0x0f || reg == 0x12 || reg == 0x14;
}

static const struct regmap_config sm5701_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x15,
	.cache_type = REGCACHE_NONE,
	.readable_reg = sm5701_readable_reg,
	.writeable_reg = sm5701_writeable_reg,
	.precious_reg = sm5701_precious_reg,
};

static int sm5701_torch_off(struct sm5701 *data)
{
	int ret;

	if (!data->torch_active)
		return 0;
	ret = regmap_update_bits(data->regmap, 0x0f, 3, 0);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x09, 7, data->saved_mode);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x14, 0x0f, data->saved_boost);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x12, 0x1f, data->saved_current);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x0f, BIT(4), data->saved_timer);
	if (ret)
		goto fault;
	data->torch_active = false;
	return 0;
fault:
	data->torch_fault = true;
	return ret;
}

static int sm5701_torch_set(struct sm5701 *data, unsigned int brightness)
{
	unsigned int mode, boost, current_code, fled, revision, status;
	int ret;

	if (!brightness)
		return sm5701_torch_off(data);
	if (data->torch_fault)
		return -EIO;
	if (data->torch_active) {
		ret = regmap_update_bits(data->regmap, 0x12, 0x1f, brightness - 1);
		if (ret)
			goto fault;
		return 0;
	}
	ret = regmap_read(data->regmap, 0x09, &mode);
	if (ret)
		return ret;
	if ((mode & 7) != 0 && (mode & 7) != 4)
		return -EBUSY;
	ret = regmap_read(data->regmap, 0x14, &boost);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, 0x12, &current_code);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, 0x0f, &fled);
	if (ret)
		return ret;
	if ((fled & 3) == 1 || (fled & 3) == 2)
		return -EBUSY;
	ret = regmap_read(data->regmap, 0x15, &revision);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, SM5701_STATUS1, &status);
	if (ret)
		return ret;
	data->saved_mode = mode & 7;
	data->saved_boost = boost & 0x0f;
	data->saved_current = current_code & 0x1f;
	data->saved_timer = fled & BIT(4);
	data->torch_active = true;
	ret = regmap_update_bits(data->regmap, 0x0f, BIT(4) | 3, 0);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x12, 0x1f, brightness - 1);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x14, 0x0f, 5);
	if (ret)
		goto fault;
	mode = (mode & 4) && (status & SM5701_VBUSOK) && revision != 4 ? 5 : 1;
	ret = regmap_update_bits(data->regmap, 0x09, 7, mode);
	if (ret)
		goto fault;
	ret = regmap_update_bits(data->regmap, 0x0f, 3, 1);
	if (!ret)
		return 0;
fault:
	data->torch_fault = true;
	sm5701_torch_off(data);
	return ret;
}

static int sm5701_torch_brightness(struct led_classdev *led, enum led_brightness brightness)
{
	struct sm5701 *data = container_of(led, struct sm5701, torch);
	int ret;

	mutex_lock(&data->torch_lock);
	ret = sm5701_torch_set(data, brightness);
	mutex_unlock(&data->torch_lock);
	return ret;
}

static void sm5701_torch_stop(void *arg)
{
	struct sm5701 *data = arg;
	int ret;

	mutex_lock(&data->torch_lock);
	data->torch_fault = true;
	ret = sm5701_torch_off(data);
	mutex_unlock(&data->torch_lock);
	if (ret)
		dev_err(data->dev, "Failed to stop torch\n");
}

static int sm5701_register_torch(struct sm5701 *data)
{
	struct led_init_data init = {};
	struct device_node *node;
	u32 current_limit;
	int ret;

	node = of_get_child_by_name(data->dev->of_node, "led");
	if (!node)
		return 0;
	ret = of_property_read_u32(node, "led-max-microamp", &current_limit);
	if (ret || current_limit < 10000 || current_limit > 320000 || current_limit % 10000) {
		of_node_put(node);
		return -EINVAL;
	}
	data->torch.max_brightness = current_limit / 10000;
	data->torch.brightness_set_blocking = sm5701_torch_brightness;
	data->torch.flags = LED_CORE_SUSPENDRESUME;
	init.fwnode = of_fwnode_handle(node);
	ret = devm_add_action_or_reset(data->dev, sm5701_torch_stop, data);
	if (!ret)
		ret = devm_led_classdev_register_ext(data->dev, &data->torch, &init);
	of_node_put(node);
	return ret;
}

static int sm5701_input_health(unsigned int status)
{
	if (status & SM5701_VBUSOVP)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	if (!(status & SM5701_VBUSOK))
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	if (status & SM5701_VBUSUVLO)
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int sm5701_read_state(struct sm5701 *data, bool *online, bool *present,
			     int *state, int *health)
{
	unsigned int status1, status2;
	int ret;

	ret = regmap_read(data->regmap, SM5701_STATUS1, &status1);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, SM5701_STATUS2, &status2);
	if (ret)
		return ret;

	*online = !!(status1 & SM5701_VBUSOK);
	*present = !(status1 & SM5701_NOBAT);
	*health = sm5701_input_health(status1);
	*state = POWER_SUPPLY_STATUS_UNKNOWN;
	if (*present) {
		if (!*online)
			*state = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!(status2 & (SM5701_TOPOFF | SM5701_DONE)))
			*state = status2 & SM5701_CHGON ?
				POWER_SUPPLY_STATUS_CHARGING :
				POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return 0;
}

static int sm5701_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sm5701 *data = power_supply_get_drvdata(psy);
	unsigned int status;
	bool online, present;
	int state, health;
	int ret;

	if (prop == POWER_SUPPLY_PROP_STATUS) {
		ret = sm5701_read_state(data, &online, &present, &state, &health);
		if (!ret)
			val->intval = state;
		return ret;
	}
	if (prop != POWER_SUPPLY_PROP_ONLINE && prop != POWER_SUPPLY_PROP_PRESENT &&
	    prop != POWER_SUPPLY_PROP_HEALTH)
		return -EINVAL;

	ret = regmap_read(data->regmap, SM5701_STATUS1, &status);
	if (ret)
		return ret;

	if (prop == POWER_SUPPLY_PROP_HEALTH)
		val->intval = sm5701_input_health(status);
	else
		val->intval = prop == POWER_SUPPLY_PROP_ONLINE ?
			!!(status & SM5701_VBUSOK) : !(status & SM5701_NOBAT);

	return 0;
}

static const enum power_supply_property sm5701_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
};

static const struct power_supply_desc sm5701_desc = {
	.name = "sm5701-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5701_properties,
	.num_properties = ARRAY_SIZE(sm5701_properties),
	.get_property = sm5701_get_property,
};

static int sm5701_ack_irq(struct sm5701 *data)
{
	unsigned int reg, pending;
	int ret;

	for (reg = SM5701_INT1; reg <= SM5701_INT3; reg++) {
		ret = regmap_read(data->regmap, reg, &pending);
		if (ret)
			return ret;
	}

	return 0;
}

static void sm5701_poll(struct work_struct *work)
{
	struct sm5701 *data = container_of(to_delayed_work(work), struct sm5701, poll);
	bool online, present;
	int state, health;

	if (data->irq)
		sm5701_ack_irq(data);
	if (!sm5701_read_state(data, &online, &present, &state, &health)) {
		if (online != data->online || present != data->present ||
		    state != data->status || health != data->health) {
			data->online = online;
			data->present = present;
			data->status = state;
			data->health = health;
			power_supply_changed(data->psy);
		}
	}
	schedule_delayed_work(&data->poll, msecs_to_jiffies(2000));
}

static irqreturn_t sm5701_irq(int irq, void *arg)
{
	struct sm5701 *data = arg;
	int ret;

	ret = sm5701_ack_irq(data);
	if (ret)
		dev_err_ratelimited(data->dev, "Failed to acknowledge interrupts: %d\n", ret);
	mod_delayed_work(system_percpu_wq, &data->poll, 0);

	return IRQ_HANDLED;
}

static int sm5701_suspend(struct device *dev)
{
	struct sm5701 *data = dev_get_drvdata(dev);

	if (data->irq)
		disable_irq(data->irq);
	cancel_delayed_work_sync(&data->poll);
	return 0;
}

static int sm5701_resume(struct device *dev)
{
	struct sm5701 *data = dev_get_drvdata(dev);

	if (data->irq) {
		sm5701_irq(data->irq, data);
		enable_irq(data->irq);
	} else {
		schedule_delayed_work(&data->poll, 0);
	}
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sm5701_pm_ops, sm5701_suspend, sm5701_resume);

static int sm5701_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct device *dev = &client->dev;
	struct power_supply *psy;
	struct regmap *regmap;
	struct sm5701 *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &sm5701_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to create regmap\n");

	data->dev = dev;
	data->regmap = regmap;
	mutex_init(&data->torch_lock);
	ret = sm5701_read_state(data, &data->online, &data->present,
				&data->status, &data->health);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read charger status\n");

	i2c_set_clientdata(client, data);
	cfg.drv_data = data;
	cfg.fwnode = dev_fwnode(dev);
	psy = devm_power_supply_register(dev, &sm5701_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "Failed to register supply\n");
	data->psy = psy;
	ret = devm_delayed_work_autocancel(dev, &data->poll, sm5701_poll);
	if (ret)
		return ret;
	if (client->irq) {
		ret = sm5701_ack_irq(data);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to clear pending interrupts\n");
		ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5701_irq,
						IRQF_ONESHOT, dev_name(dev), data);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to request interrupt\n");
		data->irq = client->irq;
	}
	ret = sm5701_register_torch(data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register torch\n");
	schedule_delayed_work(&data->poll, msecs_to_jiffies(2000));

	return 0;
}

static const struct of_device_id sm5701_of_match[] = {
	{ .compatible = "siliconmitus,sm5701" },
	{}
};
MODULE_DEVICE_TABLE(of, sm5701_of_match);

static void sm5701_shutdown(struct i2c_client *client)
{
	sm5701_torch_stop(i2c_get_clientdata(client));
}

static struct i2c_driver sm5701_driver = {
	.driver = {
		.name = "sm5701",
		.of_match_table = sm5701_of_match,
		.pm = pm_sleep_ptr(&sm5701_pm_ops),
	},
	.probe = sm5701_probe,
	.shutdown = sm5701_shutdown,
};
module_i2c_driver(sm5701_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5701 input supply monitor");
MODULE_AUTHOR("minorum <mit.mis@gmail.com>");
MODULE_LICENSE("GPL");
