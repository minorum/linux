// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

struct mt6577_usb_phy {
	void __iomem *base;
	struct clk *upll;
	struct clk *bus;
	struct regulator *vusb;
	struct regmap *pmic;
	struct phy *phy;
	struct power_supply *psy;
	struct delayed_work work;
	spinlock_t state_lock; /* Source and USB grants */
	unsigned int generation;
	int grant, source_error;
	enum power_supply_usb_type type;
	bool powered, suspended, online, detected, data_seen;
	bool failed, wait_detach;
};

static void mt6577_usb_update(struct mt6577_usb_phy *usb, unsigned int reg,
			      u8 mask, u8 value)
{
	writeb((readb(usb->base + reg) & ~mask) | value, usb->base + reg);
}

static int mt6577_bc11_update(struct mt6577_usb_phy *usb, unsigned int reg,
			      unsigned int mask, unsigned int value)
{
	unsigned int readback;
	int ret;

	ret = regmap_update_bits(usb->pmic, reg, mask, value);
	if (ret)
		return ret;
	ret = regmap_read(usb->pmic, reg, &readback);
	if (ret)
		return ret;
	return (readback & mask) == value ? 0 : -EIO;
}

static int mt6577_bc11_cleanup(struct mt6577_usb_phy *usb)
{
	int ret, cleanup;

	ret = mt6577_bc11_update(usb, 0x34, 0xff, 0);
	cleanup = mt6577_bc11_update(usb, 0x33, 0x0f, 0);
	if (!ret)
		ret = cleanup;
	mt6577_usb_update(usb, 0x1c, BIT(7), 0);
	if (!ret && (readb(usb->base + 0x1c) & BIT(7)))
		ret = -EIO;
	return ret;
}

static int mt6577_bc11_detect(struct mt6577_usb_phy *usb,
			      enum power_supply_usb_type *type)
{
	unsigned int value;
	bool charging_port;
	int ret, cleanup;

	mt6577_usb_update(usb, 0x1c, BIT(7), BIT(7));
	if (!(readb(usb->base + 0x1c) & BIT(7))) {
		ret = -EIO;
		goto out;
	}
	ret = mt6577_bc11_update(usb, 0x33, 0x0f, 3);
	if (ret)
		goto out;
	ret = mt6577_bc11_update(usb, 0x34, 0xff, BIT(7));
	if (ret)
		goto out;
	ret = mt6577_bc11_update(usb, 0x33, 0x0c, 2 << 2);
	if (ret)
		goto out;
	ret = mt6577_bc11_update(usb, 0x34, 0x1f, (1 << 3) | (1 << 1));
	if (ret)
		goto out;
	msleep(100);
	ret = regmap_read(usb->pmic, 0x33, &value);
	if (ret)
		goto out;
	charging_port = value & BIT(7);
	ret = mt6577_bc11_update(usb, 0x33, 0x0c, 0);
	if (ret)
		goto out;
	ret = mt6577_bc11_update(usb, 0x34, 0x1e, 0);
	if (ret)
		goto out;
	msleep(50);
	ret = mt6577_bc11_update(usb, 0x34, 0x7f,
				 charging_port ? 2 << 5 : (1 << 5) | (1 << 1));
	if (ret)
		goto out;
	msleep(80);
	if (charging_port) {
		*type = readb(usb->base + 0x76) & BIT(7) ?
			POWER_SUPPLY_USB_TYPE_DCP : POWER_SUPPLY_USB_TYPE_CDP;
	} else {
		ret = regmap_read(usb->pmic, 0x33, &value);
		if (ret)
			goto out;
		*type = value & BIT(7) ? POWER_SUPPLY_USB_TYPE_UNKNOWN :
			POWER_SUPPLY_USB_TYPE_SDP;
		if (*type == POWER_SUPPLY_USB_TYPE_UNKNOWN) {
			ret = mt6577_bc11_update(usb, 0x34, 0x60, 2 << 5);
			if (ret)
				goto out;
			msleep(80);
		}
	}
out:
	cleanup = mt6577_bc11_cleanup(usb);
	return ret ? ret : cleanup;
}

static void mt6577_bc11_invalidate(struct mt6577_usb_phy *usb)
{
	usb->generation++;
	usb->detected = false;
	usb->data_seen = false;
	usb->grant = 0;
	usb->type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
}

static int mt6577_bc11_refresh(struct mt6577_usb_phy *usb)
{
	enum power_supply_usb_type type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	unsigned int input, generation;
	unsigned long flags;
	bool detect;
	int ret;

	if (!usb->pmic)
		return 0;
	if (usb->source_error) {
		ret = mt6577_bc11_cleanup(usb);
		if (ret)
			return ret;
	}
	ret = regmap_read(usb->pmic, 0x21, &input);
	spin_lock_irqsave(&usb->state_lock, flags);
	if (ret || !(input & BIT(5))) {
		usb->online = false;
		if (ret) {
			usb->generation++;
			usb->detected = false;
		} else {
			mt6577_bc11_invalidate(usb);
			usb->wait_detach = false;
		}
		usb->source_error = ret;
		spin_unlock_irqrestore(&usb->state_lock, flags);
		return ret;
	}
	usb->online = true;
	generation = usb->generation;
	detect = !usb->detected && !usb->data_seen && !usb->wait_detach;
	if (usb->data_seen && !usb->detected) {
		if (usb->type != POWER_SUPPLY_USB_TYPE_CDP)
			usb->type = POWER_SUPPLY_USB_TYPE_SDP;
		usb->detected = true;
	}
	spin_unlock_irqrestore(&usb->state_lock, flags);
	if (detect) {
		ret = mt6577_bc11_detect(usb, &type);
		if (!ret)
			ret = regmap_read(usb->pmic, 0x21, &input);
	}
	spin_lock_irqsave(&usb->state_lock, flags);
	if (ret || !(input & BIT(5))) {
		if (ret) {
			usb->generation++;
			usb->detected = false;
		} else {
			mt6577_bc11_invalidate(usb);
			usb->wait_detach = false;
		}
		usb->online = !ret && (input & BIT(5));
	} else if (detect && generation == usb->generation && !usb->data_seen) {
		usb->type = type;
		usb->detected = true;
	}
	usb->source_error = ret;
	spin_unlock_irqrestore(&usb->state_lock, flags);
	return ret;
}

static void mt6577_bc11_work(struct work_struct *work)
{
	struct mt6577_usb_phy *usb = container_of(to_delayed_work(work),
					       struct mt6577_usb_phy, work);

	mutex_lock(&usb->phy->mutex);
	if (usb->powered && !usb->suspended) {
		mt6577_bc11_refresh(usb);
		schedule_delayed_work(&usb->work, HZ);
	}
	mutex_unlock(&usb->phy->mutex);
	power_supply_changed(usb->psy);
}

static int mt6577_bc11_get_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    union power_supply_propval *val)
{
	struct mt6577_usb_phy *usb = power_supply_get_drvdata(psy);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&usb->state_lock, flags);
	if (usb->source_error) {
		ret = usb->source_error;
		goto out;
	}
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = usb->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = usb->type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 0;
		if (!usb->powered || usb->suspended || !usb->online || !usb->detected)
			break;
		val->intval = usb->grant;
		if ((usb->type == POWER_SUPPLY_USB_TYPE_DCP ||
		     usb->type == POWER_SUPPLY_USB_TYPE_CDP) &&
		    (!usb->grant || usb->grant >= 100000))
			val->intval = 1500000;
		break;
	default:
		ret = -EINVAL;
	}
out:
	spin_unlock_irqrestore(&usb->state_lock, flags);
	return ret;
}

static int mt6577_bc11_set_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    const union power_supply_propval *val)
{
	struct mt6577_usb_phy *usb = power_supply_get_drvdata(psy);
	unsigned long flags;

	if (prop != POWER_SUPPLY_PROP_CURRENT_MAX || val->intval < 0)
		return -EINVAL;
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->generation++;
	if (!val->intval) {
		mt6577_bc11_invalidate(usb);
		usb->wait_detach = usb->powered;
	}
	usb->grant = val->intval;
	if (usb->grant >= 100000) {
		usb->wait_detach = false;
		usb->data_seen = true;
		if (usb->type != POWER_SUPPLY_USB_TYPE_CDP)
			usb->type = POWER_SUPPLY_USB_TYPE_SDP;
		usb->detected = usb->online;
	}
	spin_unlock_irqrestore(&usb->state_lock, flags);
	power_supply_changed(psy);
	return 0;
}

static int mt6577_bc11_writeable(struct power_supply *psy,
				 enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CURRENT_MAX;
}

static int mt6577_usb_calibrate(struct mt6577_usb_phy *usb)
{
	u8 saved = readb(usb->base + 0x12);
	u8 control = readb(usb->base + 0x703);
	u8 cycles = readb(usb->base + 0x701);
	u8 free_run = readb(usb->base + 0x711);
	u8 ready;
	u32 result, value;
	int ret;

	if ((saved & BIT(7)) || (control & 1) || (free_run & 1))
		return -EBUSY;
	writeb(0x81, usb->base + 0x12);
	writeb(1, usb->base + 0x711);
	writeb(4, usb->base + 0x701);
	writeb(control | 1, usb->base + 0x703);
	if (readb(usb->base + 0x710) & 1) {
		ret = -EBUSY;
		goto restore;
	}
	ret = readb_poll_timeout(usb->base + 0x710, ready, ready & 1, 10, 3000000);
	if (ret)
		goto restore;
	result = readl(usb->base + 0x70c);
	if (!result) {
		ret = -ERANGE;
		goto restore;
	}
	value = (1024 * 48 * 28) / result;
	value = value / 1000 + (value % 1000 >= 500);
	/* Qualification bound, not a field mask */
	if (!value || value > 7) {
		ret = -ERANGE;
		goto restore;
	}
restore:
	writeb(control, usb->base + 0x703);
	writeb(cycles, usb->base + 0x701);
	writeb(free_run, usb->base + 0x711);
	writeb(ret ? saved : value, usb->base + 0x12);
	return ret;
}

static void mt6577_usb_save_current(struct mt6577_usb_phy *usb)
{
	mt6577_usb_update(usb, 0x6b, 0x04, 0);
	mt6577_usb_update(usb, 0x6e, 0x01, 0);
	mt6577_usb_update(usb, 0x6a, 0x04, 0);
	mt6577_usb_update(usb, 0x68, 0xc0, 0xc0);
	mt6577_usb_update(usb, 0x68, 0x30, 0x10);
	mt6577_usb_update(usb, 0x68, 0x04, 0x04);
	mt6577_usb_update(usb, 0x69, 0x3c, 0);
	mt6577_usb_update(usb, 0x6a, 0x3a, 0x3a);
	mt6577_usb_update(usb, 0x1c, 0x80, 0);
	mt6577_usb_update(usb, 0x1b, 0x08, 0);
	usleep_range(800, 1000);
	mt6577_usb_update(usb, 0x63, 0x02, 0x02);
	udelay(1);
	mt6577_usb_update(usb, 0x6a, 0x04, 0x04);
	udelay(1);
}

static int mt6577_usb_power_on(struct phy *phy)
{
	struct mt6577_usb_phy *usb = phy_get_drvdata(phy);
	unsigned long flags;
	bool bus_on = false;
	int ret, cleanup;

	if (usb->failed)
		return -EIO;
	if (clk_get_rate(usb->bus) != 61000000)
		return -EINVAL;
	ret = clk_prepare_enable(usb->upll);
	if (ret)
		return ret;
	ret = regulator_enable(usb->vusb);
	if (ret)
		goto disable_upll;
	usleep_range(10000, 11000);
	ret = clk_prepare_enable(usb->bus);
	if (ret)
		goto disable_supply;
	bus_on = true;
	usleep_range(50, 100);
	mt6577_usb_update(usb, 0x6b, 0x04, 0);
	mt6577_usb_update(usb, 0x6e, 0x01, 0);
	mt6577_usb_update(usb, 0x1c, 0x80, 0);
	mt6577_usb_update(usb, 0x02, 0x7f, 0x09);
	mt6577_usb_update(usb, 0x22, 0x03, 0);
	mt6577_usb_update(usb, 0x6a, 0x04, 0);
	mt6577_usb_update(usb, 0x1b, 0x08, 0x08);
	usleep_range(800, 1000);
	mt6577_usb_update(usb, 0x68, 0xf4, 0);
	mt6577_usb_update(usb, 0x69, 0x3c, 0);
	mt6577_usb_update(usb, 0x6a, 0xbe, 0);
	usleep_range(800, 1000);
	ret = mt6577_usb_calibrate(usb);
	if (!ret) {
		spin_lock_irqsave(&usb->state_lock, flags);
		usb->powered = true;
		spin_unlock_irqrestore(&usb->state_lock, flags);
		ret = mt6577_bc11_refresh(usb);
		if (usb->psy)
			power_supply_changed(usb->psy);
	}
	if (!ret) {
		if (usb->pmic)
			schedule_delayed_work(&usb->work, HZ);
		return 0;
	}
	if (usb->pmic && mt6577_bc11_cleanup(usb)) {
		usb->failed = true;
		return ret;
	}
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->powered = false;
	mt6577_bc11_invalidate(usb);
	spin_unlock_irqrestore(&usb->state_lock, flags);
	mt6577_usb_save_current(usb);
disable_supply:
	cleanup = regulator_disable(usb->vusb);
	if (cleanup) {
		usb->failed = true;
		return cleanup;
	}
	if (bus_on)
		clk_disable_unprepare(usb->bus);
disable_upll:
	clk_disable_unprepare(usb->upll);
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->wait_detach = false;
	spin_unlock_irqrestore(&usb->state_lock, flags);
	return ret;
}

static int mt6577_usb_power_off(struct phy *phy)
{
	struct mt6577_usb_phy *usb = phy_get_drvdata(phy);
	unsigned long flags;
	int ret;

	cancel_delayed_work(&usb->work);
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->powered = false;
	usb->online = false;
	mt6577_bc11_invalidate(usb);
	spin_unlock_irqrestore(&usb->state_lock, flags);
	if (usb->psy)
		power_supply_changed(usb->psy);
	if (usb->pmic) {
		ret = mt6577_bc11_cleanup(usb);
		if (ret) {
			spin_lock_irqsave(&usb->state_lock, flags);
			usb->powered = true;
			usb->source_error = ret;
			spin_unlock_irqrestore(&usb->state_lock, flags);
			usb->failed = true;
			return ret;
		}
	}
	mt6577_usb_save_current(usb);
	ret = regulator_disable(usb->vusb);
	if (ret) {
		usb->failed = true;
		return ret;
	}
	clk_disable_unprepare(usb->bus);
	clk_disable_unprepare(usb->upll);
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->wait_detach = false;
	spin_unlock_irqrestore(&usb->state_lock, flags);
	return 0;
}

static int mt6577_usb_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct mt6577_usb_phy *usb = phy_get_drvdata(phy);

	if (usb->failed)
		return -EIO;
	return mode == PHY_MODE_USB_DEVICE && !submode ? 0 : -EINVAL;
}

static const struct phy_ops mt6577_usb_ops = {
	.power_on = mt6577_usb_power_on,
	.power_off = mt6577_usb_power_off,
	.set_mode = mt6577_usb_set_mode,
	.owner = THIS_MODULE,
};

static const enum power_supply_property mt6577_bc11_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc mt6577_bc11_desc = {
	.name = "mt6577-usb-source",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		BIT(POWER_SUPPLY_USB_TYPE_SDP) | BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		BIT(POWER_SUPPLY_USB_TYPE_CDP),
	.properties = mt6577_bc11_properties,
	.num_properties = ARRAY_SIZE(mt6577_bc11_properties),
	.get_property = mt6577_bc11_get_property,
	.set_property = mt6577_bc11_set_property,
	.property_is_writeable = mt6577_bc11_writeable,
};

static void mt6577_bc11_stop(void *data)
{
	struct mt6577_usb_phy *usb = data;

	cancel_delayed_work_sync(&usb->work);
	mutex_lock(&usb->phy->mutex);
	if (usb->powered)
		mt6577_usb_power_off(usb->phy);
	mutex_unlock(&usb->phy->mutex);
}

static int mt6577_bc11_suspend(struct device *dev)
{
	struct mt6577_usb_phy *usb = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = 0;

	mutex_lock(&usb->phy->mutex);
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->suspended = true;
	usb->generation++;
	usb->detected = false;
	spin_unlock_irqrestore(&usb->state_lock, flags);
	if (usb->pmic && usb->powered)
		ret = mt6577_bc11_cleanup(usb);
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->source_error = ret;
	if (ret)
		usb->suspended = false;
	spin_unlock_irqrestore(&usb->state_lock, flags);
	mutex_unlock(&usb->phy->mutex);
	cancel_delayed_work_sync(&usb->work);
	if (ret && usb->powered)
		schedule_delayed_work(&usb->work, HZ);
	if (usb->psy)
		power_supply_changed(usb->psy);
	return ret;
}

static int mt6577_bc11_resume(struct device *dev)
{
	struct mt6577_usb_phy *usb = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = 0;

	mutex_lock(&usb->phy->mutex);
	if (usb->pmic && usb->powered)
		ret = mt6577_bc11_cleanup(usb);
	spin_lock_irqsave(&usb->state_lock, flags);
	usb->source_error = ret;
	usb->suspended = !!ret;
	spin_unlock_irqrestore(&usb->state_lock, flags);
	if (!ret && usb->pmic && usb->powered)
		schedule_delayed_work(&usb->work, 0);
	mutex_unlock(&usb->phy->mutex);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6577_bc11_pm, mt6577_bc11_suspend, mt6577_bc11_resume);

static int mt6577_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6577_usb_phy *usb;
	struct phy_provider *provider;
	struct phy *phy;
	struct power_supply_config config = {};
	unsigned int low, high;
	int ret;

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;
	spin_lock_init(&usb->state_lock);
	INIT_DELAYED_WORK(&usb->work, mt6577_bc11_work);
	usb->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usb->base))
		return PTR_ERR(usb->base);
	usb->upll = devm_clk_get(dev, "upll");
	if (IS_ERR(usb->upll))
		return dev_err_probe(dev, PTR_ERR(usb->upll), "Missing UPLL\n");
	usb->bus = devm_clk_get(dev, "bus");
	if (IS_ERR(usb->bus))
		return dev_err_probe(dev, PTR_ERR(usb->bus), "Missing bus clock\n");
	usb->vusb = devm_regulator_get(dev, "vusb33");
	if (IS_ERR(usb->vusb))
		return dev_err_probe(dev, PTR_ERR(usb->vusb), "Missing VUSB supply\n");
	phy = devm_phy_create(dev, NULL, &mt6577_usb_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	usb->phy = phy;
	phy_set_drvdata(phy, usb);
	platform_set_drvdata(pdev, usb);
	if (device_property_read_bool(dev, "mediatek,bc11")) {
		usb->pmic = regulator_get_regmap(usb->vusb);
		if (IS_ERR(usb->pmic))
			return PTR_ERR(usb->pmic);
		ret = regmap_read(usb->pmic, 0, &low);
		if (ret)
			return ret;
		ret = regmap_read(usb->pmic, 1, &high);
		if (ret)
			return ret;
		if ((high << 8 | low) != 0x1029)
			return -ENODEV;
		config.drv_data = usb;
		config.fwnode = dev_fwnode(dev);
		usb->psy = devm_power_supply_register(dev, &mt6577_bc11_desc, &config);
		if (IS_ERR(usb->psy))
			return PTR_ERR(usb->psy);
	}
	ret = devm_add_action_or_reset(dev, mt6577_bc11_stop, usb);
	if (ret)
		return ret;
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id mt6577_usb_phy_match[] = {
	{ .compatible = "mediatek,mt6577-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6577_usb_phy_match);

static struct platform_driver mt6577_usb_phy_driver = {
	.probe = mt6577_usb_phy_probe,
	.driver = {
		.name = "mt6577-usb-phy",
		.of_match_table = mt6577_usb_phy_match,
		.pm = pm_sleep_ptr(&mt6577_bc11_pm),
	},
};
module_platform_driver(mt6577_usb_phy_driver);

MODULE_DESCRIPTION("MediaTek MT6577 USB PHY");
MODULE_LICENSE("GPL");
