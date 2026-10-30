// SPDX-License-Identifier: GPL-2.0
#include <linux/limits.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "core.h"

struct dwc2_gadget_supply {
	struct power_supply *psy;
	struct power_supply_desc desc;
	spinlock_t lock; /* Budget publication */
	unsigned long sequence;
	int current_ua;
};

static const enum power_supply_property dwc2_supply_properties[] = {
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int dwc2_supply_get_property(struct power_supply *psy,
				    enum power_supply_property property,
				    union power_supply_propval *value)
{
	struct dwc2_gadget_supply *supply = power_supply_get_drvdata(psy);

	if (property != POWER_SUPPLY_PROP_CURRENT_MAX)
		return -EINVAL;
	value->intval = READ_ONCE(supply->current_ua);
	return 0;
}

void dwc2_gadget_supply_revoke(struct dwc2_hsotg *hsotg)
{
	struct dwc2_gadget_supply *supply = hsotg->gadget_supply;
	unsigned long flags;
	bool changed;

	if (!IS_REACHABLE(CONFIG_POWER_SUPPLY) || !supply)
		return;
	spin_lock_irqsave(&supply->lock, flags);
	supply->sequence++;
	changed = supply->current_ua != 0;
	WRITE_ONCE(supply->current_ua, 0);
	spin_unlock_irqrestore(&supply->lock, flags);
	if (changed)
		power_supply_changed(supply->psy);
}

int dwc2_gadget_supply_draw(struct dwc2_hsotg *hsotg, unsigned int mA)
{
	struct dwc2_gadget_supply *supply = IS_REACHABLE(CONFIG_POWER_SUPPLY) ?
					 hsotg->gadget_supply : NULL;
	unsigned long flags, sequence = 0;
	bool changed = false;
	int ret, current_ua;

	if (supply) {
		if (mA > INT_MAX / 1000) {
			dwc2_gadget_supply_revoke(hsotg);
			return -EINVAL;
		}
		spin_lock_irqsave(&supply->lock, flags);
		sequence = ++supply->sequence;
		spin_unlock_irqrestore(&supply->lock, flags);
	}
	if (!IS_ERR_OR_NULL(hsotg->uphy))
		ret = usb_phy_set_power(hsotg->uphy, mA);
	else
		ret = supply ? 0 : -ENOTSUPP;
	if (!supply)
		return ret;

	spin_lock_irqsave(&supply->lock, flags);
	if (sequence == supply->sequence) {
		current_ua = ret ? 0 : mA * 1000;
		changed = supply->current_ua != current_ua;
		WRITE_ONCE(supply->current_ua, current_ua);
	} else if (!ret) {
		ret = -ESHUTDOWN;
	}
	spin_unlock_irqrestore(&supply->lock, flags);
	if (changed)
		power_supply_changed(supply->psy);
	return ret;
}

static void dwc2_supply_put_connector(void *data)
{
	fwnode_handle_put(data);
}

int dwc2_gadget_supply_init(struct dwc2_hsotg *hsotg)
{
	struct device *dev = hsotg->dev;
	struct power_supply_config config = {};
	struct fwnode_handle *connector, *port;
	struct dwc2_gadget_supply *supply;
	int ret;

	if (!IS_REACHABLE(CONFIG_POWER_SUPPLY))
		return 0;
	connector = device_get_named_child_node(dev, "connector");
	if (!connector)
		return 0;
	if (!fwnode_device_is_available(connector)) {
		fwnode_handle_put(connector);
		return 0;
	}
	port = device_get_named_child_node(dev, "port");
	if (port) {
		fwnode_handle_put(port);
		ret = -EINVAL;
		goto put_connector;
	}
	if (hsotg->dr_mode != USB_DR_MODE_PERIPHERAL ||
	    device_property_present(dev, "usb-role-switch") ||
	    fwnode_property_string_array_count(connector, "compatible") != 1 ||
	    fwnode_property_match_string(connector, "compatible", "usb-b-connector") != 0 ||
	    fwnode_property_present(connector, "id-gpios") ||
	    fwnode_property_present(connector, "vbus-gpios") ||
	    fwnode_property_present(connector, "vbus-supply")) {
		ret = -EINVAL;
		goto put_connector;
	}
	supply = devm_kzalloc(dev, sizeof(*supply), GFP_KERNEL);
	if (!supply) {
		ret = -ENOMEM;
		goto put_connector;
	}
	ret = devm_add_action_or_reset(dev, dwc2_supply_put_connector, connector);
	if (ret)
		return ret;
	supply->desc.name = devm_kasprintf(dev, GFP_KERNEL, "%s-vbus", dev_name(dev));
	if (!supply->desc.name)
		return -ENOMEM;
	spin_lock_init(&supply->lock);
	supply->desc.type = POWER_SUPPLY_TYPE_USB;
	supply->desc.properties = dwc2_supply_properties;
	supply->desc.num_properties = ARRAY_SIZE(dwc2_supply_properties);
	supply->desc.get_property = dwc2_supply_get_property;
	config.fwnode = connector;
	config.drv_data = supply;
	supply->psy = devm_power_supply_register(dev, &supply->desc, &config);
	if (IS_ERR(supply->psy))
		return PTR_ERR(supply->psy);
	hsotg->gadget_supply = supply;
	return 0;

put_connector:
	fwnode_handle_put(connector);
	return ret;
}
