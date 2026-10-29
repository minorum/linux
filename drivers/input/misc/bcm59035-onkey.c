// SPDX-License-Identifier: GPL-2.0-only

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define BCM59035_ENV2		0x4d
#define BCM59035_PONKEY_PRESSED	BIT(0)

struct bcm59035_onkey {
	struct input_dev *input;
	struct regmap *regmap;
	struct mutex lock; /* Input state */
	int press_irq;
};

static irqreturn_t bcm59035_onkey_irq(int irq, void *data)
{
	struct bcm59035_onkey *onkey = data;
	unsigned int state;
	int ret;

	mutex_lock(&onkey->lock);
	if (irq == onkey->press_irq) {
		ret = regmap_read(onkey->regmap, BCM59035_ENV2, &state);
		if (ret) {
			dev_err_ratelimited(&onkey->input->dev,
					    "button state read failed: %d\n", ret);
			goto out;
		}
		input_report_key(onkey->input, KEY_POWER, 1);
		input_sync(onkey->input);
		if (state & BCM59035_PONKEY_PRESSED)
			goto out;
	}
	input_report_key(onkey->input, KEY_POWER, 0);
	input_sync(onkey->input);
out:
	mutex_unlock(&onkey->lock);
	return IRQ_HANDLED;
}

static int bcm59035_onkey_sync(struct bcm59035_onkey *onkey)
{
	unsigned int state;
	int ret;

	mutex_lock(&onkey->lock);
	ret = regmap_read(onkey->regmap, BCM59035_ENV2, &state);
	if (!ret) {
		input_report_key(onkey->input, KEY_POWER,
				 !!(state & BCM59035_PONKEY_PRESSED));
		input_sync(onkey->input);
	}
	mutex_unlock(&onkey->lock);
	return ret;
}

static int bcm59035_onkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm59035_onkey *onkey;
	int release_irq, ret;

	onkey = devm_kzalloc(dev, sizeof(*onkey), GFP_KERNEL);
	if (!onkey)
		return -ENOMEM;
	mutex_init(&onkey->lock);
	onkey->regmap = dev_get_regmap(dev->parent, NULL);
	if (!onkey->regmap)
		return -ENODEV;
	onkey->press_irq = platform_get_irq_byname(pdev, "press");
	if (onkey->press_irq < 0)
		return onkey->press_irq;
	release_irq = platform_get_irq_byname(pdev, "release");
	if (release_irq < 0)
		return release_irq;
	onkey->input = devm_input_allocate_device(dev);
	if (!onkey->input)
		return -ENOMEM;
	onkey->input->name = "BCM59035 Power Button";
	onkey->input->phys = "bcm59035-onkey/input0";
	onkey->input->id.bustype = BUS_I2C;
	input_set_capability(onkey->input, EV_KEY, KEY_POWER);
	ret = input_register_device(onkey->input);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(dev, onkey->press_irq, NULL,
					bcm59035_onkey_irq, IRQF_ONESHOT,
					"bcm59035-onkey-press", onkey);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request press IRQ\n");
	ret = devm_request_threaded_irq(dev, release_irq, NULL,
					bcm59035_onkey_irq, IRQF_ONESHOT,
					"bcm59035-onkey-release", onkey);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request release IRQ\n");
	ret = bcm59035_onkey_sync(onkey);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read button state\n");
	return 0;
}

static struct platform_driver bcm59035_onkey_driver = {
	.driver.name = "bcm59035-onkey",
	.probe = bcm59035_onkey_probe,
};
module_platform_driver(bcm59035_onkey_driver);

MODULE_DESCRIPTION("Broadcom BCM59035 power button");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm59035-onkey");
