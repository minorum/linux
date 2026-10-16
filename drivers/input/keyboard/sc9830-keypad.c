// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/reset.h>

#define KPD_CTRL		0x00
#define KPD_INT_EN	0x04
#define KPD_INT_STATUS	0x0c
#define KPD_INT_CLEAR	0x10
#define KPD_POLARITY	0x18
#define KPD_DEBOUNCE	0x1c
#define KPD_SLEEP	0x24
#define KPD_DIVIDER	0x28
#define KPD_KEY_STATUS	0x2c

struct sc9830_keypad {
	void __iomem *base;
	struct input_dev *input;
	unsigned int rows;
	unsigned int cols;
	unsigned int row_shift;
	int irq;
};

static irqreturn_t sc9830_keypad_irq(int irq, void *data)
{
	struct sc9830_keypad *keypad = data;
	const unsigned short *keycodes = keypad->input->keycode;
	u32 status = readl(keypad->base + KPD_INT_STATUS) & GENMASK(11, 0);
	u32 keys = readl(keypad->base + KPD_KEY_STATUS);
	unsigned int slot, row, col, code;

	if (!status)
		return IRQ_NONE;
	writel(status, keypad->base + KPD_INT_CLEAR);
	for (slot = 0; slot < 4; slot++) {
		col = (keys >> (slot * 8)) & 7;
		row = (keys >> (slot * 8 + 4)) & 7;
		if (row >= keypad->rows || col >= keypad->cols)
			continue;
		code = keycodes[MATRIX_SCAN_CODE(row, col, keypad->row_shift)];
		if (!code)
			continue;
		if (status & BIT(slot))
			input_report_key(keypad->input, code, 1);
		if (status & BIT(slot + 4))
			input_report_key(keypad->input, code, 0);
	}
	input_sync(keypad->input);
	return IRQ_HANDLED;
}

static void sc9830_keypad_stop(void *data)
{
	struct sc9830_keypad *keypad = data;

	writel(0, keypad->base + KPD_INT_EN);
	writel(0, keypad->base + KPD_CTRL);
	writel(GENMASK(11, 0), keypad->base + KPD_INT_CLEAR);
}

static int sc9830_keypad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct clk_bulk_data *clocks;
	struct sc9830_keypad *keypad;
	struct input_dev *input;
	int ret;

	keypad = devm_kzalloc(dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;
	keypad->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(keypad->base))
		return PTR_ERR(keypad->base);
	keypad->irq = platform_get_irq(pdev, 0);
	if (keypad->irq < 0)
		return keypad->irq;
	ret = matrix_keypad_parse_properties(dev, &keypad->rows, &keypad->cols);
	if (ret)
		return ret;
	if (keypad->rows > 8 || keypad->cols > 8)
		return -EINVAL;
	keypad->row_shift = get_count_order(keypad->cols);
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	keypad->input = input;
	input->name = "SC9830 keypad";
	input->id.bustype = BUS_HOST;
	ret = matrix_keypad_build_keymap(NULL, NULL, keypad->rows,
					 keypad->cols, NULL, input);
	if (ret)
		return ret;
	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, input->evbit);
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Reset unavailable\n");
	ret = devm_clk_bulk_get_all_enabled(dev, &clocks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Clocks unavailable\n");
	if (ret != 2)
		return -EINVAL;
	ret = reset_control_reset(reset);
	if (ret)
		return ret;
	sc9830_keypad_stop(keypad);
	ret = devm_add_action_or_reset(dev, sc9830_keypad_stop, keypad);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, keypad->irq, sc9830_keypad_irq,
			       IRQF_NO_AUTOEN, dev_name(dev), keypad);
	if (ret)
		return ret;
	writel(0xffff, keypad->base + KPD_POLARITY);
	writel(1, keypad->base + KPD_DIVIDER);
	writel(5, keypad->base + KPD_DEBOUNCE);
	writel(32767, keypad->base + KPD_SLEEP);
	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
	}
	platform_set_drvdata(pdev, keypad);
	ret = input_register_device(input);
	if (ret)
		return ret;
	enable_irq(keypad->irq);
	writel(0xff, keypad->base + KPD_INT_EN);
	writel((GENMASK(keypad->rows - 1, 0) << 16) |
	       (GENMASK(keypad->cols - 1, 0) << 8) | BIT(1) | BIT(0),
	       keypad->base + KPD_CTRL);
	return 0;
}

static int sc9830_keypad_suspend(struct device *dev)
{
	struct sc9830_keypad *keypad = dev_get_drvdata(dev);

	return device_may_wakeup(dev) ? enable_irq_wake(keypad->irq) : 0;
}

static int sc9830_keypad_resume(struct device *dev)
{
	struct sc9830_keypad *keypad = dev_get_drvdata(dev);

	return device_may_wakeup(dev) ? disable_irq_wake(keypad->irq) : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc9830_keypad_pm_ops, sc9830_keypad_suspend,
			      sc9830_keypad_resume);

static const struct of_device_id sc9830_keypad_match[] = {
	{ .compatible = "sprd,sc9830-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_keypad_match);

static struct platform_driver sc9830_keypad_driver = {
	.probe = sc9830_keypad_probe,
	.driver = {
		.name = "sc9830-keypad",
		.of_match_table = sc9830_keypad_match,
		.pm = pm_sleep_ptr(&sc9830_keypad_pm_ops),
	},
};
module_platform_driver(sc9830_keypad_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 matrix keypad");
MODULE_LICENSE("GPL");
