// SPDX-License-Identifier: GPL-2.0
/*
 * Power off through MediaTek PMIC
 *
 * Copyright (C) 2018 MediaTek Inc.
 *
 * Author: Sean Wang <sean.wang@mediatek.com>
 *
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6397/rtc.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>

struct mt6323_pwrc_data {
	u32 wrtgr;
	u32 bbpu;
	bool mmio;
};

static const struct mt6323_pwrc_data mt6323_pwrc_data = {
	.wrtgr = RTC_WRTGR_MT6323,
	.bbpu = RTC_BBPU_KEY,
};

static const struct mt6323_pwrc_data mt8377_pwrc_data = {
	.wrtgr = 0x74,
	.bbpu = RTC_BBPU_KEY | BIT(3) | BIT(0),
	.mmio = true,
};

struct mt6323_pwrc {
	struct device *dev;
	struct regmap *regmap;
	u32 base;
	const struct mt6323_pwrc_data *data;
	raw_spinlock_t lock;
	struct clk_hw clock;
};

static int mt8377_pwrc_wait(struct mt6323_pwrc *pwrc)
{
	unsigned int val;

	return regmap_read_poll_timeout_atomic(pwrc->regmap, RTC_BBPU, val,
					      !(val & RTC_BBPU_CBUSY),
					      MTK_RTC_POLL_DELAY_US, MTK_RTC_POLL_TIMEOUT);
}

static int mt6323_pwrc_write(struct mt6323_pwrc *pwrc, u32 reg, u32 value)
{
	unsigned int val;
	int ret;

	ret = regmap_write(pwrc->regmap, pwrc->base + reg, value);
	if (ret)
		return ret;
	ret = regmap_write(pwrc->regmap, pwrc->base + pwrc->data->wrtgr, 1);
	if (ret)
		return ret;
	if (pwrc->data->mmio)
		return regmap_read_poll_timeout_atomic(pwrc->regmap,
				pwrc->base + RTC_BBPU, val,
				!(val & RTC_BBPU_CBUSY),
				MTK_RTC_POLL_DELAY_US, MTK_RTC_POLL_TIMEOUT);

	return regmap_read_poll_timeout(pwrc->regmap,
					pwrc->base + RTC_BBPU, val,
					!(val & RTC_BBPU_CBUSY),
					MTK_RTC_POLL_DELAY_US,
					MTK_RTC_POLL_TIMEOUT);
}

static int mt8377_pwrc_unlock(struct mt6323_pwrc *pwrc)
{
	int ret;

	ret = mt8377_pwrc_wait(pwrc);
	if (ret)
		return ret;
	ret = mt6323_pwrc_write(pwrc, 0x68, 0x586a);
	if (ret)
		return ret;
	return mt6323_pwrc_write(pwrc, 0x68, 0x9136);
}

static int mt8377_rtc_set_clock(struct mt6323_pwrc *pwrc, bool enable)
{
	unsigned int con, users;
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&pwrc->lock, flags);
	ret = mt8377_pwrc_unlock(pwrc);
	if (ret)
		goto out;
	if (!enable) {
		ret = regmap_read(pwrc->regmap, 0x58, &users);
		if (ret || (users & GENMASK(12, 8)))
			goto out;
	}
	ret = regmap_read(pwrc->regmap, 0x78, &con);
	if (ret)
		goto out;
	con = enable ? con & ~BIT(5) : con | BIT(5);
	ret = mt6323_pwrc_write(pwrc, 0x78, con);
out:
	raw_spin_unlock_irqrestore(&pwrc->lock, flags);
	return ret;
}

static int mt6323_do_pwroff(struct sys_off_data *data)
{
	struct mt6323_pwrc *pwrc = data->cb_data;
	unsigned long flags = 0;
	int ret;

	if (pwrc->data->mmio) {
		raw_spin_lock_irqsave(&pwrc->lock, flags);
		ret = mt8377_pwrc_unlock(pwrc);
		if (ret)
			goto unlock;
	}
	ret = mt6323_pwrc_write(pwrc, RTC_BBPU, pwrc->data->bbpu);
unlock:
	if (pwrc->data->mmio)
		raw_spin_unlock_irqrestore(&pwrc->lock, flags);
	if (ret)
		goto err;

	/* Wait some time until system down, otherwise, notice with a warn */
	mdelay(1000);

	WARN_ONCE(1, "Unable to power off system\n");

	return NOTIFY_DONE;

err:
	dev_err(pwrc->dev, "failed to power off: %d\n", ret);
	return NOTIFY_DONE;
}

static bool mt8377_pwrc_readable(struct device *dev, unsigned int reg)
{
	return reg == RTC_BBPU || reg == 0x58 || reg == 0x78;
}

static bool mt8377_pwrc_writeable(struct device *dev, unsigned int reg)
{
	return reg == RTC_BBPU || reg == 0x68 || reg == 0x74 || reg == 0x78;
}

static const struct regmap_config mt8377_pwrc_regmap = {
	.reg_bits = 32,
	.val_bits = 16,
	.reg_stride = 4,
	.max_register = 0x78,
	.fast_io = true,
	.use_raw_spinlock = true,
	.cache_type = REGCACHE_NONE,
	.readable_reg = mt8377_pwrc_readable,
	.writeable_reg = mt8377_pwrc_writeable,
};

static int mt8377_rtc_clock_prepare(struct clk_hw *hw)
{
	return mt8377_rtc_set_clock(container_of(hw, struct mt6323_pwrc, clock), true);
}

static void mt8377_rtc_clock_unprepare(struct clk_hw *hw)
{
	struct mt6323_pwrc *pwrc = container_of(hw, struct mt6323_pwrc, clock);
	int ret;

	ret = mt8377_rtc_set_clock(pwrc, false);
	if (ret)
		dev_err(pwrc->dev, "failed to disable RTC clock: %d\n", ret);
}

static unsigned long mt8377_rtc_clock_rate(struct clk_hw *hw, unsigned long parent)
{
	return 32768;
}

static const struct clk_ops mt8377_rtc_clock_ops = {
	.prepare = mt8377_rtc_clock_prepare,
	.unprepare = mt8377_rtc_clock_unprepare,
	.recalc_rate = mt8377_rtc_clock_rate,
};

static int mt8377_rtc_register_clock(struct mt6323_pwrc *pwrc)
{
	struct clk_init_data init = {
		.name = "mt8377_rtc_32k",
		.ops = &mt8377_rtc_clock_ops,
		.flags = CLK_IGNORE_UNUSED,
	};
	int ret;

	if (!IS_ENABLED(CONFIG_COMMON_CLK))
		return -EOPNOTSUPP;
	pwrc->clock.init = &init;
	ret = devm_clk_hw_register(pwrc->dev, &pwrc->clock);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(pwrc->dev, of_clk_hw_simple_get, &pwrc->clock);
}

static int mt6323_pwrc_probe(struct platform_device *pdev)
{
	struct mt6323_pwrc *pwrc;
	int ret;

	pwrc = devm_kzalloc(&pdev->dev, sizeof(*pwrc), GFP_KERNEL);
	if (!pwrc)
		return -ENOMEM;

	pwrc->data = of_device_get_match_data(&pdev->dev);
	if (!pwrc->data)
		pwrc->data = &mt6323_pwrc_data;
	if (pwrc->data->mmio) {
		void __iomem *base;

		base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(base))
			return PTR_ERR(base);
		pwrc->regmap = devm_regmap_init_mmio(&pdev->dev, base,
						     &mt8377_pwrc_regmap);
		if (IS_ERR(pwrc->regmap))
			return PTR_ERR(pwrc->regmap);
	} else {
		struct mt6397_chip *chip = dev_get_drvdata(pdev->dev.parent);
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -EINVAL;
		pwrc->base = res->start;
		pwrc->regmap = chip->regmap;
	}
	pwrc->dev = &pdev->dev;
	raw_spin_lock_init(&pwrc->lock);
	if (pwrc->data->mmio && of_property_present(pdev->dev.of_node, "#clock-cells")) {
		ret = mt8377_rtc_register_clock(pwrc);
		if (ret)
			return dev_err_probe(pwrc->dev, ret, "failed to register RTC clock\n");
	}

	ret = devm_register_sys_off_handler(pwrc->dev,
					    SYS_OFF_MODE_POWER_OFF,
					    SYS_OFF_PRIO_DEFAULT,
					    mt6323_do_pwroff,
					    pwrc);
	if (ret)
		return dev_err_probe(pwrc->dev, ret, "failed to register power-off handler\n");

	return 0;
}

static const struct of_device_id mt6323_pwrc_dt_match[] = {
	{ .compatible = "mediatek,mt6323-pwrc", .data = &mt6323_pwrc_data },
	{ .compatible = "mediatek,mt8377-pwrc", .data = &mt8377_pwrc_data },
	{ .compatible = "mediatek,mt6577-pwrc", .data = &mt8377_pwrc_data },
	{},
};
MODULE_DEVICE_TABLE(of, mt6323_pwrc_dt_match);

static struct platform_driver mt6323_pwrc_driver = {
	.probe          = mt6323_pwrc_probe,
	.driver         = {
		.name   = "mt6323-pwrc",
		.of_match_table = mt6323_pwrc_dt_match,
	},
};

module_platform_driver(mt6323_pwrc_driver);

MODULE_DESCRIPTION("MediaTek RTC power-off driver");
MODULE_AUTHOR("Sean Wang <sean.wang@mediatek.com>");
