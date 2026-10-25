// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define BCM2153_SDIO_INPUT 52000000UL
#define BCM2153_SDIO_MAX_DIV 2046

struct bcm2153_sdio_clock {
	struct clk_hw hw;
	void __iomem *base;
	spinlock_t lock; /* Gate and divider. */
};

#define to_bcm2153_sdio_clock(_hw) container_of(_hw, struct bcm2153_sdio_clock, hw)

static unsigned int bcm2153_sdio_clock_divisor(unsigned long rate)
{
	unsigned int divisor;

	divisor = BCM2153_SDIO_INPUT / (rate + 1) + 1;
	return divisor > 1 ? roundup(divisor, 2) : 1;
}

static int bcm2153_sdio_clock_enable(struct clk_hw *hw)
{
	struct bcm2153_sdio_clock *clock = to_bcm2153_sdio_clock(hw);
	unsigned long flags;

	spin_lock_irqsave(&clock->lock, flags);
	writel(1, clock->base);
	spin_unlock_irqrestore(&clock->lock, flags);
	return 0;
}

static void bcm2153_sdio_clock_disable(struct clk_hw *hw)
{
	struct bcm2153_sdio_clock *clock = to_bcm2153_sdio_clock(hw);
	unsigned long flags;

	spin_lock_irqsave(&clock->lock, flags);
	writel(0, clock->base);
	spin_unlock_irqrestore(&clock->lock, flags);
}

static int bcm2153_sdio_clock_is_enabled(struct clk_hw *hw)
{
	return readl(to_bcm2153_sdio_clock(hw)->base) & BIT(0);
}

static unsigned long bcm2153_sdio_clock_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct bcm2153_sdio_clock *clock = to_bcm2153_sdio_clock(hw);
	u32 divider = readl(clock->base + 4);

	if (divider >= BCM2153_SDIO_MAX_DIV)
		return 0;
	return BCM2153_SDIO_INPUT / (divider + 1);
}

static int bcm2153_sdio_clock_determine_rate(struct clk_hw *hw,
					     struct clk_rate_request *req)
{
	unsigned long rate;

	rate = clamp_val(req->rate, BCM2153_SDIO_INPUT / BCM2153_SDIO_MAX_DIV,
			 BCM2153_SDIO_INPUT);
	rate = BCM2153_SDIO_INPUT / bcm2153_sdio_clock_divisor(rate);
	if (rate < req->min_rate || rate > req->max_rate)
		return -EINVAL;
	req->rate = rate;
	return 0;
}

static int bcm2153_sdio_clock_set_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long parent_rate)
{
	struct bcm2153_sdio_clock *clock = to_bcm2153_sdio_clock(hw);
	unsigned long flags;
	unsigned int divisor;
	int ret = 0;

	if (rate < BCM2153_SDIO_INPUT / BCM2153_SDIO_MAX_DIV ||
	    rate > BCM2153_SDIO_INPUT)
		return -EINVAL;
	divisor = bcm2153_sdio_clock_divisor(rate);
	if (divisor > BCM2153_SDIO_MAX_DIV || BCM2153_SDIO_INPUT / divisor != rate)
		return -EINVAL;
	spin_lock_irqsave(&clock->lock, flags);
	if (readl(clock->base))
		ret = -EBUSY;
	else
		writel(divisor - 1, clock->base + 4);
	spin_unlock_irqrestore(&clock->lock, flags);
	return ret;
}

static const struct clk_ops bcm2153_sdio_clock_ops = {
	.enable = bcm2153_sdio_clock_enable,
	.disable = bcm2153_sdio_clock_disable,
	.is_enabled = bcm2153_sdio_clock_is_enabled,
	.recalc_rate = bcm2153_sdio_clock_recalc_rate,
	.determine_rate = bcm2153_sdio_clock_determine_rate,
	.set_rate = bcm2153_sdio_clock_set_rate,
};

static struct clk_hw *bcm2153_sdio_clock_get(struct of_phandle_args *args, void *data)
{
	return args->args_count ? ERR_PTR(-EINVAL) : data;
}

static int bcm2153_sdio_clock_probe(struct platform_device *pdev)
{
	struct clk_init_data init = { .ops = &bcm2153_sdio_clock_ops };
	struct bcm2153_sdio_clock *clock;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) != 8 || !IS_ALIGNED(res->start, 4))
		return -EINVAL;
	clock = devm_kzalloc(&pdev->dev, sizeof(*clock), GFP_KERNEL);
	if (!clock)
		return -ENOMEM;
	clock->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(clock->base))
		return PTR_ERR(clock->base);
	if (readl(clock->base) > 1 || readl(clock->base + 4) >= BCM2153_SDIO_MAX_DIV)
		return -EINVAL;
	spin_lock_init(&clock->lock);
	init.name = dev_name(&pdev->dev);
	clock->hw.init = &init;
	ret = devm_clk_hw_register(&pdev->dev, &clock->hw);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(&pdev->dev, bcm2153_sdio_clock_get, &clock->hw);
}

static const struct of_device_id bcm2153_sdio_clock_match[] = {
	{ .compatible = "brcm,bcm2153-sdio-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_sdio_clock_match);

static struct platform_driver bcm2153_sdio_clock_driver = {
	.probe = bcm2153_sdio_clock_probe,
	.driver = {
		.name = "bcm2153-sdio-clock",
		.of_match_table = bcm2153_sdio_clock_match,
	},
};
module_platform_driver(bcm2153_sdio_clock_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 retained-input SDIO clock");
MODULE_LICENSE("GPL");
