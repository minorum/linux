// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define MT8377_MIPI_CON0		0x00
#define MT8377_MIPI_CON1		0x04
#define MT8377_MIPI_CON3		0x0c
#define MT8377_MIPI_CON6		0x18
#define MT8377_MIPI_CON9		0x24
#define MT8377_MIPI_PLL_EN	BIT(0)
#define MT8377_MIPI_CLKR_EN	BIT(1)
#define MT8377_MIPI_DPI_EN	BIT(5)
#define MT8377_MIPI_DIV1		GENMASK(6, 0)
#define MT8377_MIPI_DIV2		GENMASK(11, 8)
#define MT8377_MIPI_ENABLE	(MT8377_MIPI_PLL_EN | MT8377_MIPI_CLKR_EN | \
				 MT8377_MIPI_DPI_EN)

struct mt8377_mipi_clk {
	struct clk_hw hw;
	void __iomem *base;
	struct regmap *mmsys;
	struct regmap *apmixed;
	struct regmap *infra;
	u32 div1;
	u32 div2;
	u32 reference;
};

#define to_mt8377_mipi_clk(h) container_of(h, struct mt8377_mipi_clk, hw)

static unsigned long mt8377_mipi_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct mt8377_mipi_clk *mipi = to_mt8377_mipi_clk(hw);

	if (!mipi->div2 || mipi->reference)
		return 0;

	return div_u64((u64)parent_rate * mipi->div1, 2 * mipi->div2);
}

static int mt8377_mipi_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	unsigned long rate = div_u64((u64)req->best_parent_rate * 32, 6);

	if (!rate || rate < req->min_rate || rate > req->max_rate)
		return -EINVAL;

	req->rate = rate;
	return 0;
}

static int mt8377_mipi_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct mt8377_mipi_clk *mipi = to_mt8377_mipi_clk(hw);

	if (rate != div_u64((u64)parent_rate * 32, 6))
		return -EINVAL;

	mipi->div1 = 32;
	mipi->div2 = 3;
	mipi->reference = 0;
	return 0;
}

static int mt8377_mipi_prepare(struct clk_hw *hw)
{
	struct mt8377_mipi_clk *mipi = to_mt8377_mipi_clk(hw);
	u32 val, old_apmixed, old_mmsys, old_infra;
	u32 old_control, old_divider, old_reference, old_bias;
	u16 old_ldo;
	int ret;

	if (!mipi->div2 || mipi->reference)
		return -EINVAL;
	ret = regmap_read(mipi->infra, 0x890, &old_infra);
	if (ret)
		return ret;
	if (old_infra & BIT(12))
		return -EBUSY;
	ret = regmap_read(mipi->apmixed, 0x44, &old_apmixed);
	if (ret)
		return ret;
	ret = regmap_read(mipi->mmsys, 0x858, &old_mmsys);
	if (ret)
		return ret;
	old_control = readl(mipi->base + MT8377_MIPI_CON0);
	old_divider = readl(mipi->base + MT8377_MIPI_CON1);
	old_reference = readl(mipi->base + MT8377_MIPI_CON3);
	old_bias = readl(mipi->base + MT8377_MIPI_CON6);
	old_ldo = readw(mipi->base + MT8377_MIPI_CON9);

	ret = regmap_update_bits(mipi->apmixed, 0x44, BIT(4), BIT(4));
	if (ret)
		return ret;
	ret = regmap_update_bits(mipi->mmsys, 0x858, GENMASK(15, 0), 0x8000);
	if (ret)
		goto restore_apmixed;
	ret = regmap_update_bits(mipi->infra, 0x890, BIT(4), 0);
	if (ret)
		goto restore_infra;

	writel(old_reference & ~GENMASK(3, 2), mipi->base + MT8377_MIPI_CON3);
	val = (old_divider & ~(MT8377_MIPI_DIV1 | MT8377_MIPI_DIV2)) |
		FIELD_PREP(MT8377_MIPI_DIV1, mipi->div1) |
		FIELD_PREP(MT8377_MIPI_DIV2, mipi->div2);
	writel(val, mipi->base + MT8377_MIPI_CON1);
	writel(old_control | MT8377_MIPI_ENABLE, mipi->base + MT8377_MIPI_CON0);
	writel(old_bias | GENMASK(1, 0), mipi->base + MT8377_MIPI_CON6);
	writew(0x4008, mipi->base + MT8377_MIPI_CON9);
	ret = regmap_update_bits(mipi->infra, 0x890, BIT(8), BIT(8));
	if (ret)
		goto restore_phy;
	usleep_range(1000, 2000);
	return 0;

restore_phy:
	writel(old_control, mipi->base + MT8377_MIPI_CON0);
	writel(old_divider, mipi->base + MT8377_MIPI_CON1);
	writel(old_reference, mipi->base + MT8377_MIPI_CON3);
	writel(old_bias, mipi->base + MT8377_MIPI_CON6);
	writew(old_ldo, mipi->base + MT8377_MIPI_CON9);
restore_infra:
	regmap_update_bits(mipi->infra, 0x890, BIT(4) | BIT(8), old_infra);
	regmap_update_bits(mipi->mmsys, 0x858, GENMASK(15, 0), old_mmsys);
restore_apmixed:
	regmap_update_bits(mipi->apmixed, 0x44, BIT(4), old_apmixed);
	return ret;
}

static void mt8377_mipi_unprepare(struct clk_hw *hw)
{
	struct mt8377_mipi_clk *mipi = to_mt8377_mipi_clk(hw);
	u32 val = readl(mipi->base + MT8377_MIPI_CON0);
	u32 infra, mask = MT8377_MIPI_PLL_EN | MT8377_MIPI_DPI_EN;
	bool rx_active;

	rx_active = regmap_read(mipi->infra, 0x890, &infra) || (infra & BIT(12));
	if (!rx_active)
		mask |= MT8377_MIPI_CLKR_EN;
	writel(val & ~mask, mipi->base + MT8377_MIPI_CON0);
	writew(readw(mipi->base + MT8377_MIPI_CON9) & ~BIT(3),
	       mipi->base + MT8377_MIPI_CON9);
	regmap_update_bits(mipi->infra, 0x890, BIT(8), 0);
	if (!rx_active) {
		val = readl(mipi->base + MT8377_MIPI_CON6);
		writel(val & ~GENMASK(1, 0), mipi->base + MT8377_MIPI_CON6);
		regmap_update_bits(mipi->infra, 0x890, BIT(4), BIT(4));
		regmap_update_bits(mipi->apmixed, 0x44, BIT(4), 0);
	}
}

static int mt8377_mipi_is_prepared(struct clk_hw *hw)
{
	struct mt8377_mipi_clk *mipi = to_mt8377_mipi_clk(hw);

	return (readl(mipi->base + MT8377_MIPI_CON0) & MT8377_MIPI_ENABLE) ==
		MT8377_MIPI_ENABLE;
}

static const struct clk_ops mt8377_mipi_ops = {
	.recalc_rate = mt8377_mipi_recalc_rate,
	.determine_rate = mt8377_mipi_determine_rate,
	.set_rate = mt8377_mipi_set_rate,
	.prepare = mt8377_mipi_prepare,
	.unprepare = mt8377_mipi_unprepare,
	.is_prepared = mt8377_mipi_is_prepared,
};

static int mt8377_mipi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_parent_data parent = { .index = 0 };
	struct clk_init_data init = {
		.ops = &mt8377_mipi_ops,
		.parent_data = &parent,
		.num_parents = 1,
		.flags = CLK_SET_RATE_GATE,
	};
	struct mt8377_mipi_clk *mipi;
	u32 val;
	int ret;

	mipi = devm_kzalloc(dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;

	mipi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mipi->base))
		return PTR_ERR(mipi->base);

	mipi->mmsys = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,mmsys");
	if (IS_ERR(mipi->mmsys))
		return dev_err_probe(dev, PTR_ERR(mipi->mmsys), "Missing MMSYS\n");

	mipi->apmixed = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,apmixedsys");
	if (IS_ERR(mipi->apmixed))
		return dev_err_probe(dev, PTR_ERR(mipi->apmixed), "Missing APMIXEDSYS\n");
	mipi->infra = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,infracfg");
	if (IS_ERR(mipi->infra))
		return dev_err_probe(dev, PTR_ERR(mipi->infra), "Missing INFRACFG\n");
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	val = readl(mipi->base + MT8377_MIPI_CON1);
	mipi->div1 = FIELD_GET(MT8377_MIPI_DIV1, val);
	mipi->div2 = FIELD_GET(MT8377_MIPI_DIV2, val);
	mipi->reference = readl(mipi->base + MT8377_MIPI_CON3) & GENMASK(3, 2);

	init.name = dev_name(dev);
	mipi->hw.init = &init;
	ret = devm_clk_hw_register(dev, &mipi->hw);
	if (!ret)
		ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &mipi->hw);
	pm_runtime_put(dev);
	return ret;
}

static const struct of_device_id mt8377_mipi_match[] = {
	{ .compatible = "mediatek,mt8377-mipi-tx" },
	{ .compatible = "mediatek,mt6577-mipi-tx" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8377_mipi_match);

static struct platform_driver mt8377_mipi_driver = {
	.probe = mt8377_mipi_probe,
	.driver = {
		.name = "mt8377-mipi-clock",
		.of_match_table = mt8377_mipi_match,
	},
};
module_platform_driver(mt8377_mipi_driver);

MODULE_DESCRIPTION("MediaTek MT8377 MIPI DPI PLL");
MODULE_LICENSE("GPL");
