// SPDX-License-Identifier: GPL-2.0-only
#include <dt-bindings/clock/mediatek,mt6577-clk.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-gate.h"
#include "clk-mtk.h"

struct mt6577_root_clk {
	struct clk_hw hw;
	struct regmap *map;
};

#define to_mt6577_root(h) container_of(h, struct mt6577_root_clk, hw)

static unsigned long mt6577_mainpll_rate(struct clk_hw *hw, unsigned long parent)
{
	struct mt6577_root_clk *clk = to_mt6577_root(hw);
	u32 con0, con1, fbsel, prediv, postdiv;
	u64 rate;

	if (regmap_read(clk->map, 0x120, &con0) ||
	    regmap_read(clk->map, 0x124, &con1))
		return 0;
	fbsel = 1U << min_t(u32, FIELD_GET(GENMASK(5, 4), con0), 2);
	prediv = 1U << min_t(u32, FIELD_GET(GENMASK(7, 6), con0), 2);
	postdiv = 1U << min_t(u32, FIELD_GET(GENMASK(5, 4), con1), 2);
	rate = (u64)parent * (FIELD_GET(GENMASK(14, 8), con0) + 1) * fbsel;
	rate = div_u64(rate, prediv * postdiv);
	return rate <= ULONG_MAX ? rate : 0;
}

static const struct clk_ops mt6577_mainpll_ops = {
	.recalc_rate = mt6577_mainpll_rate,
};

static u8 mt6577_axi_parent(struct clk_hw *hw)
{
	struct mt6577_root_clk *clk = to_mt6577_root(hw);
	u32 mux;

	if (regmap_read(clk->map, 0, &mux))
		return 0;
	mux &= 3;
	return mux == 3 ? 1 : mux;
}

static unsigned long mt6577_axi_rate(struct clk_hw *hw, unsigned long parent)
{
	static const unsigned int divisors[] = { 1, 4, 5, 6 };
	struct mt6577_root_clk *clk = to_mt6577_root(hw);
	u32 divider;

	if (regmap_read(clk->map, 4, &divider))
		return 0;
	return parent / divisors[FIELD_GET(GENMASK(4, 3), divider)];
}

static const struct clk_ops mt6577_axi_ops = {
	.get_parent = mt6577_axi_parent,
	.recalc_rate = mt6577_axi_rate,
};

static const struct mtk_gate_regs mt6577_peri_regs = {
	.sta_ofs = 0x10,
	.set_ofs = 0x10,
	.clr_ofs = 0x10,
};

static const struct mtk_gate mt6577_pwm_gates[] = {
	GATE_MTK(CLK_MT6577_PWM1, "mt6577_pwm1", NULL,
		 &mt6577_peri_regs, 2, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT6577_PWM2, "mt6577_pwm2", NULL,
		 &mt6577_peri_regs, 3, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT6577_PWM3, "mt6577_pwm3", NULL,
		 &mt6577_peri_regs, 4, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT6577_PWM456, "mt6577_pwm456", NULL,
		 &mt6577_peri_regs, 5, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT6577_PWM7, "mt6577_pwm7", NULL,
		 &mt6577_peri_regs, 6, &mtk_clk_gate_ops_no_setclr),
};

struct mt6577_peri {
	struct mtk_gate gates[ARRAY_SIZE(mt6577_pwm_gates)];
	struct clk_hw_onecell_data *data;
};

static void mt6577_free_data(void *data)
{
	mtk_free_clk_data(data);
}

static void mt6577_unregister_gates(void *data)
{
	struct mt6577_peri *peri = data;

	mtk_clk_unregister_gates(peri->gates, ARRAY_SIZE(peri->gates), peri->data);
}

static int mt6577_peri_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6577_peri *peri;
	struct clk_hw *reference;
	int i, ret;

	peri = devm_kzalloc(dev, sizeof(*peri), GFP_KERNEL);
	if (!peri)
		return -ENOMEM;
	peri->data = mtk_alloc_clk_data(CLK_MT6577_PERI_NR_CLK);
	if (!peri->data)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, mt6577_free_data, peri->data);
	if (ret)
		return ret;
	reference = devm_clk_hw_register_fixed_factor_index(dev, "mt6577_pwm_ref", 0, 0, 1, 4);
	if (IS_ERR(reference))
		return PTR_ERR(reference);
	peri->data->hws[CLK_MT6577_PWM_REF] = reference;
	memcpy(peri->gates, mt6577_pwm_gates, sizeof(peri->gates));
	for (i = 0; i < ARRAY_SIZE(peri->gates); i++)
		peri->gates[i].parent_name = clk_hw_get_name(reference);
	ret = mtk_clk_register_gates(dev, dev->of_node, peri->gates,
				     ARRAY_SIZE(peri->gates), peri->data);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, mt6577_unregister_gates, peri);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, peri->data);
}

struct mt6577_root_data {
	const struct clk_ops *ops;
	unsigned int parents;
};

static const struct mt6577_root_data mt6577_mainpll_data = {
	.ops = &mt6577_mainpll_ops,
	.parents = 1,
};

static const struct mt6577_root_data mt6577_axi_data = {
	.ops = &mt6577_axi_ops,
	.parents = 3,
};

static const struct mt6577_root_data mt6577_peri_data;

static int mt6577_clk_probe(struct platform_device *pdev)
{
	const struct mt6577_root_data *data = of_device_get_match_data(&pdev->dev);
	struct clk_parent_data parents[] = { { .index = 0 }, { .index = 1 }, { .index = 2 } };
	struct clk_init_data init = {
		.name = dev_name(&pdev->dev),
		.parent_data = parents,
		.flags = CLK_IS_CRITICAL | CLK_GET_RATE_NOCACHE,
	};
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *provider;
	struct mt6577_root_clk *clk;
	int ret;

	if (!data->ops)
		return mt6577_peri_probe(pdev);
	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return -ENOMEM;
	clk->map = device_node_to_regmap(dev->of_node);
	if (IS_ERR(clk->map))
		return dev_err_probe(dev, PTR_ERR(clk->map), "Missing clock registers\n");
	init.ops = data->ops;
	init.num_parents = data->parents;
	clk->hw.init = &init;
	ret = devm_clk_hw_register(dev, &clk->hw);
	if (ret)
		return ret;
	provider = devm_kzalloc(dev, struct_size(provider, hws, 1), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;
	provider->num = 1;
	provider->hws[0] = &clk->hw;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, provider);
}

static const struct of_device_id mt6577_clk_match[] = {
	{ .compatible = "mediatek,mt6577-apmixedsys", .data = &mt6577_mainpll_data },
	{ .compatible = "mediatek,mt6577-infracfg", .data = &mt6577_axi_data },
	{ .compatible = "mediatek,mt6577-pericfg", .data = &mt6577_peri_data },
	{ }
};

static struct platform_driver mt6577_clk_driver = {
	.probe = mt6577_clk_probe,
	.driver = {
		.name = "mt6577-clock",
		.of_match_table = mt6577_clk_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mt6577_clk_driver);
