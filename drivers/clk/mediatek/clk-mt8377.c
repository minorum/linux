// SPDX-License-Identifier: GPL-2.0-only
#include <dt-bindings/clock/mediatek,mt8377-clk.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-gate.h"
#include "clk-mtk.h"

struct mt8377_root_clk {
	struct clk_hw hw;
	struct regmap *map;
	u8 shift;
};

#define to_mt8377_root(h) container_of(h, struct mt8377_root_clk, hw)

static unsigned long mt8377_mainpll_rate(struct clk_hw *hw, unsigned long parent)
{
	struct mt8377_root_clk *clk = to_mt8377_root(hw);
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

static const struct clk_ops mt8377_mainpll_ops = {
	.recalc_rate = mt8377_mainpll_rate,
};

struct mt8377_wpll {
	struct clk_hw hw;
	void __iomem *base;
	u16 offset;
	u16 mask;
	u16 enabled;
	unsigned long rate;
	unsigned int delay;
};

static int mt8377_wpll_enable(struct clk_hw *hw)
{
	struct mt8377_wpll *clk = container_of(hw, struct mt8377_wpll, hw);
	u16 value = readw(clk->base + clk->offset);

	writew((value & ~clk->mask) | clk->enabled, clk->base + clk->offset);
	if ((value & BIT(0)) != (clk->enabled & BIT(0)))
		udelay(clk->delay);
	return 0;
}

static int mt8377_wpll_is_enabled(struct clk_hw *hw)
{
	struct mt8377_wpll *clk = container_of(hw, struct mt8377_wpll, hw);

	return (readw(clk->base + clk->offset) & clk->mask) == clk->enabled;
}

static unsigned long mt8377_wpll_rate(struct clk_hw *hw, unsigned long parent)
{
	struct mt8377_wpll *clk = container_of(hw, struct mt8377_wpll, hw);

	return clk->rate;
}

static const struct clk_ops mt8377_wpll_ops = {
	.enable = mt8377_wpll_enable,
	.is_enabled = mt8377_wpll_is_enabled,
	.recalc_rate = mt8377_wpll_rate,
};

static struct clk_hw *mt8377_register_wpll(struct platform_device *pdev,
					   void __iomem *base, unsigned int index)
{
	static const struct {
		const char *name;
		u16 offset, mask, enabled;
		unsigned long rate;
		unsigned int delay;
	} outputs[] = {
		{ "mt8377_wpll197", 0x1c0, 3, 3, 197000000, 30 },
		{ "mt8377_wpll61", 0x1c0, BIT(14) | BIT(0), BIT(14) | BIT(0), 61000000, 30 },
		{ "mt8377_upll", 0x160, BIT(0), 0, 0, 20 },
	};
	const struct clk_parent_data parent = { .index = 0 };
	const struct clk_init_data init = {
		.name = outputs[index].name,
		.ops = &mt8377_wpll_ops,
		.parent_data = &parent,
		.num_parents = 1,
		/* Shared firmware consumers */
		.flags = CLK_IS_CRITICAL,
	};
	struct mt8377_wpll *clk;
	int ret;

	clk = devm_kzalloc(&pdev->dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return ERR_PTR(-ENOMEM);
	clk->base = base;
	clk->offset = outputs[index].offset;
	clk->mask = outputs[index].mask;
	clk->enabled = outputs[index].enabled;
	clk->rate = outputs[index].rate;
	clk->delay = outputs[index].delay;
	clk->hw.init = &init;
	ret = devm_clk_hw_register(&pdev->dev, &clk->hw);
	return ret ? ERR_PTR(ret) : &clk->hw;
}

static u8 mt8377_msdc0_parent(struct clk_hw *hw)
{
	struct mt8377_root_clk *clk = to_mt8377_root(hw);
	u32 mux;

	if (regmap_read(clk->map, 0x0c, &mux))
		return 2;
	return min_t(u32, (mux >> clk->shift) & 3, 2);
}

static int mt8377_msdc_prepare(struct clk_hw *hw)
{
	struct mt8377_root_clk *clk = to_mt8377_root(hw);
	u32 mux;
	int ret;

	ret = regmap_read(clk->map, 0x0c, &mux);
	if (ret)
		return ret;
	return ((mux >> clk->shift) & 3) < 2 ? 0 : -EINVAL;
}

static const struct clk_ops mt8377_msdc0_ops = {
	.prepare = mt8377_msdc_prepare,
	.get_parent = mt8377_msdc0_parent,
};

static unsigned long mt8377_usb_rate(struct clk_hw *hw, unsigned long parent)
{
	struct mt8377_root_clk *clk = to_mt8377_root(hw);
	u32 mux;

	if (regmap_read(clk->map, 0x0c, &mux) || (mux & BIT(2)))
		return 0;
	return parent;
}

static const struct clk_ops mt8377_usb_ops = {
	.recalc_rate = mt8377_usb_rate,
};

static u8 mt8377_axi_parent(struct clk_hw *hw)
{
	struct mt8377_root_clk *clk = to_mt8377_root(hw);
	u32 mux;

	if (regmap_read(clk->map, 0, &mux))
		return 0;
	mux &= 3;
	return mux == 3 ? 1 : mux;
}

static unsigned long mt8377_axi_rate(struct clk_hw *hw, unsigned long parent)
{
	static const unsigned int divisors[] = { 1, 4, 5, 6 };
	struct mt8377_root_clk *clk = to_mt8377_root(hw);
	u32 divider;

	if (regmap_read(clk->map, 4, &divider))
		return 0;
	return parent / divisors[FIELD_GET(GENMASK(4, 3), divider)];
}

static const struct clk_ops mt8377_axi_ops = {
	.get_parent = mt8377_axi_parent,
	.recalc_rate = mt8377_axi_rate,
};

static const struct mtk_gate_regs mt8377_peri_regs = {
	.sta_ofs = 0x10,
	.set_ofs = 0x10,
	.clr_ofs = 0x10,
};

static const struct mtk_gate mt8377_peri_gates[] = {
	GATE_MTK(CLK_MT8377_PWM1, "mt8377_pwm1", NULL,
		 &mt8377_peri_regs, 2, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_PWM2, "mt8377_pwm2", NULL,
		 &mt8377_peri_regs, 3, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_PWM3, "mt8377_pwm3", NULL,
		 &mt8377_peri_regs, 4, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_PWM456, "mt8377_pwm456", NULL,
		 &mt8377_peri_regs, 5, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_PWM7, "mt8377_pwm7", NULL,
		 &mt8377_peri_regs, 6, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_MSDC0, "mt8377_msdc0", "mt8377_msdc0_sel",
		 &mt8377_peri_regs, 15, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_MSDC3, "mt8377_msdc3", "mt8377_msdc3_sel",
		 &mt8377_peri_regs, 18, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_USB, "mt8377_usb", "mt8377_usb_sel",
		 &mt8377_peri_regs, 9, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK_FLAGS(CLK_MT8377_I2C_DUAL, "mt8377_i2c_dual", NULL,
		       &mt8377_peri_regs, 30, &mtk_clk_gate_ops_no_setclr,
		       CLK_IS_CRITICAL),
	GATE_MTK(CLK_MT8377_I2C0, "mt8377_i2c0", NULL,
		 &mt8377_peri_regs, 28, &mtk_clk_gate_ops_no_setclr),
	GATE_MTK(CLK_MT8377_UART2, "mt8377_uart2", "mt8377_pwm_ref",
		 &mt8377_peri_regs, 26, &mtk_clk_gate_ops_no_setclr),
};

struct mt8377_peri {
	struct mtk_gate gates[ARRAY_SIZE(mt8377_peri_gates)];
	struct clk_hw_onecell_data *data;
};

static void mt8377_free_data(void *data)
{
	mtk_free_clk_data(data);
}

static void mt8377_unregister_gates(void *data)
{
	struct mt8377_peri *peri = data;

	mtk_clk_unregister_gates(peri->gates, ARRAY_SIZE(peri->gates), peri->data);
}

static int mt8377_peri_probe(struct platform_device *pdev)
{
	static const char * const msdc_names[] = { "mt8377_msdc0_sel", "mt8377_msdc3_sel" };
	const struct clk_parent_data parents[] = { { .index = 1 }, { .index = 2 } };
	struct clk_init_data init = {
		.ops = &mt8377_msdc0_ops,
		.parent_data = parents,
		.num_parents = ARRAY_SIZE(parents),
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *inputs;
	struct mt8377_peri *peri;
	struct mt8377_root_clk *mux;
	struct mt8377_root_clk *usb;
	struct clk_hw *reference;
	int i, ret;

	ret = devm_clk_bulk_get_all(dev, &inputs);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Missing peripheral clock inputs\n");
	if (ret != 4)
		return -EINVAL;
	peri = devm_kzalloc(dev, sizeof(*peri), GFP_KERNEL);
	if (!peri)
		return -ENOMEM;
	peri->data = mtk_alloc_clk_data(CLK_MT8377_PERI_NR_CLK);
	if (!peri->data)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, mt8377_free_data, peri->data);
	if (ret)
		return ret;
	reference = devm_clk_hw_register_fixed_factor_index(dev, "mt8377_pwm_ref", 0, 0, 1, 4);
	if (IS_ERR(reference))
		return PTR_ERR(reference);
	peri->data->hws[CLK_MT8377_PWM_REF] = reference;
	for (i = 0; i < ARRAY_SIZE(msdc_names); i++) {
		mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
		if (!mux)
			return -ENOMEM;
		mux->map = device_node_to_regmap(dev->of_node);
		if (IS_ERR(mux->map))
			return PTR_ERR(mux->map);
		mux->shift = i ? 7 : 0;
		init.name = msdc_names[i];
		mux->hw.init = &init;
		ret = devm_clk_hw_register(dev, &mux->hw);
		if (ret)
			return ret;
	}
	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;
	usb->map = mux->map;
	usb->hw.init = &(const struct clk_init_data) {
		.name = "mt8377_usb_sel",
		.ops = &mt8377_usb_ops,
		.parent_data = &(const struct clk_parent_data) { .index = 3 },
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	ret = devm_clk_hw_register(dev, &usb->hw);
	if (ret)
		return ret;
	memcpy(peri->gates, mt8377_peri_gates, sizeof(peri->gates));
	for (i = 0; i < ARRAY_SIZE(peri->gates); i++) {
		if (peri->gates[i].id >= CLK_MT8377_PWM1 &&
		    peri->gates[i].id <= CLK_MT8377_PWM7)
			peri->gates[i].parent_name = clk_hw_get_name(reference);
		else if (peri->gates[i].id == CLK_MT8377_I2C_DUAL ||
			 peri->gates[i].id == CLK_MT8377_I2C0)
			peri->gates[i].parent_name = __clk_get_name(inputs[0].clk);
	}
	ret = mtk_clk_register_gates(dev, dev->of_node, peri->gates,
				     ARRAY_SIZE(peri->gates), peri->data);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, mt8377_unregister_gates, peri);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, peri->data);
}

struct mt8377_root_data {
	const struct clk_ops *ops;
	unsigned int parents;
};

static const struct mt8377_root_data mt8377_mainpll_data = {
	.ops = &mt8377_mainpll_ops,
	.parents = 1,
};

static const struct mt8377_root_data mt8377_axi_data = {
	.ops = &mt8377_axi_ops,
	.parents = 3,
};

static const struct mt8377_root_data mt8377_peri_data;

static int mt8377_clk_probe(struct platform_device *pdev)
{
	const struct mt8377_root_data *data = of_device_get_match_data(&pdev->dev);
	struct clk_parent_data parents[] = { { .index = 0 }, { .index = 1 }, { .index = 2 } };
	struct clk_init_data init = {
		.name = dev_name(&pdev->dev),
		.parent_data = parents,
		.flags = CLK_IS_CRITICAL | CLK_GET_RATE_NOCACHE,
	};
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *provider;
	struct mt8377_root_clk *clk;
	void __iomem *base;
	unsigned int count = data == &mt8377_mainpll_data ? CLK_MT8377_APMIXED_NR_CLK : 1;
	int i, ret;

	if (!data->ops)
		return mt8377_peri_probe(pdev);
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
	provider = devm_kzalloc(dev, struct_size(provider, hws, count), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;
	provider->num = count;
	provider->hws[0] = &clk->hw;
	if (count > 1) {
		base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(base))
			return PTR_ERR(base);
		for (i = 1; i < count; i++) {
			provider->hws[i] = mt8377_register_wpll(pdev, base, i - 1);
			if (IS_ERR(provider->hws[i]))
				return PTR_ERR(provider->hws[i]);
		}
	}
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, provider);
}

static const struct of_device_id mt8377_clk_match[] = {
	{ .compatible = "mediatek,mt8377-apmixedsys", .data = &mt8377_mainpll_data },
	{ .compatible = "mediatek,mt6577-apmixedsys", .data = &mt8377_mainpll_data },
	{ .compatible = "mediatek,mt8377-infracfg", .data = &mt8377_axi_data },
	{ .compatible = "mediatek,mt6577-infracfg", .data = &mt8377_axi_data },
	{ .compatible = "mediatek,mt8377-pericfg", .data = &mt8377_peri_data },
	{ .compatible = "mediatek,mt6577-pericfg", .data = &mt8377_peri_data },
	{ }
};

static struct platform_driver mt8377_clk_driver = {
	.probe = mt8377_clk_probe,
	.driver = {
		.name = "mt8377-clock",
		.of_match_table = mt8377_clk_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mt8377_clk_driver);
