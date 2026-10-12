// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_address.h>

#include <dt-bindings/clock/sprd,sc9830-clk.h>

#include "common.h"
#include "composite.h"
#include "gate.h"

static struct sprd_gate adi_eb = {
	.enable_mask = BIT(16),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-adi-eb",
					       &sprd_sc_gate_ops,
					       CLK_IGNORE_UNUSED),
};

static struct sprd_gate splk_eb = {
	.enable_mask = BIT(22),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-splk-eb",
					       &sprd_sc_gate_ops,
					       CLK_IGNORE_UNUSED),
};

static struct sprd_gate sindrv = {
	.enable_mask = BIT(0),
	.common.reg = 0x18,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-sindrv",
					       &sprd_gate_ops,
					       CLK_IS_CRITICAL),
};

static struct sprd_clk_common *sc9830_aon_clks[] = {
	&adi_eb.common,
	&splk_eb.common,
	&sindrv.common,
};

static struct clk_hw_onecell_data sc9830_aon_hws = {
	.hws = {
		[CLK_ADI_EB] = &adi_eb.common.hw,
		[CLK_SPLK_EB] = &splk_eb.common.hw,
		[CLK_SINDRV] = &sindrv.common.hw,
	},
	.num = CLK_AON_GATE_NUM,
};

static const struct sprd_clk_desc sc9830_aon_desc = {
	.clk_clks = sc9830_aon_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_aon_clks),
	.hw_clks = &sc9830_aon_hws,
};

static int sc9830_twpll_decode(u32 cfg1, u32 cfg2, unsigned long parent_rate,
			       unsigned long *rate)
{
	u64 value, fraction = 0;

	if (parent_rate != 26000000)
		return -EINVAL;
	if (!(cfg1 & BIT(26)) || FIELD_GET(GENMASK(19, 18), cfg1) != 3)
		return -EOPNOTSUPP;
	if (cfg1 & BIT(24))
		fraction = FIELD_GET(GENMASK(19, 0), cfg2);
	value = 26ULL * FIELD_GET(GENMASK(29, 24), cfg2) * 1000000;
	value += DIV_ROUND_CLOSEST_ULL(26ULL * fraction * 100, 1048576) * 10000;
	if (!value)
		return -EINVAL;
	*rate = value;
	return 0;
}

static int sc9830_twpll_read(struct sprd_clk_common *clock, u32 *cfg)
{
	u32 first[2], second[2];
	int attempt, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = regmap_bulk_read(clock->regmap, clock->reg, first,
				       ARRAY_SIZE(first));
		if (ret)
			return ret;
		ret = regmap_bulk_read(clock->regmap, clock->reg, second,
				       ARRAY_SIZE(second));
		if (ret)
			return ret;
		if (first[0] == second[0] && first[1] == second[1]) {
			cfg[0] = first[0];
			cfg[1] = first[1];
			return 0;
		}
	}
	return -EAGAIN;
}

static unsigned long sc9830_twpll_recalc(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct sprd_clk_common *clock = hw_to_sprd_clk_common(hw);
	unsigned long rate;
	u32 cfg[2];
	int ret;

	ret = sc9830_twpll_read(clock, cfg);
	if (!ret)
		ret = sc9830_twpll_decode(cfg[0], cfg[1], parent_rate, &rate);
	if (ret) {
		pr_debug_ratelimited("SC9830 TWPLL rate unavailable: %d\n", ret);
		return 0;
	}
	return rate;
}

static const struct clk_ops sc9830_twpll_ops = {
	.recalc_rate = sc9830_twpll_recalc,
};

static struct sprd_clk_common twpll = {
	.reg = 0x54,
	.hw.init = CLK_HW_INIT_FW_NAME("sc9830-twpll", "ext-26m",
				      &sc9830_twpll_ops, CLK_GET_RATE_NOCACHE),
};

static CLK_FIXED_FACTOR_HW(twpll_48m, "sc9830-twpll-48m", &twpll.hw,
			   32, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_51m2, "sc9830-twpll-51m2", &twpll.hw,
			   30, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_153m6, "sc9830-twpll-153m6", &twpll.hw,
			   10, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_96m, "sc9830-twpll-96m", &twpll.hw,
			   16, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_256m, "sc9830-twpll-256m", &twpll.hw,
			   6, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_307m2, "sc9830-twpll-307m2", &twpll.hw,
			   5, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_384m, "sc9830-twpll-384m", &twpll.hw,
			   4, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_76m8, "sc9830-twpll-76m8", &twpll.hw,
			   20, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_128m, "sc9830-twpll-128m", &twpll.hw,
			   12, 1, CLK_GET_RATE_NOCACHE);

static const struct clk_parent_data aux0_parents[] = {
	{ .fw_name = "ext-32k" },
	{ .fw_name = "ext-26m" },
	{ .fw_name = "ext-26m-rf1" },
	{ .fw_name = "ext-4m" },
	{ .fw_name = "dpll-42m" },
	{ .hw = &twpll_48m.hw },
	{ .fw_name = "mpll-56m" },
};

static SPRD_MUX_CLK_DATA(aux0_mux, "sc9830-aux0-mux", aux0_parents,
			 0x88, 0, 4, 0);

static struct sprd_gate aux0_eb = {
	.enable_mask = BIT(2),
	.sc_offset = 0x1000,
	.common.reg = 0x4,
	.common.hw.init = CLK_HW_INIT_HW("sc9830-aux0", &aux0_mux.common.hw,
				       &sprd_sc_gate_ops, CLK_SET_RATE_PARENT),
};

static struct sprd_gate aon_tmr_eb = {
	.enable_mask = BIT(11),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-aon-tmr-eb", "ext-32k",
					    &sprd_sc_gate_ops, 0),
};

static struct sprd_clk_common *sc9830_aon_full_clks[] = {
	&adi_eb.common,
	&splk_eb.common,
	&sindrv.common,
	&twpll,
	&aux0_mux.common,
	&aux0_eb.common,
	&aon_tmr_eb.common,
};

static struct clk_hw_onecell_data sc9830_aon_full_hws = {
	.hws = {
		[CLK_ADI_EB] = &adi_eb.common.hw,
		[CLK_SPLK_EB] = &splk_eb.common.hw,
		[CLK_SINDRV] = &sindrv.common.hw,
		[CLK_TWPLL] = &twpll.hw,
		[CLK_TWPLL_48M] = &twpll_48m.hw,
		[CLK_TWPLL_51M2] = &twpll_51m2.hw,
		[CLK_TWPLL_153M6] = &twpll_153m6.hw,
		[CLK_AUX0_MUX] = &aux0_mux.common.hw,
		[CLK_AUX0] = &aux0_eb.common.hw,
		[CLK_TWPLL_96M] = &twpll_96m.hw,
		[CLK_TWPLL_256M] = &twpll_256m.hw,
		[CLK_TWPLL_307M2] = &twpll_307m2.hw,
		[CLK_TWPLL_384M] = &twpll_384m.hw,
		[CLK_TWPLL_76M8] = &twpll_76m8.hw,
		[CLK_TWPLL_128M] = &twpll_128m.hw,
		[CLK_AON_TMR_EB] = &aon_tmr_eb.common.hw,
	},
	.num = CLK_AON_CLK_NUM,
};

static const struct sprd_clk_desc sc9830_aon_full_desc = {
	.clk_clks = sc9830_aon_full_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_aon_full_clks),
	.hw_clks = &sc9830_aon_full_hws,
};

static const struct clk_parent_data i2c2_parents[] = {
	{ .fw_name = "ext-26m" },
	{ .fw_name = "twpll-48m" },
	{ .fw_name = "twpll-51m2" },
	{ .fw_name = "twpll-153m6" },
};

static SPRD_COMP_CLK_DATA(i2c2_clk, "sc9830-i2c2", i2c2_parents,
			  0x70, 0, 2, 8, 3, 0);

static const struct clk_parent_data sdio0_parents[] = {
	{ .fw_name = "ext-26m" },
	{ .fw_name = "twpll-256m" },
	{ .fw_name = "twpll-307m2" },
	{ .fw_name = "twpll-384m" },
};

static SPRD_COMP_CLK_DATA(sdio0_clk, "sc9830-sdio0", sdio0_parents,
			  0x3c, 0, 2, 8, 3, 0);

static const struct clk_parent_data sdio1_parents[] = {
	{ .hw = &twpll_48m.hw },
	{ .hw = &twpll_76m8.hw },
	{ .hw = &twpll_96m.hw },
	{ .hw = &twpll_128m.hw },
};

static SPRD_MUX_CLK_DATA(sdio1_clk, "sc9830-sdio1", sdio1_parents,
			 0x40, 0, 2, 0);

static const struct clk_parent_data uart0_parents[] = {
	{ .fw_name = "ext-26m" },
	{ .fw_name = "twpll-48m" },
	{ .fw_name = "twpll-51m2" },
	{ .fw_name = "twpll-96m" },
};

static SPRD_COMP_CLK_DATA(uart0_clk, "sc9830-uart0", uart0_parents,
			  0x54, 0, 2, 8, 3, 0);

static struct sprd_clk_common *sc9830_ap_clks[] = {
	&i2c2_clk.common,
	&sdio0_clk.common,
	&uart0_clk.common,
	&sdio1_clk.common,
};

static struct clk_hw_onecell_data sc9830_ap_hws = {
	.hws = {
		[CLK_I2C2] = &i2c2_clk.common.hw,
		[CLK_SDIO0] = &sdio0_clk.common.hw,
		[CLK_UART0] = &uart0_clk.common.hw,
		[CLK_SDIO1] = &sdio1_clk.common.hw,
	},
	.num = CLK_AP_CLK_NUM,
};

static const struct sprd_clk_desc sc9830_ap_desc = {
	.clk_clks = sc9830_ap_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_ap_clks),
	.hw_clks = &sc9830_ap_hws,
};

static struct sprd_gate i2c2_eb = {
	.enable_mask = BIT(10),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-i2c2-eb", "i2c2",
					    &sprd_sc_gate_ops, 0),
};

static struct sprd_gate uart0_eb = {
	.enable_mask = BIT(13),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-uart0-eb", "uart0",
					    &sprd_sc_gate_ops, 0),
};

static struct sprd_clk_common *sc9830_apapb_clks[] = {
	&i2c2_eb.common,
	&uart0_eb.common,
};

static struct clk_hw_onecell_data sc9830_apapb_hws = {
	.hws = {
		[CLK_I2C2_EB] = &i2c2_eb.common.hw,
		[CLK_UART0_EB] = &uart0_eb.common.hw,
	},
	.num = CLK_AP_APB_GATE_NUM,
};

static const struct sprd_clk_desc sc9830_apapb_desc = {
	.clk_clks = sc9830_apapb_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_apapb_clks),
	.hw_clks = &sc9830_apapb_hws,
};

static struct sprd_gate sdio0_eb = {
	.enable_mask = BIT(8),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-sdio0-eb", "sdio0",
					    &sprd_sc_gate_ops, CLK_SET_RATE_PARENT),
};

static struct sprd_gate usb_eb = {
	.enable_mask = BIT(4),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-usb-eb",
					       &sprd_sc_gate_ops, CLK_IGNORE_UNUSED),
};

static struct sprd_gate sdio1_eb = {
	.enable_mask = BIT(9),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-sdio1-eb", "sdio1",
					    &sprd_sc_gate_ops, CLK_SET_RATE_PARENT),
};

static struct sprd_clk_common *sc9830_apahb_clks[] = {
	&sdio0_eb.common,
	&usb_eb.common,
	&sdio1_eb.common,
};

static struct clk_hw_onecell_data sc9830_apahb_hws = {
	.hws = {
		[CLK_SDIO0_EB] = &sdio0_eb.common.hw,
		[CLK_USB_EB] = &usb_eb.common.hw,
		[CLK_SDIO1_EB] = &sdio1_eb.common.hw,
	},
	.num = CLK_AP_AHB_GATE_NUM,
};

static const struct sprd_clk_desc sc9830_apahb_desc = {
	.clk_clks = sc9830_apahb_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_apahb_clks),
	.hw_clks = &sc9830_apahb_hws,
};

static int __init sc9830_clk_register(struct device_node *node,
				      const struct sprd_clk_desc *desc)
{
	struct regmap_config config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.fast_io = true,
	};
	struct resource res;
	struct regmap *regmap;
	void __iomem *base;
	unsigned int i;
	int ret;

	ret = of_address_to_resource(node, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < 4 || resource_size(&res) > U32_MAX)
		return -EINVAL;
	config.max_register = resource_size(&res) - 4;
	base = of_iomap(node, 0);
	if (!base)
		return -ENOMEM;
	regmap = regmap_init_mmio(NULL, base, &config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		goto unmap;
	}
	for (i = 0; i < desc->num_clk_clks; i++)
		if (desc->clk_clks[i])
			desc->clk_clks[i]->regmap = regmap;
	for (i = 0; i < desc->hw_clks->num; i++) {
		if (!desc->hw_clks->hws[i])
			continue;
		ret = of_clk_hw_register(node, desc->hw_clks->hws[i]);
		if (ret)
			goto unregister;
	}
	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, desc->hw_clks);
	if (!ret)
		return 0;
unregister:
	while (i--)
		if (desc->hw_clks->hws[i])
			clk_hw_unregister(desc->hw_clks->hws[i]);
	for (i = 0; i < desc->num_clk_clks; i++)
		if (desc->clk_clks[i])
			desc->clk_clks[i]->regmap = NULL;
	regmap_exit(regmap);
unmap:
	iounmap(base);
	return ret;
}

static const struct of_device_id sc9830_clk_match[] = {
	{ .compatible = "sprd,sc9830-aon-gate", .data = &sc9830_aon_desc },
	{ .compatible = "sprd,sc9830-aon-clk", .data = &sc9830_aon_full_desc },
	{ .compatible = "sprd,sc9830-ap-clk", .data = &sc9830_ap_desc },
	{ .compatible = "sprd,sc9830-apapb-gate", .data = &sc9830_apapb_desc },
	{ .compatible = "sprd,sc9830-apahb-gate", .data = &sc9830_apahb_desc },
	{ }
};
static void __init sc9830_clk_init(struct device_node *node)
{
	const struct of_device_id *match = of_match_node(sc9830_clk_match, node);
	int ret;

	ret = sc9830_clk_register(node, match->data);
	if (ret)
		pr_err("%pOF: clock registration failed: %d\n", node, ret);
}

CLK_OF_DECLARE(sc9830_aon_gate, "sprd,sc9830-aon-gate", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_aon_clk, "sprd,sc9830-aon-clk", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_ap_clk, "sprd,sc9830-ap-clk", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_apapb_gate, "sprd,sc9830-apapb-gate", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_apahb_gate, "sprd,sc9830-apahb-gate", sc9830_clk_init);

MODULE_DESCRIPTION("Spreadtrum SC9830 clock driver");
MODULE_LICENSE("GPL");
