// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/reset/reset-simple.h>

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
		pr_debug_ratelimited("SC9830 PLL rate unavailable: %d\n", ret);
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

static struct sprd_clk_common mpll = {
	.reg = 0x44,
	.hw.init = CLK_HW_INIT_FW_NAME("sc9830-mpll", "ext-26m",
				      &sc9830_twpll_ops, CLK_GET_RATE_NOCACHE),
};

static CLK_FIXED_FACTOR_HW(mpll_32, "sc9830-mpll-div32", &mpll.hw,
			   32, 1, CLK_GET_RATE_NOCACHE);

static int sc9830_dpll_decode(u32 cfg1, u32 cfg2, unsigned long parent_rate,
			      unsigned long *rate)
{
	static const u32 reference_mhz[] = { 2, 4, 13, 26 };
	u32 reference = reference_mhz[FIELD_GET(GENMASK(19, 18), cfg1)];
	u64 value;

	if (parent_rate != 26000000)
		return -EINVAL;
	if (cfg1 & BIT(26)) {
		value = (u64)reference * FIELD_GET(GENMASK(29, 24), cfg2) * 1000000;
		if (cfg1 & BIT(24))
			value += DIV_ROUND_CLOSEST_ULL((u64)reference *
				FIELD_GET(GENMASK(19, 0), cfg2) * 1000000, 1048576);
	} else {
		value = (u64)reference * FIELD_GET(GENMASK(10, 0), cfg1) * 1000000;
	}
	if (!value)
		return -EINVAL;
	if (value > ULONG_MAX)
		return -ERANGE;
	*rate = value;
	return 0;
}

static unsigned long sc9830_dpll_recalc(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct sprd_clk_common *clock = hw_to_sprd_clk_common(hw);
	unsigned long rate;
	u32 cfg[2];
	int ret;

	ret = sc9830_twpll_read(clock, cfg);
	if (!ret)
		ret = sc9830_dpll_decode(cfg[0], cfg[1], parent_rate, &rate);
	if (ret) {
		pr_debug_ratelimited("SC9830 DPLL rate unavailable: %d\n", ret);
		return 0;
	}
	return rate;
}

static const struct clk_ops sc9830_dpll_ops = {
	.recalc_rate = sc9830_dpll_recalc,
};

static struct sprd_clk_common dpll = {
	.reg = 0x4c,
	.hw.init = CLK_HW_INIT_FW_NAME("sc9830-dpll", "ext-26m",
				      &sc9830_dpll_ops, CLK_GET_RATE_NOCACHE),
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
static CLK_FIXED_FACTOR_HW(twpll_192m, "sc9830-twpll-192m", &twpll.hw,
			   8, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_512m, "sc9830-twpll-512m", &twpll.hw,
			   3, 1, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_614m4, "sc9830-twpll-614m4", &twpll.hw,
			   5, 2, CLK_GET_RATE_NOCACHE);
static CLK_FIXED_FACTOR_HW(twpll_768m, "sc9830-twpll-768m", &twpll.hw,
			   2, 1, CLK_GET_RATE_NOCACHE);

static const struct clk_parent_data aux0_parents[] = {
	{ .fw_name = "ext-32k" },
	{ .fw_name = "ext-26m" },
	{ .fw_name = "ext-26m-rf1" },
	{ .fw_name = "ext-4m" },
	{ .fw_name = "dpll-42m" },
	{ .hw = &twpll_48m.hw },
	{ .hw = &mpll_32.hw },
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

static struct sprd_gate aon_intc_eb = {
	.enable_mask = BIT(15),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-aon-intc-eb",
					       &sprd_sc_gate_ops, 0),
};

static struct sprd_gate ap_tmr0_eb = {
	.enable_mask = BIT(12),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-ap-tmr0-eb", "ext-32k",
					    &sprd_sc_gate_ops, 0),
};

static struct sprd_gate ap_tmr1_eb = {
	.enable_mask = BIT(9),
	.sc_offset = 0x1000,
	.common.reg = 0x4,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-ap-tmr1-eb", "ext-32k",
					    &sprd_sc_gate_ops, 0),
};

static struct sprd_gate ap_tmr2_eb = {
	.enable_mask = BIT(10),
	.sc_offset = 0x1000,
	.common.reg = 0x4,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-ap-tmr2-eb", "ext-32k",
					    &sprd_sc_gate_ops, 0),
};

static int sc9830_mbox_is_enabled(struct clk_hw *hw)
{
	return sprd_sc_gate_ops.is_enabled(hw) > 0;
}

static const struct clk_ops sc9830_mbox_ops = {
	.is_enabled = sc9830_mbox_is_enabled,
};

static struct sprd_gate mbox_eb = {
	.enable_mask = BIT(21),
	.common.reg = 0x4,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-mbox-eb",
					       &sc9830_mbox_ops,
					       CLK_IS_CRITICAL),
};

static struct sprd_clk_common *sc9830_aon_full_clks[] = {
	&adi_eb.common,
	&splk_eb.common,
	&sindrv.common,
	&twpll,
	&mpll,
	&dpll,
	&aux0_mux.common,
	&aux0_eb.common,
	&aon_tmr_eb.common,
	&ap_tmr0_eb.common,
	&ap_tmr1_eb.common,
	&ap_tmr2_eb.common,
	&aon_intc_eb.common,
	&mbox_eb.common,
};

static struct clk_hw_onecell_data sc9830_aon_full_hws = {
	.hws = {
		[CLK_ADI_EB] = &adi_eb.common.hw,
		[CLK_SPLK_EB] = &splk_eb.common.hw,
		[CLK_SINDRV] = &sindrv.common.hw,
		[CLK_TWPLL] = &twpll.hw,
		[CLK_MPLL] = &mpll.hw,
		[CLK_MPLL_DIV32] = &mpll_32.hw,
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
		[CLK_AP_TMR0_EB] = &ap_tmr0_eb.common.hw,
		[CLK_AP_TMR1_EB] = &ap_tmr1_eb.common.hw,
		[CLK_AP_TMR2_EB] = &ap_tmr2_eb.common.hw,
		[CLK_AON_INTC_EB] = &aon_intc_eb.common.hw,
		[CLK_DPLL] = &dpll.hw,
		[CLK_TWPLL_192M] = &twpll_192m.hw,
		[CLK_TWPLL_512M] = &twpll_512m.hw,
		[CLK_TWPLL_614M4] = &twpll_614m4.hw,
		[CLK_TWPLL_768M] = &twpll_768m.hw,
		[CLK_MBOX_EB] = &mbox_eb.common.hw,
	},
	.num = CLK_AON_CLK_NUM,
};

static const struct sprd_clk_desc sc9830_aon_full_desc = {
	.clk_clks = sc9830_aon_full_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_aon_full_clks),
	.hw_clks = &sc9830_aon_full_hws,
};

static const struct clk_parent_data emc_parents[] = {
	{ .fw_name = "ext-26m" },
	{ .fw_name = "twpll-192m" },
	{ .fw_name = "twpll-307m2" },
	{ .fw_name = "twpll-384m" },
	{ .fw_name = "twpll-512m" },
	{ .fw_name = "twpll-614m4" },
	{ .fw_name = "twpll-768m" },
	{ .fw_name = "dpll" },
};

static u8 sc9830_emc_get_parent(struct clk_hw *hw)
{
	struct sprd_clk_common *clock = hw_to_sprd_clk_common(hw);
	u32 cfg;

	if (regmap_read(clock->regmap, clock->reg, &cfg))
		return 0;
	return FIELD_GET(GENMASK(2, 0), cfg);
}

static unsigned long sc9830_emc_recalc(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	static const u8 divisors[] = { 1, 8, 5, 4, 3, 5, 2, 1 };
	struct sprd_clk_common *clock = hw_to_sprd_clk_common(hw);
	unsigned long rate;
	u32 first, second, selector;
	int attempt;

	for (attempt = 0; attempt < 3; attempt++) {
		if (regmap_read(clock->regmap, clock->reg, &first))
			return 0;
		selector = FIELD_GET(GENMASK(2, 0), first);
		if (!selector) {
			rate = 26000000;
		} else if (selector == 7) {
			if (!dpll.regmap)
				return 0;
			rate = sc9830_dpll_recalc(&dpll.hw, 26000000);
		} else {
			if (!twpll.regmap)
				return 0;
			rate = sc9830_twpll_recalc(&twpll.hw, 26000000);
			rate = div_u64((u64)rate * (selector == 5 ? 2 : 1),
				       divisors[selector]);
		}
		if (regmap_read(clock->regmap, clock->reg, &second))
			return 0;
		if (first == second)
			return rate / (2 * (FIELD_GET(GENMASK(10, 8), first) + 1));
	}
	return 0;
}

static const struct clk_ops sc9830_emc_ops = {
	.get_parent = sc9830_emc_get_parent,
	.recalc_rate = sc9830_emc_recalc,
};

static struct sprd_clk_common emc_clk = {
	.reg = 0x58,
	.hw.init = CLK_HW_INIT_PARENTS_DATA("sc9830-emc", emc_parents,
					  &sc9830_emc_ops, CLK_GET_RATE_NOCACHE),
};

static struct sprd_clk_common *sc9830_aon_mux_clks[] = { &emc_clk };

static struct clk_hw_onecell_data sc9830_aon_mux_hws = {
	.hws = { [CLK_EMC] = &emc_clk.hw },
	.num = CLK_AON_MUX_NUM,
};

static const struct sprd_clk_desc sc9830_aon_mux_desc = {
	.clk_clks = sc9830_aon_mux_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_aon_mux_clks),
	.hw_clks = &sc9830_aon_mux_hws,
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

static SPRD_MUX_CLK_DATA(emmc_clk, "sc9830-emmc", sdio0_parents,
			 0x48, 0, 2, 0);

static struct sprd_clk_common *sc9830_ap_clks[] = {
	&i2c2_clk.common,
	&sdio0_clk.common,
	&uart0_clk.common,
	&sdio1_clk.common,
	&emmc_clk.common,
};

static struct clk_hw_onecell_data sc9830_ap_hws = {
	.hws = {
		[CLK_I2C2] = &i2c2_clk.common.hw,
		[CLK_SDIO0] = &sdio0_clk.common.hw,
		[CLK_UART0] = &uart0_clk.common.hw,
		[CLK_SDIO1] = &sdio1_clk.common.hw,
		[CLK_EMMC] = &emmc_clk.common.hw,
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

static struct sprd_gate intc0_eb = {
	.enable_mask = BIT(19),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-intc0-eb",
					       &sprd_sc_gate_ops, 0),
};

static struct sprd_gate intc1_eb = {
	.enable_mask = BIT(20),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-intc1-eb",
					       &sprd_sc_gate_ops, 0),
};

static struct sprd_gate intc2_eb = {
	.enable_mask = BIT(21),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-intc2-eb",
					       &sprd_sc_gate_ops, 0),
};

static struct sprd_gate intc3_eb = {
	.enable_mask = BIT(22),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_NO_PARENT("sc9830-intc3-eb",
					       &sprd_sc_gate_ops, 0),
};

static struct sprd_clk_common *sc9830_apapb_clks[] = {
	&i2c2_eb.common,
	&uart0_eb.common,
	&intc0_eb.common,
	&intc1_eb.common,
	&intc2_eb.common,
	&intc3_eb.common,
};

static struct clk_hw_onecell_data sc9830_apapb_hws = {
	.hws = {
		[CLK_I2C2_EB] = &i2c2_eb.common.hw,
		[CLK_UART0_EB] = &uart0_eb.common.hw,
		[CLK_INTC0_EB] = &intc0_eb.common.hw,
		[CLK_INTC1_EB] = &intc1_eb.common.hw,
		[CLK_INTC2_EB] = &intc2_eb.common.hw,
		[CLK_INTC3_EB] = &intc3_eb.common.hw,
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

static struct sprd_gate emmc_eb = {
	.enable_mask = BIT(11),
	.sc_offset = 0x1000,
	.common.hw.init = CLK_HW_INIT_FW_NAME("sc9830-emmc-eb", "emmc",
					    &sprd_sc_gate_ops, CLK_SET_RATE_PARENT),
};

static struct sprd_clk_common *sc9830_apahb_clks[] = {
	&sdio0_eb.common,
	&usb_eb.common,
	&sdio1_eb.common,
	&emmc_eb.common,
};

static struct clk_hw_onecell_data sc9830_apahb_hws = {
	.hws = {
		[CLK_SDIO0_EB] = &sdio0_eb.common.hw,
		[CLK_USB_EB] = &usb_eb.common.hw,
		[CLK_SDIO1_EB] = &sdio1_eb.common.hw,
		[CLK_EMMC_EB] = &emmc_eb.common.hw,
	},
	.num = CLK_AP_AHB_GATE_NUM,
};

static const struct sprd_clk_desc sc9830_apahb_desc = {
	.clk_clks = sc9830_apahb_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_apahb_clks),
	.hw_clks = &sc9830_apahb_hws,
};

static struct reset_simple_data sc9830_apahb_reset = {
	.rcdev = {
		.owner = THIS_MODULE,
		.nr_resets = 32,
		.ops = &reset_simple_ops,
	},
	.reset_us = 10,
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
	bool reset = desc == &sc9830_apahb_desc &&
		of_property_present(node, "#reset-cells");
	int ret;

	ret = of_address_to_resource(node, 0, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < (reset ? 8 : 4) ||
	    resource_size(&res) > U32_MAX)
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
	if (reset) {
		sc9830_apahb_reset.membase = base + 4;
		sc9830_apahb_reset.rcdev.of_node = node;
		spin_lock_init(&sc9830_apahb_reset.lock);
		ret = reset_controller_register(&sc9830_apahb_reset.rcdev);
		if (ret)
			goto unregister;
	}
	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, desc->hw_clks);
	if (!ret)
		return 0;
	if (reset)
		reset_controller_unregister(&sc9830_apahb_reset.rcdev);
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
	{ .compatible = "sprd,sc9830-aon-mux-clk", .data = &sc9830_aon_mux_desc },
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
CLK_OF_DECLARE(sc9830_aon_mux_clk, "sprd,sc9830-aon-mux-clk", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_ap_clk, "sprd,sc9830-ap-clk", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_apapb_gate, "sprd,sc9830-apapb-gate", sc9830_clk_init);
CLK_OF_DECLARE(sc9830_apahb_gate, "sprd,sc9830-apahb-gate", sc9830_clk_init);

MODULE_DESCRIPTION("Spreadtrum SC9830 clock driver");
MODULE_LICENSE("GPL");
