// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>

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

static struct sprd_clk_common *sc9830_aon_full_clks[] = {
	&adi_eb.common,
	&splk_eb.common,
	&sindrv.common,
	&twpll,
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

static struct sprd_clk_common *sc9830_ap_clks[] = {
	&i2c2_clk.common,
};

static struct clk_hw_onecell_data sc9830_ap_hws = {
	.hws = {
		[CLK_I2C2] = &i2c2_clk.common.hw,
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

static struct sprd_clk_common *sc9830_apapb_clks[] = {
	&i2c2_eb.common,
};

static struct clk_hw_onecell_data sc9830_apapb_hws = {
	.hws = {
		[CLK_I2C2_EB] = &i2c2_eb.common.hw,
	},
	.num = CLK_AP_APB_GATE_NUM,
};

static const struct sprd_clk_desc sc9830_apapb_desc = {
	.clk_clks = sc9830_apapb_clks,
	.num_clk_clks = ARRAY_SIZE(sc9830_apapb_clks),
	.hw_clks = &sc9830_apapb_hws,
};

static int sc9830_clk_probe(struct platform_device *pdev)
{
	const struct sprd_clk_desc *desc = device_get_match_data(&pdev->dev);
	int ret;

	ret = sprd_clk_regmap_init(pdev, desc);
	if (ret)
		return ret;

	return sprd_clk_probe(&pdev->dev, desc->hw_clks);
}

static const struct of_device_id sc9830_clk_match[] = {
	{ .compatible = "sprd,sc9830-aon-gate", .data = &sc9830_aon_desc },
	{ .compatible = "sprd,sc9830-aon-clk", .data = &sc9830_aon_full_desc },
	{ .compatible = "sprd,sc9830-ap-clk", .data = &sc9830_ap_desc },
	{ .compatible = "sprd,sc9830-apapb-gate", .data = &sc9830_apapb_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_clk_match);

static struct platform_driver sc9830_clk_driver = {
	.probe = sc9830_clk_probe,
	.driver = {
		.name = "sc9830-clock",
		.of_match_table = sc9830_clk_match,
	},
};
module_platform_driver(sc9830_clk_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 clock driver");
MODULE_LICENSE("GPL");
