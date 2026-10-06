// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/sprd,sc9830-clk.h>

#include "common.h"
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
