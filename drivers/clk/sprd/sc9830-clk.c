// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/sprd,sc9830-clk.h>

#include "common.h"
#include "gate.h"

static SPRD_SC_GATE_CLK(adi_eb, "sc9830-adi-eb", NULL, 0, 0x1000,
			BIT(16), CLK_IGNORE_UNUSED, 0);
static SPRD_SC_GATE_CLK(splk_eb, "sc9830-splk-eb", NULL, 0, 0x1000,
			BIT(22), CLK_IGNORE_UNUSED, 0);

static struct sprd_clk_common *sc9830_aon_clks[] = {
	&adi_eb.common,
	&splk_eb.common,
};

static struct clk_hw_onecell_data sc9830_aon_hws = {
	.hws = {
		[CLK_ADI_EB] = &adi_eb.common.hw,
		[CLK_SPLK_EB] = &splk_eb.common.hw,
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
