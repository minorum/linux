// SPDX-License-Identifier: GPL-2.0-only
#include <dt-bindings/clock/mediatek,mt8377-mmsys.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mt8377_mm0_regs = {
	.sta_ofs = 0x00,
	.set_ofs = 0x10,
	.clr_ofs = 0x20,
};

static const struct mtk_gate_regs mt8377_mm2_regs = {
	.sta_ofs = 0x08,
	.set_ofs = 0x18,
	.clr_ofs = 0x28,
};

static const struct mtk_gate mt8377_mm_gates[] = {
	GATE_MTK_FLAGS(CLK_MT8377_MM_DPI, "mt8377_mm_dpi", NULL,
		       &mt8377_mm2_regs, 4, &mtk_clk_gate_ops_setclr,
		       CLK_IGNORE_UNUSED),
	GATE_MTK_FLAGS(CLK_MT8377_MM_LARB1, "mt8377_mm_larb1", NULL,
		       &mt8377_mm2_regs, 7, &mtk_clk_gate_ops_setclr,
		       CLK_IGNORE_UNUSED),
	GATE_MTK_FLAGS(CLK_MT8377_MM_LARB1_EMI, "mt8377_mm_larb1_emi", NULL,
		       &mt8377_mm0_regs, 7, &mtk_clk_gate_ops_setclr,
		       CLK_IGNORE_UNUSED),
};

struct mt8377_mm {
	struct mtk_gate gates[CLK_MT8377_MM_NR_CLK];
	struct clk_hw_onecell_data *data;
};

static void mt8377_mm_free_data(void *data)
{
	mtk_free_clk_data(data);
}

static void mt8377_mm_unregister(void *data)
{
	struct mt8377_mm *mm = data;

	mtk_clk_unregister_gates(mm->gates, ARRAY_SIZE(mm->gates), mm->data);
}

static int mt8377_mm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt8377_mm *mm;
	int ret;

	mm = devm_kzalloc(dev, sizeof(*mm), GFP_KERNEL);
	if (!mm)
		return -ENOMEM;
	memcpy(mm->gates, mt8377_mm_gates, sizeof(mm->gates));
	mm->data = mtk_alloc_clk_data(ARRAY_SIZE(mm->gates));
	if (!mm->data)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, mt8377_mm_free_data, mm->data);
	if (ret)
		return ret;
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	ret = mtk_clk_register_gates(dev, dev->of_node, mm->gates,
				     ARRAY_SIZE(mm->gates), mm->data);
	if (ret)
		goto put_runtime;
	ret = devm_add_action_or_reset(dev, mt8377_mm_unregister, mm);
	if (ret)
		goto put_runtime;
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, mm->data);
put_runtime:
	pm_runtime_put(dev);
	return ret;
}

static const struct of_device_id mt8377_mm_match[] = {
	{ .compatible = "mediatek,mt8377-mmsys" },
	{ .compatible = "mediatek,mt6577-mmsys" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8377_mm_match);

static struct platform_driver mt8377_mm_driver = {
	.probe = mt8377_mm_probe,
	.driver = {
		.name = "mt8377-mmsys-clock",
		.of_match_table = mt8377_mm_match,
	},
};
module_platform_driver(mt8377_mm_driver);

MODULE_DESCRIPTION("MediaTek MT8377 display clock gates");
MODULE_LICENSE("GPL");
