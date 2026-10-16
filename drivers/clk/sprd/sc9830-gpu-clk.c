// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

struct sc9830_gpu_clk {
	struct clk_mux mux;
	struct clk_divider div;
	spinlock_t lock;
};

static int sc9830_gpu_clk_probe(struct platform_device *pdev)
{
	static const struct clk_parent_data parents[] = {
		{ .index = 0 }, { .index = 1 }, { .index = 2 },
		{ .index = 3 }, { .index = 4 }, { .index = 5 },
	};
	struct device *dev = &pdev->dev;
	struct sc9830_gpu_clk *gpu;
	struct clk_hw *hw;
	void __iomem *base;
	int ret;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	spin_lock_init(&gpu->lock);
	gpu->mux.reg = base + 4;
	gpu->mux.mask = 7;
	gpu->mux.lock = &gpu->lock;
	gpu->div.reg = base + 4;
	gpu->div.shift = 4;
	gpu->div.width = 2;
	gpu->div.lock = &gpu->lock;
	gpu->div.flags = CLK_DIVIDER_ROUND_CLOSEST;
	hw = devm_clk_hw_register_composite_pdata(dev, "sc9830-gpu-core", parents,
					       ARRAY_SIZE(parents), &gpu->mux.hw,
					       &clk_mux_ops, &gpu->div.hw,
					       &clk_divider_ops, NULL, NULL, 0);
	ret = PTR_ERR_OR_ZERO(hw);
	if (!ret)
		ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
	pm_runtime_put_sync(dev);
	return ret;
}

static const struct of_device_id sc9830_gpu_clk_match[] = {
	{ .compatible = "sprd,sc9830-gpu-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_gpu_clk_match);

static struct platform_driver sc9830_gpu_clk_driver = {
	.probe = sc9830_gpu_clk_probe,
	.driver = {
		.name = "sc9830-gpu-clk",
		.of_match_table = sc9830_gpu_clk_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sc9830_gpu_clk_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 GPU clock");
MODULE_LICENSE("GPL");
