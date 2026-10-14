// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <dt-bindings/reset/sprd,sc9830-mm-reset.h>

struct sc9830_mm;

struct sc9830_mm_clock {
	struct clk_hw hw;
	struct sc9830_mm *mm;
	bool dcam;
};

struct sc9830_mm {
	struct device *dev;
	struct regmap *ahb;
	struct regmap *ckg;
	struct regmap *pmu;
	struct sc9830_mm_clock clocks[2];
	struct reset_controller_dev reset;
	unsigned long fault;
};

static void sc9830_mm_fault(struct sc9830_mm *mm)
{
	if (!test_and_set_bit(0, &mm->fault)) {
		pm_runtime_get_noresume(mm->dev);
		dev_err(mm->dev, "multimedia register access failed; power retained\n");
	}
}

static bool sc9830_mm_ready(struct sc9830_mm *mm)
{
	u32 state;
	unsigned int i;

	if (test_bit(0, &mm->fault) || !pm_runtime_active(mm->dev))
		return false;
	for (i = 0; i < 3; i++)
		if (regmap_read(mm->pmu, 0xbc, &state) || state & 0xf0000000)
			return false;
	return true;
}

static struct sc9830_mm_clock *to_mm_clock(struct clk_hw *hw)
{
	return container_of(hw, struct sc9830_mm_clock, hw);
}

static int sc9830_mm_prepare(struct clk_hw *hw)
{
	struct sc9830_mm_clock *clock = to_mm_clock(hw);
	struct sc9830_mm *mm = clock->mm;
	u32 mux;
	int ret;

	if (!sc9830_mm_ready(mm))
		return -EIO;
	if (!clock->dcam)
		return 0;
	ret = regmap_read(mm->ckg, 0x2c, &mux);
	if (ret)
		return ret;
	ret = regmap_update_bits(mm->ckg, 0x2c, 3, 3);
	if (ret) {
		regmap_update_bits(mm->ckg, 0x2c, 3, mux);
		sc9830_mm_fault(mm);
		return ret;
	}
	ret = regmap_write(mm->ahb, 0x1000, BIT(0));
	if (ret) {
		regmap_update_bits(mm->ckg, 0x2c, 3, mux);
		sc9830_mm_fault(mm);
	}
	return ret;
}

static void sc9830_mm_unprepare(struct clk_hw *hw)
{
	struct sc9830_mm_clock *clock = to_mm_clock(hw);

	if (!clock->dcam)
		return;
	if (!sc9830_mm_ready(clock->mm) ||
	    regmap_write(clock->mm->ahb, 0x2000, BIT(0)))
		sc9830_mm_fault(clock->mm);
}

static int sc9830_mm_enable(struct clk_hw *hw)
{
	struct sc9830_mm_clock *clock = to_mm_clock(hw);
	int ret;

	if (!sc9830_mm_ready(clock->mm)) {
		sc9830_mm_fault(clock->mm);
		return -EIO;
	}
	ret = regmap_write(clock->mm->ahb, clock->dcam ? 0x1008 : 0x1000,
			   clock->dcam ? BIT(3) : BIT(7));
	if (ret)
		sc9830_mm_fault(clock->mm);
	return ret;
}

static void sc9830_mm_disable(struct clk_hw *hw)
{
	struct sc9830_mm_clock *clock = to_mm_clock(hw);

	if (!sc9830_mm_ready(clock->mm) ||
	    regmap_write(clock->mm->ahb, clock->dcam ? 0x2008 : 0x2000,
			 clock->dcam ? BIT(3) : BIT(7)))
		sc9830_mm_fault(clock->mm);
}

static int sc9830_mm_is_enabled(struct clk_hw *hw)
{
	struct sc9830_mm_clock *clock = to_mm_clock(hw);
	u32 value;

	if (!sc9830_mm_ready(clock->mm) ||
	    regmap_read(clock->mm->ahb, clock->dcam ? 8 : 0, &value))
		return 0;
	return !!(value & (clock->dcam ? BIT(3) : BIT(7)));
}

static unsigned long sc9830_mm_rate(struct clk_hw *hw, unsigned long parent)
{
	struct sc9830_mm_clock *clock = to_mm_clock(hw);
	u32 value;

	if (!clock->dcam || !sc9830_mm_ready(clock->mm) ||
	    regmap_read(clock->mm->ckg, 0x2c, &value) || (value & 3) != 3)
		return 0;
	return parent;
}

static const struct clk_ops sc9830_mm_ops = {
	.prepare = sc9830_mm_prepare,
	.unprepare = sc9830_mm_unprepare,
	.enable = sc9830_mm_enable,
	.disable = sc9830_mm_disable,
	.is_enabled = sc9830_mm_is_enabled,
	.recalc_rate = sc9830_mm_rate,
};

static int sc9830_mm_reset_assert(struct reset_controller_dev *rc, unsigned long id)
{
	struct sc9830_mm *mm = container_of(rc, struct sc9830_mm, reset);
	u32 mask;
	int ret;

	if (id != RESET_MM_DCAM && id != RESET_MM_AXI &&
	    id != RESET_MM_ROT && id != RESET_MM_ALL)
		return -EINVAL;
	mask = id == RESET_MM_ALL ? BIT(0) | BIT(1) : BIT(id);
	if (!sc9830_mm_ready(mm))
		return -EIO;
	ret = regmap_update_bits(mm->ahb, 4, mask, mask);
	if (ret)
		sc9830_mm_fault(mm);
	return ret;
}

static int sc9830_mm_reset_deassert(struct reset_controller_dev *rc, unsigned long id)
{
	struct sc9830_mm *mm = container_of(rc, struct sc9830_mm, reset);
	u32 mask;
	int ret;

	if (id != RESET_MM_DCAM && id != RESET_MM_AXI &&
	    id != RESET_MM_ROT && id != RESET_MM_ALL)
		return -EINVAL;
	mask = id == RESET_MM_ALL ? BIT(0) | BIT(1) : BIT(id);
	if (!sc9830_mm_ready(mm))
		return -EIO;
	ret = regmap_update_bits(mm->ahb, 4, mask, 0);
	if (ret)
		sc9830_mm_fault(mm);
	return ret;
}

static int sc9830_mm_reset(struct reset_controller_dev *rc, unsigned long id)
{
	int ret = sc9830_mm_reset_assert(rc, id);

	return ret ? ret : sc9830_mm_reset_deassert(rc, id);
}

static const struct reset_control_ops sc9830_mm_reset_ops = {
	.assert = sc9830_mm_reset_assert,
	.deassert = sc9830_mm_reset_deassert,
	.reset = sc9830_mm_reset,
};

static int sc9830_mm_probe(struct platform_device *pdev)
{
	static const struct clk_parent_data parent = { .fw_name = "dcam-source" };
	static const char *const names[] = { "sc9830-dcam", "sc9830-mm-mmu" };
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct sc9830_mm *mm;
	unsigned int i;
	int ret;

	mm = devm_kzalloc(dev, sizeof(*mm), GFP_KERNEL);
	data = devm_kzalloc(dev, struct_size(data, hws, 2), GFP_KERNEL);
	if (!mm || !data)
		return -ENOMEM;
	mm->dev = dev;
	mm->ahb = device_node_to_regmap(dev->of_node);
	mm->ckg = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,mm-clk");
	mm->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,pmu");
	if (IS_ERR(mm->ahb))
		return PTR_ERR(mm->ahb);
	if (IS_ERR(mm->ckg))
		return PTR_ERR(mm->ckg);
	if (IS_ERR(mm->pmu))
		return PTR_ERR(mm->pmu);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	if (!sc9830_mm_ready(mm)) {
		ret = -EIO;
		goto put;
	}
	data->num = 2;
	for (i = 0; i < 2; i++) {
		struct clk_init_data init = {
			.name = names[i],
			.ops = &sc9830_mm_ops,
			.flags = CLK_GET_RATE_NOCACHE,
			.parent_data = i ? NULL : &parent,
			.num_parents = i ? 0 : 1,
		};

		mm->clocks[i].mm = mm;
		mm->clocks[i].dcam = !i;
		mm->clocks[i].hw.init = &init;
		ret = devm_clk_hw_register(dev, &mm->clocks[i].hw);
		if (ret)
			goto put;
		data->hws[i] = &mm->clocks[i].hw;
	}
	mm->reset.owner = THIS_MODULE;
	mm->reset.ops = &sc9830_mm_reset_ops;
	mm->reset.of_node = dev->of_node;
	mm->reset.nr_resets = RESET_MM_ALL + 1;
	ret = devm_reset_controller_register(dev, &mm->reset);
	if (!ret)
		ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
put:
	pm_runtime_put_sync(dev);
	return ret;
}

static const struct of_device_id sc9830_mm_match[] = {
	{ .compatible = "sprd,sc9830-mm-gate" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_mm_match);

static struct platform_driver sc9830_mm_driver = {
	.probe = sc9830_mm_probe,
	.driver = {
		.name = "sc9830-mm-clk",
		.of_match_table = sc9830_mm_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sc9830_mm_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 multimedia clocks and resets");
MODULE_LICENSE("GPL");
