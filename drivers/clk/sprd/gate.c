// SPDX-License-Identifier: GPL-2.0
//
// Spreadtrum gate clock driver
//
// Copyright (C) 2017 Spreadtrum, Inc.
// Author: Chunyan Zhang <chunyan.zhang@spreadtrum.com>

#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include "gate.h"

static int clk_gate_toggle(const struct sprd_gate *sg, bool en)
{
	const struct sprd_clk_common *common = &sg->common;
	unsigned int reg;
	int ret;
	bool set = sg->flags & CLK_GATE_SET_TO_DISABLE ? true : false;

	set ^= en;

	ret = regmap_read(common->regmap, common->reg, &reg);
	if (ret)
		return ret;

	if (set)
		reg |= sg->enable_mask;
	else
		reg &= ~sg->enable_mask;

	return regmap_write(common->regmap, common->reg, reg);
}

static int clk_sc_gate_toggle(const struct sprd_gate *sg, bool en)
{
	const struct sprd_clk_common *common = &sg->common;
	bool set = sg->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned int offset;

	set ^= en;

	/*
	 * Each set/clear gate clock has three registers:
	 * common->reg			- base register
	 * common->reg + offset		- set register
	 * common->reg + 2 * offset	- clear register
	 */
	offset = set ? sg->sc_offset : sg->sc_offset * 2;

	return regmap_write(common->regmap, common->reg + offset,
			    sg->enable_mask);
}

static void sprd_gate_disable(struct clk_hw *hw)
{
	struct sprd_gate *sg = hw_to_sprd_gate(hw);

	clk_gate_toggle(sg, false);
}

static int sprd_gate_enable(struct clk_hw *hw)
{
	struct sprd_gate *sg = hw_to_sprd_gate(hw);

	return clk_gate_toggle(sg, true);
}

static void sprd_sc_gate_disable(struct clk_hw *hw)
{
	struct sprd_gate *sg = hw_to_sprd_gate(hw);

	clk_sc_gate_toggle(sg, false);
}

static int sprd_sc_gate_enable(struct clk_hw *hw)
{
	struct sprd_gate *sg = hw_to_sprd_gate(hw);

	return clk_sc_gate_toggle(sg, true);
}

static int sprd_pll_sc_gate_prepare(struct clk_hw *hw)
{
	struct sprd_gate *sg = hw_to_sprd_gate(hw);
	int ret;

	ret = clk_sc_gate_toggle(sg, true);
	if (ret)
		return ret;
	udelay(sg->udelay);

	return 0;
}

static int sprd_gate_is_enabled(struct clk_hw *hw)
{
	struct sprd_gate *sg = hw_to_sprd_gate(hw);
	struct sprd_clk_common *common = &sg->common;
	struct clk_hw *parent;
	unsigned int reg;
	int ret;

	if (sg->flags & SPRD_GATE_NON_AON) {
		parent = clk_hw_get_parent(hw);
		if (!parent || !clk_hw_is_enabled(parent))
			return 0;
	}

	ret = regmap_read(common->regmap, common->reg, &reg);
	if (ret)
		return ret;

	if (sg->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= sg->enable_mask;

	reg &= sg->enable_mask;

	return reg ? 1 : 0;
}

const struct clk_ops sprd_gate_ops = {
	.disable	= sprd_gate_disable,
	.enable		= sprd_gate_enable,
	.is_enabled	= sprd_gate_is_enabled,
};
EXPORT_SYMBOL_GPL(sprd_gate_ops);

const struct clk_ops sprd_sc_gate_ops = {
	.disable	= sprd_sc_gate_disable,
	.enable		= sprd_sc_gate_enable,
	.is_enabled	= sprd_gate_is_enabled,
};
EXPORT_SYMBOL_GPL(sprd_sc_gate_ops);

const struct clk_ops sprd_pll_sc_gate_ops = {
	.unprepare	= sprd_sc_gate_disable,
	.prepare	= sprd_pll_sc_gate_prepare,
	.is_enabled	= sprd_gate_is_enabled,
};
EXPORT_SYMBOL_GPL(sprd_pll_sc_gate_ops);
