// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/io.h>

struct sc9830_retention_gates {
	u32 mcu;
	u32 force;
	u32 automatic;
	u32 core;
	bool saved;
};

static int sc9830_sleep_write(void __iomem *address, u32 value)
{
	writel(value, address);
	return readl(address) == value ? 0 : -EIO;
}

static int sc9830_retention_gates_restore(struct sc9830_retention_context *ctx,
					  void __iomem *pmu,
					  struct sc9830_retention_gates *state)
{
	int ret = 0;

	if (!state->saved)
		return sc9830_retention_context_restore(ctx);
	ret |= sc9830_sleep_write(pmu + 0xe8, state->core);
	ret |= sc9830_sleep_write(ctx->ahb + 0x3c, state->mcu);
	ret |= sc9830_retention_context_restore(ctx);
	ret |= sc9830_sleep_write(ctx->ahb + 0x0c, state->force);
	ret |= sc9830_sleep_write(ctx->ahb + 0x10, state->automatic);
	if (!ret)
		state->saved = false;
	return ret;
}

static int sc9830_retention_gates_apply(struct sc9830_retention_context *ctx,
					void __iomem *pmu,
					struct sc9830_retention_gates *state)
{
	u32 value;
	int ret;

	if (!ctx->saved || state->saved)
		return -EBUSY;
	if (readl(ctx->ahb) & BIT(5))
		return -EBUSY;
	if (!(readl(pmu) & BIT(24)) || !(readl(pmu + 4) & BIT(24)) ||
	    (readl(pmu + 0x1c) & BIT(24)))
		return -EBUSY;
	state->mcu = readl(ctx->ahb + 0x3c);
	state->force = readl(ctx->ahb + 0x0c);
	state->automatic = readl(ctx->ahb + 0x10);
	state->core = readl(pmu + 0xe8);
	if ((state->mcu & 0x0c) || state->core != 1)
		return -EBUSY;
	state->saved = true;
	ret = sc9830_sleep_write(ctx->ahb + 0x3c, state->mcu | 0x0c);
	if (ret)
		goto rollback;
	value = readl(ctx->ahb) & ~0x1fffU;
	ret = sc9830_sleep_write(ctx->ahb, value);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(ctx->ahb + 0x0c, state->force | 2);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(ctx->ahb + 0x3c, state->mcu | 0x1c);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(ctx->ahb + 0x10, state->automatic | 0x3b);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(ctx->aon, readl(ctx->aon) & ~0x1800U);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(ctx->aon + 4, readl(ctx->aon + 4) & ~0xe00U);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(ctx->apb, readl(ctx->apb) & 0x00786000);
	if (ret)
		goto rollback;
	ret = sc9830_sleep_write(pmu + 0xe8, 0);
	if (!ret)
		return 0;
rollback:
	sc9830_retention_gates_restore(ctx, pmu, state);
	return -EIO;
}
