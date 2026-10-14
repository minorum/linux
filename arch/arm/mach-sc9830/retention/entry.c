// SPDX-License-Identifier: GPL-2.0-only
#include <linux/irqflags.h>
#include <linux/smp.h>
#include <asm/irqflags.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>
#include <asm/virt.h>

#include "context.c"
#include "policy.c"
#include "gates.c"

struct sc9830_retention_entry {
	struct sc9830_retention_context context;
	struct sc9830_retention_policy policy;
	struct sc9830_retention_gates gates;
	void __iomem *status;
	phys_addr_t address;
};

static int sc9830_retention_finish(unsigned long address)
{
	soft_restart(address);
	return -EIO;
}

static void __noreturn sc9830_retention_recovery(void)
{
	local_irq_disable();
	asm volatile("cpsid aif" : : : "memory");
	for (;;)
		cpu_relax();
}

static int sc9830_retention_execute(struct sc9830_retention_entry *entry)
{
	struct sc9830_retention_context *ctx = &entry->context;
	unsigned long flags;
	u32 chip, status;
	int ret, restore;

	if (num_online_cpus() != 1 || raw_smp_processor_id() ||
	    IS_ENABLED(CONFIG_OUTER_CACHE) || IS_ENABLED(CONFIG_THUMB2_KERNEL) ||
	    is_hyp_mode_available() || is_hyp_mode_mismatched() ||
	    (arch_local_save_flags() & MODE_MASK) != SVC_MODE ||
	    entry->address != 0x400 || !entry->status ||
	    !ctx->ap || !ctx->ahb || !ctx->apb || !ctx->aon || !ctx->pub ||
	    !entry->policy.pmu || !entry->policy.dmc || !entry->policy.phy)
		return -EINVAL;
	if (ctx->saved || entry->policy.saved || entry->gates.saved)
		return -EBUSY;
	raw_local_irq_save(flags);
	asm volatile("cpsid aif" : : : "memory");
	chip = readl(ctx->aon + 0xfc);
	ctx->save_pub = chip && chip != 0x8300aaaa;
	ret = sc9830_retention_context_save(ctx);
	if (ret)
		goto out;
	ret = sc9830_retention_gates_apply(ctx, entry->policy.pmu, &entry->gates);
	if (ret) {
		if (entry->gates.saved)
			sc9830_retention_recovery();
		if (ret == -EBUSY)
			ctx->saved = false;
		goto out;
	}
	ret = sc9830_retention_policy_prepare(&entry->policy);
	if (ret) {
		if (entry->policy.saved) {
			sc9830_retention_gates_restore(ctx, entry->policy.pmu, &entry->gates);
			sc9830_retention_recovery();
		}
		goto restore;
	}
	writel(0, entry->status);
	if (readl(entry->status)) {
		ret = -EIO;
		goto restore;
	}
	ret = cpu_suspend(entry->address, sc9830_retention_finish);
	status = readl(entry->status);
	if (!ret) {
		if (status != 1 && status != 2)
			sc9830_retention_recovery();
		ret = status == 2 ? 0 : -EAGAIN;
	}
restore:
	restore = sc9830_retention_policy_restore(&entry->policy);
	restore |= sc9830_retention_gates_restore(ctx, entry->policy.pmu, &entry->gates);
	if (restore || entry->policy.saved || entry->gates.saved || ctx->saved)
		sc9830_retention_recovery();
out:
	if (!(flags & PSR_A_BIT))
		asm volatile("cpsie a" : : : "memory");
	raw_local_irq_restore(flags);
	return ret;
}
