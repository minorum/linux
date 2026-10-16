// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/barrier.h>
#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/system_misc.h>

#define CORE_RESET	0x08
#define CORE_JUMP_ENABLE	0x14
#define CORE_VECTOR(cpu)	(0x18 + 4 * (cpu))
#define CORE_STANDBY	0x3008
#define CORE_POWER(cpu)	(0x04 + 4 * (cpu))
#define POWER_STATUS	0xbc
#define POWER_AUTO	BIT(24)
#define POWER_FORCE	BIT(25)
#define POWER_DELAYS	0x40404

static void __iomem *sc9830_ahb;
static void __iomem *sc9830_pmu;

static int sc9830_core(unsigned int cpu)
{
	u32 affinity = cpu_logical_map(cpu);

	if ((affinity & ~0xffU) != 0xf00 ||
	    MPIDR_AFFINITY_LEVEL(affinity, 0) > 3)
		return -EINVAL;
	return MPIDR_AFFINITY_LEVEL(affinity, 0);
}

static void __init sc9830_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *node;
	struct resource resource;

	node = of_find_compatible_node(NULL, NULL, "sprd,sc9830-apahb-gate");
	if (!node)
		return;
	if (!of_address_to_resource(node, 0, &resource) &&
	    resource_size(&resource) >= CORE_STANDBY + sizeof(u32))
		sc9830_ahb = of_iomap(node, 0);
	of_node_put(node);
	if (!sc9830_ahb)
		return;
	node = of_find_compatible_node(NULL, NULL, "sprd,sc9830-pmu");
	if (node) {
		if (!of_address_to_resource(node, 0, &resource) &&
		    resource_size(&resource) >= POWER_STATUS + sizeof(u32))
			sc9830_pmu = of_iomap(node, 0);
		of_node_put(node);
	}
	if (!sc9830_pmu) {
		iounmap(sc9830_ahb);
		sc9830_ahb = NULL;
	}
}

static int sc9830_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void __iomem *power;
	int core = sc9830_core(cpu);
	u32 value;
	int attempt;

	if (core <= 0)
		return -EINVAL;
	if (!sc9830_ahb || !sc9830_pmu)
		return -ENODEV;
	power = sc9830_pmu + CORE_POWER(core);
	writel(__pa_symbol(secondary_startup_arm), sc9830_ahb + CORE_VECTOR(core));
	dsb(sy);
	value = readl(sc9830_ahb + CORE_JUMP_ENABLE);
	writel(value | BIT(core), sc9830_ahb + CORE_JUMP_ENABLE);
	value = readl(sc9830_ahb + CORE_RESET);
	writel(value | BIT(core), sc9830_ahb + CORE_RESET);
	writel(POWER_DELAYS, power);
	value = readl(power);
	writel((value & ~(POWER_AUTO | POWER_FORCE)) | POWER_AUTO, power);
	dmb(sy);
	udelay(1000);
	for (attempt = 0; attempt < 20; attempt++) {
		if (!(readl(power) & POWER_FORCE) &&
		    (readl(sc9830_ahb + CORE_RESET) & BIT(core)))
			break;
		value = readl(sc9830_ahb + CORE_RESET);
		writel(value | BIT(core), sc9830_ahb + CORE_RESET);
		value = readl(power);
		writel((value & ~(POWER_AUTO | POWER_FORCE)) | POWER_AUTO, power);
		dmb(sy);
		udelay(500);
	}
	if (attempt == 20)
		return -ETIMEDOUT;
	dsb(st);
	value = readl(sc9830_ahb + CORE_RESET);
	writel(value & ~BIT(core), sc9830_ahb + CORE_RESET);
	dsb(sy);
	sev();
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static int sc9830_cpu_disable(unsigned int cpu)
{
	return sc9830_core(cpu) > 0 ? 0 : -EPERM;
}

static void sc9830_cpu_die(unsigned int cpu)
{
	asm volatile("clrex" : : : "memory");
	v7_exit_coherency_flush(louis);
	for (;;)
		cpu_do_idle();
}

static int sc9830_cpu_kill(unsigned int cpu)
{
	int core = sc9830_core(cpu);
	void __iomem *power;
	u32 value;
	int attempt;

	if (core <= 0 || !sc9830_ahb || !sc9830_pmu)
		return 0;
	for (attempt = 0; attempt < 20; attempt++) {
		if (readl(sc9830_ahb + CORE_STANDBY) & BIT(core))
			break;
		udelay(100);
	}
	if (attempt == 20)
		return 0;
	power = sc9830_pmu + CORE_POWER(core);
	value = readl(power);
	writel((value & ~POWER_AUTO) | POWER_FORCE, power);
	dsb(st);
	for (attempt = 0; attempt < 20; attempt++) {
		value = readl(sc9830_pmu + POWER_STATUS);
		if (((value >> (4 * (core + 1))) & 0xf) == 7) {
			udelay(60);
			return 1;
		}
		udelay(60);
	}
	return 0;
}
#endif

static const struct smp_operations sc9830_smp_ops __initconst = {
	.smp_prepare_cpus = sc9830_smp_prepare_cpus,
	.smp_boot_secondary = sc9830_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable = sc9830_cpu_disable,
	.cpu_die = sc9830_cpu_die,
	.cpu_kill = sc9830_cpu_kill,
#endif
};

CPU_METHOD_OF_DECLARE(sc9830_smp, "sprd,sc9830-smp", &sc9830_smp_ops);
