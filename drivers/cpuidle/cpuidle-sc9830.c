// SPDX-License-Identifier: GPL-2.0-only
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/context_tracking.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/panic.h>
#include <linux/regmap.h>
#include <linux/smp.h>

#include <asm/cpuidle.h>
#include <asm/irqflags.h>

#include <trace/events/ipi.h>

#include "dt_idle_states.h"

#define MCU_PAUSE	0x3c
#define MCU_LIGHT_SLEEP	BIT(3)
#define MCU_SLEEP_MODES	GENMASK(3, 1)

struct sc9830_idle {
	struct regmap *map;
	struct clk *dap;
	bool armed;
	bool gated;
};

static int sc9830_idle_restore(struct sc9830_idle *state)
{
	u32 value;
	int ret = 0, error;

	if (state->armed) {
		ret = regmap_update_bits(state->map, MCU_PAUSE, MCU_LIGHT_SLEEP, 0);
		if (!ret)
			ret = regmap_read(state->map, MCU_PAUSE, &value);
		if (!ret && (value & MCU_SLEEP_MODES))
			ret = -EIO;
		if (!ret)
			state->armed = false;
	}
	if (state->gated) {
		error = clk_enable(state->dap);
		if (!error)
			state->gated = false;
		if (!ret)
			ret = error;
	}
	return ret;
}

static int sc9830_idle_prepare(struct sc9830_idle *state)
{
	u32 value;
	int ret, error;

	ret = regmap_read(state->map, MCU_PAUSE, &value);
	if (ret)
		return ret;
	if (value & MCU_SLEEP_MODES)
		return -EBUSY;

	clk_disable(state->dap);
	state->gated = true;
	if (__clk_is_enabled(state->dap)) {
		ret = -EBUSY;
		goto restore;
	}
	state->armed = true;
	ret = regmap_update_bits(state->map, MCU_PAUSE, MCU_LIGHT_SLEEP,
				 MCU_LIGHT_SLEEP);
	if (!ret)
		ret = regmap_read(state->map, MCU_PAUSE, &value);
	if (!ret && !(value & MCU_LIGHT_SLEEP))
		ret = -EIO;
	if (!ret)
		return 0;
restore:
	error = sc9830_idle_restore(state);
	if (error)
		panic("SC9830 idle restore failed: %d\n", error);
	return ret;
}

static atomic_t sc9830_idle_barrier;
static atomic_t sc9830_idle_waker;
static struct sc9830_idle sc9830_idle;
static int sc9830_idle_error;

static __cpuidle void sc9830_idle_wfi(void)
{
	ct_cpuidle_enter();
	cpu_do_idle();
	ct_cpuidle_exit();
}

static int sc9830_enter_light_sleep(struct cpuidle_device *dev,
				    struct cpuidle_driver *drv, int index)
{
	unsigned int cpu;
	unsigned long flags = arch_local_save_flags();
	int entered, error;

	local_fiq_disable();
	cpuidle_coupled_parallel_barrier(dev, &sc9830_idle_barrier);
	if (dev->cpu == cpumask_first(cpu_online_mask)) {
		sc9830_idle_error = sc9830_idle_prepare(&sc9830_idle);
		atomic_set(&sc9830_idle_waker, 0);
	}
	cpuidle_coupled_parallel_barrier(dev, &sc9830_idle_barrier);

	sc9830_idle_wfi();
	entered = sc9830_idle_error ? 0 : index;

	if (!atomic_cmpxchg(&sc9830_idle_waker, 0, 1)) {
		if (!sc9830_idle_error) {
			error = sc9830_idle_restore(&sc9830_idle);
			if (error)
				panic("SC9830 idle restore failed: %d\n", error);
		}
		for_each_online_cpu(cpu)
			if (cpu != dev->cpu)
				smp_send_reschedule(cpu);
	}
	cpuidle_coupled_parallel_barrier(dev, &sc9830_idle_barrier);

	if (!(flags & PSR_F_BIT))
		local_fiq_enable();
	return entered;
}

static struct cpuidle_driver sc9830_idle_driver = {
	.name = "sc9830_idle",
	.states = {
		ARM_CPUIDLE_WFI_STATE,
	},
	.safe_state_index = 0,
};

static const struct of_device_id sc9830_idle_states[] __initconst = {
	{ .compatible = "sprd,sc9830-light-sleep", .data = sc9830_enter_light_sleep },
	{ },
};

static int __init sc9830_idle_init(void)
{
	struct device_node *node;
	int ret;

	if (!of_machine_is_compatible("sprd,sc9830"))
		return -ENODEV;

	sc9830_idle_driver.cpumask = (struct cpumask *)cpu_possible_mask;
	ret = dt_init_idle_driver(&sc9830_idle_driver, sc9830_idle_states, 1);
	if (ret != 1)
		return ret < 0 ? ret : -ENODEV;
	if (sc9830_idle_driver.states[1].flags & CPUIDLE_FLAG_TIMER_STOP)
		return -EINVAL;
	sc9830_idle_driver.states[1].flags |= CPUIDLE_FLAG_COUPLED | CPUIDLE_FLAG_OFF;
	sc9830_idle_driver.states[1].enter_s2idle = NULL;

	sc9830_idle.map = syscon_regmap_lookup_by_compatible("sprd,sc9830-apahb-gate");
	if (IS_ERR(sc9830_idle.map))
		return PTR_ERR(sc9830_idle.map);
	if (regmap_might_sleep(sc9830_idle.map))
		return -EINVAL;

	node = of_cpu_device_node_get(0);
	if (!node)
		return -ENODEV;
	sc9830_idle.dap = of_clk_get_by_name(node, "dap");
	of_node_put(node);
	if (IS_ERR(sc9830_idle.dap))
		return PTR_ERR(sc9830_idle.dap);

	ret = clk_prepare_enable(sc9830_idle.dap);
	if (ret)
		goto put_clock;
	ret = cpuidle_register(&sc9830_idle_driver, cpu_possible_mask);
	if (!ret)
		return 0;
	clk_disable_unprepare(sc9830_idle.dap);
put_clock:
	clk_put(sc9830_idle.dap);
	return ret;
}
device_initcall(sc9830_idle_init);
