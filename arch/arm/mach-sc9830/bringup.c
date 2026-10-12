// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/of_clk.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>

static void __init sc9830_init_irq(void)
{
	of_clk_init(NULL);
	irqchip_init();
}

static void __init sc9830_init_time(void)
{
	timer_probe();
	tick_setup_hrtimer_broadcast();
}

static const char * const sc9830_dt_compat[] __initconst = {
	"sprd,sc9830",
	NULL,
};

DT_MACHINE_START(SC9830, "Spreadtrum SC9830")
	.dt_compat = sc9830_dt_compat,
	.init_irq = sc9830_init_irq,
	.init_time = sc9830_init_time,
MACHINE_END
