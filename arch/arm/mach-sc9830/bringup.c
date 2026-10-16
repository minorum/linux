// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/of_clk.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/sys_soc.h>

#include <asm/mach/arch.h>

static struct soc_device_attribute sc9830_soc = {
	.family = "Spreadtrum",
	.soc_id = "SC9830",
};

static void __init sc9830_init_machine(void)
{
	struct soc_device *soc;
	int ret;

	ret = soc_attr_read_machine(&sc9830_soc);
	if (!ret) {
		soc = soc_device_register(&sc9830_soc);
		if (IS_ERR(soc))
			pr_warn("SC9830 SoC registration failed: %pe\n", soc);
	}
	of_platform_default_populate(NULL, NULL, NULL);
}

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
	.init_machine = sc9830_init_machine,
MACHINE_END
