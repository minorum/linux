// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clocksource.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/sched_clock.h>

#include "timer-of.h"

#define TIMER_LOAD	0x00
#define TIMER_CTL	0x08
#define TIMER_INT	0x0c
#define TIMER_SHADOW	0x10
#define TIMER_STRIDE	0x20
#define TIMER_PERIODIC	BIT(6)
#define TIMER_ENABLE	BIT(7)
#define TIMER_NEW	BIT(8)
#define TIMER_IRQ_ENABLE	BIT(0)
#define TIMER_IRQ_CLEAR	BIT(3)
#define TIMER_IRQ_BUSY	BIT(4)

static void __iomem *sc9830_source;

static u64 notrace sc9830_read_sched_clock(void)
{
	return ~readl_relaxed(sc9830_source);
}

static u64 sc9830_clocksource_read(struct clocksource *source)
{
	return sc9830_read_sched_clock();
}

static void sc9830_clocksource_suspend(struct clocksource *source)
{
	void __iomem *base = sc9830_source - TIMER_SHADOW;

	writel_relaxed(0, base + TIMER_CTL);
	readl(base + TIMER_CTL);
}

static void sc9830_clocksource_resume(struct clocksource *source)
{
	void __iomem *base = sc9830_source - TIMER_SHADOW;

	writel_relaxed(0, base + TIMER_CTL);
	writel_relaxed(TIMER_IRQ_CLEAR, base + TIMER_INT);
	writel_relaxed(U32_MAX, base + TIMER_LOAD);
	writel_relaxed(TIMER_PERIODIC | TIMER_NEW | TIMER_ENABLE,
		       base + TIMER_CTL);
	readl(base + TIMER_CTL);
}

static struct clocksource sc9830_clocksource = {
	.name = "sc9830-source",
	.rating = 200,
	.read = sc9830_clocksource_read,
	.suspend = sc9830_clocksource_suspend,
	.resume = sc9830_clocksource_resume,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static int sc9830_shutdown(struct clock_event_device *event)
{
	writel_relaxed(0, timer_of_base(to_timer_of(event)) + TIMER_CTL);
	return 0;
}

static void sc9830_event_resume(struct clock_event_device *event)
{
	void __iomem *base = timer_of_base(to_timer_of(event));

	writel_relaxed(0, base + TIMER_CTL);
	writel_relaxed(TIMER_IRQ_ENABLE | TIMER_IRQ_CLEAR, base + TIMER_INT);
	readl(base + TIMER_INT);
}

static int sc9830_next_event(unsigned long cycles,
			     struct clock_event_device *event)
{
	void __iomem *base = timer_of_base(to_timer_of(event));
	u32 status;
	int ret;

	ret = readl_poll_timeout_atomic(base + TIMER_INT, status,
					!(status & TIMER_IRQ_BUSY), 0, 1000);
	if (ret)
		return -ETIME;

	writel_relaxed(0, base + TIMER_CTL);
	writel_relaxed(cycles, base + TIMER_LOAD);
	writel_relaxed(TIMER_ENABLE, base + TIMER_CTL);
	return 0;
}

static irqreturn_t sc9830_interrupt(int irq, void *data)
{
	struct clock_event_device *event = data;
	void __iomem *base = timer_of_base(to_timer_of(event));

	writel_relaxed(TIMER_IRQ_ENABLE | TIMER_IRQ_CLEAR, base + TIMER_INT);
	sc9830_shutdown(event);
	event->event_handler(event);
	return IRQ_HANDLED;
}

static struct timer_of sc9830_timer[4];

static const struct timer_of sc9830_template = {
	.flags = TIMER_OF_BASE | TIMER_OF_CLOCK | TIMER_OF_IRQ,
	.clkevt = {
		.name = "sc9830-event",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_ONESHOT,
		.set_state_shutdown = sc9830_shutdown,
		.set_state_oneshot = sc9830_shutdown,
		.set_next_event = sc9830_next_event,
		.resume = sc9830_event_resume,
	},
	.of_irq = {
		.handler = sc9830_interrupt,
		.flags = IRQF_TIMER | IRQF_NOBALANCING | IRQF_NO_AUTOEN,
	},
};

static int sc9830_timer_starting(unsigned int cpu)
{
	struct timer_of *timer;
	int ret;

	if (cpu >= ARRAY_SIZE(sc9830_timer))
		panic("SC9830 timer CPU index out of range: %u", cpu);
	timer = &sc9830_timer[cpu];
	if (!timer_of_base(timer))
		panic("SC9830 timer missing for CPU%u", cpu);
	ret = irq_force_affinity(timer->clkevt.irq, cpumask_of(cpu));
	if (ret)
		panic("SC9830 timer affinity failed for CPU%u: %d", cpu, ret);
	sc9830_event_resume(&timer->clkevt);
	enable_irq(timer->clkevt.irq);
	clockevents_config_and_register(&timer->clkevt, timer_of_rate(timer),
					4, U32_MAX);
	return 0;
}

static int sc9830_timer_dying(unsigned int cpu)
{
	struct timer_of *timer = &sc9830_timer[cpu];
	void __iomem *base = timer_of_base(timer);

	sc9830_shutdown(&timer->clkevt);
	writel_relaxed(TIMER_IRQ_CLEAR, base + TIMER_INT);
	readl(base + TIMER_INT);
	disable_irq_nosync(timer->clkevt.irq);
	return 0;
}

static int __init sc9830_timer_init(struct device_node *node)
{
	void __iomem *base;
	unsigned int cpu;
	int ret, channel;

	if (nr_cpu_ids > ARRAY_SIZE(sc9830_timer))
		return -EINVAL;
	for_each_possible_cpu(cpu) {
		struct timer_of *timer = &sc9830_timer[cpu];

		*timer = sc9830_template;
		timer->of_base.index = cpu;
		timer->of_clk.index = cpu;
		timer->of_irq.index = cpu;
		timer->clkevt.cpumask = cpumask_of(cpu);
		ret = timer_of_init(node, timer);
		if (ret)
			goto cleanup;
		base = timer_of_base(timer);
		for (channel = 0; channel < 3; channel++) {
			writel_relaxed(0, base + channel * TIMER_STRIDE + TIMER_CTL);
			writel_relaxed(TIMER_IRQ_CLEAR,
				       base + channel * TIMER_STRIDE + TIMER_INT);
		}
	}
	base = timer_of_base(&sc9830_timer[0]);
	sc9830_source = base + TIMER_STRIDE + TIMER_SHADOW;
	sc9830_clocksource_resume(&sc9830_clocksource);
	ret = clocksource_register_hz(&sc9830_clocksource,
				      timer_of_rate(&sc9830_timer[0]));
	if (ret) {
		writel_relaxed(0, base + TIMER_STRIDE + TIMER_CTL);
		sc9830_source = NULL;
		goto cleanup;
	}
	sched_clock_register(sc9830_read_sched_clock, 32,
			     timer_of_rate(&sc9830_timer[0]));
	ret = cpuhp_setup_state(CPUHP_AP_SC9830_TIMER_STARTING,
				"clockevents/sc9830:starting",
				sc9830_timer_starting, sc9830_timer_dying);
	if (ret)
		panic("SC9830 timer hotplug setup failed: %d", ret);
	return 0;

cleanup:
	while (cpu--)
		timer_of_cleanup(&sc9830_timer[cpu]);
	return ret;
}

TIMER_OF_DECLARE(sc9830_timer, "sprd,sc9830-timer", sc9830_timer_init);
