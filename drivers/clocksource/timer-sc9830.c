// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clocksource.h>
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

static int sc9830_shutdown(struct clock_event_device *event)
{
	writel_relaxed(0, timer_of_base(to_timer_of(event)) + TIMER_CTL);
	return 0;
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

static struct timer_of sc9830_timer = {
	.flags = TIMER_OF_BASE | TIMER_OF_CLOCK | TIMER_OF_IRQ,
	.clkevt = {
		.name = "sc9830-event",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_ONESHOT,
		.set_state_shutdown = sc9830_shutdown,
		.set_state_oneshot = sc9830_shutdown,
		.set_next_event = sc9830_next_event,
		.cpumask = cpu_possible_mask,
	},
	.of_irq = {
		.handler = sc9830_interrupt,
		.flags = IRQF_TIMER,
	},
};

static int __init sc9830_timer_init(struct device_node *node)
{
	void __iomem *base;
	int ret, channel;

	ret = timer_of_init(node, &sc9830_timer);
	if (ret)
		return ret;
	base = timer_of_base(&sc9830_timer);
	for (channel = 0; channel < 3; channel++) {
		writel_relaxed(0, base + channel * TIMER_STRIDE + TIMER_CTL);
		writel_relaxed(TIMER_IRQ_CLEAR,
			       base + channel * TIMER_STRIDE + TIMER_INT);
	}
	sc9830_source = base + TIMER_STRIDE + TIMER_SHADOW;
	writel_relaxed(U32_MAX, base + TIMER_STRIDE + TIMER_LOAD);
	writel_relaxed(TIMER_PERIODIC | TIMER_NEW | TIMER_ENABLE,
		       base + TIMER_STRIDE + TIMER_CTL);
	ret = clocksource_mmio_init(sc9830_source, "sc9830-source", timer_of_rate(&sc9830_timer),
				    200, 32, clocksource_mmio_readl_down);
	if (ret) {
		writel_relaxed(0, base + TIMER_STRIDE + TIMER_CTL);
		sc9830_source = NULL;
		timer_of_cleanup(&sc9830_timer);
		return ret;
	}
	sched_clock_register(sc9830_read_sched_clock, 32,
			     timer_of_rate(&sc9830_timer));
	writel_relaxed(TIMER_IRQ_ENABLE, base + TIMER_INT);
	clockevents_config_and_register(&sc9830_timer.clkevt,
					timer_of_rate(&sc9830_timer), 4, U32_MAX);
	return 0;
}

TIMER_OF_DECLARE(sc9830_timer, "sprd,sc9830-timer", sc9830_timer_init);
