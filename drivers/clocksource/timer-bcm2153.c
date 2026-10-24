// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define BCM2153_GPT_CTRL(n)	(0x10 * (n) + 0x04)
#define BCM2153_GPT_RELOAD(n)	(0x10 * (n) + 0x08)
#define BCM2153_GPT_COUNTER(n)	(0x10 * (n) + 0x0c)
#define BCM2153_GPT_ENABLE	BIT(31)
#define BCM2153_GPT_IRQ_ENABLE	BIT(27)
#define BCM2153_GPT_POWER		BIT(24)
#define BCM2153_GPT_PENDING	BIT(0)
#define BCM2153_GPT_RUNNING	(BCM2153_GPT_ENABLE | BCM2153_GPT_POWER)
#define BCM2153_GPT_RATE		32768
#define BCM2153_GPT_MIN_DELTA	32

static void __iomem *bcm2153_base;

static u64 notrace bcm2153_sched_read(void)
{
	u32 count, next;

	count = readl(bcm2153_base + BCM2153_GPT_COUNTER(0));
	do {
		next = count;
		count = readl(bcm2153_base + BCM2153_GPT_COUNTER(0));
	} while (next != count);
	return ~count;
}

static u64 bcm2153_clock_read(struct clocksource *cs)
{
	return bcm2153_sched_read();
}

static int bcm2153_shutdown(struct clock_event_device *ce)
{
	writel(BCM2153_GPT_RUNNING | BCM2153_GPT_PENDING,
	       bcm2153_base + BCM2153_GPT_CTRL(1));
	return 0;
}

static int bcm2153_next_event(unsigned long delta, struct clock_event_device *ce)
{
	if (delta < BCM2153_GPT_MIN_DELTA || delta > U32_MAX)
		return -ETIME;
	bcm2153_shutdown(ce);
	writel(delta, bcm2153_base + BCM2153_GPT_RELOAD(1));
	writel(BCM2153_GPT_RUNNING | BCM2153_GPT_IRQ_ENABLE | BCM2153_GPT_PENDING,
	       bcm2153_base + BCM2153_GPT_CTRL(1));
	return 0;
}

static irqreturn_t bcm2153_interrupt(int irq, void *data)
{
	struct clock_event_device *ce = data;
	u32 control = readl(bcm2153_base + BCM2153_GPT_CTRL(1));

	if ((control & (BCM2153_GPT_PENDING | BCM2153_GPT_IRQ_ENABLE)) !=
	    (BCM2153_GPT_PENDING | BCM2153_GPT_IRQ_ENABLE))
		return IRQ_NONE;
	bcm2153_shutdown(ce);
	ce->event_handler(ce);
	return IRQ_HANDLED;
}

static struct clocksource bcm2153_clocksource = {
	.name = "bcm2153-gpt",
	.rating = 200,
	.read = bcm2153_clock_read,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct clock_event_device bcm2153_clockevent = {
	.name = "bcm2153-gpt",
	.rating = 200,
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown = bcm2153_shutdown,
	.set_state_oneshot = bcm2153_shutdown,
	.tick_resume = bcm2153_shutdown,
	.set_next_event = bcm2153_next_event,
};

static int __init bcm2153_timer_init(struct device_node *node)
{
	struct clock_event_device *ce = &bcm2153_clockevent;
	struct resource res;
	struct clk *clk;
	unsigned int i;
	int irq, ret;

	if (bcm2153_base)
		return -EBUSY;
	ret = of_address_to_resource(node, 0, &res);
	if (ret || resource_size(&res) != 0x60)
		return -EINVAL;
	irq = irq_of_parse_and_map(node, 0);
	if (!irq)
		return -EINVAL;
	clk = of_clk_get_by_name(node, "slow");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto dispose_irq;
	}
	ret = clk_prepare_enable(clk);
	if (ret)
		goto put_clock;
	if (clk_get_rate(clk) != BCM2153_GPT_RATE) {
		ret = -EINVAL;
		goto disable_clock;
	}
	bcm2153_base = of_iomap(node, 0);
	if (!bcm2153_base) {
		ret = -ENOMEM;
		goto disable_clock;
	}
	for (i = 0; i < 6; i++)
		writel((readl(bcm2153_base + BCM2153_GPT_CTRL(i)) &
			~BCM2153_GPT_IRQ_ENABLE) | BCM2153_GPT_PENDING,
		       bcm2153_base + BCM2153_GPT_CTRL(i));
	for (i = 0; i < 2; i++) {
		writel(0, bcm2153_base + BCM2153_GPT_CTRL(i));
		writel(BCM2153_GPT_RUNNING | BCM2153_GPT_PENDING,
		       bcm2153_base + BCM2153_GPT_CTRL(i));
		writel(U32_MAX, bcm2153_base + BCM2153_GPT_RELOAD(i));
	}
	ret = request_irq(irq, bcm2153_interrupt, IRQF_TIMER, node->full_name, ce);
	if (ret)
		goto stop_timers;
	ret = clocksource_register_hz(&bcm2153_clocksource, BCM2153_GPT_RATE);
	if (ret)
		goto free_interrupt;
	ce->irq = irq;
	ce->cpumask = cpumask_of(0);
	clockevents_config_and_register(ce, BCM2153_GPT_RATE,
					BCM2153_GPT_MIN_DELTA, U32_MAX);
	sched_clock_register(bcm2153_sched_read, 32, BCM2153_GPT_RATE);
	return 0;

free_interrupt:
	free_irq(irq, ce);
stop_timers:
	writel(0, bcm2153_base + BCM2153_GPT_CTRL(0));
	writel(0, bcm2153_base + BCM2153_GPT_CTRL(1));
	iounmap(bcm2153_base);
	bcm2153_base = NULL;
disable_clock:
	clk_disable_unprepare(clk);
put_clock:
	clk_put(clk);
dispose_irq:
	irq_dispose_mapping(irq);
	return ret;
}

TIMER_OF_DECLARE(bcm2153, "brcm,bcm2153-gpt", bcm2153_timer_init);
