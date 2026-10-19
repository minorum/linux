// SPDX-License-Identifier: GPL-2.0-only

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#include <asm/exception.h>

#define BCM2153_NR_IRQS	64
#define BCM2153_BANK_SIZE	0x100
#define BCM2153_IMR	0x00
#define BCM2153_ICR	0x08
#define BCM2153_IMSR	0x0c
#define BCM2153_ISTCR	0x10
#define BCM2153_ISELR	0x28
#define BCM2153_SICR	0x34

struct bcm2153_irq {
	void __iomem *base;
	struct irq_domain *domain;
	raw_spinlock_t lock;
	u32 owned[2];
};

static bool bcm2153_irq_valid(struct bcm2153_irq *ic, irq_hw_number_t hwirq)
{
	return hwirq < BCM2153_NR_IRQS &&
		(ic->owned[hwirq / 32] & BIT(hwirq % 32));
}

static void bcm2153_irq_enable(struct bcm2153_irq *ic, irq_hw_number_t hwirq,
			       bool enable)
{
	void __iomem *reg;
	unsigned long flags;
	u32 val, bit;

	if (!bcm2153_irq_valid(ic, hwirq))
		return;

	reg = ic->base + hwirq / 32 * BCM2153_BANK_SIZE + BCM2153_IMR;
	bit = BIT(hwirq % 32);
	raw_spin_lock_irqsave(&ic->lock, flags);
	val = readl(reg);
	writel(enable ? val | bit : val & ~bit, reg);
	raw_spin_unlock_irqrestore(&ic->lock, flags);
}

static void bcm2153_irq_mask(struct irq_data *d)
{
	bcm2153_irq_enable(irq_data_get_irq_chip_data(d), d->hwirq, false);
}

static void bcm2153_irq_unmask(struct irq_data *d)
{
	bcm2153_irq_enable(irq_data_get_irq_chip_data(d), d->hwirq, true);
}

static void bcm2153_irq_ack_hwirq(struct bcm2153_irq *ic, irq_hw_number_t hwirq)
{
	void __iomem *bank;
	u32 bit;

	if (!bcm2153_irq_valid(ic, hwirq))
		return;

	bank = ic->base + hwirq / 32 * BCM2153_BANK_SIZE;
	bit = BIT(hwirq % 32);
	writel(bit, bank + BCM2153_ICR);
	writel(bit, bank + BCM2153_SICR);
}

static void bcm2153_irq_ack(struct irq_data *d)
{
	bcm2153_irq_ack_hwirq(irq_data_get_irq_chip_data(d), d->hwirq);
}

static int bcm2153_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct bcm2153_irq *ic = irq_data_get_irq_chip_data(d);
	void __iomem *reg;
	unsigned long flags;
	unsigned int shift;
	u32 val, encoded;
	bool edge;

	if (!bcm2153_irq_valid(ic, d->hwirq))
		return -EINVAL;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		encoded = 0;
		edge = false;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		encoded = 1;
		edge = false;
		break;
	case IRQ_TYPE_EDGE_RISING:
		encoded = 4;
		edge = true;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		encoded = 5;
		edge = true;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		encoded = 6;
		edge = true;
		break;
	default:
		return -EINVAL;
	}

	reg = ic->base + d->hwirq / 32 * BCM2153_BANK_SIZE + BCM2153_ISTCR +
		(d->hwirq % 32 / 8) * 4;
	shift = (d->hwirq % 8) * 4;
	raw_spin_lock_irqsave(&ic->lock, flags);
	val = readl(reg) & ~(7U << shift);
	writel(val | encoded << shift, reg);
	raw_spin_unlock_irqrestore(&ic->lock, flags);
	irq_set_handler_locked(d, edge ? handle_edge_irq : handle_level_irq);
	return 0;
}

static void bcm2153_irq_initialize(struct bcm2153_irq *ic)
{
	unsigned int i;
	void __iomem *bank;

	for (i = 0; i < 2; i++) {
		bank = ic->base + i * BCM2153_BANK_SIZE;
		ic->owned[i] = ~readl(bank + BCM2153_ISELR);
		writel(readl(bank + BCM2153_IMR) & ~ic->owned[i], bank + BCM2153_IMR);
		writel(ic->owned[i], bank + BCM2153_ICR);
		writel(ic->owned[i], bank + BCM2153_SICR);
	}
}

static struct irq_chip bcm2153_irq_chip = {
	.name = "bcm2153-intc",
	.irq_mask = bcm2153_irq_mask,
	.irq_unmask = bcm2153_irq_unmask,
	.irq_ack = bcm2153_irq_ack,
	.irq_set_type = bcm2153_irq_set_type,
	.flags = IRQCHIP_SET_TYPE_MASKED,
};

static struct bcm2153_irq bcm2153_intc;

static int bcm2153_irq_map(struct irq_domain *domain, unsigned int irq,
			   irq_hw_number_t hwirq)
{
	struct bcm2153_irq *ic = domain->host_data;

	if (!bcm2153_irq_valid(ic, hwirq))
		return -EINVAL;

	irq_set_chip_data(irq, ic);
	irq_set_chip_and_handler(irq, &bcm2153_irq_chip, handle_level_irq);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops bcm2153_irq_ops = {
	.map = bcm2153_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static void __exception_irq_entry bcm2153_handle_irq(struct pt_regs *regs)
{
	struct bcm2153_irq *ic = &bcm2153_intc;
	unsigned int bank, hwirq;
	u32 pending;

	for (bank = 0; bank < 2; bank++) {
		pending = readl(ic->base + bank * BCM2153_BANK_SIZE + BCM2153_IMSR);
		pending &= ic->owned[bank];
		while (pending) {
			hwirq = bank * 32 + __ffs(pending);
			if (generic_handle_domain_irq(ic->domain, hwirq)) {
				bcm2153_irq_enable(ic, hwirq, false);
				bcm2153_irq_ack_hwirq(ic, hwirq);
				pr_warn_ratelimited("bcm2153-intc: unmapped IRQ %u\n", hwirq);
			}
			pending &= pending - 1;
		}
	}
}

static int __init bcm2153_irq_init(struct device_node *node,
				   struct device_node *parent)
{
	struct bcm2153_irq *ic = &bcm2153_intc;
	struct resource res;
	int ret;

	if (parent || ic->base)
		return -EINVAL;
	ret = of_address_to_resource(node, 0, &res);
	if (ret || resource_size(&res) != 0x200)
		return -EINVAL;
	ic->base = of_iomap(node, 0);
	if (!ic->base)
		return -ENOMEM;
	raw_spin_lock_init(&ic->lock);
	ic->domain = irq_domain_create_linear(of_fwnode_handle(node), BCM2153_NR_IRQS,
					      &bcm2153_irq_ops, ic);
	if (!ic->domain) {
		ret = -ENOMEM;
		goto unmap;
	}
	ret = set_handle_irq(bcm2153_handle_irq);
	if (ret)
		goto remove_domain;
	bcm2153_irq_initialize(ic);
	return 0;

remove_domain:
	irq_domain_remove(ic->domain);
unmap:
	iounmap(ic->base);
	ic->base = NULL;
	return ret;
}

IRQCHIP_DECLARE(bcm2153_intc, "brcm,bcm2153-intc", bcm2153_irq_init);
