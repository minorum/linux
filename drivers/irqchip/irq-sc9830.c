// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#define SC9830_IRQ_EN 0x08
#define SC9830_IRQ_DIS 0x0c
#define SC9830_IRQ_COUNT 128

struct sc9830_intc {
	void __iomem *base[5];
	struct clk *clocks[5];
	raw_spinlock_t lock;
	DECLARE_BITMAP(active, SC9830_IRQ_COUNT);
	unsigned int users[32];
	u32 inherited;
};

static bool sc9830_irq_valid(unsigned long hwirq)
{
	return hwirq >= 2 && hwirq <= 124 && (hwirq & 31) >= 2;
}

static int sc9830_irq_route(unsigned long hwirq)
{
	switch (hwirq) {
	case 28 ... 31:
	case 118 ... 121:
		return 2;
	case 21 ... 24:
		return 3;
	case 20:
	case 25:
	case 34 ... 36:
	case 38:
		return 4;
	case 40 ... 45:
		return 5;
	case 39:
		return 6;
	case 83 ... 84:
		return 7;
	case 123 ... 124:
		return 8;
	case 26:
	case 122:
		return 9;
	case 86:
		return 10;
	case 69:
		return 12;
	case 37:
		return 31;
	default:
		return -1;
	}
}

static void sc9830_irq_gate(struct irq_data *data, bool enable)
{
	struct sc9830_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned long hwirq = data->hwirq, flags;
	int route = sc9830_irq_route(hwirq);

	raw_spin_lock_irqsave(&intc->lock, flags);
	if (enable == test_bit(hwirq, intc->active))
		goto out;
	if (enable) {
		set_bit(hwirq, intc->active);
		if (route >= 0 && !intc->users[route]++)
			writel_relaxed(BIT(route), intc->base[4] + SC9830_IRQ_EN);
		writel_relaxed(BIT(hwirq & 31),
			       intc->base[hwirq / 32] + SC9830_IRQ_EN);
	} else {
		clear_bit(hwirq, intc->active);
		writel_relaxed(BIT(hwirq & 31),
			       intc->base[hwirq / 32] + SC9830_IRQ_DIS);
		if (route >= 0 && !--intc->users[route] &&
		    !(intc->inherited & BIT(route)))
			writel_relaxed(BIT(route), intc->base[4] + SC9830_IRQ_DIS);
	}
	if (route >= 0)
		readl(intc->base[4] + SC9830_IRQ_EN);
	readl(intc->base[hwirq / 32] + SC9830_IRQ_EN);
out:
	raw_spin_unlock_irqrestore(&intc->lock, flags);
}

static void sc9830_irq_mask(struct irq_data *data)
{
	irq_chip_mask_parent(data);
	sc9830_irq_gate(data, false);
}

static void sc9830_irq_unmask(struct irq_data *data)
{
	sc9830_irq_gate(data, true);
	irq_chip_unmask_parent(data);
}

static struct irq_chip sc9830_irq_chip = {
	.name = "SC9830-INTC",
	.irq_mask = sc9830_irq_mask,
	.irq_unmask = sc9830_irq_unmask,
	.irq_eoi = irq_chip_eoi_parent,
	.irq_set_type = irq_chip_set_type_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_retrigger = irq_chip_retrigger_hierarchy,
	.flags = IRQCHIP_SET_TYPE_MASKED | IRQCHIP_SKIP_SET_WAKE |
		 IRQCHIP_MASK_ON_SUSPEND,
};

static int sc9830_irq_translate(struct irq_domain *domain,
				struct irq_fwspec *spec,
				unsigned long *hwirq, unsigned int *type)
{
	if (!is_of_node(spec->fwnode) || spec->param_count != 3 ||
	    spec->param[0] != 0 || !sc9830_irq_valid(spec->param[1]) ||
	    !(spec->param[2] & IRQ_TYPE_SENSE_MASK) ||
	    spec->param[2] & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;
	*hwirq = spec->param[1];
	*type = spec->param[2];
	return 0;
}

static int sc9830_irq_alloc(struct irq_domain *domain, unsigned int virq,
			    unsigned int count, void *arg)
{
	struct irq_fwspec *spec = arg, parent_spec;
	unsigned long hwirq;
	unsigned int type, i;
	int ret;

	ret = sc9830_irq_translate(domain, spec, &hwirq, &type);
	if (ret)
		return ret;
	if (!count || count > SC9830_IRQ_COUNT - hwirq)
		return -EINVAL;
	for (i = 0; i < count; i++)
		if (!sc9830_irq_valid(hwirq + i))
			return -EINVAL;
	for (i = 0; i < count; i++)
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					     &sc9830_irq_chip, domain->host_data);
	parent_spec = *spec;
	parent_spec.fwnode = domain->parent->fwnode;
	return irq_domain_alloc_irqs_parent(domain, virq, count, &parent_spec);
}

static const struct irq_domain_ops sc9830_irq_ops = {
	.translate = sc9830_irq_translate,
	.alloc = sc9830_irq_alloc,
	.free = irq_domain_free_irqs_common,
};

static int __init sc9830_irq_init(struct device_node *node,
				 struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct sc9830_intc *intc;
	struct resource resource;
	unsigned int i, enabled = 0;
	int ret;

	parent_domain = parent ? irq_find_host(parent) : NULL;
	if (!parent_domain)
		return -ENODEV;
	intc = kzalloc_obj(struct sc9830_intc);
	if (!intc)
		return -ENOMEM;
	raw_spin_lock_init(&intc->lock);
	for (i = 0; i < ARRAY_SIZE(intc->base); i++) {
		ret = of_address_to_resource(node, i, &resource);
		if (ret)
			goto fail;
		if (resource_size(&resource) < SC9830_IRQ_DIS + sizeof(u32)) {
			ret = -EINVAL;
			goto fail;
		}
		intc->base[i] = of_iomap(node, i);
		if (!intc->base[i]) {
			ret = -ENOMEM;
			goto fail;
		}
		intc->clocks[i] = of_clk_get(node, i);
		if (IS_ERR(intc->clocks[i])) {
			ret = PTR_ERR(intc->clocks[i]);
			intc->clocks[i] = NULL;
			goto fail;
		}
		ret = clk_prepare_enable(intc->clocks[i]);
		if (ret)
			goto fail;
		enabled++;
	}
	intc->inherited = readl_relaxed(intc->base[4] + SC9830_IRQ_EN);
	domain = irq_domain_create_hierarchy(parent_domain, 0, SC9830_IRQ_COUNT,
					     of_fwnode_handle(node), &sc9830_irq_ops, intc);
	if (domain)
		return 0;
	ret = -ENOMEM;
fail:
	while (enabled)
		clk_disable_unprepare(intc->clocks[--enabled]);
	for (i = 0; i < ARRAY_SIZE(intc->base); i++) {
		if (intc->clocks[i])
			clk_put(intc->clocks[i]);
		if (intc->base[i])
			iounmap(intc->base[i]);
	}
	kfree(intc);
	return ret;
}

IRQCHIP_DECLARE(sc9830_intc, "sprd,sc9830-intc", sc9830_irq_init);
