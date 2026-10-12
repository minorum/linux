// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <asm/irq_regs.h>

#define DDR_PORTS	10
#define DDR_COUNTERS	6
#define DDR_ADDRESS	6
#define DDR_SLOTS	7
#define DDR_ACCESS	GENMASK_ULL(17, 16)
#define DDR_PROFILES	0x3bf
#define DDR_MATCH	BIT(31)
#define DDR_STRIDE	0x10000
#define DDR_WINDOW	0x2c
#define DDR_RESULT	0x44
#define DDR_COUNT	BIT(1)
#define DDR_START	BIT(3)
#define DDR_CLEAR	BIT(4)
#define DDR_IRQ		BIT(28)
#define DDR_ACK		BIT(29)
#define DDR_DONE		BIT(30)
#define DDR_STATUS	GENMASK(31, 30)
#define DDR_WRITABLE	(DDR_IRQ | DDR_CLEAR | DDR_START | DDR_COUNT | BIT(0))
#define DDR_BUSY		(DDR_WRITABLE & ~BIT(0))
#define DDR_GATE(p)	BIT(16 + (p))
#define DDR_EVENT_MASK	(GENMASK_ULL(11, 8) | GENMASK_ULL(2, 0))

struct sc9830_ddr_port {
	struct perf_event *events[DDR_SLOTS];
	u32 filters[10];
	u32 ctrl;
	u32 window;
	u32 gate;
	unsigned int users;
	unsigned int running;
	bool failed;
	bool address;
};

struct sc9830_ddr_pmu {
	struct pmu pmu;
	void __iomem *axi;
	void __iomem *pub;
	raw_spinlock_t lock;
	struct sc9830_ddr_port ports[DDR_PORTS];
	struct hlist_node node;
	int cpuhp_state;
	int irq;
	unsigned int cpu;
	unsigned int running;
	unsigned int users;
	unsigned int address_users;
};

struct sc9830_address_record {
	u32 version;
	u32 port;
	u32 control;
	u32 command;
	u32 id;
};

struct sc9830_address_sample {
	struct perf_event *event;
	struct sc9830_address_record record;
	u32 address;
};

#define to_ddr(p)	container_of(p, struct sc9830_ddr_pmu, pmu)

static void __iomem *ddr_base(struct sc9830_ddr_pmu *ddr, unsigned int port)
{
	return ddr->axi + port * DDR_STRIDE;
}

static bool ddr_pub_update(struct sc9830_ddr_pmu *ddr, unsigned int reg,
			   u32 mask, u32 value)
{
	u32 old = readl(ddr->pub + reg);

	writel((old & ~mask) | (value & mask), ddr->pub + reg);
	return (readl(ddr->pub + reg) & mask) == (value & mask);
}

static void ddr_rearm(struct sc9830_ddr_pmu *ddr, unsigned int port)
{
	void __iomem *base = ddr_base(ddr, port);

	if (ddr->ports[port].failed)
		return;
	if (!(readl(ddr->pub + 4) & DDR_GATE(port))) {
		ddr->ports[port].failed = true;
		pr_err_ratelimited("sc9830-ddr: port %u lost its clock\n", port);
		return;
	}
	if (ddr->ports[port].address) {
		writel(BIT(0) | DDR_ACK, base);
		if (readl(base) & DDR_STATUS) {
			ddr->ports[port].failed = true;
			pr_err_ratelimited("sc9830-ddr: port %u match acknowledgement failed\n",
					   port);
			return;
		}
		writel(BIT(0) | DDR_IRQ, base);
		return;
	}
	writel((readl(base) & 0xb) | 3, base);
	writel(3, base);
	writel(3 | DDR_CLEAR, base);
	writel(3 | DDR_ACK, base);
	writel(3 | DDR_START | DDR_IRQ, base);
}

static bool ddr_snapshot(struct sc9830_ddr_pmu *ddr, unsigned int port)
{
	struct sc9830_ddr_port *p = &ddr->ports[port];
	void __iomem *base = ddr_base(ddr, port);
	u32 values[DDR_COUNTERS];
	u32 ctrl;
	unsigned int i;

	if (p->failed)
		return false;
	if (!(readl(ddr->pub + 4) & DDR_GATE(port))) {
		p->failed = true;
		pr_err_ratelimited("sc9830-ddr: port %u lost its clock\n", port);
		return false;
	}
	ctrl = readl(base);
	if (!(ctrl & DDR_DONE))
		return false;
	for (i = 0; i < DDR_COUNTERS; i++)
		values[i] = readl(base + DDR_RESULT + i * 4);
	for (i = 0; i < DDR_COUNTERS; i++) {
		struct perf_event *event = p->events[i];

		if (event && !(event->hw.state & PERF_HES_STOPPED))
			local64_add(values[i], &event->count);
	}
	writel((ctrl & DDR_WRITABLE & ~DDR_CLEAR) | DDR_ACK, base);
	if (readl(base) & DDR_STATUS) {
		p->failed = true;
		writel(p->ctrl, base);
		pr_err_ratelimited("sc9830-ddr: port %u acknowledgement failed\n", port);
	}
	return true;
}

static int ddr_config_valid(u64 config)
{
	return !(config & ~DDR_EVENT_MASK) &&
		FIELD_GET(GENMASK_ULL(11, 8), config) < DDR_PORTS &&
		FIELD_GET(GENMASK_ULL(2, 0), config) < DDR_COUNTERS;
}

static bool ddr_is_address(struct perf_event *event)
{
	return (event->attr.config & 7) == DDR_ADDRESS;
}

static bool ddr_address_valid(struct perf_event *event)
{
	u64 config = event->attr.config;

	return !(config & ~(DDR_EVENT_MASK | DDR_ACCESS)) &&
		FIELD_GET(GENMASK_ULL(11, 8), config) < DDR_PORTS &&
		FIELD_GET(DDR_ACCESS, config) < 3 &&
		event->attr.config1 <= U32_MAX && event->attr.config2 <= U32_MAX &&
		event->attr.config1 <= event->attr.config2 &&
		event->attr.sample_period == 1 && !event->attr.freq &&
		!event->attr.precise_ip;
}

static bool ddr_group_valid(struct perf_event *event, struct perf_event *other)
{
	if (other->pmu != event->pmu)
		return is_software_event(other);
	if (ddr_is_address(event) != ddr_is_address(other))
		return false;
	if (ddr_is_address(event) &&
	    FIELD_GET(GENMASK_ULL(11, 8), event->attr.config) ==
	    FIELD_GET(GENMASK_ULL(11, 8), other->attr.config))
		return false;
	return event->attr.config != other->attr.config;
}

static int ddr_check_period(struct perf_event *event, u64 period)
{
	return ddr_is_address(event) && period == 1 ? 0 : -EINVAL;
}

static int ddr_event_init(struct perf_event *event)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(event->pmu);
	struct perf_event *sibling;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;
	if (event->attach_state & PERF_ATTACH_TASK ||
	    event->cpu < 0)
		return -EOPNOTSUPP;
	if (ddr_is_address(event)) {
		if (!ddr_address_valid(event))
			return -EINVAL;
	} else {
		if (is_sampling_event(event))
			return -EOPNOTSUPP;
		if (!ddr_config_valid(event->attr.config) || event->attr.config1 ||
		    event->attr.config2)
			return -EINVAL;
	}
	if (event->attr.exclude_user || event->attr.exclude_kernel ||
	    event->attr.exclude_hv || event->attr.exclude_idle ||
	    event->attr.exclude_host || event->attr.exclude_guest)
		return -EOPNOTSUPP;
	if (event->group_leader->pmu != event->pmu &&
	    !is_software_event(event->group_leader))
		return -EINVAL;
	for_each_sibling_event(sibling, event->group_leader) {
		if (!ddr_group_valid(event, sibling))
			return -EINVAL;
	}
	if (event != event->group_leader &&
	    !ddr_group_valid(event, event->group_leader))
		return -EINVAL;
	event->cpu = ddr->cpu;
	event->hw.idx = -1;
	if (ddr_is_address(event)) {
		event->hw.sample_period = 1;
		event->hw.last_period = 1;
		local64_set(&event->hw.period_left, 1);
	}
	return 0;
}

static unsigned int ddr_event_port(struct perf_event *event)
{
	return FIELD_GET(GENMASK_ULL(11, 8), event->attr.config);
}

static void ddr_start_locked(struct sc9830_ddr_pmu *ddr,
			     struct perf_event *event)
{
	unsigned int port = ddr_event_port(event);
	struct sc9830_ddr_port *p = &ddr->ports[port];

	if (!(event->hw.state & PERF_HES_STOPPED))
		return;
	if (p->running)
		ddr_snapshot(ddr, port);
	event->hw.state = 0;
	p->running++;
	ddr_rearm(ddr, port);
	if (p->address)
		return;
	if (!ddr->running++)
		ddr_pub_update(ddr, 0, BIT(0), BIT(0));
}

static void ddr_stop_locked(struct sc9830_ddr_pmu *ddr,
			    struct perf_event *event)
{
	unsigned int port = ddr_event_port(event);
	struct sc9830_ddr_port *p = &ddr->ports[port];

	if (event->hw.state & PERF_HES_STOPPED)
		return;
	if (p->address) {
		if (!p->failed)
			writel(p->ctrl | DDR_ACK, ddr_base(ddr, port));
		event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
		p->running--;
		return;
	}
	ddr_snapshot(ddr, port);
	if (!p->failed)
		writel(3, ddr_base(ddr, port));
	event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	p->running--;
	if (!--ddr->running)
		ddr_pub_update(ddr, 0, BIT(0), 0);
	if (p->running)
		ddr_rearm(ddr, port);
	else if (!p->failed)
		writel(p->ctrl | DDR_ACK, ddr_base(ddr, port));
}

static int ddr_check_owners(struct sc9830_ddr_pmu *ddr)
{
	u32 pub4, ctrl;
	unsigned int i;

	if (readl(ddr->pub + 4) & GENMASK(9, 0))
		return -EBUSY;
	if (!!(readl(ddr->pub) & BIT(0)) != !!ddr->running)
		return -EBUSY;
	for (i = 0; i < DDR_PORTS; i++) {
		if (ddr->ports[i].users)
			continue;
		pub4 = readl(ddr->pub + 4);
		if (!(pub4 & DDR_GATE(i)))
			continue;
		ctrl = readl(ddr_base(ddr, i));
		if (ctrl & DDR_BUSY || ctrl & ~(DDR_WRITABLE | DDR_STATUS))
			return -EBUSY;
	}
	return 0;
}

static int ddr_claim(struct sc9830_ddr_pmu *ddr, unsigned int port,
		     struct perf_event *event)
{
	struct sc9830_ddr_port *p = &ddr->ports[port];
	void __iomem *base = ddr_base(ddr, port);
	u32 filters[10] = { };
	static const u32 modes[] = { 1, 3, 0 };
	unsigned int i;
	u32 ctrl;
	int ret;

	ret = ddr_check_owners(ddr);
	if (ret)
		return ret;
	p->gate = readl(ddr->pub + 4) & DDR_GATE(port);
	if (!ddr_pub_update(ddr, 4, DDR_GATE(port), DDR_GATE(port))) {
		ddr_pub_update(ddr, 4, DDR_GATE(port), p->gate);
		return -EIO;
	}
	ctrl = readl(ddr_base(ddr, port));
	if (ctrl & DDR_BUSY || ctrl & ~(DDR_WRITABLE | DDR_STATUS)) {
		ddr_pub_update(ddr, 4, DDR_GATE(port), p->gate);
		return -EBUSY;
	}
	p->ctrl = ctrl & DDR_WRITABLE;
	p->window = readl(ddr_base(ddr, port) + DDR_WINDOW);
	p->address = ddr_is_address(event);
	if (p->address) {
		filters[0] = modes[FIELD_GET(DDR_ACCESS, event->attr.config)];
		filters[1] = event->attr.config1;
		filters[2] = event->attr.config2;
		if (DDR_PROFILES & BIT(port)) {
			filters[4] = 0x0fffffff;
			filters[5] = 0x0fffffff;
		} else {
			filters[6] = U32_MAX;
			filters[7] = U32_MAX;
		}
		for (i = 0; i < ARRAY_SIZE(filters); i++)
			p->filters[i] = readl(base + 4 + i * 4);
		if (readl(base) != ctrl || readl(base + DDR_WINDOW) != p->window)
			goto changed_owner;
		for (i = 0; i < ARRAY_SIZE(filters); i++) {
			if (readl(base + 4 + i * 4) != p->filters[i])
				goto changed_owner;
		}
		writel(p->ctrl | DDR_ACK, base);
		if (readl(base) & DDR_STATUS)
			goto restore_address;
		for (i = 0; i < 24; i++)
			writel(0, base + i * 4);
		for (i = 0; i < ARRAY_SIZE(filters); i++)
			writel(filters[i], base + 4 + i * 4);
		writel(0, base + DDR_WINDOW);
		for (i = 0; i < ARRAY_SIZE(filters); i++) {
			if (readl(base + 4 + i * 4) != filters[i])
				goto restore_address;
		}
		if (readl(base + DDR_WINDOW) || readl(base) & DDR_STATUS)
			goto restore_address;
		p->failed = false;
		return 0;
	}
	writel(6400000, ddr_base(ddr, port) + DDR_WINDOW);
	if (readl(ddr_base(ddr, port) + DDR_WINDOW) != 6400000) {
		writel(p->window, ddr_base(ddr, port) + DDR_WINDOW);
		ddr_pub_update(ddr, 4, DDR_GATE(port), p->gate);
		return -EIO;
	}
	p->failed = false;
	return 0;

restore_address:
	for (i = 0; i < ARRAY_SIZE(filters); i++)
		writel(p->filters[i], base + 4 + i * 4);
	writel(p->window, base + DDR_WINDOW);
	writel(p->ctrl, base);
	ddr_pub_update(ddr, 4, DDR_GATE(port), p->gate);
	return -EIO;

changed_owner:
	ddr_pub_update(ddr, 4, DDR_GATE(port), p->gate);
	return -EBUSY;
}

static void ddr_release(struct sc9830_ddr_pmu *ddr, unsigned int port)
{
	struct sc9830_ddr_port *p = &ddr->ports[port];
	void __iomem *base = ddr_base(ddr, port);
	unsigned int i;

	if (!(readl(ddr->pub + 4) & DDR_GATE(port))) {
		pr_err_ratelimited("sc9830-ddr: cannot restore gated port %u\n", port);
		return;
	}
	writel(p->ctrl | DDR_ACK, base);
	if (p->address) {
		for (i = 0; i < ARRAY_SIZE(p->filters); i++)
			writel(p->filters[i], base + 4 + i * 4);
		for (i = 0; i < ARRAY_SIZE(p->filters); i++) {
			if (readl(base + 4 + i * 4) != p->filters[i])
				pr_err_ratelimited("sc9830-ddr: port %u filter restore failed\n",
						   port);
		}
	}
	writel(p->ctrl, base);
	writel(p->window, base + DDR_WINDOW);
	if ((readl(base) & DDR_WRITABLE) != p->ctrl ||
	    readl(base + DDR_WINDOW) != p->window)
		pr_err_ratelimited("sc9830-ddr: port %u restore failed\n", port);
	if (!ddr_pub_update(ddr, 4, DDR_GATE(port), p->gate))
		pr_err_ratelimited("sc9830-ddr: port %u gate restore failed\n", port);
}

static void ddr_start(struct perf_event *event, int flags)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(event->pmu);
	unsigned long irqflags;

	raw_spin_lock_irqsave(&ddr->lock, irqflags);
	ddr_start_locked(ddr, event);
	raw_spin_unlock_irqrestore(&ddr->lock, irqflags);
}

static void ddr_stop(struct perf_event *event, int flags)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(event->pmu);
	unsigned long irqflags;

	raw_spin_lock_irqsave(&ddr->lock, irqflags);
	ddr_stop_locked(ddr, event);
	raw_spin_unlock_irqrestore(&ddr->lock, irqflags);
}

static int ddr_add(struct perf_event *event, int flags)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(event->pmu);
	unsigned int port = ddr_event_port(event);
	unsigned int counter = event->attr.config & 7;
	struct sc9830_ddr_port *p = &ddr->ports[port];
	unsigned long irqflags;
	int ret = 0;

	disable_irq_nosync(ddr->irq);
	synchronize_hardirq(ddr->irq);
	raw_spin_lock_irqsave(&ddr->lock, irqflags);
	if (ddr->users && !!ddr->address_users != ddr_is_address(event)) {
		ret = -EAGAIN;
		goto out;
	}
	if (p->users && p->failed) {
		ret = -EIO;
		goto out;
	}
	if (p->events[counter]) {
		ret = -EAGAIN;
		goto out;
	}
	if (!p->users) {
		ret = ddr_claim(ddr, port, event);
		if (ret)
			goto out;
	}
	p->events[counter] = event;
	p->users++;
	ddr->users++;
	if (p->address)
		ddr->address_users++;
	event->hw.idx = p->address ? DDR_PORTS * DDR_COUNTERS + port :
		port * DDR_COUNTERS + counter;
	event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	if (flags & PERF_EF_START)
		ddr_start_locked(ddr, event);
out:
	raw_spin_unlock_irqrestore(&ddr->lock, irqflags);
	enable_irq(ddr->irq);
	return ret;
}

static void ddr_del(struct perf_event *event, int flags)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(event->pmu);
	unsigned int port = ddr_event_port(event);
	struct sc9830_ddr_port *p = &ddr->ports[port];
	unsigned long irqflags;

	disable_irq_nosync(ddr->irq);
	synchronize_hardirq(ddr->irq);
	raw_spin_lock_irqsave(&ddr->lock, irqflags);
	ddr_stop_locked(ddr, event);
	p->events[event->attr.config & 7] = NULL;
	ddr->users--;
	if (p->address)
		ddr->address_users--;
	if (!--p->users)
		ddr_release(ddr, port);
	event->hw.idx = -1;
	raw_spin_unlock_irqrestore(&ddr->lock, irqflags);
	enable_irq(ddr->irq);
}

static void ddr_read(struct perf_event *event)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(event->pmu);
	unsigned int port = ddr_event_port(event);
	unsigned long irqflags;

	raw_spin_lock_irqsave(&ddr->lock, irqflags);
	if (!ddr_is_address(event) && !(event->hw.state & PERF_HES_STOPPED))
		ddr_snapshot(ddr, port);
	raw_spin_unlock_irqrestore(&ddr->lock, irqflags);
}

static irqreturn_t ddr_irq(int irq, void *data)
{
	struct sc9830_ddr_pmu *ddr = data;
	struct sc9830_address_sample samples[DDR_PORTS];
	unsigned long flags;
	unsigned int port, nr = 0, i;
	bool handled = false;

	raw_spin_lock_irqsave(&ddr->lock, flags);
	for (port = 0; port < DDR_PORTS; port++) {
		struct sc9830_ddr_port *p = &ddr->ports[port];
		void __iomem *base = ddr_base(ddr, port);
		u32 ctrl;

		if (p->address) {
			if (!p->running || p->failed)
				continue;
			if (!(readl(ddr->pub + 4) & DDR_GATE(port))) {
				p->failed = true;
				pr_err_ratelimited("sc9830-ddr: port %u lost its clock\n", port);
				continue;
			}
			ctrl = readl(base);
			if (!(ctrl & DDR_MATCH))
				continue;
			samples[nr].event = p->events[DDR_ADDRESS];
			samples[nr].address = readl(base + 0x34);
			samples[nr].record = (struct sc9830_address_record) {
				.version = 1, .port = port, .control = ctrl,
				.command = readl(base + 0x38),
				.id = readl(base + 0x64),
			};
			local64_add(1, &samples[nr].event->count);
			nr++;
			writel(BIT(0), base);
			ddr_rearm(ddr, port);
			handled = true;
			continue;
		}
		if (ddr->ports[port].running && ddr_snapshot(ddr, port))
			handled = true;
	}
	raw_spin_unlock_irqrestore(&ddr->lock, flags);
	for (i = 0; i < nr; i++) {
		struct perf_sample_data sample;
		struct perf_raw_record raw = {
			.frag = { .size = sizeof(samples[i].record),
				  .data = &samples[i].record },
		};

		perf_sample_data_init(&sample, samples[i].address, 1);
		sample.phys_addr = samples[i].address;
		sample.sample_flags |= PERF_SAMPLE_ADDR | PERF_SAMPLE_PHYS_ADDR;
		perf_sample_save_raw_data(&sample, samples[i].event, &raw);
		if (perf_event_overflow(samples[i].event, &sample, get_irq_regs()))
			ddr_stop(samples[i].event, 0);
	}
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static ssize_t cpumask_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct sc9830_ddr_pmu *ddr = to_ddr(dev_get_drvdata(dev));

	return cpumap_print_to_pagebuf(true, buf, cpumask_of(ddr->cpu));
}
static DEVICE_ATTR_RO(cpumask);

PMU_FORMAT_ATTR(event, "config:0-2");
PMU_FORMAT_ATTR(port, "config:8-11");
PMU_FORMAT_ATTR(access, "config:16-17");
PMU_FORMAT_ATTR(start, "config1:0-31");
PMU_FORMAT_ATTR(end, "config2:0-31");
static struct attribute *ddr_format_attrs[] = {
	&format_attr_event.attr, &format_attr_port.attr,
	&format_attr_access.attr, &format_attr_start.attr, &format_attr_end.attr,
	NULL,
};
static const struct attribute_group ddr_format_group = {
	.name = "format", .attrs = ddr_format_attrs,
};

#define DDR_EVENT(name, value) \
	static struct perf_pmu_events_attr ddr_event_##name = { \
		.attr = __ATTR(name, 0444, perf_event_sysfs_show, NULL), \
		.id = value, \
	}
DDR_EVENT(read_transactions, 0);
DDR_EVENT(write_transactions, 3);
DDR_EVENT(read_bandwidth_raw, 1);
DDR_EVENT(write_bandwidth_raw, 4);
DDR_EVENT(read_latency_raw, 2);
DDR_EVENT(write_latency_raw, 5);
DDR_EVENT(address_match, 6);
static struct attribute *ddr_event_attrs[] = {
	&ddr_event_read_transactions.attr.attr,
	&ddr_event_write_transactions.attr.attr,
	&ddr_event_read_bandwidth_raw.attr.attr,
	&ddr_event_write_bandwidth_raw.attr.attr,
	&ddr_event_read_latency_raw.attr.attr,
	&ddr_event_write_latency_raw.attr.attr,
	&ddr_event_address_match.attr.attr, NULL,
};
static const struct attribute_group ddr_event_group = {
	.name = "events", .attrs = ddr_event_attrs,
};
static struct attribute *ddr_attrs[] = { &dev_attr_cpumask.attr, NULL };
static const struct attribute_group ddr_group = { .attrs = ddr_attrs };
static const struct attribute_group *ddr_groups[] = {
	&ddr_group, &ddr_format_group, &ddr_event_group, NULL,
};

static int ddr_cpu_offline(unsigned int cpu, struct hlist_node *node)
{
	struct sc9830_ddr_pmu *ddr = hlist_entry(node, struct sc9830_ddr_pmu, node);
	unsigned int target;

	if (cpu != ddr->cpu)
		return 0;
	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids)
		return 0;
	perf_pmu_migrate_context(&ddr->pmu, cpu, target);
	ddr->cpu = target;
	irq_set_affinity(ddr->irq, cpumask_of(target));
	return 0;
}

static int ddr_probe(struct platform_device *pdev)
{
	struct sc9830_ddr_pmu *ddr;
	struct resource *res;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "axi");
	if (!res || resource_size(res) < DDR_PORTS * DDR_STRIDE)
		return -EINVAL;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pub");
	if (!res || resource_size(res) < 8)
		return -EINVAL;
	ddr = devm_kzalloc(&pdev->dev, sizeof(*ddr), GFP_KERNEL);
	if (!ddr)
		return -ENOMEM;
	ddr->axi = devm_platform_ioremap_resource_byname(pdev, "axi");
	if (IS_ERR(ddr->axi))
		return PTR_ERR(ddr->axi);
	ddr->pub = devm_platform_ioremap_resource_byname(pdev, "pub");
	if (IS_ERR(ddr->pub))
		return PTR_ERR(ddr->pub);
	ddr->irq = platform_get_irq(pdev, 0);
	if (ddr->irq < 0)
		return ddr->irq;
	raw_spin_lock_init(&ddr->lock);
	ret = ddr_check_owners(ddr);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "monitor already owned\n");
	ddr->cpu = raw_smp_processor_id();
	ddr->pmu = (struct pmu) {
		.module = THIS_MODULE,
		.task_ctx_nr = perf_invalid_context,
		.capabilities = PERF_PMU_CAP_NO_EXCLUDE,
		.attr_groups = ddr_groups,
		.event_init = ddr_event_init,
		.check_period = ddr_check_period,
		.add = ddr_add, .del = ddr_del,
		.start = ddr_start, .stop = ddr_stop, .read = ddr_read,
	};
	ret = devm_request_irq(&pdev->dev, ddr->irq, ddr_irq, IRQF_NO_THREAD,
			       dev_name(&pdev->dev), ddr);
	if (ret)
		return ret;
	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN,
				      "perf/sc9830-ddr:online", NULL, ddr_cpu_offline);
	if (ret < 0)
		return ret;
	ddr->cpuhp_state = ret;
	ret = cpuhp_state_add_instance_nocalls(ddr->cpuhp_state, &ddr->node);
	if (ret)
		goto remove_state;
	if (IS_ENABLED(CONFIG_SMP)) {
		ret = irq_set_affinity(ddr->irq, cpumask_of(ddr->cpu));
		if (ret)
			goto remove_instance;
	}
	ret = perf_pmu_register(&ddr->pmu, "sc9830_ddr", -1);
	if (ret)
		goto remove_instance;
	platform_set_drvdata(pdev, ddr);
	return 0;
remove_instance:
	cpuhp_state_remove_instance_nocalls(ddr->cpuhp_state, &ddr->node);
remove_state:
	cpuhp_remove_multi_state(ddr->cpuhp_state);
	return ret;
}

static void ddr_quiesce(struct sc9830_ddr_pmu *ddr)
{
	unsigned long flags;
	unsigned int port;

	disable_irq(ddr->irq);
	raw_spin_lock_irqsave(&ddr->lock, flags);
	if (ddr->running)
		ddr_pub_update(ddr, 0, BIT(0), 0);
	for (port = 0; port < DDR_PORTS; port++) {
		if (ddr->ports[port].users)
			ddr_release(ddr, port);
	}
	raw_spin_unlock_irqrestore(&ddr->lock, flags);
}

static void ddr_remove(struct platform_device *pdev)
{
	struct sc9830_ddr_pmu *ddr = platform_get_drvdata(pdev);

	perf_pmu_unregister(&ddr->pmu);
	cpuhp_state_remove_instance_nocalls(ddr->cpuhp_state, &ddr->node);
	cpuhp_remove_multi_state(ddr->cpuhp_state);
	ddr_quiesce(ddr);
}

static void ddr_shutdown(struct platform_device *pdev)
{
	ddr_quiesce(platform_get_drvdata(pdev));
}

static const struct of_device_id ddr_of_match[] = {
	{ .compatible = "sprd,sc9830-ddr-pmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, ddr_of_match);

static struct platform_driver ddr_driver = {
	.probe = ddr_probe,
	.remove = ddr_remove,
	.shutdown = ddr_shutdown,
	.driver = {
		.name = "sc9830-ddr-pmu",
		.of_match_table = ddr_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ddr_driver);
MODULE_DESCRIPTION("SC9830 DDR AXI performance monitor");
MODULE_LICENSE("GPL");
