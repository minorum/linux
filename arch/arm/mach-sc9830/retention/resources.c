// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>

static const struct {
	resource_size_t start;
	resource_size_t minimum;
	resource_size_t maximum;
} sc9830_retention_ranges[] = {
	{ 0x21500000, 0x98, 0x1000 },
	{ 0x20e00000, 0x3024, 0x10000 },
	{ 0x71300000, 0x0c, 0x10000 },
	{ 0x402e0000, 0x100, 0x10000 },
	{ 0x30020000, 0x1c, 0x10000 },
	{ 0x402b0000, 0xec, 0x10000 },
	{ 0x30000000, 0x34, 0x1000 },
	{ 0x30010000, 0x188, 0x10000 },
};

struct sc9830_retention_resources {
	void __iomem *base[ARRAY_SIZE(sc9830_retention_ranges)];
};

static void sc9830_retention_resources_release(struct sc9830_retention_resources *maps)
{
	unsigned int i;

	for (i = ARRAY_SIZE(maps->base); i-- > 0;)
		if (maps->base[i]) {
			iounmap(maps->base[i]);
			maps->base[i] = NULL;
		}
}

static int sc9830_retention_resources_map(struct sc9830_retention_resources *maps,
					  const struct resource *resources,
					  unsigned int count)
{
	resource_size_t size;
	unsigned int i;

	if (!resources || count != ARRAY_SIZE(maps->base))
		return -EINVAL;
	for (i = 0; i < count; i++)
		if (maps->base[i])
			return -EBUSY;
	for (i = 0; i < count; i++) {
		if (resource_type(&resources[i]) != IORESOURCE_MEM ||
		    resources[i].end < resources[i].start ||
		    resources[i].start != sc9830_retention_ranges[i].start)
			return -EINVAL;
		size = resource_size(&resources[i]);
		if (size < sc9830_retention_ranges[i].minimum ||
		    size > sc9830_retention_ranges[i].maximum || (size & 3))
			return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		maps->base[i] = ioremap(resources[i].start, resource_size(&resources[i]));
		if (!maps->base[i]) {
			sc9830_retention_resources_release(maps);
			return -ENOMEM;
		}
	}
	return 0;
}
