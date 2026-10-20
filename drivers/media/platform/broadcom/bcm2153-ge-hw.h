/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef BCM2153_GE_HW_H
#define BCM2153_GE_HW_H

#include <linux/io.h>
#include <linux/iopoll.h>

#define BCM2153_GE_BUSY BIT(16)
#define BCM2153_GE_INTERRUPT BIT(8)

struct bcm2153_ge_job {
	u32 source;
	u32 destination;
	u32 source_stride;
	u32 destination_stride;
	u32 width;
	u32 height;
	u32 format;
};

static inline void bcm2153_ge_program(void __iomem *base, const struct bcm2153_ge_job *job)
{
	writel(0x100, base + 0x14);
	writel(0, base + 0x10);
	writel(job->format, base + 0x30);
	writel(job->destination_stride, base + 0x38);
	writel(job->destination, base + 0x28);
	writel(job->source_stride, base + 0x34);
	writel(job->source, base + 0x20);
	writel(0, base + 0x3c);
	writel(job->width, base + 0x40);
	writel(job->height, base + 0x44);
	writel(0, base + 4);
	writel(0x100, base + 8);
}

static inline int bcm2153_ge_stop(void __iomem *base)
{
	u32 status;
	int ret;

	writel(1, base + 4);
	ret = readl_poll_timeout(base, status, !(status & BCM2153_GE_BUSY), 1, 100000);
	if (ret)
		return ret;
	writel(0x100, base);
	return readl(base) ? -EIO : 0;
}

#endif
