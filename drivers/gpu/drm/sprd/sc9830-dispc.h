/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SC9830_DISPC_H_
#define _SC9830_DISPC_H_

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>

#define SC9830_DISPC_IRQ_ERROR	BIT(2)
#define SC9830_DISPC_IRQ_UPDATE	BIT(4)
#define SC9830_DISPC_IRQ_VSYNC	BIT(5)

static u32 sc9830_dispc_interrupt(void __iomem *base)
{
	u32 status = readl(base + 0x78) & (SC9830_DISPC_IRQ_ERROR |
					 SC9830_DISPC_IRQ_UPDATE |
					 SC9830_DISPC_IRQ_VSYNC);

	if (status & SC9830_DISPC_IRQ_ERROR) {
		writel(SC9830_DISPC_IRQ_ERROR, base + 0x74);
		writel(readl(base + 0x70) & ~SC9830_DISPC_IRQ_ERROR, base + 0x70);
	}
	if (status & SC9830_DISPC_IRQ_UPDATE)
		writel(SC9830_DISPC_IRQ_UPDATE, base + 0x74);
	if (status & SC9830_DISPC_IRQ_VSYNC)
		writel(SC9830_DISPC_IRQ_VSYNC, base + 0x74);
	return status;
}

static int sc9830_dispc_timing(unsigned int sync, unsigned int back,
			       unsigned int front, u32 *value)
{
	if (sync > 0xff || back > 0xfff || front > 0xfff)
		return -EINVAL;
	*value = sync | (back << 8) | (front << 20);
	return 0;
}

static int sc9830_dispc_layer(void __iomem *base, unsigned int width,
			      unsigned int height, unsigned int pitch,
			      unsigned int bpp)
{
	u32 size, ctrl;

	if (!width || width > 0xfff || !height || height > 0xfff ||
	    pitch < width || pitch > 0xfff || (bpp != 16 && bpp != 32))
		return -EINVAL;
	size = width | (height << 16);
	ctrl = bpp == 32 ? 0x8035 : 0x255;
	writel(0, base + 0x20);
	writel(readl(base + 0x40) & ~BIT(0), base + 0x40);
	writel(ctrl, base + 0x40);
	writel(0xff, base + 0x54);
	writel(size, base + 0x48);
	writel(0, base + 0x50);
	writel(pitch, base + 0x4c);
	writel(0, base + 0x58);
	return 0;
}

static void sc9830_dispc_run(void __iomem *base, bool first)
{
	writel(readl(base + 0x80) | BIT(5), base + 0x80);
	usleep_range(30, 50);
	if (first) {
		writel(readl(base + 0x80) & ~BIT(4), base + 0x80);
		writel(readl(base) | BIT(4), base);
	}
}

#endif
