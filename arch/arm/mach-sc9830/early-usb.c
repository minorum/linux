// SPDX-License-Identifier: GPL-2.0-only
#include <linux/console.h>
#include <linux/iopoll.h>
#include <linux/serial_core.h>

#include <asm/cacheflush.h>
#include <asm/memory.h>

#define IN_CTL	0x9a0
#define IN_INT	0x9a8
#define IN_SIZE	0x9b0
#define IN_DMA	0x9b4

static char sc9830_usb_buffer[512] __aligned(64);

static void sc9830_usb_write(struct console *console, const char *text,
			     unsigned int length)
{
	struct earlycon_device *device = console->data;
	void __iomem *base = device->port.membase;
	u32 control, status, packet_size;
	unsigned int chunk;

	while (length) {
		control = readl(base + IN_CTL);
		packet_size = control & 0x7ff;
		if (!(control & BIT(15)) ||
		    (packet_size != 64 && packet_size != 512))
			return;
		if (readl_poll_timeout_atomic(base + IN_CTL, status, !(status & BIT(31)), 0, 10000))
			return;
		chunk = min_t(unsigned int, length, sizeof(sc9830_usb_buffer));
		memcpy(sc9830_usb_buffer, text, chunk);
		__cpuc_flush_dcache_area(sc9830_usb_buffer,
					 sizeof(sc9830_usb_buffer));
		dsb(sy);
		writel(U32_MAX, base + IN_INT);
		writel(__pa(sc9830_usb_buffer), base + IN_DMA);
		writel((DIV_ROUND_UP(chunk, packet_size) << 19) | chunk,
		       base + IN_SIZE);
		writel((control & ~(BIT(30) | BIT(27))) | BIT(31) | BIT(26),
		       base + IN_CTL);
		if (readl_poll_timeout_atomic(base + IN_INT, status, status & BIT(0), 0, 10000))
			return;
		text += chunk;
		length -= chunk;
	}
}

static int __init sc9830_usb_setup(struct earlycon_device *device, const char *options)
{
	if (!device->port.membase || device->port.mapbase != 0x20200000)
		return -ENODEV;
	device->con->write = sc9830_usb_write;
	return 0;
}

EARLYCON_DECLARE(j320fn_usb, sc9830_usb_setup);
