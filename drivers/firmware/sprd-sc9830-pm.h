/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SPRD_SC9830_PM_H
#define SPRD_SC9830_PM_H

#define SC9830_PM_TX_RD 0x800
#define SC9830_PM_TX_WR 0x804
#define SC9830_PM_RX_RD 0x808
#define SC9830_PM_RX_WR 0x80c
#define SC9830_PM_OPEN 0xbeee0116
#define SC9830_PM_ENABLE 0x03000d16
#define SC9830_PM_DISABLE 0x03050d16
#define SC9830_PM_QUERY 0x05000d16
#define SC9830_PM_TARGET 0x05030d16
#define SC9830_PM_RATE 0x00000d16
#define SC9830_PM_RATE_REPLY 0x00000e16
#define SC9830_PM_ENABLED 0x03000e16
#define SC9830_PM_QUERIED 0x05000e16
#define SC9830_PM_FAILED 0x0f0f0e16

struct sc9830_pm_ring {
	void __iomem *base;
	u32 tx;
	u32 rx;
};

static int sc9830_pm_ring_init(struct sc9830_pm_ring *ring)
{
	unsigned int i;

	for (i = 0; i < 4; i++)
		if (readl(ring->base + SC9830_PM_TX_RD + i * 4))
			return -EBUSY;
	ring->tx = 0;
	ring->rx = 0;
	return 0;
}

static int sc9830_pm_publish(struct sc9830_pm_ring *ring, u32 header, u32 value)
{
	u32 offset = (ring->tx & 127) * 8;

	if (readl(ring->base + SC9830_PM_TX_RD) != ring->tx ||
	    readl(ring->base + SC9830_PM_TX_WR) != ring->tx ||
	    readl(ring->base + SC9830_PM_RX_RD) != ring->rx ||
	    readl(ring->base + SC9830_PM_RX_WR) != ring->rx)
		return -EBUSY;
	writel(header, ring->base + offset);
	writel(value, ring->base + offset + 4);
	wmb();
	writel(++ring->tx, ring->base + SC9830_PM_TX_WR);
	wmb();
	return 0;
}

static int sc9830_pm_receive(struct sc9830_pm_ring *ring, u32 expected, u32 *value)
{
	static const int status[] = { 1, -EINVAL, -EBUSY, 1, -EIO, -EACCES };
	u32 rd = readl(ring->base + SC9830_PM_RX_RD);
	u32 wr = readl(ring->base + SC9830_PM_RX_WR);
	u32 header, reply, offset;

	if (rd != ring->rx || readl(ring->base + SC9830_PM_TX_WR) != ring->tx)
		return -EPROTO;
	if (wr == rd)
		return -EAGAIN;
	if (wr - rd != 1)
		return -EPROTO;
	rmb();
	offset = 0x400 + (rd & 127) * 8;
	header = readl(ring->base + offset);
	reply = readl(ring->base + offset + 4);
	if (expected == SC9830_PM_RATE_REPLY) {
		if (header != SC9830_PM_FAILED &&
		    ((header & 0xffff) != SC9830_PM_RATE_REPLY || header >> 16 > 5))
			return -EPROTO;
	} else if (header != (expected == SC9830_PM_DISABLE ? SC9830_PM_ENABLED : expected) &&
	    !(expected != SC9830_PM_OPEN && header == SC9830_PM_FAILED))
		return -EPROTO;
	if (header == SC9830_PM_OPEN && reply != 0xff00)
		return -EPROTO;
	if (header == SC9830_PM_ENABLED &&
	    reply != (expected == SC9830_PM_DISABLE ? 640 : 0))
		return -EPROTO;
	if ((header == SC9830_PM_QUERIED ||
	     (expected == SC9830_PM_RATE_REPLY && header != SC9830_PM_FAILED)) &&
	    reply != 192 && reply != 384 &&
	    reply != 500 && reply != 640)
		return -EPROTO;
	if (readl(ring->base + SC9830_PM_RX_RD) != rd ||
	    readl(ring->base + SC9830_PM_RX_WR) != wr ||
	    readl(ring->base + SC9830_PM_TX_WR) != ring->tx)
		return -EPROTO;
	*value = reply;
	mb();
	writel(++ring->rx, ring->base + SC9830_PM_RX_RD);
	wmb();
	if (header == SC9830_PM_FAILED)
		return -EREMOTEIO;
	return expected == SC9830_PM_RATE_REPLY ? status[header >> 16] : 1;
}
#endif
