// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spreadtrum mailbox driver
 *
 * Copyright (c) 2020 Spreadtrum Communications Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>

#define SPRD_MBOX_ID		0x0
#define SPRD_MBOX_MSG_LOW	0x4
#define SPRD_MBOX_MSG_HIGH	0x8
#define SPRD_MBOX_TRIGGER	0xc
#define SPRD_MBOX_FIFO_RST	0x10
#define SPRD_MBOX_FIFO_STS	0x14
#define SPRD_MBOX_IRQ_STS	0x18
#define SPRD_MBOX_IRQ_MSK	0x1c
#define SPRD_MBOX_LOCK		0x20
#define SPRD_MBOX_FIFO_DEPTH	0x24 /* outbox only */
#define SPRD_MBOX_IN_FIFO_STS2	0x24 /* inbox only, revision 2 */

/* Bit and mask definition for inbox's SPRD_MBOX_FIFO_STS register */
#define SPRD_INBOX_FIFO_DELIVER_MASK		GENMASK(23, 16)
#define SPRD_INBOX_FIFO_OVERLOW_MASK		GENMASK(15, 8)
#define SPRD_INBOX_FIFO_DELIVER_SHIFT		16
#define SPRD_INBOX_FIFO_BUSY_MASK		GENMASK(7, 0)

/* Bit and mask definition for R2 inbox's SPRD_MBOX_FIFO_RST register */
#define SPRD_INBOX_R2_FIFO_OVERFLOW_DELIVER_RST	GENMASK(31, 0)

/* Bit and mask definition for R2 inbox's SPRD_MBOX_FIFO_STS register */
#define SPRD_INBOX_R2_FIFO_DELIVER_MASK		GENMASK(15, 0)

/* Bit and mask definition for SPRD_MBOX_IN_FIFO_STS2 register */
#define SPRD_INBOX_R2_FIFO_OVERFLOW_MASK	GENMASK(31, 16)
#define SPRD_INBOX_R2_FIFO_BUSY_MASK		GENMASK(15, 0)

/* Bit and mask definition for SPRD_MBOX_IRQ_STS register */
#define SPRD_MBOX_IRQ_CLR			GENMASK(31, 0)

/* Bit and mask definition for outbox's SPRD_MBOX_FIFO_STS register */
#define SPRD_OUTBOX_FIFO_FULL			BIT(2)
#define SPRD_OUTBOX_FIFO_WR_SHIFT		16
#define SPRD_OUTBOX_FIFO_RD_SHIFT		24
#define SPRD_OUTBOX_FIFO_POS_MASK		GENMASK(7, 0)

/* Bit and mask definition for inbox's SPRD_MBOX_IRQ_MSK register */
#define SPRD_INBOX_FIFO_BLOCK_IRQ		BIT(0)
#define SPRD_INBOX_FIFO_OVERFLOW_IRQ		BIT(1)
#define SPRD_INBOX_FIFO_DELIVER_IRQ		BIT(2)
#define SPRD_INBOX_FIFO_IRQ_MASK		GENMASK(2, 0)

/* Bit and mask definition for outbox's SPRD_MBOX_IRQ_MSK register */
#define SPRD_OUTBOX_FIFO_NOT_EMPTY_IRQ		BIT(0)
#define SPRD_OUTBOX_FIFO_IRQ_MASK		GENMASK(4, 0)

#define SPRD_OUTBOX_BASE_SPAN			0x1000
#define SPRD_MBOX_R1_CHAN_MAX			8
#define SPRD_MBOX_R2_CHAN_MAX			16

enum sprd_mbox_version {
	SPRD_MBOX_R1,
	SPRD_MBOX_R2,
	SPRD_MBOX_SC9830,
};

struct sprd_mbox_info {
	enum sprd_mbox_version version;
	unsigned long supp_id;
	bool j320fn_handover;
};

struct sprd_mbox_priv {
	struct mbox_controller	mbox;
	struct device		*dev;
	void __iomem		*inbox_base;
	void __iomem		*outbox_base;
	/*  Base register address for supplementary outbox */
	void __iomem		*supp_base;
	u32			outbox_fifo_depth;

	const struct sprd_mbox_info *info;

	struct mutex		lock;
	u32			refcnt;
	spinlock_t		doorbell_lock;
	u32			active;
	u32			saved_mask;
	int			outbox_irq;
	struct mbox_chan	chan[SPRD_MBOX_R2_CHAN_MAX];
};

static struct sprd_mbox_priv *to_sprd_mbox_priv(struct mbox_controller *mbox)
{
	return container_of(mbox, struct sprd_mbox_priv, mbox);
}

static u32 sprd_mbox_get_fifo_len(struct sprd_mbox_priv *priv, u32 fifo_sts)
{
	u32 wr_pos = (fifo_sts >> SPRD_OUTBOX_FIFO_WR_SHIFT) &
		SPRD_OUTBOX_FIFO_POS_MASK;
	u32 rd_pos = (fifo_sts >> SPRD_OUTBOX_FIFO_RD_SHIFT) &
		SPRD_OUTBOX_FIFO_POS_MASK;
	u32 fifo_len;

	/*
	 * If the read pointer is equal with write pointer, which means the fifo
	 * is full or empty.
	 */
	if (wr_pos == rd_pos) {
		if (fifo_sts & SPRD_OUTBOX_FIFO_FULL)
			fifo_len = priv->outbox_fifo_depth;
		else
			fifo_len = 0;
	} else if (wr_pos > rd_pos) {
		fifo_len = wr_pos - rd_pos;
	} else {
		fifo_len = priv->outbox_fifo_depth - rd_pos + wr_pos;
	}

	return fifo_len;
}

static irqreturn_t do_outbox_isr(void __iomem *base, struct sprd_mbox_priv *priv)
{
	struct mbox_chan *chan;
	u32 fifo_sts, fifo_len, msg[2];
	int i, id;

	fifo_sts = readl(base + SPRD_MBOX_FIFO_STS);

	fifo_len = sprd_mbox_get_fifo_len(priv, fifo_sts);
	if (!fifo_len) {
		dev_warn_ratelimited(priv->dev, "spurious outbox interrupt\n");
		return IRQ_NONE;
	}

	for (i = 0; i < fifo_len; i++) {
		msg[0] = readl(base + SPRD_MBOX_MSG_LOW);
		msg[1] = readl(base + SPRD_MBOX_MSG_HIGH);
		id = readl(base + SPRD_MBOX_ID);

		chan = &priv->chan[id];
		if (chan->cl)
			mbox_chan_received_data(chan, (void *)msg);
		else
			dev_warn_ratelimited(priv->dev,
				    "message's been dropped at ch[%d]\n", id);

		/* Trigger to update outbox FIFO pointer */
		writel(0x1, base + SPRD_MBOX_TRIGGER);
	}

	/* Clear irq status after reading all message. */
	writel(SPRD_MBOX_IRQ_CLR, base + SPRD_MBOX_IRQ_STS);

	return IRQ_HANDLED;
}

static irqreturn_t sprd_mbox_outbox_isr(int irq, void *data)
{
	struct sprd_mbox_priv *priv = data;

	return do_outbox_isr(priv->outbox_base, priv);
}

static irqreturn_t sprd_mbox_supp_isr(int irq, void *data)
{
	struct sprd_mbox_priv *priv = data;

	return do_outbox_isr(priv->supp_base, priv);
}

static irqreturn_t sprd_mbox_inbox_isr(int irq, void *data)
{
	struct sprd_mbox_priv *priv = data;
	struct mbox_chan *chan;
	u32 fifo_sts, fifo_sts2, send_sts, busy, id;

	fifo_sts = readl(priv->inbox_base + SPRD_MBOX_FIFO_STS);

	if (priv->info->version == SPRD_MBOX_R2)
		fifo_sts2 = readl(priv->inbox_base + SPRD_MBOX_IN_FIFO_STS2);

	/* Get the inbox data delivery status */
	if (priv->info->version == SPRD_MBOX_R2) {
		send_sts = fifo_sts & SPRD_INBOX_R2_FIFO_DELIVER_MASK;
	} else {
		send_sts = (fifo_sts & SPRD_INBOX_FIFO_DELIVER_MASK) >>
			SPRD_INBOX_FIFO_DELIVER_SHIFT;
	}

	if (!send_sts) {
		dev_warn_ratelimited(priv->dev, "spurious inbox interrupt\n");
		return IRQ_NONE;
	}

	/* Clear FIFO delivery and overflow status first */
	if (priv->info->version == SPRD_MBOX_R2) {
		writel(SPRD_INBOX_R2_FIFO_OVERFLOW_DELIVER_RST,
		       priv->inbox_base + SPRD_MBOX_FIFO_RST);
	} else {
		writel(fifo_sts & (SPRD_INBOX_FIFO_DELIVER_MASK | SPRD_INBOX_FIFO_OVERLOW_MASK),
		       priv->inbox_base + SPRD_MBOX_FIFO_RST);
	}

	while (send_sts) {
		id = __ffs(send_sts);
		send_sts &= (send_sts - 1);

		chan = &priv->chan[id];

		/*
		 * Check if the message was fetched by remote target, if yes,
		 * that means the transmission has been completed.
		 */
		if (priv->info->version == SPRD_MBOX_R2)
			busy = fifo_sts2 & SPRD_INBOX_R2_FIFO_BUSY_MASK;
		else
			busy = fifo_sts & SPRD_INBOX_FIFO_BUSY_MASK;

		if (!(busy & BIT(id)))
			mbox_chan_txdone(chan, 0);
	}

	/* Clear irq status */
	writel(SPRD_MBOX_IRQ_CLR, priv->inbox_base + SPRD_MBOX_IRQ_STS);

	return IRQ_HANDLED;
}

static int sprd_mbox_send_data(struct mbox_chan *chan, void *msg)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);
	unsigned long id = (unsigned long)chan->con_priv;
	u32 *data = msg;

	/* Write data into inbox FIFO, and only support 8 bytes every time */
	writel(data[0], priv->inbox_base + SPRD_MBOX_MSG_LOW);
	writel(data[1], priv->inbox_base + SPRD_MBOX_MSG_HIGH);

	/* Set target core id */
	writel(id, priv->inbox_base + SPRD_MBOX_ID);

	/* Trigger remote request */
	writel(0x1, priv->inbox_base + SPRD_MBOX_TRIGGER);

	return 0;
}

static int sprd_mbox_flush(struct mbox_chan *chan, unsigned long timeout)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);
	unsigned long id = (unsigned long)chan->con_priv;
	u32 busy;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		busy = readl(priv->inbox_base + SPRD_MBOX_FIFO_STS) &
			SPRD_INBOX_FIFO_BUSY_MASK;
		if (!(busy & BIT(id))) {
			mbox_chan_txdone(chan, 0);
			return 0;
		}

		udelay(1);
	}

	return -ETIME;
}

static int sprd_mbox_startup(struct mbox_chan *chan)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);
	u32 val;

	mutex_lock(&priv->lock);
	if (priv->refcnt++ == 0) {
		/* Select outbox FIFO mode and reset the outbox FIFO status */
		writel(0x0, priv->outbox_base + SPRD_MBOX_FIFO_RST);

		/* Enable inbox FIFO delivery interrupt */
		val = SPRD_INBOX_FIFO_IRQ_MASK;
		val &= ~SPRD_INBOX_FIFO_DELIVER_IRQ;
		writel(val, priv->inbox_base + SPRD_MBOX_IRQ_MSK);

		/* Enable outbox FIFO not empty interrupt */
		val = SPRD_OUTBOX_FIFO_IRQ_MASK;
		val &= ~SPRD_OUTBOX_FIFO_NOT_EMPTY_IRQ;
		writel(val, priv->outbox_base + SPRD_MBOX_IRQ_MSK);

		/* Enable supplementary outbox as the fundamental one */
		if (priv->supp_base) {
			writel(0x0, priv->supp_base + SPRD_MBOX_FIFO_RST);
			writel(val, priv->supp_base + SPRD_MBOX_IRQ_MSK);
		}
	}
	mutex_unlock(&priv->lock);

	return 0;
}

static void sprd_mbox_shutdown(struct mbox_chan *chan)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);

	mutex_lock(&priv->lock);
	if (--priv->refcnt == 0) {
		/* Disable inbox & outbox interrupt */
		writel(SPRD_INBOX_FIFO_IRQ_MASK, priv->inbox_base + SPRD_MBOX_IRQ_MSK);
		writel(SPRD_OUTBOX_FIFO_IRQ_MASK, priv->outbox_base + SPRD_MBOX_IRQ_MSK);

		if (priv->supp_base)
			writel(SPRD_OUTBOX_FIFO_IRQ_MASK,
			       priv->supp_base + SPRD_MBOX_IRQ_MSK);
	}
	mutex_unlock(&priv->lock);
}

static const struct mbox_chan_ops sprd_mbox_ops = {
	.send_data	= sprd_mbox_send_data,
	.flush		= sprd_mbox_flush,
	.startup	= sprd_mbox_startup,
	.shutdown	= sprd_mbox_shutdown,
};

static int sc9830_mbox_send(struct mbox_chan *chan, void *data)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);
	unsigned long flags;

	spin_lock_irqsave(&priv->doorbell_lock, flags);
	writel((unsigned long)chan->con_priv, priv->inbox_base + SPRD_MBOX_ID);
	writel(1, priv->inbox_base + SPRD_MBOX_TRIGGER);
	spin_unlock_irqrestore(&priv->doorbell_lock, flags);
	return 0;
}

static irqreturn_t sc9830_mbox_inbox_isr(int irq, void *data)
{
	return IRQ_NONE;
}

static irqreturn_t sc9830_mbox_isr(int irq, void *data)
{
	struct sprd_mbox_priv *priv = data;
	unsigned long flags;
	u32 status, active;
	unsigned int id;

	spin_lock_irqsave(&priv->doorbell_lock, flags);
	active = priv->active;
	if (!active) {
		spin_unlock_irqrestore(&priv->doorbell_lock, flags);
		return IRQ_NONE;
	}
	status = readl(priv->outbox_base + SPRD_MBOX_FIFO_STS);
	writel(status & (active << 8), priv->outbox_base + SPRD_MBOX_IRQ_STS);
	spin_unlock_irqrestore(&priv->doorbell_lock, flags);
	for (id = 0; id < SPRD_MBOX_R1_CHAN_MAX; id++)
		if (active & BIT(id))
			mbox_chan_received_data(&priv->chan[id], NULL);
	return IRQ_HANDLED;
}

static int sc9830_mbox_startup(struct mbox_chan *chan)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);
	unsigned int id = (unsigned long)chan->con_priv;
	unsigned long flags;
	u32 mask, unowned, bit = BIT(id + 8);
	bool first;

	if (!chan->cl->knows_txdone)
		return -EINVAL;
	mutex_lock(&priv->lock);
	spin_lock_irqsave(&priv->doorbell_lock, flags);
	if (readl(priv->outbox_base + SPRD_MBOX_FIFO_RST) != BIT(16)) {
		spin_unlock_irqrestore(&priv->doorbell_lock, flags);
		mutex_unlock(&priv->lock);
		return -EBUSY;
	}
	mask = readl(priv->outbox_base + SPRD_MBOX_IRQ_MSK);
	unowned = GENMASK(15, 8) & ~(priv->active << 8);
	if ((mask & unowned) != unowned) {
		spin_unlock_irqrestore(&priv->doorbell_lock, flags);
		mutex_unlock(&priv->lock);
		return -EBUSY;
	}
	first = !priv->active;
	priv->saved_mask = (priv->saved_mask & ~bit) | (mask & bit);
	priv->active |= BIT(id);
	writel(mask & ~bit, priv->outbox_base + SPRD_MBOX_IRQ_MSK);
	spin_unlock_irqrestore(&priv->doorbell_lock, flags);
	if (first)
		enable_irq(priv->outbox_irq);
	mutex_unlock(&priv->lock);
	return 0;
}

static void sc9830_mbox_shutdown(struct mbox_chan *chan)
{
	struct sprd_mbox_priv *priv = to_sprd_mbox_priv(chan->mbox);
	unsigned int id = (unsigned long)chan->con_priv;
	unsigned long flags;
	u32 mask, bit = BIT(id + 8);
	bool last;

	mutex_lock(&priv->lock);
	spin_lock_irqsave(&priv->doorbell_lock, flags);
	priv->active &= ~BIT(id);
	last = !priv->active;
	mask = readl(priv->outbox_base + SPRD_MBOX_IRQ_MSK);
	writel((mask & ~bit) | (priv->saved_mask & bit),
	       priv->outbox_base + SPRD_MBOX_IRQ_MSK);
	spin_unlock_irqrestore(&priv->doorbell_lock, flags);
	if (last)
		disable_irq(priv->outbox_irq);
	else
		synchronize_irq(priv->outbox_irq);
	mutex_unlock(&priv->lock);
}

static const struct mbox_chan_ops sc9830_mbox_ops = {
	.send_data = sc9830_mbox_send,
	.startup = sc9830_mbox_startup,
	.shutdown = sc9830_mbox_shutdown,
};

static void j320fn_mbox_snapshot(struct sprd_mbox_priv *priv, void __iomem *counters,
				 void __iomem *pm, struct clk *clk, u32 *state)
{
	static const u32 offsets[] = { SPRD_MBOX_FIFO_RST, SPRD_MBOX_FIFO_STS, SPRD_MBOX_IRQ_MSK };
	unsigned int i;

	state[12] = __clk_is_enabled(clk);
	if (!state[12])
		return;
	for (i = 0; i < 3; i++) {
		state[i] = readl(priv->inbox_base + offsets[i]);
		state[i + 3] = readl(priv->outbox_base + offsets[i]);
	}
	state[6] = readl(pm + 0x10);
	state[7] = readl(pm + 0x1c);
	for (i = 0; i < 4; i++)
		state[i + 8] = readl(counters + i * 4);
}

static int j320fn_mbox_initialize(struct sprd_mbox_priv *priv, void __iomem *counters,
				   void __iomem *pm, struct clk *clk)
{
	static const u32 inherited[] = { 0, 3, 0x2f, 0, 0x07008001, 0x1e,
					0x10000, 0x1f, 0, 0, 0, 0, 1 };
	static const u32 initialized[] = { 0, 3, 0x3f, 0x10000, 2, 0xff00,
					  0x10000, 0x1f, 0, 0, 0, 0, 1 };
	u32 before[13] = {}, after[13] = {};

	j320fn_mbox_snapshot(priv, counters, pm, clk, before);
	j320fn_mbox_snapshot(priv, counters, pm, clk, after);
	if (memcmp(before, inherited, sizeof(before)) || memcmp(before, after, sizeof(before)))
		return -EBUSY;
	writel(1, priv->inbox_base + SPRD_MBOX_FIFO_RST);
	writel(1, priv->outbox_base + SPRD_MBOX_FIFO_RST);
	writel(0, priv->inbox_base + SPRD_MBOX_FIFO_RST);
	writel(0xffffffff, priv->inbox_base + SPRD_MBOX_IRQ_MSK);
	writel(BIT(16), priv->outbox_base + SPRD_MBOX_FIFO_RST);
	writel(0xff00, priv->outbox_base + SPRD_MBOX_IRQ_MSK);
	j320fn_mbox_snapshot(priv, counters, pm, clk, after);
	return memcmp(after, initialized, sizeof(after)) ? -EIO : 0;
}

static int j320fn_mbox_prepare(struct platform_device *pdev, struct sprd_mbox_priv *priv,
			      struct clk *clk)
{
	static const resource_size_t starts[] = { 0x400a0000, 0x400a8000, 0x5000c800, 0x400a8100 };
	static const resource_size_t sizes[] = { 0x100, 0x100, 0x10, 0x100 };
	struct device *dev = &pdev->dev;
	struct resource *resource;
	struct irq_data *data;
	void __iomem *counters, *pm;
	unsigned int i;
	int ret, irq;

	if (!of_machine_is_compatible("samsung,j3xnlte"))
		return -ENODEV;
	for (i = 0; i < 2; i++) {
		irq = platform_get_irq_byname(pdev, i ? "outbox" : "inbox");
		if (irq < 0)
			return irq;
		data = irq_get_irq_data(irq);
		if (!data || data->hwirq != 68 + i ||
		    irq_get_trigger_type(irq) != IRQ_TYPE_LEVEL_HIGH ||
		    !of_device_is_compatible(irq_domain_get_of_node(data->domain),
				     "sprd,sc9830-intc"))
			return -EINVAL;
	}
	for (i = 0; i < ARRAY_SIZE(starts); i++) {
		resource = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!resource || resource->start != starts[i] ||
		    resource_size(resource) != sizes[i])
			return -EINVAL;
	}
	counters = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(counters))
		return PTR_ERR(counters);
	pm = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(pm))
		return PTR_ERR(pm);
	ret = j320fn_mbox_initialize(priv, counters, pm, clk);
	devm_iounmap(dev, counters);
	devm_release_mem_region(dev, starts[2], sizes[2]);
	return ret;
}

static int sprd_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sprd_mbox_priv *priv;
	int ret, inbox_irq, outbox_irq, supp_irq;
	unsigned long id;
	struct clk *clk;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	mutex_init(&priv->lock);
	spin_lock_init(&priv->doorbell_lock);

	priv->info = of_device_get_match_data(dev);
	if (!priv->info)
		return -EINVAL;

	/*
	 * Unisoc mailbox uses an inbox to send messages to the target
	 * core, and uses (an) outbox(es) to receive messages from other
	 * cores.
	 *
	 * Thus in general the mailbox controller supplies 2 different
	 * register addresses and IRQ numbers for inbox and outbox.
	 *
	 * If necessary, a supplementary inbox could be enabled optionally
	 * with an independent FIFO and an extra interrupt.
	 */
	priv->inbox_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->inbox_base))
		return PTR_ERR(priv->inbox_base);

	priv->outbox_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(priv->outbox_base))
		return PTR_ERR(priv->outbox_base);

	clk = devm_clk_get_enabled(dev, "enable");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get mailbox clock\n");
		return PTR_ERR(clk);
	}
	if (priv->info->version == SPRD_MBOX_SC9830) {
		if (!__clk_is_enabled(clk))
			return dev_err_probe(dev, -EIO, "firmware mailbox gate is disabled\n");
		if (!priv->info->j320fn_handover &&
		    readl(priv->outbox_base + SPRD_MBOX_FIFO_RST) != BIT(16))
			return dev_err_probe(dev, -EBUSY, "mailbox mode handover is unqualified\n");
		inbox_irq = platform_get_irq_byname(pdev, "inbox");
		if (inbox_irq < 0)
			return inbox_irq;
		ret = devm_request_irq(dev, inbox_irq, sc9830_mbox_inbox_isr,
				       IRQF_NO_AUTOEN, dev_name(dev), priv);
		if (ret)
			return ret;
		outbox_irq = platform_get_irq_byname(pdev, "outbox");
		if (outbox_irq < 0)
			return outbox_irq;
		priv->outbox_irq = outbox_irq;
		ret = devm_request_irq(dev, outbox_irq, sc9830_mbox_isr, IRQF_NO_AUTOEN,
				       dev_name(dev), priv);
		if (ret)
			return ret;
		if (priv->info->j320fn_handover) {
			ret = j320fn_mbox_prepare(pdev, priv, clk);
			if (ret)
				return dev_err_probe(dev, ret,
						     "firmware mailbox handover failed\n");
		}
		priv->mbox.ops = &sc9830_mbox_ops;
		goto register_controller;
	}

	inbox_irq = platform_get_irq_byname(pdev, "inbox");
	if (inbox_irq < 0)
		return inbox_irq;

	ret = devm_request_irq(dev, inbox_irq, sprd_mbox_inbox_isr,
			       IRQF_NO_SUSPEND, dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "failed to request inbox IRQ: %d\n", ret);
		return ret;
	}

	outbox_irq = platform_get_irq_byname(pdev, "outbox");
	if (outbox_irq < 0)
		return outbox_irq;

	ret = devm_request_irq(dev, outbox_irq, sprd_mbox_outbox_isr,
			       IRQF_NO_SUSPEND, dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "failed to request outbox IRQ: %d\n", ret);
		return ret;
	}

	/* Supplementary outbox IRQ is optional */
	supp_irq = platform_get_irq_byname(pdev, "supp-outbox");
	if (supp_irq > 0) {
		ret = devm_request_irq(dev, supp_irq, sprd_mbox_supp_isr,
				       IRQF_NO_SUSPEND, dev_name(dev), priv);
		if (ret) {
			dev_err(dev, "failed to request outbox IRQ: %d\n", ret);
			return ret;
		}

		if (!priv->info->supp_id) {
			dev_err(dev, "no supplementary outbox specified\n");
			return -ENODEV;
		}
		priv->supp_base = priv->outbox_base +
			(SPRD_OUTBOX_BASE_SPAN * priv->info->supp_id);
	}

	/* Get the default outbox FIFO depth */
	priv->outbox_fifo_depth =
		readl(priv->outbox_base + SPRD_MBOX_FIFO_DEPTH) + 1;
	priv->mbox.ops = &sprd_mbox_ops;
	priv->mbox.txdone_irq = true;

register_controller:
	priv->mbox.dev = dev;
	priv->mbox.chans = &priv->chan[0];

	if (priv->info->version == SPRD_MBOX_R2)
		priv->mbox.num_chans = SPRD_MBOX_R2_CHAN_MAX;
	else
		priv->mbox.num_chans = SPRD_MBOX_R1_CHAN_MAX;

	for (id = 0; id < priv->mbox.num_chans; id++)
		priv->chan[id].con_priv = (void *)id;

	ret = devm_mbox_controller_register(dev, &priv->mbox);
	if (ret) {
		dev_err(dev, "failed to register mailbox: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct sprd_mbox_info sc9860_mbox_info = {
	.version = SPRD_MBOX_R1,
};

static const struct sprd_mbox_info sc9830_mbox_info = {
	.version = SPRD_MBOX_SC9830,
};

static const struct sprd_mbox_info j320fn_mbox_info = {
	.version = SPRD_MBOX_SC9830,
	.j320fn_handover = true,
};

static const struct sprd_mbox_info sc9863a_mbox_info = {
	.version = SPRD_MBOX_R1,
	.supp_id = 7,
};

static const struct sprd_mbox_info ums9230_mbox_info = {
	.version = SPRD_MBOX_R2,
	.supp_id = 6,
};

static const struct of_device_id sprd_mbox_of_match[] = {
	{ .compatible = "samsung,j320fn-mailbox", .data = &j320fn_mbox_info },
	{ .compatible = "sprd,sc9830-mailbox", .data = &sc9830_mbox_info },
	{ .compatible = "sprd,sc9860-mailbox", .data = &sc9860_mbox_info },
	{ .compatible = "sprd,sc9863a-mailbox", .data = &sc9863a_mbox_info },
	{ .compatible = "sprd,ums9230-mailbox", .data = &ums9230_mbox_info },
	{ },
};
MODULE_DEVICE_TABLE(of, sprd_mbox_of_match);

static struct platform_driver sprd_mbox_driver = {
	.driver = {
		.name = "sprd-mailbox",
		.of_match_table = sprd_mbox_of_match,
	},
	.probe	= sprd_mbox_probe,
};
module_platform_driver(sprd_mbox_driver);

MODULE_AUTHOR("Baolin Wang <baolin.wang@unisoc.com>");
MODULE_DESCRIPTION("Spreadtrum mailbox driver");
MODULE_LICENSE("GPL v2");
