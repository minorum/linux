// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "musb_core.h"

#define MT6577_SWRST 0x74
#define MT6577_L1INTM 0xa4

static u8 mt6577_clearb(void __iomem *base, u32 offset)
{
	u8 value = musb_readb(base, offset);

	musb_writeb(base, offset, value);
	return value;
}

static u16 mt6577_clearw(void __iomem *base, u32 offset)
{
	u16 value = musb_readw(base, offset);

	musb_writew(base, offset, value);
	return value;
}

static irqreturn_t mt6577_interrupt(int irq, void *data)
{
	struct musb *musb = data;
	unsigned long flags;
	irqreturn_t result = IRQ_NONE;

	spin_lock_irqsave(&musb->lock, flags);
	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB) &
			musb_readb(musb->mregs, MUSB_INTRUSBE);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX) &
		       musb_readw(musb->mregs, MUSB_INTRTXE);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX) &
		       musb_readw(musb->mregs, MUSB_INTRRXE);
	musb_writeb(musb->mregs, MUSB_INTRUSB, musb->int_usb);
	musb_writew(musb->mregs, MUSB_INTRTX, musb->int_tx);
	musb_writew(musb->mregs, MUSB_INTRRX, musb->int_rx);
	if (musb->int_usb & MUSB_INTR_RESET)
		musb_writeb(musb->mregs, MUSB_FADDR, 0);
	if (musb->int_usb || musb->int_tx || musb->int_rx)
		result = musb_interrupt(musb);
	spin_unlock_irqrestore(&musb->lock, flags);
	return result;
}

static int mt6577_init(struct musb *musb)
{
	/* shortcut: inherited PHY calibration, replace for cold boot */
	musb_writeb(musb->mregs, MUSB_POWER, 0);
	msleep(20);
	musb_writew(musb->mregs, MT6577_SWRST,
		    musb_readw(musb->mregs, MT6577_SWRST) | 3);
	musb_writel(musb->mregs, MT6577_L1INTM, 7);
	musb->dyn_fifo = true;
	musb->isr = mt6577_interrupt;
	return 0;
}

static const struct musb_platform_ops mt6577_ops = {
	.quirks = MUSB_INDEXED_EP,
	.init = mt6577_init,
	.clearb = mt6577_clearb,
	.clearw = mt6577_clearw,
};

static const struct musb_fifo_cfg mt6577_fifo[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
};

static const struct musb_hdrc_config mt6577_config = {
	.fifo_cfg = mt6577_fifo,
	.fifo_cfg_size = ARRAY_SIZE(mt6577_fifo),
	.num_eps = 3,
	.ram_bits = 11,
};

static const struct musb_hdrc_platform_data mt6577_pdata = {
	.mode = MUSB_PERIPHERAL,
	.config = &mt6577_config,
	.platform_ops = &mt6577_ops,
};

static void mt6577_unregister(void *data)
{
	platform_device_unregister(data);
}

static int mt6577_probe(struct platform_device *pdev)
{
	struct platform_device_info info = {
		.parent = &pdev->dev,
		.name = "musb-hdrc",
		.id = PLATFORM_DEVID_AUTO,
		.res = pdev->resource,
		.num_res = pdev->num_resources,
		.data = &mt6577_pdata,
		.size_data = sizeof(mt6577_pdata),
		.dma_mask = DMA_BIT_MASK(32),
		.fwnode = dev_fwnode(&pdev->dev),
		.of_node_reused = true,
	};
	struct platform_device *musb;

	musb = platform_device_register_full(&info);
	if (IS_ERR(musb))
		return PTR_ERR(musb);
	return devm_add_action_or_reset(&pdev->dev, mt6577_unregister, musb);
}

static const struct of_device_id mt6577_match[] = {
	{ .compatible = "mediatek,mt6577-musb-ram" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6577_match);

static struct platform_driver mt6577_driver = {
	.probe = mt6577_probe,
	.driver = {
		.name = "mt6577-musb-ram",
		.of_match_table = mt6577_match,
	},
};
module_platform_driver(mt6577_driver);

MODULE_DESCRIPTION("MT6577 RAM-stage MUSB glue");
MODULE_LICENSE("GPL");
