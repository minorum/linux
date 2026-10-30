// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>

#include "musb_core.h"

#define MT6577_SWRST 0x74
#define MT6577_L1INTM 0xa4

struct mt6577_glue {
	struct clk *bus;
	struct phy *phy;
};

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
	struct mt6577_glue *glue = dev_get_drvdata(musb->controller->parent);
	int ret;

	ret = clk_prepare_enable(glue->bus);
	if (ret)
		return ret;
	ret = phy_set_mode(glue->phy, PHY_MODE_USB_DEVICE);
	if (ret)
		goto disable_clock;
	ret = phy_init(glue->phy);
	if (ret)
		goto disable_clock;
	ret = phy_power_on(glue->phy);
	if (ret)
		goto exit_phy;
	musb_writeb(musb->mregs, MUSB_POWER, 0);
	msleep(20);
	musb_writew(musb->mregs, MT6577_SWRST,
		    musb_readw(musb->mregs, MT6577_SWRST) | 3);
	musb_writel(musb->mregs, MT6577_L1INTM, 7);
	musb->dyn_fifo = true;
	musb->isr = mt6577_interrupt;
	return 0;
exit_phy:
	phy_exit(glue->phy);
disable_clock:
	clk_disable_unprepare(glue->bus);
	return ret;
}

static int mt6577_exit(struct musb *musb)
{
	struct mt6577_glue *glue = dev_get_drvdata(musb->controller->parent);
	int ret;

	musb_writel(musb->mregs, MT6577_L1INTM, 0);
	musb_writeb(musb->mregs, MUSB_POWER, 0);
	ret = phy_power_off(glue->phy);
	if (ret)
		return ret;
	phy_exit(glue->phy);
	clk_disable_unprepare(glue->bus);
	return 0;
}

static const struct musb_platform_ops mt6577_ops = {
	.quirks = MUSB_INDEXED_EP,
	.init = mt6577_init,
	.exit = mt6577_exit,
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
	struct mt6577_glue *glue;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;
	glue->bus = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(glue->bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(glue->bus), "Missing bus clock\n");
	glue->phy = devm_phy_get(&pdev->dev, "usb2-phy");
	if (IS_ERR(glue->phy))
		return dev_err_probe(&pdev->dev, PTR_ERR(glue->phy), "Missing USB PHY\n");
	platform_set_drvdata(pdev, glue);
	musb = platform_device_register_full(&info);
	if (IS_ERR(musb))
		return PTR_ERR(musb);
	return devm_add_action_or_reset(&pdev->dev, mt6577_unregister, musb);
}

static const struct of_device_id mt6577_match[] = {
	{ .compatible = "mediatek,mt6577-musb" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6577_match);

static struct platform_driver mt6577_driver = {
	.probe = mt6577_probe,
	.driver = {
		.name = "mt6577-musb",
		.of_match_table = mt6577_match,
	},
};
module_platform_driver(mt6577_driver);

MODULE_DESCRIPTION("MediaTek MT6577 MUSB glue");
MODULE_LICENSE("GPL");
