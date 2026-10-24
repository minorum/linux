// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include "sdhci-pltfm.h"

#define BCM2153_SDIO_RATE	52000000
#define BCM2153_SDIO_IP_IRQ	BIT(0)

struct bcm2153_host {
	void __iomem *clock;
	void __iomem *irq_mask;
	u16 transfer_mode;
	u16 block_size;
};

static u16 bcm2153_readw(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + (reg & ~3)) >> ((reg & 3) * 8);
}

static u8 bcm2153_readb(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + (reg & ~3)) >> ((reg & 3) * 8);
}

static void bcm2153_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm = sdhci_priv(host);
	struct bcm2153_host *priv = sdhci_pltfm_priv(pltfm);
	u32 shift = (reg & 3) * 8;
	u32 old;

	if (reg == SDHCI_BLOCK_SIZE) {
		priv->block_size = val;
		return;
	}
	if (reg == SDHCI_BLOCK_COUNT) {
		writel((u32)val << 16 | priv->block_size,
		       host->ioaddr + SDHCI_BLOCK_SIZE);
		return;
	}
	if (reg == SDHCI_TRANSFER_MODE) {
		priv->transfer_mode = val;
		return;
	}
	if (reg == SDHCI_COMMAND) {
		writel((u32)val << 16 | priv->transfer_mode,
		       host->ioaddr + SDHCI_TRANSFER_MODE);
		return;
	}
	if (reg == SDHCI_CLOCK_CONTROL) {
		val &= ~(SDHCI_DIV_MASK << SDHCI_DIVIDER_SHIFT);
		val &= ~(SDHCI_DIV_HI_MASK >> SDHCI_DIV_MASK_LEN
			 << SDHCI_DIVIDER_HI_SHIFT);
		if (!(val & SDHCI_CLOCK_INT_EN))
			writel(0, priv->clock);
	}
	if ((reg & ~3) == SDHCI_INT_STATUS) {
		writel((u32)val << shift, host->ioaddr + SDHCI_INT_STATUS);
		return;
	}
	old = readl(host->ioaddr + (reg & ~3));
	writel((old & ~(0xffffU << shift)) | (u32)val << shift,
	       host->ioaddr + (reg & ~3));
}

static void bcm2153_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 shift = (reg & 3) * 8;
	u32 old;

	if ((reg & ~3) == SDHCI_INT_STATUS) {
		writel((u32)val << shift, host->ioaddr + SDHCI_INT_STATUS);
		return;
	}
	old = readl(host->ioaddr + (reg & ~3));
	writel((old & ~(0xffU << shift)) | (u32)val << shift,
	       host->ioaddr + (reg & ~3));
}

static void bcm2153_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm = sdhci_priv(host);
	struct bcm2153_host *priv = sdhci_pltfm_priv(pltfm);
	u32 div;
	u16 clk;

	host->mmc->actual_clock = 0;
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	if (!clock)
		return;

	clk = sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	div = (clk >> SDHCI_DIVIDER_SHIFT) & SDHCI_DIV_MASK;
	div |= ((clk >> SDHCI_DIVIDER_HI_SHIFT) << SDHCI_DIV_MASK_LEN) &
		SDHCI_DIV_HI_MASK;
	writel(div ? 2 * div - 1 : 0, priv->clock + 4);
	writel(1, priv->clock);
	sdhci_enable_clk(host, clk & ~(SDHCI_DIV_MASK << SDHCI_DIVIDER_SHIFT) &
			 ~(SDHCI_DIV_HI_MASK >> SDHCI_DIV_MASK_LEN <<
			   SDHCI_DIVIDER_HI_SHIFT));
}

static unsigned int bcm2153_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm = sdhci_priv(host);

	return pltfm->clock;
}

static void bcm2153_dump_vendor_regs(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm = sdhci_priv(host);
	struct bcm2153_host *priv = sdhci_pltfm_priv(pltfm);

	pr_err("%s: BCM2153 mask: %08x gate: %08x divider: %08x requested: %u reported: %u\n",
	       mmc_hostname(host->mmc), readl(priv->irq_mask),
	       readl(priv->clock), readl(priv->clock + 4),
	       host->clock, host->mmc->actual_clock);
}

static const struct sdhci_ops bcm2153_ops = {
	.read_w = bcm2153_readw,
	.read_b = bcm2153_readb,
	.write_w = bcm2153_writew,
	.write_b = bcm2153_writeb,
	.set_clock = bcm2153_set_clock,
	.get_max_clock = bcm2153_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.dump_vendor_regs = bcm2153_dump_vendor_regs,
};

static const struct sdhci_pltfm_data bcm2153_data = {
	.ops = &bcm2153_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_BROKEN_DMA | SDHCI_QUIRK_BROKEN_ADMA,
	.quirks2 = SDHCI_QUIRK2_NO_1_8_V | SDHCI_QUIRK2_HOST_NO_CMD23 |
		   SDHCI_QUIRK2_CLEAR_TRANSFERMODE_REG_BEFORE_CMD,
};

static int bcm2153_probe(struct platform_device *pdev)
{
	static const char * const names[] = { "host", "interrupt-mask", "clock" };
	static const resource_size_t sizes[] = { 0x100, 4, 8 };
	struct sdhci_pltfm_host *pltfm;
	struct bcm2153_host *priv;
	struct sdhci_host *host;
	struct pinctrl *pins;
	struct pinctrl_state *pins_init, *pins_powered;
	struct reset_control *reset;
	struct resource *res;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, names[i]);
		if (!res || resource_size(res) != sizes[i] ||
		    res != platform_get_resource(pdev, IORESOURCE_MEM, i))
			return -EINVAL;
	}
	reset = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(&pdev->dev, PTR_ERR(reset),
				     "failed to get external reset\n");

	host = sdhci_pltfm_init(pdev, &bcm2153_data, sizeof(*priv));
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm);
	priv->irq_mask = devm_platform_ioremap_resource_byname(pdev, "interrupt-mask");
	if (IS_ERR(priv->irq_mask))
		return PTR_ERR(priv->irq_mask);
	priv->clock = devm_platform_ioremap_resource_byname(pdev, "clock");
	if (IS_ERR(priv->clock))
		return PTR_ERR(priv->clock);

	sdhci_get_property(pdev);
	if (pltfm->clock != BCM2153_SDIO_RATE)
		return -EINVAL;
	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;
	if (!host->mmc->f_max || host->mmc->f_max > 25000000)
		host->mmc->f_max = 25000000;
	if (readl(host->ioaddr + SDHCI_CLOCK_CONTROL) & BIT(24))
		return -EBUSY;

	ret = mmc_regulator_get_supply(host->mmc);
	if (ret)
		return ret;
	if (IS_ERR(host->mmc->supply.vmmc))
		return PTR_ERR(host->mmc->supply.vmmc);
	ret = regulator_enable(host->mmc->supply.vmmc);
	if (ret)
		return ret;
	host->mmc->regulator_enabled = true;
	ret = mmc_regulator_enable_vqmmc(host->mmc);
	if (ret)
		goto disable_vmmc;
	pins = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pins)) {
		ret = PTR_ERR(pins);
		goto disable_vqmmc;
	}
	pins_init = pinctrl_lookup_state(pins, "powered-init");
	if (IS_ERR(pins_init)) {
		ret = PTR_ERR(pins_init);
		goto disable_vqmmc;
	}
	pins_powered = pinctrl_lookup_state(pins, "powered");
	if (IS_ERR(pins_powered)) {
		ret = PTR_ERR(pins_powered);
		goto disable_vqmmc;
	}
	ret = pinctrl_select_state(pins, pins_init);
	if (ret)
		goto disable_vqmmc;
	ret = pinctrl_select_state(pins, pins_powered);
	if (ret)
		goto disable_vqmmc;

	ret = reset_control_reset(reset);
	if (ret)
		goto disable_vqmmc;

	ret = sdhci_setup_host(host);
	if (ret) {
		writel(0, priv->clock);
		goto disable_vqmmc;
	}
	if (host->clk_mul) {
		ret = -EOPNOTSUPP;
		goto cleanup_host;
	}
	writel(readl(priv->irq_mask) | BCM2153_SDIO_IP_IRQ, priv->irq_mask);
	ret = __sdhci_add_host(host);
	if (!ret)
		return 0;
	writel(readl(priv->irq_mask) & ~BCM2153_SDIO_IP_IRQ, priv->irq_mask);

cleanup_host:
	sdhci_cleanup_host(host);
	writel(0, priv->clock);
disable_vqmmc:
	mmc_regulator_disable_vqmmc(host->mmc);
disable_vmmc:
	mmc_regulator_set_ocr(host->mmc, host->mmc->supply.vmmc, 0);
	return ret;
}

static void bcm2153_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm = sdhci_priv(host);
	struct bcm2153_host *priv = sdhci_pltfm_priv(pltfm);

	sdhci_pltfm_remove(pdev);
	writel(readl(priv->irq_mask) & ~BCM2153_SDIO_IP_IRQ, priv->irq_mask);
	writel(0, priv->clock);
	mmc_regulator_disable_vqmmc(host->mmc);
	mmc_regulator_set_ocr(host->mmc, host->mmc->supply.vmmc, 0);
}

static const struct of_device_id bcm2153_match[] = {
	{ .compatible = "brcm,bcm2153-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_match);

static struct platform_driver bcm2153_driver = {
	.probe = bcm2153_probe,
	.remove = bcm2153_remove,
	.driver = {
		.name = "sdhci-bcm2153",
		.of_match_table = bcm2153_match,
	},
};
module_platform_driver(bcm2153_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 SDHCI driver");
MODULE_LICENSE("GPL");
