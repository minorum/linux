// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "sdhci-pltfm.h"

static void sc9830_writeb(struct sdhci_host *host, u8 val, int reg)
{
	if (reg == SDHCI_SOFTWARE_RESET)
		val |= BIT(3);
	writeb(val, host->ioaddr + reg);
}

static void sc9830_reset(struct sdhci_host *host, u8 mask)
{
	sdhci_reset(host, mask);
	if (mask & SDHCI_RESET_ALL) {
		writel(0x34, host->ioaddr + 0x80);
		writel(8, host->ioaddr + 0x84);
		writel(8, host->ioaddr + 0x88);
	}
}

static void sc9830_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u32 rate = sdhci_pltfm_clk_get_max_clock(host);
	u32 div;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	host->mmc->actual_clock = 0;
	if (!clock)
		return;
	div = min_t(u32, DIV_ROUND_UP(rate, 2 * clock) - 1, 1023);
	host->mmc->actual_clock = rate / (2 * (div + 1));
	sdhci_enable_clk(host, ((div & 0x300) >> 2) | ((div & 0xff) << 8));
}

static void sc9830_set_power(struct sdhci_host *host, unsigned char mode,
			     unsigned short vdd)
{
}

static const struct sdhci_ops sc9830_ops = {
	.write_b = sc9830_writeb,
	.set_clock = sc9830_set_clock,
	.set_power = sc9830_set_power,
	.set_bus_width = sdhci_set_bus_width,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.reset = sc9830_reset,
};

static const struct sdhci_pltfm_data sc9830_data = {
	.ops = &sc9830_ops,
	.quirks = SDHCI_QUIRK_BROKEN_DMA | SDHCI_QUIRK_BROKEN_ADMA |
		  SDHCI_QUIRK_FORCE_1_BIT_DATA | SDHCI_QUIRK_NO_MULTIBLOCK |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int sc9830_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm;
	int ret;

	host = sdhci_pltfm_init(pdev, &sc9830_data, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm = sdhci_priv(host);
	pltfm->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(pltfm->clk))
		return PTR_ERR(pltfm->clk);
	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;
	sdhci_get_property(pdev);
	sdhci_read_caps(host);
	host->caps &= ~(SDHCI_CAN_DO_HISPD | SDHCI_CAN_DO_8BIT);
	host->caps1 = 0;
	host->mmc->ocr_avail = MMC_VDD_30_31;
	return sdhci_add_host(host);
}

static const struct of_device_id sc9830_match[] = {
	{ .compatible = "sprd,sc9830-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_match);

static struct platform_driver sc9830_driver = {
	.probe = sc9830_probe,
	.remove = sdhci_pltfm_remove,
	.driver = {
		.name = "sdhci-sc9830",
		.of_match_table = sc9830_match,
	},
};
module_platform_driver(sc9830_driver);

MODULE_DESCRIPTION("SC9830 SDHCI RAM bring-up");
MODULE_LICENSE("GPL");
