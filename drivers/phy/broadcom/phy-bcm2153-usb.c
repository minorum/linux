// SPDX-License-Identifier: GPL-2.0-only

#include <linux/align.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct bcm2153_usb_phy {
	void __iomem *wrapper;
	void __iomem *analog;
};

static int bcm2153_usb_phy_power_on(struct phy *phy)
{
	struct bcm2153_usb_phy *usb = phy_get_drvdata(phy);

	writel(readl(usb->wrapper) | 0x8000, usb->wrapper);
	writel(readl(usb->wrapper) & ~0xc00U, usb->wrapper);
	writel(readl(usb->wrapper) | 0x18c00, usb->wrapper);
	writel(0x4000007f, usb->analog);
	writel(0x4000005f, usb->analog);
	writel(0x4000007f, usb->analog);
	return 0;
}

static int bcm2153_usb_phy_power_off(struct phy *phy)
{
	struct bcm2153_usb_phy *usb = phy_get_drvdata(phy);

	writel(readl(usb->wrapper) & ~0x8000U, usb->wrapper);
	return 0;
}

static const struct phy_ops bcm2153_usb_phy_ops = {
	.power_on = bcm2153_usb_phy_power_on,
	.power_off = bcm2153_usb_phy_power_off,
	.owner = THIS_MODULE,
};

static int bcm2153_usb_phy_probe(struct platform_device *pdev)
{
	static const char * const names[] = { "wrapper", "analog" };
	struct device *dev = &pdev->dev;
	struct bcm2153_usb_phy *usb;
	struct phy_provider *provider;
	struct phy *phy;
	struct resource *res;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, names[i]);
		if (!res || resource_size(res) < sizeof(u32) ||
		    !IS_ALIGNED(res->start, sizeof(u32)))
			return -EINVAL;
	}

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;
	usb->wrapper = devm_platform_ioremap_resource_byname(pdev, "wrapper");
	if (IS_ERR(usb->wrapper))
		return PTR_ERR(usb->wrapper);
	usb->analog = devm_platform_ioremap_resource_byname(pdev, "analog");
	if (IS_ERR(usb->analog))
		return PTR_ERR(usb->analog);
	phy = devm_phy_create(dev, NULL, &bcm2153_usb_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, usb);
	phy_set_bus_width(phy, 8);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id bcm2153_usb_phy_of_match[] = {
	{ .compatible = "brcm,bcm2153-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_usb_phy_of_match);

static struct platform_driver bcm2153_usb_phy_driver = {
	.probe = bcm2153_usb_phy_probe,
	.driver = {
		.name = "bcm2153-usb-phy",
		.of_match_table = bcm2153_usb_phy_of_match,
	},
};
module_platform_driver(bcm2153_usb_phy_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 USB PHY");
MODULE_LICENSE("GPL");
