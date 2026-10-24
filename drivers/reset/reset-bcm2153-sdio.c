// SPDX-License-Identifier: GPL-2.0-only

#include <linux/align.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#define BCM2153_SDIO_RESET	BIT(1)

struct bcm2153_sdio_reset {
	void __iomem *base;
	/* Pulse serialization */
	struct mutex lock;
	struct reset_controller_dev rcdev;
};

static int bcm2153_sdio_reset_pulse(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct bcm2153_sdio_reset *reset =
		container_of(rcdev, struct bcm2153_sdio_reset, rcdev);

	if (id)
		return -EINVAL;

	mutex_lock(&reset->lock);
	writel(readl(reset->base) | BCM2153_SDIO_RESET, reset->base);
	readl(reset->base);
	fsleep(10000);
	writel(readl(reset->base) & ~BCM2153_SDIO_RESET, reset->base);
	readl(reset->base);
	mutex_unlock(&reset->lock);

	return 0;
}

static int bcm2153_sdio_reset_xlate(struct reset_controller_dev *rcdev,
				    const struct of_phandle_args *args)
{
	return args->args_count ? -EINVAL : 0;
}

static const struct reset_control_ops bcm2153_sdio_reset_ops = {
	.reset = bcm2153_sdio_reset_pulse,
};

static int bcm2153_sdio_reset_probe(struct platform_device *pdev)
{
	struct bcm2153_sdio_reset *reset;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) != sizeof(u32) ||
	    !IS_ALIGNED(res->start, sizeof(u32)))
		return -EINVAL;

	reset = devm_kzalloc(&pdev->dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;
	reset->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reset->base))
		return PTR_ERR(reset->base);
	mutex_init(&reset->lock);
	reset->rcdev.owner = THIS_MODULE;
	reset->rcdev.dev = &pdev->dev;
	reset->rcdev.of_node = pdev->dev.of_node;
	reset->rcdev.of_reset_n_cells = 0;
	reset->rcdev.of_xlate = bcm2153_sdio_reset_xlate;
	reset->rcdev.nr_resets = 1;
	reset->rcdev.ops = &bcm2153_sdio_reset_ops;

	return devm_reset_controller_register(&pdev->dev, &reset->rcdev);
}

static const struct of_device_id bcm2153_sdio_reset_of_match[] = {
	{ .compatible = "brcm,bcm2153-sdio-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_sdio_reset_of_match);

static struct platform_driver bcm2153_sdio_reset_driver = {
	.probe = bcm2153_sdio_reset_probe,
	.driver = {
		.name = "bcm2153-sdio-reset",
		.of_match_table = bcm2153_sdio_reset_of_match,
	},
};
module_platform_driver(bcm2153_sdio_reset_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 SDIO reset controller");
MODULE_LICENSE("GPL");
