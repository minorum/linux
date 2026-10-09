// SPDX-License-Identifier: GPL-2.0-only
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot-mode.h>

struct sprd_reboot_mode {
	struct reboot_mode_driver reboot;
	void __iomem *mailbox;
};

static int sprd_reboot_mode_write(struct reboot_mode_driver *reboot,
				  unsigned int magic)
{
	struct sprd_reboot_mode *mode =
		container_of(reboot, struct sprd_reboot_mode, reboot);

	writel(magic, mode->mailbox);
	if (readl(mode->mailbox) != magic)
		return -EIO;

	return 0;
}

static int sprd_reboot_mode_probe(struct platform_device *pdev)
{
	struct sprd_reboot_mode *mode;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) != sizeof(u32))
		return -EINVAL;

	mode = devm_kzalloc(&pdev->dev, sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return -ENOMEM;

	mode->mailbox = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mode->mailbox))
		return PTR_ERR(mode->mailbox);

	mode->reboot.dev = &pdev->dev;
	mode->reboot.write = sprd_reboot_mode_write;
	return devm_reboot_mode_register(&pdev->dev, &mode->reboot);
}

static const struct of_device_id sprd_reboot_mode_of_match[] = {
	{ .compatible = "sprd,sc9830-reboot-mode" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_reboot_mode_of_match);

static struct platform_driver sprd_reboot_mode_driver = {
	.probe = sprd_reboot_mode_probe,
	.driver = {
		.name = "sprd-reboot-mode",
		.of_match_table = sprd_reboot_mode_of_match,
	},
};
module_platform_driver(sprd_reboot_mode_driver);

MODULE_DESCRIPTION("Spreadtrum SRAM reboot mode mailbox");
MODULE_LICENSE("GPL");
