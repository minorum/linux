// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

#define EFUSE_DATA	0x00
#define EFUSE_INDEX	0x08
#define EFUSE_MODE	0x0c
#define EFUSE_CFG	0x10
#define EFUSE_STATUS	0x20
#define EFUSE_READ	BIT(1)
#define EFUSE_K1		BIT(28)
#define EFUSE_K2		BIT(29)
#define EFUSE_VDD		BIT(30)
#define EFUSE_PROGRAM	BIT(31)

struct sc9830_efuse {
	void __iomem *base;
	struct clk *clk;
	struct hwspinlock *lock;
	struct mutex mutex; /* controller access */
};

static int sc9830_efuse_read(void *context, unsigned int offset,
			     void *buffer, size_t bytes)
{
	struct sc9830_efuse *efuse = context;
	u32 cfg, status;
	int ret;

	if (offset % 4 || bytes % 4 || offset >= 64 || bytes > 64 - offset)
		return -EINVAL;
	mutex_lock(&efuse->mutex);
	ret = hwspin_lock_timeout_raw(efuse->lock, 5000);
	if (ret)
		goto mutex_out;
	ret = clk_prepare_enable(efuse->clk);
	if (ret)
		goto lock_out;
	cfg = readl(efuse->base + EFUSE_CFG);
	if ((cfg & (EFUSE_K1 | EFUSE_PROGRAM | EFUSE_VDD)) ||
	    readl(efuse->base + EFUSE_STATUS)) {
		ret = -EBUSY;
		goto clock_out;
	}
	writel(cfg | EFUSE_VDD | EFUSE_K2, efuse->base + EFUSE_CFG);
	usleep_range(1000, 1200);
	while (bytes) {
		writel(offset / 4, efuse->base + EFUSE_INDEX);
		writel(EFUSE_READ, efuse->base + EFUSE_MODE);
		ret = readl_poll_timeout(efuse->base + EFUSE_STATUS, status,
					 !(status & BIT(1)), 10, 300000);
		if (ret)
			break;
		put_unaligned_le32(readl(efuse->base + EFUSE_DATA), buffer);
		buffer += 4;
		offset += 4;
		bytes -= 4;
	}
	writel(cfg | EFUSE_K2, efuse->base + EFUSE_CFG);
	usleep_range(1000, 1200);
clock_out:
	clk_disable_unprepare(efuse->clk);
lock_out:
	hwspin_unlock_raw(efuse->lock);
mutex_out:
	mutex_unlock(&efuse->mutex);
	return ret;
}

static int sc9830_efuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {};
	struct sc9830_efuse *efuse;
	int id;

	efuse = devm_kzalloc(dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;
	efuse->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(efuse->base))
		return PTR_ERR(efuse->base);
	efuse->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(efuse->clk))
		return dev_err_probe(dev, PTR_ERR(efuse->clk), "clock unavailable\n");
	id = of_hwspin_lock_get_id(dev->of_node, 0);
	if (id < 0)
		return dev_err_probe(dev, id, "hardware lock unavailable\n");
	efuse->lock = devm_hwspin_lock_request_specific(dev, id);
	if (!efuse->lock)
		return -EBUSY;
	mutex_init(&efuse->mutex);
	config.dev = dev;
	config.name = "sc9830-efuse";
	config.priv = efuse;
	config.size = 64;
	config.stride = 4;
	config.word_size = 4;
	config.read_only = true;
	config.root_only = true;
	config.reg_read = sc9830_efuse_read;
	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static const struct of_device_id sc9830_efuse_match[] = {
	{ .compatible = "sprd,sc9830-efuse" },
	{}
};
MODULE_DEVICE_TABLE(of, sc9830_efuse_match);

static struct platform_driver sc9830_efuse_driver = {
	.probe = sc9830_efuse_probe,
	.driver = {
		.name = "sc9830-efuse",
		.of_match_table = sc9830_efuse_match,
	},
};
module_platform_driver(sc9830_efuse_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 read-only AP efuse");
MODULE_LICENSE("GPL");
