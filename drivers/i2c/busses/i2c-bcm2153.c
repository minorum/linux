// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define BSC_CS		0x00
#define BSC_TIM		0x04
#define BSC_DAT		0x08
#define BSC_ISR		0x28
#define BSC_CLKEN	0x2c
#define BSC_BUSY		0x80
#define BSC_COMPLETE	0x0c
#define BSC_TIMEOUT_US	10000

struct bcm2153_bsc {
	void __iomem *base;
	struct i2c_adapter adapter;
	bool failed;
};

static int bcm2153_idle(struct bcm2153_bsc *bsc)
{
	u8 status;

	return readb_poll_timeout(bsc->base + BSC_ISR, status,
				 !(status & BSC_BUSY), 1, BSC_TIMEOUT_US);
}

static int bcm2153_operation(struct bcm2153_bsc *bsc, u8 value, bool data)
{
	u8 status;
	int ret;

	writeb(0x8f, bsc->base + BSC_ISR);
	ret = bcm2153_idle(bsc);
	if (ret)
		return ret;
	if (data)
		writeb(value, bsc->base + BSC_DAT);
	else
		writeb((readb(bsc->base + BSC_CS) & (value == 4 ? 0xf9 : 0xf1)) |
		       value, bsc->base + BSC_CS);
	ret = readb_poll_timeout(bsc->base + BSC_ISR, status,
				 status & BSC_COMPLETE, 1, BSC_TIMEOUT_US);
	if (ret)
		return ret;
	writeb(0x8f, bsc->base + BSC_ISR);
	writeb(readb(bsc->base + BSC_CS) & 0xf9, bsc->base + BSC_CS);
	ret = bcm2153_idle(bsc);
	if (ret)
		return ret;
	if (status & 4)
		return -EIO;
	if (status & 1)
		return -EREMOTEIO;
	usleep_range(31, 62);
	return 0;
}

static int bcm2153_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct bcm2153_bsc *bsc = i2c_get_adapdata(adapter);
	int i, j, ret;

	if (num < 0 || (num && !msgs))
		return -EINVAL;
	for (i = 0; i < num; i++) {
		if (msgs[i].addr > 0x7f || (msgs[i].flags & ~(I2C_M_RD | I2C_M_STOP)) ||
		    (i + 1 < num && !(msgs[i].flags & I2C_M_STOP)))
			return -EOPNOTSUPP;
		if (msgs[i].len && !msgs[i].buf)
			return -EINVAL;
	}
	if (bsc->failed)
		return -ESHUTDOWN;
	for (i = 0; i < num; i++) {
		ret = bcm2153_operation(bsc, 2, false);
		if (ret)
			goto failed;
		ret = bcm2153_operation(bsc, msgs[i].addr << 1 |
					!!(msgs[i].flags & I2C_M_RD), true);
		if (ret)
			goto failed;
		for (j = 0; j < msgs[i].len; j++) {
			if (msgs[i].flags & I2C_M_RD) {
				ret = bcm2153_operation(bsc, j + 1 == msgs[i].len ? 0x0e : 6,
							false);
				if (!ret)
					msgs[i].buf[j] = readb(bsc->base + BSC_DAT);
			} else {
				ret = bcm2153_operation(bsc, msgs[i].buf[j], true);
			}
			if (ret)
				goto failed;
		}
		ret = bcm2153_operation(bsc, 4, false);
		if (ret)
			goto failed;
	}
	return num;

failed:
	bsc->failed = true;
	return ret;
}

static u32 bcm2153_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm bcm2153_algorithm = {
	.xfer = bcm2153_xfer,
	.functionality = bcm2153_functionality,
};

static const struct i2c_adapter_quirks bcm2153_quirks = {
	.flags = I2C_AQ_NO_REP_START,
};

static int bcm2153_bsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2153_bsc *bsc;
	int ret;

	bsc = devm_kzalloc(dev, sizeof(*bsc), GFP_KERNEL);
	if (!bsc)
		return -ENOMEM;
	bsc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bsc->base))
		return PTR_ERR(bsc->base);
	if (readb(bsc->base + BSC_TIM) != 0x11 ||
	    readb(bsc->base + BSC_CLKEN) != 0x13 || !(readb(bsc->base + BSC_CS) & 1))
		return dev_err_probe(dev, -EOPNOTSUPP, "unsupported firmware BSC state\n");
	ret = bcm2153_idle(bsc);
	if (ret)
		return ret;
	bsc->adapter.owner = THIS_MODULE;
	bsc->adapter.algo = &bcm2153_algorithm;
	bsc->adapter.quirks = &bcm2153_quirks;
	bsc->adapter.dev.parent = dev;
	device_set_node(&bsc->adapter.dev, dev_fwnode(dev));
	strscpy(bsc->adapter.name, "BCM2153 BSC");
	i2c_set_adapdata(&bsc->adapter, bsc);
	return devm_i2c_add_adapter(dev, &bsc->adapter);
}

static const struct of_device_id bcm2153_bsc_match[] = {
	{ .compatible = "brcm,bcm2153-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_bsc_match);

static struct platform_driver bcm2153_bsc_driver = {
	.probe = bcm2153_bsc_probe,
	.driver = {
		.name = "i2c-bcm2153",
		.of_match_table = bcm2153_bsc_match,
	},
};
module_platform_driver(bcm2153_bsc_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 firmware-initialized BSC");
MODULE_LICENSE("GPL");
