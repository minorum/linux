// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define DUAL_DATA	0x00
#define DUAL_ADDR	0x04
#define DUAL_MASK	0x08
#define DUAL_STATUS	0x0c
#define DUAL_CONTROL	0x10
#define DUAL_LENGTH	0x14
#define DUAL_COUNT	0x18
#define DUAL_TIMING	0x20
#define DUAL_START	0x24
#define DUAL_FIFO_STATUS 0x30
#define DUAL_FIFO_CLEAR	0x38
#define DUAL_IO		0x40
#define DUAL_HS		0x48

struct mt8377_dual {
	struct i2c_adapter adapter;
	void __iomem *base;
	struct clk *clock;
	bool push_pull;
};

static int mt8377_dual_timing(unsigned long rate)
{
	u64 step;
	unsigned int sample;

	if (!rate)
		return -EINVAL;
	for (sample = 1; sample <= 8; sample++) {
		step = DIV_ROUND_UP_ULL(rate, 3200000ULL * sample);
		if (step <= 64)
			return ((sample - 1) << 8) | (step - 1);
	}
	return -ERANGE;
}

static u16 mt8377_dual_control(u16 value, bool reading)
{
	return (value & ~0x7e) | (reading ? 0x3a : 0x28);
}

static s32 mt8377_dual_smbus(struct i2c_adapter *adapter, u16 address,
			     unsigned short flags, char direction, u8 command,
			     int protocol, union i2c_smbus_data *data)
{
	struct mt8377_dual *dual = i2c_get_adapdata(adapter);
	void __iomem *base = dual->base;
	bool reading = direction == I2C_SMBUS_READ;
	u16 status, value;
	int timing, ret;

	if (flags || protocol != I2C_SMBUS_BYTE_DATA || !data)
		return -EOPNOTSUPP;
	if (address < 0x08 || address > 0x77 ||
	    (direction != I2C_SMBUS_READ && direction != I2C_SMBUS_WRITE))
		return -EINVAL;
	ret = clk_prepare_enable(dual->clock);
	if (ret)
		return ret;
	if ((readw(base + DUAL_MASK) & 7) || (readw(base + DUAL_STATUS) & 7) ||
	    (readl(base + DUAL_START) & 1) || !(readw(base + DUAL_FIFO_STATUS) & 1)) {
		ret = -EBUSY;
		goto disable_clock;
	}
	timing = mt8377_dual_timing(clk_get_rate(dual->clock));
	if (timing < 0) {
		ret = timing;
		goto disable_clock;
	}
	value = readw(base + DUAL_TIMING);
	writew((value & ~0x73f) | timing, base + DUAL_TIMING);
	writew(dual->push_pull ? 0 : 3, base + DUAL_IO);
	writew(readw(base + DUAL_HS) & ~1, base + DUAL_HS);
	writew(mt8377_dual_control(readw(base + DUAL_CONTROL), reading),
	       base + DUAL_CONTROL);
	writew(address << 1, base + DUAL_ADDR);
	value = readw(base + DUAL_LENGTH) & ~0x1fff;
	writew(value | (reading ? 0x101 : 2), base + DUAL_LENGTH);
	writew(reading ? 2 : 1, base + DUAL_COUNT);
	writew(1, base + DUAL_FIFO_CLEAR);
	writeb(command, base + DUAL_DATA);
	if (!reading)
		writeb(data->byte, base + DUAL_DATA);
	writel(1, base + DUAL_START);
	ret = readw_poll_timeout(base + DUAL_STATUS, status, status & 7, 20, 100000);
	if (!ret && (status & 6))
		ret = -ENXIO;
	if (!ret && reading) {
		if (readw(base + DUAL_FIFO_STATUS) & 1)
			ret = -EIO;
		else
			data->byte = readb(base + DUAL_DATA);
	}
	writew(status & 7, base + DUAL_STATUS);
	/* shortcut: shared reset unqualified, revisit with ownership proof */
disable_clock:
	clk_disable_unprepare(dual->clock);
	return ret;
}

static u32 mt8377_dual_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_BYTE_DATA;
}

static const struct i2c_algorithm mt8377_dual_algorithm = {
	.smbus_xfer = mt8377_dual_smbus,
	.functionality = mt8377_dual_functionality,
};

static int mt8377_dual_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt8377_dual *dual;

	dual = devm_kzalloc(dev, sizeof(*dual), GFP_KERNEL);
	if (!dual)
		return -ENOMEM;
	dual->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dual->base))
		return PTR_ERR(dual->base);
	dual->clock = devm_clk_get(dev, "main");
	if (IS_ERR(dual->clock))
		return dev_err_probe(dev, PTR_ERR(dual->clock), "Missing controller clock\n");
	dual->push_pull = device_property_read_bool(dev, "mediatek,use-push-pull");
	dual->adapter.owner = THIS_MODULE;
	dual->adapter.algo = &mt8377_dual_algorithm;
	dual->adapter.dev.parent = dev;
	dual->adapter.dev.of_node = dev->of_node;
	strscpy(dual->adapter.name, "MT8377 dual I2C channel1", sizeof(dual->adapter.name));
	i2c_set_adapdata(&dual->adapter, dual);
	return devm_i2c_add_adapter(dev, &dual->adapter);
}

static const struct of_device_id mt8377_dual_match[] = {
	{ .compatible = "mediatek,mt8377-i2c-dual" },
	{ .compatible = "mediatek,mt6577-i2c-dual" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8377_dual_match);

static struct platform_driver mt8377_dual_driver = {
	.probe = mt8377_dual_probe,
	.driver = {
		.name = "mt8377-i2c-dual",
		.of_match_table = mt8377_dual_match,
	},
};
module_platform_driver(mt8377_dual_driver);

MODULE_DESCRIPTION("MediaTek MT8377 dual-controller SMBus adapter");
MODULE_LICENSE("GPL");
