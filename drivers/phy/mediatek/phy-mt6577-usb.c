// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct mt6577_usb_phy {
	void __iomem *base;
	struct clk *upll;
	struct clk *bus;
	struct regulator *vusb;
	bool failed;
};

static void mt6577_usb_update(struct mt6577_usb_phy *usb, unsigned int reg,
			      u8 mask, u8 value)
{
	writeb((readb(usb->base + reg) & ~mask) | value, usb->base + reg);
}

static int mt6577_usb_calibrate(struct mt6577_usb_phy *usb)
{
	u8 saved = readb(usb->base + 0x12);
	u8 control = readb(usb->base + 0x703);
	u8 cycles = readb(usb->base + 0x701);
	u8 free_run = readb(usb->base + 0x711);
	u8 ready;
	u32 result, value;
	int ret;

	if ((saved & BIT(7)) || (control & 1) || (free_run & 1))
		return -EBUSY;
	writeb(0x81, usb->base + 0x12);
	writeb(1, usb->base + 0x711);
	writeb(4, usb->base + 0x701);
	writeb(control | 1, usb->base + 0x703);
	if (readb(usb->base + 0x710) & 1) {
		ret = -EBUSY;
		goto restore;
	}
	ret = readb_poll_timeout(usb->base + 0x710, ready, ready & 1, 10, 3000000);
	if (ret)
		goto restore;
	result = readl(usb->base + 0x70c);
	if (!result) {
		ret = -ERANGE;
		goto restore;
	}
	value = (1024 * 48 * 28) / result;
	value = value / 1000 + (value % 1000 >= 500);
	/* Qualification bound, not a field mask */
	if (!value || value > 7) {
		ret = -ERANGE;
		goto restore;
	}
restore:
	writeb(control, usb->base + 0x703);
	writeb(cycles, usb->base + 0x701);
	writeb(free_run, usb->base + 0x711);
	writeb(ret ? saved : value, usb->base + 0x12);
	return ret;
}

static void mt6577_usb_save_current(struct mt6577_usb_phy *usb)
{
	mt6577_usb_update(usb, 0x6b, 0x04, 0);
	mt6577_usb_update(usb, 0x6e, 0x01, 0);
	mt6577_usb_update(usb, 0x6a, 0x04, 0);
	mt6577_usb_update(usb, 0x68, 0xc0, 0xc0);
	mt6577_usb_update(usb, 0x68, 0x30, 0x10);
	mt6577_usb_update(usb, 0x68, 0x04, 0x04);
	mt6577_usb_update(usb, 0x69, 0x3c, 0);
	mt6577_usb_update(usb, 0x6a, 0x3a, 0x3a);
	mt6577_usb_update(usb, 0x1c, 0x80, 0);
	mt6577_usb_update(usb, 0x1b, 0x08, 0);
	usleep_range(800, 1000);
	mt6577_usb_update(usb, 0x63, 0x02, 0x02);
	udelay(1);
	mt6577_usb_update(usb, 0x6a, 0x04, 0x04);
	udelay(1);
}

static int mt6577_usb_power_on(struct phy *phy)
{
	struct mt6577_usb_phy *usb = phy_get_drvdata(phy);
	bool bus_on = false;
	int ret, cleanup;

	if (usb->failed)
		return -EIO;
	if (clk_get_rate(usb->bus) != 61000000)
		return -EINVAL;
	ret = clk_prepare_enable(usb->upll);
	if (ret)
		return ret;
	ret = regulator_enable(usb->vusb);
	if (ret)
		goto disable_upll;
	usleep_range(10000, 11000);
	ret = clk_prepare_enable(usb->bus);
	if (ret)
		goto disable_supply;
	bus_on = true;
	usleep_range(50, 100);
	mt6577_usb_update(usb, 0x6b, 0x04, 0);
	mt6577_usb_update(usb, 0x6e, 0x01, 0);
	mt6577_usb_update(usb, 0x1c, 0x80, 0);
	mt6577_usb_update(usb, 0x02, 0x7f, 0x09);
	mt6577_usb_update(usb, 0x22, 0x03, 0);
	mt6577_usb_update(usb, 0x6a, 0x04, 0);
	mt6577_usb_update(usb, 0x1b, 0x08, 0x08);
	usleep_range(800, 1000);
	mt6577_usb_update(usb, 0x68, 0xf4, 0);
	mt6577_usb_update(usb, 0x69, 0x3c, 0);
	mt6577_usb_update(usb, 0x6a, 0xbe, 0);
	usleep_range(800, 1000);
	ret = mt6577_usb_calibrate(usb);
	if (!ret)
		return 0;
	mt6577_usb_save_current(usb);
disable_supply:
	cleanup = regulator_disable(usb->vusb);
	if (cleanup) {
		usb->failed = true;
		return cleanup;
	}
	if (bus_on)
		clk_disable_unprepare(usb->bus);
disable_upll:
	clk_disable_unprepare(usb->upll);
	return ret;
}

static int mt6577_usb_power_off(struct phy *phy)
{
	struct mt6577_usb_phy *usb = phy_get_drvdata(phy);
	int ret;

	mt6577_usb_save_current(usb);
	ret = regulator_disable(usb->vusb);
	if (ret) {
		usb->failed = true;
		return ret;
	}
	clk_disable_unprepare(usb->bus);
	clk_disable_unprepare(usb->upll);
	return 0;
}

static int mt6577_usb_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct mt6577_usb_phy *usb = phy_get_drvdata(phy);

	if (usb->failed)
		return -EIO;
	return mode == PHY_MODE_USB_DEVICE && !submode ? 0 : -EINVAL;
}

static const struct phy_ops mt6577_usb_ops = {
	.power_on = mt6577_usb_power_on,
	.power_off = mt6577_usb_power_off,
	.set_mode = mt6577_usb_set_mode,
	.owner = THIS_MODULE,
};

static int mt6577_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6577_usb_phy *usb;
	struct phy_provider *provider;
	struct phy *phy;

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;
	usb->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usb->base))
		return PTR_ERR(usb->base);
	usb->upll = devm_clk_get(dev, "upll");
	if (IS_ERR(usb->upll))
		return dev_err_probe(dev, PTR_ERR(usb->upll), "Missing UPLL\n");
	usb->bus = devm_clk_get(dev, "bus");
	if (IS_ERR(usb->bus))
		return dev_err_probe(dev, PTR_ERR(usb->bus), "Missing bus clock\n");
	usb->vusb = devm_regulator_get(dev, "vusb33");
	if (IS_ERR(usb->vusb))
		return dev_err_probe(dev, PTR_ERR(usb->vusb), "Missing VUSB supply\n");
	phy = devm_phy_create(dev, NULL, &mt6577_usb_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, usb);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id mt6577_usb_phy_match[] = {
	{ .compatible = "mediatek,mt6577-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6577_usb_phy_match);

static struct platform_driver mt6577_usb_phy_driver = {
	.probe = mt6577_usb_phy_probe,
	.driver = {
		.name = "mt6577-usb-phy",
		.of_match_table = mt6577_usb_phy_match,
	},
};
module_platform_driver(mt6577_usb_phy_driver);

MODULE_DESCRIPTION("MediaTek MT6577 USB PHY");
MODULE_LICENSE("GPL");
