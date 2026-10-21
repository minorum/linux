// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#define BCM2153_WDT_ENABLE	BIT(15)
#define BCM2153_WDT_MODE		(BIT(14) | BIT(12))
#define BCM2153_WDT_COUNT	0xff

struct bcm2153_wdt {
	void __iomem *base;
	struct watchdog_device wdd;
};

static int bcm2153_wdt_start(struct watchdog_device *wdd)
{
	struct bcm2153_wdt *wdt = watchdog_get_drvdata(wdd);

	if (!wdd->timeout || wdd->timeout > BCM2153_WDT_COUNT)
		return -EINVAL;

	writew(BCM2153_WDT_ENABLE | BCM2153_WDT_MODE | wdd->timeout, wdt->base);
	return 0;
}

static int bcm2153_wdt_stop(struct watchdog_device *wdd)
{
	struct bcm2153_wdt *wdt = watchdog_get_drvdata(wdd);

	writew(readw(wdt->base) & ~BCM2153_WDT_ENABLE, wdt->base);
	return 0;
}

static int bcm2153_wdt_set_timeout(struct watchdog_device *wdd, unsigned int seconds)
{
	if (!seconds || seconds > BCM2153_WDT_COUNT)
		return -EINVAL;

	wdd->timeout = seconds;
	if (watchdog_active(wdd) || watchdog_hw_running(wdd))
		return bcm2153_wdt_start(wdd);

	return 0;
}

static unsigned int bcm2153_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct bcm2153_wdt *wdt = watchdog_get_drvdata(wdd);

	return readw(wdt->base + 4) & BCM2153_WDT_COUNT;
}

static int bcm2153_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	struct bcm2153_wdt *wdt = watchdog_get_drvdata(wdd);

	writew(BCM2153_WDT_ENABLE | BCM2153_WDT_MODE | 1, wdt->base);
	mdelay(1000);
	return 0;
}

static void bcm2153_wdt_shutdown(struct platform_device *pdev)
{
	struct bcm2153_wdt *wdt = platform_get_drvdata(pdev);

	if (system_state == SYSTEM_HALT || system_state == SYSTEM_POWER_OFF)
		bcm2153_wdt_stop(&wdt->wdd);
}

static const struct watchdog_info bcm2153_wdt_info = {
	.identity = "BCM2153 AP watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops bcm2153_wdt_ops = {
	.owner = THIS_MODULE,
	.start = bcm2153_wdt_start,
	.stop = bcm2153_wdt_stop,
	.ping = bcm2153_wdt_start,
	.set_timeout = bcm2153_wdt_set_timeout,
	.get_timeleft = bcm2153_wdt_get_timeleft,
	.restart = bcm2153_wdt_restart,
};

static unsigned int timeout;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

static int bcm2153_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2153_wdt *wdt;
	struct clk *clk;
	u16 control;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	clk = devm_clk_get_enabled(dev, "slow");
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	if (clk_get_rate(clk) != 32768)
		return dev_err_probe(dev, -EINVAL, "unsupported slow clock\n");

	wdt->wdd.info = &bcm2153_wdt_info;
	wdt->wdd.ops = &bcm2153_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.timeout = 60;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = BCM2153_WDT_COUNT;
	wdt->wdd.max_hw_heartbeat_ms = BCM2153_WDT_COUNT * 1000;
	ret = watchdog_init_timeout(&wdt->wdd, timeout, dev);
	if (ret)
		return ret;

	control = readw(wdt->base);
	if (control & BCM2153_WDT_ENABLE) {
		if ((control & 0x7f00) != BCM2153_WDT_MODE ||
		    !(control & BCM2153_WDT_COUNT))
			return dev_err_probe(dev, -EINVAL, "unsupported running mode\n");
		wdt->wdd.timeout = control & BCM2153_WDT_COUNT;
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
	}
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_set_restart_priority(&wdt->wdd, 128);
	watchdog_stop_on_unregister(&wdt->wdd);
	platform_set_drvdata(pdev, wdt);
	return devm_watchdog_register_device(dev, &wdt->wdd);
}

static const struct of_device_id bcm2153_wdt_match[] = {
	{ .compatible = "brcm,bcm2153-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_wdt_match);

static struct platform_driver bcm2153_wdt_driver = {
	.probe = bcm2153_wdt_probe,
	.shutdown = bcm2153_wdt_shutdown,
	.driver = {
		.name = "bcm2153-wdt",
		.of_match_table = bcm2153_wdt_match,
	},
};
module_platform_driver(bcm2153_wdt_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 AP watchdog");
MODULE_LICENSE("GPL");
