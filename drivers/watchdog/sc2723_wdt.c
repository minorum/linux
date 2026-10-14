// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/suspend.h>
#include <linux/watchdog.h>

struct sc2723_wdt {
	struct watchdog_device wdd;
	struct regmap *map;
	/* Watchdog ownership. */
	struct mutex lock;
	bool owned;
	bool fault;
};

static int sc2723_wdt_program(struct sc2723_wdt *wdt, unsigned int seconds, bool start)
{
	unsigned int control, value, ticks = seconds * 32768;
	int ret, lock_ret;

	if (seconds < 3 || seconds > 180)
		return -EINVAL;
	if (wdt->fault)
		return -EIO;
	ret = regmap_read(wdt->map, 0x48, &control);
	if (ret)
		return ret;
	if (start && !wdt->owned && (control & 0xb))
		return -EBUSY;
	if (!start && (!wdt->owned || control != 0xe))
		return -EINVAL;
	if (start) {
		ret = regmap_update_bits(wdt->map, 0x800, BIT(2), BIT(2));
		if (ret)
			return ret;
		ret = regmap_update_bits(wdt->map, 0x808, BIT(2), BIT(2));
		if (ret)
			return ret;
	}
	ret = regmap_write(wdt->map, 0x60, 0xe551);
	if (ret)
		return ret;
	wdt->owned = true;
	if (start) {
		ret = regmap_update_bits(wdt->map, 0x48, GENMASK(3, 0), BIT(2));
		if (ret)
			goto relock;
	}
	ret = regmap_write(wdt->map, 0x44, ticks >> 16);
	if (ret)
		goto relock;
	ret = regmap_write(wdt->map, 0x40, ticks & 0xffff);
	if (ret)
		goto relock;
	ret = regmap_read_poll_timeout(wdt->map, 0x50, value,
				       !(value & BIT(4)), 10, 10000);
	if (ret)
		goto relock;
	if (start)
		ret = regmap_update_bits(wdt->map, 0x48, GENMASK(3, 0), 0xe);
relock:
	lock_ret = regmap_write(wdt->map, 0x60, 0x1aae);
	if (!ret)
		ret = lock_ret;
	if (!ret) {
		ret = regmap_read(wdt->map, 0x48, &value);
		if (!ret && value != (start ? 0xe : control))
			ret = -EIO;
	}
	if (ret)
		wdt->fault = true;
	return ret;
}

static int sc2723_wdt_start(struct watchdog_device *wdd)
{
	struct sc2723_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	mutex_lock(&wdt->lock);
	ret = sc2723_wdt_program(wdt, wdd->timeout, !wdt->owned);
	mutex_unlock(&wdt->lock);
	return ret;
}

static int sc2723_wdt_ping(struct watchdog_device *wdd)
{
	struct sc2723_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	mutex_lock(&wdt->lock);
	ret = sc2723_wdt_program(wdt, wdd->timeout, false);
	mutex_unlock(&wdt->lock);
	return ret;
}

static int sc2723_wdt_stop(struct watchdog_device *wdd)
{
	struct sc2723_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int control;
	int ret, lock_ret;

	mutex_lock(&wdt->lock);
	if (!wdt->owned || wdt->fault) {
		ret = wdt->fault ? -EIO : -EINVAL;
		goto out;
	}
	ret = regmap_write(wdt->map, 0x60, 0xe551);
	if (ret)
		goto out;
	ret = regmap_update_bits(wdt->map, 0x48, BIT(1) | BIT(3), 0);
	lock_ret = regmap_write(wdt->map, 0x60, 0x1aae);
	if (!ret)
		ret = lock_ret;
	if (!ret)
		ret = regmap_read(wdt->map, 0x48, &control);
	if (!ret && (control & (BIT(1) | BIT(3))))
		ret = -EIO;
	if (!ret)
		wdt->owned = false;
	else
		wdt->fault = true;
out:
	mutex_unlock(&wdt->lock);
	return ret;
}

static int sc2723_wdt_set_timeout(struct watchdog_device *wdd, unsigned int seconds)
{
	struct sc2723_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret = 0;

	mutex_lock(&wdt->lock);
	if (seconds < 3 || seconds > 180)
		ret = -EINVAL;
	else if (wdt->owned)
		ret = sc2723_wdt_program(wdt, seconds, false);
	if (!ret)
		wdd->timeout = seconds;
	mutex_unlock(&wdt->lock);
	return ret;
}

static const struct watchdog_info sc2723_wdt_info = {
	.identity = "SC2723 watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops sc2723_wdt_ops = {
	.owner = THIS_MODULE,
	.start = sc2723_wdt_start,
	.stop = sc2723_wdt_stop,
	.ping = sc2723_wdt_ping,
	.set_timeout = sc2723_wdt_set_timeout,
};

static int sc2723_wdt_suspend(struct device *dev)
{
	struct sc2723_wdt *wdt = dev_get_drvdata(dev);
	struct rtc_device *rtc;
	struct rtc_wkalrm alarm;
	struct rtc_time now;
	time64_t interval;
	int ret;

	mutex_lock(&wdt->lock);
	ret = wdt->fault || wdt->owned != watchdog_active(&wdt->wdd) ? -EIO : 0;
	mutex_unlock(&wdt->lock);
	if (ret)
		return ret;
	if (!watchdog_active(&wdt->wdd))
		return 0;
	if (pm_suspend_target_state == PM_SUSPEND_MEM) {
		if (wdt->wdd.timeout <= 5)
			return -ETIME;
		rtc = rtc_class_open("rtc0");
		if (!rtc)
			return -ENODEV;
		if (!device_may_wakeup(rtc->dev.parent) ||
		    !of_device_is_compatible(rtc->dev.parent->of_node, "sprd,sc2723-rtc")) {
			rtc_class_close(rtc);
			return -EINVAL;
		}
		ret = rtc_read_time(rtc, &now);
		if (!ret)
			ret = rtc_read_alarm(rtc, &alarm);
		rtc_class_close(rtc);
		if (ret)
			return ret;
		interval = rtc_tm_to_time64(&alarm.time) - rtc_tm_to_time64(&now);
		if (!alarm.enabled || interval <= 0 || interval > wdt->wdd.timeout - 5)
			return -ETIME;
	}
	return sc2723_wdt_ping(&wdt->wdd);
}

static int sc2723_wdt_resume(struct device *dev)
{
	struct sc2723_wdt *wdt = dev_get_drvdata(dev);

	return watchdog_active(&wdt->wdd) ? sc2723_wdt_ping(&wdt->wdd) : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc2723_wdt_pm, sc2723_wdt_suspend, sc2723_wdt_resume);

static int sc2723_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc2723_wdt *wdt;
	unsigned int id, control, base;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	ret = of_property_read_u32(dev->of_node, "reg", &base);
	if (ret || base != 0x40)
		return ret ?: -EINVAL;
	wdt->map = dev_get_regmap(dev->parent, NULL);
	if (!wdt->map)
		return -ENODEV;
	ret = regmap_read(wdt->map, 0x940, &id);
	if (ret || id != 0x2723)
		return ret ?: -ENODEV;
	ret = regmap_read(wdt->map, 0x48, &control);
	if (ret || (control & 0xb))
		return ret ?: -EBUSY;
	mutex_init(&wdt->lock);
	wdt->wdd.info = &sc2723_wdt_info;
	wdt->wdd.ops = &sc2723_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 3;
	wdt->wdd.max_timeout = 180;
	wdt->wdd.timeout = 180;
	ret = watchdog_init_timeout(&wdt->wdd, 0, dev);
	if (ret)
		return ret;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_set_nowayout(&wdt->wdd, WATCHDOG_NOWAYOUT);
	watchdog_stop_ping_on_suspend(&wdt->wdd);
	watchdog_stop_on_unregister(&wdt->wdd);
	platform_set_drvdata(pdev, wdt);
	return devm_watchdog_register_device(dev, &wdt->wdd);
}

static const struct of_device_id sc2723_wdt_match[] = {
	{ .compatible = "sprd,sc2723-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2723_wdt_match);

static struct platform_driver sc2723_wdt_driver = {
	.probe = sc2723_wdt_probe,
	.driver = {
		.name = "sc2723-wdt",
		.of_match_table = sc2723_wdt_match,
		.pm = pm_sleep_ptr(&sc2723_wdt_pm),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sc2723_wdt_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SC2723 analog watchdog owner");
