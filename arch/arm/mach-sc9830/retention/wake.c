// SPDX-License-Identifier: GPL-2.0-only
#include <linux/pm_wakeup.h>
#include <linux/rtc.h>

static int sc9830_retention_wake_deadline(time64_t interval, u32 ticks)
{
	if (interval <= 0 || ticks / 32768 <= 5 ||
	    interval > ticks / 32768 - 5)
		return -ETIME;
	return 0;
}

static int sc9830_retention_wake_check(struct regmap *map, struct device *pmic)
{
	struct rtc_device *rtc;
	struct rtc_wkalrm alarm;
	struct rtc_time now;
	u32 ticks;
	int ret;

	rtc = rtc_class_open("rtc0");
	if (!rtc)
		return -ENODEV;
	if (!rtc->dev.parent || rtc->dev.parent->parent != pmic ||
	    !device_may_wakeup(rtc->dev.parent) ||
	    !of_device_is_compatible(rtc->dev.parent->of_node, "sprd,sc2723-rtc")) {
		ret = -EINVAL;
		goto close;
	}
	ret = rtc_read_time(rtc, &now);
	if (!ret)
		ret = rtc_read_alarm(rtc, &alarm);
	if (!ret)
		ret = rtc_valid_tm(&alarm.time);
	if (!ret && !alarm.enabled)
		ret = -ETIME;
close:
	rtc_class_close(rtc);
	if (ret)
		return ret;
	ret = sc9830_retention_guard_check(map, &ticks);
	if (ret)
		return ret;
	return sc9830_retention_wake_deadline(rtc_tm_to_time64(&alarm.time) -
					     rtc_tm_to_time64(&now), ticks);
}
