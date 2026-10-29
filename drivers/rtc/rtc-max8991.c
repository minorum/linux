// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/string.h>

#define MAX8991_TIME_REG	0x00
#define MAX8991_TIME_SIZE	8
#define MAX8991_READ_TRIES	4

static int max8991_decode(const u8 *data, struct rtc_time *tm)
{
	unsigned int i;
	int ret;

	if (data[2] & BIT(7))
		return -EOPNOTSUPP;
	for (i = 0; i < MAX8991_TIME_SIZE; i++) {
		if (i == 3)
			continue;
		if ((data[i] & 0x0f) > 9 || (data[i] >> 4) > 9)
			return -EINVAL;
	}

	tm->tm_sec = bcd2bin(data[0]);
	tm->tm_min = bcd2bin(data[1]);
	tm->tm_hour = bcd2bin(data[2]);
	tm->tm_mday = bcd2bin(data[4]);
	tm->tm_mon = bcd2bin(data[5]) - 1;
	tm->tm_year = bcd2bin(data[7]) * 100 + bcd2bin(data[6]) - 1900;
	ret = rtc_valid_tm(tm);
	if (ret)
		return ret;
	rtc_time64_to_tm(rtc_tm_to_time64(tm), tm);
	return 0;
}

static int max8991_read_calendar(struct i2c_client *client, u8 *data)
{
	u8 address = MAX8991_TIME_REG;
	struct i2c_msg msgs[] = {
		{ .addr = client->addr,
		  .len = 1, .buf = &address },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = MAX8991_TIME_SIZE, .buf = data },
	};
	int ret;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_transfer(client->adapter, &msgs[0], 1);
	if (ret == 1)
		ret = __i2c_transfer(client->adapter, &msgs[1], 1);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret == 1 ? 0 : ret < 0 ? ret : -EIO;
}

static int max8991_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 first[MAX8991_TIME_SIZE], second[MAX8991_TIME_SIZE];
	u64 start = ktime_get_ns();
	unsigned int attempt;
	int ret;

	for (attempt = 0; attempt < MAX8991_READ_TRIES; attempt++) {
		ret = max8991_read_calendar(client, first);
		if (ret)
			return ret;
		ret = max8991_read_calendar(client, second);
		if (ret)
			return ret;
		if (ktime_get_ns() - start >= NSEC_PER_SEC)
			return -EAGAIN;
		if (!memcmp(first, second, sizeof(first)))
			return max8991_decode(second, tm);
	}
	return -EAGAIN;
}

static const struct rtc_class_ops max8991_rtc_ops = {
	.read_time = max8991_read_time,
};

static int max8991_probe(struct i2c_client *client)
{
	struct rtc_device *rtc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
	rtc->ops = &max8991_rtc_ops;
	rtc->range_max = RTC_TIMESTAMP_END_9999;
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, rtc->features);
	return devm_rtc_register_device(rtc);
}

static const struct of_device_id max8991_of_match[] = {
	{ .compatible = "maxim,max8991-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, max8991_of_match);

static const struct i2c_device_id max8991_id[] = {
	{ "max8991-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max8991_id);

static struct i2c_driver max8991_driver = {
	.driver = {
		.name = "max8991-rtc",
		.of_match_table = max8991_of_match,
	},
	.probe = max8991_probe,
	.id_table = max8991_id,
};
module_i2c_driver(max8991_driver);

MODULE_DESCRIPTION("Maxim MAX8991 read-only RTC driver");
MODULE_LICENSE("GPL");
