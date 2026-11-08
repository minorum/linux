// SPDX-License-Identifier: GPL-2.0-only
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define MT6329_FGADC_CLOCK	0x15
#define MT6329_FGADC_CLOCK_PDN	BIT(4)
#define MT6329_FGADC_MODE		0x69
#define MT6329_FGADC_CONTROL	0x6a
#define MT6329_FGADC_CURRENT_H	0x78
#define MT6329_FGADC_CURRENT_L	0x79
#define MT6329_FGADC_READY	BIT(2)

struct mt6329_fgadc {
	struct regmap *regmap;
	struct regmap *power;
	struct mutex lock; /* Current sampling */
	u32 shunt_uohm;
	u32 calibration_percent;
	bool restore_pending;
};

static int mt6329_fgadc_init(struct mt6329_fgadc *adc)
{
	unsigned int value;
	int ret;

	ret = regmap_read(adc->regmap, MT6329_FGADC_CONTROL, &value);
	if (ret)
		return ret;
	if (value)
		return -EBUSY;
	ret = regmap_read_poll_timeout(adc->power, 0xc8, value,
				       (value & (BIT(7) | BIT(4) | BIT(0))) ==
				       (BIT(7) | BIT(0)), 1000, 50000);
	if (ret)
		return ret;
	ret = regmap_clear_bits(adc->regmap, MT6329_FGADC_CLOCK,
				MT6329_FGADC_CLOCK_PDN);
	if (ret)
		return ret;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CLOCK, &value);
	if (ret)
		return ret;
	if (value & MT6329_FGADC_CLOCK_PDN)
		return -EIO;
	ret = regmap_read(adc->regmap, MT6329_FGADC_MODE, &value);
	if (ret)
		return ret;
	if (value != 0x29) {
		ret = regmap_write(adc->regmap, MT6329_FGADC_MODE, 0x28);
		if (ret)
			return ret;
		ret = regmap_write(adc->regmap, MT6329_FGADC_MODE, 0x29);
		if (ret)
			return ret;
	}
	ret = regmap_read(adc->regmap, MT6329_FGADC_MODE, &value);
	if (ret)
		return ret;
	if (value != 0x29)
		return -EIO;
	fsleep(50000);

	return 0;
}

static int mt6329_fgadc_convert(struct mt6329_fgadc *adc, unsigned int raw,
				int *value)
{
	unsigned int magnitude = raw < 0x8000 ? raw : 0xffff - raw;
	u64 scaled_current;

	if (!adc->shunt_uohm || !adc->calibration_percent)
		return -EINVAL;

	scaled_current = div_u64((u64)magnitude * 158122, 100000);
	scaled_current = div_u64(scaled_current * 20000, adc->shunt_uohm);
	scaled_current = div_u64(scaled_current * adc->calibration_percent, 100);
	scaled_current = div_u64(scaled_current, 10);
	if (scaled_current > INT_MAX / 1000)
		return -ERANGE;
	*value = raw < 0x8000 ? (int)scaled_current : -(int)scaled_current;

	return 0;
}

static int mt6329_fgadc_restore(struct mt6329_fgadc *adc)
{
	unsigned int control;
	int ret;

	if (!adc->restore_pending)
		return 0;
	ret = regmap_write(adc->regmap, MT6329_FGADC_CONTROL, 0x08);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(adc->regmap, MT6329_FGADC_CONTROL,
				       control, !(control & MT6329_FGADC_READY),
				       1000, 50000);
	if (ret)
		return ret;
	ret = regmap_write(adc->regmap, MT6329_FGADC_CONTROL, 0);
	if (ret)
		return ret;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CONTROL, &control);
	if (ret)
		return ret;
	if (control)
		return -EIO;
	adc->restore_pending = false;

	return 0;
}

static int mt6329_fgadc_sample(struct mt6329_fgadc *adc, int *value)
{
	unsigned int control, low, high;
	int ret, cleanup;

	ret = mt6329_fgadc_restore(adc);
	if (ret)
		return ret;
	ret = regmap_read(adc->power, 0xc8, &control);
	if (ret)
		return ret;
	if ((control & (BIT(7) | BIT(4) | BIT(0))) != (BIT(7) | BIT(0)))
		return -EOPNOTSUPP;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CLOCK, &control);
	if (ret)
		return ret;
	if (control & MT6329_FGADC_CLOCK_PDN)
		return -EOPNOTSUPP;
	ret = regmap_read(adc->regmap, MT6329_FGADC_MODE, &control);
	if (ret)
		return ret;
	if (control != 0x29)
		return -EOPNOTSUPP;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CONTROL, &control);
	if (ret)
		return ret;
	if (control)
		return -EBUSY;

	adc->restore_pending = true;
	ret = regmap_write(adc->regmap, MT6329_FGADC_CONTROL, 0x02);
	if (ret)
		goto restore;
	ret = regmap_read_poll_timeout(adc->regmap, MT6329_FGADC_CONTROL,
				       control, control & MT6329_FGADC_READY,
				       1000, 50000);
	if (ret)
		goto restore;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CURRENT_L, &low);
	if (ret)
		goto restore;
	ret = regmap_read(adc->regmap, MT6329_FGADC_CURRENT_H, &high);
restore:
	cleanup = mt6329_fgadc_restore(adc);
	if (ret || cleanup)
		return ret ?: cleanup;
	*value = high << 8 | low;

	return 0;
}

static int mt6329_fgadc_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct mt6329_fgadc *adc = iio_priv(indio_dev);
	int raw, ret;

	if (mask != IIO_CHAN_INFO_PROCESSED)
		return -EINVAL;
	guard(mutex)(&adc->lock);
	ret = mt6329_fgadc_sample(adc, &raw);
	if (ret)
		return ret;
	ret = mt6329_fgadc_convert(adc, raw, val);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static void mt6329_fgadc_cleanup(void *data)
{
	struct mt6329_fgadc *adc = data;
	int ret;

	guard(mutex)(&adc->lock);
	ret = mt6329_fgadc_restore(adc);
	if (ret)
		dev_err(regmap_get_device(adc->regmap),
			"Failed to release current latch: %d\n", ret);
}

static const struct iio_chan_spec mt6329_fgadc_channel = {
	.type = IIO_CURRENT,
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
};

static const struct iio_info mt6329_fgadc_info = {
	.read_raw = mt6329_fgadc_read_raw,
};

static int mt6329_fgadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6329_fgadc *adc;
	struct iio_dev *indio_dev;
	int ret, maximum;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->power = dev_get_regmap(dev->parent, "power");
	adc->regmap = dev_get_regmap(dev->parent, "bank1");
	if (!adc->power || !adc->regmap)
		return -ENODEV;
	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				       &adc->shunt_uohm);
	if (ret)
		return ret;
	ret = device_property_read_u32(dev, "mediatek,current-calibration-percent",
				       &adc->calibration_percent);
	if (ret)
		return ret;
	ret = mt6329_fgadc_convert(adc, 0x7fff, &maximum);
	if (ret)
		return ret;
	ret = devm_regulator_get_enable_read_voltage(dev, "va2");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to enable VA2\n");
	if (ret != 2500000)
		return dev_err_probe(dev, -EOPNOTSUPP, "Unsupported VA2 voltage\n");
	mutex_init(&adc->lock);
	ret = devm_add_action_or_reset(dev, mt6329_fgadc_cleanup, adc);
	if (ret)
		return ret;
	ret = mt6329_fgadc_init(adc);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to start converter\n");

	indio_dev->name = "mt6329-fgadc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mt6329_fgadc_info;
	indio_dev->channels = &mt6329_fgadc_channel;
	indio_dev->num_channels = 1;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id mt6329_fgadc_of_match[] = {
	{ .compatible = "mediatek,mt6329-fgadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6329_fgadc_of_match);

static struct platform_driver mt6329_fgadc_driver = {
	.probe = mt6329_fgadc_probe,
	.driver = {
		.name = "mt6329-fgadc",
		.of_match_table = mt6329_fgadc_of_match,
	},
};
module_platform_driver(mt6329_fgadc_driver);

MODULE_DESCRIPTION("MediaTek MT6329 fuel gauge ADC driver");
MODULE_LICENSE("GPL");
