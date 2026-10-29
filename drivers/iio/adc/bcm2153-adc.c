// SPDX-License-Identifier: GPL-2.0-only

#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ADC_CONTROL 0
#define ADC_COUNT 4
#define ADC_RESULT 8
#define ADC_READY BIT(15)

struct bcm2153_adc {
	void __iomem *base;
	struct mutex lock; /* Conversion state. */
};

static int bcm2153_adc_sample(struct bcm2153_adc *adc, unsigned int channel,
			      int *value)
{
	u16 result;
	int ret;

	if (channel >= 8)
		return -EINVAL;

	mutex_lock(&adc->lock);
	if (readw(adc->base + ADC_CONTROL) || readw(adc->base + ADC_COUNT)) {
		ret = -EBUSY;
		goto unlock;
	}
	writew(channel, adc->base + ADC_CONTROL);
	result = readw(adc->base + ADC_RESULT);
	if (result & ADC_READY) {
		ret = -EAGAIN;
		goto stop;
	}
	writew(0xf8 | channel, adc->base + ADC_CONTROL);
	ret = readw_poll_timeout(adc->base + ADC_RESULT, result,
				 result & ADC_READY, 10, 50000);
stop:
	writew(0, adc->base + ADC_COUNT);
	writew(0, adc->base + ADC_CONTROL);
	if (readw(adc->base + ADC_CONTROL) || readw(adc->base + ADC_COUNT))
		ret = -EIO;
	if (!ret)
		*value = result & 0xff;
unlock:
	mutex_unlock(&adc->lock);
	return ret;
}

#define BCM2153_ADC_CHANNEL(n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
}

static const struct iio_chan_spec bcm2153_adc_channels[] = {
	BCM2153_ADC_CHANNEL(0),
	BCM2153_ADC_CHANNEL(1),
	BCM2153_ADC_CHANNEL(2),
	BCM2153_ADC_CHANNEL(3),
	BCM2153_ADC_CHANNEL(4),
	BCM2153_ADC_CHANNEL(5),
	BCM2153_ADC_CHANNEL(6),
	BCM2153_ADC_CHANNEL(7),
};

static int bcm2153_adc_read_raw(struct iio_dev *indio_dev,
				const struct iio_chan_spec *channel,
				int *value, int *value2, long mask)
{
	struct bcm2153_adc *adc = iio_priv(indio_dev);
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	ret = bcm2153_adc_sample(adc, channel->channel, value);
	return ret ? ret : IIO_VAL_INT;
}

static const struct iio_info bcm2153_adc_info = {
	.read_raw = bcm2153_adc_read_raw,
};

static int bcm2153_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct bcm2153_adc *adc;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);
	mutex_init(&adc->lock);
	indio_dev->name = "bcm2153-adc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bcm2153_adc_info;
	indio_dev->channels = bcm2153_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(bcm2153_adc_channels);
	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id bcm2153_adc_of_match[] = {
	{ .compatible = "brcm,bcm2153-auxadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_adc_of_match);

static struct platform_driver bcm2153_adc_driver = {
	.probe = bcm2153_adc_probe,
	.driver = {
		.name = "bcm2153-adc",
		.of_match_table = bcm2153_adc_of_match,
	},
};
module_platform_driver(bcm2153_adc_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 auxiliary ADC");
MODULE_LICENSE("GPL");
