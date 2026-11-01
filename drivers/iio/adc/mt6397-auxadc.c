// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6397 PMIC AUXADC IIO driver
 *
 * Copyright (c) 2026 Ryan Brue <ryanbrue.dev@gmail.com>
 *
 * Based on drivers/iio/adc/mt6323-auxadc.c
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/time.h>
#include <linux/types.h>

#include <linux/mfd/mt6397/core.h>

#include <dt-bindings/iio/adc/mediatek,mt6329-auxadc.h>
#include <dt-bindings/iio/adc/mediatek,mt6397-auxadc.h>

#define MT6329_AUXADC_CON0		0xe8
#define MT6329_AUXADC_START		BIT(0)
#define MT6329_AUXADC_TRIM_COMP		BIT(1)
#define MT6329_AUXADC_CHSEL		GENMASK(7, 4)
#define MT6329_AUXADC_CON1		0xe9
#define MT6329_AUXADC_POWER		BIT(0)
#define MT6329_AUXADC_BUFFER		BIT(1)
#define MT6329_AUXADC_SAMPLES		GENMASK(7, 4)
#define MT6329_AUXADC_RESULT(n)		(0xda + 2 * (n))
#define MT6329_AUXADC_READY		BIT(7)
#define MT6329_AUXADC_HIGH		GENMASK(1, 0)
#define MT6329_AUXADC_TDET		0x38
#define MT6329_AUXADC_TDET_EN		BIT(5)
#define MT6329_AUXADC_SETTLE_US		(50 * USEC_PER_MSEC)

/* The ready bit is in the raw result register, the value in the trimmed one. */
#define MT6397_AUXADC_ADC(n)			(0x0514 + 2 * (n))
#define   MT6397_AUXADC_ADC_RDY			BIT(15)
#define   MT6397_AUXADC_ADC_VAL			GENMASK(9, 0)
/* The trimmed copy of ADC(n), which is ADC11 + n. */
#define MT6397_AUXADC_ADC_TRIM(n)		(0x052a + 2 * (n))

/* The thermistor needs the battery-detect bias and the input buffer on. */
#define MT6397_AUXADC_CON0			0x0542
#define   MT6397_AUXADC_CON0_BUF_PWD_B		BIT(1)
#define   MT6397_AUXADC_CON0_BUF_PWD_ON		BIT(3)
/* Samples accumulated per conversion; the chip's accumulator is left at one. */
#define   MT6397_AUXADC_CON0_SPL_NUM		GENMASK(11, 7)
#define MT6397_CHR_CON7				0x000e
#define   MT6397_CHR_CON7_BATON_TDET_EN		BIT(2)

#define MT6397_AUXADC_BATTEMP_SETTLE_US		(20 * USEC_PER_MSEC)

#define MT6397_AUXADC_CON1			0x0544
#define   MT6397_AUXADC_CON1_CHSEL		GENMASK(10, 7)
#define   MT6397_AUXADC_CON1_START		BIT(0)

/*
 * CHR_CON16 and the two SOURCE_CH0 selects route either BATSNS or ISENSE onto
 * the battery channel. Only ISENSE is used: with a switching charger in the
 * power path BATSNS sits on the system rail rather than on the pack. All five
 * ADCIN bits are written out because they are one enable per analog input
 * and not a select field; the driver touches only the two it needs.
 */
#define MT6397_CHR_CON16			0x0020
#define   MT6397_CHR_CON16_ADCIN_VCHR_EN	BIT(12)
#define   MT6397_CHR_CON16_ADCIN_VSEN_EN	BIT(11)
#define   MT6397_CHR_CON16_ADCIN_VBAT_EN	BIT(10)
#define   MT6397_CHR_CON16_ADCIN_VSEN_EXT_BATON_EN	BIT(9)
#define   MT6397_CHR_CON16_ADCIN_VSEN_MUX_EN	BIT(8)
#define MT6397_AUXADC_CON14			0x055e
#define   MT6397_AUXADC_CON14_CH0_NORM_SEL	BIT(2)
#define   MT6397_AUXADC_CON14_CH0_LBAT_SEL	BIT(0)

#define MT6397_AUXADC_ISENSE_SETTLE_US		(1 * USEC_PER_MSEC)

/* Conversions averaged in software per read. */
#define MT6397_AUXADC_SAMPLES			16

/* Hardware limitation: the result is not ready for this long after START. */
#define MT6397_AUXADC_START_US			30

/* The battery channel is divided down by 4; the thermistor is not divided. */
#define MT6397_AUXADC_ISENSE_DIVIDER		4

/* The PMIC's channel numbers, as the CON1 select field takes them. */
#define MT6397_AUXADC_HWCHAN_BATSNS		0
#define MT6397_AUXADC_HWCHAN_BAT_TEMP		3

struct mt6397_auxadc {
	struct regmap *regmap;
	bool mt6329;
	int gain;
	int offset;
	unsigned int saved_control;
	unsigned int saved_config;
	unsigned int saved_bias;
	bool restore_pending;
	bool restore_temperature;
	/*
	 * A read is a multi-register sequence: analog setup, then a channel
	 * select and an edge-triggered start per conversion. Interleaving two
	 * of them steals the select and tears down the setup, so a read holds
	 * this for its whole burst.
	 */
	struct mutex lock;
};

/*
 * A device tree channel ID indexes this array; .address holds the PMIC channel
 * number. The thermistor reports as a voltage because that is all the PMIC
 * measures: the divider across an NTC whose curve belongs to the board.
 */
static const struct iio_chan_spec mt6397_auxadc_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = MT6397_AUXADC_ISENSE,
		.address = MT6397_AUXADC_HWCHAN_BATSNS,
		.datasheet_name = "isense",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	}, {
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = MT6397_AUXADC_BAT_TEMP,
		.address = MT6397_AUXADC_HWCHAN_BAT_TEMP,
		.datasheet_name = "bat_temp",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static const struct iio_chan_spec mt6329_auxadc_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = MT6329_AUXADC_ISENSE,
		.address = 1,
		.datasheet_name = "isense",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
	}, {
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = MT6329_AUXADC_BAT_TEMP,
		.address = 3,
		.datasheet_name = "bat_temp",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int mt6329_auxadc_read_once(struct mt6397_auxadc *adc,
				   const struct iio_chan_spec *chan, int *val)
{
	struct regmap *map = adc->regmap;
	unsigned int low, high;
	int ret;

	ret = regmap_clear_bits(map, MT6329_AUXADC_CON0, MT6329_AUXADC_START);
	if (ret)
		return ret;
	ret = regmap_set_bits(map, MT6329_AUXADC_CON0, MT6329_AUXADC_START);
	if (ret)
		return ret;
	fsleep(MT6397_AUXADC_START_US);
	ret = regmap_read_poll_timeout(map, MT6329_AUXADC_RESULT(chan->address) + 1,
				       high, high & MT6329_AUXADC_READY,
				       100, 100 * USEC_PER_MSEC);
	if (ret)
		return ret;
	ret = regmap_read(map, MT6329_AUXADC_RESULT(chan->address), &low);
	if (ret)
		return ret;
	ret = regmap_read(map, MT6329_AUXADC_RESULT(chan->address) + 1, &high);
	if (ret)
		return ret;
	if (!(high & MT6329_AUXADC_READY))
		return -EIO;
	*val = low | FIELD_GET(MT6329_AUXADC_HIGH, high) << 8;

	return 0;
}

static int mt6329_auxadc_calibrate(struct mt6397_auxadc *adc,
				   const struct iio_chan_spec *chan, int raw)
{
	int corrected = raw + raw * adc->gain / 1024 + adc->offset;
	int fullscale = chan->address == 1 ? 4800 : 1200;

	if (corrected < 0)
		return -ERANGE;

	return corrected * fullscale / 1024;
}

static int mt6397_auxadc_read_once(struct mt6397_auxadc *adc,
				   const struct iio_chan_spec *chan, int *val)
{
	struct regmap *map = adc->regmap;
	unsigned int reg;
	int ret;

	ret = regmap_update_bits(map, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_CHSEL,
				 FIELD_PREP(MT6397_AUXADC_CON1_CHSEL, chan->address));
	if (ret)
		return ret;

	/*
	 * START is edge triggered: lower it before raising it again. No delay
	 * is needed in between -- the ready bit survives the fall and is
	 * cleared by the rise, before the next transaction on this bus can
	 * observe it, so a ready bit seen by the poll can only be this
	 * conversion's.
	 */
	ret = regmap_clear_bits(map, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_START);
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_START);
	if (ret)
		return ret;

	fsleep(MT6397_AUXADC_START_US);

	ret = regmap_read_poll_timeout(map, MT6397_AUXADC_ADC(chan->address),
				       reg, reg & MT6397_AUXADC_ADC_RDY,
				       100, 100 * USEC_PER_MSEC);
	if (ret)
		return ret;

	ret = regmap_read(map, MT6397_AUXADC_ADC_TRIM(chan->address), &reg);
	if (ret)
		return ret;

	*val = FIELD_GET(MT6397_AUXADC_ADC_VAL, reg);

	return 0;
}

static int mt6397_auxadc_battemp_bias_on(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;
	int ret;

	ret = regmap_set_bits(map, MT6397_AUXADC_CON0,
			      MT6397_AUXADC_CON0_BUF_PWD_ON);
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6397_AUXADC_CON0,
			      MT6397_AUXADC_CON0_BUF_PWD_B);
	if (ret)
		return ret;

	return regmap_set_bits(map, MT6397_CHR_CON7,
			       MT6397_CHR_CON7_BATON_TDET_EN);
}

static void mt6397_auxadc_battemp_bias_off(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;

	regmap_clear_bits(map, MT6397_CHR_CON7, MT6397_CHR_CON7_BATON_TDET_EN);
	regmap_clear_bits(map, MT6397_AUXADC_CON0, MT6397_AUXADC_CON0_BUF_PWD_B);
	regmap_clear_bits(map, MT6397_AUXADC_CON0, MT6397_AUXADC_CON0_BUF_PWD_ON);
}

static int mt6397_auxadc_isense_enable(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;
	int ret;

	/*
	 * Each ADCIN bit connects one input to the battery channel. BATSNS is
	 * disconnected before ISENSE is connected, so that the two are never
	 * connected at the same time.
	 */
	ret = regmap_clear_bits(map, MT6397_CHR_CON16, MT6397_CHR_CON16_ADCIN_VBAT_EN);
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6397_CHR_CON16, MT6397_CHR_CON16_ADCIN_VSEN_EN);
	if (ret)
		return ret;

	return regmap_set_bits(map, MT6397_AUXADC_CON14,
			       MT6397_AUXADC_CON14_CH0_NORM_SEL |
			       MT6397_AUXADC_CON14_CH0_LBAT_SEL);
}

static void mt6397_auxadc_isense_disable(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;

	regmap_clear_bits(map, MT6397_AUXADC_CON14,
			  MT6397_AUXADC_CON14_CH0_NORM_SEL |
			  MT6397_AUXADC_CON14_CH0_LBAT_SEL);
	regmap_clear_bits(map, MT6397_CHR_CON16,
			  MT6397_CHR_CON16_ADCIN_VSEN_EN |
			  MT6397_CHR_CON16_ADCIN_VBAT_EN);
}

/*
 * One burst on a channel that is already prepared: the caller owns the lock,
 * the analog setup and the teardown.
 */
static int mt6397_auxadc_read_burst(struct mt6397_auxadc *adc,
				    const struct iio_chan_spec *chan, int *val)
{
	unsigned int sum = 0;
	int sample;
	int ret;

	for (unsigned int i = 0; i < MT6397_AUXADC_SAMPLES; i++) {
		if (adc->mt6329)
			ret = mt6329_auxadc_read_once(adc, chan, &sample);
		else
			ret = mt6397_auxadc_read_once(adc, chan, &sample);
		if (ret)
			return ret;

		sum += sample;
	}

	*val = DIV_ROUND_CLOSEST(sum, MT6397_AUXADC_SAMPLES);

	return 0;
}

static int mt6329_auxadc_restore(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;
	int ret, err;

	if (!adc->restore_pending)
		return 0;
	ret = regmap_clear_bits(map, MT6329_AUXADC_CON0, MT6329_AUXADC_START);
	if (adc->restore_temperature) {
		err = regmap_update_bits(map, MT6329_AUXADC_TDET, MT6329_AUXADC_TDET_EN,
					 adc->saved_bias & MT6329_AUXADC_TDET_EN);
		if (!ret)
			ret = err;
	}
	err = regmap_write(map, MT6329_AUXADC_CON1, adc->saved_config);
	if (!ret)
		ret = err;
	err = regmap_write(map, MT6329_AUXADC_CON0, adc->saved_control);
	if (!ret)
		ret = err;
	if (!ret)
		adc->restore_pending = false;

	return ret;
}

static int mt6329_auxadc_read(struct mt6397_auxadc *adc,
			      const struct iio_chan_spec *chan, int *val)
{
	struct regmap *map = adc->regmap;
	unsigned int control, config, bias = 0;
	bool temperature = chan->address == 3;
	int ret, cleanup;

	ret = mt6329_auxadc_restore(adc);
	if (ret)
		return ret;
	ret = regmap_read(map, MT6329_AUXADC_CON0, &control);
	if (ret)
		return ret;
	if (control & MT6329_AUXADC_START)
		return -EBUSY;
	ret = regmap_read(map, MT6329_AUXADC_CON1, &config);
	if (ret)
		return ret;
	if (temperature) {
		ret = regmap_read(map, MT6329_AUXADC_TDET, &bias);
		if (ret)
			return ret;
	}
	adc->saved_control = control;
	adc->saved_config = config;
	adc->saved_bias = bias;
	adc->restore_temperature = temperature;
	adc->restore_pending = true;
	ret = regmap_update_bits(map, MT6329_AUXADC_CON0,
				 MT6329_AUXADC_CHSEL | MT6329_AUXADC_TRIM_COMP,
				 FIELD_PREP(MT6329_AUXADC_CHSEL, chan->address) |
				 MT6329_AUXADC_TRIM_COMP);
	if (ret)
		goto restore;
	ret = regmap_update_bits(map, MT6329_AUXADC_CON1,
				 MT6329_AUXADC_SAMPLES | MT6329_AUXADC_POWER |
				 MT6329_AUXADC_BUFFER,
				 FIELD_PREP(MT6329_AUXADC_SAMPLES, temperature ? 15 : 1) |
				 MT6329_AUXADC_POWER |
				 (temperature ? MT6329_AUXADC_BUFFER :
				  config & MT6329_AUXADC_BUFFER));
	if (ret)
		goto restore;
	if (temperature) {
		ret = regmap_set_bits(map, MT6329_AUXADC_TDET, MT6329_AUXADC_TDET_EN);
		if (ret)
			goto restore;
		fsleep(MT6329_AUXADC_SETTLE_US);
	}
	ret = mt6397_auxadc_read_burst(adc, chan, val);

restore:
	cleanup = mt6329_auxadc_restore(adc);

	return ret ?: cleanup;
}

static void mt6397_auxadc_stop(struct mt6397_auxadc *adc)
{
	/* Lowering START leaves the converter unarmed between reads. */
	regmap_clear_bits(adc->regmap, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_START);
}

static int mt6397_auxadc_isense_read(struct mt6397_auxadc *adc,
				     const struct iio_chan_spec *chan, int *val)
{
	int ret;

	ret = mt6397_auxadc_isense_enable(adc);
	if (ret)
		goto out_disable;

	fsleep(MT6397_AUXADC_ISENSE_SETTLE_US);

	ret = mt6397_auxadc_read_burst(adc, chan, val);

out_disable:
	mt6397_auxadc_stop(adc);
	mt6397_auxadc_isense_disable(adc);

	return ret;
}

static int mt6397_auxadc_battemp_read(struct mt6397_auxadc *adc,
				      const struct iio_chan_spec *chan, int *val)
{
	int ret;

	ret = mt6397_auxadc_battemp_bias_on(adc);
	if (ret)
		goto out_bias_off;

	fsleep(MT6397_AUXADC_BATTEMP_SETTLE_US);

	ret = mt6397_auxadc_read_burst(adc, chan, val);

out_bias_off:
	mt6397_auxadc_stop(adc);
	mt6397_auxadc_battemp_bias_off(adc);

	return ret;
}

static int mt6397_auxadc_read_channel(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      int *val)
{
	struct mt6397_auxadc *adc = iio_priv(indio_dev);

	guard(mutex)(&adc->lock);

	if (adc->mt6329)
		return mt6329_auxadc_read(adc, chan, val);

	if (chan->channel == MT6397_AUXADC_ISENSE)
		return mt6397_auxadc_isense_read(adc, chan, val);

	return mt6397_auxadc_battemp_read(adc, chan, val);
}

static int mt6397_auxadc_read_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int *val, int *val2, long mask)
{
	struct mt6397_auxadc *adc = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		if (!adc->mt6329)
			return -EINVAL;
		ret = mt6397_auxadc_read_channel(indio_dev, chan, val);
		if (ret)
			return ret;
		ret = mt6329_auxadc_calibrate(adc, chan, *val);
		if (ret < 0)
			return ret;
		*val = ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		ret = mt6397_auxadc_read_channel(indio_dev, chan, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* 1200 mV full range with 10-bit resolution. */
		if (chan->channel == MT6397_AUXADC_ISENSE)
			*val = 1200 * MT6397_AUXADC_ISENSE_DIVIDER;
		else
			*val = 1200;

		*val2 = 10;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}
}

static const struct iio_info mt6397_auxadc_info = {
	.read_raw = mt6397_auxadc_read_raw,
};

static void mt6329_auxadc_cleanup(void *data)
{
	struct iio_dev *indio_dev = data;
	struct mt6397_auxadc *adc = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&adc->lock);
	ret = mt6329_auxadc_restore(adc);
	if (ret)
		dev_err(&indio_dev->dev, "failed to restore ADC controls: %d\n", ret);
}

static int mt6329_auxadc_init(struct device *dev, struct mt6397_auxadc *adc)
{
#if IS_REACHABLE(CONFIG_I2C)
	struct i2c_client *client;
	struct i2c_client *calibration;
	unsigned int low, high;
	int trim0, trim1, ret;

	client = i2c_verify_client(dev->parent);
	if (!client)
		return -ENODEV;
	adc->regmap = dev_get_regmap(dev->parent, "power");
	if (!adc->regmap)
		return -ENODEV;
	ret = regmap_read(adc->regmap, 0, &low);
	if (ret)
		return ret;
	ret = regmap_read(adc->regmap, 1, &high);
	if (ret)
		return ret;
	if ((high << 8 | low) != 0x1029)
		return -ENODEV;
	calibration = devm_i2c_new_dummy_device(dev, client->adapter, 0x61);
	if (IS_ERR(calibration))
		return PTR_ERR(calibration);
	trim0 = i2c_smbus_read_byte_data(calibration, 0x92);
	if (trim0 < 0)
		return trim0;
	trim1 = i2c_smbus_read_byte_data(calibration, 0x93);
	if (trim1 < 0)
		return trim1;
	adc->gain = trim0 & 0x7f;
	if (adc->gain > 64)
		adc->gain -= 128;
	adc->offset = (trim0 >> 7) | ((trim1 & 0x1f) << 1);
	if (adc->offset > 32)
		adc->offset -= 64;

	return 0;
#else
	return -EOPNOTSUPP;
#endif
}

static int mt6397_auxadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6397_chip *chip = dev_get_drvdata(dev->parent);
	struct mt6397_auxadc *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->mt6329 = of_device_is_compatible(dev->of_node, "mediatek,mt6329-auxadc");
	if (adc->mt6329) {
		ret = mt6329_auxadc_init(dev, adc);
		if (ret)
			return dev_err_probe(dev, ret, "failed to initialize auxadc\n");
	} else {
		adc->regmap = chip->regmap;
	}

	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	if (adc->mt6329) {
		ret = devm_add_action_or_reset(dev, mt6329_auxadc_cleanup, indio_dev);
		if (ret)
			return ret;
	}

	if (!adc->mt6329) {
		ret = regmap_update_bits(adc->regmap, MT6397_AUXADC_CON0,
					 MT6397_AUXADC_CON0_SPL_NUM,
					 FIELD_PREP(MT6397_AUXADC_CON0_SPL_NUM, 1));
		if (ret)
			return dev_err_probe(dev, ret, "failed to initialize auxadc\n");
	}

	indio_dev->name = adc->mt6329 ? "mt6329-auxadc" : "mt6397-auxadc";
	indio_dev->info = &mt6397_auxadc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adc->mt6329 ? mt6329_auxadc_channels : mt6397_auxadc_channels;
	indio_dev->num_channels = adc->mt6329 ? ARRAY_SIZE(mt6329_auxadc_channels) :
					      ARRAY_SIZE(mt6397_auxadc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id mt6397_auxadc_of_match[] = {
	{ .compatible = "mediatek,mt6329-auxadc" },
	{ .compatible = "mediatek,mt6397-auxadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6397_auxadc_of_match);

static struct platform_driver mt6397_auxadc_driver = {
	.driver = {
		.name = "mt6397-auxadc",
		.of_match_table = mt6397_auxadc_of_match,
	},
	.probe = mt6397_auxadc_probe,
};
module_platform_driver(mt6397_auxadc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan Brue <ryanbrue.dev@gmail.com>");
MODULE_DESCRIPTION("MediaTek MT6397 PMIC AUXADC Driver");
