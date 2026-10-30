/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GT_S5620_BATTERY_H
#define _GT_S5620_BATTERY_H

#include <linux/errno.h>
#include <linux/time64.h>
#include <linux/types.h>

static inline bool gt_s5620_cache_fresh(bool sampled, u64 updated, u64 now)
{
	return sampled && now >= updated && now - updated <= 2ULL * NSEC_PER_SEC;
}

struct gt_s5620_filter {
	u16 samples[16];
	u16 sum;
	u16 index;
	u16 previous;
	u16 retries;
	bool initialized;
};

static inline int gt_s5620_temperature(unsigned int value, int *result)
{
	static const unsigned int knots[] = {
		227, 215, 200, 193, 177, 151, 135, 125, 109, 91,
		82, 69, 60, 51, 43, 36, 29, 26, 21,
	};
	unsigned int i;

	if (value > knots[0] || value < knots[18])
		return -ERANGE;
	for (i = 1; i < 19; i++) {
		if (value >= knots[i]) {
			*result = -200 + (int)(i - 1) * 50 +
				(int)((knots[i - 1] - value) * 50 / (knots[i - 1] - knots[i]));
			return 0;
		}
	}
	return -ERANGE;
}

static inline int gt_s5620_filter_sample(struct gt_s5620_filter *state,
					 unsigned int value, unsigned int minimum,
					 unsigned int maximum, unsigned int clamp_low,
					 unsigned int clamp_high)
{
	unsigned int i;

	if (value > 255 || value * 16 < minimum || value * 16 > maximum)
		return -EINVAL;
	if (value < clamp_low)
		value = clamp_low;
	if (value > clamp_high)
		value = clamp_high;
	if (!state->previous)
		state->previous = value;
	if ((int)value < (int)state->previous - 5 && state->retries < 20) {
		state->retries++;
		return -EAGAIN;
	}
	state->retries = 0;
	state->previous = value;
	if (!state->initialized) {
		for (i = 0; i < 16; i++)
			state->samples[i] = value;
		state->sum = value * 16;
		state->index = 0;
		state->initialized = true;
	}
	state->sum -= state->samples[state->index];
	state->samples[state->index] = value;
	state->sum += value;
	state->index = (state->index + 1) % 16;
	return 0;
}

static inline int gt_s5620_calibration(unsigned int low, unsigned int high,
				       u16 *result)
{
	if (low < 255)
		low *= 16;
	if (high < 255)
		high *= 16;
	if (!low || high > 4080 || high <= low || high - low < 10)
		return -EINVAL;

	result[0] = low;
	result[1] = high;
	return 0;
}

static inline int gt_s5620_voltage(unsigned int sum, unsigned int low,
				   unsigned int high)
{
	if (sum > 4080 || low < 10 || high > 4080 || high <= low ||
	    high - low < 10)
		return -EINVAL;
	if (sum < low - 10)
		sum = low - 10;
	if (sum > high + 10)
		sum = high + 10;

	return 3300 + ((int)sum - (int)low) * 900 / (int)(high - low);
}

static inline void gt_s5620_thresholds(unsigned int low, unsigned int high,
				       u16 *result)
{
	static const unsigned int weights[] = { 280, 339, 395, 443, 535 };
	unsigned int i;

	result[0] = low;
	for (i = 0; i < 5; i++)
		result[i + 1] = (high * weights[i] + low * (900 - weights[i])) / 900;
}

static inline int gt_s5620_voltage_raw(unsigned int value, unsigned int low,
				       unsigned int high)
{
	unsigned int delta, minimum;

	if (!low || high > 4080 || high <= low || high - low < 10)
		return -EINVAL;
	delta = (high * 850 + low * 50) / 900 - low;
	minimum = delta > low ? 0 : low - delta;
	if (value > 255 || value * 16 < minimum || value * 16 > high + delta)
		return -ERANGE;
	return gt_s5620_voltage(value * 16, low, high);
}

static inline int gt_s5620_level(unsigned int value, bool charging,
				 const u16 *thresholds, unsigned int hysteresis,
				 u16 *previous, bool *initialized)
{
	u16 level = *previous;

	if (!*initialized) {
		*initialized = true;
		level = charging ? 0 : 5;
	}
	if (charging) {
		if (value > thresholds[5] + hysteresis && level < 5)
			level = 5;
		else if (value > thresholds[4] + hysteresis && level < 4)
			level = 4;
		else if (value > thresholds[3] + hysteresis && level < 3)
			level = 3;
		else if (value > thresholds[2] + hysteresis && level <= 1)
			level = 2;
		else if (value > thresholds[1] && !level)
			level = 1;
	} else {
		switch (level) {
		case 5:
			if (value <= thresholds[5])
				level = 4;
			break;
		case 4:
		case 3:
		case 2:
		case 1:
			if (value > thresholds[level + 1] + hysteresis)
				level++;
			else if (value <= thresholds[level])
				level--;
			break;
		case 0:
			if (value > thresholds[1] + hysteresis)
				level = 1;
			else if (value <= thresholds[0])
				level = 6;
			break;
		}
	}
	*previous = level;
	return level;
}

struct gt_s5620_gauge {
	struct gt_s5620_filter filter;
	u16 calibration[2];
	u16 thresholds[6];
	u16 limits[4];
	u16 level;
	bool level_initialized;
	unsigned int samples;
	int mode;
	int capacity_error;
	int average;
	int average_error;
};

static inline int gt_s5620_gauge_init(struct gt_s5620_gauge *state,
				      unsigned int low, unsigned int high)
{
	u16 calibration[2];
	unsigned int delta;
	int ret, clamp;

	ret = gt_s5620_calibration(low, high, calibration);
	if (ret)
		return ret;
	low = calibration[0];
	high = calibration[1];
	delta = (high * 850 + low * 50) / 900 - low;
	if (delta > low)
		return -EINVAL;
	*state = (struct gt_s5620_gauge){};
	state->calibration[0] = low;
	state->calibration[1] = high;
	gt_s5620_thresholds(low, high, state->thresholds);
	state->limits[0] = low - delta;
	state->limits[1] = high + delta;
	clamp = ((int)low * 900 - (int)(high - low) * 500) / 900;
	state->limits[2] = clamp > 0 ? clamp / 16 : 0;
	state->limits[3] = high / 16;
	state->mode = -1;
	state->capacity_error = -EAGAIN;
	state->average_error = -EAGAIN;
	return 0;
}

static inline void gt_s5620_gauge_update(struct gt_s5620_gauge *state,
					 int error, unsigned int raw, int mode)
{
	int ret;

	if (mode < 0 || mode != state->mode) {
		state->level_initialized = false;
		state->samples = 0;
	}
	state->mode = mode < 0 ? -1 : mode;
	if (error)
		goto sample_error;
	ret = gt_s5620_filter_sample(&state->filter, raw,
				     state->limits[0], state->limits[1],
				     state->limits[2], state->limits[3]);
	if (ret && ret != -EAGAIN) {
		error = ret;
		goto sample_error;
	}
	state->average = gt_s5620_voltage(state->filter.sum,
					  state->calibration[0], state->calibration[1]);
	state->average_error = 0;
	if (mode < 0) {
		state->capacity_error = mode;
		return;
	}
	gt_s5620_level(state->filter.sum, mode == 1, state->thresholds, 85,
		       &state->level, &state->level_initialized);
	if (state->samples < 6)
		state->samples++;
	state->capacity_error = state->samples < 6 ? -EAGAIN : 0;
	return;

sample_error:
	state->capacity_error = error;
	state->average_error = error;
}

#endif
