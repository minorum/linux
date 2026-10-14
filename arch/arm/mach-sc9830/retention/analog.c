// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/regmap.h>

static const unsigned int sc9830_sleep_regs[] = { 0x8e8, 0x890, 0x894 };
static const unsigned int sc9830_sleep_masks[] = { 0x200, 0x60, 1 };

struct sc9830_retention_analog {
	struct regmap *map;
	unsigned int saved[ARRAY_SIZE(sc9830_sleep_regs)];
	bool owned;
};

static int sc9830_retention_analog_restore(struct sc9830_retention_analog *state)
{
	unsigned int i, value;
	int ret = 0, error;

	if (!state->owned)
		return 0;
	for (i = ARRAY_SIZE(sc9830_sleep_regs); i-- > 0;) {
		error = regmap_update_bits(state->map, sc9830_sleep_regs[i],
					   sc9830_sleep_masks[i], state->saved[i]);
		if (error && !ret)
			ret = error;
		error = regmap_read(state->map, sc9830_sleep_regs[i], &value);
		if (!error && (value & sc9830_sleep_masks[i]) != state->saved[i])
			error = -EIO;
		if (error && !ret)
			ret = error;
	}
	if (!ret)
		state->owned = false;
	return ret;
}

static int sc9830_retention_analog_prepare(struct sc9830_retention_analog *state)
{
	unsigned int i, value;
	int ret;

	if (!state->map)
		return -ENODEV;
	if (state->owned)
		return -EBUSY;
	for (i = 0; i < ARRAY_SIZE(sc9830_sleep_regs); i++) {
		ret = regmap_read(state->map, sc9830_sleep_regs[i], &value);
		if (ret)
			return ret;
		state->saved[i] = value & sc9830_sleep_masks[i];
	}
	state->owned = true;
	for (i = 0; i < ARRAY_SIZE(sc9830_sleep_regs); i++) {
		ret = regmap_update_bits(state->map, sc9830_sleep_regs[i],
					 sc9830_sleep_masks[i], 0);
		if (ret)
			goto restore;
		ret = regmap_read(state->map, sc9830_sleep_regs[i], &value);
		if (ret)
			goto restore;
		if (value & sc9830_sleep_masks[i]) {
			ret = -EIO;
			goto restore;
		}
	}
	return 0;
restore:
	sc9830_retention_analog_restore(state);
	return ret;
}
