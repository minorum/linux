/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _BCM59035_CHARGER_H
#define _BCM59035_CHARGER_H

#include <linux/errno.h>

static inline int bcm59035_current_reg(int value)
{
	if (value >= 50000 && value <= 200000 && !((value - 50000) % 10000))
		return (value - 50000) / 10000;
	if (value >= 250000 && value <= 1000000 && !((value - 250000) % 50000))
		return 16 + (value - 250000) / 50000;
	return -EINVAL;
}

static inline int bcm59035_current_ua(unsigned int value)
{
	value &= 0x1f;
	return value < 16 ? 50000 + value * 10000 : 250000 + (value - 16) * 50000;
}

static inline int bcm59035_voltage_reg(int value)
{
	if (value >= 3900000 && value <= 4200000 && !((value - 3900000) % 50000))
		return (4200000 - value) / 50000;
	if (value >= 3200000 && value <= 3600000 && !((value - 3200000) % 100000))
		return 7 + (value - 3200000) / 100000;
	return -EINVAL;
}

static inline int bcm59035_voltage_uv(unsigned int value)
{
	value &= 0xf;
	if (value < 7)
		return 4200000 - value * 50000;
	if (value < 12)
		return 3200000 + (value - 7) * 100000;
	return -EINVAL;
}

static inline int bcm59035_eoc_reg(int value)
{
	int reg = bcm59035_current_reg(value);

	return reg >= 0 && reg < 16 ? reg : -EINVAL;
}

static inline int bcm59035_eoc_ua(unsigned int value)
{
	return 50000 + (value & 0xf) * 10000;
}

#endif
