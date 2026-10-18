/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2012 Samsung Electronics */
#ifndef _S6E8AA5X01_J3_GAMMA_H
#define _S6E8AA5X01_J3_GAMMA_H

#include <linux/math64.h>
#include <linux/slab.h>

#include "panel-samsung-s6e8aa5x01-j3-data.h"

struct j3_gamma_work {
	s32 mtp[10][3];
	s32 voltage[10][3];
	s32 gray[256][3];
	s32 vt[3];
};

static int j3_gamma_index(u64 target)
{
	int low = 0, high = 255, mid;
	u64 lower, upper, value;

	if (target > (u64)j3_curve[255] * j3_candela[61])
		return -EINVAL;
	while (low <= high) {
		mid = low + (high - low) / 2;
		value = (u64)j3_curve[mid] * j3_candela[61];
		if (value == target)
			return mid;
		if (value < target)
			low = mid + 1;
		else
			high = mid - 1;
	}
	lower = target - (u64)j3_curve[high] * j3_candela[61];
	upper = (u64)j3_curve[low] * j3_candela[61] - target;
	return lower <= upper ? high : low;
}

static int s6e8aa5x01_j3_gamma(const u8 mtp[33], u8 output[62][33])
{
	static const u16 vt_coeff[] = {
		0, 12, 24, 36, 48, 60, 72, 84,
		96, 108, 138, 148, 158, 168, 178, 186,
	};
	static const u8 points[] = { 0, 3, 11, 23, 35, 51, 87, 151, 203, 255 };
	const s32 vreg = 24326963;
	struct j3_gamma_work *work;
	u64 factor, span;
	s32 anchor, value, numerator, denominator;
	int row, point, color, offset = 0, gray, low, high, index[10];
	int ret = -EINVAL;

	work = kzalloc_obj(*work);
	if (!work)
		return -ENOMEM;
	for (point = 9; point >= 0; point--)
		for (color = 0; color < 3; color++) {
			if (point == 9) {
				value = mtp[offset + 1];
				work->mtp[point][color] = mtp[offset] ? -value : value;
				offset += 2;
			} else {
				work->mtp[point][color] = mtp[offset++];
			}
		}
	for (color = 0; color < 3; color++) {
		value = work->mtp[0][color];
		if ((unsigned int)value >= ARRAY_SIZE(vt_coeff))
			goto out;
		factor = div_u64((u64)vt_coeff[value] << 22, 860);
		work->vt[color] = vreg - (((u64)vreg * factor) >> 22);
		work->voltage[0][color] = vreg;
		for (point = 9; point >= 1; point--) {
			numerator = point == 9 ? 72 : 64;
			denominator = point == 9 ? 860 : 320;
			value = (point == 9 ? 256 : 128) + work->mtp[point][color];
			if (value + numerator < 0)
				goto out;
			factor = div_u64((u64)(value + numerator) << 22, denominator);
			anchor = point == 9 || point == 1 ? vreg : work->vt[color];
			span = point == 9 ? vreg : anchor - work->voltage[point + 1][color];
			work->voltage[point][color] = anchor - ((span * factor) >> 22);
		}
		for (point = 0; point < 9; point++) {
			low = points[point];
			high = points[point + 1];
			work->gray[low][color] = work->voltage[point][color];
			for (gray = low + 1; gray < high; gray++) {
				value = work->voltage[point][color] -
					work->voltage[point + 1][color];
				span = (u64)abs(value) * (high - gray);
				value = div_u64(span, high - low) * (value < 0 ? -1 : 1);
				work->gray[gray][color] = work->voltage[point + 1][color] + value;
			}
		}
		work->gray[255][color] = work->voltage[9][color];
	}
	for (row = 0; row < 62; row++) {
		for (point = 0; point < 10; point++) {
			index[point] = j3_gamma_index((u64)j3_curve[points[point]] *
						    j3_base_lux[row]);
			if (index[point] < 0)
				goto out;
			if (point) {
				index[point] += j3_gray_comp[row][9 - point];
				index[point] = max(index[point], 1);
				if (index[point] > 255)
					goto out;
			}
		}
		offset = 0;
		for (point = 9; point >= 0; point--)
			for (color = 0; color < 3; color++) {
				value = 0;
				if (point) {
					anchor = point == 9 || point == 1 ? vreg : work->vt[color];
					span = anchor - work->gray[index[point]][color];
					denominator = point == 9 ? vreg :
						anchor - work->gray[index[point + 1]][color];
					if (denominator <= 0)
						goto out;
					value = div_u64(span * (point == 9 ? 860 : 320),
							denominator);
					value -= point == 9 ? 72 : 64;
					if (point >= 2)
						value += j3_rgb_comp[row][(9 - point) * 3 + color];
					value -= work->mtp[point][color];
				}
				if (point == 9)
					output[row][offset++] = value >> 8;
				output[row][offset++] = point == 9 ? value : min(value, 255);
			}
	}
	offset = 0;
	for (point = 9; point >= 0; point--)
		for (color = 0; color < 3; color++) {
			if (point == 9)
				output[61][offset++] = 1;
			output[61][offset++] = point == 9 || !point ? 0 : 128;
		}
	ret = 0;
out:
	kfree(work);
	return ret;
}

#endif
