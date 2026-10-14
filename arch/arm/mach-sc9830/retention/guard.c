// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/regmap.h>

static int sc9830_retention_guard_check(struct regmap *map, u32 *ticks)
{
	static const unsigned int regs[] = { 0x940, 0x48, 0x800, 0x808, 0x44, 0x40 };
	unsigned int values[ARRAY_SIZE(regs)], high, low, after, i, attempt;
	u32 count;
	int ret;

	if (!map || !ticks)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = regmap_read(map, regs[i], &values[i]);
		if (ret)
			return ret;
	}
	if (values[0] != 0x2723 || values[1] != 0xe ||
	    (values[2] & 6) != 6 || (values[3] & 6) != 6 ||
	    values[4] != 90 || values[5])
		return -EINVAL;
	for (attempt = 0; attempt < 3; attempt++) {
		ret = regmap_read(map, 0x5c, &high);
		if (ret)
			return ret;
		ret = regmap_read(map, 0x58, &low);
		if (ret)
			return ret;
		ret = regmap_read(map, 0x5c, &after);
		if (ret)
			return ret;
		if (high > 0xffff || after > 0xffff || low > 0xffff)
			return -EIO;
		if (high != after)
			continue;
		count = (high << 16) | low;
		if (count < 30 * 32768 || count > 180 * 32768)
			return -ETIME;
		for (i = 1; i < 4; i++) {
			ret = regmap_read(map, regs[i], &values[i]);
			if (ret)
				return ret;
		}
		if (values[1] != 0xe || (values[2] & 6) != 6 || (values[3] & 6) != 6)
			return -EINVAL;
		*ticks = count;
		return 0;
	}
	return -EAGAIN;
}
