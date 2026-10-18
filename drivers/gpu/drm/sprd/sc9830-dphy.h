/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SC9830_DPHY_H_
#define _SC9830_DPHY_H_

#include <linux/io.h>

#define SC9830_PHY_RSTZ		0xa0
#define SC9830_PHY_IF_CFG		0xa4
#define SC9830_PHY_TEST_CTRL0	0xb4
#define SC9830_PHY_TEST_CTRL1	0xb8

static void sc9830_dphy_update(void __iomem *base, u32 reg, u32 mask, u32 value)
{
	writel((readl(base + reg) & ~mask) | (value & mask), base + reg);
}

static void sc9830_dphy_test_write(void __iomem *base, u8 code,
				   const u8 *data, unsigned int count)
{
	sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL0, BIT(1), BIT(1));
	writel(code, base + SC9830_PHY_TEST_CTRL1);
	sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL1, BIT(16), BIT(16));
	sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL0, BIT(1), 0);
	sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL1, BIT(16), 0);
	while (count) {
		writel(data[--count], base + SC9830_PHY_TEST_CTRL1);
		sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL0, BIT(1), BIT(1));
		sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL0, BIT(1), 0);
	}
}

static int sc9830_dphy_configure_500(void __iomem *base, unsigned long ref_rate,
				     unsigned int lanes)
{
	static const u8 setup[][2] = {
		{ 0x60, 0x83 }, { 0x70, 0x83 }, { 0x16, 0x70 },
		{ 0x44, 0x2c }, { 0x10, 0x90 }, { 0x19, 0x30 },
		{ 0x11, 0x06 }, { 0x12, 0xc4 }, { 0x17, 0x04 },
		{ 0x07, 0x04 }, { 0x22, 0x8b },
	};
	static const u8 feedback[] = { 0x1f, 0x82 };
	unsigned int i;

	if (ref_rate != 26000000 || !lanes || lanes > 4)
		return -EINVAL;
	sc9830_dphy_update(base, SC9830_PHY_RSTZ, BIT(1), 0);
	sc9830_dphy_update(base, SC9830_PHY_RSTZ, BIT(2), 0);
	sc9830_dphy_update(base, SC9830_PHY_RSTZ, BIT(0), 0);
	sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL0, BIT(0), BIT(0));
	sc9830_dphy_update(base, SC9830_PHY_TEST_CTRL0, BIT(0), 0);
	for (i = 0; i < ARRAY_SIZE(setup); i++)
		sc9830_dphy_test_write(base, setup[i][0], &setup[i][1], 1);
	sc9830_dphy_test_write(base, 0x18, feedback, ARRAY_SIZE(feedback));
	sc9830_dphy_update(base, SC9830_PHY_IF_CFG, 0x3, lanes - 1);
	sc9830_dphy_update(base, SC9830_PHY_IF_CFG, 0xff00, 0x1c00);
	sc9830_dphy_update(base, SC9830_PHY_RSTZ, BIT(2), BIT(2));
	sc9830_dphy_update(base, SC9830_PHY_RSTZ, BIT(0), BIT(0));
	sc9830_dphy_update(base, SC9830_PHY_RSTZ, BIT(1), BIT(1));
	return 0;
}

#endif
