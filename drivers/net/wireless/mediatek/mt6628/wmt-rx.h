/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_WMT_RX_H
#define __MT6628_WMT_RX_H

#include <linux/types.h>

#define MT6628_WMT_RX_MAX 14

struct regmap;

struct mt6628_wmt_rx_entry {
	u32 address;
	u16 len;
	u8 descriptor[24];
	u8 hif[64];
	u8 rfb[52];
	u8 rx[24];
};

struct mt6628_wmt_rx {
	unsigned int count;
	struct mt6628_wmt_rx_entry entries[MT6628_WMT_RX_MAX];
};

int mt6628_wmt_rx_capture(struct regmap *map, const u16 *lengths,
			  unsigned int count, struct mt6628_wmt_rx *snapshot);
bool mt6628_wmt_rx_matches(const struct mt6628_wmt_rx *snapshot,
			   unsigned int index, const u8 *packet, size_t len);
int mt6628_wmt_rx_frame(const struct mt6628_wmt_rx_entry *entry,
			const u8 *packet, size_t len, const u8 *bssid,
			const u8 *local, u8 *frame, size_t capacity);

#endif
