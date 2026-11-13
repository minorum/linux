/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_FRAME_H
#define __MT6628_FRAME_H

#include <linux/types.h>

#define MT6628_NATIVE_MAX_FRAME 1552
#define MT6628_PATCH_HEADER_SIZE 20
#define MT6628_PATCH_ENTRY_SIZE 12
#define MT6628_PATCH_WORDS 152
#define MT6628_NATIVE_RATE_V1 0x80

struct mt6628_native_rx {
	const u8 *frame;
	size_t len;
	u8 header_len;
	u8 channel;
	u8 rcpi;
	u8 rate_idx;
};

int mt6628_native_tx_packet(u8 *buf, size_t capacity, const u8 *frame,
			    size_t len, u8 peer, u8 token);
int mt6628_ethernet_tx_packet(u8 *buf, size_t capacity, const u8 *frame,
			      size_t len, u8 peer, u8 tc, bool more);
int mt6628_native_rx_packet(const u8 *packet, size_t len,
			    struct mt6628_native_rx *view);

#endif
