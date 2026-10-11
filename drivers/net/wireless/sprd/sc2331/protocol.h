/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SC2331_PROTOCOL_H
#define SC2331_PROTOCOL_H

#include <linux/types.h>

struct sc2331_record {
	const u8 *data;
	size_t len;
	u8 type;
	u8 id;
};

struct sc2331_scan_frame {
	const u8 *data;
	u16 len;
	u16 channel;
	s16 signal;
	u8 done;
};

struct sc2331_boot_frame {
	const u8 *data;
	u16 command;
	u16 len;
};

int sc2331_decode_record(const u8 *data, size_t len,
			 struct sc2331_record *record, size_t *used);
int sc2331_decode_scan_frame(const u8 *data, size_t len,
			     struct sc2331_scan_frame *frame);
ssize_t sc2331_encode_uart(u8 *data, size_t capacity, u16 command,
			   const u8 *payload, size_t len);
int sc2331_decode_uart(u8 *data, size_t len, struct sc2331_boot_frame *frame);

#endif
