/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SC2331_PROTOCOL_H
#define SC2331_PROTOCOL_H

#include <linux/types.h>

struct sdio_driver;
extern struct sdio_driver sc2331_sdio_driver;

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

struct sc2331_connect_result {
	const u8 *bssid;
	const u8 *request_ie;
	const u8 *response_ie;
	size_t request_ie_len;
	size_t response_ie_len;
	u8 status;
};

struct sc2331_boot_frame {
	const u8 *data;
	u16 command;
	u16 len;
};

struct sc2331_uart_stream {
	u8 data[128];
	size_t len;
};

int sc2331_decode_record(const u8 *data, size_t len,
			 struct sc2331_record *record, size_t *used);
int sc2331_decode_scan_frame(const u8 *data, size_t len,
			     struct sc2331_scan_frame *frame);
int sc2331_decode_connect_result(const u8 *data, size_t len,
				 struct sc2331_connect_result *result);
ssize_t sc2331_encode_ethernet(u8 *data, size_t capacity, u32 sequence,
			       u32 packet_sequence, const u8 *frame, size_t len);
ssize_t sc2331_encode_uart(u8 *data, size_t capacity, u16 command,
			   const u8 *payload, size_t len);
int sc2331_decode_uart(u8 *data, size_t len, struct sc2331_boot_frame *frame);
int sc2331_uart_feed(struct sc2331_uart_stream *stream, u8 byte);
int sc2331_decode_sdio_ack(const u8 *data, size_t len);
ssize_t sc2331_encode_command(u8 *data, size_t capacity, u8 command,
			      u32 sequence, const u8 *payload, size_t len);
ssize_t sc2331_encode_sdio(u8 *data, size_t capacity, u16 command,
			   const u8 *payload, size_t len);

#endif
