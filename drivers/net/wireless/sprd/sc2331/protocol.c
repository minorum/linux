// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/crc-itu-t.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "protocol.h"

int sc2331_decode_record(const u8 *data, size_t len,
			 struct sc2331_record *record, size_t *used)
{
	size_t payload, stride, offset = 0;
	u8 type;

	if (!len)
		return 0;
	if (len < 4)
		return -EMSGSIZE;
	type = data[0] & 0x7f;
	if (type == 0x7f || data[1] == 0xff)
		return 0;
	if (data[0] & 0x80 || type > 3)
		return -EPROTO;
	if (type == 3)
		return -EOPNOTSUPP;
	payload = get_unaligned_le16(data + 2);
	stride = 4 + ((payload + 3) & ~3UL);
	if (stride > len)
		return -EMSGSIZE;
	if (type == 2) {
		offset = data[1];
		if (offset > payload || payload - offset < 14)
			return -EPROTO;
	}
	record->type = type;
	record->id = data[1];
	record->data = data + 4 + offset;
	record->len = payload - offset;
	*used = stride;
	return 1;
}

int sc2331_decode_scan_frame(const u8 *data, size_t len,
			     struct sc2331_scan_frame *frame)
{
	u16 channel, frame_len;

	if (len < 8)
		return -EMSGSIZE;
	if (data[0] > 1)
		return -EPROTO;
	frame_len = get_unaligned_le16(data + 6);
	if (frame_len > len - 8)
		return -EMSGSIZE;
	channel = get_unaligned_le16(data + 2);
	if ((!channel || channel > 14) && (!data[0] || frame_len))
		return -EPROTO;
	frame->done = data[0];
	frame->channel = channel;
	frame->signal = (s16)get_unaligned_le16(data + 4);
	frame->data = data + 8;
	frame->len = frame_len;
	return 0;
}

ssize_t sc2331_encode_uart(u8 *data, size_t capacity, u16 command,
			   const u8 *payload, size_t len)
{
	size_t raw_len = len + 6, encoded, i, end;
	u16 crc;
	u8 byte;

	if (len > 1024 || capacity < raw_len + 2)
		return -EMSGSIZE;
	if ((len && !payload) || (!command && len))
		return -EINVAL;
	put_unaligned_be16(command, data);
	put_unaligned_be16(len, data + 2);
	if (len)
		memcpy(data + 4, payload, len);
	crc = command ? crc_itu_t(0, data, len + 4) : 0;
	put_unaligned_be16(crc, data + 4 + len);
	encoded = raw_len + 2;
	for (i = 0; i < raw_len; i++)
		if (data[i] == 0x7e || data[i] == 0x7d)
			encoded++;
	if (encoded > capacity)
		return -EMSGSIZE;
	end = encoded - 1;
	data[end] = 0x7e;
	for (i = raw_len; i; ) {
		byte = data[--i];
		if (byte == 0x7e || byte == 0x7d) {
			data[--end] = byte ^ 0x20;
			data[--end] = 0x7d;
		} else {
			data[--end] = byte;
		}
	}
	data[0] = 0x7e;
	return encoded;
}

int sc2331_uart_feed(struct sc2331_uart_stream *stream, u8 byte)
{
	if (!stream->len) {
		if (byte == 0x7e)
			stream->data[stream->len++] = byte;
		return 0;
	}
	if (byte == 0x7e && stream->len == 1)
		return 0;
	if (stream->len == sizeof(stream->data))
		return -EMSGSIZE;
	stream->data[stream->len++] = byte;
	return byte == 0x7e;
}

int sc2331_decode_uart(u8 *data, size_t len, struct sc2331_boot_frame *frame)
{
	size_t i, raw_len = 0;
	u16 payload_len;
	u8 byte;

	if (len < 8 || len > 2 * (1024 + 6) + 2)
		return -EMSGSIZE;
	if (data[0] != 0x7e || data[len - 1] != 0x7e)
		return -EPROTO;
	for (i = 1; i < len - 1; i++) {
		byte = data[i];
		if (byte == 0x7e)
			return -EPROTO;
		if (byte == 0x7d) {
			if (++i >= len - 1 ||
			    (data[i] != 0x5e && data[i] != 0x5d))
				return -EPROTO;
			byte = data[i] ^ 0x20;
		}
		data[raw_len++] = byte;
	}
	if (raw_len < 6)
		return -EMSGSIZE;
	payload_len = get_unaligned_be16(data + 2);
	if (payload_len > 1024 || payload_len != raw_len - 6)
		return -EMSGSIZE;
	if (crc_itu_t(0, data, raw_len - 2) !=
	    get_unaligned_be16(data + raw_len - 2))
		return -EBADMSG;
	frame->command = get_unaligned_be16(data);
	frame->len = payload_len;
	frame->data = data + 4;
	return 0;
}
