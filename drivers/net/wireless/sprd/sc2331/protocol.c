// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
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
