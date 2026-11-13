// SPDX-License-Identifier: GPL-2.0-only
#include <linux/crc32.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "mt6628.h"

int mt6628_fw_validate(const struct firmware *fw)
{
	u32 count;
	size_t table_end;
	unsigned int i;

	if (fw->size < 16 || fw->size > U32_MAX)
		return -EINVAL;
	if (get_unaligned_le32(fw->data) != 0x574b544d ||
	    get_unaligned_le32(fw->data + 4) !=
	    ~crc32_le(~0U, fw->data + 8, fw->size - 8))
		return -EBADMSG;
	count = get_unaligned_le32(fw->data + 8);
	if (!count || count > (fw->size - 16) / 16)
		return -EINVAL;
	table_end = 16 + (size_t)count * 16;
	for (i = 0; i < count; i++) {
		const u8 *entry = fw->data + 16 + i * 16;
		u32 offset = get_unaligned_le32(entry);
		u32 len = get_unaligned_le32(entry + 8);
		u32 address = get_unaligned_le32(entry + 12);

		if (!len || offset < table_end || offset > fw->size ||
		    len > fw->size - offset || len - 1 > U32_MAX - address)
			return -EINVAL;
	}
	return 0;
}

int mt6628_patch_validate(const struct firmware *fw, const struct firmware *patch)
{
	u32 base, len;
	unsigned int i, j;
	int ret = mt6628_fw_validate(fw);

	if (ret)
		return ret;
	if (!patch || patch->size != MT6628_PATCH_HEADER_SIZE +
	    MT6628_PATCH_WORDS * MT6628_PATCH_ENTRY_SIZE)
		return -EINVAL;
	if (get_unaligned_le32(patch->data) != 0x504e544d ||
	    get_unaligned_le32(patch->data + 8) != 1 ||
	    get_unaligned_le32(patch->data + 12) != get_unaligned_le32(fw->data + 4) ||
	    get_unaligned_le32(patch->data + 16) != MT6628_PATCH_WORDS)
		return -EINVAL;
	if (get_unaligned_le32(patch->data + 4) !=
	    ~crc32_le(~0U, patch->data + 8, patch->size - 8))
		return -EBADMSG;
	base = get_unaligned_le32(fw->data + 28);
	len = get_unaligned_le32(fw->data + 24);
	if (base != 0x60000 || len < 4 || len > 0x25000)
		return -EINVAL;
	for (i = 0; i < MT6628_PATCH_WORDS; i++) {
		const u8 *entry = patch->data + MT6628_PATCH_HEADER_SIZE +
				  i * MT6628_PATCH_ENTRY_SIZE;
		u32 address = get_unaligned_le32(entry);

		if ((address & 3) || address < base || address - base > len - 4 ||
		    get_unaligned_le32(entry + 4) == get_unaligned_le32(entry + 8))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (get_unaligned_le32(patch->data + MT6628_PATCH_HEADER_SIZE +
					      j * MT6628_PATCH_ENTRY_SIZE) == address)
				return -EINVAL;
	}
	return 0;
}

int mt6628_init_download(u8 *buf, size_t capacity, const u8 *data,
			 size_t len, u32 address, u8 sequence)
{
	size_t size;

	if (!len || len > 2048 || len - 1 > U32_MAX - address)
		return -EINVAL;
	size = ALIGN(24 + len, 4);
	if (capacity < size)
		return -ENOSPC;
	memset(buf, 0, size);
	put_unaligned_le16(size, buf);
	buf[4] = 1;
	buf[5] = sequence;
	put_unaligned_le32(address, buf + 8);
	put_unaligned_le32(len, buf + 12);
	put_unaligned_le32(~crc32_le(~0U, data, len), buf + 16);
	put_unaligned_le32(0x80000001, buf + 20);
	memcpy(buf + 24, data, len);
	return size;
}

int mt6628_init_result(const u8 *buf, size_t len, u8 sequence)
{
	if (len != 8 || get_unaligned_le16(buf) != 8 ||
	    buf[2] != 1 || buf[3] != sequence)
		return -EPROTO;
	return buf[4] ? -EREMOTEIO : 0;
}

int mt6628_mcu_command(u8 *buf, size_t capacity, const u8 *data, size_t len,
		       u8 command, bool set, u8 sequence)
{
	size_t size;

	if (len > 4084)
		return -EMSGSIZE;
	size = ALIGN(8 + len, 4);
	if (capacity < size)
		return -ENOSPC;
	if (len)
		memmove(buf + 8, data, len);
	memset(buf, 0, 8);
	memset(buf + 8 + len, 0, size - 8 - len);
	put_unaligned_le16(size, buf);
	buf[3] = 0x50;
	buf[4] = command;
	buf[5] = set;
	buf[6] = sequence;
	return size;
}

int mt6628_mcu_event(const u8 *buf, size_t len, u8 event, u8 sequence)
{
	if (len < 8 || get_unaligned_le16(buf) != len ||
	    get_unaligned_le16(buf + 2) != 1 || buf[4] != event ||
	    buf[5] != sequence)
		return -EPROTO;
	return 0;
}

int mt6628_ethernet_tx_packet(u8 *buf, size_t capacity, const u8 *frame,
			      size_t len, u8 peer, u8 tc, bool more)
{
	size_t size;

	if (len > 1518)
		return -EMSGSIZE;
	if (!buf || !frame || len < 14 || peer >= 20 || (tc != 1 && tc != 5))
		return -EINVAL;
	size = ALIGN(16 + len, 4);
	if (capacity < size)
		return -ENOSPC;
	memmove(buf + 16, frame, len);
	memset(buf, 0, 16);
	memset(buf + 16 + len, 0, size - 16 - len);
	put_unaligned_le16(16 + len, buf);
	buf[2] = 14;
	buf[3] = tc << 2;
	buf[4] = 14;
	buf[10] = tc == 5 ? 0xff : peer;
	buf[11] = 0x20 | (tc == 1 && more ? 3 : 0);
	return size;
}

int mt6628_native_tx_packet(u8 *buf, size_t capacity, const u8 *frame,
			    size_t len, u8 peer, u8 token)
{
	size_t size;
	u16 fc;
	bool management;

	if (len > MT6628_NATIVE_MAX_FRAME)
		return -EMSGSIZE;
	if (!buf || !frame || len < 24 || peer >= 20)
		return -EINVAL;
	fc = get_unaligned_le16(frame);
	management = !(fc & 0x000c);
	if ((fc & 3) || (!management && (fc & 0x00fc) != 8) ||
	    (fc & 0x8000) || (fc & 0x0300) != (management ? 0 : 0x0100) ||
	    (fc & 0x0400) || (get_unaligned_le16(frame + 22) & 15))
		return -EOPNOTSUPP;
	if (management && len > 1536)
		return -EMSGSIZE;
	if ((fc & 0x4000) && len < 40)
		return -EINVAL;
	size = ALIGN(16 + len, 4);
	if (capacity < size)
		return -ENOSPC;
	memmove(buf + 16, frame, len);
	memset(buf, 0, 16);
	memset(buf + 16 + len, 0, size - 16 - len);
	put_unaligned_le16(16 + len, buf);
	buf[2] = 20;
	buf[3] = management ? 0xd0 : 0x10;
	buf[4] = 24;
	buf[5] = 0x80;
	buf[10] = peer;
	buf[12] = token;
	buf[13] = management ? 5 : 1;
	return size;
}

int mt6628_native_rx_packet(const u8 *packet, size_t len,
			    struct mt6628_native_rx *view)
{
	static const u8 rates[] = { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 };
	unsigned int offset, header = 24, rate;
	u16 fc;

	if (!packet || !view || len < 12 || get_unaligned_le16(packet) != len ||
	    (get_unaligned_le16(packet + 2) & 3) != 3 || !(packet[5] & 1) ||
	    (packet[4] & 1) || !packet[10] || packet[10] > 14)
		return -EPROTO;
	offset = 12 + (packet[4] & 3);
	if (len < offset + 24)
		return -EPROTO;
	if (len - offset > MT6628_NATIVE_MAX_FRAME)
		return -EMSGSIZE;
	fc = get_unaligned_le16(packet + offset);
	if (!(packet[11] & MT6628_NATIVE_RATE_V1) &&
	    ((fc & 0x000c) || packet[11]))
		return -EOPNOTSUPP;
	if ((fc & 3) || ((fc & 0x000c) != 0 && (fc & 0x000c) != 8))
		return -EOPNOTSUPP;
	if ((fc & 0x000c) == 8) {
		if ((fc & 0x0300) != 0x0200)
			return -EOPNOTSUPP;
		if (fc & 0x0080)
			header += 2 + ((fc & 0x8000) ? 4 : 0);
	} else {
		if (fc & 0x0300)
			return -EOPNOTSUPP;
		if (fc & 0x8000)
			header += 4;
	}
	if ((packet[4] >> 2) != header || len < offset + header ||
	    ((fc & 0x4000) && len < offset + header + 16))
		return -EPROTO;
	rate = 0;
	if (packet[11] & MT6628_NATIVE_RATE_V1) {
		for (; rate < sizeof(rates); rate++)
			if ((packet[11] & ~MT6628_NATIVE_RATE_V1) == rates[rate])
				break;
		if (rate == sizeof(rates))
			return -EOPNOTSUPP;
	}
	view->frame = packet + offset;
	view->len = len - offset;
	view->header_len = header;
	view->channel = packet[10];
	view->rcpi = packet[9];
	view->rate_idx = rate;
	return 0;
}
