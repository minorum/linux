// SPDX-License-Identifier: GPL-2.0-only
#include <linux/crc32.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "mt6628_mcu.h"

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
