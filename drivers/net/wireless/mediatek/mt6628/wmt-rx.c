// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "wmt-rx.h"

static bool mt6628_wmt_ram(u32 address, unsigned int size)
{
	if (!size || size > 0x16588 || (address & 3) || (size & 3))
		return false;
	return (address >= 0x200000 && address <= 0x216588 - size) ||
		(size <= 0x9798 && address >= 0xf0098400 &&
		 address <= 0xf00a1b98 - size);
}

static int mt6628_wmt_window(struct regmap *map, u32 address, u8 *data,
			     unsigned int size)
{
	unsigned int i, value;
	int ret;

	if (!mt6628_wmt_ram(address, size))
		return -ERANGE;
	for (i = 0; i < size; i += 4) {
		ret = regmap_read(map, address + i, &value);
		if (ret)
			return ret;
		put_unaligned_le32(value, data + i);
	}
	return 0;
}

static int mt6628_wmt_same(struct regmap *map, u32 address,
			   const u8 *data, unsigned int size)
{
	u8 again[64];
	int ret;

	if (size > sizeof(again))
		return -EINVAL;
	ret = mt6628_wmt_window(map, address, again, size);
	if (ret)
		return ret;
	return memcmp(data, again, size) ? -EAGAIN : 0;
}

static int mt6628_wmt_rx_guard(struct regmap *map, u32 soft, unsigned int count)
{
	unsigned int before, hardware, after;
	int ret;

	ret = regmap_read(map, soft + 0x150, &before);
	if (ret)
		return ret;
	ret = regmap_read(map, 0x50000148, &hardware);
	if (ret)
		return ret;
	ret = regmap_read(map, soft + 0x150, &after);
	if (ret)
		return ret;
	if (before != after || before < count || before > 16 || (hardware & 31) > 16 ||
	    before + (hardware & 31) != 16)
		return -EAGAIN;
	return 0;
}

int mt6628_wmt_rx_capture(struct regmap *map, const u16 *lengths,
			  unsigned int count, struct mt6628_wmt_rx *snapshot)
{
	u32 roots[] = { 0x80000008, 0xf0099a54, 0xf0098400, 0xf0098408 };
	unsigned int values[4], value, head, address, queued, i, j;
	int ret;

	snapshot->count = 0;
	if (!count || count > MT6628_WMT_RX_MAX)
		return -EINVAL;
	for (i = 0; i < 4; i++) {
		ret = regmap_read(map, roots[i], &values[i]);
		if (ret)
			return ret;
	}
	if (values[0] != 0x6628 || values[1] != 5)
		return -EPROTO;
	if (!mt6628_wmt_ram(values[2], 0x24) ||
	    !mt6628_wmt_ram(values[3], 0x15c))
		return -ERANGE;
	ret = mt6628_wmt_rx_guard(map, values[3], count);
	if (ret)
		return ret;
	ret = regmap_read(map, values[2] + 0x18, &head);
	if (ret)
		return ret;
	ret = regmap_read(map, values[2] + 0x20, &queued);
	if (ret)
		return ret;
	if (queued < count || queued > MT6628_WMT_RX_MAX)
		return -EAGAIN;
	address = head;
	for (i = 0; i < count; i++) {
		struct mt6628_wmt_rx_entry *entry = &snapshot->entries[i];
		u32 rfb, rx, hif;

		if (lengths[i] < 8 || lengths[i] > 2352)
			return -EPROTO;
		for (j = 0; j < i; j++)
			if (address == snapshot->entries[j].address)
				return -ELOOP;
		memset(entry, 0, sizeof(*entry));
		entry->address = address;
		entry->len = lengths[i];
		ret = mt6628_wmt_window(map, address, entry->descriptor, 24);
		if (ret)
			return ret;
		hif = get_unaligned_le32(entry->descriptor + 12);
		if (get_unaligned_le16(entry->descriptor + 16) != entry->len)
			return -EPROTO;
		ret = mt6628_wmt_window(map, hif, entry->hif,
					ALIGN(min_t(unsigned int, entry->len, 64), 4));
		if (ret)
			return ret;
		if (get_unaligned_le16(entry->hif) != entry->len)
			return -EPROTO;
		if (!(get_unaligned_le16(entry->hif + 2) & 3)) {
			if (entry->len < 12)
				return -EPROTO;
			rfb = get_unaligned_le32(entry->descriptor + 20);
			ret = mt6628_wmt_window(map, rfb, entry->rfb, 52);
			if (ret)
				return ret;
			if (get_unaligned_le32(entry->rfb + 40) != hif)
				return -EPROTO;
			rx = get_unaligned_le32(entry->rfb + 8);
			ret = mt6628_wmt_window(map, rx, entry->rx, 24);
			if (ret)
				return ret;
			ret = mt6628_wmt_same(map, rfb, entry->rfb, 52);
			if (ret)
				return ret;
			ret = mt6628_wmt_same(map, rx, entry->rx, 24);
			if (ret)
				return ret;
		}
		ret = mt6628_wmt_same(map, address, entry->descriptor, 24);
		if (ret)
			return ret;
		ret = mt6628_wmt_same(map, hif, entry->hif,
				      ALIGN(min_t(unsigned int, entry->len, 64), 4));
		if (ret)
			return ret;
		address = get_unaligned_le32(entry->descriptor);
	}
	for (i = 0; i < 4; i++) {
		ret = regmap_read(map, roots[i], &value);
		if (ret)
			return ret;
		if (value != values[i])
			return -EAGAIN;
	}
	ret = regmap_read(map, values[2] + 0x18, &value);
	if (ret)
		return ret;
	if (value != head)
		return -EAGAIN;
	ret = mt6628_wmt_rx_guard(map, values[3], count);
	if (ret)
		return ret;
	snapshot->count = count;
	return 0;
}

bool mt6628_wmt_rx_matches(const struct mt6628_wmt_rx *snapshot,
			   unsigned int index, const u8 *packet, size_t len)
{
	const struct mt6628_wmt_rx_entry *entry;

	if (index >= snapshot->count)
		return false;
	entry = &snapshot->entries[index];
	return len == entry->len &&
		!memcmp(packet, entry->hif, min_t(size_t, len, 64));
}

int mt6628_wmt_rx_frame(const struct mt6628_wmt_rx_entry *entry,
			const u8 *packet, size_t len, const u8 *bssid,
			const u8 *local, u8 *frame, size_t capacity)
{
	static const u8 snap[] = { 0xaa, 0xaa, 3, 0, 0, 0 };
	static const u8 zero[6];
	const u8 *ether, *pn = entry->rx + 18;
	unsigned int offset, payload, llc, iv, size;
	u32 rx, mac;
	u16 fc, sequence, protocol;

	if (len != entry->len || len < 12 ||
	    get_unaligned_le16(entry->rfb + 44) != len ||
	    memcmp(packet, entry->hif, min_t(size_t, len, 64)) ||
	    get_unaligned_le16(packet) != len ||
	    (get_unaligned_le16(packet + 2) & 3) || (packet[5] & 1))
		return -EPROTO;
	fc = get_unaligned_le16(entry->rfb + 50);
	sequence = get_unaligned_le16(entry->rfb + 48);
	if ((fc & 0x00ff) != 8 || (fc & 0x0300) != 0x0200 ||
	    (fc & 0x8400) || (sequence & 15))
		return -EOPNOTSUPP;
	if ((sequence >> 4) != (get_unaligned_le16(packet + 6) & 0xfff) ||
	    (entry->rx[11] & 63) != 24)
		return -EPROTO;
	if ((entry->rx[17] & 15) != ((fc & 0x4000) ? 4 : 0))
		return -EOPNOTSUPP;
	rx = get_unaligned_le32(entry->rfb + 8);
	mac = get_unaligned_le32(entry->rfb + 20);
	if (rx > 0xffffff94 || mac != ((rx + 0x68 + (entry->rx[17] >> 6)) & ~1U) ||
	    get_unaligned_le32(entry->rx) != mac || mac > 0xfffffffb ||
	    get_unaligned_le32(entry->rfb + 40) != mac + 4)
		return -EPROTO;
	offset = 12 + (packet[4] & 3);
	if (len < offset + 14 || len - offset > 1518)
		return -EPROTO;
	ether = packet + offset;
	if ((!(ether[0] & 1) && memcmp(ether, local, 6)) ||
	    (ether[6] & 1) || !memcmp(ether + 6, zero, 6))
		return -EPROTO;
	protocol = get_unaligned_be16(ether + 12);
	payload = len - offset - 14;
	if (protocol > 1500 && protocol < 1536)
		return -EPROTO;
	llc = protocol >= 1536 ? 8 : 0;
	if ((!llc && protocol != payload) ||
	    get_unaligned_le16(entry->rx + 8) != 24 + llc + payload)
		return -EPROTO;
	iv = (fc & 0x4000) ? 8 : 0;
	size = 24 + iv + llc + payload;
	if (capacity < size)
		return -ENOSPC;
	memset(frame, 0, 24);
	put_unaligned_le16(fc, frame);
	memcpy(frame + 4, ether, 6);
	memcpy(frame + 10, bssid, 6);
	memcpy(frame + 16, ether + 6, 6);
	put_unaligned_le16(sequence, frame + 22);
	if (iv) {
		frame[24] = pn[0];
		frame[25] = pn[1];
		frame[26] = 0;
		frame[27] = 0x20 | ((entry->rx[17] & 0x30) << 2);
		memcpy(frame + 28, pn + 2, 4);
	}
	if (llc) {
		memcpy(frame + 24 + iv, snap, sizeof(snap));
		if (protocol == 0x80f3 || protocol == 0x8137)
			frame[24 + iv + 5] = 0xf8;
		memcpy(frame + 24 + iv + 6, ether + 12, 2);
	}
	memcpy(frame + 24 + iv + llc, ether + 14, payload);
	return size;
}
