// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/unaligned.h>

#include "mt6628.h"

static u8 mt6628_factory_codeword(u16 code)
{
	static const struct {
		u16 code;
		u8 value;
	} table[] = {
		{ 0x100, 0x80 }, { 0x104, 0x80 }, { 0x128, 0x80 },
		{ 0x14c, 0x80 }, { 0x170, 0x80 }, { 0x194, 0x94 },
		{ 0x1b8, 0xb8 }, { 0x1dc, 0xdc }, { 0x000, 0x00 },
		{ 0x024, 0x24 }, { 0x048, 0x48 }, { 0x06c, 0x6c },
		{ 0x090, 0x7f }, { 0x0b4, 0x7f }, { 0x0d8, 0x7f },
		{ 0x0fc, 0x7f }, { 0x0ff, 0x7f },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(table); i++)
		if (table[i].code == code)
			return table[i].value;
	return 0;
}

int mt6628_read_factory(struct device *device, struct mt6628_factory *factory)
{
	struct mt6628_factory result = {};
	const struct firmware *fw;
	u32 legacy, value;
	unsigned int i;
	int ret;

	ret = request_firmware(&fw, "mediatek/mt6628-nvram.bin", device);
	if (ret)
		return ret;
	if (fw->size < 512) {
		ret = -EINVAL;
		goto out;
	}
	if (get_unaligned_le16(fw->data + 2) > 0x200 ||
	    get_unaligned_le16(fw->data + 0x102) > 0x200) {
		ret = -EPROTONOSUPPORT;
		goto out;
	}
	memcpy(result.mac_addr, fw->data + 4, sizeof(result.mac_addr));
	memcpy(result.tx_power, fw->data + 12, sizeof(result.tx_power));
	memcpy(result.phy, fw->data + 0x34, sizeof(result.phy));
	result.set_tx_power = get_unaligned_le16(fw->data) != 1 && fw->data[0xc4];
	result.edge_enabled = fw->data[0xc6];
	result.enable_5ghz = fw->data[0xc5] && fw->data[0x106];
	legacy = get_unaligned_le32(result.phy + 72);
	if (legacy & BIT(27) && !get_unaligned_le32(result.phy + 28)) {
		value = BIT(24);
		for (i = 0; i < 3; i++)
			value |= (u32)mt6628_factory_codeword((legacy >> (i * 9)) &
							    0x1ff) << (i * 8);
		put_unaligned_le32(value, result.phy + 28);
	}
	*factory = result;
out:
	release_firmware(fw);
	return ret;
}
