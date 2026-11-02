// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/unaligned.h>

#include "mt6628.h"

int mt6628_read_factory_mac(struct device *device, u8 address[6])
{
	const struct firmware *fw;
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
	memcpy(address, fw->data + 4, 6);
out:
	release_firmware(fw);
	return ret;
}
