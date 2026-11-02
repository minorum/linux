// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "mt6628_mcu.h"

static int mt6628_sdio_poll(struct sdio_func *func, u32 reg, u32 mask,
			    unsigned int timeout)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout);
	int ret;

	for (;;) {
		u32 value = sdio_readl(func, reg, &ret);

		if (ret)
			return ret;
		if (value & mask)
			return 0;
		if (time_after_eq(jiffies, end))
			return -ETIMEDOUT;
		usleep_range(1000, 2000);
	}
}

static int mt6628_sdio_credit(struct sdio_func *func, u8 *credits)
{
	unsigned long end = jiffies + msecs_to_jiffies(12800);
	int ret;

	if (*credits)
		return 0;
	for (;;) {
		u32 value = sdio_readl(func, 0x20, &ret);

		if (ret)
			return ret;
		sdio_readl(func, 0x24, &ret);
		if (ret)
			return ret;
		value &= 0xff;
		if (value > 8)
			return -EPROTO;
		if (value) {
			*credits = value;
			return 0;
		}
		if (time_after_eq(jiffies, end))
			return -ETIMEDOUT;
		msleep(50);
	}
}

static int mt6628_sdio_result(struct sdio_func *func, u8 sequence, u8 *buf)
{
	int ret = mt6628_sdio_poll(func, 0x50, 0xffff, 1000);
	u32 len;

	if (ret)
		return ret;
	len = sdio_readl(func, 0x50, &ret) & 0xffff;
	if (ret)
		return ret;
	if (len != 8)
		return -EPROTO;
	ret = sdio_readsb(func, buf, 0x30, len);
	if (ret)
		return ret;
	return mt6628_init_result(buf, len, sequence);
}

int mt6628_sdio_load_firmware(struct sdio_func *func, const struct firmware *fw)
{
	u8 credits = 8, sequence = 0;
	u32 value, count, i;
	u8 *buf;
	int ret;

	ret = mt6628_fw_validate(fw);
	if (ret)
		return ret;
	if (func->cur_blksize != 512)
		return -EINVAL;
	buf = kzalloc(2560, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	sdio_claim_host(func);
	sdio_writel(func, BIT(9), 0x04, &ret);
	if (ret)
		goto out;
	ret = mt6628_sdio_poll(func, 0x04, BIT(8), 8192);
	if (ret)
		goto out;
	value = sdio_readl(func, 0x00, &ret);
	if (ret)
		goto out;
	if ((value & 0xffff) != 0x6628) {
		ret = -ENODEV;
		goto out;
	}
	if (value & BIT(21)) {
		ret = -EALREADY;
		goto out;
	}
	sdio_writeb(func, 2, 0x04, &ret);
	if (ret)
		goto out;
	value = sdio_readl(func, 0x0c, &ret);
	if (ret)
		goto out;
	sdio_writel(func, value & ~0x100f0, 0x0c, &ret);
	if (ret)
		goto out;
	sdio_writel(func, 0xffffff0f, 0x14, &ret);
	if (ret)
		goto out;
	count = get_unaligned_le32(fw->data + 8);
	for (i = 0; i < count; i++) {
		const u8 *entry = fw->data + 16 + i * 16;
		u32 offset = get_unaligned_le32(entry);
		u32 size = get_unaligned_le32(entry + 8);
		u32 address = get_unaligned_le32(entry + 12);
		u32 done = 0;

		while (done < size) {
			size_t len = min_t(u32, size - done, 2048);
			int packet, wire;

			ret = mt6628_sdio_credit(func, &credits);
			if (ret)
				goto out;
			packet = mt6628_init_download(buf, 2560,
						      fw->data + offset + done, len,
						      address + done, ++sequence);
			if (packet < 0) {
				ret = packet;
				goto out;
			}
			wire = packet >= 512 ? roundup(packet, 512) : packet;
			memset(buf + packet, 0, wire - packet);
			ret = sdio_writesb(func, 0x28, buf, wire);
			if (ret)
				goto out;
			credits--;
			ret = mt6628_sdio_result(func, sequence, buf);
			if (ret)
				goto out;
			done += len;
		}
	}
	ret = mt6628_sdio_credit(func, &credits);
	if (ret)
		goto out;
	memset(buf, 0, 16);
	put_unaligned_le16(16, buf);
	buf[4] = 2;
	buf[5] = ++sequence;
	ret = sdio_writesb(func, 0x28, buf, 16);
	if (ret)
		goto out;
	ret = mt6628_sdio_poll(func, 0x00, BIT(21), 5130);
	if (ret)
		goto out;
	value = sdio_readl(func, 0x10, &ret);
	if (ret)
		goto out;
	if (value & BIT(0)) {
		sdio_readl(func, 0x20, &ret);
		if (ret)
			goto out;
		sdio_readl(func, 0x24, &ret);
	}
out:
	sdio_release_host(func);
	kfree(buf);
	return ret;
}
