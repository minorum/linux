// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mmc/core.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "mt6628.h"

const u8 mt6628_sdio_tx_max[6] = { 1, 20, 1, 1, 4, 1 };

int mt6628_sdio_decode_status(const u8 *buf, size_t len,
			      struct mt6628_sdio_status *status)
{
	struct mt6628_sdio_status decoded = {};
	unsigned int port, i;

	if (len != 88)
		return -EPROTO;
	decoded.isr = get_unaligned_le32(buf);
	memcpy(decoded.tx, buf + 4, sizeof(decoded.tx));
	for (i = 0; i < 6; i++)
		if (decoded.tx[i])
			decoded.isr |= BIT(0);
	for (port = 0; port < 2; port++) {
		decoded.rx_count[port] = get_unaligned_le16(buf + 12 + port * 2);
		if (decoded.rx_count[port] > 16)
			return -EPROTO;
		for (i = 0; i < decoded.rx_count[port]; i++) {
			u16 size = get_unaligned_le16(buf + 16 + port * 32 + i * 2);

			if (!size || size > 2352)
				return -EPROTO;
			decoded.rx_len[port][i] = size;
		}
	}
	decoded.mailbox[0] = get_unaligned_le32(buf + 80);
	decoded.mailbox[1] = get_unaligned_le32(buf + 84);
	*status = decoded;
	return 0;
}

int mt6628_sdio_return_credits(u8 credits[6], const u8 returned[6])
{
	unsigned int i, total = 0;

	for (i = 0; i < 6; i++) {
		if (credits[i] > mt6628_sdio_tx_max[i] ||
		    returned[i] > mt6628_sdio_tx_max[i] - credits[i])
			return -EPROTO;
		total += returned[i];
	}
	for (i = 0; i < 6; i++)
		credits[i] += returned[i];
	return total;
}

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

static int mt6628_sdio_patch_xfer(struct sdio_func *func, const struct firmware *patch,
				  u8 *credits, u8 *sequence, u8 *buf,
				  unsigned int index, bool set, u32 *value)
{
	const u8 *entry;
	u32 address, len;
	int ret;

	if (index >= MT6628_PATCH_WORDS)
		return -EINVAL;
	entry = patch->data + MT6628_PATCH_HEADER_SIZE + index * MT6628_PATCH_ENTRY_SIZE;
	address = get_unaligned_le32(entry);
	ret = mt6628_sdio_credit(func, credits);
	if (ret)
		return ret;
	memset(buf, 0, 20);
	put_unaligned_le16(20, buf);
	buf[4] = 3;
	buf[5] = ++*sequence;
	buf[8] = set;
	put_unaligned_le32(address, buf + 12);
	if (set)
		put_unaligned_le32(get_unaligned_le32(entry + 8), buf + 16);
	ret = sdio_writesb(func, 0x28, buf, 20);
	if (ret)
		return ret;
	(*credits)--;
	ret = mt6628_sdio_poll(func, 0x50, 0xffff, 1000);
	if (ret)
		return ret;
	len = sdio_readl(func, 0x50, &ret) & 0xffff;
	if (ret)
		return ret;
	if (len != 8 && len != 12)
		return -EPROTO;
	ret = sdio_readsb(func, buf, 0x30, len);
	if (ret)
		return ret;
	if (len == 8) {
		ret = mt6628_init_result(buf, len, *sequence);
		return ret ? ret : (set ? 0 : -EPROTO);
	}
	if (get_unaligned_le16(buf) != 12 || buf[2] != 2 || buf[3] != *sequence ||
	    get_unaligned_le32(buf + 4) != address ||
	    (set && get_unaligned_le32(buf + 8) != get_unaligned_le32(entry + 8)))
		return -EPROTO;
	if (value)
		*value = get_unaligned_le32(buf + 8);
	return 0;
}

static int mt6628_sdio_apply_patch(struct sdio_func *func, const struct firmware *patch,
				   u8 *credits, u8 *sequence, u8 *buf)
{
	unsigned long end = jiffies + msecs_to_jiffies(10000);
	unsigned int checked = 0, acknowledged = 0, verified = 0, i;
	u32 value = 0;
	int ret = 0;

	for (i = 0; i < MT6628_PATCH_WORDS; i++) {
		const u8 *entry = patch->data + MT6628_PATCH_HEADER_SIZE +
				  i * MT6628_PATCH_ENTRY_SIZE;

		if (time_after_eq(jiffies, end)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = mt6628_sdio_patch_xfer(func, patch, credits, sequence, buf, i, false, &value);
		if (!ret && value != get_unaligned_le32(entry + 4))
			ret = -EBADMSG;
		if (ret)
			goto out;
		checked++;
	}
	for (i = 0; i < MT6628_PATCH_WORDS; i++) {
		const u8 *entry = patch->data + MT6628_PATCH_HEADER_SIZE +
				  i * MT6628_PATCH_ENTRY_SIZE;

		if (time_after_eq(jiffies, end)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = mt6628_sdio_patch_xfer(func, patch, credits, sequence, buf, i, true, NULL);
		if (ret)
			goto out;
		acknowledged++;
		if (time_after_eq(jiffies, end)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = mt6628_sdio_patch_xfer(func, patch, credits, sequence, buf, i, false, &value);
		if (!ret && value != get_unaligned_le32(entry + 8))
			ret = -EBADMSG;
		if (ret)
			goto out;
		verified++;
	}
	if (time_after_eq(jiffies, end))
		ret = -ETIMEDOUT;
out:
	dev_dbg(&func->dev, "native patch result=%d checked=%u acknowledged=%u verified=%u\n",
		ret, checked, acknowledged, verified);
	return ret;
}

int mt6628_sdio_load_firmware(struct sdio_func *func, const struct firmware *fw,
			      const struct firmware *patch)
{
	u8 credits = 8, sequence = 0;
	u32 value, count, i;
	u8 *buf;
	int ret;

	ret = patch ? mt6628_patch_validate(fw, patch) : mt6628_fw_validate(fw);
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
		dev_err(&func->dev, "download WCIR %#x\n", value);
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
	if (patch) {
		ret = mt6628_sdio_apply_patch(func, patch, &credits, &sequence, buf);
		if (ret)
			goto out;
		dev_info(&func->dev, "native patch verified (%u words)\n",
			 (unsigned int)MT6628_PATCH_WORDS);
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

int mt6628_sdio_bootstrap(struct sdio_func *func)
{
	const struct firmware *fw;
	int ret, err;

	ret = request_firmware(&fw, "mediatek/WIFI_RAM_CODE_MT6628", &func->dev);
	if (ret)
		return ret;
	ret = mt6628_fw_validate(fw);
	if (ret)
		goto release;
	sdio_claim_host(func);
	ret = mmc_hw_reset(func->card);
	if (ret) {
		if (ret > 0)
			ret = -EAGAIN;
		goto unlock;
	}
	ret = sdio_enable_func(func);
	if (!ret)
		ret = sdio_set_block_size(func, 512);
	if (!ret)
		ret = mt6628_sdio_load_firmware(func, fw, NULL);
	if (ret) {
		err = sdio_disable_func(func);
		if (err)
			dev_warn(&func->dev, "function disable failed: %d\n", err);
		err = mmc_hw_reset(func->card);
		if (err)
			dev_warn(&func->dev, "recovery reset incomplete: %d\n", err);
	}
unlock:
	sdio_release_host(func);
release:
	release_firmware(fw);
	return ret;
}
