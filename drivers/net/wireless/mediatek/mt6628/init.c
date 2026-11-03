// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "mt6628.h"

struct mt6628_init_io {
	u8 status[88];
	u8 command[24];
	u8 rx[37888];
	u8 credits[6];
};

static int mt6628_init_setup(struct sdio_func *func)
{
	u32 value;
	int ret;

	sdio_writeb(func, 2, 0x04, &ret);
	if (ret)
		return ret;
	value = sdio_readl(func, 0x04, &ret);
	if (ret)
		return ret;
	if (!(value & BIT(8)) || value & BIT(0))
		return -EACCES;
	value = sdio_readl(func, 0x00, &ret);
	if (ret)
		return ret;
	if ((value & 0xffff) != 0x6628 || !(value & BIT(21))) {
		dev_err(&func->dev, "initialization WCIR %#x\n", value);
		return -ENODEV;
	}
	value = sdio_readl(func, 0x0c, &ret);
	if (ret)
		return ret;
	sdio_writel(func, value & ~0x100f6, 0x0c, &ret);
	if (ret)
		return ret;
	value = sdio_readl(func, 0x0c, &ret);
	if (ret)
		return ret;
	if (value & 0x100f6)
		return -EIO;
	sdio_writel(func, 0xffffff0f, 0x14, &ret);
	if (ret)
		return ret;
	value = sdio_readl(func, 0x14, &ret);
	if (ret)
		return ret;
	return value == 0xffffff0f ? 0 : -EIO;
}

static int mt6628_init_event(const u8 *packet, unsigned int count,
			     u8 command, u8 sequence, u8 *response)
{
	u8 event = command == 0x80 ? 2 : 9;
	unsigned int size = command == 0x80 ? 24 : 20;

	if (count < 8 || get_unaligned_le16(packet) != count ||
	    get_unaligned_le16(packet + 2) != 1)
		return -EPROTO;
	if (packet[5] != sequence)
		return 0;
	if (packet[4] == 1) {
		if (count != 12 || packet[8] != command)
			return -EPROTO;
		return packet[9] ? -EREMOTEIO : -EPROTO;
	}
	if (packet[4] != event || count != size)
		return -EPROTO;
	memcpy(response, packet, count);
	return 1;
}

static int mt6628_init_query(struct sdio_func *func, struct mt6628_init_io *io,
			     u8 command, u8 sequence, u8 *response)
{
	unsigned long end = jiffies + msecs_to_jiffies(1000);
	unsigned int size = command == 0x80 ? 24 : 20;
	int ret;

	if (!io->credits[4])
		return -ENOBUFS;
	memset(io->command, 0, sizeof(io->command));
	ret = mt6628_mcu_command(io->command, sizeof(io->command),
				 io->command + 8, size - 8, command, false,
				 sequence);
	if (ret < 0)
		return ret;
	ret = sdio_writesb(func, 0x2c, io->command, ret);
	if (ret)
		return ret;
	io->credits[4]--;
	for (;;) {
		struct mt6628_sdio_status status;
		bool found = false;
		int port;

		ret = sdio_readsb(func, io->status, 0x10, sizeof(io->status));
		if (ret)
			return ret;
		ret = mt6628_sdio_decode_status(io->status, sizeof(io->status),
						&status);
		if (ret)
			return ret;
		if (status.isr & (BIT(3) | BIT(31)))
			return -EIO;
		ret = mt6628_sdio_return_credits(io->credits, status.tx);
		if (ret < 0)
			return ret;
		for (port = 1; port >= 0; port--) {
			unsigned int i, len = 0, offset = 0;

			for (i = 0; i < status.rx_count[port]; i++)
				len += ALIGN(status.rx_len[port][i] + 4, 4);
			if (!len)
				continue;
			if (len >= 512)
				len = roundup(len, 512);
			if (len > sizeof(io->rx))
				return -EMSGSIZE;
			ret = sdio_readsb(func, io->rx, port ? 0x34 : 0x30, len);
			if (ret)
				return ret;
			for (i = 0; i < status.rx_count[port]; i++) {
				const u8 *packet = io->rx + offset;
				unsigned int count = status.rx_len[port][i];

				ret = mt6628_init_event(packet, count, command,
							sequence, response);
				if (ret < 0)
					return ret;
				if (ret) {
					if (found)
						return -EPROTO;
					found = true;
				}
				offset += ALIGN(count + 4, 4);
			}
		}
		if (found)
			return 0;
		if (time_after_eq(jiffies, end))
			return -ETIMEDOUT;
		usleep_range(1000, 2000);
	}
}

int mt6628_sdio_get_info(struct sdio_func *func, struct mt6628_fw_info *info,
			 u8 credits[6])
{
	struct mt6628_fw_info result = {};
	struct mt6628_init_io *io;
	u8 response[24];
	int ret;

	if (func->cur_blksize != 512)
		return -EINVAL;
	io = kzalloc_obj(*io, GFP_KERNEL);
	if (!io)
		return -ENOMEM;
	memcpy(io->credits, mt6628_sdio_tx_max, sizeof(io->credits));
	sdio_claim_host(func);
	ret = mt6628_init_setup(func);
	if (ret)
		goto out;
	ret = mt6628_init_query(func, io, 0x80, 1, response);
	if (ret)
		goto out;
	result.product_id = get_unaligned_le16(response + 8);
	if (result.product_id != 0x6628) {
		dev_err(&func->dev, "firmware product %#x\n", result.product_id);
		ret = -ENODEV;
		goto out;
	}
	result.firmware_version = get_unaligned_le16(response + 10);
	result.peer_version = get_unaligned_le16(response + 12);
	result.hw_5ghz_disabled = response[14];
	result.eeprom_used = response[15];
	result.efuse_valid = response[16];
	result.mac_valid = response[17];
	ret = mt6628_init_query(func, io, 0xc1, 2, response);
	if (ret)
		goto out;
	memcpy(result.permanent_addr, response + 8, sizeof(result.permanent_addr));
	*info = result;
	memcpy(credits, io->credits, sizeof(io->credits));
out:
	sdio_release_host(func);
	kfree(io);
	return ret;
}
