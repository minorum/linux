// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "boot.h"
#include "protocol.h"

#define SC2331_WLAN_NAME "sprd/sc2331-wlan.bin"
#define SC2331_WLAN_ADDRESS 0x00100000
#define SC2331_WLAN_SIZE 786432
#define SC2331_CHUNK_SIZE 131072
#define SC2331_CALIB_NAME "sprd/sc2331-calibration.bin"
#define SC2331_CALIB_SIZE 4096
#define SC2331_CALIB_OFFSET 616
#define SC2331_CALIB_REPLY_MIN 3008

static int sc2331_sdio_length(struct sdio_func *func, unsigned int channel)
{
	unsigned int address = (0x200 + channel) * 4;
	int error = 0, length = 0;
	unsigned int i;
	u8 byte;

	for (i = 0; i < 3; i++) {
		byte = sdio_f0_readb(func, address + i, &error);
		if (error)
			return error;
		length |= byte << (8 * i);
	}
	return length;
}

static int sc2331_sdio_wait_tx(struct sdio_func *func, size_t len)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(1000);
	int available;

	do {
		sdio_claim_host(func);
		available = sc2331_sdio_length(func, 3);
		sdio_release_host(func);
		if (available < 0)
			return available;
		if ((size_t)available >= len)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int sc2331_sdio_read(struct sdio_func *func, u8 *data, size_t minimum,
			    size_t capacity, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	int error, length;
	u8 ready;

	do {
		sdio_claim_host(func);
		ready = sdio_f0_readb(func, 0x841, &error);
		if (!error && (ready & BIT(4))) {
			length = sc2331_sdio_length(func, 12);
			if (length < 0)
				error = length;
			else if ((size_t)length < minimum || (size_t)length > capacity)
				error = -EMSGSIZE;
			else
				error = sdio_readsb(func, data, 12, length);
			sdio_release_host(func);
			if (error)
				return error;
			return length;
		}
		sdio_release_host(func);
		if (error)
			return error;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int sc2331_sdio_ack(struct sdio_func *func, u8 *data)
{
	int length = sc2331_sdio_read(func, data, 6, 6, 1000);

	if (length < 0)
		return length;
	dev_info(&func->dev, "SDIO download reply: %*ph\n", 6, data);
	return sc2331_decode_sdio_ack(data, length);
}

static int sc2331_sdio_calibrate(struct sdio_func *func, u8 *data)
{
	const struct firmware *firmware;
	u8 version, calibrated;
	int ret;

	ret = request_firmware(&firmware, SC2331_CALIB_NAME, &func->dev);
	if (ret)
		return ret;
	if (firmware->size != SC2331_CALIB_SIZE ||
	    firmware->data[SC2331_CALIB_OFFSET + 1] > 1) {
		ret = -EINVAL;
		goto release;
	}
	memcpy(data, firmware->data, SC2331_CALIB_SIZE);
	version = data[SC2331_CALIB_OFFSET];
	calibrated = data[SC2331_CALIB_OFFSET + 1];
	sdio_claim_host(func);
	ret = sdio_writesb(func, 0, data, SC2331_CALIB_SIZE);
	sdio_release_host(func);
	if (ret)
		goto release;
	if (!calibrated) {
		ret = sc2331_sdio_read(func, data, SC2331_CALIB_REPLY_MIN,
				       SC2331_CALIB_SIZE, 2000);
		if (ret < 0)
			goto release;
		dev_info(&func->dev, "WLAN calibration reply: %d bytes, version %u, calibrated %u\n",
			 ret, data[0], data[1]);
		if (data[0] != version || data[1] != 1) {
			ret = -EPROTO;
			goto release;
		}
	}
	ret = 0;
	dev_info(&func->dev, "SC2331 WLAN calibration applied\n");
release:
	release_firmware(firmware);
	return ret;
}

static int sc2331_sdio_command(struct sdio_func *func, u8 *data,
			       size_t capacity, u16 command,
			       const u8 *payload, size_t len)
{
	ssize_t encoded;
	size_t offset, chunk;
	int ret;

	encoded = sc2331_encode_sdio(data, capacity, command, payload, len);
	if (encoded < 0)
		return encoded;
	ret = sc2331_sdio_wait_tx(func, encoded);
	if (ret)
		return ret;
	sdio_claim_host(func);
	for (offset = 0; offset < (size_t)encoded; offset += chunk) {
		chunk = min_t(size_t, encoded - offset,
			      func->card->host->max_req_size);
		ret = sdio_writesb(func, 3, data + offset, chunk);
		if (ret)
			break;
	}
	sdio_release_host(func);
	return ret ? ret : sc2331_sdio_ack(func, data);
}

static int sc2331_sdio_probe(struct sdio_func *func,
			     const struct sdio_device_id *id)
{
	const struct firmware *firmware;
	struct sc2331_boot *boot;
	size_t offset, copied, capacity = SC2331_CHUNK_SIZE + 8;
	u8 payload[8], *data;
	int ret;

	if (func->num != 1)
		return -ENODEV;
	boot = sc2331_boot_get(&func->dev);
	if (IS_ERR(boot))
		return dev_err_probe(&func->dev, PTR_ERR(boot), "Missing UART bootstrap\n");
	ret = request_firmware(&firmware, SC2331_WLAN_NAME, &func->dev);
	if (ret)
		return ret;
	if (!firmware->size || firmware->size > SC2331_WLAN_SIZE) {
		ret = -EINVAL;
		goto release;
	}
	data = kmalloc(capacity, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto release;
	}
	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	if (!ret)
		ret = sdio_set_block_size(func, 512);
	sdio_release_host(func);
	if (ret)
		goto disable;
	put_unaligned_be32(SC2331_WLAN_ADDRESS, payload);
	put_unaligned_be32(SC2331_WLAN_SIZE, payload + 4);
	ret = sc2331_sdio_command(func, data, capacity, 1, payload, 8);
	if (ret)
		goto disable;
	for (offset = 0; offset < SC2331_WLAN_SIZE; offset += SC2331_CHUNK_SIZE) {
		memset(data + 8, 0xff, SC2331_CHUNK_SIZE);
		copied = offset < firmware->size ?
			min_t(size_t, firmware->size - offset, SC2331_CHUNK_SIZE) : 0;
		if (copied)
			memcpy(data + 8, firmware->data + offset, copied);
		ret = sc2331_sdio_command(func, data, capacity, 2, data + 8,
					  SC2331_CHUNK_SIZE);
		if (ret)
			goto disable;
		dev_info(&func->dev, "WLAN download: %zu/%u\n",
			 offset + SC2331_CHUNK_SIZE, SC2331_WLAN_SIZE);
	}
	ret = sc2331_sdio_command(func, data, capacity, 3, NULL, 0);
	if (ret)
		goto disable;
	sc2331_boot_arm(boot);
	ret = sc2331_sdio_command(func, data, capacity, 4, payload, 4);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 WLAN image executed\n");
	ret = sc2331_boot_wait_ready(boot);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 WLAN calibration ready\n");
	ret = sc2331_sdio_calibrate(func, data);
	if (ret)
		goto disable;
	ret = sc2331_boot_wake_device(boot);
	if (ret)
		goto disable;
	sdio_set_drvdata(func, boot);
	dev_info(&func->dev, "SC2331 WLAN runtime awake\n");
	goto free;
disable:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	dev_err(&func->dev, "SDIO download failed: %d\n", ret);
free:
	kfree(data);
release:
	release_firmware(firmware);
	return ret;
}

static void sc2331_sdio_remove(struct sdio_func *func)
{
	sc2331_boot_sleep_device(sdio_get_drvdata(func));
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
}

static const struct sdio_device_id sc2331_sdio_ids[] = {
	{ SDIO_DEVICE(0x0000, 0x2331) },
	{ }
};
MODULE_DEVICE_TABLE(sdio, sc2331_sdio_ids);

struct sdio_driver sc2331_sdio_driver = {
	.name = "sc2331",
	.id_table = sc2331_sdio_ids,
	.probe = sc2331_sdio_probe,
	.remove = sc2331_sdio_remove,
};
MODULE_FIRMWARE(SC2331_WLAN_NAME);
MODULE_FIRMWARE(SC2331_CALIB_NAME);
