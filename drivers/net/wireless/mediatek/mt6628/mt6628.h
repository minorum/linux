/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_NATIVE_H
#define __MT6628_NATIVE_H

#include <linux/firmware.h>
#include <linux/types.h>

struct sdio_func;

struct mt6628_fw_info {
	u16 product_id;
	u16 firmware_version;
	u16 peer_version;
	bool hw_5ghz_disabled;
	bool eeprom_used;
	bool efuse_valid;
	bool mac_valid;
	u8 permanent_addr[6];
};

struct mt6628_sdio_status {
	u32 isr;
	u8 tx[6];
	u16 rx_count[2];
	u16 rx_len[2][16];
	u32 mailbox[2];
};

extern const u8 mt6628_sdio_tx_max[6];

int mt6628_fw_validate(const struct firmware *fw);
int mt6628_init_download(u8 *buf, size_t capacity, const u8 *data,
			 size_t len, u32 address, u8 sequence);
int mt6628_init_result(const u8 *buf, size_t len, u8 sequence);
int mt6628_mcu_command(u8 *buf, size_t capacity, const u8 *data, size_t len,
		       u8 command, bool set, u8 sequence);
int mt6628_mcu_event(const u8 *buf, size_t len, u8 event, u8 sequence);
int mt6628_sdio_decode_status(const u8 *buf, size_t len,
			      struct mt6628_sdio_status *status);
int mt6628_sdio_return_credits(u8 credits[6], const u8 returned[6]);
int mt6628_sdio_load_firmware(struct sdio_func *func, const struct firmware *fw);
int mt6628_sdio_bootstrap(struct sdio_func *func);
int mt6628_sdio_get_info(struct sdio_func *func, struct mt6628_fw_info *info);

#endif
