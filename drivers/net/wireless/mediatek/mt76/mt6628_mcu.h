/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_MCU_H
#define __MT6628_MCU_H

#include <linux/firmware.h>
#include <linux/types.h>

struct sdio_func;
struct mt76_dev;
struct sk_buff;

#define MT6628_MCU_SET 0x100

int mt6628_fw_validate(const struct firmware *fw);
int mt6628_init_download(u8 *buf, size_t capacity, const u8 *data,
			 size_t len, u32 address, u8 sequence);
int mt6628_init_result(const u8 *buf, size_t len, u8 sequence);
int mt6628_mcu_command(u8 *buf, size_t capacity, const u8 *data, size_t len,
		       u8 command, bool set, u8 sequence);
int mt6628_mcu_event(const u8 *buf, size_t len, u8 event, u8 sequence);
int mt6628_sdio_load_firmware(struct sdio_func *func, const struct firmware *fw);
int mt6628_sdio_bootstrap(struct sdio_func *func);
void mt6628_mcu_init(struct mt76_dev *dev);
bool mt6628_mcu_rx_event(struct mt76_dev *dev, struct sk_buff *skb);

#endif
