/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_MCU_H
#define __MT6628_MCU_H

#include <linux/firmware.h>
#include <linux/types.h>

int mt6628_fw_validate(const struct firmware *fw);
int mt6628_init_download(u8 *buf, size_t capacity, const u8 *data,
			 size_t len, u32 address, u8 sequence);
int mt6628_init_result(const u8 *buf, size_t len, u8 sequence);

#endif
