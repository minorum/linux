/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_SDIO_H
#define __MT6628_SDIO_H

#include <linux/types.h>

struct mt76_dev;
struct mt76_queue;

struct mt6628_sdio_status {
	u32 isr;
	u8 tx[6];
	u16 rx_count[2];
	u16 rx_len[2][16];
	u32 mailbox[2];
};

struct mt6628_sdio_state {
	struct mt6628_sdio_status pending;
	u8 credits[6];
	bool ready;
};

extern const u8 mt6628_sdio_tx_max[6];

int mt6628_sdio_decode_status(const u8 *buf, size_t len,
			      struct mt6628_sdio_status *status);
int mt6628_sdio_return_credits(u8 credits[6], const u8 returned[6]);
int mt6628_sdio_tx_one(struct mt76_dev *dev, struct mt76_queue *q,
		       u8 credits[6]);
int mt6628_sdio_rx_run(struct mt76_dev *dev, unsigned int port,
		       struct mt6628_sdio_status *status);
int mt6628_sdio_step(struct mt76_dev *dev, struct mt6628_sdio_state *state);
int mt6628_sdio_setup(struct mt76_dev *dev, struct mt6628_sdio_state *state);

#endif
