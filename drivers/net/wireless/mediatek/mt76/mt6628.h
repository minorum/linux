/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_H
#define __MT6628_H

#include <linux/workqueue.h>

#include "mt76.h"
#include "mt6628_sdio.h"

struct mt6628_dev {
	struct mt76_dev mt76;
	struct mt6628_sdio_state sdio_state;
	struct delayed_work sdio_retry;
	bool stopping;
	bool irq_claimed;
};

int mt6628_sdio_start(struct mt6628_dev *dev);
int mt6628_sdio_stop(struct mt6628_dev *dev);

#endif
