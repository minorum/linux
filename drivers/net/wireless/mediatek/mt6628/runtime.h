/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6628_RUNTIME_H
#define __MT6628_RUNTIME_H

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>

#include "mt6628.h"

struct mt6628_dev {
	struct sdio_func *func;
	struct mt6628_fw_info info;
	struct mt6628_factory factory;
	struct wiphy *wiphy;
	/* Commands */
	struct mutex command_mutex;
	struct completion command_done;
	struct delayed_work io_work;
	struct sk_buff_head tx;
	struct sk_buff_head data_tx;
	struct net_device *data_netdev;
	u8 data_peer;
	u8 data_channel;
	bool data_enabled;
	atomic_t error;
	u8 credits[6];
	u8 mac_addr[6];
	u8 sequence;
	u8 *status_buf;
	u8 *rx_buf;
	u8 *tx_buf;
	bool initialized;
	bool irq_claimed;
};

int mt6628_sdio_start(struct mt6628_dev *dev);
int mt6628_hw_start(struct mt6628_dev *dev);
int mt6628_sdio_set(struct mt6628_dev *dev, u8 command,
		    const void *data, size_t len);
int mt6628_sdio_set_seq(struct mt6628_dev *dev, u8 command,
			const void *data, size_t len, u8 *sequence);
int mt6628_sdio_mgmt(struct mt6628_dev *dev, const void *frame, size_t len,
		     u8 peer, u8 token, unsigned long deadline);
int mt6628_sdio_stop(struct mt6628_dev *dev);
void mt6628_data_start(struct mt6628_dev *dev, struct net_device *netdev,
		       u8 peer, u8 channel);
void mt6628_data_stop(struct mt6628_dev *dev);
netdev_tx_t mt6628_data_xmit(struct mt6628_dev *dev, struct net_device *netdev,
			     struct sk_buff *skb);
int mt6628_sdio_abort(struct mt6628_dev *dev);
void mt6628_sdio_fault(struct mt6628_dev *dev, int error);
int mt6628_radio_register(struct mt6628_dev *dev);
void mt6628_radio_unregister(struct mt6628_dev *dev);
void mt6628_radio_rx(struct mt6628_dev *dev, const u8 *packet, size_t len);
void mt6628_radio_error(struct mt6628_dev *dev, int error);

#endif
