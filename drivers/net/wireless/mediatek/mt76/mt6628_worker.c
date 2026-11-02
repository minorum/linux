// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mmc/sdio_func.h>

#include "mt76.h"
#include "mt6628_sdio.h"

int mt6628_sdio_step(struct mt76_dev *dev, struct mt6628_sdio_state *state)
{
	struct mt6628_sdio_status *status = &state->pending;
	unsigned int i;
	u8 raw[88];
	int ret, port, work = 0;

	if (atomic_read(&dev->bus_hung))
		return -EIO;
	if (test_bit(MT76_RESET, &dev->phy.state) ||
	    test_bit(MT76_MCU_RESET, &dev->phy.state))
		return -ESHUTDOWN;
	if (!status->rx_count[0] && !status->rx_count[1]) {
		sdio_claim_host(dev->sdio.func);
		ret = sdio_readsb(dev->sdio.func, raw, 0x10, sizeof(raw));
		sdio_release_host(dev->sdio.func);
		if (ret)
			goto fault;
		ret = mt6628_sdio_decode_status(raw, sizeof(raw), status);
		if (ret)
			goto fault;
		if (status->isr & (BIT(3) | BIT(31))) {
			ret = -EIO;
			goto fault;
		}
		ret = mt6628_sdio_return_credits(state->credits, status->tx);
		if (ret < 0)
			goto fault;
		work += ret;
		memset(status->tx, 0, sizeof(status->tx));
	}
	for (port = 1; port >= 0; port--) {
		ret = mt6628_sdio_rx_run(dev, port, status);
		if (ret == -ENOMEM || ret == -ENOSPC)
			continue;
		if (ret < 0)
			goto fault;
		work += ret;
	}
	ret = mt6628_sdio_tx_one(dev, dev->q_mcu[MT_MCUQ_WM], state->credits);
	if (ret < 0)
		goto fault;
	work += ret;
	for (i = 0; i <= MT_TXQ_PSD; i++) {
		if (!dev->phy.q_tx[i])
			continue;
		ret = mt6628_sdio_tx_one(dev, dev->phy.q_tx[i], state->credits);
		if (ret < 0)
			goto fault;
		work += ret;
	}
	if (status->rx_count[0] || status->rx_count[1])
		return -EAGAIN;
	return work;
fault:
	atomic_set(&dev->bus_hung, true);
	set_bit(MT76_RESET, &dev->phy.state);
	set_bit(MT76_MCU_RESET, &dev->phy.state);
	wake_up(&dev->mcu.wait);
	return ret;
}
