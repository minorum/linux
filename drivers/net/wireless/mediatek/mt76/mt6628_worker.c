// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mmc/sdio_func.h>

#include "mt76.h"
#include "mt6628_sdio.h"

static int mt6628_sdio_fault(struct mt76_dev *dev, int ret)
{
	atomic_set(&dev->bus_hung, true);
	set_bit(MT76_RESET, &dev->phy.state);
	set_bit(MT76_MCU_RESET, &dev->phy.state);
	wake_up(&dev->mcu.wait);
	return ret;
}

int mt6628_sdio_setup(struct mt76_dev *dev, struct mt6628_sdio_state *state)
{
	struct sdio_func *func = dev->sdio.func;
	u32 value;
	int ret;

	if (atomic_read(&dev->bus_hung))
		return -EIO;
	if (test_bit(MT76_RESET, &dev->phy.state) ||
	    test_bit(MT76_MCU_RESET, &dev->phy.state))
		return -ESHUTDOWN;
	if (state->ready)
		return -EALREADY;
	if (func->cur_blksize != 512 || !dev->sdio.intr_data)
		return -EINVAL;
	sdio_claim_host(func);
	sdio_writeb(func, 2, 0x04, &ret);
	if (ret)
		goto out;
	value = sdio_readl(func, 0x04, &ret);
	if (ret)
		goto out;
	if (!(value & BIT(8)) || value & BIT(0)) {
		ret = -EACCES;
		goto out;
	}
	value = sdio_readl(func, 0x00, &ret);
	if (ret)
		goto out;
	if ((value & 0xffff) != 0x6628 || !(value & BIT(21))) {
		ret = -ENODEV;
		goto out;
	}
	value = sdio_readl(func, 0x0c, &ret);
	if (ret)
		goto out;
	sdio_writel(func, value & ~0x100f6, 0x0c, &ret);
	if (ret)
		goto out;
	value = sdio_readl(func, 0x0c, &ret);
	if (ret)
		goto out;
	if (value & 0x100f6) {
		ret = -EIO;
		goto out;
	}
	sdio_writel(func, 0xffffff0f, 0x14, &ret);
	if (ret)
		goto out;
	value = sdio_readl(func, 0x14, &ret);
	if (!ret && value != 0xffffff0f)
		ret = -EIO;
out:
	sdio_release_host(func);
	if (ret)
		return mt6628_sdio_fault(dev, ret);
	memset(state, 0, sizeof(*state));
	memcpy(state->credits, mt6628_sdio_tx_max, sizeof(state->credits));
	state->ready = true;
	return 0;
}

int mt6628_sdio_step(struct mt76_dev *dev, struct mt6628_sdio_state *state)
{
	struct mt6628_sdio_status *status = &state->pending;
	unsigned int i;
	u8 *raw = dev->sdio.intr_data;
	int ret, port, work = 0;

	if (atomic_read(&dev->bus_hung))
		return -EIO;
	if (test_bit(MT76_RESET, &dev->phy.state) ||
	    test_bit(MT76_MCU_RESET, &dev->phy.state) || !state->ready)
		return -ESHUTDOWN;
	if (!status->rx_count[0] && !status->rx_count[1]) {
		sdio_claim_host(dev->sdio.func);
		ret = sdio_readsb(dev->sdio.func, raw, 0x10, 88);
		sdio_release_host(dev->sdio.func);
		if (ret)
			goto fault;
		ret = mt6628_sdio_decode_status(raw, 88, status);
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
	state->ready = false;
	return mt6628_sdio_fault(dev, ret);
}
