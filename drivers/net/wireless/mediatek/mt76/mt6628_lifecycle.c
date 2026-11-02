// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mmc/sdio_func.h>

#include "mt6628.h"

static void mt6628_sdio_halt(struct mt6628_dev *dev)
{
	WRITE_ONCE(dev->stopping, true);
	atomic_set(&dev->mt76.bus_hung, true);
	set_bit(MT76_RESET, &dev->mt76.phy.state);
	set_bit(MT76_MCU_RESET, &dev->mt76.phy.state);
	wake_up(&dev->mt76.mcu.wait);
}

static void mt6628_sdio_retry(struct work_struct *work)
{
	struct mt6628_dev *dev = container_of(to_delayed_work(work),
						     struct mt6628_dev, sdio_retry);

	if (!READ_ONCE(dev->stopping))
		mt76_worker_schedule(&dev->mt76.sdio.txrx_worker);
}

static void mt6628_sdio_worker(struct mt76_worker *worker)
{
	struct mt76_sdio *sdio = container_of(worker, struct mt76_sdio, txrx_worker);
	struct mt76_dev *mdev = container_of(sdio, struct mt76_dev, sdio);
	struct mt6628_dev *dev = container_of(mdev, struct mt6628_dev, mt76);
	unsigned int budget;
	int ret = 0;

	sdio_claim_host(sdio->func);
	if (!READ_ONCE(dev->stopping))
		sdio_writeb(sdio->func, 2, 0x04, &ret);
	sdio_release_host(sdio->func);
	if (ret)
		goto fault;
	for (budget = 0; budget < 64; budget++) {
		if (READ_ONCE(dev->stopping))
			return;
		ret = mt6628_sdio_step(mdev, &dev->sdio_state);
		if (ret == -EAGAIN)
			break;
		if (ret < 0)
			goto fault;
		if (!ret) {
			sdio_claim_host(sdio->func);
			if (!READ_ONCE(dev->stopping))
				sdio_writeb(sdio->func, 1, 0x04, &ret);
			sdio_release_host(sdio->func);
			if (ret)
				goto fault;
			return;
		}
	}
	if (!READ_ONCE(dev->stopping))
		queue_delayed_work(mdev->wq, &dev->sdio_retry, msecs_to_jiffies(10));
	return;
fault:
	if (!READ_ONCE(dev->stopping))
		dev_err(mdev->dev, "SDIO worker failed: %d\n", ret);
	mt6628_sdio_halt(dev);
}

static void mt6628_sdio_irq(struct sdio_func *func)
{
	struct mt6628_dev *dev = sdio_get_drvdata(func);
	int ret;

	if (READ_ONCE(dev->stopping))
		return;
	sdio_writeb(func, 2, 0x04, &ret);
	if (ret) {
		dev_err(dev->mt76.dev, "IRQ mask failed: %d\n", ret);
		mt6628_sdio_halt(dev);
		return;
	}
	mt76_worker_schedule(&dev->mt76.sdio.txrx_worker);
}

int mt6628_sdio_start(struct mt6628_dev *dev)
{
	struct mt76_dev *mdev = &dev->mt76;
	struct sdio_func *func = mdev->sdio.func;
	int ret, err;

	if (mdev->sdio.txrx_worker.task || dev->irq_claimed)
		return -EALREADY;
	if (func->irq_handler)
		return -EBUSY;
	WRITE_ONCE(dev->stopping, true);
	INIT_DELAYED_WORK(&dev->sdio_retry, mt6628_sdio_retry);
	ret = mt6628_sdio_setup(mdev, &dev->sdio_state);
	if (ret)
		return ret;
	ret = mt76_worker_setup(mdev->hw, &mdev->sdio.txrx_worker,
				mt6628_sdio_worker, "sdio-txrx");
	if (ret)
		goto fault;
	WRITE_ONCE(dev->stopping, false);
	sdio_claim_host(func);
	ret = sdio_claim_irq(func, mt6628_sdio_irq);
	if (!ret) {
		dev->irq_claimed = true;
	} else {
		err = sdio_release_irq(func);
		if (err)
			dev_warn(mdev->dev, "IRQ cleanup failed: %d\n", err);
	}
	sdio_release_host(func);
	if (ret) {
		mt6628_sdio_halt(dev);
		mt76_worker_disable(&mdev->sdio.txrx_worker);
		cancel_delayed_work_sync(&dev->sdio_retry);
		mt76_worker_teardown(&mdev->sdio.txrx_worker);
		goto fault;
	}
	mt76_worker_schedule(&mdev->sdio.txrx_worker);
	return 0;
fault:
	dev->sdio_state.ready = false;
	mt6628_sdio_halt(dev);
	return ret;
}

int mt6628_sdio_stop(struct mt6628_dev *dev)
{
	struct mt76_dev *mdev = &dev->mt76;
	struct sdio_func *func = mdev->sdio.func;
	int ret, err;

	if (!mdev->sdio.txrx_worker.task && !dev->irq_claimed)
		return 0;
	sdio_claim_host(func);
	mt6628_sdio_halt(dev);
	sdio_writeb(func, 2, 0x04, &ret);
	sdio_release_host(func);
	mt76_worker_disable(&mdev->tx_worker);
	mutex_lock(&mdev->mcu.mutex);
	mutex_unlock(&mdev->mcu.mutex);
	mt76_worker_disable(&mdev->sdio.txrx_worker);
	cancel_delayed_work_sync(&dev->sdio_retry);
	mt76_worker_teardown(&mdev->sdio.txrx_worker);
	sdio_claim_host(func);
	if (dev->irq_claimed) {
		err = sdio_release_irq(func);
		dev->irq_claimed = false;
		if (!ret)
			ret = err;
	}
	sdio_release_host(func);
	mt76_worker_disable(&mdev->sdio.status_worker);
	mt76_worker_disable(&mdev->sdio.net_worker);
	mt76_worker_disable(&mdev->sdio.stat_worker);
	mt76s_tx_drain(mdev);
	skb_queue_purge(&mdev->mcu.res_q);
	memset(&dev->sdio_state, 0, sizeof(dev->sdio_state));
	return ret;
}
