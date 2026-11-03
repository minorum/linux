// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mmc/core.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "runtime.h"

void mt6628_sdio_fault(struct mt6628_dev *dev, int error)
{
	if (!atomic_cmpxchg(&dev->error, 0, error))
		mod_delayed_work(system_wq, &dev->io_work, 0);
	complete_all(&dev->command_done);
}

static int mt6628_runtime_packet(const u8 *packet, unsigned int count)
{
	unsigned int type, offset;

	if (count < 4 || get_unaligned_le16(packet) != count)
		return -EPROTO;
	type = get_unaligned_le16(packet + 2) & 3;
	if (type == 1)
		return count >= 8 ? 0 : -EPROTO;
	if (type == 2)
		return -EOPNOTSUPP;
	if (count < 12)
		return -EPROTO;
	offset = 12 + (packet[4] & 3);
	if (count < offset)
		return -EPROTO;
	if (!type)
		return count - offset >= 14 ? 0 : -EINVAL;
	if (count - offset < 24 || (packet[4] >> 2) > count - offset ||
	    (get_unaligned_le16(packet + offset) & 0xf))
		return -EINVAL;
	return 0;
}

static int mt6628_runtime_rx(struct mt6628_dev *dev, unsigned int port,
			     const struct mt6628_sdio_status *status)
{
	unsigned int i, len = 0, offset = 0;
	int ret;

	for (i = 0; i < status->rx_count[port]; i++)
		len += ALIGN(status->rx_len[port][i] + 4, 4);
	if (!len)
		return 0;
	if (len >= 512)
		len = roundup(len, 512);
	if (len > 37888)
		return -EMSGSIZE;
	ret = sdio_readsb(dev->func, dev->rx_buf, port ? 0x34 : 0x30, len);
	if (ret)
		return ret;
	for (i = 0; i < status->rx_count[port]; i++) {
		const u8 *packet = dev->rx_buf + offset;
		unsigned int count = status->rx_len[port][i];

		ret = mt6628_runtime_packet(packet, count);
		if (ret && ret != -EINVAL)
			return ret;
		if (!ret && (get_unaligned_le16(packet + 2) & 3))
			mt6628_radio_rx(dev, packet, count);
		offset += ALIGN(count + 4, 4);
	}
	return status->rx_count[port];
}

static int mt6628_runtime_step(struct mt6628_dev *dev)
{
	struct mt6628_sdio_status status;
	struct sk_buff *skb;
	unsigned int wire;
	int ret, work, port;

	ret = sdio_readsb(dev->func, dev->status_buf, 0x10, 88);
	if (ret)
		return ret;
	ret = mt6628_sdio_decode_status(dev->status_buf, 88, &status);
	if (ret)
		return ret;
	if (status.isr & (BIT(3) | BIT(31)))
		return -EIO;
	work = mt6628_sdio_return_credits(dev->credits, status.tx);
	if (work < 0)
		return work;
	for (port = 1; port >= 0; port--) {
		ret = mt6628_runtime_rx(dev, port, &status);
		if (ret < 0)
			return ret;
		work += ret;
	}
	if (!dev->credits[4] || atomic_read(&dev->error))
		return work;
	skb = skb_dequeue(&dev->tx);
	if (!skb)
		return work;
	wire = skb->len >= 512 ? roundup(skb->len, 512) : skb->len;
	memset(dev->tx_buf, 0, wire);
	memcpy(dev->tx_buf, skb->data, skb->len);
	ret = sdio_writesb(dev->func, 0x2c, dev->tx_buf, wire);
	dev_kfree_skb(skb);
	if (ret)
		return ret;
	dev->credits[4]--;
	complete(&dev->command_done);
	return work + 1;
}

static void mt6628_runtime_work(struct work_struct *work)
{
	struct mt6628_dev *dev = container_of(to_delayed_work(work),
						     struct mt6628_dev, io_work);
	unsigned int budget;
	int ret = 0;

	sdio_claim_host(dev->func);
	sdio_writeb(dev->func, 2, 0x04, &ret);
	if (atomic_read(&dev->error)) {
		if (dev->irq_claimed) {
			int err = sdio_release_irq(dev->func);

			dev->irq_claimed = false;
			if (!ret)
				ret = err;
		}
		goto out;
	}
	if (ret)
		goto out;
	for (budget = 0; budget < 64; budget++) {
		if (atomic_read(&dev->error))
			goto out;
		ret = mt6628_runtime_step(dev);
		if (ret < 0)
			goto out;
		if (!ret) {
			sdio_writeb(dev->func, 1, 0x04, &ret);
			goto out;
		}
	}
	mod_delayed_work(system_wq, &dev->io_work, msecs_to_jiffies(10));
out:
	sdio_release_host(dev->func);
	if (ret < 0)
		mt6628_sdio_fault(dev, ret);
}

static void mt6628_runtime_irq(struct sdio_func *func)
{
	struct mt6628_dev *dev = sdio_get_drvdata(func);
	int ret;

	sdio_writeb(func, 2, 0x04, &ret);
	if (ret) {
		mt6628_sdio_fault(dev, ret);
		return;
	}
	if (!atomic_read(&dev->error))
		mod_delayed_work(system_wq, &dev->io_work, 0);
}

int mt6628_sdio_start(struct mt6628_dev *dev)
{
	int ret;

	if (READ_ONCE(dev->initialized))
		return -EALREADY;
	mutex_init(&dev->command_mutex);
	init_completion(&dev->command_done);
	skb_queue_head_init(&dev->tx);
	INIT_DELAYED_WORK(&dev->io_work, mt6628_runtime_work);
	atomic_set(&dev->error, 0);
	WRITE_ONCE(dev->initialized, true);
	dev->sequence = 2;
	if (!dev->status_buf)
		dev->status_buf = devm_kmalloc(&dev->func->dev, 88, GFP_KERNEL);
	if (!dev->rx_buf)
		dev->rx_buf = devm_kmalloc(&dev->func->dev, 37888, GFP_KERNEL);
	if (!dev->tx_buf)
		dev->tx_buf = devm_kmalloc(&dev->func->dev, 4096, GFP_KERNEL);
	if (!dev->status_buf || !dev->rx_buf || !dev->tx_buf) {
		mt6628_sdio_fault(dev, -ENOMEM);
		return -ENOMEM;
	}
	sdio_claim_host(dev->func);
	if (dev->func->irq_handler) {
		ret = -EBUSY;
	} else {
		dev->irq_claimed = true;
		ret = sdio_claim_irq(dev->func, mt6628_runtime_irq);
	}
	sdio_release_host(dev->func);
	if (!ret)
		mod_delayed_work(system_wq, &dev->io_work, 0);
	else
		mt6628_sdio_fault(dev, ret);
	return ret;
}

int mt6628_sdio_set(struct mt6628_dev *dev, u8 command,
		    const void *data, size_t len)
{
	struct sk_buff *skb;
	int ret;

	if (!READ_ONCE(dev->initialized))
		return -ESHUTDOWN;
	if (len > 4084)
		return -EMSGSIZE;
	mutex_lock(&dev->command_mutex);
	reinit_completion(&dev->command_done);
	ret = atomic_read(&dev->error);
	if (ret)
		goto unlock;
	skb = alloc_skb(ALIGN(8 + len, 4), GFP_KERNEL);
	if (!skb) {
		ret = -ENOMEM;
		goto unlock;
	}
	ret = mt6628_mcu_command(skb_put(skb, ALIGN(8 + len, 4)),
				 ALIGN(8 + len, 4), data, len, command, true,
				 ++dev->sequence);
	if (ret < 0) {
		dev_kfree_skb(skb);
		goto unlock;
	}
	ret = atomic_read(&dev->error);
	if (ret) {
		dev_kfree_skb(skb);
		goto unlock;
	}
	skb_queue_tail(&dev->tx, skb);
	mod_delayed_work(system_wq, &dev->io_work, 0);
	if (!wait_for_completion_timeout(&dev->command_done,
					 msecs_to_jiffies(1000)))
		mt6628_sdio_fault(dev, -ETIMEDOUT);
	ret = atomic_read(&dev->error);
unlock:
	mutex_unlock(&dev->command_mutex);
	return ret;
}

int mt6628_sdio_stop(struct mt6628_dev *dev)
{
	int ret, err;

	if (!READ_ONCE(dev->initialized))
		return 0;
	mt6628_sdio_fault(dev, -ESHUTDOWN);
	sdio_claim_host(dev->func);
	sdio_writeb(dev->func, 2, 0x04, &ret);
	if (dev->irq_claimed) {
		err = sdio_release_irq(dev->func);
		dev->irq_claimed = false;
		if (!ret)
			ret = err;
	}
	sdio_release_host(dev->func);
	mutex_lock(&dev->command_mutex);
	cancel_delayed_work_sync(&dev->io_work);
	skb_queue_purge(&dev->tx);
	mutex_unlock(&dev->command_mutex);
	WRITE_ONCE(dev->initialized, false);
	return ret;
}

int mt6628_sdio_abort(struct mt6628_dev *dev)
{
	int ret, err;

	if (!READ_ONCE(dev->initialized))
		return atomic_read(&dev->error) ?: -ESHUTDOWN;
	ret = mt6628_sdio_stop(dev);
	sdio_claim_host(dev->func);
	err = mmc_hw_reset(dev->func->card);
	sdio_release_host(dev->func);
	if (err > 0)
		err = -EAGAIN;
	return err ?: ret;
}
