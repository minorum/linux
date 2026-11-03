// SPDX-License-Identifier: GPL-2.0-only
#include <linux/jiffies.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/mmc/core.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "runtime.h"

struct mt6628_tx_info {
	unsigned long deadline;
	bool management;
	u8 tc;
};

#define MT6628_DATA_QUEUE_LEN 64
#define MT6628_DATA_LIFETIME_MS 1000

void mt6628_data_start(struct mt6628_dev *dev, struct net_device *netdev,
		       u8 peer, u8 channel)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->data_tx.lock, flags);
	if (!atomic_read(&dev->error)) {
		dev->data_netdev = netdev;
		dev->data_peer = peer;
		dev->data_channel = channel;
		dev->data_enabled = true;
		netif_carrier_on(netdev);
		netif_wake_queue(netdev);
	}
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
}

void mt6628_data_stop(struct mt6628_dev *dev)
{
	struct net_device *netdev;
	struct sk_buff *skb;
	unsigned long flags;

	if (!READ_ONCE(dev->initialized))
		return;
	spin_lock_irqsave(&dev->data_tx.lock, flags);
	dev->data_enabled = false;
	netdev = dev->data_netdev;
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
	if (netdev) {
		netif_carrier_off(netdev);
		netif_tx_disable(netdev);
	}
	sdio_claim_host(dev->func);
	while ((skb = skb_dequeue(&dev->data_tx))) {
		dev_core_stats_tx_dropped_inc(netdev);
		dev_kfree_skb_any(skb);
	}
	spin_lock_irqsave(&dev->data_tx.lock, flags);
	dev->data_netdev = NULL;
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
	sdio_release_host(dev->func);
}

netdev_tx_t mt6628_data_xmit(struct mt6628_dev *dev, struct net_device *netdev,
			     struct sk_buff *skb)
{
	struct mt6628_tx_info *info = (void *)skb->cb;
	struct ethhdr header, *eth;
	unsigned long flags;
	__be16 protocol;

	spin_lock_irqsave(&dev->data_tx.lock, flags);
	if (!dev->data_enabled || dev->data_netdev != netdev || atomic_read(&dev->error))
		goto drop;
	if (skb_queue_len(&dev->data_tx) >= MT6628_DATA_QUEUE_LEN) {
		netif_stop_queue(dev->data_netdev);
		spin_unlock_irqrestore(&dev->data_tx.lock, flags);
		return NETDEV_TX_BUSY;
	}
	eth = skb_header_pointer(skb, 0, sizeof(header), &header);
	if (!eth || skb->len > VLAN_ETH_FRAME_LEN || skb_is_gso(skb) ||
	    skb_vlan_tag_present(skb) || skb->ip_summed == CHECKSUM_PARTIAL)
		goto drop;
	protocol = __vlan_get_protocol(skb, eth->h_proto, NULL);
	if (!protocol || protocol == htons(ETH_P_PAE) || protocol == htons(ETH_P_PREAUTH))
		goto drop;
	info->tc = is_multicast_ether_addr(eth->h_dest) ? 5 : 1;
	info->deadline = jiffies + msecs_to_jiffies(MT6628_DATA_LIFETIME_MS);
	__skb_queue_tail(&dev->data_tx, skb);
	if (skb_queue_len(&dev->data_tx) == MT6628_DATA_QUEUE_LEN)
		netif_stop_queue(dev->data_netdev);
	mod_delayed_work(system_wq, &dev->io_work, 0);
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
	return NETDEV_TX_OK;
drop:
	dev_core_stats_tx_dropped_inc(netdev);
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void mt6628_data_rx(struct mt6628_dev *dev, const u8 *packet,
			   unsigned int count)
{
	unsigned int offset = 12 + (packet[4] & 3), len = count - offset;
	struct net_device *netdev;
	struct sk_buff *skb;
	unsigned long flags;
	__be16 protocol;

	spin_lock_irqsave(&dev->data_tx.lock, flags);
	netdev = dev->data_netdev;
	if (!dev->data_enabled || atomic_read(&dev->error) ||
	    (packet[5] & 0xf7) || (get_unaligned_le16(packet + 6) & BIT(15)) ||
	    packet[8] != dev->data_peer || packet[10] != dev->data_channel ||
	    (packet[4] >> 2) != ETH_HLEN || len > VLAN_ETH_FRAME_LEN ||
	    (!is_multicast_ether_addr(packet + offset) &&
	     !ether_addr_equal(packet + offset, netdev->dev_addr))) {
		spin_unlock_irqrestore(&dev->data_tx.lock, flags);
		return;
	}
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
	skb = netdev_alloc_skb_ip_align(netdev, len);
	if (!skb) {
		dev_core_stats_rx_dropped_inc(netdev);
		return;
	}
	skb_put_data(skb, packet + offset, len);
	skb->protocol = ((struct ethhdr *)skb->data)->h_proto;
	protocol = vlan_get_protocol(skb);
	if (!protocol || protocol == htons(ETH_P_PAE) || protocol == htons(ETH_P_PREAUTH)) {
		dev_core_stats_rx_dropped_inc(netdev);
		dev_kfree_skb_any(skb);
		return;
	}
	skb->protocol = eth_type_trans(skb, netdev);
	skb->ip_summed = CHECKSUM_NONE;
	local_bh_disable();
	dev_sw_netstats_rx_add(netdev, len);
	local_bh_enable();
	netif_rx(skb);
}

static int mt6628_data_tx(struct mt6628_dev *dev)
{
	struct sk_buff *skb, *next, *selected = NULL;
	struct net_device *netdev;
	struct mt6628_tx_info *info;
	unsigned long flags;
	unsigned int wire;
	bool more = false;
	u8 tc;
	int ret, work = 0;

	spin_lock_irqsave(&dev->data_tx.lock, flags);
	netdev = dev->data_netdev;
	if (!dev->data_enabled || atomic_read(&dev->error))
		goto unlock;
	skb_queue_walk_safe(&dev->data_tx, skb, next) {
		info = (void *)skb->cb;
		if (!time_before(jiffies, info->deadline)) {
			__skb_unlink(skb, &dev->data_tx);
			dev_core_stats_tx_dropped_inc(netdev);
			dev_kfree_skb_any(skb);
			work++;
		} else if (!selected && dev->credits[info->tc]) {
			selected = skb;
		}
	}
	if (selected) {
		__skb_unlink(selected, &dev->data_tx);
		skb_queue_walk(&dev->data_tx, skb)
			if (((struct mt6628_tx_info *)skb->cb)->tc == 1)
				more = true;
	}
	if (skb_queue_len(&dev->data_tx) < MT6628_DATA_QUEUE_LEN)
		netif_wake_queue(netdev);
	if (!skb_queue_empty(&dev->data_tx))
		queue_delayed_work(system_wq, &dev->io_work, msecs_to_jiffies(100));
unlock:
	spin_unlock_irqrestore(&dev->data_tx.lock, flags);
	if (!selected)
		return work;
	tc = ((struct mt6628_tx_info *)selected->cb)->tc;
	wire = ALIGN(16 + selected->len, 4);
	if (wire >= 512)
		wire = roundup(wire, 512);
	memset(dev->tx_buf, 0, wire);
	put_unaligned_le16(16 + selected->len, dev->tx_buf);
	dev->tx_buf[2] = ETH_HLEN;
	dev->tx_buf[3] = tc << 2;
	dev->tx_buf[4] = ETH_HLEN;
	dev->tx_buf[10] = tc == 5 ? 0xff : dev->data_peer;
	dev->tx_buf[11] = 0x20 | (tc == 1 && more ? 3 : 0);
	ret = skb_copy_bits(selected, 0, dev->tx_buf + 16, selected->len);
	if (ret) {
		dev_core_stats_tx_dropped_inc(netdev);
		dev_kfree_skb_any(selected);
		return work + 1;
	}
	ret = sdio_writesb(dev->func, 0x28, dev->tx_buf, wire);
	if (ret) {
		dev_core_stats_tx_dropped_inc(netdev);
	} else {
		dev->credits[tc]--;
		local_bh_disable();
		dev_sw_netstats_tx_add(netdev, 1, selected->len);
		local_bh_enable();
	}
	dev_kfree_skb_any(selected);
	return ret ?: work + 1;
}

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
		else if (!ret)
			mt6628_data_rx(dev, packet, count);
		offset += ALIGN(count + 4, 4);
	}
	return status->rx_count[port];
}

static int mt6628_runtime_step(struct mt6628_dev *dev)
{
	struct mt6628_sdio_status status;
	struct mt6628_tx_info *info;
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
	if (atomic_read(&dev->error))
		return work;
	skb = dev->credits[4] ? skb_dequeue(&dev->tx) : NULL;
	if (!skb) {
		ret = mt6628_data_tx(dev);
		return ret < 0 ? ret : work + ret;
	}
	info = (void *)skb->cb;
	if (info->management && !time_before(jiffies, info->deadline)) {
		dev_kfree_skb(skb);
		return -ETIMEDOUT;
	}
	wire = skb->len >= 512 ? roundup(skb->len, 512) : skb->len;
	memset(dev->tx_buf, 0, wire);
	memcpy(dev->tx_buf, skb->data, skb->len);
	ret = sdio_writesb(dev->func, 0x2c, dev->tx_buf, wire);
	dev_kfree_skb(skb);
	if (ret)
		return ret;
	dev->credits[4]--;
	complete(&dev->command_done);
	ret = mt6628_data_tx(dev);
	return ret < 0 ? ret : work + 1 + ret;
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
	queue_delayed_work(system_wq, &dev->io_work, msecs_to_jiffies(10));
out:
	sdio_release_host(dev->func);
	if (ret < 0)
		mt6628_sdio_fault(dev, ret);
	ret = atomic_read(&dev->error);
	if (ret && ret != -ESHUTDOWN) {
		sdio_claim_host(dev->func);
		mt6628_radio_error(dev, ret);
		sdio_release_host(dev->func);
	}
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
	skb_queue_head_init(&dev->data_tx);
	dev->data_enabled = false;
	dev->data_netdev = NULL;
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

static int mt6628_sdio_submit(struct mt6628_dev *dev, struct sk_buff *skb)
{
	int ret;

	reinit_completion(&dev->command_done);
	ret = atomic_read(&dev->error);
	if (ret) {
		dev_kfree_skb(skb);
		return ret;
	}
	skb_queue_tail(&dev->tx, skb);
	mod_delayed_work(system_wq, &dev->io_work, 0);
	if (!wait_for_completion_timeout(&dev->command_done,
					 msecs_to_jiffies(1000)))
		mt6628_sdio_fault(dev, -ETIMEDOUT);
	return atomic_read(&dev->error);
}

int mt6628_sdio_set_seq(struct mt6628_dev *dev, u8 command,
			const void *data, size_t len, u8 *sequence)
{
	struct sk_buff *skb;
	int ret;

	if (!READ_ONCE(dev->initialized))
		return -ESHUTDOWN;
	if (len > 4084)
		return -EMSGSIZE;
	mutex_lock(&dev->command_mutex);
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
	if (sequence)
		*sequence = dev->sequence;
	((struct mt6628_tx_info *)skb->cb)->management = false;
	ret = mt6628_sdio_submit(dev, skb);
unlock:
	mutex_unlock(&dev->command_mutex);
	return ret;
}

int mt6628_sdio_set(struct mt6628_dev *dev, u8 command,
		    const void *data, size_t len)
{
	return mt6628_sdio_set_seq(dev, command, data, len, NULL);
}

int mt6628_sdio_mgmt(struct mt6628_dev *dev, const void *frame, size_t len,
		     u8 peer, u8 token, unsigned long deadline)
{
	struct sk_buff *skb;
	struct mt6628_tx_info *info;
	u8 *packet;
	int ret;

	if (!READ_ONCE(dev->initialized))
		return -ESHUTDOWN;
	if (len > 1536)
		return -EMSGSIZE;
	if (len < 24 || !frame || peer >= 20 || (get_unaligned_le16(frame) & 0xf))
		return -EINVAL;
	mutex_lock(&dev->command_mutex);
	ret = atomic_read(&dev->error);
	if (ret)
		goto unlock;
	if (!time_before(jiffies, deadline)) {
		ret = -ETIMEDOUT;
		goto unlock;
	}
	skb = alloc_skb(ALIGN(16 + len, 4), GFP_KERNEL);
	if (!skb) {
		ret = -ENOMEM;
		goto unlock;
	}
	packet = skb_put(skb, ALIGN(16 + len, 4));
	memset(packet, 0, skb->len);
	put_unaligned_le16(16 + len, packet);
	packet[2] = 20;
	packet[3] = 0xd0;
	packet[4] = 24;
	packet[5] = 0x80;
	packet[10] = peer;
	packet[12] = token;
	packet[13] = 5;
	memcpy(packet + 16, frame, len);
	info = (void *)skb->cb;
	info->management = true;
	info->deadline = deadline;
	ret = mt6628_sdio_submit(dev, skb);
unlock:
	mutex_unlock(&dev->command_mutex);
	return ret;
}

int mt6628_sdio_stop(struct mt6628_dev *dev)
{
	int ret, err;

	if (!READ_ONCE(dev->initialized))
		return 0;
	mt6628_data_stop(dev);
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
