// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mmc/sdio_func.h>
#include <linux/unaligned.h>

#include "mt76.h"
#include "mt6628_sdio.h"

int mt6628_sdio_tx_one(struct mt76_dev *dev, struct mt76_queue *q,
		       u8 credits[6])
{
	struct mt76_sdio *sdio = &dev->sdio;
	struct mt76_queue_entry *entry;
	struct sk_buff *skb;
	unsigned int count, type, tc, wire;
	u8 header[4];
	int ret;

	if (atomic_read(&dev->bus_hung))
		return -EIO;
	if (test_bit(MT76_RESET, &dev->phy.state) ||
	    test_bit(MT76_MCU_RESET, &dev->phy.state))
		return -ESHUTDOWN;
	spin_lock_bh(&q->lock);
	if (q->first == q->head) {
		spin_unlock_bh(&q->lock);
		return 0;
	}
	entry = &q->entry[q->first];
	skb = entry->skb;
	spin_unlock_bh(&q->lock);
	if (!skb || skb->len < 4)
		return -EPROTO;
	ret = skb_copy_bits(skb, 0, header, sizeof(header));
	if (ret)
		return ret;
	count = get_unaligned_le16(header) & 0xfff;
	type = header[3] >> 6;
	tc = (header[3] >> 2) & 0xf;
	if (type == 2)
		return -EOPNOTSUPP;
	if (tc >= 6 || count < (type == 1 ? 8 : 16) ||
	    (skb->len != count && skb->len != ALIGN(count, 4)))
		return -EPROTO;
	if (q == dev->q_mcu[MT_MCUQ_WM] && (type != 1 || tc != 4))
		return -EPROTO;
	if (credits[tc] > mt6628_sdio_tx_max[tc])
		return -EPROTO;
	if (!credits[tc])
		return 0;
	wire = ALIGN(count, 4);
	if (wire >= 512)
		wire = roundup(wire, 512);
	if (sdio->func->cur_blksize != 512 || wire > sdio->xmit_buf_sz)
		return -EMSGSIZE;
	memset(sdio->xmit_buf, 0, wire);
	ret = skb_copy_bits(skb, 0, sdio->xmit_buf, count);
	if (ret)
		return ret;
	sdio_claim_host(sdio->func);
	ret = sdio_writesb(sdio->func, tc == 4 ? 0x2c : 0x28,
			   sdio->xmit_buf, wire);
	sdio_release_host(sdio->func);
	if (ret) {
		atomic_set(&dev->bus_hung, true);
		set_bit(MT76_RESET, &dev->phy.state);
		set_bit(MT76_MCU_RESET, &dev->phy.state);
		wake_up(&dev->mcu.wait);
		return ret;
	}
	credits[tc]--;
	spin_lock_bh(&q->lock);
	q->first = (q->first + 1) % q->ndesc;
	entry->done = true;
	spin_unlock_bh(&q->lock);
	mt76_worker_schedule(&sdio->status_worker);
	return 1;
}
