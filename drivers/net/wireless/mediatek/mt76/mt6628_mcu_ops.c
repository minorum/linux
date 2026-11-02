// SPDX-License-Identifier: GPL-2.0-only
#include <linux/unaligned.h>

#include "mt76.h"
#include "mt6628_mcu.h"

static int mt6628_mcu_send(struct mt76_dev *dev, struct sk_buff *skb,
			   int cmd, int *wait_sequence)
{
	struct mt76_queue *queue = dev->q_mcu[MT_MCUQ_WM];
	unsigned int len = skb->len;
	int ret;
	u8 sequence;

	if (cmd < 0 || cmd & ~0x1ff || skb_is_nonlinear(skb)) {
		ret = -EINVAL;
		goto free;
	}
	if (len > 4084) {
		ret = -EMSGSIZE;
		goto free;
	}
	if (wait_sequence && !(cmd & MT6628_MCU_SET) &&
	    cmd != 0xc1 && cmd != 0x80) {
		ret = -EOPNOTSUPP;
		goto free;
	}
	if (wait_sequence && !(cmd & MT6628_MCU_SET) &&
	    ((cmd == 0xc1 && len != 12) || (cmd == 0x80 && len != 16))) {
		ret = -EINVAL;
		goto free;
	}
	ret = skb_cow_head(skb, 8);
	if (ret)
		goto free;
	skb_push(skb, 8);
	ret = skb_put_padto(skb, ALIGN(skb->len, 4));
	if (ret)
		return ret;
	sequence = ++dev->mcu.msg_seq;
	ret = mt6628_mcu_command(skb->data, skb->len, skb->data + 8, len,
				 cmd, cmd & MT6628_MCU_SET, sequence);
	if (ret < 0)
		goto free;
	if (wait_sequence)
		*wait_sequence = sequence;
	ret = dev->queue_ops->tx_queue_skb_raw(dev, queue, skb, 0);
	if (!ret)
		dev->queue_ops->kick(dev, queue);
	return ret;
free:
	dev_kfree_skb(skb);
	return ret;
}

static int mt6628_mcu_response(struct mt76_dev *dev, int cmd,
			       struct sk_buff *skb, int sequence)
{
	u8 event;
	int ret;

	if (!skb)
		return -ETIMEDOUT;
	if (skb->len < 8 || get_unaligned_le16(skb->data) != skb->len ||
	    get_unaligned_le16(skb->data + 2) != 1)
		return -EPROTO;
	if (skb->data[5] != sequence)
		return -EAGAIN;
	if (cmd & MT6628_MCU_SET)
		event = 1;
	else if (cmd == 0xc1)
		event = 9;
	else if (cmd == 0x80)
		event = 2;
	else
		return -EOPNOTSUPP;
	if (skb->data[4] == 1) {
		if (skb->len != 12 || skb->data[8] != (u8)cmd)
			return -EPROTO;
		if (skb->data[9])
			return -EREMOTEIO;
	}
	ret = mt6628_mcu_event(skb->data, skb->len, event, sequence);
	if (ret)
		return ret;
	if ((event == 9 && skb->len != 20) ||
	    (event == 2 && skb->len != 24))
		return -EPROTO;
	skb_pull(skb, 8);
	return 0;
}

void mt6628_mcu_init(struct mt76_dev *dev)
{
	static const struct mt76_mcu_ops ops = {
		.headroom = 8,
		.tailroom = 3,
		.mcu_skb_send_msg = mt6628_mcu_send,
		.mcu_parse_response = mt6628_mcu_response,
	};

	dev->mcu.timeout = HZ;
	dev->mcu_ops = &ops;
}
