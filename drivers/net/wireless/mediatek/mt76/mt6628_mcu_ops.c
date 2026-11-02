// SPDX-License-Identifier: GPL-2.0-only
#include <linux/etherdevice.h>
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

	if (!skb) {
		if (atomic_read(&dev->bus_hung))
			return -EIO;
		if (test_bit(MT76_MCU_RESET, &dev->phy.state))
			return -ESHUTDOWN;
		return -ETIMEDOUT;
	}
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

bool mt6628_mcu_rx_event(struct mt76_dev *dev, struct sk_buff *skb)
{
	if (skb_is_nonlinear(skb) || skb->len < 8 ||
	    get_unaligned_le16(skb->data) != skb->len ||
	    get_unaligned_le16(skb->data + 2) != 1)
		return false;
	switch (skb->data[4]) {
	case 1:
	case 2:
	case 9:
		mt76_mcu_rx_event(dev, skb);
		return true;
	default:
		return false;
	}
}

int mt6628_mcu_get_info(struct mt76_dev *dev, struct mt6628_fw_info *info)
{
	struct mt6628_fw_info result = {};
	struct sk_buff *skb;
	u8 query[16] = {};
	int ret;

	ret = mt76_mcu_send_and_get_msg(dev, 0x80, query, sizeof(query), true, &skb);
	if (ret)
		return ret;
	result.product_id = get_unaligned_le16(skb->data);
	result.firmware_version = get_unaligned_le16(skb->data + 2);
	result.peer_version = get_unaligned_le16(skb->data + 4);
	result.hw_5ghz_disabled = skb->data[6];
	result.eeprom_used = skb->data[7];
	result.efuse_valid = skb->data[8];
	result.mac_valid = skb->data[9];
	dev_kfree_skb(skb);
	ret = mt76_mcu_send_and_get_msg(dev, 0xc1, query, 12, true, &skb);
	if (ret)
		return ret;
	if (!is_valid_ether_addr(skb->data)) {
		dev_kfree_skb(skb);
		return -ENODATA;
	}
	memcpy(result.permanent_addr, skb->data, sizeof(result.permanent_addr));
	dev_kfree_skb(skb);
	*info = result;
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
