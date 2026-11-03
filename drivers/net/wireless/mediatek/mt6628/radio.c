// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/limits.h>
#include <linux/mmc/sdio_func.h>
#include <linux/rtnetlink.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>

#include "runtime.h"

struct mt6628_radio {
	struct mt6628_dev *dev;
	struct ieee80211_supported_band band;
	struct ieee80211_channel channels[14];
	struct ieee80211_rate rates[12];
	struct sk_buff_head rx;
	struct wiphy_work rx_work;
	/* Policy */
	struct mutex policy_mutex;
	char alpha2[2];
	int power_limit;
	unsigned int channel_mask;
	int regulatory_power;
	int policy_error;
	int configure_error;
	bool configured;
};

static void mt6628_radio_rx_work(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	struct sk_buff *skb;
	unsigned int budget;

	for (budget = 0; budget < 48; budget++) {
		struct cfg80211_inform_bss data = {};
		struct cfg80211_bss *bss;
		unsigned int channel;

		skb = skb_dequeue(&radio->rx);
		if (!skb)
			break;
		channel = (u8)skb->cb[0];

		mutex_lock(&radio->policy_mutex);
		if (radio->configured && (radio->channel_mask & BIT(channel - 1))) {
			data.chan = &radio->channels[channel - 1];
			data.signal = (((u8)skb->cb[1] >> 1) - 110) * 100;
			bss = cfg80211_inform_bss_frame_data(wiphy, &data,
							     (void *)skb->data,
							     skb->len, GFP_KERNEL);
			if (bss)
				cfg80211_put_bss(wiphy, bss);
		}
		mutex_unlock(&radio->policy_mutex);
		dev_kfree_skb(skb);
	}
	if (!skb_queue_empty(&radio->rx))
		wiphy_work_queue(wiphy, &radio->rx_work);
}

void mt6628_radio_rx(struct mt6628_dev *dev, const u8 *packet, size_t len)
{
	struct wiphy *wiphy = dev->wiphy;
	struct mt6628_radio *radio;
	struct sk_buff *skb;
	unsigned int offset;
	u16 fc;

	if (!wiphy || len < 12 || (get_unaligned_le16(packet + 2) & 3) != 3)
		return;
	offset = 12 + (packet[4] & 3);
	if (len < offset + 36 || !packet[10] || packet[10] > 14 || packet[9] > 220)
		return;
	fc = get_unaligned_le16(packet + offset);
	if ((fc & 0xf) || ((fc & 0xfc) != 0x80 && (fc & 0xfc) != 0x50) ||
	    !is_valid_ether_addr(packet + offset + 16))
		return;
	radio = wiphy_priv(wiphy);
	if (skb_queue_len(&radio->rx) >= 48)
		return;
	skb = alloc_skb(len - offset, GFP_KERNEL);
	if (!skb)
		return;
	skb_put_data(skb, packet + offset, len - offset);
	skb->cb[0] = packet[10];
	skb->cb[1] = packet[9];
	skb_queue_tail(&radio->rx, skb);
	wiphy_work_queue(wiphy, &radio->rx_work);
}

static bool mt6628_power_index(unsigned int index)
{
	return index < 22 && index != 2 && index != 3 && index != 7;
}

static int mt6628_radio_max_power(const struct mt6628_factory *factory)
{
	int power = -128;
	unsigned int i;

	for (i = 0; i < 22; i++)
		if (mt6628_power_index(i))
			power = max(power, (int)(s8)factory->tx_power[i]);
	return power >= 0 ? power / 2 : -DIV_ROUND_UP(-power, 2);
}

static void mt6628_radio_snapshot(struct mt6628_radio *radio)
{
	unsigned int mask = 0, i;
	int budget = 127;

	ASSERT_RTNL();
	radio->configured = false;
	for (i = 0; i < 14; i++) {
		struct ieee80211_channel *channel = &radio->channels[i];

		if (channel->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_20MHZ))
			continue;
		if (channel->max_power < -64) {
			radio->policy_error = -ERANGE;
			return;
		}
		mask |= BIT(i);
		budget = min(budget, min(channel->max_power, 63) * 2);
	}
	radio->channel_mask = mask;
	radio->regulatory_power = budget;
	radio->policy_error = mask ? 0 : -EACCES;
}

static int mt6628_radio_build(struct mt6628_radio *radio, const char alpha2[2],
			      int limit, u8 domain[56], u8 power[40])
{
	unsigned int mask = radio->channel_mask, groups = 0, i;
	int budget = radio->regulatory_power;

	if (limit < -6400)
		return -ERANGE;
	if (radio->policy_error)
		return radio->policy_error;
	if (limit != INT_MAX)
		budget = min(budget, limit >= 0 ? limit / 50 :
			     -DIV_ROUND_UP(-limit, 50));
	memset(domain, 0, 56);
	if (alpha2[0] >= 'A' && alpha2[0] <= 'Z' &&
	    alpha2[1] >= 'A' && alpha2[1] <= 'Z')
		put_unaligned_le16((u16)alpha2[0] << 8 | alpha2[1], domain);
	domain[52] = 1;
	domain[53] = 1;
	while (mask) {
		unsigned int first, span, count, best_count = 0, best_span = 1;
		u8 *subband;

		if (groups == 6)
			return -E2BIG;
		first = __ffs(mask);
		if (first == 13) {
			best_count = 1;
		} else {
			for (span = 1; span <= 8; span *= 2) {
				count = 0;
				while (first + count * span < 13 &&
				       mask & BIT(first + count * span))
					count++;
				if (count > best_count) {
					best_count = count;
					best_span = span;
				}
			}
		}
		subband = domain + 4 + groups++ * 8;
		subband[0] = first == 13 ? 82 : 81;
		subband[1] = 1;
		subband[2] = best_span;
		subband[3] = first + 1;
		subband[4] = best_count;
		for (i = 0; i < best_count; i++)
			mask &= ~BIT(first + i * best_span);
	}
	memcpy(power, radio->dev->factory.tx_power, 40);
	for (i = 0; i < 22; i++)
		if (mt6628_power_index(i))
			power[i] = min((int)(s8)power[i], budget);
	return 0;
}

static int mt6628_radio_update(struct mt6628_radio *radio,
			       const char alpha2[2], int limit)
{
	u8 domain[56], power[40];
	int ret;

	ret = mt6628_radio_build(radio, alpha2, limit, domain, power);
	if (ret) {
		radio->configure_error = ret;
		return ret;
	}
	ret = mt6628_sdio_set(radio->dev, 0x28, power, sizeof(power));
	if (!ret)
		ret = mt6628_sdio_set(radio->dev, 0x13, domain, sizeof(domain));
	if (ret) {
		int err;

		radio->configure_error = ret;
		radio->configured = false;
		mt6628_sdio_fault(radio->dev, ret);
		err = mt6628_sdio_abort(radio->dev);
		if (err)
			dev_warn(&radio->dev->func->dev,
				 "terminal reset failed: %d\n", err);
		return ret;
	}
	radio->alpha2[0] = alpha2[0];
	radio->alpha2[1] = alpha2[1];
	radio->power_limit = limit;
	radio->configured = true;
	radio->configure_error = 0;
	return 0;
}

static int mt6628_set_tx_power(struct wiphy *wiphy, struct wireless_dev *wdev,
			       int radio_idx, enum nl80211_tx_power_setting type,
			       int mbm)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	int limit, ret;

	if (radio_idx >= 0 || type == NL80211_TX_POWER_FIXED)
		return -EOPNOTSUPP;
	if (type == NL80211_TX_POWER_AUTOMATIC)
		limit = INT_MAX;
	else if (type == NL80211_TX_POWER_LIMITED)
		limit = mbm;
	else
		return -EINVAL;
	mutex_lock(&radio->policy_mutex);
	ret = mt6628_radio_update(radio, radio->alpha2, limit);
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static void mt6628_reg_notifier(struct wiphy *wiphy,
				struct regulatory_request *request)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	int ret;

	mutex_lock(&radio->policy_mutex);
	mt6628_radio_snapshot(radio);
	ret = mt6628_radio_update(radio, request ? request->alpha2 : radio->alpha2,
				  radio->power_limit);
	mutex_unlock(&radio->policy_mutex);
	if (ret)
		mt6628_sdio_fault(radio->dev, ret);
}

static const struct cfg80211_ops mt6628_cfg_ops = {
	.set_tx_power = mt6628_set_tx_power,
};

int mt6628_radio_register(struct mt6628_dev *dev)
{
	static const int bitrates[] = {
		10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540,
	};
	struct mt6628_radio *radio;
	struct wiphy *wiphy;
	unsigned int i;
	int ret, max_power;

	if (!dev->factory.set_tx_power || dev->factory.edge_enabled ||
	    (dev->factory.enable_5ghz && !dev->info.hw_5ghz_disabled))
		return -EOPNOTSUPP;
	wiphy = wiphy_new(&mt6628_cfg_ops, sizeof(*radio));
	if (!wiphy)
		return -ENOMEM;
	radio = wiphy_priv(wiphy);
	radio->dev = dev;
	skb_queue_head_init(&radio->rx);
	wiphy_work_init(&radio->rx_work, mt6628_radio_rx_work);
	mutex_init(&radio->policy_mutex);
	radio->policy_error = -EAGAIN;
	radio->configure_error = -EAGAIN;
	radio->alpha2[0] = '0';
	radio->alpha2[1] = '0';
	radio->power_limit = INT_MAX;
	max_power = mt6628_radio_max_power(&dev->factory);
	for (i = 0; i < ARRAY_SIZE(radio->channels); i++) {
		radio->channels[i].band = NL80211_BAND_2GHZ;
		radio->channels[i].hw_value = i + 1;
		radio->channels[i].center_freq =
			ieee80211_channel_to_frequency(i + 1, NL80211_BAND_2GHZ);
		radio->channels[i].max_power = max_power;
	}
	for (i = 0; i < ARRAY_SIZE(radio->rates); i++)
		radio->rates[i].bitrate = bitrates[i];
	radio->band.channels = radio->channels;
	radio->band.n_channels = ARRAY_SIZE(radio->channels);
	radio->band.bitrates = radio->rates;
	radio->band.n_bitrates = ARRAY_SIZE(radio->rates);
	wiphy->bands[NL80211_BAND_2GHZ] = &radio->band;
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->reg_notifier = mt6628_reg_notifier;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	ether_addr_copy(wiphy->perm_addr, dev->mac_addr);
	set_wiphy_dev(wiphy, &dev->func->dev);
	ret = wiphy_register(wiphy);
	if (ret)
		goto free;
	mutex_lock(&radio->policy_mutex);
	if (!radio->configured)
		ret = radio->configure_error;
	mutex_unlock(&radio->policy_mutex);
	if (ret) {
		wiphy_unregister(wiphy);
		goto free;
	}
	sdio_claim_host(dev->func);
	dev->wiphy = wiphy;
	sdio_release_host(dev->func);
	return 0;
free:
	wiphy_free(wiphy);
	return ret;
}

void mt6628_radio_unregister(struct mt6628_dev *dev)
{
	struct wiphy *wiphy;
	struct mt6628_radio *radio;

	sdio_claim_host(dev->func);
	wiphy = dev->wiphy;
	dev->wiphy = NULL;
	sdio_release_host(dev->func);
	if (!wiphy)
		return;
	radio = wiphy_priv(wiphy);
	wiphy_lock(wiphy);
	wiphy_work_cancel(wiphy, &radio->rx_work);
	skb_queue_purge(&radio->rx);
	wiphy_unlock(wiphy);
	wiphy_unregister(wiphy);
	wiphy_free(wiphy);
}
