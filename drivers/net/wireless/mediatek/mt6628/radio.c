// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/limits.h>
#include <linux/jiffies.h>
#include <linux/mmc/sdio_func.h>
#include <linux/rtnetlink.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>

#include "runtime.h"

enum mt6628_join_state {
	MT6628_JOIN_IDLE,
	MT6628_JOIN_CHANNEL,
	MT6628_JOIN_AUTH,
	MT6628_JOIN_AUTHENTICATED,
	MT6628_JOIN_ASSOC,
	MT6628_JOIN_ACTIVATE,
	MT6628_JOIN_CONNECTED,
};

struct mt6628_bss {
	u8 rcpi;
	bool valid;
};

struct mt6628_radio {
	struct mt6628_dev *dev;
	struct ieee80211_supported_band band;
	struct ieee80211_channel channels[14];
	struct ieee80211_rate rates[12];
	struct sk_buff_head rx;
	struct wiphy_work rx_work;
	struct net_device *netdev;
	struct cfg80211_scan_request *scan_request;
	struct delayed_work scan_timeout;
	unsigned long scan_deadline;
	atomic_t rx_error;
	unsigned int passive_mask;
	unsigned int scan_passive;
	u16 scan_token;
	u8 scan_sequence;
	/* Policy */
	struct mutex policy_mutex;
	char alpha2[2];
	int power_limit;
	unsigned int channel_mask;
	int regulatory_power;
	int policy_error;
	int configure_error;
	bool configured;
	bool ready;
	bool active;
	bool scan_uncertain;
	/* Join */
	enum mt6628_join_state join_state;
	struct cfg80211_bss *auth_bss;
	struct cfg80211_bss *assoc_bss;
	struct delayed_work join_timeout;
	unsigned long join_deadline;
	int join_error;
	u16 peer_next;
	u16 channel_next;
	u16 tx_next;
	u16 join_rates;
	u16 join_basic;
	u16 join_capability;
	u16 join_aid;
	u8 join_peer;
	u8 join_channel;
	u8 join_token;
	u8 join_tx;
	u8 activation_sequence;
	u8 join_phy;
	u8 join_rcpi;
	u8 join_erp;
	u8 join_ssid_len;
	u8 join_ssid[32];
	u8 assoc_ies[64];
	size_t assoc_ies_len;
	u8 assoc_response[1536];
	size_t assoc_response_len;
	bool peer_created;
	bool channel_owned;
};

static void mt6628_radio_fault(struct mt6628_radio *radio, int error);
static void mt6628_join_fail(struct mt6628_radio *radio);
static void mt6628_join_event(struct mt6628_radio *radio, struct sk_buff *skb);

static void mt6628_join_expire(struct work_struct *work)
{
	struct mt6628_radio *radio = container_of(to_delayed_work(work),
						struct mt6628_radio, join_timeout);
	unsigned long now;

	mutex_lock(&radio->policy_mutex);
	now = jiffies;
	if (radio->join_state != MT6628_JOIN_IDLE &&
	    radio->join_state != MT6628_JOIN_CONNECTED) {
		if (time_before(now, radio->join_deadline)) {
			mod_delayed_work(system_wq, &radio->join_timeout,
					 radio->join_deadline - now);
		} else {
			if (!radio->join_error)
				radio->join_error = -ETIMEDOUT;
			wiphy_work_queue(radio->netdev->ieee80211_ptr->wiphy, &radio->rx_work);
		}
	}
	mutex_unlock(&radio->policy_mutex);
}

static void mt6628_scan_finish(struct mt6628_radio *radio, bool aborted)
{
	struct cfg80211_scan_request *request = radio->scan_request;
	struct cfg80211_scan_info info = { .aborted = aborted };

	if (!request)
		return;
	radio->scan_request = NULL;
	radio->scan_passive = 0;
	cancel_delayed_work(&radio->scan_timeout);
	cfg80211_scan_done(request, &info);
}

static int mt6628_scan_send(struct mt6628_radio *radio, unsigned int mask, bool active)
{
	struct cfg80211_scan_request *request = radio->scan_request;
	u8 command[710] = {};
	unsigned int i, count = 0;
	int ret;

	if (radio->scan_token >= 256)
		return -EOVERFLOW;
	/* shortcut: no token reuse until qualified reset recovery */
	radio->scan_sequence = radio->scan_token++;
	command[0] = radio->scan_sequence;
	command[2] = active;
	if (active) {
		command[3] = request->ssids[0].ssid_len ? BIT(2) : BIT(0);
		command[4] = request->ssids[0].ssid_len;
		memcpy(command + 8, request->ssids[0].ssid, command[4]);
	}
	command[42] = 4;
	for (i = 0; i < 14; i++) {
		if (!(mask & BIT(i)))
			continue;
		command[44 + count * 2] = 1;
		command[45 + count * 2] = i + 1;
		count++;
	}
	command[43] = count;
	put_unaligned_le16(request->ie_len, command + 108);
	if (request->ie_len)
		memcpy(command + 110, request->ie, request->ie_len);
	ret = mt6628_sdio_set(radio->dev, 0x1e, command, 110 + request->ie_len);
	if (!ret) {
		radio->scan_deadline = jiffies + msecs_to_jiffies(15000);
		mod_delayed_work(system_wq, &radio->scan_timeout, msecs_to_jiffies(15000));
	}
	return ret;
}

static int mt6628_scan_cancel(struct mt6628_radio *radio)
{
	u8 command[4] = { radio->scan_sequence };
	int ret;

	if (!radio->scan_request)
		return 0;
	radio->scan_uncertain = true;
	mt6628_scan_finish(radio, true);
	ret = mt6628_sdio_set(radio->dev, 0x1f, command, sizeof(command));
	if (ret) {
		dev_warn(&radio->dev->func->dev, "scan cancel failed: %d\n", ret);
		mt6628_radio_fault(radio, ret);
	}
	return ret;
}

static void mt6628_scan_timeout(struct work_struct *work)
{
	struct mt6628_radio *radio = container_of(to_delayed_work(work),
						struct mt6628_radio, scan_timeout);

	mutex_lock(&radio->policy_mutex);
	if (radio->scan_request) {
		if (time_before(jiffies, radio->scan_deadline))
			mod_delayed_work(system_wq, &radio->scan_timeout,
					 radio->scan_deadline - jiffies);
		else
			mt6628_radio_fault(radio, -ETIMEDOUT);
	}
	mutex_unlock(&radio->policy_mutex);
}

static void mt6628_scan_event(struct mt6628_radio *radio, const u8 *event)
{
	unsigned int passive;
	int ret;

	if (!radio->scan_request || event[8] != radio->scan_sequence)
		return;
	passive = radio->scan_passive;
	radio->scan_passive = 0;
	if (!passive) {
		mt6628_scan_finish(radio, false);
		return;
	}
	ret = mt6628_scan_send(radio, passive, false);
	if (ret)
		mt6628_radio_fault(radio, ret);
}

static void mt6628_radio_rx_work(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	struct sk_buff *skb;
	unsigned int budget;
	int error;

	mutex_lock(&radio->policy_mutex);
	error = atomic_xchg(&radio->rx_error, 0);
	if (error && radio->configured)
		mt6628_radio_fault(radio, error);
	if (radio->join_error)
		mt6628_join_fail(radio);
	mutex_unlock(&radio->policy_mutex);

	for (budget = 0; budget < 48; budget++) {
		struct cfg80211_inform_bss data = {};
		struct cfg80211_bss *bss;
		unsigned int channel;

		mutex_lock(&radio->policy_mutex);
		skb = skb_dequeue(&radio->rx);
		if (!skb) {
			mutex_unlock(&radio->policy_mutex);
			break;
		}
		if (skb->cb[2]) {
			if (skb->cb[2] == 1)
				mt6628_scan_event(radio, skb->data);
			else
				mt6628_join_event(radio, skb);
			mutex_unlock(&radio->policy_mutex);
			dev_kfree_skb(skb);
			continue;
		}
		channel = (u8)skb->cb[0];
		if (radio->configured && (radio->channel_mask & BIT(channel - 1))) {
			data.chan = &radio->channels[channel - 1];
			data.signal = (((u8)skb->cb[1] >> 1) - 110) * 100;
			bss = cfg80211_inform_bss_frame_data(wiphy, &data,
							     (void *)skb->data,
							     skb->len, GFP_KERNEL);
			if (bss) {
				struct mt6628_bss *metadata = (void *)bss->priv;

				metadata->rcpi = skb->cb[1];
				metadata->valid = true;
				cfg80211_put_bss(wiphy, bss);
			}
		}
		mutex_unlock(&radio->policy_mutex);
		dev_kfree_skb(skb);
	}
	if (!skb_queue_empty(&radio->rx))
		wiphy_work_queue(wiphy, &radio->rx_work);
}

void mt6628_radio_error(struct mt6628_dev *dev, int error)
{
	struct wiphy *wiphy = dev->wiphy;
	struct mt6628_radio *radio;

	if (!wiphy)
		return;
	radio = wiphy_priv(wiphy);
	atomic_cmpxchg(&radio->rx_error, 0, error);
	wiphy_work_queue(wiphy, &radio->rx_work);
}

void mt6628_radio_rx(struct mt6628_dev *dev, const u8 *packet, size_t len)
{
	struct wiphy *wiphy = dev->wiphy;
	struct mt6628_radio *radio;
	struct sk_buff *skb;
	unsigned int offset, type, limit, kind = 0;
	int error = -EPROTO;
	u16 fc;

	if (!wiphy || len < 8)
		return;
	radio = wiphy_priv(wiphy);
	type = get_unaligned_le16(packet + 2) & 3;
	if (type == 1) {
		unsigned int expected;

		switch (packet[4]) {
		case 0x15:
			expected = 12;
			kind = 1;
			break;
		case 0x18:
		case 0x13:
			expected = 20;
			kind = 2;
			break;
		case 0x17:
			expected = 24;
			kind = 2;
			break;
		case 1:
			expected = 12;
			kind = 2;
			break;
		default:
			return;
		}
		if (len != expected)
			goto fault;
		offset = 0;
		limit = 64;
	} else if (type == 3 && len >= 12) {
		offset = 12 + (packet[4] & 3);
		if (len < offset + 24 || !packet[10] || packet[10] > 14)
			return;
		fc = get_unaligned_le16(packet + offset);
		if ((fc & 0xc70f) ||
		    !is_valid_ether_addr(packet + offset + 16))
			return;
		if ((fc & 0xfc) == 0xb0 || (fc & 0xfc) == 0x10 ||
		    (fc & 0xfc) == 0xc0 || (fc & 0xfc) == 0xa0) {
			if ((get_unaligned_le16(packet + offset + 22) & 0xf) ||
			    len - offset < ((fc & 0xfc) == 0xb0 || (fc & 0xfc) == 0x10 ? 30 : 26))
				return;
			kind = 2;
			offset = 0;
			limit = 64;
		} else {
			if (len < offset + 36 || packet[9] > 220 ||
			    ((fc & 0xfc) != 0x80 && (fc & 0xfc) != 0x50))
				return;
			limit = 48;
		}
	} else {
		return;
	}
	if (skb_queue_len(&radio->rx) >= limit) {
		error = -ENOBUFS;
		goto drop;
	}
	skb = alloc_skb(len - offset, GFP_KERNEL);
	if (!skb) {
		error = -ENOMEM;
		goto drop;
	}
	skb_put_data(skb, packet + offset, len - offset);
	skb->cb[0] = type == 3 ? packet[10] : 0;
	skb->cb[1] = type == 3 ? packet[9] : 0;
	skb->cb[2] = kind;
	skb_queue_tail(&radio->rx, skb);
	wiphy_work_queue(wiphy, &radio->rx_work);
	return;
drop:
	if (!kind)
		return;
fault:
	mt6628_sdio_fault(dev, error);
	mt6628_radio_error(dev, error);
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
	unsigned int mask = 0, passive = 0, i;
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
		if (channel->flags & (IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR |
				      IEEE80211_CHAN_NO_OFDM))
			passive |= BIT(i);
		budget = min(budget, min(channel->max_power, 63) * 2);
	}
	radio->channel_mask = mask;
	radio->passive_mask = passive;
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

static void mt6628_radio_fault(struct mt6628_radio *radio, int error)
{
	int ret;

	mt6628_scan_finish(radio, true);
	if (radio->join_state != MT6628_JOIN_IDLE) {
		if (!radio->join_error)
			radio->join_error = error;
		wiphy_work_queue(radio->netdev->ieee80211_ptr->wiphy, &radio->rx_work);
	}
	radio->configured = false;
	radio->configure_error = error;
	radio->active = false;
	mt6628_sdio_fault(radio->dev, error);
	ret = mt6628_sdio_abort(radio->dev);
	if (ret)
		dev_warn(&radio->dev->func->dev, "terminal reset failed: %d\n", ret);
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
		radio->configure_error = ret;
		radio->configured = false;
		return ret;
	}
	radio->alpha2[0] = alpha2[0];
	radio->alpha2[1] = alpha2[1];
	radio->power_limit = limit;
	radio->configured = true;
	radio->configure_error = 0;
	return 0;
}

static int mt6628_radio_restart(struct mt6628_radio *radio)
{
	int ret;

	if (radio->policy_error)
		return radio->policy_error;
	ret = mt6628_sdio_stop(radio->dev);
	if (ret)
		dev_warn(&radio->dev->func->dev, "recovery stop failed: %d\n", ret);
	radio->configured = false;
	sdio_claim_host(radio->dev->func);
	skb_queue_purge(&radio->rx);
	atomic_set(&radio->rx_error, 0);
	sdio_release_host(radio->dev->func);
	ret = mt6628_hw_start(radio->dev);
	if (ret)
		goto fault;
	if (radio->dev->factory.enable_5ghz && !radio->dev->info.hw_5ghz_disabled) {
		ret = -EOPNOTSUPP;
		goto fault;
	}
	ret = mt6628_radio_update(radio, radio->alpha2, radio->power_limit);
	if (ret)
		goto fault;
	radio->scan_uncertain = false;
	return 0;
fault:
	mt6628_radio_fault(radio, ret);
	return ret;
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
	if (radio->scan_request || radio->scan_uncertain || radio->join_state) {
		mutex_unlock(&radio->policy_mutex);
		return -EBUSY;
	}
	ret = mt6628_radio_update(radio, radio->alpha2, limit);
	if (ret && !radio->configured)
		mt6628_radio_fault(radio, ret);
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static void mt6628_reg_notifier(struct wiphy *wiphy,
				struct regulatory_request *request)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	int ret;

	mutex_lock(&radio->policy_mutex);
	if (request) {
		radio->alpha2[0] = request->alpha2[0];
		radio->alpha2[1] = request->alpha2[1];
	}
	if (radio->join_state) {
		mt6628_radio_snapshot(radio);
		mt6628_radio_fault(radio, -EAGAIN);
		mutex_unlock(&radio->policy_mutex);
		return;
	}
	if (radio->scan_request || radio->scan_uncertain) {
		ret = mt6628_scan_cancel(radio);
		mt6628_radio_snapshot(radio);
		if (!ret)
			mt6628_radio_fault(radio, -EAGAIN);
		mutex_unlock(&radio->policy_mutex);
		return;
	}
	mt6628_radio_snapshot(radio);
	ret = mt6628_radio_update(radio, radio->alpha2, radio->power_limit);
	if (ret)
		mt6628_radio_fault(radio, ret);
	mutex_unlock(&radio->policy_mutex);
}

static int mt6628_net_open(struct net_device *netdev)
{
	struct mt6628_radio *radio = wiphy_priv(netdev->ieee80211_ptr->wiphy);
	u8 command[4] = { 0, 1 };
	int ret;

	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	mutex_lock(&radio->policy_mutex);
	if (!radio->ready || radio->netdev != netdev) {
		ret = -ESHUTDOWN;
	} else if (radio->active || radio->join_state) {
		ret = -EBUSY;
	} else {
		if (!radio->configured || radio->scan_uncertain) {
			ret = mt6628_radio_restart(radio);
			if (ret)
				goto unlock;
		}
		ret = mt6628_sdio_set(radio->dev, 0x15, command, sizeof(command));
		if (ret)
			mt6628_radio_fault(radio, ret);
		else
			radio->active = true;
	}
unlock:
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static int mt6628_net_stop(struct net_device *netdev)
{
	struct mt6628_radio *radio = wiphy_priv(netdev->ieee80211_ptr->wiphy);
	u8 command[4] = {};
	int ret;

	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	mutex_lock(&radio->policy_mutex);
	mt6628_scan_cancel(radio);
	if (radio->join_state)
		mt6628_radio_fault(radio, -ESHUTDOWN);
	if (radio->active) {
		radio->active = false;
		ret = mt6628_sdio_set(radio->dev, 0x15, command, sizeof(command));
		if (ret) {
			dev_warn(&radio->dev->func->dev, "network stop failed: %d\n", ret);
			mt6628_radio_fault(radio, ret);
		}
	}
	mutex_unlock(&radio->policy_mutex);
	return 0;
}

static netdev_tx_t mt6628_net_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	dev_core_stats_tx_dropped_inc(netdev);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops mt6628_net_ops = {
	.ndo_open = mt6628_net_open,
	.ndo_stop = mt6628_net_stop,
	.ndo_start_xmit = mt6628_net_xmit,
};

static struct wireless_dev *
mt6628_add_interface(struct wiphy *wiphy, const char *name, unsigned char assign_type,
		     enum nl80211_iftype type, struct vif_params *params)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	struct net_device *netdev;
	struct wireless_dev *wdev;
	int ret;

	if (type != NL80211_IFTYPE_STATION ||
	    (params && (params->flags || params->use_4addr ||
			!is_zero_ether_addr(params->macaddr))))
		return ERR_PTR(-EOPNOTSUPP);
	mutex_lock(&radio->policy_mutex);
	if (!radio->ready) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	if (radio->netdev) {
		ret = -EBUSY;
		goto unlock;
	}
	if (!radio->configured) {
		ret = radio->configure_error ?: -EAGAIN;
		goto unlock;
	}
	netdev = alloc_netdev(sizeof(*wdev), name, assign_type, ether_setup);
	if (!netdev) {
		ret = -ENOMEM;
		goto unlock;
	}
	wdev = netdev_priv(netdev);
	wdev->wiphy = wiphy;
	wdev->netdev = netdev;
	wdev->iftype = NL80211_IFTYPE_STATION;
	netdev->ieee80211_ptr = wdev;
	netdev->netdev_ops = &mt6628_net_ops;
	netdev->needs_free_netdev = true;
	dev_net_set(netdev, wiphy_net(wiphy));
	SET_NETDEV_DEV(netdev, &radio->dev->func->dev);
	eth_hw_addr_set(netdev, radio->dev->mac_addr);
	ether_addr_copy(netdev->perm_addr, radio->dev->mac_addr);
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	ret = cfg80211_register_netdevice(netdev);
	if (ret) {
		free_netdev(netdev);
		goto unlock;
	}
	radio->netdev = netdev;
	mutex_unlock(&radio->policy_mutex);
	return wdev;
unlock:
	mutex_unlock(&radio->policy_mutex);
	return ERR_PTR(ret);
}

static int mt6628_del_interface(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);

	mutex_lock(&radio->policy_mutex);
	if (!radio->netdev || radio->netdev != wdev->netdev) {
		mutex_unlock(&radio->policy_mutex);
		return -ENODEV;
	}
	mt6628_join_fail(radio);
	radio->netdev = NULL;
	mutex_unlock(&radio->policy_mutex);
	cfg80211_unregister_netdevice(wdev->netdev);
	return 0;
}

static int mt6628_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	unsigned int mask = 0, active, passive, i, channel;
	int ret;

	if (request->n_ssids < 0 || request->n_ssids > 1 ||
	    (request->n_ssids && (!request->ssids || request->ssids[0].ssid_len > 32)) ||
	    request->ie_len > 600 || (request->ie_len && !request->ie) ||
	    !request->n_channels || request->n_channels > 14)
		return -EINVAL;
	if ((request->flags & ~NL80211_SCAN_FLAG_FLUSH) || request->duration ||
	    request->duration_mandatory || request->no_cck || request->scan_6ghz ||
	    request->rates[NL80211_BAND_2GHZ] != BIT(12) - 1 ||
	    !is_broadcast_ether_addr(request->bssid))
		return -EOPNOTSUPP;
	mutex_lock(&radio->policy_mutex);
	if (!radio->ready || !radio->active || !radio->netdev ||
	    request->wdev != radio->netdev->ieee80211_ptr) {
		ret = -ENETDOWN;
		goto unlock;
	}
	if (!radio->configured) {
		ret = radio->configure_error ?: -EAGAIN;
		goto unlock;
	}
	if (radio->scan_request || radio->join_state) {
		ret = -EBUSY;
		goto unlock;
	}
	for (i = 0; i < request->n_channels; i++) {
		for (channel = 0; channel < 14; channel++)
			if (request->channels[i] == &radio->channels[channel])
				break;
		if (channel == 14 || !(radio->channel_mask & BIT(channel))) {
			ret = -EINVAL;
			goto unlock;
		}
		mask |= BIT(channel);
	}
	active = request->n_ssids ? mask & ~radio->passive_mask : 0;
	passive = mask & ~active;
	if (radio->scan_token + !!active + !!passive > 256) {
		ret = -EOVERFLOW;
		goto unlock;
	}
	radio->scan_request = request;
	radio->scan_passive = active ? passive : 0;
	ret = mt6628_scan_send(radio, active ? active : passive, !!active);
	if (ret) {
		radio->scan_request = NULL;
		radio->scan_passive = 0;
		mt6628_radio_fault(radio, ret);
	}
unlock:
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static void mt6628_abort_scan(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);

	mutex_lock(&radio->policy_mutex);
	if (radio->scan_request && radio->scan_request->wdev == wdev)
		mt6628_scan_cancel(radio);
	mutex_unlock(&radio->policy_mutex);
}

static void mt6628_join_header(struct mt6628_radio *radio, u8 *frame, u8 subtype)
{
	memset(frame, 0, 24);
	frame[0] = subtype;
	memcpy(frame + 4, radio->auth_bss->bssid, 6);
	memcpy(frame + 10, radio->dev->mac_addr, 6);
	memcpy(frame + 16, radio->auth_bss->bssid, 6);
}

static int mt6628_join_tx(struct mt6628_radio *radio, const u8 *frame, size_t len)
{
	if (radio->tx_next >= 256)
		return -EOVERFLOW;
	radio->join_tx = radio->tx_next++;
	return mt6628_sdio_mgmt(radio->dev, frame, len, radio->join_peer, radio->join_tx,
				 radio->join_state == MT6628_JOIN_CONNECTED ?
				 jiffies + msecs_to_jiffies(1000) : radio->join_deadline);
}

static int mt6628_join_peer(struct mt6628_radio *radio, u8 state)
{
	u8 command[40] = {};

	command[0] = radio->join_peer;
	command[1] = 0x41;
	memcpy(command + 2, radio->auth_bss->bssid, 6);
	put_unaligned_le16(radio->join_aid, command + 8);
	put_unaligned_le16(1, command + 10);
	command[13] = radio->join_phy;
	put_unaligned_le16(radio->join_rates, command + 14);
	put_unaligned_le16(radio->join_basic, command + 16);
	command[20] = state;
	command[33] = radio->join_rcpi;
	command[34] = state == 2;
	return mt6628_sdio_set_seq(radio->dev, 0x17, command, sizeof(command),
				   &radio->activation_sequence);
}

static int mt6628_join_bss(struct mt6628_radio *radio, bool connected)
{
	u8 command[80] = {};

	command[1] = !connected;
	command[3] = radio->join_ssid_len;
	memcpy(command + 4, radio->join_ssid, radio->join_ssid_len);
	memcpy(command + 36, radio->auth_bss->bssid, 6);
	put_unaligned_le16(radio->join_rates, command + 44);
	put_unaligned_le16(radio->join_basic, command + 46);
	command[48] = radio->join_peer;
	command[51] = !!(radio->join_phy & BIT(1));
	command[53] = 1;
	command[54] = radio->join_phy;
	memcpy(command + 55, radio->dev->mac_addr, 6);
	command[65] = 1;
	command[66] = radio->join_channel;
	command[68] = radio->join_erp;
	command[77] = !!(radio->join_capability & WLAN_CAPABILITY_SHORT_PREAMBLE);
	command[78] = !!(radio->join_capability & WLAN_CAPABILITY_SHORT_SLOT_TIME);
	command[79] = 0x72;
	return mt6628_sdio_set(radio->dev, 0x16, command, sizeof(command));
}

static int mt6628_join_release_channel(struct mt6628_radio *radio)
{
	u8 command[20] = { 0, radio->join_token, 1 };

	if (!radio->channel_owned)
		return 0;
	radio->channel_owned = false;
	return mt6628_sdio_set(radio->dev, 0x20, command, sizeof(command));
}

static void mt6628_join_fail(struct mt6628_radio *radio)
{
	struct wiphy *wiphy;
	struct cfg80211_assoc_failure failure = { .timeout = true };
	enum mt6628_join_state state = radio->join_state;
	u8 remove[8] = { radio->join_peer };
	int ret = 0;

	if (state == MT6628_JOIN_IDLE)
		return;
	wiphy = radio->netdev->ieee80211_ptr->wiphy;
	radio->join_state = MT6628_JOIN_IDLE;
	radio->join_error = 0;
	cancel_delayed_work(&radio->join_timeout);
	if (radio->configured) {
		if (state >= MT6628_JOIN_ACTIVATE)
			ret = mt6628_join_bss(radio, false);
		if (!ret && radio->peer_created) {
			memcpy(remove + 2, radio->auth_bss->bssid, 6);
			ret = mt6628_sdio_set(radio->dev, 0x18, remove, sizeof(remove));
		}
		if (!ret)
			ret = mt6628_join_release_channel(radio);
	}
	radio->peer_created = false;
	radio->channel_owned = false;
	if (state == MT6628_JOIN_CHANNEL || state == MT6628_JOIN_AUTH)
		cfg80211_auth_timeout(radio->netdev, radio->auth_bss->bssid);
	if (radio->assoc_bss) {
		failure.bss[0] = radio->assoc_bss;
		radio->assoc_bss = NULL;
		cfg80211_assoc_failure(radio->netdev, &failure);
	}
	if (state == MT6628_JOIN_CONNECTED)
		cfg80211_disconnected(radio->netdev, WLAN_REASON_UNSPECIFIED,
				      NULL, 0, true, GFP_KERNEL);
	cfg80211_put_bss(wiphy, radio->auth_bss);
	radio->auth_bss = NULL;
	if (ret)
		mt6628_radio_fault(radio, ret);
}

static int mt6628_join_rates(struct mt6628_radio *radio, const u8 *ies, size_t len,
			     bool ssid)
{
	u16 rates = 0, basic = 0;
	bool found_ssid = false;
	size_t offset = 0;
	unsigned int i, j;

	while (offset < len) {
		u8 id, size;

		if (len - offset < 2)
			return -EINVAL;
		id = ies[offset];
		size = ies[offset + 1];
		offset += 2;
		if (size > len - offset)
			return -EINVAL;
		if (id == WLAN_EID_ERP_INFO) {
			if (size != 1 || ies[offset] & ~7)
				return -EINVAL;
			radio->join_erp = !!(ies[offset] & BIT(1));
		}
		if (id == WLAN_EID_SSID && ssid) {
			if (found_ssid || !size || size > 32)
				return -EINVAL;
			found_ssid = true;
			radio->join_ssid_len = size;
			memcpy(radio->join_ssid, ies + offset, size);
		}
		if (id == WLAN_EID_SUPP_RATES || id == WLAN_EID_EXT_SUPP_RATES) {
			if (!size || (id == WLAN_EID_SUPP_RATES && size > 8))
				return -EINVAL;
			for (i = 0; i < size; i++) {
				for (j = 0; j < ARRAY_SIZE(radio->rates); j++)
					if ((ies[offset + i] & 0x7f) * 5 == radio->rates[j].bitrate)
						break;
				if (j == ARRAY_SIZE(radio->rates)) {
					if (ies[offset + i] & 0x80)
						return -EOPNOTSUPP;
					continue;
				}
				j = j < 4 ? j : j + 2;
				rates |= BIT(j);
				if (ies[offset + i] & 0x80)
					basic |= BIT(j);
			}
		}
		offset += size;
	}
	if (!rates || !basic || (ssid && !found_ssid))
		return -EINVAL;
	if (!ssid && ((basic & ~radio->join_rates) || (rates & ~radio->join_rates)))
		return -EOPNOTSUPP;
	radio->join_rates = rates;
	radio->join_basic = basic;
	radio->join_phy = (rates & 0xf ? BIT(0) : 0) | (rates & ~0xf ? BIT(1) : 0);
	return 0;
}

static int mt6628_auth(struct wiphy *wiphy, struct net_device *netdev,
		       struct cfg80211_auth_request *request)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	const struct cfg80211_bss_ies *ies;
	const struct mt6628_bss *metadata;
	u8 command[20] = {};
	unsigned int channel;
	int ret;

	if (!request->bss || request->auth_type != NL80211_AUTHTYPE_OPEN_SYSTEM ||
	    request->link_id >= 0 || request->key_len || request->auth_data_len ||
	    request->ie_len || request->supported_selectors_len ||
	    !(request->bss->capability & WLAN_CAPABILITY_ESS) ||
	    request->bss->capability & (WLAN_CAPABILITY_PRIVACY | WLAN_CAPABILITY_IBSS))
		return -EOPNOTSUPP;
	mutex_lock(&radio->policy_mutex);
	if (!radio->ready || !radio->active || !radio->configured || radio->netdev != netdev) {
		ret = -ENETDOWN;
		goto out;
	}
	if (radio->scan_request || radio->scan_uncertain || radio->join_state) {
		ret = -EBUSY;
		goto out;
	}
	if (radio->peer_next >= 20 || radio->channel_next >= 256 || radio->tx_next >= 256) {
		ret = -EOVERFLOW;
		goto out;
	}
	for (channel = 0; channel < 14; channel++)
		if (request->bss->channel == &radio->channels[channel])
			break;
	if (channel == 14 || !(radio->channel_mask & BIT(channel)) ||
	    radio->passive_mask & BIT(channel)) {
		ret = -EACCES;
		goto out;
	}
	metadata = (void *)request->bss->priv;
	if (!metadata->valid) {
		ret = -EAGAIN;
		goto out;
	}
	radio->join_rcpi = metadata->rcpi;
	radio->join_erp = 0;
	rcu_read_lock();
	ies = rcu_dereference(request->bss->ies);
	ret = ies ? mt6628_join_rates(radio, ies->data, ies->len, true) : -EINVAL;
	rcu_read_unlock();
	if (ret)
		goto out;
	radio->join_peer = radio->peer_next++;
	radio->join_token = radio->channel_next++;
	radio->join_channel = channel + 1;
	radio->join_aid = 0;
	radio->join_capability = WLAN_CAPABILITY_ESS |
		(request->bss->capability & (WLAN_CAPABILITY_SHORT_PREAMBLE |
					   WLAN_CAPABILITY_SHORT_SLOT_TIME));
	radio->auth_bss = request->bss;
	cfg80211_ref_bss(wiphy, radio->auth_bss);
	radio->join_state = MT6628_JOIN_CHANNEL;
	command[1] = radio->join_token;
	command[3] = radio->join_channel;
	command[5] = 1;
	put_unaligned_le32(2000, command + 8);
	memcpy(command + 12, request->bss->bssid, 6);
	ret = mt6628_sdio_set(radio->dev, 0x20, command, sizeof(command));
	if (ret) {
		radio->join_state = MT6628_JOIN_IDLE;
		cfg80211_put_bss(wiphy, radio->auth_bss);
		radio->auth_bss = NULL;
		mt6628_radio_fault(radio, ret);
	} else {
		radio->channel_owned = true;
		radio->join_deadline = jiffies + msecs_to_jiffies(2000);
		mod_delayed_work(system_wq, &radio->join_timeout, msecs_to_jiffies(2000));
	}
out:
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static int mt6628_assoc(struct wiphy *wiphy, struct net_device *netdev,
			struct cfg80211_assoc_request *request)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	u8 frame[128] = {};
	unsigned int i, count = 0;
	size_t len;
	int ret;

	if (!request->bss || !(request->bss->capability & WLAN_CAPABILITY_ESS) ||
	    request->bss->capability & (WLAN_CAPABILITY_PRIVACY | WLAN_CAPABILITY_IBSS) ||
	    request->link_id >= 0 || request->prev_bssid ||
	    request->use_mfp || request->ie_len || request->crypto.wpa_versions ||
	    request->crypto.cipher_group || request->crypto.n_ciphers_pairwise ||
	    request->crypto.n_akm_suites || request->crypto.control_port ||
	    request->fils_kek_len || request->supported_selectors_len ||
	    request->flags & ~(ASSOC_REQ_DISABLE_HT | ASSOC_REQ_DISABLE_VHT |
			       ASSOC_REQ_DISABLE_HE | ASSOC_REQ_DISABLE_EHT |
			       ASSOC_REQ_DISABLE_UHR))
		return -EOPNOTSUPP;
	mutex_lock(&radio->policy_mutex);
	if (radio->netdev != netdev || !radio->ready || !radio->configured ||
	    radio->join_state != MT6628_JOIN_AUTHENTICATED ||
	    !ether_addr_equal(request->bss->bssid, radio->auth_bss->bssid) ||
	    request->bss->channel != radio->auth_bss->channel) {
		ret = -EINVAL;
		goto out;
	}
	if (!time_before(jiffies, radio->join_deadline)) {
		ret = -ETIMEDOUT;
		goto out;
	}
	mt6628_join_header(radio, frame, 0);
	put_unaligned_le16(radio->join_capability, frame + 24);
	put_unaligned_le16(1, frame + 26);
	len = 28;
	frame[len++] = WLAN_EID_SSID;
	frame[len++] = radio->join_ssid_len;
	memcpy(frame + len, radio->join_ssid, radio->join_ssid_len);
	len += radio->join_ssid_len;
	frame[len++] = WLAN_EID_SUPP_RATES;
	for (i = 0; i < ARRAY_SIZE(radio->rates); i++) {
		unsigned int bit = i < 4 ? i : i + 2;

		if (!(radio->join_rates & BIT(bit)))
			continue;
		if (count == 8) {
			frame[len] = 8;
			len += 9;
			frame[len++] = WLAN_EID_EXT_SUPP_RATES;
			count = 0;
		}
		frame[len + 1 + count++] = radio->rates[i].bitrate / 5 |
			(radio->join_basic & BIT(bit) ? 0x80 : 0);
	}
	frame[len] = count;
	len += count + 1;
	radio->assoc_ies_len = len - 28;
	memcpy(radio->assoc_ies, frame + 28, radio->assoc_ies_len);
	ret = mt6628_join_tx(radio, frame, len);
	if (ret) {
		mt6628_radio_fault(radio, ret);
		goto out;
	}
	radio->assoc_bss = request->bss;
	radio->join_state = MT6628_JOIN_ASSOC;
out:
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static void mt6628_join_assoc_done(struct mt6628_radio *radio, u16 status)
{
	struct cfg80211_rx_assoc_resp_data data = {
		.buf = radio->assoc_response,
		.len = radio->assoc_response_len,
		.req_ies = radio->assoc_ies,
		.req_ies_len = radio->assoc_ies_len,
	};

	data.links[0].bss = radio->assoc_bss;
	radio->assoc_bss = NULL;
	if (!status) {
		radio->join_state = MT6628_JOIN_CONNECTED;
		cancel_delayed_work(&radio->join_timeout);
	} else {
		radio->join_state = MT6628_JOIN_AUTHENTICATED;
	}
	cfg80211_rx_assoc_resp(radio->netdev, &data);
	if (status)
		mt6628_join_fail(radio);
}

static bool mt6628_join_ies_valid(const u8 *ies, size_t len)
{
	while (len) {
		size_t size;

		if (len < 2)
			return false;
		size = ies[1] + 2;
		if (size > len)
			return false;
		ies += size;
		len -= size;
	}
	return true;
}

static void mt6628_join_event(struct mt6628_radio *radio, struct sk_buff *skb)
{
	const u8 *packet = skb->data, *frame;
	size_t len;
	u32 interval;
	u16 status, aid;
	int ret = 0;
	u8 auth[30] = {};

	if (!radio->auth_bss || radio->join_error || !radio->configured)
		return;
	if (radio->join_state != MT6628_JOIN_CONNECTED &&
	    !time_before(jiffies, radio->join_deadline)) {
		mt6628_radio_fault(radio, -ETIMEDOUT);
		mt6628_join_fail(radio);
		return;
	}
	if ((get_unaligned_le16(packet + 2) & 3) == 1) {
		switch (packet[4]) {
		case 0x18:
			if (radio->join_state != MT6628_JOIN_CHANNEL || packet[8] ||
			    packet[9] != radio->join_token)
				return;
			interval = get_unaligned_le32(packet + 16);
			if (packet[10] || packet[11] != radio->join_channel || packet[12] ||
			    packet[13] != 1 || packet[14] || interval <= 10 || interval > 2000) {
				ret = -EPROTO;
				break;
			}
			radio->join_deadline = jiffies + msecs_to_jiffies(interval - 10);
			mod_delayed_work(system_wq, &radio->join_timeout,
					 msecs_to_jiffies(interval - 10));
			ret = mt6628_join_peer(radio, 0);
			if (ret)
				break;
			radio->peer_created = true;
			mt6628_join_header(radio, auth, 0xb0);
			put_unaligned_le16(1, auth + 26);
			ret = mt6628_join_tx(radio, auth, sizeof(auth));
			if (!ret)
				radio->join_state = MT6628_JOIN_AUTH;
			break;
		case 0x17:
			if ((radio->join_state == MT6628_JOIN_AUTH ||
			     radio->join_state == MT6628_JOIN_ASSOC) &&
			    packet[8] == radio->join_tx && packet[9])
				ret = -EREMOTEIO;
			break;
		case 0x13:
			if (radio->join_state != MT6628_JOIN_ACTIVATE ||
			    packet[5] != radio->activation_sequence ||
			    !ether_addr_equal(packet + 8, radio->auth_bss->bssid) ||
			    packet[14] != radio->join_peer || packet[15])
				return;
			ret = mt6628_join_release_channel(radio);
			if (!ret)
				mt6628_join_assoc_done(radio, 0);
			break;
		case 1:
			if (radio->join_state == MT6628_JOIN_ACTIVATE &&
			    packet[5] == radio->activation_sequence && packet[8] == 0x17)
				ret = -EPROTO;
			break;
		}
		goto out;
	}
	len = 12 + (packet[4] & 3);
	frame = packet + len;
	len = skb->len - len;
	if (packet[8] != radio->join_peer || packet[10] != radio->join_channel ||
	    !ether_addr_equal(frame + 4, radio->dev->mac_addr) ||
	    !ether_addr_equal(frame + 10, radio->auth_bss->bssid) ||
	    !ether_addr_equal(frame + 16, radio->auth_bss->bssid))
		return;
	if (frame[0] == 0xb0 && radio->join_state == MT6628_JOIN_AUTH) {
		if (len < 30 || get_unaligned_le16(frame + 24) ||
		    get_unaligned_le16(frame + 26) != 2 ||
		    !mt6628_join_ies_valid(frame + 30, len - 30))
			return;
		status = get_unaligned_le16(frame + 28);
		radio->join_state = MT6628_JOIN_AUTHENTICATED;
		cfg80211_rx_mlme_mgmt(radio->netdev, frame, len);
		if (status)
			mt6628_join_fail(radio);
	} else if (frame[0] == 0x10 && radio->join_state == MT6628_JOIN_ASSOC) {
		if (len < 30 || len > sizeof(radio->assoc_response) ||
		    !mt6628_join_ies_valid(frame + 30, len - 30))
			return;
		status = get_unaligned_le16(frame + 26);
		if (packet[9] <= 220)
			radio->join_rcpi = packet[9];
		memcpy(radio->assoc_response, frame, len);
		radio->assoc_response_len = len;
		if (status) {
			mt6628_join_assoc_done(radio, status);
			return;
		}
		aid = get_unaligned_le16(frame + 28) & 0x3fff;
		if (!aid || aid > 2007 ||
		    !(get_unaligned_le16(frame + 24) & WLAN_CAPABILITY_ESS) ||
		    (get_unaligned_le16(frame + 24) &
		     (WLAN_CAPABILITY_PRIVACY | WLAN_CAPABILITY_IBSS))) {
			ret = -EPROTO;
			goto out;
		}
		ret = mt6628_join_rates(radio, frame + 30, len - 30, false);
		if (ret)
			goto out;
		radio->join_aid = aid;
		radio->join_capability &= get_unaligned_le16(frame + 24);
		radio->join_state = MT6628_JOIN_ACTIVATE;
		ret = mt6628_join_bss(radio, true);
		if (!ret)
			ret = mt6628_join_peer(radio, 2);
	} else if ((frame[0] == 0xc0 || frame[0] == 0xa0) && len >= 26) {
		cfg80211_rx_mlme_mgmt(radio->netdev, frame, len);
		if (radio->join_state == MT6628_JOIN_CONNECTED)
			radio->join_state = MT6628_JOIN_ACTIVATE;
		mt6628_join_fail(radio);
	}
out:
	if (ret) {
		mt6628_radio_fault(radio, ret);
		mt6628_join_fail(radio);
	}
}

static int mt6628_disconnect(struct wiphy *wiphy, struct net_device *netdev,
			     const u8 *address, u16 reason, bool local, u8 subtype)
{
	struct mt6628_radio *radio = wiphy_priv(wiphy);
	u8 frame[26];
	int ret = 0;

	mutex_lock(&radio->policy_mutex);
	if (radio->netdev != netdev || !radio->auth_bss ||
	    !ether_addr_equal(address, radio->auth_bss->bssid)) {
		ret = -ENOTCONN;
		goto out;
	}
	mt6628_join_header(radio, frame, subtype);
	put_unaligned_le16(reason, frame + 24);
	if (!local && radio->peer_created && radio->configured)
		ret = mt6628_join_tx(radio, frame, sizeof(frame));
	if (ret)
		mt6628_radio_fault(radio, ret);
	if (radio->join_state == MT6628_JOIN_CONNECTED)
		radio->join_state = MT6628_JOIN_ACTIVATE;
	mt6628_join_fail(radio);
	cfg80211_tx_mlme_mgmt(netdev, frame, sizeof(frame), false);
out:
	mutex_unlock(&radio->policy_mutex);
	return ret;
}

static int mt6628_deauth(struct wiphy *wiphy, struct net_device *netdev,
			 struct cfg80211_deauth_request *request)
{
	if (request->ie_len)
		return -EOPNOTSUPP;
	return mt6628_disconnect(wiphy, netdev, request->bssid, request->reason_code,
				 request->local_state_change, 0xc0);
}

static int mt6628_disassoc(struct wiphy *wiphy, struct net_device *netdev,
			   struct cfg80211_disassoc_request *request)
{
	if (request->ie_len)
		return -EOPNOTSUPP;
	return mt6628_disconnect(wiphy, netdev, request->ap_addr, request->reason_code,
				 request->local_state_change, 0xa0);
}

static const struct cfg80211_ops mt6628_cfg_ops = {
	.add_virtual_intf = mt6628_add_interface,
	.del_virtual_intf = mt6628_del_interface,
	.set_tx_power = mt6628_set_tx_power,
	.scan = mt6628_scan,
	.abort_scan = mt6628_abort_scan,
	.auth = mt6628_auth,
	.assoc = mt6628_assoc,
	.deauth = mt6628_deauth,
	.disassoc = mt6628_disassoc,
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
	INIT_DELAYED_WORK(&radio->scan_timeout, mt6628_scan_timeout);
	INIT_DELAYED_WORK(&radio->join_timeout, mt6628_join_expire);
	atomic_set(&radio->rx_error, 0);
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
	wiphy->bss_priv_size = sizeof(struct mt6628_bss);
	wiphy->max_scan_ssids = 1;
	wiphy->max_scan_ie_len = 600;
	ether_addr_copy(wiphy->perm_addr, dev->mac_addr);
	set_wiphy_dev(wiphy, &dev->func->dev);
	ret = wiphy_register(wiphy);
	if (ret)
		goto free;
	mutex_lock(&radio->policy_mutex);
	if (!radio->configured)
		ret = radio->configure_error;
	if (ret) {
		mutex_unlock(&radio->policy_mutex);
		wiphy_unregister(wiphy);
		goto free;
	}
	sdio_claim_host(dev->func);
	dev->wiphy = wiphy;
	sdio_release_host(dev->func);
	radio->ready = true;
	mutex_unlock(&radio->policy_mutex);
	return 0;
free:
	wiphy_free(wiphy);
	return ret;
}

void mt6628_radio_unregister(struct mt6628_dev *dev)
{
	struct wiphy *wiphy;
	struct mt6628_radio *radio;

	wiphy = dev->wiphy;
	if (!wiphy)
		return;
	radio = wiphy_priv(wiphy);
	rtnl_lock();
	mutex_lock(&radio->policy_mutex);
	radio->ready = false;
	mutex_unlock(&radio->policy_mutex);
	if (radio->netdev)
		dev_close(radio->netdev);
	wiphy_lock(wiphy);
	sdio_claim_host(dev->func);
	dev->wiphy = NULL;
	sdio_release_host(dev->func);
	if (radio->netdev)
		mt6628_del_interface(wiphy, radio->netdev->ieee80211_ptr);
	wiphy_work_cancel(wiphy, &radio->rx_work);
	skb_queue_purge(&radio->rx);
	wiphy_unlock(wiphy);
	rtnl_unlock();
	cancel_delayed_work_sync(&radio->scan_timeout);
	cancel_delayed_work_sync(&radio->join_timeout);
	wiphy_unregister(wiphy);
	wiphy_free(wiphy);
}
