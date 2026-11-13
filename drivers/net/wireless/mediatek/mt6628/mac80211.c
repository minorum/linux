// SPDX-License-Identifier: GPL-2.0-only
#include <linux/etherdevice.h>
#include <linux/bottom_half.h>
#include <linux/mmc/sdio_func.h>
#include <linux/unaligned.h>
#include <net/mac80211.h>

#include "runtime.h"
#include "wmt-rx.h"

struct mt6628_mac {
	struct mt6628_dev *dev;
	struct ieee80211_hw *hw;
	struct ieee80211_vif *vif;
	struct ieee80211_supported_band band;
	struct ieee80211_channel channels[14];
	struct ieee80211_rate rates[12];
	struct mutex mutex;
	struct sk_buff_head tx;
	struct sk_buff_head rx;
	struct wiphy_work rx_work;
	u32 rx_generation;
	struct ieee80211_key_conf *keys[5];
	u8 pending_ptk[ETH_ALEN];	u8 pending_keys;
	struct wiphy_work key_work;
	struct work_struct tx_work;
	struct completion event_done;
	u8 event_id;
	u8 event_token;
	u8 event_sequence;
	int event_result;
	u16 tx_next;
	u16 channel_next;
	u8 peer;
	u8 channel_token;
	u8 channel;
	u16 supported;
	u16 basic;
	u8 phy;
	bool started;
	bool peer_created;
	bool associated;
	bool bss_configured;
	bool channel_owned;
	unsigned long channel_deadline;
	u8 multicast[200];
	u8 bssid[ETH_ALEN];
	u8 ssid[32];
	u8 ssid_len;
	u8 rcpi;
	char alpha2[2];
	unsigned int channel_mask;
	unsigned int passive_mask;
	int regulatory_power;
	int policy_error;
	struct cfg80211_scan_request *scan_request;
	struct work_struct scan_work;
	struct delayed_work scan_timeout;
	u16 scan_next;
	u8 scan_sequence;
	unsigned int scan_passive;
	unsigned long scan_deadline;
	bool scan_done;
};

static void mt6628_mac_scan_cancel(struct mt6628_mac *mac);

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

static void mt6628_radio_snapshot(struct mt6628_mac *radio)
{
	unsigned int mask = 0, passive = 0, i;
	int budget = 127;

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

static int mt6628_radio_build(struct mt6628_mac *radio, const char alpha2[2],
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

static struct mt6628_mac *mt6628_mac_priv(struct wiphy *wiphy)
{
	return wiphy_to_ieee80211_hw(wiphy)->priv;
}

static int mt6628_mac_policy(struct mt6628_mac *mac)
{
	u8 domain[56], power[40];
	int ret;

	mt6628_radio_snapshot(mac);
	ret = mt6628_radio_build(mac, mac->alpha2, mac->hw->conf.power_level * 100,
				 domain, power);
	if (!ret)
		ret = mt6628_sdio_set(mac->dev, 0x28, power, sizeof(power));
	if (!ret)
		ret = mt6628_sdio_set(mac->dev, 0x13, domain, sizeof(domain));
	return ret;
}

static void mt6628_mac_regulatory(struct wiphy *wiphy, struct regulatory_request *request)
{
	struct mt6628_mac *mac = mt6628_mac_priv(wiphy);
	int ret;

	mutex_lock(&mac->mutex);
	mt6628_mac_scan_cancel(mac);
	if (request)
		memcpy(mac->alpha2, request->alpha2, 2);
	ret = mt6628_mac_policy(mac);
	if (!ret && mac->peer_created &&
	    (!(mac->channel_mask & BIT(mac->channel - 1)) ||
	     mac->passive_mask & BIT(mac->channel - 1)))
		ret = -EACCES;
	mutex_unlock(&mac->mutex);
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
}

static void mt6628_event_arm(struct mt6628_mac *mac, u8 id, u8 token)
{
	reinit_completion(&mac->event_done);
	mac->event_token = token;
	mac->event_result = -ETIMEDOUT;
	/* Event consistency */
	smp_store_release(&mac->event_id, id);
}

static int mt6628_event_wait(struct mt6628_mac *mac, int ret)
{
	if (!ret) {
		if (!wait_for_completion_timeout(&mac->event_done, msecs_to_jiffies(1000)))
			ret = -ETIMEDOUT;
		else
			ret = mac->event_result;
	}
	WRITE_ONCE(mac->event_id, 0);
	return ret;
}

static bool mt6628_mac_control_tx(const u8 *frame, size_t len)
{
	static const u8 eapol[] = { 0xaa, 0xaa, 3, 0, 0, 0, 0x88, 0x8e };

	return len >= 32 && (get_unaligned_le16(frame) & 0x87ff) == 0x0108 &&
		!(get_unaligned_le16(frame + 22) & 15) &&
		!memcmp(frame + 24, eapol, sizeof(eapol));
}

static bool mt6628_mac_ethernet_valid(struct mt6628_mac *mac,
				      const struct ieee80211_tx_info *info,
				      const u8 *frame, size_t len)
{
	return mac->associated && mac->vif && mac->keys[4] &&
		info->control.hw_key == mac->keys[4] && info->control.vif == mac->vif &&
		len >= ETH_HLEN && len <= 1518 && !is_zero_ether_addr(frame) &&
		ether_addr_equal(frame + ETH_ALEN, mac->dev->mac_addr);
}

static void mt6628_mac_tx_work(struct work_struct *work)
{
	struct mt6628_mac *mac = container_of(work, struct mt6628_mac, tx_work);
	struct sk_buff *skb;

	mutex_lock(&mac->mutex);
	while ((skb = skb_dequeue(&mac->tx))) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
		bool management, pending;
		unsigned long deadline;
		u8 token = 0;
		int ret;

		if (atomic_read(&mac->dev->error) || !READ_ONCE(mac->started) ||
		    !mac->peer_created ||
		    (!mac->associated && !time_before(jiffies, mac->channel_deadline)) ||
		    skb_linearize(skb))
			goto drop;
		if (info->flags & IEEE80211_TX_CTL_HW_80211_ENCAP) {
			if (mt6628_mac_ethernet_valid(mac, info, skb->data, skb->len))
				mt6628_sdio_ethernet(mac->dev, skb->data, skb->len, mac->peer,
						     jiffies + msecs_to_jiffies(1000));
			goto drop;
		}
		if (skb->len < 24)
			goto drop;
		management = ieee80211_is_mgmt(((struct ieee80211_hdr *)skb->data)->frame_control);
		if (!management && !mt6628_mac_control_tx(skb->data, skb->len))
			goto drop;
		if (!management && (get_unaligned_le16(skb->data) & 0x4000)) {
			u8 addresses[12];

			if (!mac->associated || !mac->vif || !mac->keys[4] ||
			    info->control.hw_key != mac->keys[4] || info->control.vif != mac->vif)
				goto drop;
			memcpy(addresses, skb->data + 16, 6);
			memcpy(addresses + 6, skb->data + 10, 6);
			skb_pull(skb, 18);
			memcpy(skb->data, addresses, sizeof(addresses));
			mt6628_sdio_ethernet(mac->dev, skb->data, skb->len, mac->peer,
					     jiffies + msecs_to_jiffies(1000));
			goto drop;
		}
		if (management) {
			if (mac->tx_next == 256)
				mac->tx_next = 1;
			token = mac->tx_next++;
			mt6628_event_arm(mac, 0x17, token);
		}
		deadline = jiffies + msecs_to_jiffies(1000);
		if (!mac->associated && time_before(mac->channel_deadline, deadline))
			deadline = mac->channel_deadline;
		ret = mt6628_sdio_native(mac->dev, skb->data, skb->len, mac->peer, token, deadline);
		if (!management)
			goto drop;
		pending = !ret;
		ret = mt6628_event_wait(mac, ret);
		if (ret < 0) {
			if (pending)
				mt6628_sdio_fault(mac->dev, ret);
			goto drop;
		}
		ieee80211_tx_info_clear_status(info);
		memset(info->status.rates, 0, sizeof(info->status.rates));
		info->status.rates[0].idx = -1;
		if (!ret)
			info->flags |= IEEE80211_TX_STAT_ACK;
		ieee80211_tx_status_irqsafe(mac->hw, skb);
		continue;
drop:
		ieee80211_free_txskb(mac->hw, skb);
	}
	mutex_unlock(&mac->mutex);
}

static void mt6628_mac_tx(struct ieee80211_hw *hw,
			  struct ieee80211_tx_control *control, struct sk_buff *skb)
{
	struct mt6628_mac *mac = hw->priv;
	unsigned long flags;

	spin_lock_irqsave(&mac->tx.lock, flags);
	if (!mac->started || !READ_ONCE(mac->peer_created) || mac->tx.qlen >= 64) {
		spin_unlock_irqrestore(&mac->tx.lock, flags);
		ieee80211_free_txskb(hw, skb);
		return;
	}
	__skb_queue_tail(&mac->tx, skb);
	queue_work(system_percpu_wq, &mac->tx_work);
	spin_unlock_irqrestore(&mac->tx.lock, flags);
}

static int mt6628_mac_release_channel(struct mt6628_mac *mac)
{
	u8 command[20] = { 0, mac->channel_token, 1 };
	int ret;

	if (!mac->channel_owned)
		return 0;
	ret = mt6628_sdio_set(mac->dev, 0x20, command, sizeof(command));
	if (!ret)
		mac->channel_owned = false;
	return ret;
}

static int mt6628_mac_channel(struct mt6628_mac *mac, const u8 *bssid)
{
	u8 command[20] = {};
	int ret;

	if (!mac->channel ||
	    !(mac->channel_mask & BIT(mac->channel - 1)) ||
	    mac->passive_mask & BIT(mac->channel - 1))
		return -EOPNOTSUPP;
	ret = mt6628_mac_release_channel(mac);
	if (ret)
		return ret;
	if (mac->channel_next == 256)
		mac->channel_next = 0;
	mac->channel_token = mac->channel_next++;
	command[1] = mac->channel_token;
	command[3] = mac->channel;
	command[5] = 1;
	put_unaligned_le32(2000, command + 8);
	ether_addr_copy(command + 12, bssid);
	mt6628_event_arm(mac, 0x18, mac->channel_token);
	ret = mt6628_sdio_set(mac->dev, 0x20, command, sizeof(command));
	if (ret)
		return mt6628_event_wait(mac, ret);
	mac->channel_owned = true;
	ret = mt6628_event_wait(mac, 0);
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
	return ret;
}

static u16 mt6628_mac_rates(u32 rates)
{
	return (rates & 0xf) | ((rates & 0xff0) << 2);
}

static int mt6628_mac_bss(struct mt6628_mac *mac, bool connected)
{
	struct ieee80211_vif *vif = mac->vif;
	u8 command[80] = {};

	if (!connected) {
		u8 abort[4] = {};
		int ret;

		ret = mt6628_sdio_set(mac->dev, 0x1b, abort, sizeof(abort));
		if (ret)
			return ret;
	}
	command[1] = !connected;
	command[3] = mac->ssid_len;
	memcpy(command + 4, mac->ssid, mac->ssid_len);
	ether_addr_copy(command + 36, mac->bssid);
	put_unaligned_le16(mac->supported, command + 44);
	put_unaligned_le16(mac->basic, command + 46);
	command[48] = mac->peer;
	command[51] = !!(mac->phy & BIT(1));
	command[53] = 1;
	command[54] = mac->phy;
	ether_addr_copy(command + 55, mac->dev->mac_addr);
	command[65] = 1;
	command[66] = mac->channel;
	command[68] = vif->bss_conf.use_cts_prot;
	command[77] = vif->bss_conf.use_short_preamble;
	command[78] = vif->bss_conf.use_short_slot;
	command[79] = 0x72;
	return mt6628_sdio_set(mac->dev, 0x16, command, sizeof(command));
}

static int mt6628_mac_peer(struct mt6628_mac *mac, const u8 *address, u8 state)
{
	u8 command[40] = {};
	int ret;

	command[0] = mac->peer;
	command[1] = 0x41;
	ether_addr_copy(command + 2, address);
	put_unaligned_le16(mac->vif->cfg.aid, command + 8);
	put_unaligned_le16(1, command + 10);
	command[13] = mac->phy;
	put_unaligned_le16(mac->supported, command + 14);
	put_unaligned_le16(mac->basic, command + 16);
	command[20] = state;
	command[33] = mac->rcpi;
	command[34] = state == 2;
	if (state == 2) {
		mt6628_event_arm(mac, 0x13, mac->peer);
		ret = mt6628_sdio_set_seq(mac->dev, 0x17, command, sizeof(command),
					 &mac->event_sequence);
		return mt6628_event_wait(mac, ret);
	}
	return mt6628_sdio_set(mac->dev, 0x17, command, sizeof(command));
}

static int mt6628_mac_target(struct mt6628_mac *mac, struct ieee80211_sta *sta)
{
	struct cfg80211_bss *bss;
	const struct cfg80211_bss_ies *ies;
	const u8 *ssid;
	int ret = -EINVAL;

	bss = cfg80211_get_bss(mac->hw->wiphy, &mac->channels[mac->channel - 1],
			       sta->addr, NULL, 0, IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);
	if (!bss)
		return -ENOENT;
	rcu_read_lock();
	ies = rcu_dereference(bss->ies);
	ssid = ies ? cfg80211_find_ie(WLAN_EID_SSID, ies->data, ies->len) : NULL;
	if (ssid && ssid[1] && ssid[1] <= sizeof(mac->ssid)) {
		mac->ssid_len = ssid[1];
		memcpy(mac->ssid, ssid + 2, mac->ssid_len);
		ether_addr_copy(mac->bssid, sta->addr);
		mac->rcpi = clamp((bss->signal / 100 + 110) * 2, 0, 220);
		ret = 0;
	}
	rcu_read_unlock();
	cfg80211_put_bss(mac->hw->wiphy, bss);
	return ret;
}

static int mt6628_mac_sta_state(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
				struct ieee80211_sta *sta, enum ieee80211_sta_state old,
				enum ieee80211_sta_state new)
{
	struct mt6628_mac *mac = hw->priv;
	u8 command[8] = {};
	int ret = 0;

	if (new == IEEE80211_STA_NOTEXIST)
		wiphy_work_flush(hw->wiphy, &mac->key_work);
	mutex_lock(&mac->mutex);
	if (old == IEEE80211_STA_NOTEXIST && new == IEEE80211_STA_NONE) {
		if (mac->peer_created) {
			ret = -ENOSPC;
			goto out;
		}
		mac->rx_generation++;
		mac->peer = 0;
		mac->supported = mt6628_mac_rates(sta->deflink.supp_rates[NL80211_BAND_2GHZ]);
		mac->basic = mt6628_mac_rates(vif->bss_conf.basic_rates);
		if (!mac->channel || !mac->supported || !mac->basic ||
		    (mac->basic & ~mac->supported)) {
			ret = -EINVAL;
			goto out;
		}
		mac->phy = (mac->supported & 0xf ? BIT(0) : 0) |
			   (mac->supported & ~0xf ? BIT(1) : 0);
		ret = mt6628_mac_target(mac, sta);
		if (!ret)
			ret = mt6628_mac_channel(mac, sta->addr);
		if (!ret)
			ret = mt6628_mac_peer(mac, sta->addr, 0);
		if (!ret)
			mac->peer_created = true;
		if (ret) {
			int err;

			err = mt6628_mac_release_channel(mac);
			if (err)
				mt6628_sdio_fault(mac->dev, err);
		}
	} else if (new == IEEE80211_STA_NOTEXIST && mac->peer_created) {
		struct sk_buff *skb;
		int err;

		mac->rx_generation++;
		WRITE_ONCE(mac->peer_created, false);
		while ((skb = skb_dequeue(&mac->tx)))
			ieee80211_free_txskb(hw, skb);
		command[0] = mac->peer;
		ether_addr_copy(command + 2, sta->addr);
		if (mac->bss_configured)
			ret = mt6628_mac_bss(mac, false);
		mac->bss_configured = false;
		err = mt6628_sdio_set(mac->dev, 0x18, command, sizeof(command));
		ret = ret ?: err;
		mac->associated = false;
		err = mt6628_mac_release_channel(mac);
		ret = ret ?: err;
		if (ret)
			mt6628_sdio_fault(mac->dev, ret);
		ret = 0;
	}
out:
	mutex_unlock(&mac->mutex);
	return ret;
}

static void mt6628_mac_bss_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
				  struct ieee80211_bss_conf *info, u64 changed)
{
	struct mt6628_mac *mac = hw->priv;
	int ret = 0;

	mutex_lock(&mac->mutex);
	if (mac->associated && !vif->cfg.assoc)
		mac->rx_generation++;
	if (mac->peer_created && (vif->cfg.assoc || mac->bss_configured)) {
		mac->basic = mt6628_mac_rates(info->basic_rates);
		ret = mt6628_mac_bss(mac, vif->cfg.assoc);
		if (!ret) {
			mac->bss_configured = vif->cfg.assoc;
			if (!vif->cfg.assoc)
				mac->associated = false;
		}
		if (!ret && vif->cfg.assoc && !mac->associated) {
			ret = mt6628_mac_peer(mac, info->bssid, 2);
			if (!ret)
				ret = mt6628_mac_release_channel(mac);
			mac->associated = !ret;
		}
	}
	mutex_unlock(&mac->mutex);
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
}

static int mt6628_mac_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt6628_mac *mac = hw->priv;

	if (vif->type != NL80211_IFTYPE_STATION || vif->p2p || mac->vif ||
	    !ether_addr_equal(vif->addr, mac->dev->mac_addr))
		return -EOPNOTSUPP;
	mac->vif = vif;
	vif->offload_flags |= IEEE80211_OFFLOAD_ENCAP_ENABLED;
	return 0;
}

static void mt6628_mac_remove_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt6628_mac *mac = hw->priv;

	cancel_work_sync(&mac->tx_work);
	mutex_lock(&mac->mutex);
	mac->vif = NULL;
	mutex_unlock(&mac->mutex);
}

static void mt6628_mac_flush(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			     u32 queues, bool drop)
{
	struct mt6628_mac *mac = hw->priv;
	struct sk_buff *skb;

	if (drop) {
		while ((skb = skb_dequeue(&mac->tx)))
			ieee80211_free_txskb(hw, skb);
	}
	flush_work(&mac->tx_work);
	mutex_lock(&mac->mutex);
	mt6628_sdio_drain(mac->dev);
	mutex_unlock(&mac->mutex);
}

static int mt6628_mac_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	struct mt6628_mac *mac = hw->priv;
	struct cfg80211_chan_def *def = &hw->conf.chandef;
	int ret = 0;

	if (!def->chan || def->chan->band != NL80211_BAND_2GHZ ||
	    (def->width != NL80211_CHAN_WIDTH_20 && def->width != NL80211_CHAN_WIDTH_20_NOHT) ||
	    def->chan->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_20MHZ))
		return -EOPNOTSUPP;
	mutex_lock(&mac->mutex);
	if (mac->peer_created && mac->channel != def->chan->hw_value) {
		mutex_unlock(&mac->mutex);
		return -EOPNOTSUPP;
	}
	mac->channel = def->chan->hw_value;
	if (changed & IEEE80211_CONF_CHANGE_POWER)
		ret = mt6628_mac_policy(mac);
	mutex_unlock(&mac->mutex);
	return ret;
}

static int mt6628_mac_frag(struct ieee80211_hw *hw, int radio_idx, u32 value)
{
	return value == (u32)-1 ? 0 : -EOPNOTSUPP;
}

static int mt6628_mac_scan_send(struct mt6628_mac *mac, unsigned int mask, bool active)
{
	struct cfg80211_scan_request *req = mac->scan_request;
	u8 command[710] = {};
	unsigned int i, count = 0;
	int ret;

	if (mac->scan_next == 256)
		mac->scan_next = 0;
	mac->scan_sequence = mac->scan_next++;
	command[0] = mac->scan_sequence;
	command[2] = active;
	command[3] = BIT(0);
	if (active) {
		command[3] = req->ssids[0].ssid_len ? BIT(2) : BIT(0);
		command[4] = req->ssids[0].ssid_len;
		memcpy(command + 8, req->ssids[0].ssid, command[4]);
	}
	command[42] = 4;
	for (i = 0; i < 14; i++) {
		if (!(mask & BIT(i)))
			continue;
		command[44 + count * 2] = 1;
		command[45 + count++ * 2] = i + 1;
	}
	command[43] = count;
	put_unaligned_le16(req->ie_len, command + 108);
	memcpy(command + 110, req->ie, req->ie_len);
	WRITE_ONCE(mac->scan_done, false);
	ret = mt6628_sdio_set(mac->dev, 0x1e, command, 110 + req->ie_len);
	if (!ret) {
		mac->scan_deadline = jiffies + msecs_to_jiffies(15000);
		mod_delayed_work(system_percpu_wq, &mac->scan_timeout, msecs_to_jiffies(15000));
	}
	return ret;
}

static void mt6628_mac_scan_finish(struct mt6628_mac *mac, bool aborted)
{
	struct cfg80211_scan_info info = { .aborted = aborted };

	if (!mac->scan_request)
		return;
	mac->scan_request = NULL;
	mac->scan_passive = 0;
	cancel_delayed_work(&mac->scan_timeout);
	ieee80211_scan_completed(mac->hw, &info);
}

static void mt6628_mac_scan_work(struct work_struct *work)
{
	struct mt6628_mac *mac = container_of(work, struct mt6628_mac, scan_work);
	unsigned int passive;
	int ret = 0;

	mutex_lock(&mac->mutex);
	if (mac->scan_request && READ_ONCE(mac->scan_done)) {
		passive = mac->scan_passive;
		mac->scan_passive = 0;
		if (passive)
			ret = mt6628_mac_scan_send(mac, passive, false);
		else
			mt6628_mac_scan_finish(mac, false);
		if (ret)
			mt6628_mac_scan_finish(mac, true);
	}
	mutex_unlock(&mac->mutex);
}

static void mt6628_mac_scan_cancel(struct mt6628_mac *mac)
{
	u8 command[4] = { mac->scan_sequence };
	int ret;

	if (!mac->scan_request)
		return;
	ret = mt6628_sdio_set(mac->dev, 0x1f, command, sizeof(command));
	mt6628_mac_scan_finish(mac, true);
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
}

static void mt6628_mac_scan_timeout(struct work_struct *work)
{
	struct mt6628_mac *mac = container_of(to_delayed_work(work), struct mt6628_mac, scan_timeout);

	mutex_lock(&mac->mutex);
	if (mac->scan_request && !READ_ONCE(mac->scan_done)) {
		unsigned long now = jiffies;

		if (time_before(now, mac->scan_deadline)) {
			mod_delayed_work(system_percpu_wq, &mac->scan_timeout,
					 mac->scan_deadline - now);
		} else {
			mt6628_mac_scan_cancel(mac);
			mt6628_sdio_fault(mac->dev, -ETIMEDOUT);
		}
	}
	mutex_unlock(&mac->mutex);
}

static int mt6628_mac_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			   struct ieee80211_scan_request *request)
{
	struct mt6628_mac *mac = hw->priv;
	struct cfg80211_scan_request *req = &request->req;
	unsigned int mask = 0, active, passive, i, channel;
	int ret = 0;

	if (req->n_ssids > 1 || req->ie_len > 600 || !req->n_channels ||
	    req->n_channels > 14 || (req->n_ssids && req->ssids[0].ssid_len > 32))
		return -EINVAL;
	if ((req->flags & ~(NL80211_SCAN_FLAG_FLUSH | NL80211_SCAN_FLAG_COLOCATED_6GHZ)) ||
	    req->duration || req->duration_mandatory || req->no_cck || req->scan_6ghz ||
	    !is_broadcast_ether_addr(req->bssid))
		return -EOPNOTSUPP;
	mutex_lock(&mac->mutex);
	if (!mac->started || mac->scan_request || mac->peer_created) {
		ret = -EBUSY;
		goto out;
	}
	for (i = 0; i < req->n_channels; i++) {
		for (channel = 0; channel < 14; channel++)
			if (req->channels[i] == &mac->channels[channel])
				break;
		if (channel == 14 || !(mac->channel_mask & BIT(channel))) {
			ret = -EINVAL;
			goto out;
		}
		mask |= BIT(channel);
	}
	active = req->n_ssids ? mask & ~mac->passive_mask : 0;
	passive = mask & ~active;
	mac->scan_request = req;
	mac->scan_passive = active ? passive : 0;
	ret = mt6628_mac_scan_send(mac, active ? active : passive, !!active);
	if (ret) {
		mac->scan_request = NULL;
		mac->scan_passive = 0;
	}
out:
	mutex_unlock(&mac->mutex);
	return ret;
}

static void mt6628_mac_cancel_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt6628_mac *mac = hw->priv;

	mutex_lock(&mac->mutex);
	mt6628_mac_scan_cancel(mac);
	mutex_unlock(&mac->mutex);
}

static u64 mt6628_mac_multicast(struct ieee80211_hw *hw, struct netdev_hw_addr_list *list)
{
	struct mt6628_mac *mac = hw->priv;
	struct netdev_hw_addr *address;
	unsigned long flags;
	unsigned int count = 0;

	spin_lock_irqsave(&mac->tx.lock, flags);
	memset(mac->multicast, 0, sizeof(mac->multicast));
	netdev_hw_addr_list_for_each(address, list) {
		if (count == 32)
			break;
		ether_addr_copy(mac->multicast + 8 + 6 * count++, address->addr);
	}
	put_unaligned_le32(count, mac->multicast);
	spin_unlock_irqrestore(&mac->tx.lock, flags);
	return netdev_hw_addr_list_count(list) > 32;
}

static void mt6628_mac_filter(struct ieee80211_hw *hw, unsigned int changed,
			      unsigned int *total, u64 multicast)
{
	struct mt6628_mac *mac = hw->priv;
	u8 list[200], filter[4] = {};
	unsigned long flags;
	int ret;

	*total = 0;
	if (multicast) {
		mt6628_sdio_fault(mac->dev, -ENOSPC);
		return;
	}
	spin_lock_irqsave(&mac->tx.lock, flags);
	memcpy(list, mac->multicast, sizeof(list));
	spin_unlock_irqrestore(&mac->tx.lock, flags);
	filter[0] = get_unaligned_le32(list) ? 0x0b : 9;
	mutex_lock(&mac->mutex);
	ret = mt6628_sdio_set(mac->dev, 0xc3, list, sizeof(list));
	if (!ret)
		ret = mt6628_sdio_set(mac->dev, 0x0b, filter, sizeof(filter));
	mutex_unlock(&mac->mutex);
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
}

static int mt6628_mac_start(struct ieee80211_hw *hw)
{
	struct mt6628_mac *mac = hw->priv;
	u8 command[4] = { 0, 1 };
	unsigned long flags;
	int ret = atomic_read(&mac->dev->error);

	if (ret)
		return ret;
	mutex_lock(&mac->mutex);
	ret = mt6628_mac_policy(mac);
	if (!ret)
		ret = mt6628_sdio_set(mac->dev, 0x15, command, sizeof(command));
	mutex_unlock(&mac->mutex);
	if (ret)
		return ret;
	spin_lock_irqsave(&mac->tx.lock, flags);
	mac->rx_generation++;
	mac->started = true;
	spin_unlock_irqrestore(&mac->tx.lock, flags);
	return 0;
}

static void mt6628_mac_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct mt6628_mac *mac = hw->priv;
	struct sk_buff *skb;
	unsigned long flags;
	u8 command[4] = {};
	int ret;

	wiphy_work_flush(hw->wiphy, &mac->key_work);
	spin_lock_irqsave(&mac->tx.lock, flags);
	mac->rx_generation++;
	mac->started = false;
	spin_unlock_irqrestore(&mac->tx.lock, flags);
	cancel_work_sync(&mac->tx_work);
	mutex_lock(&mac->mutex);
	mt6628_mac_scan_cancel(mac);
	ret = mt6628_sdio_set(mac->dev, 0x15, command, sizeof(command));
	mutex_unlock(&mac->mutex);
	cancel_delayed_work_sync(&mac->scan_timeout);
	cancel_work_sync(&mac->scan_work);
	sdio_claim_host(mac->dev->func);
	sdio_release_host(mac->dev->func);
	wiphy_work_cancel(hw->wiphy, &mac->rx_work);
	skb_queue_purge(&mac->rx);
	while ((skb = skb_dequeue(&mac->tx)))
		ieee80211_free_txskb(hw, skb);
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
}

static int mt6628_mac_delete_key(struct mt6628_mac *mac, unsigned int slot)
{
	u8 command[64] = {};
	int ret;

	if (!(mac->pending_keys & BIT(slot)))
		return 0;
	ret = mt6628_sdio_drain(mac->dev);
	if (ret)
		return ret;
	if (slot == 4) {
		command[2] = 1;
		memcpy(command + 4, mac->pending_ptk, ETH_ALEN);
		eth_zero_addr(mac->pending_ptk);
	} else {
		memset(command + 4, 0xff, ETH_ALEN);
		command[12] = slot;
	}
	mac->pending_keys &= ~BIT(slot);
	mac->rx_generation++;
	ret = mt6628_sdio_set(mac->dev, 8, command, sizeof(command));
	mac->rx_generation++;
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
	return ret;
}

static void mt6628_mac_key_work(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct mt6628_mac *mac = container_of(work, struct mt6628_mac, key_work);
	unsigned int i;

	mutex_lock(&mac->mutex);
	for (i = 0; i < ARRAY_SIZE(mac->keys); i++)
		mt6628_mac_delete_key(mac, i);
	mutex_unlock(&mac->mutex);
}

static int mt6628_mac_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			      struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			      struct ieee80211_key_conf *key)
{
	struct mt6628_mac *mac = hw->priv;
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	unsigned int slot = pairwise ? 4 : key->keyidx;
	u8 command[64] = {};
	int ret = 0;

	if (cmd != SET_KEY && cmd != DISABLE_KEY)
		return -EINVAL;
	if (key->cipher != WLAN_CIPHER_SUITE_CCMP || key->keylen != 16 ||
	    (unsigned int)key->keyidx > 3 || vif->valid_links ||
	    key->link_id < -1 || key->link_id > 0 || (pairwise && key->link_id != -1) ||
	    (key->flags & ~IEEE80211_KEY_FLAG_PAIRWISE) ||
	    (pairwise ? key->keyidx || !sta : sta != NULL))
		return cmd == DISABLE_KEY ? 0 : -EOPNOTSUPP;
	mutex_lock(&mac->mutex);
	if (cmd == DISABLE_KEY) {
		if (mac->keys[slot] != key)
			goto out;
		mac->rx_generation++;
		mac->keys[slot] = NULL;
		mac->pending_keys |= BIT(slot);
		if (pairwise)
			ether_addr_copy(mac->pending_ptk, sta->addr);
		wiphy_work_queue(hw->wiphy, &mac->key_work);
		goto out;
	}
	if (!mac->started || !mac->associated || !mac->peer_created || vif != mac->vif) {
		ret = -ENOTCONN;
		goto out;
	}
	if (pairwise && !ether_addr_equal(sta->addr, mac->bssid)) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	if (mac->keys[slot]) {
		ret = -EBUSY;
		goto out;
	}
	ret = atomic_read(&mac->dev->error);
	if (ret)
		goto out;
	if (pairwise && (mac->pending_keys & BIT(4)) &&
	    !ether_addr_equal(mac->pending_ptk, sta->addr)) {
		ret = mt6628_mac_delete_key(mac, 4);
		if (ret)
			goto out;
	}
	ret = mt6628_sdio_drain(mac->dev);
	if (ret)
		goto out;
	command[0] = 1;
	command[1] = pairwise;
	command[2] = pairwise;
	if (pairwise)
		memcpy(command + 4, sta->addr, ETH_ALEN);
	else
		memset(command + 4, 0xff, ETH_ALEN);
	command[11] = 4;
	command[12] = key->keyidx;
	command[13] = 16;
	memcpy(command + 16, key->key, 16);
	mac->rx_generation++;
	ret = mt6628_sdio_set(mac->dev, 8, command, sizeof(command));
	mac->rx_generation++;
	if (ret)
		mt6628_sdio_fault(mac->dev, ret);
	if (!ret) {
		mac->keys[slot] = key;
		key->hw_key_idx = slot;
		mac->pending_keys &= ~BIT(slot);
		if (pairwise)
			eth_zero_addr(mac->pending_ptk);
	}
out:
	mutex_unlock(&mac->mutex);
	memzero_explicit(command, sizeof(command));
	return ret;
}

static const struct ieee80211_ops mt6628_mac_ops = {
	.tx = mt6628_mac_tx,
	.wake_tx_queue = ieee80211_handle_wake_tx_queue,
	.start = mt6628_mac_start,
	.stop = mt6628_mac_stop,
	.set_key = mt6628_mac_set_key,
	.add_interface = mt6628_mac_add_interface,
	.remove_interface = mt6628_mac_remove_interface,
	.flush = mt6628_mac_flush,
	.config = mt6628_mac_config,
	.bss_info_changed = mt6628_mac_bss_changed,
	.sta_state = mt6628_mac_sta_state,
	.set_frag_threshold = mt6628_mac_frag,
	.set_rts_threshold = mt6628_mac_frag,
	.hw_scan = mt6628_mac_scan,
	.cancel_hw_scan = mt6628_mac_cancel_scan,
	.prepare_multicast = mt6628_mac_multicast,
	.configure_filter = mt6628_mac_filter,
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
};

static void mt6628_mac_rx_status(struct sk_buff *skb, u8 channel, u8 rcpi, u8 rate)
{
	struct ieee80211_rx_status *status = IEEE80211_SKB_RXCB(skb);

	memset(status, 0, sizeof(*status));
	status->band = NL80211_BAND_2GHZ;
	status->freq = ieee80211_channel_to_frequency(channel, NL80211_BAND_2GHZ);
	status->rate_idx = rate;
	if (rcpi > 220)
		status->flag |= RX_FLAG_NO_SIGNAL_VAL;
	else
		status->signal = (int)rcpi / 2 - 110;
}

static bool mt6628_mac_control_valid(struct mt6628_mac *mac,
				    const struct mt6628_wmt_rx_entry *entry,
				    const u8 *packet, size_t len, u32 generation)
{
	unsigned int offset;

	if (!mac->started || !mac->associated || !mac->peer_created || !mac->vif ||
	    atomic_read(&mac->dev->error) ||
	    generation != mac->rx_generation || len < 12 ||
	    packet[8] != mac->peer || packet[10] != mac->channel || packet[11])
		return false;
	offset = 12 + (packet[4] & 3);
	if (get_unaligned_le16(entry->rfb + 50) & 0x4000) {
		unsigned int index = (entry->rx[17] >> 4) & 3;
		bool group;

		if (len < offset + ETH_HLEN || (entry->rx[14] & 0x3e) ||
		    (entry->rx[17] & 15) != 4)
			return false;
		group = packet[offset] & 1;
		if (group)
			return mac->keys[index];
		return !index && mac->keys[4];
	}
	return len >= offset + ETH_HLEN &&
		get_unaligned_be16(packet + offset + 12) == ETH_P_PAE &&
		ether_addr_equal(packet + offset, mac->dev->mac_addr) &&
		ether_addr_equal(packet + offset + 6, mac->bssid);
}

static void mt6628_mac_rx_work(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct mt6628_mac *mac = container_of(work, struct mt6628_mac, rx_work);
	struct sk_buff *input;

	while ((input = skb_dequeue(&mac->rx))) {
		const struct mt6628_wmt_rx_entry *entry = (void *)input->data;
		const u8 *packet = input->data + sizeof(*entry);
		struct sk_buff *skb;
		u32 generation;
		int size;

		memcpy(&generation, input->cb, sizeof(generation));
		if (!mt6628_mac_control_valid(mac, entry, packet,
					      input->len - sizeof(*entry), generation))
			goto next;
		skb = dev_alloc_skb(MT6628_NATIVE_MAX_FRAME);
		if (!skb)
			goto next;
		size = mt6628_wmt_rx_frame(entry, packet, input->len - sizeof(*entry),
					 mac->bssid, mac->dev->mac_addr, skb->data,
					 MT6628_NATIVE_MAX_FRAME);
		if (size < 0) {
			dev_kfree_skb_any(skb);
			goto next;
		}
		skb_put(skb, size);
		mt6628_mac_rx_status(skb, packet[10], packet[9], 0);
		if (get_unaligned_le16(entry->rfb + 50) & 0x4000)
			IEEE80211_SKB_RXCB(skb)->flag |= RX_FLAG_DECRYPTED | RX_FLAG_MIC_STRIPPED;
		local_bh_disable();
		ieee80211_rx_napi(mac->hw, NULL, skb, NULL);
		local_bh_enable();
next:
		dev_kfree_skb_any(input);
	}
}

u32 mt6628_radio_rx_generation(struct mt6628_dev *dev)
{
	return dev->wiphy ? READ_ONCE(mt6628_mac_priv(dev->wiphy)->rx_generation) : 0;
}

void mt6628_radio_control_rx(struct mt6628_dev *dev,
			     const struct mt6628_wmt_rx_entry *entry,
			     const u8 *packet, size_t len, u32 generation)
{
	struct mt6628_mac *mac;
	struct sk_buff *skb;

	if (!dev->wiphy)
		return;
	mac = mt6628_mac_priv(dev->wiphy);
	if (!READ_ONCE(mac->started) || skb_queue_len(&mac->rx) >= 64)
		return;
	skb = alloc_skb(sizeof(*entry) + len, GFP_KERNEL);
	if (!skb)
		return;
	memcpy(skb->cb, &generation, sizeof(generation));
	skb_put_data(skb, entry, sizeof(*entry));
	skb_put_data(skb, packet, len);
	skb_queue_tail(&mac->rx, skb);
	wiphy_work_queue(dev->wiphy, &mac->rx_work);
}

void mt6628_radio_rx(struct mt6628_dev *dev, const u8 *packet, size_t len)
{
	struct mt6628_native_rx view;
	struct mt6628_mac *mac;
	struct sk_buff *skb;
	u8 event;

	if (!dev->wiphy || len < 8)
		return;
	mac = mt6628_mac_priv(dev->wiphy);
	if ((get_unaligned_le16(packet + 2) & 3) == 1) {
		if (READ_ONCE(mac->started) && packet[4] == 0x15 && len == 12 &&
		    packet[8] == READ_ONCE(mac->scan_sequence)) {
			WRITE_ONCE(mac->scan_done, true);
			queue_work(system_percpu_wq, &mac->scan_work);
			return;
		}
		/* Event consistency */
		event = smp_load_acquire(&mac->event_id);
		if (!event || packet[4] != event)
			return;
		if (event == 0x17 && (len == 12 || len == 24) && packet[8] == mac->event_token)
			mac->event_result = packet[9] <= 5 ? packet[9] : -EPROTO;
		else if (event == 0x18 && len == 20 && !packet[8] && packet[9] == mac->event_token) {
			mac->event_result = packet[10] || packet[11] != mac->channel ||
				packet[12] || packet[13] != 1 || packet[14] ||
				get_unaligned_le32(packet + 16) <= 10 ||
				get_unaligned_le32(packet + 16) > 2000 ? -EPROTO : 0;
			if (!mac->event_result)
				mac->channel_deadline = jiffies +
					msecs_to_jiffies(get_unaligned_le32(packet + 16) - 10);
		} else if (event == 0x13 && len == 20 && packet[14] == mac->event_token &&
			 packet[5] == mac->event_sequence && ether_addr_equal(packet + 8, mac->bssid))
			mac->event_result = packet[15] ? -EPROTO : 0;
		else
			return;
		complete(&mac->event_done);
		return;
	}
	if (!READ_ONCE(mac->started) || mt6628_native_rx_packet(packet, len, &view))
		return;
	skb = dev_alloc_skb(view.len);
	if (!skb)
		return;
	skb_put_data(skb, view.frame, view.len);
	mt6628_mac_rx_status(skb, view.channel, view.rcpi, view.rate_idx);
	ieee80211_rx_irqsafe(mac->hw, skb);
}

void mt6628_radio_error(struct mt6628_dev *dev, int error)
{
	struct mt6628_mac *mac;

	if (!dev->wiphy)
		return;
	mac = mt6628_mac_priv(dev->wiphy);
	mac->event_result = error;
	complete(&mac->event_done);
	ieee80211_stop_queues(mac->hw);
}

int mt6628_radio_register(struct mt6628_dev *dev)
{
	static const int bitrates[] = { 10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540 };
	static const u32 ciphers[] = { WLAN_CIPHER_SUITE_CCMP };
	struct ieee80211_hw *hw;
	struct mt6628_mac *mac;
	int i, ret, power = -128;

	if (!dev->factory.set_tx_power || dev->factory.edge_enabled ||
	    (dev->factory.enable_5ghz && !dev->info.hw_5ghz_disabled))
		return -EOPNOTSUPP;
	hw = ieee80211_alloc_hw(sizeof(*mac), &mt6628_mac_ops);
	if (!hw)
		return -ENOMEM;
	mac = hw->priv;
	mac->dev = dev;
	mac->hw = hw;
	mac->tx_next = 1;
	memcpy(mac->alpha2, "00", 2);
	mutex_init(&mac->mutex);
	init_completion(&mac->event_done);
	skb_queue_head_init(&mac->tx);
	skb_queue_head_init(&mac->rx);
	wiphy_work_init(&mac->rx_work, mt6628_mac_rx_work);
	wiphy_work_init(&mac->key_work, mt6628_mac_key_work);
	INIT_WORK(&mac->tx_work, mt6628_mac_tx_work);
	INIT_WORK(&mac->scan_work, mt6628_mac_scan_work);
	INIT_DELAYED_WORK(&mac->scan_timeout, mt6628_mac_scan_timeout);
	power = mt6628_radio_max_power(&dev->factory);
	for (i = 0; i < 14; i++) {
		mac->channels[i].band = NL80211_BAND_2GHZ;
		mac->channels[i].hw_value = i + 1;
		mac->channels[i].center_freq = ieee80211_channel_to_frequency(i + 1, NL80211_BAND_2GHZ);
		mac->channels[i].max_power = power;
	}
	for (i = 0; i < ARRAY_SIZE(bitrates); i++)
		mac->rates[i].bitrate = bitrates[i];
	mac->band.channels = mac->channels;
	mac->band.n_channels = ARRAY_SIZE(mac->channels);
	mac->band.bitrates = mac->rates;
	mac->band.n_bitrates = ARRAY_SIZE(mac->rates);
	hw->wiphy->bands[NL80211_BAND_2GHZ] = &mac->band;
	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	hw->wiphy->reg_notifier = mt6628_mac_regulatory;
	hw->wiphy->max_scan_ssids = 1;
	hw->wiphy->max_scan_ie_len = 600;
	hw->wiphy->cipher_suites = ciphers;
	hw->wiphy->n_cipher_suites = ARRAY_SIZE(ciphers);
	hw->queues = 1;
	hw->max_mtu = ETH_DATA_LEN;
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, SW_CRYPTO_CONTROL);
	ieee80211_hw_set(hw, SUPPORTS_TX_ENCAP_OFFLOAD);
	SET_IEEE80211_PERM_ADDR(hw, dev->mac_addr);
	SET_IEEE80211_DEV(hw, &dev->func->dev);
	ret = ieee80211_register_hw(hw);
	if (ret) {
		ieee80211_free_hw(hw);
		return ret;
	}
	sdio_claim_host(dev->func);
	dev->wiphy = hw->wiphy;
	sdio_release_host(dev->func);
	return 0;
}

void mt6628_radio_unregister(struct mt6628_dev *dev)
{
	struct ieee80211_hw *hw;

	if (!dev->wiphy)
		return;
	hw = wiphy_to_ieee80211_hw(dev->wiphy);
	ieee80211_unregister_hw(hw);
	sdio_claim_host(dev->func);
	dev->wiphy = NULL;
	sdio_release_host(dev->func);
	ieee80211_free_hw(hw);
}
