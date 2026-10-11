// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/etherdevice.h>
#include <linux/jiffies.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_net.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/if_vlan.h>
#include <linux/workqueue.h>
#include <net/cfg80211.h>

#include "boot.h"
#include "protocol.h"

#define SC2331_WLAN_NAME "sprd/sc2331-wlan.bin"
#define SC2331_WLAN_ADDRESS 0x00100000
#define SC2331_WLAN_SIZE 786432
#define SC2331_CHUNK_SIZE 131072
#define SC2331_CALIB_NAME "sprd/sc2331-calibration.bin"
#define SC2331_CALIB_SIZE 4096
#define SC2331_CALIB_OFFSET 616
#define SC2331_CALIB_REPLY_MIN 3008
#define SC2331_RUNTIME_RX_SIZE 65536

struct sc2331_wifi {
	struct sdio_func *func;
	struct sc2331_boot *boot;
	struct wireless_dev wdev;
	struct ieee80211_supported_band band;
	struct ieee80211_channel channels[14];
	struct mutex lock;
	struct delayed_work rx_work;
	struct work_struct tx_work;
	struct sk_buff_head tx_queue;
	struct cfg80211_scan_request *scan;
	unsigned long scan_deadline;
	u32 sequence;
	u32 packet_sequence;
	unsigned long connect_deadline;
	u8 bssid[ETH_ALEN];
	u8 *data;
	bool stopping;
	bool scan_done;
	bool connecting;
	bool connected;
	unsigned int scan_frames;
};

static int sc2331_sdio_length(struct sdio_func *func, unsigned int channel)
{
	unsigned int address = (0x200 + channel) * 4;
	int error = 0, length = 0;
	unsigned int i;
	u8 byte;

	for (i = 0; i < 3; i++) {
		byte = sdio_f0_readb(func, address + i, &error);
		if (error)
			return error;
		length |= byte << (8 * i);
	}
	return length;
}

static int sc2331_sdio_runtime_tx(struct sdio_func *func, u8 *data, size_t len)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(1000);
	unsigned int channel;
	int ret, available;
	u8 busy;

	if (!len || len > 64512 || len % 1024)
		return -EINVAL;
	do {
		sdio_claim_host(func);
		busy = sdio_f0_readb(func, 0x840, &ret);
		if (!ret) {
			for (channel = 0; channel < 3; channel++) {
				if (busy & BIT(channel))
					continue;
				if (channel == 2) {
					available = sc2331_sdio_length(func, channel);
					if (available < 0) {
						ret = available;
						break;
					}
					if ((size_t)available < len)
						continue;
				}
				ret = sdio_writesb(func, channel, data, len);
				sdio_release_host(func);
				return ret;
			}
		}
		sdio_release_host(func);
		if (ret)
			return ret;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int sc2331_sdio_runtime_records(u8 *data, size_t len, u8 command,
				       void (*event)(void *, const struct sc2331_record *),
				       void *context)
{
	struct sc2331_record record;
	size_t offset = 8, used;
	int ret, result = -EAGAIN;
	unsigned int i;

	if (len < 12)
		return -EMSGSIZE;
	while (offset < len) {
		ret = sc2331_decode_record(data + offset, len - offset, &record, &used);
		if (ret < 0)
			return ret;
		if (!ret)
			break;
		if (record.type == 1) {
			if (record.id != command) {
				offset += used;
				continue;
			}
			if (result != -EAGAIN)
				return -EPROTO;
			if (command == 0x21) {
				if (record.len != 8)
					return -EMSGSIZE;
				if (record.data[0] != 2 || record.data[1] != 0x28 ||
				    get_unaligned_le16(record.data + 2) != 4)
					return -EPROTO;
				i = 4;
			} else {
				if (record.len != 4)
					return -EMSGSIZE;
				i = 0;
			}
			result = 0;
			for (; i < record.len; i++)
				if (record.data[i])
					result = -EIO;
		} else if ((record.type == 0 || record.type == 2) && event) {
			event(context, &record);
		}
		offset += used;
	}
	return result;
}

static int sc2331_sdio_runtime_response(struct sdio_func *func, u8 *data,
				       u8 command, unsigned int timeout_ms,
			void (*event)(void *, const struct sc2331_record *), void *context)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	unsigned int channel;
	int ret, length;
	u8 ready;

	do {
		for (channel = 8; channel <= 9; channel++) {
			sdio_claim_host(func);
			ready = sdio_f0_readb(func, 0x841, &ret);
			if (!ret && (ready & BIT(channel - 8))) {
				length = sc2331_sdio_length(func, channel);
				if (length < 0)
					ret = length;
				else if (length < 12 || length > SC2331_RUNTIME_RX_SIZE)
					ret = -EMSGSIZE;
				else
					ret = sdio_readsb(func, data, channel, length);
				sdio_release_host(func);
				if (ret)
					return ret;
				ret = sc2331_sdio_runtime_records(data, length, command,
								  event, context);
				if (ret != -EAGAIN) {
					dev_info(&func->dev, "Runtime reply: channel %u, %d bytes, first record %u/%u/%u\n",
						 channel, length, data[8], data[9],
						 get_unaligned_le16(data + 10));
					return ret;
				}
			} else {
				sdio_release_host(func);
				if (ret)
					return ret;
			}
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int sc2331_sdio_wait_tx(struct sdio_func *func, size_t len)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(1000);
	int available;

	do {
		sdio_claim_host(func);
		available = sc2331_sdio_length(func, 3);
		sdio_release_host(func);
		if (available < 0)
			return available;
		if ((size_t)available >= len)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int sc2331_sdio_read(struct sdio_func *func, u8 *data, size_t minimum,
			    size_t capacity, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	int error, length;
	u8 ready;

	do {
		sdio_claim_host(func);
		ready = sdio_f0_readb(func, 0x841, &error);
		if (!error && (ready & BIT(4))) {
			length = sc2331_sdio_length(func, 12);
			if (length < 0)
				error = length;
			else if ((size_t)length < minimum || (size_t)length > capacity)
				error = -EMSGSIZE;
			else
				error = sdio_readsb(func, data, 12, length);
			sdio_release_host(func);
			if (error)
				return error;
			return length;
		}
		sdio_release_host(func);
		if (error)
			return error;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int sc2331_sdio_ack(struct sdio_func *func, u8 *data)
{
	int length = sc2331_sdio_read(func, data, 6, 6, 1000);

	if (length < 0)
		return length;
	dev_info(&func->dev, "SDIO download reply: %*ph\n", 6, data);
	return sc2331_decode_sdio_ack(data, length);
}

static int sc2331_sdio_calibrate(struct sdio_func *func, u8 *data)
{
	const struct firmware *firmware;
	u8 version, calibrated;
	int ret;

	ret = request_firmware(&firmware, SC2331_CALIB_NAME, &func->dev);
	if (ret)
		return ret;
	if (firmware->size != SC2331_CALIB_SIZE ||
	    firmware->data[SC2331_CALIB_OFFSET + 1] > 1) {
		ret = -EINVAL;
		goto release;
	}
	memcpy(data, firmware->data, SC2331_CALIB_SIZE);
	version = data[SC2331_CALIB_OFFSET];
	calibrated = data[SC2331_CALIB_OFFSET + 1];
	sdio_claim_host(func);
	ret = sdio_writesb(func, 0, data, SC2331_CALIB_SIZE);
	sdio_release_host(func);
	if (ret)
		goto release;
	if (!calibrated) {
		ret = sc2331_sdio_read(func, data, SC2331_CALIB_REPLY_MIN,
				       SC2331_CALIB_SIZE, 2000);
		if (ret < 0)
			goto release;
		dev_info(&func->dev, "WLAN calibration reply: %d bytes, version %u, calibrated %u\n",
			 ret, data[0], data[1]);
		if (data[0] != version || data[1] != 1) {
			ret = -EPROTO;
			goto release;
		}
	}
	ret = 0;
	dev_info(&func->dev, "SC2331 WLAN calibration applied\n");
release:
	release_firmware(firmware);
	return ret;
}

static int sc2331_sdio_command(struct sdio_func *func, u8 *data,
			       size_t capacity, u16 command,
			       const u8 *payload, size_t len)
{
	u8 *bounce;
	ssize_t encoded;
	size_t offset, chunk;
	int ret;

	encoded = sc2331_encode_sdio(data, capacity, command, payload, len);
	if (encoded < 0)
		return encoded;
	ret = sc2331_sdio_wait_tx(func, encoded);
	if (ret)
		return ret;
	bounce = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;
	sdio_claim_host(func);
	for (offset = 0; offset < (size_t)encoded; offset += chunk) {
		chunk = min_t(size_t, encoded - offset, PAGE_SIZE);
		chunk = min_t(size_t, chunk, func->card->host->max_req_size);
		memcpy(bounce, data + offset, chunk);
		ret = sdio_writesb(func, 3, bounce, chunk);
		if (ret)
			break;
	}
	sdio_release_host(func);
	if (!ret)
		ret = sc2331_sdio_ack(func, bounce);
	kfree(bounce);
	return ret;
}

static void sc2331_scan_finish(struct sc2331_wifi *wifi, bool aborted)
{
	struct cfg80211_scan_info info = { .aborted = aborted };
	struct cfg80211_scan_request *request = wifi->scan;

	if (!request)
		return;
	wifi->scan = NULL;
	cfg80211_scan_done(request, &info);
	dev_info(&wifi->func->dev, "SC2331 scan completed: %u frames, aborted %u\n",
		 wifi->scan_frames, aborted);
}

static void sc2331_scan_event(void *context, const struct sc2331_record *record)
{
	struct sc2331_wifi *wifi = context;
	struct sc2331_scan_frame frame;
	struct ieee80211_channel *channel;
	struct cfg80211_bss *bss;
	struct ieee80211_mgmt *mgmt;
	int frequency, ret;

	if (!wifi->scan || record->id != 0x8a)
		return;
	ret = sc2331_decode_scan_frame(record->data, record->len, &frame);
	if (ret) {
		dev_warn_ratelimited(&wifi->func->dev, "Invalid scan event: %d\n", ret);
		return;
	}
	if (frame.done) {
		wifi->scan_done = true;
		return;
	}
	if (frame.len < offsetof(struct ieee80211_mgmt, u.beacon.variable))
		return;
	mgmt = (struct ieee80211_mgmt *)frame.data;
	if (!ieee80211_is_beacon(mgmt->frame_control) &&
	    !ieee80211_is_probe_resp(mgmt->frame_control))
		return;
	frequency = ieee80211_channel_to_frequency(frame.channel, NL80211_BAND_2GHZ);
	channel = ieee80211_get_channel(wifi->wdev.wiphy, frequency);
	if (!channel || channel->flags & IEEE80211_CHAN_DISABLED)
		return;
	bss = cfg80211_inform_bss_frame(wifi->wdev.wiphy, channel, mgmt,
					frame.len, frame.signal * 100, GFP_KERNEL);
	if (bss) {
		wifi->scan_frames++;
		cfg80211_put_bss(wifi->wdev.wiphy, bss);
	}
}

static void sc2331_connection_down(struct sc2331_wifi *wifi, u16 reason, bool local)
{
	if (wifi->connecting)
		cfg80211_connect_result(wifi->wdev.netdev, NULL, NULL, 0, NULL, 0,
					WLAN_STATUS_UNSPECIFIED_FAILURE, GFP_KERNEL);
	else if (wifi->connected)
		cfg80211_disconnected(wifi->wdev.netdev, reason, NULL, 0, local, GFP_KERNEL);
	wifi->connecting = false;
	wifi->connected = false;
	netif_carrier_off(wifi->wdev.netdev);
	skb_queue_purge(&wifi->tx_queue);
}

static void sc2331_runtime_event(void *context, const struct sc2331_record *record)
{
	struct sc2331_wifi *wifi = context;
	struct net_device *netdev = wifi->wdev.netdev;
	struct sc2331_connect_result result;
	struct sk_buff *skb;
	int ret;

	if (record->type == 2) {
		if (!wifi->connected || record->len > netdev->mtu + ETH_HLEN + VLAN_HLEN)
			return;
		skb = netdev_alloc_skb_ip_align(netdev, record->len);
		if (!skb) {
			DEV_STATS_INC(netdev, rx_dropped);
			return;
		}
		skb_put_data(skb, record->data, record->len);
		skb->protocol = eth_type_trans(skb, netdev);
		DEV_STATS_INC(netdev, rx_packets);
		DEV_STATS_ADD(netdev, rx_bytes, record->len);
		netif_rx(skb);
		return;
	}
	if (record->id == 0x8a) {
		sc2331_scan_event(wifi, record);
		return;
	}
	if (record->id == 0x81) {
		if (record->len >= 2) {
			sc2331_connection_down(wifi, get_unaligned_le16(record->data), false);
			dev_info(&wifi->func->dev, "SC2331 station disconnected\n");
		}
		return;
	}
	if (record->id != 0x80 || !wifi->connecting)
		return;
	ret = sc2331_decode_connect_result(record->data, record->len, &result);
	if (ret) {
		dev_warn_ratelimited(&wifi->func->dev, "Invalid connection event: %d\n", ret);
		return;
	}
	if (result.status) {
		sc2331_connection_down(wifi, WLAN_REASON_UNSPECIFIED, false);
		dev_info(&wifi->func->dev, "SC2331 station connection failed: %u\n", result.status);
		return;
	}
	if (!is_valid_ether_addr(result.bssid))
		return;
	ether_addr_copy(wifi->bssid, result.bssid);
	wifi->connecting = false;
	wifi->connected = true;
	cfg80211_connect_result(netdev, result.bssid, result.request_ie,
				result.request_ie_len, result.response_ie,
				result.response_ie_len, WLAN_STATUS_SUCCESS, GFP_KERNEL);
	netif_carrier_on(netdev);
	netif_wake_queue(netdev);
	dev_info(&wifi->func->dev, "SC2331 station connected\n");
}

static int sc2331_wifi_command(struct sc2331_wifi *wifi, u8 command,
			       const u8 *payload, size_t len, unsigned int timeout)
{
	ssize_t encoded;
	int ret;

	ret = sc2331_boot_wake_device(wifi->boot);
	if (ret)
		return ret;
	encoded = sc2331_encode_command(wifi->data, SC2331_RUNTIME_RX_SIZE,
					command, wifi->sequence++, payload, len);
	if (encoded < 0)
		return encoded;
	ret = sc2331_sdio_runtime_tx(wifi->func, wifi->data, encoded);
	if (ret)
		return ret;
	return sc2331_sdio_runtime_response(wifi->func, wifi->data, command,
					    timeout, sc2331_runtime_event, wifi);
}

static void sc2331_rx_work(struct work_struct *work)
{
	struct sc2331_wifi *wifi = container_of(to_delayed_work(work),
					       struct sc2331_wifi, rx_work);
	int ret;

	mutex_lock(&wifi->lock);
	if (wifi->stopping || (!wifi->scan && !wifi->connecting && !wifi->connected))
		goto unlock;
	ret = sc2331_sdio_runtime_response(wifi->func, wifi->data, 0xff, 1,
					   sc2331_runtime_event, wifi);
	if (wifi->scan && wifi->scan_done)
		sc2331_scan_finish(wifi, false);
	else if (wifi->scan && ((ret && ret != -ETIMEDOUT) ||
				time_after_eq(jiffies, wifi->scan_deadline)))
		sc2331_scan_finish(wifi, true);
	if (wifi->connecting && time_after_eq(jiffies, wifi->connect_deadline)) {
		cfg80211_connect_timeout(wifi->wdev.netdev, NULL, NULL, 0, GFP_KERNEL,
					 NL80211_TIMEOUT_UNSPECIFIED);
		wifi->connecting = false;
		dev_info(&wifi->func->dev, "SC2331 station connection timed out\n");
	}
	if (wifi->scan || wifi->connecting || wifi->connected)
		schedule_delayed_work(&wifi->rx_work, msecs_to_jiffies(20));
unlock:
	mutex_unlock(&wifi->lock);
}

static int sc2331_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
	struct sc2331_wifi *wifi = wiphy_priv(wiphy);
	u8 payload[1 + 14 + 4 + 9 * 33];
	size_t offset, ssid_start;
	unsigned int i, channel;
	int ret;

	if (request->n_channels > 14 || request->n_ssids > 9 || request->ie_len)
		return -EOPNOTSUPP;
	payload[0] = request->n_channels;
	for (i = 0; i < request->n_channels; i++) {
		channel = ieee80211_frequency_to_channel(request->channels[i]->center_freq);
		if (request->channels[i]->band != NL80211_BAND_2GHZ ||
		    channel < 1 || channel > 14)
			return -EINVAL;
		payload[1 + i] = channel;
	}
	ssid_start = request->n_channels + 5;
	offset = ssid_start;
	for (i = 0; i < request->n_ssids; i++) {
		if (request->ssids[i].ssid_len > IEEE80211_MAX_SSID_LEN)
			return -EINVAL;
		if (!request->ssids[i].ssid_len)
			continue;
		payload[offset++] = request->ssids[i].ssid_len;
		memcpy(payload + offset, request->ssids[i].ssid, request->ssids[i].ssid_len);
		offset += request->ssids[i].ssid_len;
	}
	put_unaligned_le32(offset - ssid_start, payload + request->n_channels + 1);
	mutex_lock(&wifi->lock);
	if (wifi->stopping) {
		ret = -ENODEV;
		goto unlock;
	}
	if (wifi->scan) {
		ret = -EBUSY;
		goto unlock;
	}
	wifi->scan = request;
	wifi->scan_done = false;
	wifi->scan_frames = 0;
	wifi->scan_deadline = jiffies + msecs_to_jiffies(15000);
	ret = sc2331_wifi_command(wifi, 4, payload, offset, 3000);
	if (ret) {
		wifi->scan = NULL;
		goto unlock;
	}
	dev_info(&wifi->func->dev, "SC2331 scan command accepted\n");
	if (wifi->scan_done)
		sc2331_scan_finish(wifi, false);
	else
		schedule_delayed_work(&wifi->rx_work, 0);
unlock:
	mutex_unlock(&wifi->lock);
	return ret;
}

static int sc2331_connect(struct wiphy *wiphy, struct net_device *netdev,
			  struct cfg80211_connect_params *sme)
{
	struct sc2331_wifi *wifi = wiphy_priv(wiphy);
	u8 payload[34];
	u32 values[6];
	const u8 commands[] = { 6, 5, 7, 8, 5, 9 };
	unsigned int i, channel;
	int ret;

	if (!sme->ssid || !sme->ssid_len || sme->ssid_len > IEEE80211_MAX_SSID_LEN ||
	    sme->crypto.wpa_versions != NL80211_WPA_VERSION_2 ||
	    sme->crypto.n_ciphers_pairwise != 1 || sme->crypto.n_akm_suites != 1 ||
	    sme->crypto.akm_suites[0] != WLAN_AKM_SUITE_PSK ||
	    sme->mfp == NL80211_MFP_REQUIRED ||
	    (sme->auth_type != NL80211_AUTHTYPE_OPEN_SYSTEM &&
	     sme->auth_type != NL80211_AUTHTYPE_AUTOMATIC))
		return -EOPNOTSUPP;
	values[0] = NL80211_WPA_VERSION_2;
	values[1] = 0;
	values[2] = sme->crypto.ciphers_pairwise[0] == WLAN_CIPHER_SUITE_CCMP ? 4 :
		    sme->crypto.ciphers_pairwise[0] == WLAN_CIPHER_SUITE_TKIP ? 3 : 0;
	values[3] = sme->crypto.cipher_group == WLAN_CIPHER_SUITE_CCMP ? 4 :
		    sme->crypto.cipher_group == WLAN_CIPHER_SUITE_TKIP ? 3 : 0;
	values[4] = 0;
	values[5] = 1;
	if (!values[2] || !values[3])
		return -EOPNOTSUPP;
	if (sme->channel && (sme->channel->band != NL80211_BAND_2GHZ ||
			     sme->channel->flags & IEEE80211_CHAN_DISABLED))
		return -EINVAL;
	if (sme->bssid && !is_valid_ether_addr(sme->bssid))
		return -EINVAL;
	mutex_lock(&wifi->lock);
	if (wifi->stopping || wifi->scan || wifi->connecting || wifi->connected) {
		ret = wifi->stopping ? -ENODEV : -EBUSY;
		goto unlock;
	}
	for (i = 0; i < ARRAY_SIZE(commands); i++) {
		put_unaligned_le32(values[i], payload);
		ret = sc2331_wifi_command(wifi, commands[i], payload, 4, 3000);
		if (ret)
			goto unlock;
	}
	put_unaligned_le16(sme->crypto.psk ? 32 : 0, payload);
	if (sme->crypto.psk)
		memcpy(payload + 2, sme->crypto.psk, 32);
	ret = sc2331_wifi_command(wifi, 0x18, payload, sme->crypto.psk ? 34 : 2, 3000);
	memzero_explicit(payload, sizeof(payload));
	memzero_explicit(wifi->data, SC2331_RUNTIME_RX_SIZE);
	if (ret)
		goto unlock;
	if (sme->channel) {
		channel = ieee80211_frequency_to_channel(sme->channel->center_freq);
		put_unaligned_le32(channel, payload);
		ret = sc2331_wifi_command(wifi, 0x0a, payload, 4, 3000);
		if (ret)
			goto unlock;
	}
	if (sme->bssid) {
		ret = sc2331_wifi_command(wifi, 0x0b, sme->bssid, ETH_ALEN, 3000);
		if (ret)
			goto unlock;
	}
	put_unaligned_le16(sme->ssid_len, payload);
	memcpy(payload + 2, sme->ssid, sme->ssid_len);
	wifi->connecting = true;
	wifi->connect_deadline = jiffies + msecs_to_jiffies(20000);
	ret = sc2331_wifi_command(wifi, 0x0c, payload, sme->ssid_len + 2, 3000);
	if (ret && wifi->connecting) {
		wifi->connecting = false;
		goto unlock;
	}
	ret = 0;
	schedule_delayed_work(&wifi->rx_work, 0);
unlock:
	mutex_unlock(&wifi->lock);
	return ret;
}

static int sc2331_disconnect(struct wiphy *wiphy, struct net_device *netdev, u16 reason)
{
	struct sc2331_wifi *wifi = wiphy_priv(wiphy);
	u8 payload[2];
	int ret;

	mutex_lock(&wifi->lock);
	put_unaligned_le16(reason, payload);
	ret = wifi->stopping ? -ENODEV : sc2331_wifi_command(wifi, 0x10, payload, 2, 3000);
	if (!ret)
		sc2331_connection_down(wifi, reason, true);
	mutex_unlock(&wifi->lock);
	return ret;
}

static int sc2331_add_key(struct wiphy *wiphy, struct wireless_dev *wdev,
			  int link_id, u8 key_index, bool pairwise,
			  const u8 *mac_addr, struct key_params *params)
{
	struct sc2331_wifi *wifi = wiphy_priv(wiphy);
	u8 payload[50] = {};
	u8 cipher;
	int ret;

	if (link_id != -1 || key_index > 3 || params->seq_len < 0 ||
	    params->seq_len > 8 || !params->key ||
	    (params->seq_len && !params->seq) ||
	    (pairwise && (!mac_addr || !is_valid_ether_addr(mac_addr))))
		return -EINVAL;
	if (params->cipher == WLAN_CIPHER_SUITE_CCMP && params->key_len == 16)
		cipher = 4;
	else if (params->cipher == WLAN_CIPHER_SUITE_TKIP && params->key_len == 32)
		cipher = 3;
	else
		return -EINVAL;
	if (mac_addr)
		ether_addr_copy(payload, mac_addr);
	if (params->seq_len)
		memcpy(payload + 6, params->seq, params->seq_len);
	payload[14] = pairwise;
	payload[15] = cipher;
	payload[16] = key_index;
	payload[17] = params->key_len;
	memcpy(payload + 18, params->key, params->key_len);
	mutex_lock(&wifi->lock);
	ret = wifi->stopping ? -ENODEV :
		sc2331_wifi_command(wifi, 0x0d, payload, 18 + params->key_len, 3000);
	memzero_explicit(payload, sizeof(payload));
	memzero_explicit(wifi->data, SC2331_RUNTIME_RX_SIZE);
	mutex_unlock(&wifi->lock);
	return ret;
}

static int sc2331_del_key(struct wiphy *wiphy, struct wireless_dev *wdev,
			  int link_id, u8 key_index, bool pairwise, const u8 *mac_addr)
{
	struct sc2331_wifi *wifi = wiphy_priv(wiphy);
	u8 payload[8] = { key_index };
	int ret;

	if (link_id != -1 || key_index > 3)
		return -EINVAL;
	if (mac_addr)
		ether_addr_copy(payload + 2, mac_addr);
	mutex_lock(&wifi->lock);
	ret = wifi->stopping ? -ENODEV :
		sc2331_wifi_command(wifi, 0x0e, payload, sizeof(payload), 3000);
	mutex_unlock(&wifi->lock);
	return ret;
}

static int sc2331_set_default_key(struct wiphy *wiphy, struct net_device *netdev,
				  int link_id, u8 key_index, bool unicast, bool multicast)
{
	struct sc2331_wifi *wifi = wiphy_priv(wiphy);
	u8 payload[4];
	int ret;

	if (link_id != -1 || key_index > 3)
		return -EINVAL;
	put_unaligned_le32(key_index, payload);
	mutex_lock(&wifi->lock);
	ret = wifi->stopping ? -ENODEV :
		sc2331_wifi_command(wifi, 0x0f, payload, sizeof(payload), 3000);
	mutex_unlock(&wifi->lock);
	return ret;
}

static const struct cfg80211_ops sc2331_cfg80211_ops = {
	.add_key = sc2331_add_key,
	.del_key = sc2331_del_key,
	.set_default_key = sc2331_set_default_key,
	.scan = sc2331_scan,
	.connect = sc2331_connect,
	.disconnect = sc2331_disconnect,
};

static void sc2331_tx_work(struct work_struct *work)
{
	struct sc2331_wifi *wifi = container_of(work, struct sc2331_wifi, tx_work);
	struct net_device *netdev = wifi->wdev.netdev;
	struct sk_buff *skb;
	ssize_t encoded;
	unsigned int budget = 16;
	int ret;

	while (budget-- && (skb = skb_dequeue(&wifi->tx_queue))) {
		mutex_lock(&wifi->lock);
		ret = -ENOTCONN;
		if (!wifi->stopping && wifi->connected) {
			ret = sc2331_boot_wake_device(wifi->boot);
			if (!ret)
				ret = skb_copy_bits(skb, 0, wifi->data + 102, skb->len);
			if (!ret) {
				encoded = sc2331_encode_ethernet(wifi->data, SC2331_RUNTIME_RX_SIZE,
								 wifi->sequence++,
								 wifi->packet_sequence++,
								 wifi->data + 102, skb->len);
				ret = encoded < 0 ? encoded :
					sc2331_sdio_runtime_tx(wifi->func, wifi->data, encoded);
			}
		}
		if (ret) {
			DEV_STATS_INC(netdev, tx_errors);
			DEV_STATS_INC(netdev, tx_dropped);
		} else {
			DEV_STATS_INC(netdev, tx_packets);
			DEV_STATS_ADD(netdev, tx_bytes, skb->len);
		}
		dev_kfree_skb(skb);
		mutex_unlock(&wifi->lock);
	}
	mutex_lock(&wifi->lock);
	if (!wifi->stopping && wifi->connected)
		netif_wake_queue(netdev);
	if (!wifi->stopping && !skb_queue_empty(&wifi->tx_queue))
		schedule_work(&wifi->tx_work);
	mutex_unlock(&wifi->lock);
}

static netdev_tx_t sc2331_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct sc2331_wifi *wifi = *(struct sc2331_wifi **)netdev_priv(dev);

	if (skb->len < ETH_HLEN || skb->len > 1518 || READ_ONCE(wifi->stopping)) {
		DEV_STATS_INC(dev, tx_dropped);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	skb_queue_tail(&wifi->tx_queue, skb);
	if (skb_queue_len(&wifi->tx_queue) >= 64) {
		netif_stop_queue(dev);
		if (skb_queue_len(&wifi->tx_queue) < 64)
			netif_wake_queue(dev);
	}
	schedule_work(&wifi->tx_work);
	return NETDEV_TX_OK;
}

static int sc2331_net_open(struct net_device *netdev)
{
	netif_start_queue(netdev);
	return 0;
}

static int sc2331_net_stop(struct net_device *netdev)
{
	netif_stop_queue(netdev);
	return 0;
}

static const struct net_device_ops sc2331_netdev_ops = {
	.ndo_start_xmit = sc2331_start_xmit,
	.ndo_open = sc2331_net_open,
	.ndo_stop = sc2331_net_stop,
};

static struct ieee80211_rate sc2331_rates[] = {
	{ .bitrate = 10 }, { .bitrate = 20 }, { .bitrate = 55 }, { .bitrate = 110 },
	{ .bitrate = 60 }, { .bitrate = 90 }, { .bitrate = 120 }, { .bitrate = 180 },
	{ .bitrate = 240 }, { .bitrate = 360 }, { .bitrate = 480 }, { .bitrate = 540 },
};

static const u32 sc2331_ciphers[] = { WLAN_CIPHER_SUITE_CCMP, WLAN_CIPHER_SUITE_TKIP };

static int sc2331_wifi_register(struct sdio_func *func, struct sc2331_boot *boot,
				const u8 *mac)
{
	struct wiphy *wiphy;
	struct sc2331_wifi *wifi;
	struct net_device *netdev;
	unsigned int i;
	int ret;

	wiphy = wiphy_new(&sc2331_cfg80211_ops, sizeof(*wifi));
	if (!wiphy)
		return -ENOMEM;
	wifi = wiphy_priv(wiphy);
	wifi->data = kmalloc(SC2331_RUNTIME_RX_SIZE, GFP_KERNEL);
	if (!wifi->data) {
		ret = -ENOMEM;
		goto free_wiphy;
	}
	netdev = alloc_netdev(sizeof(struct sc2331_wifi *), "wlan%d", NET_NAME_ENUM, ether_setup);
	if (!netdev) {
		ret = -ENOMEM;
		goto free_data;
	}
	wifi->func = func;
	wifi->boot = boot;
	wifi->sequence = 2;
	mutex_init(&wifi->lock);
	INIT_DELAYED_WORK(&wifi->rx_work, sc2331_rx_work);
	INIT_WORK(&wifi->tx_work, sc2331_tx_work);
	skb_queue_head_init(&wifi->tx_queue);
	*(struct sc2331_wifi **)netdev_priv(netdev) = wifi;
	wifi->wdev.wiphy = wiphy;
	wifi->wdev.netdev = netdev;
	wifi->wdev.iftype = NL80211_IFTYPE_STATION;
	for (i = 0; i < ARRAY_SIZE(wifi->channels); i++) {
		wifi->channels[i].band = NL80211_BAND_2GHZ;
		wifi->channels[i].hw_value = i + 1;
		wifi->channels[i].center_freq =
			ieee80211_channel_to_frequency(i + 1, NL80211_BAND_2GHZ);
		wifi->channels[i].max_power = 20;
	}
	wifi->band.channels = wifi->channels;
	wifi->band.n_channels = ARRAY_SIZE(wifi->channels);
	wifi->band.bitrates = sc2331_rates;
	wifi->band.n_bitrates = ARRAY_SIZE(sc2331_rates);
	wiphy->bands[NL80211_BAND_2GHZ] = &wifi->band;
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->max_scan_ssids = 9;
	wiphy->cipher_suites = sc2331_ciphers;
	wiphy->n_cipher_suites = ARRAY_SIZE(sc2331_ciphers);
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	set_wiphy_dev(wiphy, &func->dev);
	memcpy(wiphy->perm_addr, mac, ETH_ALEN);
	netdev->ieee80211_ptr = &wifi->wdev;
	netdev->netdev_ops = &sc2331_netdev_ops;
	SET_NETDEV_DEV(netdev, &func->dev);
	eth_hw_addr_set(netdev, mac);
	netif_carrier_off(netdev);
	ret = wiphy_register(wiphy);
	if (ret)
		goto free_netdev;
	ret = register_netdev(netdev);
	if (ret)
		goto unregister_wiphy;
	sdio_set_drvdata(func, wifi);
	dev_info(&func->dev, "SC2331 station interface registered\n");
	return 0;
unregister_wiphy:
	wiphy_unregister(wiphy);
free_netdev:
	free_netdev(netdev);
free_data:
	kfree(wifi->data);
free_wiphy:
	wiphy_free(wiphy);
	return ret;
}

static int sc2331_sdio_probe(struct sdio_func *func,
			     const struct sdio_device_id *id)
{
	const struct firmware *firmware;
	struct sc2331_boot *boot;
	size_t offset, copied, capacity = SC2331_CHUNK_SIZE + 8;
	u8 mac[ETH_ALEN], payload[8], *data;
	ssize_t encoded;
	int ret;

	if (func->num != 1)
		return -ENODEV;
	boot = sc2331_boot_get(&func->dev);
	if (IS_ERR(boot))
		return dev_err_probe(&func->dev, PTR_ERR(boot), "Missing UART bootstrap\n");
	ret = of_get_mac_address(func->dev.of_node, mac);
	if (ret)
		return dev_err_probe(&func->dev, ret, "Missing factory MAC address\n");
	ret = request_firmware(&firmware, SC2331_WLAN_NAME, &func->dev);
	if (ret)
		return ret;
	if (!firmware->size || firmware->size > SC2331_WLAN_SIZE) {
		ret = -EINVAL;
		goto release;
	}
	data = kvmalloc(capacity, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto release;
	}
	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	if (!ret)
		ret = sdio_set_block_size(func, 512);
	sdio_release_host(func);
	if (ret)
		goto disable;
	put_unaligned_be32(SC2331_WLAN_ADDRESS, payload);
	put_unaligned_be32(SC2331_WLAN_SIZE, payload + 4);
	ret = sc2331_sdio_command(func, data, capacity, 1, payload, 8);
	if (ret)
		goto disable;
	for (offset = 0; offset < SC2331_WLAN_SIZE; offset += SC2331_CHUNK_SIZE) {
		memset(data + 8, 0xff, SC2331_CHUNK_SIZE);
		copied = offset < firmware->size ?
			min_t(size_t, firmware->size - offset, SC2331_CHUNK_SIZE) : 0;
		if (copied)
			memcpy(data + 8, firmware->data + offset, copied);
		ret = sc2331_sdio_command(func, data, capacity, 2, data + 8,
					  SC2331_CHUNK_SIZE);
		if (ret)
			goto disable;
		dev_info(&func->dev, "WLAN download: %zu/%u\n",
			 offset + SC2331_CHUNK_SIZE, SC2331_WLAN_SIZE);
	}
	ret = sc2331_sdio_command(func, data, capacity, 3, NULL, 0);
	if (ret)
		goto disable;
	sc2331_boot_arm(boot);
	ret = sc2331_sdio_command(func, data, capacity, 4, payload, 4);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 WLAN image executed\n");
	kvfree(data);
	data = kmalloc(SC2331_RUNTIME_RX_SIZE, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto disable;
	}
	capacity = SC2331_RUNTIME_RX_SIZE;
	ret = sc2331_boot_wait_ready(boot);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 WLAN calibration ready\n");
	ret = sc2331_sdio_calibrate(func, data);
	if (ret)
		goto disable;
	ret = sc2331_boot_wake_device(boot);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 WLAN runtime awake\n");
	put_unaligned_le16(1, payload);
	memcpy(payload + 2, mac, ETH_ALEN);
	encoded = sc2331_encode_command(data, capacity, 0x16, 0, payload, sizeof(payload));
	if (encoded < 0) {
		ret = encoded;
		goto disable;
	}
	ret = sc2331_sdio_runtime_tx(func, data, encoded);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 MAC_OPEN command sent\n");
	ret = sc2331_sdio_runtime_response(func, data, 0x16, 8000, NULL, NULL);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 MAC_OPEN response accepted\n");
	ret = sc2331_boot_wake_device(boot);
	if (ret)
		goto disable;
	payload[0] = 1;
	payload[1] = 0x28;
	put_unaligned_le16(4, payload + 2);
	put_unaligned_le32(3, payload + 4);
	encoded = sc2331_encode_command(data, capacity, 0x21, 1, payload, sizeof(payload));
	if (encoded < 0) {
		ret = encoded;
		goto disable;
	}
	ret = sc2331_sdio_runtime_tx(func, data, encoded);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 normal capability command sent\n");
	ret = sc2331_sdio_runtime_response(func, data, 0x21, 5000, NULL, NULL);
	if (ret)
		goto disable;
	dev_info(&func->dev, "SC2331 normal capability response accepted\n");
	ret = sc2331_wifi_register(func, boot, mac);
	if (ret)
		goto disable;
	goto free;
disable:
	sc2331_boot_sleep_device(boot);
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	dev_err(&func->dev, "SDIO download failed: %d\n", ret);
free:
	kvfree(data);
release:
	release_firmware(firmware);
	return ret;
}

static void sc2331_sdio_remove(struct sdio_func *func)
{
	struct sc2331_wifi *wifi = sdio_get_drvdata(func);
	struct wiphy *wiphy = wifi->wdev.wiphy;

	netif_tx_disable(wifi->wdev.netdev);
	mutex_lock(&wifi->lock);
	wifi->stopping = true;
	sc2331_scan_finish(wifi, true);
	sc2331_connection_down(wifi, WLAN_REASON_DEAUTH_LEAVING, true);
	mutex_unlock(&wifi->lock);
	cancel_delayed_work_sync(&wifi->rx_work);
	cancel_work_sync(&wifi->tx_work);
	unregister_netdev(wifi->wdev.netdev);
	wiphy_unregister(wiphy);
	free_netdev(wifi->wdev.netdev);
	sc2331_boot_sleep_device(wifi->boot);
	kfree(wifi->data);
	wiphy_free(wiphy);
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
}

static const struct sdio_device_id sc2331_sdio_ids[] = {
	{ SDIO_DEVICE(0x0000, 0x2331) },
	{ }
};
MODULE_DEVICE_TABLE(sdio, sc2331_sdio_ids);

struct sdio_driver sc2331_sdio_driver = {
	.name = "sc2331",
	.id_table = sc2331_sdio_ids,
	.probe = sc2331_sdio_probe,
	.remove = sc2331_sdio_remove,
};
MODULE_FIRMWARE(SC2331_WLAN_NAME);
MODULE_FIRMWARE(SC2331_CALIB_NAME);
