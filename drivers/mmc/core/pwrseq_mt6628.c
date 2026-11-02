// SPDX-License-Identifier: GPL-2.0-only
#include <linux/crc16.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <linux/mmc/host.h>

#include "pwrseq.h"

#define MT6628_STP_MAX 2048

struct mt6628_stp {
	u8 data[MT6628_STP_MAX + 6];
	size_t used;
	size_t length;
	u8 tx_seq;
	u8 rx_seq;
	bool full;
	bool pending;
};

static int mt6628_stp_tx(struct mt6628_stp *stp, const u8 *data,
			 size_t length, u8 *wire, size_t capacity)
{
	u16 crc;

	if (!length || length > MT6628_STP_MAX || capacity < length + 6)
		return -EMSGSIZE;
	if (stp->pending)
		return -EBUSY;

	wire[0] = 0x80;
	wire[1] = 0x40 | (length >> 8);
	wire[2] = length;
	wire[3] = 0;
	crc = 0;
	if (stp->full) {
		wire[0] |= stp->tx_seq << 3 | ((stp->rx_seq + 7) & 7);
		wire[3] = wire[0] + wire[1] + wire[2];
		crc = crc16(0, data, length);
		stp->pending = true;
	}
	memcpy(wire + 4, data, length);
	put_unaligned_le16(crc, wire + length + 4);
	return length + 6;
}

static void mt6628_stp_ack(struct mt6628_stp *stp, u8 *wire)
{
	wire[0] = 0x80 | ((stp->rx_seq + 7) & 7);
	wire[1] = 0;
	wire[2] = 0;
	wire[3] = wire[0];
}

static int mt6628_stp_rx(struct mt6628_stp *stp, u8 byte)
{
	u8 ack, seq;
	size_t size;

	if (stp->used >= sizeof(stp->data))
		return -EMSGSIZE;
	stp->data[stp->used++] = byte;
	if (stp->used == 1 &&
	    (stp->full ? (byte & 0xc0) != 0x80 : byte != 0x80))
		return -EPROTO;
	if (stp->used < 4)
		return 0;
	if (stp->used == 4) {
		stp->length = (stp->data[1] & 0xf) << 8 | stp->data[2];
		if (stp->length > MT6628_STP_MAX)
			return -EMSGSIZE;
		if (stp->length ? (stp->data[1] & 0xf0) != 0x40 :
		    (!stp->full || stp->data[1] || (stp->data[0] & 0x38)))
			return -EPROTO;
		if (stp->data[3] != (stp->full ?
		    (u8)(stp->data[0] + stp->data[1] + stp->data[2]) : 0))
			return -EBADMSG;
	}
	size = stp->length ? stp->length + 6 : 4;
	if (stp->used < size)
		return 0;
	stp->used = 0;
	if (stp->length && get_unaligned_le16(stp->data + size - 2) !=
	    (stp->full ? crc16(0, stp->data + 4, stp->length) : 0))
		return -EBADMSG;
	if (!stp->full)
		return 1;

	ack = stp->data[0] & 7;
	if (stp->pending && ack == stp->tx_seq) {
		stp->pending = false;
		stp->tx_seq = (stp->tx_seq + 1) & 7;
	}
	if (!stp->length)
		return 2;
	seq = (stp->data[0] >> 3) & 7;
	if (seq == ((stp->rx_seq + 7) & 7))
		return 3;
	if (seq != stp->rx_seq)
		return -EILSEQ;
	stp->rx_seq = (stp->rx_seq + 1) & 7;
	return 1;
}

struct mt6628 {
	struct mmc_pwrseq pwrseq;
	struct serdev_device *serdev;
	struct clk *rtc;
	struct gpio_desc *reset;
	struct gpio_desc *ldo;
	struct gpio_desc *pmu[2];
	struct pinctrl *pinctrl;
	struct pinctrl_state *safe;
	struct pinctrl_state *active;
	const struct firmware *patch;
	const struct firmware *calibration;
	struct mutex lock; /* Ownership */
	struct mutex rx_lock; /* Transport */
	struct completion reply;
	struct work_struct ack_work;
	struct mt6628_stp stp;
	u8 wire[MT6628_STP_MAX + 6];
	u8 event[64];
	size_t event_len;
	size_t expected;
	int rx_error;
	u32 hw_version;
	u32 rom_version;
	u8 antenna_mode;
	bool waiting;
	bool stopping;
	bool powered;
	bool opened;
	bool rtc_enabled;
};

static void mt6628_ack_work(struct work_struct *work)
{
	struct mt6628 *ctx = container_of(work, struct mt6628, ack_work);
	u8 ack[4];
	int ret;

	mutex_lock(&ctx->rx_lock);
	if (ctx->stopping || ctx->rx_error) {
		mutex_unlock(&ctx->rx_lock);
		return;
	}
	mt6628_stp_ack(&ctx->stp, ack);
	mutex_unlock(&ctx->rx_lock);

	ret = serdev_device_write(ctx->serdev, ack, sizeof(ack),
				  msecs_to_jiffies(1000));
	if (ret == sizeof(ack))
		return;
	mutex_lock(&ctx->rx_lock);
	ctx->rx_error = ret < 0 ? ret : -EIO;
	complete(&ctx->reply);
	mutex_unlock(&ctx->rx_lock);
}

static size_t mt6628_receive(struct serdev_device *serdev, const u8 *data,
			     size_t count)
{
	struct mt6628 *ctx = serdev_device_get_drvdata(serdev);
	size_t i;
	int ret;

	mutex_lock(&ctx->rx_lock);
	if (ctx->stopping || ctx->rx_error)
		goto unlock;
	for (i = 0; i < count; i++) {
		ret = mt6628_stp_rx(&ctx->stp, data[i]);
		if (ret < 0) {
			ctx->rx_error = ret;
			break;
		}
		if (ctx->stp.full && (ret == 1 || ret == 3))
			schedule_work(&ctx->ack_work);
		if (ret == 1 && ctx->waiting) {
			if (ctx->stp.length > ctx->expected - ctx->event_len) {
				ctx->rx_error = -EMSGSIZE;
				break;
			}
			memcpy(ctx->event + ctx->event_len, ctx->stp.data + 4,
			       ctx->stp.length);
			ctx->event_len += ctx->stp.length;
		}
		if (ctx->waiting && ctx->event_len == ctx->expected &&
		    !ctx->stp.pending)
			complete(&ctx->reply);
	}
	if (ctx->rx_error)
		complete(&ctx->reply);
unlock:
	mutex_unlock(&ctx->rx_lock);
	return count;
}

static int mt6628_exchange(struct mt6628 *ctx, const u8 *command,
			   size_t command_len, u8 *event, size_t event_len,
			   bool raw)
{
	unsigned int attempt;
	int length, written, ret;
	bool retry;

	if (!event_len || event_len > sizeof(ctx->event))
		return -EMSGSIZE;
	mutex_lock(&ctx->rx_lock);
	if (ctx->stopping || ctx->rx_error || ctx->waiting) {
		ret = ctx->rx_error ? ctx->rx_error : -EBUSY;
		goto unlock;
	}
	if (raw) {
		if (ctx->stp.full || command_len != 1 || command[0] != 0xff) {
			ret = -EINVAL;
			goto unlock;
		}
		ctx->wire[0] = 0xff;
		length = 1;
	} else {
		length = mt6628_stp_tx(&ctx->stp, command, command_len,
				       ctx->wire, sizeof(ctx->wire));
		if (length < 0) {
			ret = length;
			goto unlock;
		}
	}
	ctx->event_len = 0;
	ctx->expected = event_len;
	ctx->waiting = true;
	reinit_completion(&ctx->reply);
	mutex_unlock(&ctx->rx_lock);

	ret = -ETIMEDOUT;
	for (attempt = 0; attempt < 3; attempt++) {
		written = serdev_device_write(ctx->serdev, ctx->wire, length,
					      msecs_to_jiffies(1000));
		if (written != length) {
			ret = written < 0 ? written : -EIO;
			break;
		}
		wait_for_completion_timeout(&ctx->reply, msecs_to_jiffies(1000));
		mutex_lock(&ctx->rx_lock);
		if (ctx->rx_error)
			ret = ctx->rx_error;
		else if (ctx->event_len == event_len && !ctx->stp.pending)
			ret = 0;
		retry = ctx->stp.full && ctx->stp.pending && !ctx->rx_error;
		mutex_unlock(&ctx->rx_lock);
		if (!ret || !retry)
			break;
	}

	mutex_lock(&ctx->rx_lock);
	ctx->waiting = false;
	if (!ret && ctx->rx_error)
		ret = ctx->rx_error;
	if (!ret)
		memcpy(event, ctx->event, event_len);
	else
		ctx->rx_error = ret;
unlock:
	mutex_unlock(&ctx->rx_lock);
	return ret;
}

static int mt6628_set_baud(struct mt6628 *ctx, unsigned int baud)
{
	unsigned int actual = serdev_device_set_baudrate(ctx->serdev, baud);
	u64 difference = actual > baud ? actual - baud : baud - actual;

	return !actual || !baud || difference * 100 > (u64)baud * 3 ?
		-ERANGE : 0;
}

static int mt6628_command(struct mt6628 *ctx, const u8 *command,
			  size_t length, const u8 *expected, size_t event_len)
{
	u8 event[64];
	int ret;

	if (event_len > sizeof(event))
		return -EMSGSIZE;
	ret = mt6628_exchange(ctx, command, length, event, event_len, false);
	if (ret)
		return ret;
	return memcmp(event, expected, event_len) ? -EBADMSG : 0;
}

static int mt6628_read_reg(struct mt6628 *ctx, u32 address, u32 *value)
{
	u8 command[20] = { 1, 8, 16, 0, 2, 1, 0, 1 };
	u8 event[16];
	int ret;

	put_unaligned_le32(address, command + 8);
	put_unaligned_le32(0xffff, command + 16);
	ret = mt6628_exchange(ctx, command, sizeof(command), event,
			      sizeof(event), false);
	if (ret)
		return ret;
	if (event[0] != 2 || event[1] != 8 ||
	    get_unaligned_le16(event + 2) != 12 || event[4] ||
	    get_unaligned_le32(event + 8) != address)
		return -EBADMSG;
	*value = get_unaligned_le32(event + 12);
	return 0;
}

static int mt6628_download_patch(struct mt6628 *ctx,
				 const struct firmware *firmware)
{
	static const u8 expected[] = { 2, 1, 1, 0, 0 };
	size_t offset, length, payload;
	u16 version;
	u8 *command;
	int ret = 0;

	if (firmware->size <= 28 || firmware->size > SZ_1M)
		return -EINVAL;
	switch (ctx->hw_version) {
	case 0x8a00:
		version = 0x8a00;
		break;
	case 0x8a10:
	case 0x8b10:
	case 0x8b11:
	case 0x8a11:
		version = 0x8a10;
		break;
	default:
		return -ENODEV;
	}
	if (get_unaligned_be16(firmware->data + 20) != version)
		return -EBADMSG;
	command = kmalloc(1005, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	payload = firmware->size - 28;
	command[0] = 1;
	command[1] = 1;
	for (offset = 0; offset < payload; offset += length) {
		length = min_t(size_t, 1000, payload - offset);
		put_unaligned_le16(length + 1, command + 2);
		command[4] = offset + length == payload ? 3 : offset ? 2 : 1;
		memcpy(command + 5, firmware->data + 28 + offset, length);
		ret = mt6628_command(ctx, command, length + 5, expected,
				     sizeof(expected));
		if (ret)
			break;
	}
	kfree(command);
	return ret;
}

static int mt6628_crystal_trim(struct mt6628 *ctx,
			       const struct firmware *calibration)
{
	static const u8 query[] = { 1, 0x12, 2, 0, 0, 0 };
	static const u8 prefix[] = { 2, 0x12, 2, 0, 0 };
	u8 command[] = { 1, 0x12, 2, 0, 1, 0 };
	u8 expected[] = { 2, 0x12, 2, 0, 1, 0 };
	u8 event[6];
	int offset, value, ret;

	if (calibration->size < 110)
		return -EINVAL;
	if (!(calibration->data[109] & 0x80))
		return 0;
	offset = calibration->data[109] & 0x7f;
	if (offset & 0x40)
		offset -= 128;
	ret = mt6628_exchange(ctx, query, sizeof(query), event, sizeof(event),
			      false);
	if (ret)
		return ret;
	if (memcmp(event, prefix, sizeof(prefix)))
		return -EBADMSG;
	value = (event[5] & 0x7f) + offset;
	command[5] = value < 0 ? 0 : value;
	expected[5] = command[5];
	ret = mt6628_command(ctx, command, sizeof(command), expected,
			     sizeof(expected));
	if (ret)
		return ret;
	expected[4] = 0;
	return mt6628_command(ctx, query, sizeof(query), expected,
			      sizeof(expected));
}

static const char *mt6628_patch_name(u32 version)
{
	switch (version) {
	case 0x8a00:
		return "mediatek/mt6628_patch_e1_hdr.bin";
	case 0x8a10:
	case 0x8b10:
	case 0x8b11:
	case 0x8a11:
		return "mediatek/mt6628_patch_e2_hdr.bin";
	default:
		return NULL;
	}
}

static int mt6628_identify(struct mt6628 *ctx)
{
	u32 chip, version, rom;
	int ret;

	ret = mt6628_read_reg(ctx, 0x80000008, &chip);
	if (ret)
		return ret;
	if (chip != 0x6628) {
		dev_err(&ctx->serdev->dev, "unsupported chip %#x\n", chip);
		return -ENODEV;
	}
	ret = mt6628_read_reg(ctx, 0x80000000, &version);
	if (ret)
		return ret;
	if (!mt6628_patch_name(version)) {
		dev_err(&ctx->serdev->dev, "unsupported MT6628 revision %#x\n", version);
		return -ENODEV;
	}
	ret = mt6628_read_reg(ctx, 0x80000004, &rom);
	if (ret)
		return ret;
	ctx->hw_version = version;
	ctx->rom_version = rom;
	dev_info(&ctx->serdev->dev, "MT6628 hw %#x ROM %#x\n", version, rom);
	return 0;
}

static int mt6628_query(struct mt6628 *ctx, u8 selector, u32 value)
{
	u8 command[] = { 1, 4, 1, 0, selector };
	u8 expected[] = { 2, 4, 6, 0, 0, selector, 0, 0, 0, 0 };

	put_unaligned_le32(value, expected + 6);
	return mt6628_command(ctx, command, sizeof(command), expected,
			      sizeof(expected));
}

static int mt6628_bootstrap(struct mt6628 *ctx, const struct firmware *patch,
			    const struct firmware *calibration, u8 antenna_mode)
{
	static const u8 baud[] = { 1, 4, 5, 0, 1, 0, 9, 0x3d, 0 };
	static const u8 baud_event[] = { 2, 4, 2, 0, 0, 1 };
	static const u8 wake[] = { 0xff };
	static const u8 wake_event[] = { 2, 3, 2, 0, 0, 3 };
	static const u8 reset[] = { 1, 7, 1, 0, 4 };
	static const u8 reset_event[] = { 2, 7, 1, 0, 0 };
	static const u8 coex_event[] = { 2, 0x10, 1, 0, 0 };
	static const u8 full[] = { 1, 4, 5, 0, 3, 0xdf, 0x0e, 0x68, 1 };
	static const u8 full_event[] = { 2, 4, 2, 0, 0, 3 };
	u8 coex[] = { 1, 0x10, 2, 0, 1, antenna_mode };
	u8 event[sizeof(wake_event)];
	int ret;

	if (antenna_mode < 1 || antenna_mode > 4)
		return -EINVAL;
	ret = mt6628_query(ctx, 2, 115200);
	if (ret)
		return ret;
	ret = mt6628_query(ctx, 4, 0x11);
	if (ret)
		return ret;
	ret = mt6628_command(ctx, baud, sizeof(baud), baud_event,
			     sizeof(baud_event));
	if (ret)
		return ret;
	ret = mt6628_set_baud(ctx, 4000000);
	if (ret)
		return ret;
	ret = mt6628_exchange(ctx, wake, sizeof(wake), event, sizeof(event), true);
	if (ret)
		return ret;
	if (memcmp(event, wake_event, sizeof(event)))
		return -EBADMSG;
	ret = mt6628_query(ctx, 2, 4000000);
	if (ret)
		return ret;
	ret = mt6628_download_patch(ctx, patch);
	if (ret)
		return ret;
	ret = mt6628_command(ctx, reset, sizeof(reset), reset_event,
			     sizeof(reset_event));
	if (ret)
		return ret;
	ret = mt6628_command(ctx, coex, sizeof(coex), coex_event,
			     sizeof(coex_event));
	if (ret)
		return ret;
	ret = mt6628_crystal_trim(ctx, calibration);
	if (ret)
		return ret;
	ret = mt6628_command(ctx, full, sizeof(full), full_event,
			     sizeof(full_event));
	if (ret)
		return ret;
	flush_work(&ctx->ack_work);
	mutex_lock(&ctx->rx_lock);
	ret = ctx->rx_error;
	if (!ret && (ctx->waiting || ctx->stp.pending || ctx->stp.used))
		ret = -EBUSY;
	if (!ret)
		ctx->stp = (struct mt6628_stp) { .full = true };
	mutex_unlock(&ctx->rx_lock);
	if (ret)
		return ret;
	usleep_range(10000, 12000);
	ret = mt6628_query(ctx, 4, 0x01680edf);
	if (ret)
		return ret;
	ret = mt6628_query(ctx, 2, 4000000);
	if (ret)
		return ret;
	flush_work(&ctx->ack_work);
	mutex_lock(&ctx->rx_lock);
	ret = ctx->rx_error;
	mutex_unlock(&ctx->rx_lock);
	return ret;
}

static int mt6628_power_off(struct mt6628 *ctx)
{
	unsigned int i;
	int ret;

	lockdep_assert_held(&ctx->lock);
	ctx->powered = false;
	mutex_lock(&ctx->rx_lock);
	ctx->stopping = true;
	ctx->waiting = false;
	if (!ctx->rx_error)
		ctx->rx_error = -ESHUTDOWN;
	complete(&ctx->reply);
	mutex_unlock(&ctx->rx_lock);
	cancel_work_sync(&ctx->ack_work);
	if (ctx->opened) {
		serdev_device_close(ctx->serdev);
		ctx->opened = false;
	}
	gpiod_set_value_cansleep(ctx->reset, 1);
	for (i = 0; i < ARRAY_SIZE(ctx->pmu); i++)
		gpiod_set_value_cansleep(ctx->pmu[i], 0);
	ret = pinctrl_select_state(ctx->pinctrl, ctx->safe);
	if (ret)
		dev_err(&ctx->serdev->dev, "safe pin state failed, retaining LDO: %d\n", ret);
	if (ctx->rtc_enabled) {
		clk_disable_unprepare(ctx->rtc);
		ctx->rtc_enabled = false;
	}
	if (!ret)
		gpiod_set_value_cansleep(ctx->ldo, 0);
	return ret;
}

static int mt6628_power_on(struct mt6628 *ctx)
{
	unsigned int i;
	int ret;

	lockdep_assert_held(&ctx->lock);
	mutex_lock(&ctx->rx_lock);
	ret = ctx->rx_error;
	mutex_unlock(&ctx->rx_lock);
	if (ctx->powered && !ret)
		return 0;
	ret = mt6628_power_off(ctx);
	if (ret)
		return ret;
	gpiod_set_value_cansleep(ctx->ldo, 1);
	msleep(100);
	ret = clk_prepare_enable(ctx->rtc);
	if (ret)
		goto off;
	ctx->rtc_enabled = true;
	msleep(100);
	ret = pinctrl_select_state(ctx->pinctrl, ctx->active);
	if (ret)
		goto off;
	ret = serdev_device_open(ctx->serdev);
	if (ret)
		goto off;
	ctx->opened = true;
	serdev_device_set_flow_control(ctx->serdev, false);
	ret = serdev_device_set_parity(ctx->serdev, SERDEV_PARITY_NONE);
	if (ret)
		goto off;
	ret = mt6628_set_baud(ctx, 115200);
	if (ret)
		goto off;
	mutex_lock(&ctx->rx_lock);
	ctx->stp = (struct mt6628_stp) { 0 };
	ctx->rx_error = 0;
	ctx->event_len = 0;
	ctx->expected = 0;
	ctx->stopping = false;
	mutex_unlock(&ctx->rx_lock);
	usleep_range(10000, 12000);
	for (i = 0; i < ARRAY_SIZE(ctx->pmu); i++)
		gpiod_set_value_cansleep(ctx->pmu[i], 1);
	msleep(30);
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(30);
	ret = mt6628_identify(ctx);
	if (ret)
		goto off;
	if (!ctx->patch) {
		ret = request_firmware(&ctx->patch,
				       mt6628_patch_name(ctx->hw_version),
				       &ctx->serdev->dev);
		if (ret)
			goto off;
	}
	ret = mt6628_bootstrap(ctx, ctx->patch, ctx->calibration, ctx->antenna_mode);
	if (ret)
		goto off;
	mutex_lock(&ctx->rx_lock);
	ret = ctx->rx_error;
	if (!ret)
		ctx->powered = true;
	mutex_unlock(&ctx->rx_lock);
	if (ret)
		goto off;
	return 0;
off:
	mt6628_power_off(ctx);
	return ret;
}

static void mt6628_pre_power_on(struct mmc_host *host)
{
	struct mt6628 *ctx = container_of(host->pwrseq, struct mt6628, pwrseq);
	int ret;

	mutex_lock(&ctx->lock);
	ret = mt6628_power_on(ctx);
	mutex_unlock(&ctx->lock);
	if (ret)
		dev_err(&ctx->serdev->dev, "bootstrap failed: %d\n", ret);
}

static void mt6628_mmc_power_off(struct mmc_host *host)
{
	struct mt6628 *ctx = container_of(host->pwrseq, struct mt6628, pwrseq);

	mutex_lock(&ctx->lock);
	mt6628_power_off(ctx);
	mutex_unlock(&ctx->lock);
}

static const struct mmc_pwrseq_ops mt6628_pwrseq_ops = {
	.pre_power_on = mt6628_pre_power_on,
	.power_off = mt6628_mmc_power_off,
};

static void mt6628_shutdown(struct serdev_device *serdev)
{
	struct mt6628 *ctx = serdev_device_get_drvdata(serdev);

	mutex_lock(&ctx->lock);
	mt6628_power_off(ctx);
	mutex_unlock(&ctx->lock);
}

static void mt6628_remove(struct serdev_device *serdev)
{
	struct mt6628 *ctx = serdev_device_get_drvdata(serdev);

	mmc_pwrseq_unregister(&ctx->pwrseq);
	mt6628_shutdown(serdev);
	serdev_device_set_client_ops(serdev, NULL);
	release_firmware(ctx->patch);
	release_firmware(ctx->calibration);
	serdev_device_set_drvdata(serdev, NULL);
}

static const struct serdev_device_ops mt6628_serdev_ops = {
	.receive_buf = mt6628_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static int mt6628_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct mt6628 *ctx;
	u32 antenna_mode;
	unsigned int i;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->serdev = serdev;
	ctx->stopping = true;
	mutex_init(&ctx->lock);
	mutex_init(&ctx->rx_lock);
	init_completion(&ctx->reply);
	INIT_WORK(&ctx->ack_work, mt6628_ack_work);
	ret = device_property_read_u32(dev, "mediatek,antenna-mode", &antenna_mode);
	if (ret)
		return dev_err_probe(dev, ret, "antenna mode missing\n");
	if (antenna_mode < 1 || antenna_mode > 4)
		return -EINVAL;
	ctx->antenna_mode = antenna_mode;
	ctx->rtc = devm_clk_get(dev, "rtc");
	if (IS_ERR(ctx->rtc))
		return dev_err_probe(dev, PTR_ERR(ctx->rtc), "RTC clock unavailable\n");
	ctx->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ctx->pinctrl))
		return dev_err_probe(dev, PTR_ERR(ctx->pinctrl), "pin control unavailable\n");
	ctx->safe = pinctrl_lookup_state(ctx->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(ctx->safe))
		return dev_err_probe(dev, PTR_ERR(ctx->safe), "safe pins unavailable\n");
	ctx->active = pinctrl_lookup_state(ctx->pinctrl, "active");
	if (IS_ERR(ctx->active))
		return dev_err_probe(dev, PTR_ERR(ctx->active), "active pins unavailable\n");
	ret = gpiod_count(dev, "power");
	if (ret < 0)
		return dev_err_probe(dev, ret, "power GPIOs unavailable\n");
	if (ret != ARRAY_SIZE(ctx->pmu))
		return -EINVAL;
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "reset GPIO unavailable\n");
	ctx->ldo = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->ldo))
		return dev_err_probe(dev, PTR_ERR(ctx->ldo), "enable GPIO unavailable\n");
	for (i = 0; i < ARRAY_SIZE(ctx->pmu); i++) {
		ctx->pmu[i] = devm_gpiod_get_index(dev, "power", i, GPIOD_OUT_LOW);
		if (IS_ERR(ctx->pmu[i]))
			return dev_err_probe(dev, PTR_ERR(ctx->pmu[i]),
					     "power GPIO unavailable\n");
	}
	ret = request_firmware(&ctx->calibration, "mediatek/mt6628-nvram.bin", dev);
	if (ret)
		return dev_err_probe(dev, ret, "factory calibration unavailable\n");
	if (ctx->calibration->size < 110) {
		ret = -EINVAL;
		goto release;
	}
	serdev_device_set_drvdata(serdev, ctx);
	serdev_device_set_client_ops(serdev, &mt6628_serdev_ops);
	mutex_lock(&ctx->lock);
	ret = mt6628_power_on(ctx);
	mutex_unlock(&ctx->lock);
	if (ret)
		goto release;
	ctx->pwrseq.dev = dev;
	ctx->pwrseq.ops = &mt6628_pwrseq_ops;
	ctx->pwrseq.owner = THIS_MODULE;
	ret = mmc_pwrseq_register(&ctx->pwrseq);
	if (ret)
		goto off;
	dev_info(dev, "MT6628 hw %#x ROM %#x bootstrap ready\n",
		 ctx->hw_version, ctx->rom_version);
	return 0;
off:
	mt6628_shutdown(serdev);
release:
	serdev_device_set_client_ops(serdev, NULL);
	release_firmware(ctx->patch);
	release_firmware(ctx->calibration);
	serdev_device_set_drvdata(serdev, NULL);
	return dev_err_probe(dev, ret, "bootstrap unavailable\n");
}

static const struct of_device_id mt6628_match[] = {
	{ .compatible = "mediatek,mt6628-wmt" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6628_match);

static struct serdev_device_driver mt6628_driver = {
	.probe = mt6628_probe,
	.remove = mt6628_remove,
	.shutdown = mt6628_shutdown,
	.driver = {
		.name = "mt6628-wmt",
		.of_match_table = mt6628_match,
	},
};
module_serdev_device_driver(mt6628_driver);

MODULE_DESCRIPTION("MediaTek MT6628 WMT bootstrap and MMC power sequence");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("mediatek/mt6628_patch_e1_hdr.bin");
MODULE_FIRMWARE("mediatek/mt6628_patch_e2_hdr.bin");
MODULE_FIRMWARE("mediatek/mt6628-nvram.bin");
