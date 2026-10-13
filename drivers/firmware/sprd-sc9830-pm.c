// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "sprd-sc9830-pm.h"

struct sc9830_pm {
	struct clk_hw clock;
	struct sc9830_pm_ring ring;
	struct mbox_client client;
	struct mbox_chan *channel;
	struct mutex lock;
	spinlock_t rx_lock;
	struct completion done;
	u32 expected;
	u32 value;
	u32 desired;
	int result;
	bool pending;
	bool fault;
};

static void sc9830_pm_rx(struct mbox_client *client, void *message)
{
	struct sc9830_pm *pm = container_of(client, struct sc9830_pm, client);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pm->rx_lock, flags);
	if (!pm->pending)
		goto unlock;
	ret = sc9830_pm_receive(&pm->ring, pm->expected, &pm->value);
	if (ret == -EAGAIN)
		goto unlock;
	pm->result = ret == 1 ? 0 : ret;
	pm->fault |= ret < 0;
	pm->pending = false;
	complete(&pm->done);
unlock:
	spin_unlock_irqrestore(&pm->rx_lock, flags);
}

static int sc9830_pm_exchange(struct sc9830_pm *pm, u32 request, u32 reply, u32 value)
{
	unsigned long flags;
	u32 consumed;
	int ret;

	lockdep_assert_held(&pm->lock);
	spin_lock_irqsave(&pm->rx_lock, flags);
	if (pm->fault) {
		spin_unlock_irqrestore(&pm->rx_lock, flags);
		return -EIO;
	}
	reinit_completion(&pm->done);
	pm->expected = reply;
	pm->pending = true;
	ret = sc9830_pm_publish(&pm->ring, request, value);
	if (ret) {
		pm->pending = false;
		pm->fault = true;
	}
	spin_unlock_irqrestore(&pm->rx_lock, flags);
	if (ret)
		return ret;
	ret = mbox_send_message(pm->channel, pm);
	if (ret < 0)
		goto fault;
	if (!wait_for_completion_timeout(&pm->done, msecs_to_jiffies(500))) {
		spin_lock_irqsave(&pm->rx_lock, flags);
		pm->pending = false;
		pm->fault = true;
		spin_unlock_irqrestore(&pm->rx_lock, flags);
		ret = -ETIMEDOUT;
	} else {
		ret = pm->result;
	}
	if (!ret && request == SC9830_PM_RATE && value == 640 && pm->value != 640)
		ret = -EPROTO;
	if (!ret)
		ret = readl_poll_timeout(pm->ring.base + SC9830_PM_TX_RD,
					 consumed, consumed == pm->ring.tx, 10, 500000);
	mbox_client_txdone(pm->channel, ret);
	if (ret)
		goto fault;
	return 0;
fault:
	spin_lock_irqsave(&pm->rx_lock, flags);
	pm->pending = false;
	pm->fault = true;
	spin_unlock_irqrestore(&pm->rx_lock, flags);
	return ret;
}

static ssize_t sc9830_pm_frequency(struct device *dev, u32 request, char *buf)
{
	struct sc9830_pm *pm = dev_get_drvdata(dev);
	u32 mhz;
	int ret;

	mutex_lock(&pm->lock);
	ret = sc9830_pm_exchange(pm, request, SC9830_PM_QUERIED, pm->desired);
	mhz = pm->value;
	mutex_unlock(&pm->lock);
	return ret ? ret : sysfs_emit(buf, "%u\n", mhz * 1000000);
}

static unsigned long sc9830_pm_clock_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct sc9830_pm *pm = container_of(hw, struct sc9830_pm, clock);
	u32 mhz;
	int ret;

	mutex_lock(&pm->lock);
	ret = sc9830_pm_exchange(pm, SC9830_PM_QUERY, SC9830_PM_QUERIED, pm->desired);
	mhz = pm->value;
	mutex_unlock(&pm->lock);
	return ret ? 0 : mhz * 1000000UL;
}

static const struct clk_ops sc9830_pm_clock_ops = {
	.recalc_rate = sc9830_pm_clock_rate,
};

static int sc9830_pm_register_clock(struct device *dev, struct sc9830_pm *pm)
{
	struct clk_init_data init = {
		.ops = &sc9830_pm_clock_ops,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	u32 cells;
	int ret;

	if (!of_property_present(dev->of_node, "#clock-cells"))
		return 0;
	ret = of_property_read_u32(dev->of_node, "#clock-cells", &cells);
	if (ret || cells)
		return -EINVAL;
	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-ddr", dev_name(dev));
	if (!init.name)
		return -ENOMEM;
	pm->clock.init = &init;
	ret = devm_clk_hw_register(dev, &pm->clock);
	if (ret)
		return ret;
	if (READ_ONCE(pm->fault))
		return -EIO;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &pm->clock);
}

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sc9830_pm_frequency(dev, SC9830_PM_QUERY, buf);
}
static DEVICE_ATTR_RO(frequency);

static ssize_t target_frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sc9830_pm_frequency(dev, SC9830_PM_TARGET, buf);
}
static DEVICE_ATTR_RO(target_frequency);

static ssize_t high_rate_lock_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sc9830_pm *pm = dev_get_drvdata(dev);
	bool enabled;
	int ret;

	ret = kstrtobool(buf, &enabled);
	if (ret)
		return ret;
	mutex_lock(&pm->lock);
	ret = sc9830_pm_exchange(pm, SC9830_PM_RATE, SC9830_PM_RATE_REPLY,
				enabled ? 640 : 192);
	mutex_unlock(&pm->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(high_rate_lock);

static struct attribute *sc9830_pm_attrs[] = {
	&dev_attr_frequency.attr,
	&dev_attr_target_frequency.attr,
	&dev_attr_high_rate_lock.attr,
	NULL,
};
static const struct attribute_group sc9830_pm_group = {
	.attrs = sc9830_pm_attrs,
};

static void sc9830_pm_release(void *data)
{
	struct sc9830_pm *pm = data;

	mbox_free_channel(pm->channel);
}

static int sc9830_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc9830_pm *pm;
	struct resource *resource;
	struct clk *emc;
	int ret;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource || resource_size(resource) < 0x810)
		return -EINVAL;
	pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return -ENOMEM;
	pm->ring.base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pm->ring.base))
		return PTR_ERR(pm->ring.base);
	mutex_init(&pm->lock);
	spin_lock_init(&pm->rx_lock);
	init_completion(&pm->done);
	ret = sc9830_pm_ring_init(&pm->ring);
	if (ret)
		return dev_err_probe(dev, ret, "PM rings require zero indices\n");
	emc = devm_clk_get(dev, "emc");
	if (IS_ERR(emc))
		return PTR_ERR(emc);
	pm->desired = DIV_ROUND_CLOSEST(clk_get_rate(emc), 1000000);
	if (pm->desired != 192 && pm->desired != 384 && pm->desired != 500 && pm->desired != 640)
		return -ERANGE;
	pm->client.dev = dev;
	pm->client.rx_callback = sc9830_pm_rx;
	pm->client.knows_txdone = true;
	pm->channel = mbox_request_channel_byname(&pm->client, "pm");
	if (IS_ERR(pm->channel))
		return dev_err_probe(dev, PTR_ERR(pm->channel), "PM mailbox unavailable\n");
	ret = devm_add_action_or_reset(dev, sc9830_pm_release, pm);
	if (ret)
		return ret;
	mutex_lock(&pm->lock);
	ret = sc9830_pm_exchange(pm, SC9830_PM_OPEN, SC9830_PM_OPEN, 0);
	if (!ret)
		ret = sc9830_pm_exchange(pm, SC9830_PM_ENABLE, SC9830_PM_ENABLED, pm->desired);
	if (!ret)
		ret = sc9830_pm_exchange(pm, SC9830_PM_QUERY, SC9830_PM_QUERIED, pm->desired);
	mutex_unlock(&pm->lock);
	if (ret)
		return dev_err_probe(dev, ret, "PM startup exchange failed\n");
	platform_set_drvdata(pdev, pm);
	ret = sc9830_pm_register_clock(dev, pm);
	if (ret)
		return dev_err_probe(dev, ret, "PM clock registration failed\n");
	ret = devm_device_add_group(dev, &sc9830_pm_group);
	if (ret)
		return ret;
	dev_info(dev, "PM firmware frequency %u MHz\n", pm->value);
	return 0;
}

static const struct of_device_id sc9830_pm_match[] = {
	{ .compatible = "samsung,j320fn-pm" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_pm_match);

static struct platform_driver sc9830_pm_driver = {
	.probe = sc9830_pm_probe,
	.driver = {
		.name = "sc9830-pm",
		.of_match_table = sc9830_pm_match,
	},
};
module_platform_driver(sc9830_pm_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung J320FN PM firmware transport");
