// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
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

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc9830_pm *pm = dev_get_drvdata(dev);
	u32 mhz;
	int ret;

	mutex_lock(&pm->lock);
	ret = sc9830_pm_exchange(pm, SC9830_PM_QUERY, SC9830_PM_QUERIED, pm->desired);
	mhz = pm->value;
	mutex_unlock(&pm->lock);
	return ret ? ret : sysfs_emit(buf, "%u\n", mhz * 1000000);
}
static DEVICE_ATTR_RO(frequency);

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
	ret = device_create_file(dev, &dev_attr_frequency);
	if (ret)
		return ret;
	dev_info(dev, "PM firmware frequency %u MHz\n", pm->value);
	return 0;
}

static void sc9830_pm_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_frequency);
}

static const struct of_device_id sc9830_pm_match[] = {
	{ .compatible = "samsung,j320fn-pm" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_pm_match);

static struct platform_driver sc9830_pm_driver = {
	.probe = sc9830_pm_probe,
	.remove = sc9830_pm_remove,
	.driver = {
		.name = "sc9830-pm",
		.of_match_table = sc9830_pm_match,
	},
};
module_platform_driver(sc9830_pm_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung J320FN PM firmware transport");
