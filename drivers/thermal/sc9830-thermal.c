// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/thermal.h>

#define SC9830_THM_CRITICAL BIT(5)
#define SC9830_THM_RAW_MASK 0x3fff

struct sc9830_thermal;

struct sc9830_sensor {
	struct sc9830_thermal *thermal;
	struct thermal_zone_device *tz;
	unsigned int id;
};

struct sc9830_thermal {
	void __iomem *base;
	struct sc9830_sensor sensors[2];
	struct device *dev;
	struct work_struct work;
	raw_spinlock_t lock;
	int irq;
	int trim;
};

static int sc9830_raw_to_temp(unsigned int raw, int trim, int *temp)
{
	if (!raw || raw > 0x7fff)
		return -EAGAIN;
	*temp = -40000 + div_s64(((s64)raw - 623) * 160000, 407) - trim * 1000;
	return 0;
}

static int sc9830_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct sc9830_sensor *sensor = thermal_zone_device_priv(tz);
	struct sc9830_thermal *thermal = sensor->thermal;
	u32 raw = readl(thermal->base + sensor->id * 0x100 + 0x60) & 0x7fff;

	return sc9830_raw_to_temp(raw, thermal->trim, temp);
}

static int sc9830_critical_raw(int temp, int trim, u32 *raw)
{
	if (temp < 0 || temp > 120000)
		return -EINVAL;
	*raw = 623 + div_s64(((s64)temp + 40000 + trim * 1000) * 407, 160000);
	return 0;
}

static const struct thermal_zone_device_ops sc9830_thermal_ops = {
	.get_temp = sc9830_get_temp,
};

static void sc9830_thermal_stop(void *data)
{
	struct sc9830_thermal *thermal = data;
	int id;

	for (id = 0; id < 2; id++)
		writel(0, thermal->base + id * 0x100 + 0x20);
	writel(0, thermal->base);
}

static void sc9830_thm_update(void __iomem *reg, u32 mask, u32 value)
{
	writel((readl(reg) & ~mask) | value, reg);
}

static void sc9830_mask_events(struct sc9830_thermal *thermal)
{
	int id;

	for (id = 0; id < 2; id++)
		sc9830_thm_update(thermal->base + id * 0x100 + 0x28,
				  SC9830_THM_CRITICAL, 0);
	sc9830_thm_update(thermal->base + 4, GENMASK(1, 0), 0);
}

static void sc9830_thermal_work(struct work_struct *work)
{
	struct sc9830_thermal *thermal = container_of(work, struct sc9830_thermal, work);
	int id;

	for (id = 0; id < 2; id++)
		thermal_zone_device_update(thermal->sensors[id].tz,
					   THERMAL_EVENT_UNSPECIFIED);
}

static irqreturn_t sc9830_thermal_irq(int irq, void *data)
{
	struct sc9830_thermal *thermal = data;
	unsigned long flags;
	bool pending = false;
	int id;

	raw_spin_lock_irqsave(&thermal->lock, flags);
	for (id = 0; id < 2; id++)
		pending |= !!(readl(thermal->base + id * 0x100 + 0x2c) &
			      SC9830_THM_CRITICAL);
	if (pending) {
		sc9830_mask_events(thermal);
		for (id = 0; id < 2; id++)
			writel(SC9830_THM_CRITICAL, thermal->base + id * 0x100 + 0x34);
	}
	raw_spin_unlock_irqrestore(&thermal->lock, flags);
	if (!pending)
		return IRQ_NONE;
	pm_wakeup_event(thermal->dev, 0);
	queue_work(system_freezable_wq, &thermal->work);
	return IRQ_HANDLED;
}

static int sc9830_arm(struct sc9830_thermal *thermal)
{
	u32 critical[2], raw;
	unsigned long flags;
	int id, ret, temp;

	raw_spin_lock_irqsave(&thermal->lock, flags);
	sc9830_mask_events(thermal);
	raw_spin_unlock_irqrestore(&thermal->lock, flags);
	synchronize_irq(thermal->irq);
	for (id = 0; id < 2; id++) {
		ret = thermal_zone_get_crit_temp(thermal->sensors[id].tz, &temp);
		if (ret)
			return ret;
		ret = sc9830_critical_raw(temp, thermal->trim, &critical[id]);
		if (ret)
			return ret;
		raw = readl(thermal->base + id * 0x100 + 0x60) & 0x7fff;
		if (!raw)
			return -EAGAIN;
		if (raw >= critical[id])
			return -EBUSY;
	}
	for (id = 0; id < 2; id++) {
		void __iomem *base = thermal->base + id * 0x100;

		sc9830_thm_update(base + 0x40, SC9830_THM_RAW_MASK, critical[id]);
		writel(SC9830_THM_CRITICAL, base + 0x34);
		sc9830_thm_update(base + 0x20, BIT(3), BIT(3));
		ret = readl_poll_timeout(base + 0x30, raw,
					 !(raw & SC9830_THM_CRITICAL), 100, 2000000);
		if (ret)
			return ret;
	}
	raw_spin_lock_irqsave(&thermal->lock, flags);
	for (id = 0; id < 2; id++)
		sc9830_thm_update(thermal->base + id * 0x100 + 0x28,
				  SC9830_THM_CRITICAL, SC9830_THM_CRITICAL);
	sc9830_thm_update(thermal->base + 4, GENMASK(1, 0), GENMASK(1, 0));
	raw_spin_unlock_irqrestore(&thermal->lock, flags);
	return 0;
}

static void sc9830_thermal_mask(void *data)
{
	struct sc9830_thermal *thermal = data;
	unsigned long flags;

	raw_spin_lock_irqsave(&thermal->lock, flags);
	sc9830_mask_events(thermal);
	raw_spin_unlock_irqrestore(&thermal->lock, flags);
}

static int sc9830_thermal_suspend(struct device *dev)
{
	struct sc9830_thermal *thermal = dev_get_drvdata(dev);

	if (!thermal->irq)
		return 0;
	if (device_may_wakeup(dev))
		return sc9830_arm(thermal);
	sc9830_thermal_mask(thermal);
	synchronize_irq(thermal->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc9830_thermal_pm_ops, sc9830_thermal_suspend, NULL);

static int sc9830_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct sc9830_thermal *thermal;
	struct thermal_zone_device *tz;
	struct clk_bulk_data *clocks;
	u32 field, raw;
	int count, ret, id;

	thermal = devm_kzalloc(dev, sizeof(*thermal), GFP_KERNEL);
	if (!thermal)
		return -ENOMEM;
	thermal->dev = dev;
	raw_spin_lock_init(&thermal->lock);
	platform_set_drvdata(pdev, thermal);
	ret = nvmem_cell_read_variable_le_u32(dev, "calibration", &field);
	if (ret)
		return dev_err_probe(dev, ret, "calibration unavailable\n");
	if (field > 31)
		return dev_err_probe(dev, -ENODATA, "invalid calibration\n");
	thermal->trim = field ? (int)field - 16 : 0;
	if (!field)
		dev_warn(dev, "unprogrammed trim; using nominal conversion\n");
	thermal->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(thermal->base))
		return PTR_ERR(thermal->base);
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "reset unavailable\n");
	count = devm_clk_bulk_get_all_enabled(dev, &clocks);
	if (count < 0)
		return dev_err_probe(dev, count, "clocks unavailable\n");
	if (count != 6)
		return -EINVAL;
	ret = reset_control_reset(reset);
	if (ret)
		return ret;
	writel(3, thermal->base);
	writel(0, thermal->base + 4);
	for (id = 0; id < 2; id++) {
		void __iomem *base = thermal->base + id * 0x100;

		writel(0, base + 0x28);
		sc9830_thm_update(base + 0x24, BIT(14), BIT(14));
		sc9830_thm_update(base + 0x5c, 0x21, 0x21);
		sc9830_thm_update(base + 0x58, 0x100, 0x400);
		if (!id)
			sc9830_thm_update(thermal->base + 0x120, 0x30, 0x30);
		sc9830_thm_update(base + 0x20, 0x30, 0x39);
	}
	ret = devm_add_action_or_reset(dev, sc9830_thermal_stop, thermal);
	if (ret)
		return ret;
	for (id = 0; id < 2; id++) {
		ret = readl_poll_timeout(thermal->base + id * 0x100 + 0x60,
					 raw, (raw & 0x7fff) != 0, 10000, 2000000);
		if (ret)
			return dev_err_probe(dev, ret, "sensor not ready\n");
		thermal->sensors[id].thermal = thermal;
		thermal->sensors[id].id = id;
		tz = devm_thermal_of_zone_register(dev, id, &thermal->sensors[id],
						   &sc9830_thermal_ops);
		if (IS_ERR(tz))
			return dev_err_probe(dev, PTR_ERR(tz), "thermal zone unavailable\n");
		thermal->sensors[id].tz = tz;
	}
	ret = platform_get_irq_optional(pdev, 0);
	if (ret < 0 && ret != -ENXIO)
		return dev_err_probe(dev, ret, "interrupt unavailable\n");
	if (ret > 0) {
		thermal->irq = ret;
		ret = devm_work_autocancel(dev, &thermal->work, sc9830_thermal_work);
		if (ret)
			return ret;
		if (device_property_read_bool(dev, "wakeup-source")) {
			ret = devm_device_init_wakeup(dev);
			if (ret)
				return ret;
		}
		ret = devm_request_irq(dev, thermal->irq, sc9830_thermal_irq, 0,
				       dev_name(dev), thermal);
		if (ret)
			return dev_err_probe(dev, ret, "interrupt request failed\n");
		ret = devm_pm_set_wake_irq(dev, thermal->irq);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, sc9830_thermal_mask, thermal);
		if (ret)
			return ret;
		ret = sc9830_arm(thermal);
		if (ret && ret != -EBUSY)
			return dev_err_probe(dev, ret, "critical wake setup failed\n");
	}
	dev_info(dev, "two die sensors, calibration %d C\n", thermal->trim);
	return 0;
}

static const struct of_device_id sc9830_thermal_match[] = {
	{ .compatible = "sprd,sc9830-thermal" },
	{}
};
MODULE_DEVICE_TABLE(of, sc9830_thermal_match);

static struct platform_driver sc9830_thermal_driver = {
	.probe = sc9830_thermal_probe,
	.driver = {
		.name = "sc9830-thermal",
		.of_match_table = sc9830_thermal_match,
		.pm = pm_sleep_ptr(&sc9830_thermal_pm_ops),
	},
};
module_platform_driver(sc9830_thermal_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 die temperature sensors");
MODULE_LICENSE("GPL");
