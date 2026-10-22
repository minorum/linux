// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>

struct sc9830_thermal;

struct sc9830_sensor {
	struct sc9830_thermal *thermal;
	unsigned int id;
};

struct sc9830_thermal {
	void __iomem *base;
	struct sc9830_sensor sensors[2];
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
	},
};
module_platform_driver(sc9830_thermal_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 die temperature sensors");
MODULE_LICENSE("GPL");
