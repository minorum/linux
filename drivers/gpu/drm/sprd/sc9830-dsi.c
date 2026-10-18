// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>

#include "sc9830-dphy.h"

struct sc9830_dsi {
	struct device *dev;
	struct dw_mipi_dsi *dsi;
	struct dw_mipi_dsi_plat_data pdata;
	void __iomem *base;
	struct regmap *aon;
	struct clk *ref;
	struct clk *cfg;
	unsigned int lanes;
};

static int sc9830_dsi_power_down(struct sc9830_dsi *dsi)
{
	int ret, second;

	ret = regmap_update_bits(dsi->aon, 0x24, BIT(15), BIT(15));
	second = regmap_update_bits(dsi->aon, 0x24, BIT(14), BIT(14));
	if (ret || second)
		dev_err(dsi->dev, "PHY power-down failed: %d\n", ret ?: second);
	return ret ?: second;
}

static int sc9830_dsi_runtime_resume(struct device *dev)
{
	struct sc9830_dsi *dsi = dev_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(dsi->aon, 0x24, BIT(15), 0);
	if (ret)
		goto power_down;
	ret = regmap_update_bits(dsi->aon, 0x24, BIT(14), 0);
	if (ret)
		goto power_down;
	ret = clk_prepare_enable(dsi->ref);
	if (ret)
		goto power_down;
	ret = clk_prepare_enable(dsi->cfg);
	if (ret) {
		sc9830_dsi_power_down(dsi);
		clk_disable_unprepare(dsi->ref);
		return ret;
	}
	return 0;

power_down:
	sc9830_dsi_power_down(dsi);
	return ret;
}

static int sc9830_dsi_runtime_suspend(struct device *dev)
{
	struct sc9830_dsi *dsi = dev_get_drvdata(dev);
	int ret;

	ret = sc9830_dsi_power_down(dsi);
	if (ret)
		return ret;
	clk_disable_unprepare(dsi->cfg);
	clk_disable_unprepare(dsi->ref);
	return 0;
}

static void sc9830_dsi_phy_power_off(void *data)
{
	struct sc9830_dsi *dsi = data;

	sc9830_dsi_power_down(dsi);
}

static int sc9830_dsi_phy_init(void *data)
{
	struct sc9830_dsi *dsi = data;

	return sc9830_dphy_configure_500(dsi->base, clk_get_rate(dsi->ref), dsi->lanes);
}

static enum drm_mode_status
sc9830_dsi_mode_valid(void *data, const struct drm_display_mode *mode,
		      unsigned long flags, u32 lanes, u32 format)
{
	if (!lanes || lanes > 4 || format != MIPI_DSI_FMT_RGB888 ||
	    !(flags & MIPI_DSI_MODE_VIDEO) || !(flags & MIPI_DSI_MODE_VIDEO_BURST))
		return MODE_BAD;
	if (mode->clock <= 0 || (u64)mode->clock * 24 > 499200ULL * lanes)
		return MODE_CLOCK_HIGH;
	return MODE_OK;
}

static int sc9830_dsi_lane_mbps(void *data, const struct drm_display_mode *mode,
				unsigned long flags, u32 lanes, u32 format,
				unsigned int *mbps)
{
	struct sc9830_dsi *dsi = data;

	if (sc9830_dsi_mode_valid(data, mode, flags, lanes, format) != MODE_OK)
		return -EINVAL;
	dsi->lanes = lanes;
	*mbps = 500;
	return 0;
}

static int sc9830_dsi_timing(void *data, unsigned int mbps,
			     struct dw_mipi_dsi_dphy_timing *timing)
{
	if (mbps != 500)
		return -EINVAL;
	*timing = (struct dw_mipi_dsi_dphy_timing) {
		.data_hs2lp = 4,
		.data_lp2hs = 15,
		.clk_hs2lp = 4,
		.clk_lp2hs = 15,
	};
	return 0;
}

static const struct dw_mipi_dsi_phy_ops sc9830_dsi_phy_ops = {
	.init = sc9830_dsi_phy_init,
	.power_off = sc9830_dsi_phy_power_off,
	.get_lane_mbps = sc9830_dsi_lane_mbps,
	.get_timing = sc9830_dsi_timing,
};

static int sc9830_dsi_attach(void *data, struct mipi_dsi_device *device)
{
	if (device->hs_rate && device->hs_rate != 500000000)
		return -EINVAL;
	return 0;
}

static const struct dw_mipi_dsi_host_ops sc9830_dsi_host_ops = {
	.attach = sc9830_dsi_attach,
};

static int sc9830_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc9830_dsi *dsi;
	struct clk *pclk;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;
	dsi->dev = dev;
	dsi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->base))
		return PTR_ERR(dsi->base);
	dsi->aon = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,aon");
	if (IS_ERR(dsi->aon))
		return dev_err_probe(dev, PTR_ERR(dsi->aon), "Missing AON syscon\n");
	dsi->ref = devm_clk_get(dev, "ref");
	if (IS_ERR(dsi->ref))
		return dev_err_probe(dev, PTR_ERR(dsi->ref), "Missing reference clock\n");
	dsi->cfg = devm_clk_get(dev, "cfg");
	if (IS_ERR(dsi->cfg))
		return dev_err_probe(dev, PTR_ERR(dsi->cfg), "Missing configuration clock\n");
	pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk), "Missing bus clock\n");
	if (clk_get_rate(dsi->ref) != 26000000)
		return dev_err_probe(dev, -EINVAL, "PHY requires 26MHz reference\n");
	platform_set_drvdata(pdev, dsi);
	dsi->pdata = (struct dw_mipi_dsi_plat_data) {
		.base = dsi->base,
		.max_data_lanes = 4,
		.mode_valid = sc9830_dsi_mode_valid,
		.phy_ops = &sc9830_dsi_phy_ops,
		.host_ops = &sc9830_dsi_host_ops,
		.priv_data = dsi,
	};
	ret = sc9830_dsi_runtime_resume(dev);
	if (ret)
		return ret;
	ret = clk_prepare_enable(pclk);
	if (ret)
		goto suspend;
	dsi->dsi = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	ret = PTR_ERR_OR_ZERO(dsi->dsi);
	sc9830_dsi_phy_power_off(dsi);
	clk_disable_unprepare(pclk);
suspend:
	if (sc9830_dsi_runtime_suspend(dev)) {
		clk_disable_unprepare(dsi->cfg);
		clk_disable_unprepare(dsi->ref);
		if (!ret)
			ret = -EIO;
	}
	if (ret && !IS_ERR_OR_NULL(dsi->dsi))
		dw_mipi_dsi_remove(dsi->dsi);
	return ret;
}

static void sc9830_dsi_remove(struct platform_device *pdev)
{
	struct sc9830_dsi *dsi = platform_get_drvdata(pdev);

	dw_mipi_dsi_remove(dsi->dsi);
}

static DEFINE_RUNTIME_DEV_PM_OPS(sc9830_dsi_pm_ops, sc9830_dsi_runtime_suspend,
				sc9830_dsi_runtime_resume, NULL);

static const struct of_device_id sc9830_dsi_of_match[] = {
	{ .compatible = "sprd,sc9830-dsi" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_dsi_of_match);

static struct platform_driver sc9830_dsi_driver = {
	.probe = sc9830_dsi_probe,
	.remove = sc9830_dsi_remove,
	.driver = {
		.name = "sc9830-dsi",
		.of_match_table = sc9830_dsi_of_match,
		.pm = pm_ptr(&sc9830_dsi_pm_ops),
	},
};
module_platform_driver(sc9830_dsi_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 MIPI DSI host");
MODULE_LICENSE("GPL");
