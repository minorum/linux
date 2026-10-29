// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct freetab_panel {
	struct drm_panel panel;
	struct gpio_descs *controls;
};

static const struct drm_display_mode freetab_mode = {
	.clock = 69333,
	.hdisplay = 1024,
	.hsync_start = 1024 + 40,
	.hsync_end = 1024 + 40 + 128,
	.htotal = 1024 + 40 + 128 + 152,
	.vdisplay = 768,
	.vsync_start = 768 + 10,
	.vsync_end = 768 + 10 + 3,
	.vtotal = 768 + 10 + 3 + 12,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static void freetab_power_down(struct freetab_panel *ctx, unsigned int enabled)
{
	static const unsigned int order[] = { 2, 5, 4, 3, 1 };
	static const unsigned int delays[] = { 120, 50, 10, 10, 0 };
	unsigned int i;

	usleep_range(10000, 11000);
	for (i = 0; i < ARRAY_SIZE(order); i++) {
		if (!(enabled & BIT(order[i])))
			continue;
		gpiod_set_value_cansleep(ctx->controls->desc[order[i]], 0);
		if (delays[i])
			fsleep(delays[i] * 1000);
	}
}

static int freetab_prepare(struct drm_panel *panel)
{
	static const unsigned int delays[] = { 100, 50, 50, 50, 120, 10 };
	struct freetab_panel *ctx = container_of(panel, struct freetab_panel, panel);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(delays); i++) {
		ret = gpiod_direction_output(ctx->controls->desc[i], 1);
		if (ret) {
			freetab_power_down(ctx, BIT(i) - 1);
			return ret;
		}
		fsleep(delays[i] * 1000);
	}
	return 0;
}

static int freetab_unprepare(struct drm_panel *panel)
{
	struct freetab_panel *ctx = container_of(panel, struct freetab_panel, panel);

	freetab_power_down(ctx, GENMASK(5, 0));
	return 0;
}

static int freetab_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	const u32 format = MEDIA_BUS_FMT_RGB888_1X24;
	struct drm_display_mode *mode;
	int ret;

	ret = drm_display_info_set_bus_formats(&connector->display_info, &format, 1);
	if (ret)
		return ret;
	connector->display_info.bpc = 8;
	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH |
		DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;
	mode = drm_mode_duplicate(connector->dev, &freetab_mode);
	if (!mode)
		return -ENOMEM;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	return 1;
}

static const struct drm_panel_funcs freetab_funcs = {
	.prepare = freetab_prepare,
	.unprepare = freetab_unprepare,
	.get_modes = freetab_get_modes,
};

static int freetab_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct freetab_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct freetab_panel, panel,
				   &freetab_funcs, DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;
	ctx->controls = devm_gpiod_get_array(dev, "modecom,control", GPIOD_ASIS);
	if (IS_ERR(ctx->controls))
		return dev_err_probe(dev, PTR_ERR(ctx->controls), "Missing control GPIOs\n");
	if (ctx->controls->ndescs != 6)
		return dev_err_probe(dev, -EINVAL, "Expected six control GPIOs\n");
	platform_set_drvdata(pdev, ctx);
	drm_panel_add(&ctx->panel);
	return 0;
}

static void freetab_shutdown(struct platform_device *pdev)
{
	struct freetab_panel *ctx = platform_get_drvdata(pdev);

	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
}

static void freetab_remove(struct platform_device *pdev)
{
	struct freetab_panel *ctx = platform_get_drvdata(pdev);

	drm_panel_remove(&ctx->panel);
	freetab_shutdown(pdev);
}

static const struct of_device_id freetab_of_match[] = {
	{ .compatible = "modecom,freetab8001-display" },
	{ }
};
MODULE_DEVICE_TABLE(of, freetab_of_match);

static struct platform_driver freetab_driver = {
	.probe = freetab_probe,
	.remove = freetab_remove,
	.shutdown = freetab_shutdown,
	.driver = {
		.name = "panel-modecom-freetab8001",
		.of_match_table = freetab_of_match,
	},
};
module_platform_driver(freetab_driver);

MODULE_DESCRIPTION("MODECOM FreeTAB 8001 display assembly");
MODULE_LICENSE("GPL");
