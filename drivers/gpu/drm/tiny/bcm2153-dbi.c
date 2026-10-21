// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Based on ili9341.c
 * Copyright 2018 David Lechner <david@lechnology.com>
 * Copyright 2016 Noralf Trønnes
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>
#include <video/mipi_display.h>

#define DBI_COMMAND	0x00
#define DBI_DATA		0x04
#define DBI_READ_REQUEST	0x08
#define DBI_WRITE_TIMING	0x10
#define DBI_READ_TIMING	0x14
#define DBI_CONTROL	0x18
#define DBI_STATUS	0x1c
#define DBI_FIFO_FULL		BIT(30)
#define DBI_PANEL_SELECT	BIT(26)
#define DBI_EIGHT_BIT		BIT(30)
#define DBI_REQUEST_READY	BIT(31)
#define DBI_DATA_READY		BIT(21)

struct bcm2153_dbi {
	struct mipi_dbi_dev dbidev;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	void __iomem *base;
	int error;
};

static int bcm2153_dbi_write(struct bcm2153_dbi *lcd, u32 reg, u32 value)
{
	u32 status;
	int ret;

	if (lcd->error)
		return lcd->error;
	ret = readl_poll_timeout(lcd->base + DBI_STATUS, status,
				 !(status & DBI_FIFO_FULL), 0, 10000);
	if (ret) {
		lcd->error = ret;
		return ret;
	}
	writel(value, lcd->base + reg);
	return 0;
}

static int bcm2153_dbi_setup(struct bcm2153_dbi *lcd)
{
	int ret;

	ret = bcm2153_dbi_write(lcd, DBI_WRITE_TIMING, 0x10301);
	if (ret)
		return ret;
	ret = bcm2153_dbi_write(lcd, DBI_READ_TIMING, 0x11101);
	if (ret)
		return ret;
	return bcm2153_dbi_write(lcd, DBI_CONTROL,
				 readl(lcd->base + DBI_CONTROL) & ~DBI_EIGHT_BIT);
}

static int bcm2153_dbi_command(struct mipi_dbi *dbi, u8 *cmd, u8 *data,
			       size_t len)
{
	struct mipi_dbi_dev *dbidev = container_of(dbi, struct mipi_dbi_dev, dbi);
	struct bcm2153_dbi *lcd = container_of(dbidev, struct bcm2153_dbi, dbidev);
	size_t i;
	u32 status;
	int ret;

	if (*cmd == MIPI_DCS_GET_DISPLAY_ID && (len != 4 || !data))
		return -EINVAL;
	if (*cmd == MIPI_DCS_WRITE_MEMORY_START && len % sizeof(u16))
		return -EINVAL;
	if (lcd->error)
		return lcd->error;
	ret = readl_poll_timeout(lcd->base + DBI_STATUS, status,
				 !(status & DBI_FIFO_FULL), 0, 10000);
	if (ret) {
		lcd->error = ret;
		return ret;
	}
	writel(readl(lcd->base + DBI_CONTROL) & ~DBI_PANEL_SELECT,
	       lcd->base + DBI_CONTROL);
	ret = bcm2153_dbi_write(lcd, DBI_COMMAND, *cmd);
	if (ret)
		return ret;
	if (*cmd == MIPI_DCS_GET_DISPLAY_ID) {
		msleep(25);
		for (i = 0; i < len; i++) {
			ret = readl_poll_timeout(lcd->base + DBI_STATUS, status,
						 status & DBI_REQUEST_READY, 0, 10000);
			if (ret)
				goto read_error;
			writel(0, lcd->base + DBI_READ_REQUEST);
			ret = readl_poll_timeout(lcd->base + DBI_STATUS, status,
						 status & DBI_DATA_READY, 0, 10000);
			if (ret)
				goto read_error;
			data[i] = readl(lcd->base + DBI_DATA);
		}
		return 0;
	}
	for (i = 0; i < len;) {
		u32 value;

		if (*cmd == MIPI_DCS_WRITE_MEMORY_START) {
			value = get_unaligned_le16(data + i);
			i += sizeof(u16);
		} else {
			value = data[i++];
		}
		ret = bcm2153_dbi_write(lcd, DBI_DATA, value);
		if (ret)
			return ret;
	}
	return 0;

read_error:
	lcd->error = ret;
	return ret;
}

static const u32 bcm2153_dbi_formats[] = { DRM_MIPI_DBI_PLANE_FORMATS };
static const u64 bcm2153_dbi_modifiers[] = { DRM_MIPI_DBI_PLANE_FORMAT_MODIFIERS };

struct bcm2153_panel_command {
	u8 command;
	u8 length;
	u8 data[15];
	u16 delay_ms;
};

static const struct bcm2153_panel_command bcm2153_panel_init[] = {
	{ 0, 0, { 0 }, 10 },
	{ 0xf3, 7, { 0x80, 0x00, 0x00, 0x02, 0x44, 0x3c, 0x3c }, 0 },
	{ 0xf4, 5, { 0x33, 0x33, 0x5a, 0x5a, 0x33 }, 0 },
	{ 0xf5, 5, { 0x12, 0x00, 0x03, 0xf0, 0x70 }, 0 },
	{ 0x11, 0, { 0 }, 120 },
	{ 0x35, 1, { 0x00 }, 0 },
	{ 0x36, 1, { 0x08 }, 0 },
	{ 0x3a, 1, { 0x05 }, 0 },
	{ 0xf2, 11, { 0x14, 0x14, 0x03, 0x08, 0x08, 0x08, 0x08, 0x10, 0x06, 0x15, 0x15 }, 0 },
	{ 0xf6, 2, { 0x00, 0x80 }, 0 },
	{ 0xfd, 2, { 0x22, 0x01 }, 0 },
	{ 0xf7, 15, { 0x00, 0x25, 0x2f, 0x1b, 0x21, 0x27, 0x2c, 0x33,
		      0x0a, 0x0d, 0x1a, 0x15, 0x15, 0x22, 0x22 }, 0 },
	{ 0xf8, 15, { 0x25, 0x00, 0x2f, 0x1b, 0x21, 0x27, 0x2c, 0x33,
		      0x0a, 0x0d, 0x1a, 0x15, 0x15, 0x22, 0x22 }, 0 },
	{ 0xf9, 15, { 0x00, 0x25, 0x0a, 0x24, 0x21, 0x26, 0x2b, 0x34,
		      0x0d, 0x0a, 0x18, 0x15, 0x15, 0x22, 0x22 }, 0 },
	{ 0xfa, 15, { 0x25, 0x00, 0x0a, 0x24, 0x21, 0x26, 0x2b, 0x34,
		      0x0d, 0x0a, 0x18, 0x15, 0x15, 0x22, 0x22 }, 0 },
	{ 0xfb, 15, { 0x00, 0x23, 0x2a, 0x09, 0x23, 0x24, 0x26, 0x2b,
		      0x1a, 0x1d, 0x3e, 0x15, 0x15, 0x22, 0x22 }, 0 },
	{ 0xfc, 15, { 0x23, 0x00, 0x2a, 0x09, 0x23, 0x24, 0x26, 0x2b,
		      0x1a, 0x1d, 0x3e, 0x15, 0x15, 0x22, 0x22 }, 0 },
};

static int bcm2153_dbi_init_panel(struct mipi_dbi *dbi)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(bcm2153_panel_init); i++) {
		const struct bcm2153_panel_command *op = &bcm2153_panel_init[i];

		if (op->command) {
			ret = mipi_dbi_command_stackbuf(dbi, op->command, op->data, op->length);
			if (ret)
				return ret;
		}
		if (op->delay_ms)
			msleep(op->delay_ms);
	}
	return 0;
}

static int bcm2153_dbi_prepare(struct bcm2153_dbi *lcd)
{
	struct mipi_dbi *dbi = &lcd->dbidev.dbi;
	u8 id[4];
	int ret;

	if (!dbi->reset)
		return 0;
	ret = gpiod_direction_output(dbi->reset, 1);
	if (ret)
		return ret;
	usleep_range(2000, 2500);
	ret = gpiod_set_value_cansleep(dbi->reset, 0);
	if (ret)
		return ret;
	usleep_range(10000, 11000);
	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_GET_DISPLAY_ID, id, sizeof(id));
	if (ret)
		return ret;
	if (id[1] != 0x52 || (id[2] != 0x09 && id[2] != 0x0a) || id[3] != 0x11) {
		drm_err(&lcd->dbidev.drm, "Unsupported panel ID: %*ph\n", 4, id);
		return -ENODEV;
	}
	ret = bcm2153_dbi_init_panel(dbi);
	if (!ret)
		drm_info(&lcd->dbidev.drm, "Initialized panel ID: %*ph\n", 4, id);
	return ret;
}

static const struct drm_plane_helper_funcs bcm2153_dbi_plane_helpers = {
	DRM_MIPI_DBI_PLANE_HELPER_FUNCS,
};

static const struct drm_plane_funcs bcm2153_dbi_plane_funcs = {
	DRM_MIPI_DBI_PLANE_FUNCS,
	.destroy = drm_plane_cleanup,
};

static void bcm2153_dbi_enable(struct drm_crtc *crtc,
			       struct drm_atomic_commit *state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(crtc->dev);
	struct bcm2153_dbi *lcd = container_of(dbidev, struct bcm2153_dbi, dbidev);
	int ret, idx;

	if (!drm_dev_enter(crtc->dev, &idx))
		return;
	ret = bcm2153_dbi_prepare(lcd);
	if (!ret)
		ret = mipi_dbi_command_buf(&dbidev->dbi, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	if (ret) {
		lcd->error = ret;
		drm_err(crtc->dev, "Failed to enable display: %d\n", ret);
	}
	drm_dev_exit(idx);
}

static void bcm2153_dbi_disable(struct drm_crtc *crtc,
				struct drm_atomic_commit *state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(crtc->dev);
	int ret, idx;

	if (!drm_dev_enter(crtc->dev, &idx))
		return;
	ret = mipi_dbi_command_buf(&dbidev->dbi, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	if (ret)
		drm_err(crtc->dev, "Failed to disable display: %d\n", ret);
	drm_dev_exit(idx);
}

static const struct drm_crtc_helper_funcs bcm2153_dbi_crtc_helpers = {
	.mode_valid = drm_mipi_dbi_crtc_helper_mode_valid,
	.atomic_check = drm_mipi_dbi_crtc_helper_atomic_check,
	.atomic_enable = bcm2153_dbi_enable,
	.atomic_disable = bcm2153_dbi_disable,
};

static const struct drm_crtc_funcs bcm2153_dbi_crtc_funcs = {
	DRM_MIPI_DBI_CRTC_FUNCS,
	.destroy = drm_crtc_cleanup,
};

static const struct drm_encoder_funcs bcm2153_dbi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_connector_helper_funcs bcm2153_dbi_connector_helpers = {
	DRM_MIPI_DBI_CONNECTOR_HELPER_FUNCS,
};

static const struct drm_connector_funcs bcm2153_dbi_connector_funcs = {
	DRM_MIPI_DBI_CONNECTOR_FUNCS,
	.destroy = drm_connector_cleanup,
};

static const struct drm_mode_config_helper_funcs bcm2153_dbi_config_helpers = {
	DRM_MIPI_DBI_MODE_CONFIG_HELPER_FUNCS,
};

static const struct drm_mode_config_funcs bcm2153_dbi_config_funcs = {
	DRM_MIPI_DBI_MODE_CONFIG_FUNCS,
};

static const struct drm_display_mode bcm2153_monte_mode = {
	DRM_SIMPLE_MODE(240, 400, 0, 0),
};

DEFINE_DRM_GEM_DMA_FOPS(bcm2153_dbi_fops);

static const struct drm_driver bcm2153_dbi_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &bcm2153_dbi_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "bcm2153-dbi",
	.desc = "Broadcom BCM2153 DBI",
	.major = 1,
};

static int bcm2153_dbi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *panel;
	struct gpio_desc *reset = NULL;
	struct bcm2153_dbi *lcd;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct clk *clk;
	int ret;

	panel = of_get_child_by_name(dev->of_node, "panel");
	if (!panel)
		return -ENODEV;
	ret = of_device_is_available(panel) &&
	      of_device_is_compatible(panel, "samsung,gt-s5620-lcd") ? 0 : -ENODEV;
	if (!ret)
		reset = devm_fwnode_gpiod_get_optional(dev, of_fwnode_handle(panel),
						     "reset", GPIOD_ASIS, "gt-s5620-panel-reset");
	of_node_put(panel);
	if (ret)
		return ret;
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get panel reset\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	lcd = devm_drm_dev_alloc(dev, &bcm2153_dbi_driver, struct bcm2153_dbi, dbidev.drm);
	if (IS_ERR(lcd))
		return PTR_ERR(lcd);
	lcd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcd->base))
		return PTR_ERR(lcd->base);
	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to enable clock\n");
	ret = bcm2153_dbi_setup(lcd);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize controller\n");

	dbidev = &lcd->dbidev;
	drm = &dbidev->drm;
	mutex_init(&dbidev->dbi.cmdlock);
	dbidev->dbi.dev = dev;
	dbidev->dbi.command = bcm2153_dbi_command;
	dbidev->dbi.reset = reset;
	ret = drm_mipi_dbi_dev_init(dbidev, &bcm2153_monte_mode, DRM_FORMAT_RGB565, 0, 0);
	if (ret)
		return ret;
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = dbidev->mode.hdisplay;
	drm->mode_config.max_width = dbidev->mode.hdisplay;
	drm->mode_config.min_height = dbidev->mode.vdisplay;
	drm->mode_config.max_height = dbidev->mode.vdisplay;
	drm->mode_config.funcs = &bcm2153_dbi_config_funcs;
	drm->mode_config.helper_private = &bcm2153_dbi_config_helpers;
	drm->mode_config.preferred_depth = 16;
	ret = drm_universal_plane_init(drm, &lcd->plane, 0, &bcm2153_dbi_plane_funcs,
				       bcm2153_dbi_formats, ARRAY_SIZE(bcm2153_dbi_formats),
				       bcm2153_dbi_modifiers, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&lcd->plane, &bcm2153_dbi_plane_helpers);
	drm_plane_enable_fb_damage_clips(&lcd->plane);
	ret = drm_crtc_init_with_planes(drm, &lcd->crtc, &lcd->plane, NULL,
				       &bcm2153_dbi_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&lcd->crtc, &bcm2153_dbi_crtc_helpers);
	ret = drm_encoder_init(drm, &lcd->encoder, &bcm2153_dbi_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	lcd->encoder.possible_crtcs = drm_crtc_mask(&lcd->crtc);
	ret = drm_connector_init(drm, &lcd->connector, &bcm2153_dbi_connector_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;
	drm_connector_helper_add(&lcd->connector, &bcm2153_dbi_connector_helpers);
	ret = drm_connector_attach_encoder(&lcd->connector, &lcd->encoder);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, drm);
	drm_client_setup(drm, NULL);
	return 0;
}

static void bcm2153_dbi_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void bcm2153_dbi_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static const struct of_device_id bcm2153_dbi_of_match[] = {
	{ .compatible = "brcm,bcm2153-dbi" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_dbi_of_match);

static struct platform_driver bcm2153_dbi_platform_driver = {
	.probe = bcm2153_dbi_probe,
	.remove = bcm2153_dbi_remove,
	.shutdown = bcm2153_dbi_shutdown,
	.driver = {
		.name = "bcm2153-dbi",
		.of_match_table = bcm2153_dbi_of_match,
	},
};
module_platform_driver(bcm2153_dbi_platform_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 parallel DBI display driver");
MODULE_LICENSE("GPL");
