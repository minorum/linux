// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_module.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#define DPI_EN			0x00
#define DPI_CNTL		0x04
#define DPI_INT_ENABLE		0x08
#define DPI_INT_STATUS		0x0c
#define DPI_SIZE		0x10
#define DPI_CLK_CNTL		0x14
#define DPI_FB_ADDR		0x30
#define DPI_FB_PITCH		0x34
#define DPI_FIFO_TH		0x60
#define DPI_FIFO_STEP		0x64
#define DPI_REQ_TH		0x68
#define DPI_HCNTL		0x70
#define DPI_VCNTL		0x74
#define DPI_CURSOR_CNTL		0x80
#define DPI_GMC_CNTL		0x90
#define DPI_TMODE		0x94
#define DPI_INT_VSYNC		BIT(8)

struct mt8377_dpi {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_bridge *bridge;
	void __iomem *regs;
	struct regmap *mmsys;
	struct clk_bulk_data clocks[3];
	struct clk *pll;
	int irq;
	bool enabled;
};

#define to_mt8377_dpi(p) container_of(p, struct mt8377_dpi, pipe)

struct mt8377_crtc_state {
	struct drm_crtc_state base;
	u32 bus_flags;
};

#define to_mt8377_crtc_state(s) container_of(s, struct mt8377_crtc_state, base)

static void mt8377_dpi_destroy_crtc_state(struct drm_simple_display_pipe *pipe,
					  struct drm_crtc_state *crtc_state)
{
	struct mt8377_crtc_state *state = to_mt8377_crtc_state(crtc_state);

	__drm_atomic_helper_crtc_destroy_state(crtc_state);
	kfree(state);
}

static void mt8377_dpi_reset_crtc(struct drm_simple_display_pipe *pipe)
{
	struct mt8377_crtc_state *state;

	if (pipe->crtc.state)
		mt8377_dpi_destroy_crtc_state(pipe, pipe->crtc.state);
	state = kzalloc_obj(*state);
	__drm_atomic_helper_crtc_reset(&pipe->crtc, state ? &state->base : NULL);
}

static struct drm_crtc_state *
mt8377_dpi_duplicate_crtc_state(struct drm_simple_display_pipe *pipe)
{
	struct mt8377_crtc_state *state;

	if (!pipe->crtc.state)
		return NULL;
	state = kmemdup(to_mt8377_crtc_state(pipe->crtc.state), sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;
	__drm_atomic_helper_crtc_duplicate_state(&pipe->crtc, &state->base);
	return &state->base;
}

static int mt8377_dpi_clock(struct mt8377_dpi *dpi, unsigned long rate,
			    unsigned long *pll_rate)
{
	unsigned long best_error = ULONG_MAX;
	unsigned int div, best_div = 0;
	long rounded;

	if (!rate || rate > ULONG_MAX / 32)
		return -EINVAL;

	for (div = 2; div <= 32; div++) {
		unsigned long actual, error;

		rounded = clk_round_rate(dpi->pll, rate * div);
		if (rounded <= 0)
			continue;
		actual = rounded / div;
		error = actual > rate ? actual - rate : rate - actual;
		if (error < best_error) {
			best_error = error;
			best_div = div;
			*pll_rate = rounded;
		}
	}

	if (!best_div || best_error > rate / 100)
		return -EINVAL;

	return best_div;
}

static enum drm_mode_status mt8377_dpi_mode_valid(struct drm_simple_display_pipe *pipe,
						  const struct drm_display_mode *mode)
{
	unsigned long pll_rate;
	int hfp = mode->hsync_start - mode->hdisplay;
	int hpw = mode->hsync_end - mode->hsync_start;
	int hbp = mode->htotal - mode->hsync_end;
	int vfp = mode->vsync_start - mode->vdisplay;
	int vpw = mode->vsync_end - mode->vsync_start;
	int vbp = mode->vtotal - mode->vsync_end;

	if (mode->flags & ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NHSYNC |
			    DRM_MODE_FLAG_PVSYNC | DRM_MODE_FLAG_NVSYNC) ||
	    mode->vscan > 1)
		return MODE_BAD;
	if (mode->hdisplay < 1 || mode->hdisplay > U16_MAX ||
	    hfp < 1 || hfp > 256 || hpw < 1 || hpw > 256 || hbp < 1 || hbp > 256)
		return MODE_BAD_HVALUE;
	if (mode->vdisplay < 1 || mode->vdisplay > U16_MAX ||
	    vfp < 1 || vfp > 256 || vpw < 1 || vpw > 256 || vbp < 1 || vbp > 256)
		return MODE_BAD_VVALUE;
	if (mode->clock <= 0 || mode->clock > ULONG_MAX / 32000)
		return MODE_CLOCK_RANGE;
	if (mt8377_dpi_clock(to_mt8377_dpi(pipe), mode->clock * 1000UL, &pll_rate) < 0)
		return MODE_NOCLOCK;
	return MODE_OK;
}

static int mt8377_dpi_check(struct drm_simple_display_pipe *pipe,
			    struct drm_plane_state *plane,
			    struct drm_crtc_state *crtc)
{
	struct mt8377_dpi *dpi = to_mt8377_dpi(pipe);
	struct drm_bridge_state *bridge;
	struct mt8377_crtc_state *state = to_mt8377_crtc_state(crtc);

	if (!crtc->active)
		return 0;
	if (mt8377_dpi_mode_valid(pipe, &crtc->adjusted_mode) != MODE_OK)
		return -EINVAL;
	bridge = drm_atomic_get_new_bridge_state(crtc->state, dpi->bridge);
	if (bridge && bridge->input_bus_cfg.format != MEDIA_BUS_FMT_RGB888_1X24 &&
	    bridge->input_bus_cfg.format != MEDIA_BUS_FMT_FIXED)
		return -EINVAL;
	if (bridge)
		state->bus_flags = bridge->input_bus_cfg.flags;
	else if (dpi->bridge->timings)
		state->bus_flags = dpi->bridge->timings->input_bus_flags;
	if (!plane->fb)
		return 0;
	if (plane->fb->pitches[0] > U16_MAX || plane->fb->pitches[0] % 8 ||
	    drm_fb_dma_get_gem_addr(plane->fb, plane, 0) % 8)
		return -EINVAL;
	return 0;
}

static void mt8377_dpi_complete_event(struct drm_crtc *crtc)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void mt8377_dpi_update(struct drm_simple_display_pipe *pipe,
			      struct drm_plane_state *old_state)
{
	struct mt8377_dpi *dpi = to_mt8377_dpi(pipe);
	struct drm_plane_state *plane = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	unsigned long flags;
	u32 control;

	if (!dpi->enabled) {
		if (!crtc->state->active || !drm_atomic_crtc_needs_modeset(crtc->state))
			mt8377_dpi_complete_event(crtc);
		return;
	}
	if (!plane->fb)
		return;

	writel(drm_fb_dma_get_gem_addr(plane->fb, plane, 0), dpi->regs + DPI_FB_ADDR);
	writel(plane->fb->pitches[0], dpi->regs + DPI_FB_PITCH);
	control = readl(dpi->regs + DPI_CNTL) & ~GENMASK(5, 4);
	if (plane->fb->format->format == DRM_FORMAT_RGB888)
		control |= BIT(4);
	writel(control, dpi->regs + DPI_CNTL);
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->event) {
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		else
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void mt8377_dpi_enable(struct drm_simple_display_pipe *pipe,
			      struct drm_crtc_state *crtc,
			      struct drm_plane_state *plane)
{
	struct mt8377_dpi *dpi = to_mt8377_dpi(pipe);
	const struct drm_display_mode *m = &crtc->adjusted_mode;
	unsigned long pll_rate;
	u32 hsync, vsync, clock, bus_flags = to_mt8377_crtc_state(crtc)->bus_flags;
	int div, ret;

	div = mt8377_dpi_clock(dpi, m->clock * 1000UL, &pll_rate);
	if (div < 0) {
		ret = div;
		goto fail;
	}
	ret = pm_runtime_resume_and_get(dpi->drm.dev);
	if (ret < 0)
		goto fail;
	ret = clk_set_rate(dpi->pll, pll_rate);
	if (ret)
		goto put_runtime;
	ret = clk_prepare_enable(dpi->pll);
	if (ret)
		goto put_runtime;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(dpi->clocks), dpi->clocks);
	if (ret)
		goto disable_pll;
	writel(0, dpi->regs + DPI_EN);
	ret = regmap_update_bits(dpi->mmsys, 0x900, GENMASK(2, 0), BIT(0));
	if (ret)
		goto disable_engine;

	clock = (div - 1) | ((div / 2) << 8);
	if (bus_flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		clock |= BIT(31);
	hsync = (m->hsync_end - m->hsync_start - 1) |
		((m->htotal - m->hsync_end - 1) << 8) |
		((m->hsync_start - m->hdisplay - 1) << 16);
	if (m->flags & DRM_MODE_FLAG_PHSYNC)
		hsync |= BIT(24);
	if (bus_flags & DRM_BUS_FLAG_DE_LOW)
		hsync |= BIT(25);
	vsync = (m->vsync_end - m->vsync_start - 1) |
		((m->vtotal - m->vsync_end - 1) << 8) |
		((m->vsync_start - m->vdisplay - 1) << 16);
	if (m->flags & DRM_MODE_FLAG_PVSYNC)
		vsync |= BIT(24);

	writel(0, dpi->regs + DPI_INT_ENABLE);
	writel(0, dpi->regs + DPI_INT_STATUS);
	writel(BIT(12) | BIT(13) | BIT(14) |
	       (plane->fb->format->format == DRM_FORMAT_RGB888 ? BIT(4) : 0),
	       dpi->regs + DPI_CNTL);
	writel((u32)m->hdisplay | ((u32)m->vdisplay << 16), dpi->regs + DPI_SIZE);
	writel(clock, dpi->regs + DPI_CLK_CNTL);
	writel(hsync, dpi->regs + DPI_HCNTL);
	writel(vsync, dpi->regs + DPI_VCNTL);
	writel(400 | (496 << 16), dpi->regs + DPI_FIFO_TH);
	writel(8 | (8 << 16), dpi->regs + DPI_FIFO_STEP);
	writel(512, dpi->regs + DPI_REQ_TH);
	writel(0, dpi->regs + DPI_CURSOR_CNTL);
	writel(0, dpi->regs + DPI_TMODE);
	writel(0x41, dpi->regs + DPI_GMC_CNTL);
	writel(drm_fb_dma_get_gem_addr(plane->fb, plane, 0), dpi->regs + DPI_FB_ADDR);
	writel(plane->fb->pitches[0], dpi->regs + DPI_FB_PITCH);
	dpi->enabled = true;
	enable_irq(dpi->irq);
	writel(1, dpi->regs + DPI_EN);
	drm_crtc_vblank_on(&pipe->crtc);
	mt8377_dpi_update(pipe, NULL);
	return;

disable_engine:
	clk_bulk_disable_unprepare(ARRAY_SIZE(dpi->clocks), dpi->clocks);
disable_pll:
	clk_disable_unprepare(dpi->pll);
put_runtime:
	pm_runtime_put(dpi->drm.dev);
fail:
	dev_err(dpi->drm.dev, "Cannot enable DPI: %d\n", ret);
	mt8377_dpi_complete_event(&pipe->crtc);
}

static void mt8377_dpi_disable(struct drm_simple_display_pipe *pipe)
{
	struct mt8377_dpi *dpi = to_mt8377_dpi(pipe);

	if (!dpi->enabled)
		goto complete;
	drm_crtc_vblank_off(&pipe->crtc);
	disable_irq(dpi->irq);
	writel(0, dpi->regs + DPI_EN);
	dpi->enabled = false;
	clk_bulk_disable_unprepare(ARRAY_SIZE(dpi->clocks), dpi->clocks);
	clk_disable_unprepare(dpi->pll);
	pm_runtime_put(dpi->drm.dev);
complete:
	if (!pipe->crtc.state->active)
		mt8377_dpi_complete_event(&pipe->crtc);
}

static int mt8377_dpi_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct mt8377_dpi *dpi = to_mt8377_dpi(pipe);

	if (!dpi->enabled)
		return -EINVAL;
	writel(DPI_INT_VSYNC, dpi->regs + DPI_INT_ENABLE);
	return 0;
}

static void mt8377_dpi_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct mt8377_dpi *dpi = to_mt8377_dpi(pipe);

	if (dpi->enabled)
		writel(0, dpi->regs + DPI_INT_ENABLE);
}

static irqreturn_t mt8377_dpi_irq(int irq, void *data)
{
	struct mt8377_dpi *dpi = data;
	u32 status = readl(dpi->regs + DPI_INT_STATUS);

	if (!(status & DPI_INT_VSYNC))
		return IRQ_NONE;
	writel(0, dpi->regs + DPI_INT_STATUS);
	drm_crtc_handle_vblank(&dpi->pipe.crtc);
	return IRQ_HANDLED;
}

static const struct drm_simple_display_pipe_funcs mt8377_dpi_pipe_funcs = {
	.reset_crtc = mt8377_dpi_reset_crtc,
	.duplicate_crtc_state = mt8377_dpi_duplicate_crtc_state,
	.destroy_crtc_state = mt8377_dpi_destroy_crtc_state,
	.mode_valid = mt8377_dpi_mode_valid,
	.check = mt8377_dpi_check,
	.enable = mt8377_dpi_enable,
	.disable = mt8377_dpi_disable,
	.update = mt8377_dpi_update,
	.enable_vblank = mt8377_dpi_enable_vblank,
	.disable_vblank = mt8377_dpi_disable_vblank,
};

static const struct drm_mode_config_funcs mt8377_dpi_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(mt8377_dpi_fops);

static const struct drm_driver mt8377_dpi_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.name = "mt8377-dpi",
	.desc = "MediaTek MT8377 DPI",
	.major = 1,
	.fops = &mt8377_dpi_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static int mt8377_dpi_probe(struct platform_device *pdev)
{
	static const u32 formats[] = { DRM_FORMAT_RGB565, DRM_FORMAT_RGB888 };
	struct device *dev = &pdev->dev;
	struct drm_connector *connector;
	struct mt8377_dpi *dpi;
	int ret;

	dpi = devm_drm_dev_alloc(dev, &mt8377_dpi_driver, struct mt8377_dpi, drm);
	if (IS_ERR(dpi))
		return PTR_ERR(dpi);
	dpi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dpi->regs))
		return PTR_ERR(dpi->regs);
	dpi->clocks[0].id = "larb-emi";
	dpi->clocks[1].id = "larb";
	dpi->clocks[2].id = "engine";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(dpi->clocks), dpi->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "Missing engine or fabric clocks\n");
	dpi->pll = devm_clk_get(dev, "pll");
	if (IS_ERR(dpi->pll))
		return dev_err_probe(dev, PTR_ERR(dpi->pll), "Missing PLL\n");
	dpi->mmsys = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,mmsys");
	if (IS_ERR(dpi->mmsys))
		return dev_err_probe(dev, PTR_ERR(dpi->mmsys), "Missing MMSYS\n");
	dpi->bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(dpi->bridge))
		return dev_err_probe(dev, PTR_ERR(dpi->bridge), "Missing output bridge\n");
	dpi->irq = platform_get_irq(pdev, 0);
	if (dpi->irq < 0)
		return dpi->irq;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = drmm_mode_config_init(&dpi->drm);
	if (ret)
		return ret;
	dpi->drm.mode_config.funcs = &mt8377_dpi_mode_config_funcs;
	dpi->drm.mode_config.max_width = U16_MAX / 3;
	dpi->drm.mode_config.max_height = U16_MAX;
	ret = drm_simple_display_pipe_init(&dpi->drm, &dpi->pipe, &mt8377_dpi_pipe_funcs,
					   formats, ARRAY_SIZE(formats), NULL, NULL);
	if (ret)
		return ret;
	ret = drm_bridge_attach(&dpi->pipe.encoder, dpi->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;
	connector = drm_bridge_connector_init(&dpi->drm, &dpi->pipe.encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);
	ret = drm_vblank_init(&dpi->drm, 1);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, dpi->irq, mt8377_dpi_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), dpi);
	if (ret)
		return ret;
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	drm_mode_config_reset(&dpi->drm);
	drm_crtc_vblank_reset(&dpi->pipe.crtc);
	platform_set_drvdata(pdev, dpi);
	ret = drm_dev_register(&dpi->drm, 0);
	if (ret)
		return ret;
	drm_client_setup_with_fourcc(&dpi->drm, DRM_FORMAT_RGB565);
	return 0;
}

static void mt8377_dpi_shutdown(struct platform_device *pdev)
{
	struct mt8377_dpi *dpi = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&dpi->drm);
}

static void mt8377_dpi_remove(struct platform_device *pdev)
{
	struct mt8377_dpi *dpi = platform_get_drvdata(pdev);

	drm_dev_unregister(&dpi->drm);
	mt8377_dpi_shutdown(pdev);
}

static int mt8377_dpi_suspend(struct device *dev)
{
	struct mt8377_dpi *dpi = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&dpi->drm);
}

static int mt8377_dpi_resume(struct device *dev)
{
	struct mt8377_dpi *dpi = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&dpi->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt8377_dpi_pm_ops, mt8377_dpi_suspend,
			      mt8377_dpi_resume);

static const struct of_device_id mt8377_dpi_match[] = {
	{ .compatible = "mediatek,mt8377-dpi" },
	{ .compatible = "mediatek,mt6577-dpi" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt8377_dpi_match);

static struct platform_driver mt8377_dpi_platform_driver = {
	.probe = mt8377_dpi_probe,
	.remove = mt8377_dpi_remove,
	.shutdown = mt8377_dpi_shutdown,
	.driver = {
		.name = "mt8377-dpi",
		.of_match_table = mt8377_dpi_match,
		.pm = pm_sleep_ptr(&mt8377_dpi_pm_ops),
	},
};

drm_module_platform_driver(mt8377_dpi_platform_driver);

MODULE_DESCRIPTION("MediaTek MT8377 DPI display controller");
MODULE_LICENSE("GPL");
