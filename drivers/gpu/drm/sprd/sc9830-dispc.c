// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reset.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "sc9830-dispc.h"

struct sc9830_dispc {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	void __iomem *base;
	struct clk_bulk_data clocks[4];
	struct reset_control *reset;
	struct completion update;
	spinlock_t lock; /* IRQ serialization */
	struct drm_framebuffer *held;
	struct drm_framebuffer *quarantined;
	bool running;
	bool fault;
};

static struct sc9830_dispc *to_dispc(struct drm_device *drm)
{
	return container_of(drm, struct sc9830_dispc, drm);
}

static bool sc9830_dispc_stop(struct sc9830_dispc *dispc)
{
	u32 value;
	int ret;

	writel(readl(dispc->base + 0x80) | BIT(4), dispc->base + 0x80);
	writel(readl(dispc->base) & ~BIT(4), dispc->base);
	ret = readl_poll_timeout(dispc->base + 0x90, value, !(value & BIT(16)),
				 100, 100000);
	if (ret) {
		WRITE_ONCE(dispc->fault, true);
		ret = reset_control_assert(dispc->reset);
		if (ret) {
			drm_err(&dispc->drm, "DISPC reset failed: %d\n", ret);
			return false;
		}
	}
	dispc->running = false;
	return true;
}

static int sc9830_plane_check(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct sc9830_dispc *dispc = to_dispc(plane->dev);
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *c;
	struct drm_gem_dma_object *gem;
	u64 address, end;
	unsigned int cpp, width, height;
	int ret;

	if (!p->crtc)
		return 0;
	if (READ_ONCE(dispc->fault))
		return -EIO;
	c = drm_atomic_get_new_crtc_state(state, p->crtc);
	ret = drm_atomic_helper_check_plane_state(p, c, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING, false, false);
	if (ret)
		return ret;
	if (!p->visible)
		return -EINVAL;
	width = p->src_w >> 16;
	height = p->src_h >> 16;
	cpp = p->fb->format->cpp[0];
	if (p->src_x || p->src_y || !width || width > 4095 || !height || height > 4095 ||
	    p->fb->pitches[0] % cpp || p->fb->pitches[0] / cpp < width ||
	    p->fb->pitches[0] / cpp > 4095)
		return -EINVAL;
	gem = drm_fb_dma_get_gem_obj(p->fb, 0);
	end = (u64)p->fb->offsets[0] + (height - 1) * (u64)p->fb->pitches[0] + width * cpp;
	address = (u64)gem->dma_addr + p->fb->offsets[0];
	if ((u64)gem->dma_addr > U32_MAX || end > gem->base.size ||
	    address & 3 || (u64)gem->dma_addr + end > BIT_ULL(32))
		return -EINVAL;
	return 0;
}

static void sc9830_plane_defer_to_crtc(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
}

static const struct drm_plane_helper_funcs sc9830_plane_helper = {
	.atomic_check = sc9830_plane_check,
	.atomic_update = sc9830_plane_defer_to_crtc,
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
};

static const struct drm_plane_funcs sc9830_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static enum drm_mode_status
sc9830_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	struct sc9830_dispc *dispc = to_dispc(crtc->dev);
	u32 value;
	long rate;

	if (mode->hdisplay <= 0 || mode->hdisplay > 4095 ||
	    mode->vdisplay <= 0 || mode->vdisplay > 4095 || mode->clock <= 0 ||
	    mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN))
		return MODE_BAD;
	if (mode->clock > 384000)
		return MODE_CLOCK_HIGH;
	if (sc9830_dispc_timing(mode->hsync_end - mode->hsync_start,
				mode->htotal - mode->hsync_end,
				mode->hsync_start - mode->hdisplay, &value) ||
	    sc9830_dispc_timing(mode->vsync_end - mode->vsync_start,
				mode->vtotal - mode->vsync_end,
				mode->vsync_start - mode->vdisplay, &value))
		return MODE_BAD;
	rate = clk_round_rate(dispc->clocks[2].clk, mode->clock * 1000UL);
	if (rate <= 0 || abs(rate - mode->clock * 1000L) > mode->clock * 10L)
		return MODE_CLOCK_RANGE;
	return MODE_OK;
}

static void sc9830_send_event(struct drm_crtc *crtc)
{
	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void sc9830_dispc_frame(struct sc9830_dispc *dispc, bool first)
{
	struct drm_plane_state *p = dispc->plane.state;
	struct drm_framebuffer *old;
	unsigned long flags;

	if (!p->fb || READ_ONCE(dispc->fault))
		goto event;
	drm_framebuffer_get(p->fb);
	old = dispc->held;
	dispc->held = p->fb;
	drm_fb_dma_sync_non_coherent(&dispc->drm, p, p);
	sc9830_dispc_layer(dispc->base, p->src_w >> 16, p->src_h >> 16,
			   p->fb->pitches[0] / p->fb->format->cpp[0],
			   p->fb->format->cpp[0] * 8);
	writel(lower_32_bits(drm_fb_dma_get_gem_addr(p->fb, p, 0)), dispc->base + 0x44);
	spin_lock_irqsave(&dispc->lock, flags);
	writel(SC9830_DISPC_IRQ_UPDATE, dispc->base + 0x74);
	readl(dispc->base + 0x78);
	reinit_completion(&dispc->update);
	spin_unlock_irqrestore(&dispc->lock, flags);
	sc9830_dispc_run(dispc->base, first);
	if (!first && !wait_for_completion_timeout(&dispc->update, msecs_to_jiffies(100))) {
		drm_err(&dispc->drm, "DISPC update timed out\n");
		WRITE_ONCE(dispc->fault, true);
		if (!sc9830_dispc_stop(dispc)) {
			dispc->quarantined = old;
			goto event;
		}
	}
	if (old)
		drm_framebuffer_put(old);
event:
	sc9830_send_event(&dispc->crtc);
}

static void sc9830_crtc_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct sc9830_dispc *dispc = to_dispc(crtc->dev);
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	u32 horizontal, vertical, dpi = 2 << 6;
	int ret;

	if (READ_ONCE(dispc->fault))
		goto event;
	ret = clk_set_rate(dispc->clocks[2].clk, mode->clock * 1000UL);
	if (ret) {
		drm_err(crtc->dev, "Pixel clock failed: %d\n", ret);
		WRITE_ONCE(dispc->fault, true);
		goto event;
	}
	sc9830_dispc_timing(mode->hsync_end - mode->hsync_start,
			    mode->htotal - mode->hsync_end,
			    mode->hsync_start - mode->hdisplay, &horizontal);
	sc9830_dispc_timing(mode->vsync_end - mode->vsync_start,
			    mode->vtotal - mode->vsync_end,
			    mode->vsync_start - mode->vdisplay, &vertical);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		dpi |= BIT(0);
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		dpi |= BIT(1);
	writel(0xc1, dispc->base);
	writel(mode->hdisplay | (mode->vdisplay << 16), dispc->base + 4);
	writel(0x13881388, dispc->base + 0x0c);
	writel(0, dispc->base + 0x3c);
	writel(dpi | BIT(4), dispc->base + 0x80);
	writel(horizontal, dispc->base + 0x84);
	writel(vertical, dispc->base + 0x88);
	writel(0x3f, dispc->base + 0x74);
	writel(SC9830_DISPC_IRQ_ERROR | SC9830_DISPC_IRQ_UPDATE, dispc->base + 0x70);
	dispc->running = true;
	drm_crtc_vblank_on(crtc);
	sc9830_dispc_frame(dispc, true);
	return;
event:
	sc9830_send_event(crtc);
}

static void sc9830_crtc_disable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct sc9830_dispc *dispc = to_dispc(crtc->dev);

	drm_crtc_vblank_off(crtc);
	if (sc9830_dispc_stop(dispc)) {
		if (dispc->held)
			drm_framebuffer_put(dispc->held);
		if (dispc->quarantined)
			drm_framebuffer_put(dispc->quarantined);
		dispc->held = NULL;
		dispc->quarantined = NULL;
	}
	writel(0, dispc->base + 0x70);
	sc9830_send_event(crtc);
}

static void sc9830_crtc_flush(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct sc9830_dispc *dispc = to_dispc(crtc->dev);

	if (dispc->running)
		sc9830_dispc_frame(dispc, false);
}

static int sc9830_vblank_enable(struct drm_crtc *crtc)
{
	struct sc9830_dispc *dispc = to_dispc(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&dispc->lock, flags);
	writel(readl(dispc->base + 0x70) | SC9830_DISPC_IRQ_VSYNC, dispc->base + 0x70);
	spin_unlock_irqrestore(&dispc->lock, flags);
	return 0;
}

static void sc9830_vblank_disable(struct drm_crtc *crtc)
{
	struct sc9830_dispc *dispc = to_dispc(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&dispc->lock, flags);
	writel(readl(dispc->base + 0x70) & ~SC9830_DISPC_IRQ_VSYNC, dispc->base + 0x70);
	spin_unlock_irqrestore(&dispc->lock, flags);
}

static int sc9830_crtc_check(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_crtc_state *c = drm_atomic_get_new_crtc_state(state, crtc);

	return c->active ? drm_atomic_helper_check_crtc_primary_plane(c) : 0;
}

static const struct drm_crtc_helper_funcs sc9830_crtc_helper = {
	.mode_valid = sc9830_mode_valid,
	.atomic_check = sc9830_crtc_check,
	.atomic_enable = sc9830_crtc_enable,
	.atomic_disable = sc9830_crtc_disable,
	.atomic_flush = sc9830_crtc_flush,
};

static const struct drm_crtc_funcs sc9830_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = drm_crtc_cleanup,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = sc9830_vblank_enable,
	.disable_vblank = sc9830_vblank_disable,
};

static irqreturn_t sc9830_irq(int irq, void *data)
{
	struct sc9830_dispc *dispc = data;
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&dispc->lock, flags);
	status = sc9830_dispc_interrupt(dispc->base);
	if (status & SC9830_DISPC_IRQ_UPDATE)
		complete(&dispc->update);
	spin_unlock_irqrestore(&dispc->lock, flags);
	if (status & SC9830_DISPC_IRQ_VSYNC)
		drm_crtc_handle_vblank(&dispc->crtc);
	if (status & SC9830_DISPC_IRQ_ERROR)
		drm_err_ratelimited(&dispc->drm, "DISPC underflow\n");
	return status ? IRQ_HANDLED : IRQ_NONE;
}

static const struct drm_encoder_funcs sc9830_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_mode_config_funcs sc9830_mode_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(sc9830_fops);

static const struct drm_driver sc9830_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &sc9830_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "sc9830-dispc",
	.desc = "SC9830 display controller",
	.major = 1,
};

static void sc9830_clocks_disable(void *data)
{
	struct sc9830_dispc *dispc = data;

	clk_bulk_disable_unprepare(ARRAY_SIZE(dispc->clocks), dispc->clocks);
}

static int sc9830_dispc_probe(struct platform_device *pdev)
{
	static const u32 formats[] = { DRM_FORMAT_XRGB8888, DRM_FORMAT_RGB565 };
	struct device *dev = &pdev->dev;
	struct sc9830_dispc *dispc;
	struct drm_connector *connector;
	struct drm_bridge *bridge;
	struct drm_device *drm;
	int irq, ret;

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "Missing DSI bridge\n");
	dispc = devm_drm_dev_alloc(dev, &sc9830_driver, struct sc9830_dispc, drm);
	if (IS_ERR(dispc))
		return PTR_ERR(dispc);
	drm = &dispc->drm;
	platform_set_drvdata(pdev, dispc);
	spin_lock_init(&dispc->lock);
	init_completion(&dispc->update);
	dispc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dispc->base))
		return PTR_ERR(dispc->base);
	dispc->clocks[0].id = "core";
	dispc->clocks[1].id = "dbi";
	dispc->clocks[2].id = "dpi";
	dispc->clocks[3].id = "emc";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(dispc->clocks), dispc->clocks);
	if (ret)
		return ret;
	dispc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(dispc->reset))
		return PTR_ERR(dispc->reset);
	ret = clk_set_rate(dispc->clocks[0].clk, 256000000);
	if (ret)
		return ret;
	ret = clk_set_rate(dispc->clocks[1].clk, 256000000);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(dispc->clocks), dispc->clocks);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, sc9830_clocks_disable, dispc);
	if (ret)
		return ret;
	ret = reset_control_reset(dispc->reset);
	if (ret)
		return ret;
	writel(0, dispc->base + 0x70);
	writel(0x3f, dispc->base + 0x74);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.funcs = &sc9830_mode_funcs;
	drm->mode_config.max_width = 4095;
	drm->mode_config.max_height = 4095;
	ret = drm_universal_plane_init(drm, &dispc->plane, 1, &sc9830_plane_funcs,
				       formats, ARRAY_SIZE(formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&dispc->plane, &sc9830_plane_helper);
	ret = drm_crtc_init_with_planes(drm, &dispc->crtc, &dispc->plane, NULL,
					&sc9830_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&dispc->crtc, &sc9830_crtc_helper);
	ret = drm_encoder_init(drm, &dispc->encoder, &sc9830_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;
	dispc->encoder.possible_crtcs = 1;
	ret = drm_bridge_attach(&dispc->encoder, bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;
	connector = drm_bridge_connector_init(drm, &dispc->encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);
	ret = drm_connector_attach_encoder(connector, &dispc->encoder);
	if (ret)
		return ret;
	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, sc9830_irq, 0, dev_name(dev), dispc);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;
	drm_client_setup(drm, NULL);
	return 0;
}

static void sc9830_dispc_remove(struct platform_device *pdev)
{
	struct sc9830_dispc *dispc = platform_get_drvdata(pdev);

	drm_dev_unregister(&dispc->drm);
	drm_atomic_helper_shutdown(&dispc->drm);
}

static void sc9830_dispc_shutdown(struct platform_device *pdev)
{
	struct sc9830_dispc *dispc = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&dispc->drm);
}

static int sc9830_dispc_suspend(struct device *dev)
{
	struct sc9830_dispc *dispc = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&dispc->drm);
}

static int sc9830_dispc_resume(struct device *dev)
{
	struct sc9830_dispc *dispc = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&dispc->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc9830_dispc_pm_ops, sc9830_dispc_suspend,
			      sc9830_dispc_resume);

static const struct of_device_id sc9830_dispc_match[] = {
	{ .compatible = "sprd,sc9830-dispc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_dispc_match);

static struct platform_driver sc9830_dispc_driver = {
	.probe = sc9830_dispc_probe,
	.remove = sc9830_dispc_remove,
	.shutdown = sc9830_dispc_shutdown,
	.driver = {
		.name = "sc9830-dispc",
		.of_match_table = sc9830_dispc_match,
		.pm = pm_sleep_ptr(&sc9830_dispc_pm_ops),
	},
};
module_platform_driver(sc9830_dispc_driver);

MODULE_DESCRIPTION("SC9830 DRM display controller");
MODULE_LICENSE("GPL");
