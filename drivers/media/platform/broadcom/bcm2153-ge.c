// SPDX-License-Identifier: GPL-2.0-only
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-vmalloc.h>

#include "bcm2153-ge-hw.h"

#define GE_WIDTH 240
#define GE_HEIGHT 400
#define GE_BYTES (GE_WIDTH * GE_HEIGHT * 4)

struct bcm2153_ge {
	struct device *dev;
	void __iomem *base;
	int irq;
	bool irq_requested;
	bool removing;
	bool failed;
	struct mutex lock;
	spinlock_t run_lock;
	struct completion done;
	u32 status;
	struct workqueue_struct *workqueue;
	struct v4l2_device v4l2;
	struct video_device *video;
	struct v4l2_m2m_dev *m2m;
	void *source;
	void *destination;
	dma_addr_t source_dma;
	dma_addr_t destination_dma;
};

struct bcm2153_ge_context {
	struct v4l2_fh fh;
	struct bcm2153_ge *ge;
	struct v4l2_pix_format format[2];
	struct work_struct work;
	atomic_t aborting;
	u32 sequence[2];
};

static struct bcm2153_ge_context *ge_context(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct bcm2153_ge_context, fh);
}

static unsigned int ge_queue_index(enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? 0 : 1;
}

static void ge_finish(struct bcm2153_ge_context *ctx, enum vb2_buffer_state state)
{
	v4l2_m2m_buf_done_and_job_finish(ctx->ge->m2m, ctx->fh.m2m_ctx, state);
}

static void ge_pack(void *destination, const void *source, const struct v4l2_pix_format *format)
{
	unsigned int x, y;

	for (y = 0; y < format->height; y++) {
		for (x = 0; x < format->width; x++) {
			u32 pixel = get_unaligned_le32(source + y * format->bytesperline + x * 4);

			put_unaligned_le32(pixel | 0xff000000,
					   destination + (y * format->width + x) * 4);
		}
	}
}

static void ge_work(struct work_struct *work)
{
	struct bcm2153_ge_context *ctx = container_of(work, struct bcm2153_ge_context, work);
	struct bcm2153_ge *ge = ctx->ge;
	struct v4l2_pix_format *input = &ctx->format[0], *output = &ctx->format[1];
	struct vb2_v4l2_buffer *src, *dst;
	struct bcm2153_ge_job job = {
		.source = ge->source_dma,
		.destination = ge->destination_dma,
		.source_stride = input->width * 4,
		.destination_stride = input->width * 4,
		.width = input->width,
		.height = input->height,
		.format = 0x101,
	};
	void *source, *destination;
	unsigned int y;
	int ret = -EIO;
	unsigned long completed;

	reinit_completion(&ge->done);
	WRITE_ONCE(ge->status, 0);
	if (READ_ONCE(ge->removing) || READ_ONCE(ge->failed) || atomic_read(&ctx->aborting))
		goto finish;
	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!src || !dst)
		goto finish;
	src->sequence = ctx->sequence[0]++;
	dst->sequence = ctx->sequence[1]++;
	v4l2_m2m_buf_copy_metadata(src, dst);
	source = vb2_plane_vaddr(&src->vb2_buf, 0);
	destination = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (!source || !destination)
		goto finish;
	ge_pack(ge->source, source, input);
	if (READ_ONCE(ge->removing) || atomic_read(&ctx->aborting))
		goto finish;
	dma_wmb();
	writel(0x100, ge->base + 0x0c);
	bcm2153_ge_program(ge->base, &job);
	completed = wait_for_completion_timeout(&ge->done, msecs_to_jiffies(1000));
	disable_irq(ge->irq);
	ret = bcm2153_ge_stop(ge->base);
	if (ret) {
		WRITE_ONCE(ge->failed, true);
		dev_err(ge->dev, "GE stop failed; DMA memory retained\n");
		goto finish;
	}
	enable_irq(ge->irq);
	if (!completed || atomic_read(&ctx->aborting) || READ_ONCE(ge->removing) ||
	    READ_ONCE(ge->status) != 0x101) {
		ret = -EIO;
		goto finish;
	}
	dma_rmb();
	for (y = 0; y < output->height; y++)
		memcpy(destination + y * output->bytesperline,
		       ge->destination + y * job.destination_stride, output->width * 4);
finish:
	ge_finish(ctx, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void ge_run(void *priv)
{
	struct bcm2153_ge_context *ctx = priv;
	struct bcm2153_ge *ge = ctx->ge;
	unsigned long flags;
	bool removing;

	spin_lock_irqsave(&ge->run_lock, flags);
	removing = ge->removing;
	if (!removing)
		queue_work(ge->workqueue, &ctx->work);
	spin_unlock_irqrestore(&ge->run_lock, flags);
	if (removing)
		ge_finish(ctx, VB2_BUF_STATE_ERROR);
}

static void ge_abort(void *priv)
{
	struct bcm2153_ge_context *ctx = priv;

	atomic_set(&ctx->aborting, 1);
	complete(&ctx->ge->done);
}

static irqreturn_t ge_irq(int irq, void *priv)
{
	struct bcm2153_ge *ge = priv;
	u32 status = readl(ge->base);

	if (!(status & BCM2153_GE_INTERRUPT))
		return IRQ_NONE;
	writel(1, ge->base + 4);
	writel(0x100, ge->base);
	readl(ge->base);
	WRITE_ONCE(ge->status, status);
	complete(&ge->done);
	return IRQ_HANDLED;
}

static int ge_queue_setup(struct vb2_queue *queue, unsigned int *buffers,
			  unsigned int *planes, unsigned int sizes[], struct device *devices[])
{
	struct bcm2153_ge_context *ctx = vb2_get_drv_priv(queue);
	u32 size = ctx->format[ge_queue_index(queue->type)].sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int ge_prepare_buffer(struct vb2_buffer *buffer)
{
	struct bcm2153_ge_context *ctx = vb2_get_drv_priv(buffer->vb2_queue);
	u32 size = ctx->format[ge_queue_index(buffer->type)].sizeimage;

	if (vb2_plane_size(buffer, 0) < size)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(buffer->type) && vb2_get_plane_payload(buffer, 0) < size)
		return -EINVAL;
	vb2_set_plane_payload(buffer, 0, size);
	return 0;
}

static int ge_validate_output(struct vb2_buffer *buffer)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buffer);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	return vbuf->field == V4L2_FIELD_NONE ? 0 : -EINVAL;
}

static void ge_queue_buffer(struct vb2_buffer *buffer)
{
	struct bcm2153_ge_context *ctx = vb2_get_drv_priv(buffer->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(buffer));
}

static int ge_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct bcm2153_ge_context *ctx = vb2_get_drv_priv(queue);

	ctx->sequence[ge_queue_index(queue->type)] = 0;
	atomic_set(&ctx->aborting, 0);
	return 0;
}

static void ge_stop_streaming(struct vb2_queue *queue)
{
	struct bcm2153_ge_context *ctx = vb2_get_drv_priv(queue);
	struct vb2_v4l2_buffer *buffer;

	for (;;) {
		buffer = V4L2_TYPE_IS_OUTPUT(queue->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!buffer)
			break;
		v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops ge_queue_ops = {
	.queue_setup = ge_queue_setup,
	.buf_prepare = ge_prepare_buffer,
	.buf_out_validate = ge_validate_output,
	.buf_queue = ge_queue_buffer,
	.start_streaming = ge_start_streaming,
	.stop_streaming = ge_stop_streaming,
};

static int ge_queue_init(void *priv, struct vb2_queue *source, struct vb2_queue *destination)
{
	struct bcm2153_ge_context *ctx = priv;
	struct vb2_queue *queue;
	unsigned int i;
	int ret;

	for (i = 0; i < 2; i++) {
		queue = i ? destination : source;
		queue->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		queue->io_modes = VB2_MMAP;
		queue->drv_priv = ctx;
		queue->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		queue->ops = &ge_queue_ops;
		queue->mem_ops = &vb2_vmalloc_memops;
		queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		queue->lock = &ctx->ge->lock;
		queue->dev = ctx->ge->dev;
		ret = vb2_queue_init(queue);
		if (ret)
			return ret;
	}
	return 0;
}

static void ge_format(struct v4l2_pix_format *format)
{
	format->width = clamp_t(u32, format->width, 1, GE_WIDTH);
	format->height = clamp_t(u32, format->height, 1, GE_HEIGHT);
	format->bytesperline = clamp_t(u32, format->bytesperline,
				     format->width * 4, GE_WIDTH * 4);
	format->bytesperline = ALIGN(format->bytesperline, 4);
	format->sizeimage = format->bytesperline * format->height;
	format->pixelformat = V4L2_PIX_FMT_XBGR32;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->xfer_func = V4L2_XFER_FUNC_SRGB;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->flags = 0;
	format->priv = 0;
}

static int ge_open(struct file *file)
{
	struct bcm2153_ge *ge = video_drvdata(file);
	struct bcm2153_ge_context *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->ge = ge;
	INIT_WORK(&ctx->work, ge_work);
	atomic_set(&ctx->aborting, 0);
	ctx->format[0].width = GE_WIDTH;
	ctx->format[0].height = GE_HEIGHT;
	ge_format(&ctx->format[0]);
	ctx->format[1] = ctx->format[0];
	ret = mutex_lock_interruptible(&ge->lock);
	if (ret)
		goto free;
	if (READ_ONCE(ge->removing)) {
		ret = -ENODEV;
		goto unlock;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(ge->m2m, ctx, ge_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		goto unlock;
	}
	v4l2_fh_add(&ctx->fh, file);
	mutex_unlock(&ge->lock);
	return 0;
unlock:
	mutex_unlock(&ge->lock);
free:
	kfree(ctx);
	return ret;
}

static int ge_release(struct file *file)
{
	struct bcm2153_ge_context *ctx = ge_context(file);

	mutex_lock(&ctx->ge->lock);
	v4l2_m2m_suspend(ctx->ge->m2m);
	flush_work(&ctx->work);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (!READ_ONCE(ctx->ge->removing))
		v4l2_m2m_resume(ctx->ge->m2m);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&ctx->ge->lock);
	kfree(ctx);
	return 0;
}

static int ge_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct bcm2153_ge *ge = video_drvdata(file);

	strscpy(cap->driver, "bcm2153-ge", sizeof(cap->driver));
	strscpy(cap->card, "BCM2153 GE", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(ge->dev));
	return 0;
}

static int ge_enum_format(struct file *file, void *priv, struct v4l2_fmtdesc *format)
{
	if (format->index)
		return -EINVAL;
	format->pixelformat = V4L2_PIX_FMT_XBGR32;
	return 0;
}

static int ge_get_format(struct file *file, void *priv, struct v4l2_format *format)
{
	format->fmt.pix = ge_context(file)->format[ge_queue_index(format->type)];
	return 0;
}

static int ge_try_format(struct file *file, void *priv, struct v4l2_format *format)
{
	struct bcm2153_ge_context *ctx = ge_context(file);

	if (format->fmt.pix.field != V4L2_FIELD_ANY && format->fmt.pix.field != V4L2_FIELD_NONE)
		return -EINVAL;
	if (!V4L2_TYPE_IS_OUTPUT(format->type)) {
		format->fmt.pix.width = ctx->format[0].width;
		format->fmt.pix.height = ctx->format[0].height;
	}
	ge_format(&format->fmt.pix);
	return 0;
}

static int ge_set_format(struct file *file, void *priv, struct v4l2_format *format)
{
	struct bcm2153_ge_context *ctx = ge_context(file);
	unsigned int index = ge_queue_index(format->type);
	int ret;

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, format->type)))
		return -EBUSY;
	if (!index && vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	ret = ge_try_format(file, priv, format);
	if (ret)
		return ret;
	ctx->format[index] = format->fmt.pix;
	if (!index)
		ctx->format[1] = format->fmt.pix;
	return 0;
}

static const struct v4l2_file_operations ge_fops = {
	.owner = THIS_MODULE,
	.open = ge_open,
	.release = ge_release,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops ge_ioctl_ops = {
	.vidioc_querycap = ge_querycap,
	.vidioc_enum_fmt_vid_cap = ge_enum_format,
	.vidioc_enum_fmt_vid_out = ge_enum_format,
	.vidioc_g_fmt_vid_cap = ge_get_format,
	.vidioc_g_fmt_vid_out = ge_get_format,
	.vidioc_try_fmt_vid_cap = ge_try_format,
	.vidioc_try_fmt_vid_out = ge_try_format,
	.vidioc_s_fmt_vid_cap = ge_set_format,
	.vidioc_s_fmt_vid_out = ge_set_format,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static const struct v4l2_m2m_ops ge_m2m_ops = {
	.device_run = ge_run,
	.job_abort = ge_abort,
};

static void ge_device_release(struct v4l2_device *v4l2)
{
	struct bcm2153_ge *ge = container_of(v4l2, struct bcm2153_ge, v4l2);

	if (ge->m2m)
		v4l2_m2m_release(ge->m2m);
	kfree(ge);
}

static void ge_free_resources(struct bcm2153_ge *ge)
{
	if (ge->workqueue)
		destroy_workqueue(ge->workqueue);
	if (ge->irq_requested)
		devm_free_irq(ge->dev, ge->irq, ge);
	if (ge->failed) {
		get_device(ge->dev);
		dev_crit(ge->dev, "DMA buffers retained until hardware reset\n");
		return;
	}
	if (ge->destination)
		dma_free_coherent(ge->dev, GE_BYTES, ge->destination, ge->destination_dma);
	if (ge->source)
		dma_free_coherent(ge->dev, GE_BYTES, ge->source, ge->source_dma);
}

static int ge_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2153_ge *ge;
	bool registered = false;
	int ret;

	ge = kzalloc_obj(*ge);
	if (!ge)
		return -ENOMEM;
	ge->dev = dev;
	mutex_init(&ge->lock);
	spin_lock_init(&ge->run_lock);
	init_completion(&ge->done);
	ge->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ge->base)) {
		ret = PTR_ERR(ge->base);
		goto error;
	}
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto error;
	if (readl(ge->base) & ~BCM2153_GE_INTERRUPT) {
		ret = -EBUSY;
		goto error;
	}
	ret = bcm2153_ge_stop(ge->base);
	if (ret)
		goto error;
	ge->irq = platform_get_irq(pdev, 0);
	if (ge->irq < 0) {
		ret = ge->irq;
		goto error;
	}
	ge->source = dma_alloc_coherent(dev, GE_BYTES, &ge->source_dma, GFP_KERNEL);
	ge->destination = dma_alloc_coherent(dev, GE_BYTES, &ge->destination_dma, GFP_KERNEL);
	if (!ge->source || !ge->destination) {
		ret = -ENOMEM;
		goto error;
	}
	ge->workqueue = alloc_ordered_workqueue("bcm2153-ge", WQ_MEM_RECLAIM);
	if (!ge->workqueue) {
		ret = -ENOMEM;
		goto error;
	}
	ge->v4l2.release = ge_device_release;
	ret = v4l2_device_register(dev, &ge->v4l2);
	if (ret)
		goto error;
	registered = true;
	ge->m2m = v4l2_m2m_init(&ge_m2m_ops);
	if (IS_ERR(ge->m2m)) {
		ret = PTR_ERR(ge->m2m);
		ge->m2m = NULL;
		goto error;
	}
	ge->video = video_device_alloc();
	if (!ge->video) {
		ret = -ENOMEM;
		goto error;
	}
	strscpy(ge->video->name, "bcm2153-ge", sizeof(ge->video->name));
	ge->video->fops = &ge_fops;
	ge->video->ioctl_ops = &ge_ioctl_ops;
	ge->video->release = video_device_release;
	ge->video->v4l2_dev = &ge->v4l2;
	ge->video->lock = &ge->lock;
	ge->video->vfl_dir = VFL_DIR_M2M;
	ge->video->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(ge->video, ge);
	ret = devm_request_irq(dev, ge->irq, ge_irq, 0, dev_name(dev), ge);
	if (ret)
		goto error;
	ge->irq_requested = true;
	ret = video_register_device(ge->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto error;
	platform_set_drvdata(pdev, ge);
	return 0;
error:
	if (ge->video)
		video_device_release(ge->video);
	ge_free_resources(ge);
	if (registered) {
		v4l2_device_unregister(&ge->v4l2);
		v4l2_device_put(&ge->v4l2);
	} else {
		kfree(ge);
	}
	return ret;
}

static void ge_quiesce(struct bcm2153_ge *ge)
{
	unsigned long flags;

	mutex_lock(&ge->lock);
	v4l2_m2m_suspend(ge->m2m);
	spin_lock_irqsave(&ge->run_lock, flags);
	WRITE_ONCE(ge->removing, true);
	spin_unlock_irqrestore(&ge->run_lock, flags);
	complete_all(&ge->done);
	flush_workqueue(ge->workqueue);
	mutex_unlock(&ge->lock);
}

static void ge_remove(struct platform_device *pdev)
{
	struct bcm2153_ge *ge = platform_get_drvdata(pdev);

	ge_quiesce(ge);
	video_unregister_device(ge->video);
	ge_free_resources(ge);
	v4l2_device_unregister(&ge->v4l2);
	v4l2_device_put(&ge->v4l2);
}

static void ge_shutdown(struct platform_device *pdev)
{
	ge_quiesce(platform_get_drvdata(pdev));
}

static const struct of_device_id ge_of_match[] = {
	{ .compatible = "brcm,bcm2153-ge" },
	{ }
};
MODULE_DEVICE_TABLE(of, ge_of_match);

static struct platform_driver ge_driver = {
	.probe = ge_probe,
	.remove = ge_remove,
	.shutdown = ge_shutdown,
	.driver = {
		.name = "bcm2153-ge",
		.of_match_table = ge_of_match,
	},
};
module_platform_driver(ge_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 graphics engine");
MODULE_LICENSE("GPL");
