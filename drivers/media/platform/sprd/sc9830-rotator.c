// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define SC9830_ROT_IRQ BIT(15)
#define SC9830_DCAM_IRQS 0x011fffff
#define SC9830_JOB_MS 5000

struct sc9830_rot_io {
	void *ctx;
	u32 (*read)(void *ctx, unsigned int offset);
	void (*write)(void *ctx, unsigned int offset, u32 value);
	u32 (*mmu_read)(void *ctx);
	int (*reset)(void *ctx, bool all);
};

static void sc9830_rot_update(struct sc9830_rot_io *io, unsigned int offset,
			      u32 clear, u32 set)
{
	io->write(io->ctx, offset, (io->read(io->ctx, offset) & ~clear) | set);
}

static int sc9830_rot_translation(struct sc9830_rot_io *io, u32 *snapshot)
{
	u32 value = io->mmu_read(io->ctx);

	if (io->mmu_read(io->ctx) != value || io->mmu_read(io->ctx) != value ||
	    (value & BIT(0)))
		return -EBUSY;
	*snapshot = value;
	return 0;
}

static int sc9830_rot_start16(struct sc9830_rot_io *io, u32 source,
			      u32 destination, u32 width, u32 height, u32 angle)
{
	int ret;

	if (!source || !destination || ((source | destination) & 3) ||
	    !width || width > 8191 || !height || height > 8191 || angle > 3)
		return -EINVAL;
	ret = io->reset(io->ctx, false);
	if (ret)
		return ret;
	io->write(io->ctx, 0x80, source);
	io->write(io->ctx, 0x84, destination);
	io->write(io->ctx, 0x8008, 0);
	io->write(io->ctx, 0x800c, width | height << 13);
	io->write(io->ctx, 0x8004, width);
	sc9830_rot_update(io, 0x8000, 2, 0);
	sc9830_rot_update(io, 0x8000, 0, 2);
	sc9830_rot_update(io, 0x8000, 16, 0);
	sc9830_rot_update(io, 0x8000, 0, 0);
	sc9830_rot_update(io, 0x8000, 12, 0);
	sc9830_rot_update(io, 0x8000, 0, angle << 2);
	sc9830_rot_update(io, 0x6c, 0x3c000, 0);
	sc9830_rot_update(io, 0x6c, 0, 0xd4000);
	sc9830_rot_update(io, 0x8000, 0, 1);
	sc9830_rot_update(io, 0x8000, 0, 32);
	return 0;
}

static int sc9830_rot_drain(struct sc9830_rot_io *io, unsigned int *reads)
{
	unsigned int i;
	int ret;

	io->write(io->ctx, 0x40, 0);
	sc9830_rot_update(io, 0x8000, BIT(0), 0);
	sc9830_rot_update(io, 0x70, 0, BIT(6));
	for (i = 0; i < 999; i++)
		if (!(io->read(io->ctx, 0x70) & BIT(0)))
			break;
	*reads = min(i + 1, 999U);
	if (i == 999)
		return -ETIMEDOUT;
	ret = io->reset(io->ctx, true);
	if (ret)
		return ret;
	io->write(io->ctx, 0x44, SC9830_DCAM_IRQS);
	io->read(io->ctx, 0x3c);
	sc9830_rot_update(io, 0x70, BIT(6), 0);
	return 0;
}

struct sc9830_rot;

struct sc9830_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler controls;
	struct sc9830_rot *rot;
	struct v4l2_pix_format source;
	struct v4l2_pix_format destination;
	unsigned int rotation;
	atomic_t aborted;
};

struct sc9830_rot {
	struct device *dev;
	void __iomem *dcam;
	void __iomem *mmu;
	struct regmap *pmu;
	struct regmap *mm_ahb;
	struct clk *dcam_clk;
	struct clk *mmu_clk;
	struct reset_control *rot_reset;
	struct reset_control *all_reset;
	struct icc_path *path;
	struct sc9830_rot_io io;
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock; /* Queues */
	spinlock_t irq_lock; /* IRQ admission */
	struct completion done;
	struct work_struct work;
	struct sc9830_ctx *job_ctx;
	struct dma_buf *pins[2];
	int irq;
	bool irq_live;
	bool irq_enabled;
	bool irq_done;
	bool faulted;
	bool pm_held;
	bool dcam_held;
	bool mmu_held;
	bool module_held;
};

static u32 sc9830_read(void *ctx, unsigned int offset)
{
	struct sc9830_rot *rot = ctx;

	return readl(rot->dcam + offset);
}

static void sc9830_write(void *ctx, unsigned int offset, u32 value)
{
	struct sc9830_rot *rot = ctx;

	writel(value, rot->dcam + offset);
}

static u32 sc9830_mmu_read(void *ctx)
{
	struct sc9830_rot *rot = ctx;

	return readl(rot->mmu);
}

static int sc9830_reset(void *ctx, bool all)
{
	struct sc9830_rot *rot = ctx;

	return reset_control_reset(all ? rot->all_reset : rot->rot_reset);
}

static int sc9830_ready(struct sc9830_rot *rot)
{
	u32 value;
	unsigned int i;
	int ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_read(rot->pmu, 0xbc, &value);
		if (ret)
			return ret;
		if (value >> 28)
			return -EIO;
	}
	ret = regmap_read(rot->mm_ahb, 0, &value);
	if (ret)
		return ret;
	if ((value & (BIT(0) | BIT(7))) != (BIT(0) | BIT(7)))
		return -EIO;
	ret = regmap_read(rot->mm_ahb, 8, &value);
	return ret ? ret : value & BIT(3) ? 0 : -EIO;
}

static irqreturn_t sc9830_irq(int irq, void *data)
{
	struct sc9830_rot *rot = data;
	unsigned long flags;
	irqreturn_t result = IRQ_NONE;

	spin_lock_irqsave(&rot->irq_lock, flags);
	if (rot->irq_live && (sc9830_read(rot, 0x3c) & SC9830_ROT_IRQ)) {
		sc9830_write(rot, 0x44, SC9830_ROT_IRQ);
		sc9830_read(rot, 0x3c);
		rot->irq_done = true;
		complete(&rot->done);
		result = IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&rot->irq_lock, flags);
	return result;
}

static void sc9830_irq_stop(struct sc9830_rot *rot)
{
	unsigned long flags;

	sc9830_write(rot, 0x40, 0);
	sc9830_read(rot, 0x40);
	spin_lock_irqsave(&rot->irq_lock, flags);
	rot->irq_live = false;
	spin_unlock_irqrestore(&rot->irq_lock, flags);
	if (rot->irq_enabled) {
		disable_irq(rot->irq);
		rot->irq_enabled = false;
	}
	synchronize_irq(rot->irq);
}

static void sc9830_unpin(struct sc9830_rot *rot)
{
	unsigned int i;

	for (i = 0; i < 2; i++) {
		if (rot->pins[i])
			dma_buf_put(rot->pins[i]);
		rot->pins[i] = NULL;
	}
}

static int sc9830_pin(struct sc9830_rot *rot, struct vb2_v4l2_buffer *src,
		      struct vb2_v4l2_buffer *dst)
{
	struct vb2_buffer *buffers[] = { &src->vb2_buf, &dst->vb2_buf };
	struct dma_buf *pin;
	unsigned int i;

	for (i = 0; i < 2; i++) {
		pin = vb2_dma_contig_memops.get_dmabuf(buffers[i],
				buffers[i]->planes[0].mem_priv, O_RDWR);
		if (IS_ERR_OR_NULL(pin)) {
			sc9830_unpin(rot);
			return pin ? PTR_ERR(pin) : -ENOMEM;
		}
		rot->pins[i] = pin;
	}
	return 0;
}

static void sc9830_fault(struct sc9830_rot *rot)
{
	WRITE_ONCE(rot->faulted, true);
	dev_err(rot->dev, "DCAM fault; device blocked\n");
}

static int sc9830_release_resources(struct sc9830_rot *rot)
{
	int ret = 0;

	if (rot->dcam_held)
		clk_disable_unprepare(rot->dcam_clk);
	if (rot->mmu_held)
		clk_disable_unprepare(rot->mmu_clk);
	if (rot->pm_held)
		ret = pm_runtime_put_sync(rot->dev);
	rot->dcam_held = false;
	rot->mmu_held = false;
	rot->pm_held = false;
	if (ret < 0)
		sc9830_fault(rot);
	else if (rot->module_held)
		module_put(THIS_MODULE);
	if (ret >= 0)
		rot->module_held = false;
	return ret;
}

static void sc9830_job(struct work_struct *work)
{
	struct sc9830_rot *rot = container_of(work, struct sc9830_rot, work);
	struct sc9830_ctx *ctx = rot->job_ctx;
	struct vb2_v4l2_buffer *src, *dst;
	enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
	dma_addr_t source, destination;
	u32 translation = 0, start_translation = 0, after = 0, vote = 0;
	unsigned int drain_reads = 0;
	unsigned long flags;
	bool voted = false;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (READ_ONCE(rot->faulted) || atomic_read(&ctx->aborted))
		goto finish;
	ret = sc9830_pin(rot, src, dst);
	if (ret)
		goto finish;
	if (!try_module_get(THIS_MODULE))
		goto release;
	rot->module_held = true;
	ret = pm_runtime_resume_and_get(rot->dev);
	if (ret < 0)
		goto release;
	rot->pm_held = true;
	ret = clk_prepare_enable(rot->mmu_clk);
	if (ret)
		goto release;
	rot->mmu_held = true;
	ret = clk_prepare_enable(rot->dcam_clk);
	if (ret)
		goto release;
	rot->dcam_held = true;
	ret = sc9830_ready(rot);
	if (ret)
		goto release;
	sc9830_write(rot, 0x40, 0);
	sc9830_write(rot, 0x44, SC9830_DCAM_IRQS);
	sc9830_read(rot, 0x3c);
	ret = sc9830_rot_translation(&rot->io, &translation);
	if (ret)
		goto release;
	source = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	destination = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	if (!source || !destination || ((source | destination) & 3) ||
	    source > U32_MAX - ctx->source.sizeimage + 1 ||
	    destination > U32_MAX - ctx->destination.sizeimage + 1 ||
	    (source < destination + ctx->destination.sizeimage &&
	     destination < source + ctx->source.sizeimage))
		goto release;
	vote = DIV_ROUND_UP(ctx->source.sizeimage + ctx->destination.sizeimage,
			    SC9830_JOB_MS);
	ret = icc_set_bw(rot->path, vote, vote);
	if (ret) {
		sc9830_fault(rot);
		goto finish;
	}
	voted = true;
	ret = sc9830_rot_translation(&rot->io, &start_translation);
	if (ret || start_translation != translation || atomic_read(&ctx->aborted))
		goto release;
	spin_lock_irqsave(&rot->irq_lock, flags);
	if (atomic_read(&ctx->aborted)) {
		spin_unlock_irqrestore(&rot->irq_lock, flags);
		goto release;
	}
	reinit_completion(&rot->done);
	rot->irq_done = false;
	rot->irq_live = true;
	spin_unlock_irqrestore(&rot->irq_lock, flags);
	enable_irq(rot->irq);
	rot->irq_enabled = true;
	sc9830_write(rot, 0x40, SC9830_ROT_IRQ);
	ret = sc9830_rot_start16(&rot->io, source, destination,
				 ctx->source.width, ctx->source.height,
				 ctx->rotation == 90 ? 0 : ctx->rotation == 270 ? 1 : 2);
	if (ret) {
		sc9830_irq_stop(rot);
		sc9830_fault(rot);
		goto finish;
	}
	ret = wait_for_completion_timeout(&rot->done,
					  msecs_to_jiffies(SC9830_JOB_MS));
	if (ret && READ_ONCE(rot->irq_done) && !atomic_read(&ctx->aborted))
		state = VB2_BUF_STATE_DONE;
	sc9830_irq_stop(rot);
	ret = sc9830_rot_drain(&rot->io, &drain_reads);
	if (ret) {
		sc9830_fault(rot);
		state = VB2_BUF_STATE_ERROR;
		goto finish;
	}
	ret = sc9830_rot_translation(&rot->io, &after);
	if (ret || after != translation) {
		sc9830_fault(rot);
		state = VB2_BUF_STATE_ERROR;
		goto finish;
	}
release:
	if (voted && icc_set_bw(rot->path, 0, 0)) {
		sc9830_fault(rot);
		state = VB2_BUF_STATE_ERROR;
		sc9830_unpin(rot);
		goto finish;
	}
	sc9830_unpin(rot);
	if (sc9830_release_resources(rot) < 0)
		state = VB2_BUF_STATE_ERROR;
	if (state == VB2_BUF_STATE_DONE)
		dev_info(rot->dev,
			 "SC9830_ROTATOR_JOB_OK width=%u height=%u angle=%u bytes=%u bw=%u irq=%08x mmu_before=%08x mmu_start=%08x mmu_after=%08x drain_reads=%u\n",
			 ctx->source.width, ctx->source.height, ctx->rotation,
			 ctx->source.sizeimage + ctx->destination.sizeimage,
			 vote, (u32)SC9830_ROT_IRQ, translation, start_translation,
			 after, drain_reads);
finish:
	v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (state == VB2_BUF_STATE_DONE)
		v4l2_m2m_buf_copy_metadata(src, dst);
	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);
	spin_lock_irqsave(&rot->irq_lock, flags);
	rot->job_ctx = NULL;
	spin_unlock_irqrestore(&rot->irq_lock, flags);
	v4l2_m2m_job_finish(rot->m2m, ctx->fh.m2m_ctx);
}

static void sc9830_device_run(void *priv)
{
	struct sc9830_ctx *ctx = priv;
	struct sc9830_rot *rot = ctx->rot;
	unsigned long flags;

	spin_lock_irqsave(&rot->irq_lock, flags);
	rot->job_ctx = ctx;
	spin_unlock_irqrestore(&rot->irq_lock, flags);
	schedule_work(&rot->work);
}

static void sc9830_job_abort(void *priv)
{
	struct sc9830_ctx *ctx = priv;
	unsigned long flags;

	spin_lock_irqsave(&ctx->rot->irq_lock, flags);
	atomic_set(&ctx->aborted, 1);
	if (ctx->rot->job_ctx == ctx)
		complete(&ctx->rot->done);
	spin_unlock_irqrestore(&ctx->rot->irq_lock, flags);
}

static const struct v4l2_m2m_ops sc9830_m2m_ops = {
	.device_run = sc9830_device_run,
	.job_abort = sc9830_job_abort,
};

static struct sc9830_ctx *sc9830_file_ctx(struct file *file)
{
	return container_of(file->private_data, struct sc9830_ctx, fh);
}

static void sc9830_format(struct v4l2_pix_format *pix, u32 width, u32 height)
{
	memset(pix, 0, sizeof(*pix));
	pix->width = width;
	pix->height = height;
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = width * 2;
	pix->sizeimage = width * height * 2;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
}

static void sc9830_capture_format(struct sc9830_ctx *ctx)
{
	bool swap = ctx->rotation != 180;

	sc9830_format(&ctx->destination,
		      swap ? ctx->source.height : ctx->source.width,
		      swap ? ctx->source.width : ctx->source.height);
}

static bool sc9830_busy(struct sc9830_ctx *ctx)
{
	return vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	       vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
}

static int sc9830_control(struct v4l2_ctrl *ctrl)
{
	struct sc9830_ctx *ctx = container_of(ctrl->handler,
					 struct sc9830_ctx, controls);

	if (ctx->fh.m2m_ctx && sc9830_busy(ctx))
		return -EBUSY;
	ctx->rotation = ctrl->val;
	sc9830_capture_format(ctx);
	return 0;
}

static const struct v4l2_ctrl_ops sc9830_ctrl_ops = {
	.s_ctrl = sc9830_control,
};

static int sc9830_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "sc9830-rotator", sizeof(cap->driver));
	strscpy(cap->card, "SC9830 DCAM rotator", sizeof(cap->card));
	return 0;
}

static int sc9830_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *format)
{
	if (format->index)
		return -EINVAL;
	format->pixelformat = V4L2_PIX_FMT_RGB565;
	return 0;
}

static int sc9830_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc9830_ctx *ctx = sc9830_file_ctx(file);

	f->fmt.pix = V4L2_TYPE_IS_OUTPUT(f->type) ? ctx->source : ctx->destination;
	return 0;
}

static int sc9830_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc9830_ctx *ctx = sc9830_file_ctx(file);
	u32 width, height;

	if (!V4L2_TYPE_IS_OUTPUT(f->type)) {
		f->fmt.pix = ctx->destination;
		return 0;
	}
	width = clamp_t(u32, f->fmt.pix.width, 2, 8190) & ~1U;
	height = clamp_t(u32, f->fmt.pix.height, 2, 8190) & ~1U;
	sc9830_format(&f->fmt.pix, width, height);
	return 0;
}

static int sc9830_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc9830_ctx *ctx = sc9830_file_ctx(file);

	if (sc9830_busy(ctx))
		return -EBUSY;
	sc9830_try_fmt(file, priv, f);
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->source = f->fmt.pix;
		sc9830_capture_format(ctx);
	}
	return 0;
}

static int sc9830_queue_setup(struct vb2_queue *q, unsigned int *count,
			      unsigned int *planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct sc9830_ctx *ctx = vb2_get_drv_priv(q);
	u32 size = V4L2_TYPE_IS_OUTPUT(q->type) ? ctx->source.sizeimage :
		   ctx->destination.sizeimage;

	if (READ_ONCE(ctx->rot->faulted))
		return -EIO;
	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int sc9830_buf_prepare(struct vb2_buffer *vb)
{
	struct sc9830_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	u32 size = V4L2_TYPE_IS_OUTPUT(vb->type) ? ctx->source.sizeimage :
		   ctx->destination.sizeimage;

	if (READ_ONCE(ctx->rot->faulted))
		return -EIO;
	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void sc9830_buf_queue(struct vb2_buffer *vb)
{
	struct sc9830_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int sc9830_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct sc9830_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buffer;

	if (READ_ONCE(ctx->rot->faulted)) {
		while ((buffer = V4L2_TYPE_IS_OUTPUT(q->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_QUEUED);
		return -EIO;
	}
	atomic_set(&ctx->aborted, 0);
	return 0;
}

static void sc9830_stop_streaming(struct vb2_queue *q)
{
	struct sc9830_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buffer;

	while ((buffer = V4L2_TYPE_IS_OUTPUT(q->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops sc9830_queue_ops = {
	.queue_setup = sc9830_queue_setup,
	.buf_prepare = sc9830_buf_prepare,
	.buf_queue = sc9830_buf_queue,
	.start_streaming = sc9830_start_streaming,
	.stop_streaming = sc9830_stop_streaming,
};

static int sc9830_queue_init(void *priv, struct vb2_queue *src,
			     struct vb2_queue *dst)
{
	struct sc9830_ctx *ctx = priv;
	struct vb2_queue *queues[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < 2; i++) {
		queues[i]->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE :
				 V4L2_BUF_TYPE_VIDEO_OUTPUT;
		queues[i]->io_modes = VB2_MMAP;
		queues[i]->drv_priv = ctx;
		queues[i]->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		queues[i]->ops = &sc9830_queue_ops;
		queues[i]->mem_ops = &vb2_dma_contig_memops;
		queues[i]->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		queues[i]->lock = &ctx->rot->lock;
		queues[i]->dev = ctx->rot->dev;
		ret = vb2_queue_init(queues[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int sc9830_open(struct file *file)
{
	struct sc9830_rot *rot = video_drvdata(file);
	struct sc9830_ctx *ctx;
	int ret;

	if (READ_ONCE(rot->faulted))
		return -EIO;
	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->rot = rot;
	ctx->rotation = 90;
	sc9830_format(&ctx->source, 32, 16);
	sc9830_capture_format(ctx);
	v4l2_fh_init(&ctx->fh, &rot->video);
	file->private_data = &ctx->fh;
	v4l2_ctrl_handler_init(&ctx->controls, 1);
	v4l2_ctrl_new_std(&ctx->controls, &sc9830_ctrl_ops,
			  V4L2_CID_ROTATE, 90, 270, 90, 90);
	ret = ctx->controls.error;
	if (ret)
		goto error;
	ctx->fh.ctrl_handler = &ctx->controls;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(rot->m2m, ctx, sc9830_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto error;
	}
	v4l2_fh_add(&ctx->fh, file);
	return 0;
error:
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int sc9830_close(struct file *file)
{
	struct sc9830_ctx *ctx = sc9830_file_ctx(file);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->controls);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations sc9830_fops = {
	.owner = THIS_MODULE,
	.open = sc9830_open,
	.release = sc9830_close,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops sc9830_ioctl_ops = {
	.vidioc_querycap = sc9830_querycap,
	.vidioc_enum_fmt_vid_cap = sc9830_enum_fmt,
	.vidioc_enum_fmt_vid_out = sc9830_enum_fmt,
	.vidioc_g_fmt_vid_cap = sc9830_get_fmt,
	.vidioc_g_fmt_vid_out = sc9830_get_fmt,
	.vidioc_try_fmt_vid_cap = sc9830_try_fmt,
	.vidioc_try_fmt_vid_out = sc9830_try_fmt,
	.vidioc_s_fmt_vid_cap = sc9830_set_fmt,
	.vidioc_s_fmt_vid_out = sc9830_set_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int sc9830_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc9830_rot *rot;
	int ret;

	rot = devm_kzalloc(dev, sizeof(*rot), GFP_KERNEL);
	if (!rot)
		return -ENOMEM;
	rot->dev = dev;
	rot->dcam = devm_platform_ioremap_resource_byname(pdev, "dcam");
	if (IS_ERR(rot->dcam))
		return PTR_ERR(rot->dcam);
	rot->mmu = devm_platform_ioremap_resource_byname(pdev, "mmu");
	if (IS_ERR(rot->mmu))
		return PTR_ERR(rot->mmu);
	rot->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,pmu");
	if (IS_ERR(rot->pmu))
		return PTR_ERR(rot->pmu);
	rot->mm_ahb = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,mm-ahb");
	if (IS_ERR(rot->mm_ahb))
		return PTR_ERR(rot->mm_ahb);
	rot->dcam_clk = devm_clk_get(dev, "dcam");
	if (IS_ERR(rot->dcam_clk))
		return PTR_ERR(rot->dcam_clk);
	rot->mmu_clk = devm_clk_get(dev, "mmu");
	if (IS_ERR(rot->mmu_clk))
		return PTR_ERR(rot->mmu_clk);
	rot->rot_reset = devm_reset_control_get_exclusive(dev, "rotator");
	if (IS_ERR(rot->rot_reset))
		return PTR_ERR(rot->rot_reset);
	rot->all_reset = devm_reset_control_get_exclusive(dev, "all");
	if (IS_ERR(rot->all_reset))
		return PTR_ERR(rot->all_reset);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	rot->path = of_icc_get(dev, "dcam-ddr");
	if (IS_ERR(rot->path))
		return PTR_ERR(rot->path);
	if (!rot->path)
		return -ENODEV;
	rot->irq = platform_get_irq(pdev, 0);
	if (rot->irq < 0) {
		ret = rot->irq;
		goto put_path;
	}
	mutex_init(&rot->lock);
	spin_lock_init(&rot->irq_lock);
	init_completion(&rot->done);
	INIT_WORK(&rot->work, sc9830_job);
	rot->io = (struct sc9830_rot_io) {
		.ctx = rot, .read = sc9830_read, .write = sc9830_write,
		.mmu_read = sc9830_mmu_read, .reset = sc9830_reset,
	};
	ret = devm_request_irq(dev, rot->irq, sc9830_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), rot);
	if (ret)
		goto put_path;
	ret = v4l2_device_register(dev, &rot->v4l2);
	if (ret)
		goto put_path;
	rot->m2m = v4l2_m2m_init(&sc9830_m2m_ops);
	if (IS_ERR(rot->m2m)) {
		ret = PTR_ERR(rot->m2m);
		goto unregister_v4l2;
	}
	strscpy(rot->video.name, "sc9830-rotator", sizeof(rot->video.name));
	rot->video.v4l2_dev = &rot->v4l2;
	rot->video.fops = &sc9830_fops;
	rot->video.ioctl_ops = &sc9830_ioctl_ops;
	rot->video.release = video_device_release_empty;
	rot->video.lock = &rot->lock;
	rot->video.vfl_dir = VFL_DIR_M2M;
	rot->video.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(&rot->video, rot);
	pm_runtime_enable(dev);
	ret = video_register_device(&rot->video, VFL_TYPE_VIDEO, -1);
	if (!ret)
		return 0;
	pm_runtime_disable(dev);
	v4l2_m2m_release(rot->m2m);
unregister_v4l2:
	v4l2_device_unregister(&rot->v4l2);
put_path:
	icc_put(rot->path);
	return ret;
}

static const struct of_device_id sc9830_of_match[] = {
	{ .compatible = "sprd,sc9830-rotator" },
	{ }
};

static struct platform_driver sc9830_driver = {
	.probe = sc9830_probe,
	.driver = {
		.name = "sc9830-rotator",
		.of_match_table = sc9830_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sc9830_driver);

MODULE_DESCRIPTION("SC9830 DCAM RGB565 rotator");
MODULE_IMPORT_NS("DMA_BUF");
MODULE_LICENSE("GPL");
