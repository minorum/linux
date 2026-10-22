// SPDX-License-Identifier: GPL-2.0 OR MIT
#include <linux/capability.h>
#include <linux/device.h>
#include <linux/dma-resv.h>
#include <linux/slab.h>

#include "lima_drv.h"
#include "lima_perfmon.h"
#include "lima_regs.h"
#include "lima_vm.h"

static bool lima_perfmon_valid_event(unsigned int slot, u32 event)
{
	if (slot >= 12)
		return false;
	if (event == DRM_LIMA_PERFMON_DISABLED)
		return true;
	if (slot < 2)
		return (event >= 1 && event <= 0xc) ||
		       (event >= 0xe && event <= 0x10) || event == 0x13 ||
		       (event >= 0x16 && event <= 0x19) ||
		       (event >= 0x1b && event <= 0x1c) || event == 0x1e ||
		       (event >= 0x20 && event <= 0x21);
	if (slot < 10)
		return event == 0 || (event >= 2 && event <= 7) ||
		       (event >= 9 && event <= 0xa) ||
		       (event >= 0xd && event <= 0x15) ||
		       (event >= 0x17 && event <= 0x3d);
	return event == 1 || event == 2 || (event >= 8 && event <= 0xb) ||
	       (event >= 0x10 && event <= 0x68 && (event & 0xf) <= 8);
}

static void lima_perfmon_release(struct kref *ref)
{
	kfree(container_of(ref, struct lima_perfmon, ref));
}

struct lima_perfmon *lima_perfmon_get(struct lima_perfmon *perfmon)
{
	if (perfmon)
		kref_get(&perfmon->ref);
	return perfmon;
}

void lima_perfmon_put(struct lima_perfmon *perfmon)
{
	if (perfmon)
		kref_put(&perfmon->ref, lima_perfmon_release);
}

struct lima_perfmon *lima_perfmon_find(struct drm_file *file, u32 id)
{
	struct lima_drm_priv *priv = file->driver_priv;
	struct lima_perfmon *perfmon;

	xa_lock(&priv->perfmons);
	perfmon = lima_perfmon_get(xa_load(&priv->perfmons, id));
	xa_unlock(&priv->perfmons);
	return perfmon;
}

void lima_perfmon_close(struct lima_drm_priv *priv)
{
	struct lima_perfmon *perfmon;
	unsigned long id;

	xa_for_each(&priv->perfmons, id, perfmon)
		lima_perfmon_put(perfmon);
	xa_destroy(&priv->perfmons);
}

int lima_perfmon_create_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_perfmon_create *args = data;
	struct lima_device *ldev = to_lima_dev(dev);
	struct lima_drm_priv *priv = file->driver_priv;
	struct lima_perfmon *perfmon;
	bool enabled = false;
	int i, ret;

	if (!perfmon_capable())
		return -EPERM;
	if (ldev->id != lima_gpu_mali400)
		return -EOPNOTSUPP;
	if (args->pad)
		return -EINVAL;
	for (i = 0; i < DRM_LIMA_PERFMON_COUNTERS; i++) {
		if (!lima_perfmon_valid_event(i, args->events[i]))
			return -EINVAL;
		if (args->events[i] == DRM_LIMA_PERFMON_DISABLED)
			continue;
		if (i >= 2 && i < 10 &&
		    (!ldev->ip[lima_ip_pp0 + (i - 2) / 2].present ||
		     !ldev->ip[lima_ip_ppmmu0 + (i - 2) / 2].present))
			return -EINVAL;
		enabled = true;
	}
	if (!enabled)
		return -EINVAL;
	perfmon = kzalloc_obj(*perfmon);
	if (!perfmon)
		return -ENOMEM;
	kref_init(&perfmon->ref);
	spin_lock_init(&perfmon->lock);
	init_waitqueue_head(&perfmon->wait);
	memcpy(perfmon->events, args->events, sizeof(perfmon->events));
	perfmon->l2 = args->events[10] != DRM_LIMA_PERFMON_DISABLED ||
		      args->events[11] != DRM_LIMA_PERFMON_DISABLED;
	perfmon->status = DRM_LIMA_PERFMON_RAW32;
	ret = xa_alloc(&priv->perfmons, &args->id, perfmon, xa_limit_32b, GFP_KERNEL);
	if (ret)
		lima_perfmon_put(perfmon);
	return ret;
}

int lima_perfmon_destroy_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_perfmon_destroy *args = data;
	struct lima_drm_priv *priv = file->driver_priv;
	struct lima_perfmon *perfmon;

	if (!perfmon_capable())
		return -EPERM;
	if (args->pad)
		return -EINVAL;
	perfmon = xa_erase(&priv->perfmons, args->id);
	if (!perfmon)
		return -ENOENT;
	lima_perfmon_put(perfmon);
	return 0;
}

int lima_perfmon_get_values_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_perfmon_get_values *args = data;
	struct lima_perfmon *perfmon;
	unsigned long irqflags;
	int ret = 0;

	if (!perfmon_capable())
		return -EPERM;
	if (args->pad || args->flags & ~DRM_LIMA_PERFMON_WAIT)
		return -EINVAL;
	perfmon = lima_perfmon_find(file, args->id);
	if (!perfmon)
		return -ENOENT;
	if (args->flags & DRM_LIMA_PERFMON_WAIT)
		ret = wait_event_interruptible(perfmon->wait, !atomic_read(&perfmon->pending));
	else if (atomic_read(&perfmon->pending))
		ret = -EBUSY;
	if (!ret) {
		spin_lock_irqsave(&perfmon->lock, irqflags);
		memcpy(args->values, perfmon->values, sizeof(args->values));
		args->status = perfmon->status;
		spin_unlock_irqrestore(&perfmon->lock, irqflags);
	}
	lima_perfmon_put(perfmon);
	return ret;
}

void lima_perfmon_task_init(struct lima_sched_task *task, struct lima_perfmon *perfmon)
{
	task->perfmon = lima_perfmon_get(perfmon);
	if (perfmon)
		atomic_inc(&perfmon->pending);
}

void lima_perfmon_task_fini(struct lima_sched_task *task)
{
	struct lima_perfmon *perfmon = task->perfmon;
	unsigned long irqflags;

	if (!perfmon)
		return;
	spin_lock_irqsave(&perfmon->lock, irqflags);
	if (task->perfmon_started ||
	    (!task->perfmon_sampled && dma_fence_is_signaled(&task->base.s_fence->finished)))
		perfmon->status |= DRM_LIMA_PERFMON_LOST;
	spin_unlock_irqrestore(&perfmon->lock, irqflags);
	if (atomic_dec_and_test(&perfmon->pending))
		wake_up_all(&perfmon->wait);
	lima_perfmon_put(perfmon);
}

void lima_perfmon_start_l2(struct lima_sched_pipe *pipe, struct lima_sched_task *task)
{
	struct lima_perfmon *perfmon = task->perfmon;
	struct lima_ip *ip = &pipe->ldev->ip[lima_ip_l2_cache0];
	unsigned long irqflags;
	int i;

	if (!perfmon)
		return;
	spin_lock_irqsave(&perfmon->lock, irqflags);
	task->perfmon_started = true;
	spin_unlock_irqrestore(&perfmon->lock, irqflags);
	if (!perfmon->l2)
		return;
	for (i = 0; i < 2; i++) {
		u32 event = perfmon->events[10 + i];

		writel(event == DRM_LIMA_PERFMON_DISABLED ? 0 : event,
		       ip->iomem + LIMA_L2_CACHE_PERFCNT_SRC0 + i * 8);
		writel(0, ip->iomem + LIMA_L2_CACHE_PERFCNT_VAL0 + i * 8);
	}
}

void lima_perfmon_start_ip(struct lima_ip *ip, struct lima_sched_task *task)
{
	struct lima_perfmon *perfmon = task->perfmon;
	bool gp = ip->id == lima_ip_gp;
	u32 source = gp ? LIMA_GP_PERF_CNT_0_SRC : LIMA_PP_PERF_CNT_0_SRC;
	u32 enable = gp ? LIMA_GP_PERF_CNT_0_ENABLE : LIMA_PP_PERF_CNT_0_ENABLE;
	unsigned int slot = gp ? 0 : 2 + (ip->id - lima_ip_pp0) * 2;
	int i;

	if (!perfmon)
		return;
	for (i = 0; i < 2; i++) {
		u32 event = perfmon->events[slot + i];
		u32 stride = gp ? 4 : 32;

		if (event != DRM_LIMA_PERFMON_DISABLED)
			writel(event, ip->iomem + source + i * stride);
		writel(event != DRM_LIMA_PERFMON_DISABLED,
		       ip->iomem + enable + i * stride);
	}
}

void lima_perfmon_stop(struct lima_sched_pipe *pipe, struct lima_sched_task *task, bool capture)
{
	struct lima_perfmon *perfmon = task->perfmon;
	u32 values[DRM_LIMA_PERFMON_COUNTERS] = {};
	bool gp = pipe == &pipe->ldev->pipe[lima_pipe_gp];
	unsigned long irqflags;
	int i, j, count;

	if (!perfmon)
		return;
	spin_lock_irqsave(&perfmon->lock, irqflags);
	if (!task->perfmon_started)
		goto out;
	task->perfmon_started = false;
	task->perfmon_sampled |= capture;
	count = gp ? 1 : ((struct drm_lima_m400_pp_frame *)task->frame)->num_pp;
	if (capture) {
		for (i = 0; i < count; i++) {
			struct lima_ip *ip = pipe->processor[i];
			unsigned int slot = gp ? 0 : 2 + (ip->id - lima_ip_pp0) * 2;
			u32 reg = gp ? LIMA_GP_PERF_CNT_0_VALUE : LIMA_PP_PERF_CNT_0_VALUE;

			for (j = 0; j < 2; j++)
				if (perfmon->events[slot + j] != DRM_LIMA_PERFMON_DISABLED)
					values[slot + j] =
						readl(ip->iomem + reg + j * (gp ? 4 : 32));
		}
	}
	if (perfmon->l2) {
		struct lima_ip *ip = &pipe->ldev->ip[lima_ip_l2_cache0];

		for (i = 0; i < 2; i++) {
			if (capture && perfmon->events[10 + i] != DRM_LIMA_PERFMON_DISABLED)
				values[10 + i] =
					readl(ip->iomem + LIMA_L2_CACHE_PERFCNT_VAL0 + i * 8);
			writel(0, ip->iomem + LIMA_L2_CACHE_PERFCNT_SRC0 + i * 8);
		}
	}
	for (i = 0; i < DRM_LIMA_PERFMON_COUNTERS; i++)
		perfmon->values[i] += values[i];
	if (capture) {
		for (i = 0; i < count; i++) {
			struct lima_ip *ip = pipe->processor[i];
			unsigned int slot = gp ? 0 : 2 + (ip->id - lima_ip_pp0) * 2;
			u32 enable = gp ? LIMA_GP_PERF_CNT_0_ENABLE : LIMA_PP_PERF_CNT_0_ENABLE;
			u32 limit = gp ? LIMA_GP_PERF_CNT_0_LIMIT : LIMA_PP_PERF_CNT_0_LIMIT;

			dev_dbg(pipe->ldev->dev,
				"%s perfmon rawstat=%08x mask=%08x enable0=%08x limit0=%08x value0=%08x value1=%08x\n",
				lima_ip_name(ip),
				readl(ip->iomem + (gp ? LIMA_GP_INT_RAWSTAT : LIMA_PP_INT_RAWSTAT)),
				readl(ip->iomem + (gp ? LIMA_GP_INT_MASK : LIMA_PP_INT_MASK)),
				readl(ip->iomem + enable), readl(ip->iomem + limit),
				values[slot], values[slot + 1]);
		}
	}
	if (!capture)
		perfmon->status |= DRM_LIMA_PERFMON_LOST;
out:
	spin_unlock_irqrestore(&perfmon->lock, irqflags);
}

int lima_perfmon_add_deps(struct lima_sched_task *task)
{
	struct dma_resv *resv = &task->vm->dev->perfmon_resv;
	struct dma_resv_iter cursor;
	struct dma_fence *fence;
	enum dma_resv_usage usage = task->perfmon && task->perfmon->l2 ?
		DMA_RESV_USAGE_READ : DMA_RESV_USAGE_WRITE;
	int ret;

	ret = dma_resv_reserve_fences(resv, 1);
	if (ret)
		return ret;
	dma_resv_for_each_fence(&cursor, resv, usage, fence) {
		ret = drm_sched_job_add_dependency(&task->base, dma_fence_get(fence));
		if (ret)
			return ret;
	}
	return 0;
}
