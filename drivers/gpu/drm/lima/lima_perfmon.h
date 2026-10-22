/* SPDX-License-Identifier: GPL-2.0 OR MIT */
#ifndef __LIMA_PERFMON_H__
#define __LIMA_PERFMON_H__

#include <linux/kref.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <drm/lima_drm.h>

struct lima_ip;
struct lima_sched_pipe;
struct lima_sched_task;
struct lima_drm_priv;
struct drm_device;
struct drm_file;

struct lima_perfmon {
	struct kref ref;
	spinlock_t lock; /* Snapshot consistency */
	wait_queue_head_t wait;
	atomic_t pending;
	u32 events[DRM_LIMA_PERFMON_COUNTERS];
	u64 values[DRM_LIMA_PERFMON_COUNTERS];
	u32 status;
	bool l2;
};

struct lima_perfmon *lima_perfmon_get(struct lima_perfmon *perfmon);
void lima_perfmon_put(struct lima_perfmon *perfmon);
struct lima_perfmon *lima_perfmon_find(struct drm_file *file, u32 id);
void lima_perfmon_close(struct lima_drm_priv *priv);
int lima_perfmon_create_ioctl(struct drm_device *dev, void *data, struct drm_file *file);
int lima_perfmon_destroy_ioctl(struct drm_device *dev, void *data, struct drm_file *file);
int lima_perfmon_get_values_ioctl(struct drm_device *dev, void *data, struct drm_file *file);
void lima_perfmon_task_init(struct lima_sched_task *task, struct lima_perfmon *perfmon);
void lima_perfmon_task_fini(struct lima_sched_task *task);
void lima_perfmon_start_l2(struct lima_sched_pipe *pipe, struct lima_sched_task *task);
void lima_perfmon_start_ip(struct lima_ip *ip, struct lima_sched_task *task);
void lima_perfmon_stop(struct lima_sched_pipe *pipe, struct lima_sched_task *task, bool capture);
int lima_perfmon_add_deps(struct lima_sched_task *task);

#endif
