.. SPDX-License-Identifier: GPL-2.0

====
Lima
====

Client statistics
=================

DRM fdinfo reports allocated/shared GEM memory and cumulative GP and PP elapsed
job time. The PP engine represents a pixel job, rather than the sum of time on
individual pixel processors. Time includes dispatch and interrupt latency and
heap recovery. Completed and timed-out intervals are accounted; a running job
does not contribute until its interval ends. Reading fdinfo does not resume
the GPU.

Mali-400 performance monitors
============================

``DRM_LIMA_PARAM_PERFMON`` returns one when the monitor interface is implemented
and zero otherwise. Unknown parameters on older kernels return ``EINVAL``.
Mali-450 monitors are not implemented. Monitor operations and monitored job
submission require ``CAP_PERFMON`` or ``CAP_SYS_ADMIN``.
``DRM_LIMA_PARAM_PP_MASK`` identifies usable physical PP instances by bit number;
it does not assume that those instances are contiguous.

``DRM_IOCTL_LIMA_PERFMON_CREATE`` creates a file-local, nonzero monitor ID.
All padding must be zero. Its twelve event selectors have these slots:

* 0 and 1: geometry processor counters.
* 2 through 9: two counters per physical PP0 through PP3.
* 10 and 11: shared L2 counters.

Set unused slots to ``DRM_LIMA_PERFMON_DISABLED``. PP event zero is valid.
Requests for unavailable PP/MMU instances, unknown selectors or an entirely
disabled monitor return ``EINVAL``. Accepted selector encodings follow the
Arm Mali-4xx event catalogue; an encoding's presence does not establish that
every hardware revision implements its event or every L2 port is connected.

Attach a monitor by setting ``perfmon_id`` in ``drm_lima_gem_submit``. Zero
disables monitoring. The appended ID and padding preserve old submission
layouts through DRM's zero-extension of short ioctl arguments. Counter
selection is immutable; create a new monitor to select a different group.
The monitor may be shared by multiple contexts belonging to the same DRM file.

GP and PP jobs normally run independently. A job measuring shared L2 waits
for all preceding GP/PP jobs, and subsequent jobs wait for that job to finish.
This prevents concurrent clients from contaminating its L2 interval, at the
cost of changing overlap and cache behavior during profiling. L2 counters
cover the monitored job interval, not normal concurrent whole-device behavior.

``DRM_IOCTL_LIMA_PERFMON_GET_VALUES`` returns cumulative samples, with zero
in disabled slots. Without ``DRM_LIMA_PERFMON_WAIT``, pending jobs return
``EBUSY``. With that flag, the call waits interruptibly for queued monitor jobs
to retire. Concurrent submissions can extend the wait; callers should end
their monitored submissions before requesting final values.

``status`` always includes ``DRM_LIMA_PERFMON_RAW32``: each hardware sample
is only 32 bits, even though the sum of samples is stored in 64 bits. Wraps
within a job are not reconstructed. Results must not be presented as exact
64-bit hardware totals. A reset, timeout or incomplete started job sets
``DRM_LIMA_PERFMON_LOST``; those results are incomplete. Accepted jobs canceled
before execution also set this status. Unpublished submission rollback does
not invalidate earlier samples. Reads do not wake the
GPU, and monitor creation does not hold a runtime-PM reference.

``DRM_IOCTL_LIMA_PERFMON_DESTROY`` removes the ID. Queued jobs retain their
references, so destroying an ID or closing a file cannot free a monitor still
in use. Unknown IDs return ``ENOENT``. Driver errors discard the affected
sample and retain the lost-data status. GP heap recovery continues the same
hardware sample without resetting it.
