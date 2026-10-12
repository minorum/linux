.. SPDX-License-Identifier: GPL-2.0-only

SC9830 DDR AXI PMU
=================

The ``sc9830_ddr`` PMU provides system-wide counting on ten AXI ports.
``port`` selects 0 through 9. ``event`` selects a fixed result register:

======= =====================
Event   Counter
======= =====================
0       read_transactions
1       read_bandwidth_raw
2       read_latency_raw
3       write_transactions
4       write_bandwidth_raw
5       write_latency_raw
======= =====================

For example::

  perf stat -a -e sc9830_ddr/read_transactions,port=0/ sleep 1

Sampling, task attribution and privilege exclusions are unsupported. Distinct
counters on one port share a window; one event can occupy each fixed counter.
Groups containing duplicate counters or another hardware PMU are rejected.

Counts sum completed windows into 64-bit software totals. The native window
length is 6400000 clock ticks; its physical time and raw bandwidth/latency units
are uncalibrated. Partial windows at event start/stop and same-port event changes
are discarded, including a window completing during the boundary operation.
Events on other ports continue. Interrupt service delays cause gaps between
acknowledgements; raw 32-bit hardware counter overflow cannot be reconstructed.
These counts are not calibrated bytes, cycles or continuous utilization.

The driver preserves the inherited match configuration. Firmware and other
agents must leave monitoring idle: a Linux MMIO claim cannot exclude firmware.
The driver refuses inherited global counting, reset bits, active monitor
controls and unknown writable controls. It examines only already clocked
unowned ports and enables only the requested port. An inherited clock gate
alone does not establish monitoring ownership. Last-owner deletion restores
the saved writable control, window and gate state. Counter and match history
are destructive and cannot be restored.

A clock lost to another owner or failed completion acknowledgement stops
accounting for that port and emits a kernel error. Reopen events only after
the conflicting owner or hardware fault has been resolved.
