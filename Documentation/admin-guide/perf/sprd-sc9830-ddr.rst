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

Counter sampling, task attribution and privilege exclusions are unsupported. Distinct
counters on one port share a window; one event can occupy each fixed counter.
Groups containing duplicate counters or another hardware PMU are rejected.

Counts sum completed windows into 64-bit software totals. The native window
length is 6400000 clock ticks; its physical time and raw bandwidth/latency units
are uncalibrated. Partial windows at event start/stop and same-port event changes
are discarded, including a window completing during the boundary operation.
Events on other ports continue. Interrupt service delays cause gaps between
acknowledgements; raw 32-bit hardware counter overflow cannot be reconstructed.
These counts are not calibrated bytes, cycles or continuous utilization.

Counter events preserve the inherited match configuration. Firmware and other
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

Address matching
----------------

``address_match`` (event 6) samples matching bus transactions. ``start`` and
``end`` encode inclusive 32-bit physical bus-start-address bounds in config1
and config2. ``access`` is 0 for reads, 1 for writes, or 2 for either.
The sampling period must be 1; frequency mode and precise IP are unsupported.
There is one address event per port. Address and counter events cannot coexist,
including stopped events retaining their leases. Separate address ports can
operate together. All events remain system-wide.

For example, with a known physical range owned by the test producer::

  perf record -a -c 1 -e sc9830_ddr/address_match,port=0,access=2,start=0x82000000,end=0x82000fff/ -- sleep 1

PERF_SAMPLE_ADDR and PERF_SAMPLE_PHYS_ADDR contain the captured bus address.
The sampled IP and task describe interrupt context, not the transaction's
origin. PERF_SAMPLE_RAW contains five native-endian 32-bit words: version
(currently 1), port, captured control, captured command, and captured ID.
The ID is an uninterpreted register value; its origin and stability are unverified.
Captured data payloads are not exported because their meaning is unverified.
Counts report serviced matches; transactions while handling or rearming the
monitor may be lost. Address sampling does not enable global counter windows.

The native filter profile differs for port 6; the driver selects the installed
SC9830 profile for each port. Address events temporarily replace filters and
clear monitor history. Closing the last event restores all ten saved filter
words, the window, writable control and owned gate bit. Initial DDR training
and bootloader configuration are unchanged.
