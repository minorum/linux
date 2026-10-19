// SPDX-License-Identifier: GPL-2.0-only

#include <asm/mach/arch.h>

static const char * const bcm2153_dt_compat[] = {
	"brcm,bcm2153",
	NULL,
};

DT_MACHINE_START(BCM2153_DT, "Broadcom BCM2153 AP")
	.dt_compat = bcm2153_dt_compat,
MACHINE_END
