/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SC2331_BOOT_H
#define SC2331_BOOT_H

struct device;
struct sc2331_boot;

struct sc2331_boot *sc2331_boot_get(struct device *dev);
void sc2331_boot_arm(struct sc2331_boot *boot);
int sc2331_boot_wait_ready(struct sc2331_boot *boot);

#endif
