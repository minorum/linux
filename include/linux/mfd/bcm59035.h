/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MFD_BCM59035_H
#define _LINUX_MFD_BCM59035_H

struct device;

int bcm59035_irq_get_pending(struct device *dev, unsigned int bank);

#endif
