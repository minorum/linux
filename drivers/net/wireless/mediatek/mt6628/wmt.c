// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "runtime.h"

int mt6628_wmt_init(struct mt6628_dev *dev)
{
	struct device *consumer = &dev->func->dev;
	struct device *supplier;
	struct fwnode_handle *node;
	struct regmap *map;
	unsigned int chip;
	int ret;

	node = fwnode_find_reference(dev_fwnode(dev->func->card->host->parent),
				     "mmc-pwrseq", 0);
	if (IS_ERR(node))
		return PTR_ERR(node);
	supplier = get_dev_from_fwnode(node);
	fwnode_handle_put(node);
	if (!supplier)
		return -EPROBE_DEFER;
	if (!device_link_add(consumer, supplier, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto put;
	}
	map = dev_get_regmap(supplier, "wmt");
	if (!map) {
		ret = -EPROBE_DEFER;
		goto put;
	}
	ret = regmap_read(map, 0x80000008, &chip);
	if (!ret && chip != 0x6628)
		ret = -ENODEV;
	if (!ret)
		dev->wmt = map;
put:
	put_device(supplier);
	return ret;
}
