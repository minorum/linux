// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>

#include "mt6628.h"

static int mt6628_probe(struct sdio_func *func,
			const struct sdio_device_id *id)
{
	int ret;

	if (func->num != 1)
		return -ENODEV;
	if (func->card->sdio_funcs != 1)
		return -EOPNOTSUPP;
	ret = mt6628_sdio_bootstrap(func);
	if (ret)
		return ret;
	dev_info(&func->dev, "firmware initialized; station frontend pending\n");
	return 0;
}

static void mt6628_remove(struct sdio_func *func)
{
	int ret;

	sdio_claim_host(func);
	sdio_writeb(func, 2, 0x04, &ret);
	if (ret)
		dev_warn(&func->dev, "interrupt disable failed: %d\n", ret);
	ret = sdio_disable_func(func);
	if (ret)
		dev_warn(&func->dev, "function disable failed: %d\n", ret);
	sdio_release_host(func);
}

static const struct sdio_device_id mt6628_ids[] = {
	{ SDIO_DEVICE(0x037a, 0x6628) },
	{}
};
MODULE_DEVICE_TABLE(sdio, mt6628_ids);

static struct sdio_driver mt6628_driver = {
	.name = "mt6628",
	.id_table = mt6628_ids,
	.probe = mt6628_probe,
	.remove = mt6628_remove,
};
module_sdio_driver(mt6628_driver);

MODULE_DESCRIPTION("MediaTek MT6628 SDIO driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("mediatek/WIFI_RAM_CODE_MT6628");
