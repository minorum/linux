// SPDX-License-Identifier: GPL-2.0-only
#include <linux/etherdevice.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>

#include "runtime.h"

static void mt6628_remove(struct sdio_func *func);

static void mt6628_hw_stop(struct mt6628_dev *dev)
{
	struct sdio_func *func = dev->func;
	int ret;

	ret = mt6628_sdio_stop(dev);
	if (ret)
		dev_warn(&func->dev, "transport stop failed: %d\n", ret);
	sdio_claim_host(func);
	sdio_writeb(func, 2, 0x04, &ret);
	if (ret)
		dev_warn(&func->dev, "interrupt disable failed: %d\n", ret);
	ret = sdio_disable_func(func);
	if (ret)
		dev_warn(&func->dev, "function disable failed: %d\n", ret);
	sdio_release_host(func);
}

int mt6628_hw_start(struct mt6628_dev *dev)
{
	struct sdio_func *func = dev->func;
	const u8 *address;
	u8 config[12] = {};
	int ret;

	if (READ_ONCE(dev->initialized))
		return -EBUSY;
	ret = mt6628_sdio_bootstrap(func);
	if (ret)
		return ret;
	sdio_set_drvdata(func, dev);
	ret = mt6628_sdio_get_info(func, &dev->info, dev->credits);
	if (ret)
		goto stop;
	address = dev->factory.mac_addr;
	if (!is_valid_ether_addr(address)) {
		if (!dev->info.mac_valid ||
		    !is_valid_ether_addr(dev->info.permanent_addr)) {
			ret = -ENODATA;
			goto stop;
		}
		address = dev->info.permanent_addr;
	}
	if (is_valid_ether_addr(dev->mac_addr) &&
	    !ether_addr_equal(dev->mac_addr, address)) {
		ret = -ESTALE;
		goto stop;
	}
	ether_addr_copy(dev->mac_addr, address);
	ret = mt6628_sdio_start(dev);
	if (ret)
		goto stop;
	memcpy(config, dev->mac_addr, 6);
	ret = mt6628_sdio_set(dev, 0xc1, config, sizeof(config));
	if (ret)
		goto stop;
	if (dev->factory.set_tx_power) {
		ret = mt6628_sdio_set(dev, 0x28, dev->factory.tx_power,
				      sizeof(dev->factory.tx_power));
		if (ret)
			goto stop;
	}
	ret = mt6628_sdio_set(dev, 0x31, dev->factory.phy, sizeof(dev->factory.phy));
	if (ret)
		goto stop;
	return 0;
stop:
	mt6628_hw_stop(dev);
	return ret;
}

static int mt6628_probe(struct sdio_func *func,
			const struct sdio_device_id *id)
{
	struct mt6628_dev *dev;
	int ret;

	if (func->num != 1)
		return -ENODEV;
	if (func->card->sdio_funcs != 1)
		return -EOPNOTSUPP;
	dev = devm_kzalloc(&func->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->func = func;
	ret = mt6628_read_factory(&func->dev, &dev->factory);
	if (ret)
		return dev_err_probe(&func->dev, ret, "factory calibration failed\n");
	ret = mt6628_hw_start(dev);
	if (ret) {
		sdio_set_drvdata(func, NULL);
		return dev_err_probe(&func->dev, ret, "hardware startup failed\n");
	}
	ret = mt6628_radio_register(dev);
	if (ret) {
		mt6628_remove(func);
		return dev_err_probe(&func->dev, ret, "radio registration failed\n");
	}
	dev_info(&func->dev, "firmware %04x peer %04x; association pending\n",
		 dev->info.firmware_version, dev->info.peer_version);
	return 0;
}

static void mt6628_remove(struct sdio_func *func)
{
	struct mt6628_dev *dev = sdio_get_drvdata(func);

	if (dev) {
		mt6628_radio_unregister(dev);
		mt6628_hw_stop(dev);
	}
	sdio_set_drvdata(func, NULL);
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
MODULE_FIRMWARE("mediatek/mt6628-nvram.bin");
