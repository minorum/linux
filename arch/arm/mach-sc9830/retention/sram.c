// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/string.h>

#include "sleep.h"

struct sc9830_retention_sram {
	void __iomem *base;
	struct resource *vector;
	struct resource *leaf;
	u32 original_vector[64];
	u32 vector_code[64];
	u32 original_leaf[512];
	u32 payload[512];
	u32 observed[512];
	size_t bytes;
	size_t status;
	bool installed;
};

static int sc9830_retention_sram_restore(struct sc9830_retention_sram *sram)
{
	int ret = 0;

	if (!sram->installed)
		return 0;
	memcpy_toio(sram->base, sram->original_vector, sizeof(sram->original_vector));
	memcpy_fromio(sram->observed, sram->base, sizeof(sram->original_vector));
	if (memcmp(sram->original_vector, sram->observed, sizeof(sram->original_vector)))
		ret = -EIO;
	memcpy_toio(sram->base + 0x400, sram->original_leaf, sram->bytes);
	memcpy_fromio(sram->observed, sram->base + 0x400, sram->bytes);
	if (memcmp(sram->original_leaf, sram->observed, sram->bytes))
		ret = -EIO;
	if (!ret)
		sram->installed = false;
	return ret;
}

static int sc9830_retention_sram_release(struct sc9830_retention_sram *sram)
{
	if (sram->installed)
		return -EBUSY;
	if (sram->base)
		iounmap(sram->base);
	if (sram->leaf)
		release_mem_region(0x400, sram->bytes);
	if (sram->vector)
		release_mem_region(0, 0x100);
	sram->base = NULL;
	sram->leaf = NULL;
	sram->vector = NULL;
	return 0;
}

static int sc9830_retention_sram_stage(struct sc9830_retention_sram *sram,
				       u32 resume, u32 limit)
{
	size_t bytes = sc9830_retention_end - sc9830_retention_start;
	size_t reset = sc9830_retention_reset - sc9830_retention_start;
	size_t target = sc9830_retention_resume - sc9830_retention_start;
	size_t status = sc9830_retention_status - sc9830_retention_start;
	size_t loops = sc9830_retention_limit - sc9830_retention_start;
	unsigned int i;
	int ret;

	if (sram->base || sram->vector || sram->leaf || sram->installed)
		return -EBUSY;
	if (!bytes || bytes > sizeof(sram->payload) ||
	    (bytes | reset | target | status | loops | resume) & 3 ||
	    target >= bytes || loops >= bytes || status >= bytes ||
	    reset >= target || target + 4 != loops || loops + 4 != status ||
	    status + 4 != bytes || !resume || !limit)
		return -EINVAL;
	sram->bytes = bytes;
	sram->status = status;
	sram->vector = request_mem_region(0, 0x100, "sc9830-retention-vector");
	if (!sram->vector)
		return -EBUSY;
	sram->leaf = request_mem_region(0x400, bytes, "sc9830-retention-leaf");
	ret = -EBUSY;
	if (!sram->leaf)
		goto release;
	sram->base = ioremap(0, 0xc00);
	ret = -ENOMEM;
	if (!sram->base)
		goto release;
	memcpy_fromio(sram->original_vector, sram->base, sizeof(sram->original_vector));
	memcpy_fromio(sram->original_leaf, sram->base + 0x400, bytes);
	memcpy(sram->payload, sc9830_retention_start, bytes);
	sram->payload[target / 4] = resume;
	sram->payload[loops / 4] = limit;
	sram->payload[status / 4] = 0;
	sram->installed = true;
	memcpy_toio(sram->base + 0x400, sram->payload, bytes);
	memcpy_fromio(sram->observed, sram->base + 0x400, bytes);
	ret = -EIO;
	if (memcmp(sram->payload, sram->observed, bytes))
		goto restore;
	for (i = 0; i < 64; i++)
		sram->vector_code[i] = 0xe320f000;
	sram->vector_code[62] = 0xe51ff004;
	sram->vector_code[63] = 0x400 + reset;
	memcpy_toio(sram->base, sram->vector_code, 0x100);
	memcpy_fromio(sram->observed, sram->base, 0x100);
	if (memcmp(sram->vector_code, sram->observed, 0x100))
		goto restore;
	return 0;
restore:
	if (sc9830_retention_sram_restore(sram))
		return -EIO;
release:
	sc9830_retention_sram_release(sram);
	return ret;
}

static int sc9830_retention_sram_verify(struct sc9830_retention_sram *sram)
{
	if (!sram->installed)
		return -EINVAL;
	memcpy_fromio(sram->observed, sram->base + 0x400, sram->bytes);
	if (memcmp(sram->payload, sram->observed, sram->bytes))
		return -EIO;
	memcpy_fromio(sram->observed, sram->base, 0x100);
	return memcmp(sram->vector_code, sram->observed, 0x100) ? -EIO : 0;
}
