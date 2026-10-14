// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>

static const u16 sc9830_ap_offsets[] = {
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x54, 0x58, 0x5c, 0x60,
	0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c, 0x80,
	0x84, 0x88, 0x8c, 0x90, 0x94,
};

static const u16 sc9830_ahb_offsets[] = { 0, 0x38, 0x54, 0x3020, 0x40, 0x3000 };
static const u16 sc9830_apb_offsets[] = { 0, 8 };
static const u16 sc9830_aon_offsets[] = { 0, 4 };
static const u16 sc9830_pub_offsets[] = { 0x10, 0x14, 0x18 };

struct sc9830_retention_context {
	void __iomem *ap;
	void __iomem *ahb;
	void __iomem *apb;
	void __iomem *aon;
	void __iomem *pub;
	u32 ap_saved[ARRAY_SIZE(sc9830_ap_offsets)];
	u32 ahb_saved[ARRAY_SIZE(sc9830_ahb_offsets)];
	u32 apb_saved[ARRAY_SIZE(sc9830_apb_offsets)];
	u32 aon_saved[ARRAY_SIZE(sc9830_aon_offsets)];
	u32 pub_saved[ARRAY_SIZE(sc9830_pub_offsets)];
	bool save_pub;
	bool saved;
};

static void sc9830_context_read(void __iomem *base, const u16 *offsets,
				u32 *values, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		values[i] = readl(base + offsets[i]);
}

static int sc9830_context_write(void __iomem *base, u16 offset, u32 value,
				u32 mask, unsigned int delay)
{
	unsigned int i;
	int ret = 0;

	if (mask) {
		writel(value & ~mask, base + offset);
		if (readl(base + offset) != (value & ~mask))
			ret = -EIO;
		for (i = 0; i < delay; i++)
			cpu_relax();
	}
	writel(value, base + offset);
	if (readl(base + offset) != value)
		ret = -EIO;
	return ret;
}

static int sc9830_retention_context_save(struct sc9830_retention_context *ctx)
{
	if (ctx->saved)
		return -EBUSY;
	sc9830_context_read(ctx->ap, sc9830_ap_offsets, ctx->ap_saved,
			    ARRAY_SIZE(sc9830_ap_offsets));
	sc9830_context_read(ctx->ahb, sc9830_ahb_offsets, ctx->ahb_saved,
			    ARRAY_SIZE(sc9830_ahb_offsets));
	sc9830_context_read(ctx->apb, sc9830_apb_offsets, ctx->apb_saved,
			    ARRAY_SIZE(sc9830_apb_offsets));
	sc9830_context_read(ctx->aon, sc9830_aon_offsets, ctx->aon_saved,
			    ARRAY_SIZE(sc9830_aon_offsets));
	if (ctx->save_pub)
		sc9830_context_read(ctx->pub, sc9830_pub_offsets, ctx->pub_saved,
				    ARRAY_SIZE(sc9830_pub_offsets));
	ctx->saved = true;
	return 0;
}

static int sc9830_retention_context_restore(struct sc9830_retention_context *ctx)
{
	unsigned int i;
	u16 offset;
	u32 mask;
	int ret = 0;

	if (!ctx->saved)
		return 0;
	for (i = 0; i < ARRAY_SIZE(sc9830_apb_offsets); i++)
		ret |= sc9830_context_write(ctx->apb, sc9830_apb_offsets[i],
					    ctx->apb_saved[i], 0, 0);
	for (i = 0; i < ARRAY_SIZE(sc9830_ap_offsets); i++) {
		offset = sc9830_ap_offsets[i];
		mask = offset >= 0x2c && (offset < 0x40 || offset > 0x4c) ? 3 : 0;
		ret |= sc9830_context_write(ctx->ap, offset, ctx->ap_saved[i], mask, 10);
	}
	if (ctx->aon_saved[1] & BIT(11)) {
		writel(readl(ctx->aon + 4) | BIT(11), ctx->aon + 4);
		if (!(readl(ctx->aon + 4) & BIT(11)))
			ret = -EIO;
	}
	for (i = 0; i < ARRAY_SIZE(sc9830_aon_offsets); i++)
		ret |= sc9830_context_write(ctx->aon, sc9830_aon_offsets[i],
					    ctx->aon_saved[i], 0, 0);
	for (i = 0; i < ARRAY_SIZE(sc9830_ahb_offsets); i++) {
		offset = sc9830_ahb_offsets[i];
		mask = offset == 0x38 || offset == 0x54 ? 7 : 0;
		ret |= sc9830_context_write(ctx->ahb, offset, ctx->ahb_saved[i], mask, 20);
	}
	if (ctx->save_pub)
		for (i = 0; i < ARRAY_SIZE(sc9830_pub_offsets); i++)
			ret |= sc9830_context_write(ctx->pub, sc9830_pub_offsets[i],
						    ctx->pub_saved[i], 0, 0);
	if (!ret)
		ctx->saved = false;
	return ret;
}
