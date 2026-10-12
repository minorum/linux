// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>

static void __init sc9830_set_bits(phys_addr_t address, u32 bits)
{
	void __iomem *reg = ioremap(address, sizeof(u32));

	if (!reg)
		panic("SC9830 gate mapping failed\n");
	writel(readl(reg) | bits, reg);
	readl(reg);
	iounmap(reg);
}

static void __init sc9830_enable_irq(phys_addr_t address, u32 bits)
{
	void __iomem *reg = ioremap(address, sizeof(u32));

	if (!reg)
		panic("SC9830 IRQ mapping failed\n");
	writel(bits, reg);
	readl(reg);
	iounmap(reg);
}

static void __init sc9830_init_irq(void)
{
	sc9830_set_bits(0x71300000, BIT(19) | BIT(20));
	sc9830_set_bits(0x402e0000, BIT(15));
	sc9830_enable_irq(0x71400008, BIT(28));
	sc9830_enable_irq(0x71500008, BIT(6) | BIT(23) | BIT(28));
	sc9830_enable_irq(0x40200008, BIT(2) | BIT(4));
	irqchip_init();
}

static void __init sc9830_emmc_setup(void)
{
	static const u16 offsets[] = {
		0x1b0, 0x1c4, 0x1a0, 0x1a4, 0x1a8, 0x1ac,
		0x1b4, 0x1b8, 0x1c0, 0x1c8, 0x1cc,
	};
	void __iomem *ahb, *clock, *pins;
	u32 reset;
	int i;

	if (!IS_ENABLED(CONFIG_MMC_SDHCI_SPRD))
		return;
	ahb = ioremap(0x20e00000, 8);
	clock = ioremap(0x21500048, 4);
	pins = ioremap(0x402a0000, 0x1000);
	if (!ahb || !clock || !pins)
		panic("SC9830 eMMC mapping failed\n");
	writel(readl(ahb) | BIT(11), ahb);
	readl(ahb);
	for (i = 0; i < ARRAY_SIZE(offsets); i++)
		writel(offsets[i] == 0x1b0 ? 0x100010 : 0x80090,
		       pins + offsets[i]);
	writel((readl(clock) & ~3) | 3, clock);
	readl(clock);
	reset = readl(ahb + 4);
	writel(reset | BIT(14), ahb + 4);
	readl(ahb + 4);
	usleep_range(10, 20);
	writel(reset & ~BIT(14), ahb + 4);
	readl(ahb + 4);
	iounmap(pins);
	iounmap(clock);
	iounmap(ahb);
}

static void __init sc9830_init_machine(void)
{
	sc9830_emmc_setup();
}

static const char * const sc9830_dt_compat[] __initconst = {
	"samsung,j3xnlte",
	NULL,
};

DT_MACHINE_START(SC9830, "SC9830 RAM bring-up")
	.dt_compat = sc9830_dt_compat,
	.init_irq = sc9830_init_irq,
	.init_machine = sc9830_init_machine,
MACHINE_END
