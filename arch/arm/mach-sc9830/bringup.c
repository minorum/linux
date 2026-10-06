// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>

#include <asm/mach/arch.h>

static DEFINE_SPINLOCK(sc9830_usb_clock_lock);

static void __init sc9830_usb_clock_init(struct device_node *node)
{
	void __iomem *base = of_iomap(node, 0);
	struct clk_hw *clock;
	int ret;

	if (!base)
		panic("SC9830 USB clock mapping failed\n");
	clock = clk_hw_register_gate(NULL, "sc9830-usb", NULL, 0, base, 4, 0,
				     &sc9830_usb_clock_lock);
	if (IS_ERR(clock))
		panic("SC9830 USB clock registration failed\n");
	ret = of_clk_add_hw_provider(node, of_clk_hw_simple_get, clock);
	if (ret)
		panic("SC9830 USB clock provider failed\n");
}

CLK_OF_DECLARE(sc9830_usb_clock, "sprd,sc9830-usb-clock",
	       sc9830_usb_clock_init);

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
	sc9830_set_bits(0x402e0000, BIT(11) | BIT(15));
	sc9830_set_bits(0x20e00000, BIT(4));
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

static void __init sc9830_usb_snapshot(void)
{
	void __iomem *base = ioremap(0x20200000, 0x1000);
	u32 hwcfg4, count, fifo;

	if (!base)
		return;
	hwcfg4 = readl(base + 0x50);
	pr_info("SC9830_USB_INHERITED cfg2=%08x cfg3=%08x cfg4=%08x rx=%08x np=%08x\n",
		readl(base + 0x48), readl(base + 0x4c), hwcfg4,
		readl(base + 0x24), readl(base + 0x28));
	count = hwcfg4 & BIT(25) ? (hwcfg4 >> 26) & 0xf : hwcfg4 & 0xf;
	for (fifo = 1; fifo <= count; fifo++)
		pr_info("SC9830_USB_INHERITED tx%u=%08x\n", fifo,
			readl(base + 0x100 + fifo * 4));
	iounmap(base);
}

static void __init sc9830_init_machine(void)
{
	sc9830_emmc_setup();
	sc9830_usb_snapshot();
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
