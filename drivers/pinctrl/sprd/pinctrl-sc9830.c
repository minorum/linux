// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#define SC9830_AF_MASK GENMASK(5, 4)

struct sc9830_pinctrl {
	void __iomem *base;
	/* Pad state */
	spinlock_t lock;
};

static const struct pinctrl_pin_desc sc9830_pins[] = {
	PINCTRL_PIN(0x20 / 4, "RFSDA0"),
	PINCTRL_PIN(0x24 / 4, "RFSCK0"),
	PINCTRL_PIN(0x28 / 4, "RFSEN0"),
	PINCTRL_PIN(0x2c / 4, "RFSDA1"),
	PINCTRL_PIN(0x30 / 4, "RFSCK1"),
	PINCTRL_PIN(0x34 / 4, "RFSEN1"),
	PINCTRL_PIN(0x38 / 4, "RFCTL15"),
	PINCTRL_PIN(0x3c / 4, "RFCTL16"),
	PINCTRL_PIN(0x40 / 4, "RFCTL17"),
	PINCTRL_PIN(0x44 / 4, "RFCTL18"),
	PINCTRL_PIN(0x48 / 4, "RFCTL19"),
	PINCTRL_PIN(0x4c / 4, "RFCTL20"),
	PINCTRL_PIN(0x50 / 4, "RFCTL21"),
	PINCTRL_PIN(0x54 / 4, "RFCTL22"),
	PINCTRL_PIN(0x58 / 4, "RFCTL23"),
	PINCTRL_PIN(0x5c / 4, "RFCTL24"),
	PINCTRL_PIN(0x60 / 4, "RFCTL25"),
	PINCTRL_PIN(0x64 / 4, "RFCTL26"),
	PINCTRL_PIN(0x68 / 4, "RFCTL0"),
	PINCTRL_PIN(0x6c / 4, "RFCTL1"),
	PINCTRL_PIN(0x70 / 4, "RFCTL2"),
	PINCTRL_PIN(0x74 / 4, "RFCTL3"),
	PINCTRL_PIN(0x78 / 4, "RFCTL4"),
	PINCTRL_PIN(0x7c / 4, "RFCTL5"),
	PINCTRL_PIN(0x80 / 4, "RFCTL6"),
	PINCTRL_PIN(0x84 / 4, "RFCTL7"),
	PINCTRL_PIN(0x88 / 4, "RFCTL8"),
	PINCTRL_PIN(0x8c / 4, "RFCTL9"),
	PINCTRL_PIN(0x90 / 4, "RFCTL10"),
	PINCTRL_PIN(0x94 / 4, "RFCTL11"),
	PINCTRL_PIN(0x98 / 4, "RFCTL12"),
	PINCTRL_PIN(0x9c / 4, "RFCTL13"),
	PINCTRL_PIN(0xa0 / 4, "RFCTL14"),
	PINCTRL_PIN(0xa4 / 4, "RFCTL27"),
	PINCTRL_PIN(0xa8 / 4, "XTL_EN"),
	PINCTRL_PIN(0xac / 4, "RFFE_SCK0"),
	PINCTRL_PIN(0xb0 / 4, "RFFE_SDA0"),
	PINCTRL_PIN(0xb4 / 4, "RFCTL28"),
	PINCTRL_PIN(0xb8 / 4, "RFCTL29"),
	PINCTRL_PIN(0xbc / 4, "SIMCLK0"),
	PINCTRL_PIN(0xc0 / 4, "SIMDA0"),
	PINCTRL_PIN(0xc4 / 4, "SIMRST0"),
	PINCTRL_PIN(0xc8 / 4, "SIMCLK1"),
	PINCTRL_PIN(0xcc / 4, "SIMDA1"),
	PINCTRL_PIN(0xd0 / 4, "SIMRST1"),
	PINCTRL_PIN(0xd4 / 4, "SIMCLK2"),
	PINCTRL_PIN(0xd8 / 4, "SIMDA2"),
	PINCTRL_PIN(0xdc / 4, "SIMRST2"),
	PINCTRL_PIN(0xe0 / 4, "SD0_D3"),
	PINCTRL_PIN(0xe4 / 4, "SD0_D2"),
	PINCTRL_PIN(0xe8 / 4, "SD0_CMD"),
	PINCTRL_PIN(0xec / 4, "SD0_D0"),
	PINCTRL_PIN(0xf0 / 4, "SD0_D1"),
	PINCTRL_PIN(0xf4 / 4, "SD0_CLK0"),
	PINCTRL_PIN(0xf8 / 4, "SD1_CLK"),
	PINCTRL_PIN(0xfc / 4, "SD1_CMD"),
	PINCTRL_PIN(0x100 / 4, "SD1_D0"),
	PINCTRL_PIN(0x104 / 4, "SD1_D1"),
	PINCTRL_PIN(0x108 / 4, "SD1_D2"),
	PINCTRL_PIN(0x10c / 4, "SD1_D3"),
	PINCTRL_PIN(0x110 / 4, "IIS0DI"),
	PINCTRL_PIN(0x114 / 4, "IIS0DO"),
	PINCTRL_PIN(0x118 / 4, "IIS0CLK"),
	PINCTRL_PIN(0x11c / 4, "IIS0LRCK"),
	PINCTRL_PIN(0x120 / 4, "U0TXD"),
	PINCTRL_PIN(0x124 / 4, "U0RXD"),
	PINCTRL_PIN(0x128 / 4, "U0CTS"),
	PINCTRL_PIN(0x12c / 4, "U0RTS"),
	PINCTRL_PIN(0x130 / 4, "PTEST"),
	PINCTRL_PIN(0x134 / 4, "ANA_INT"),
	PINCTRL_PIN(0x138 / 4, "EXT_RST_B"),
	PINCTRL_PIN(0x13c / 4, "CHIP_SLEEP"),
	PINCTRL_PIN(0x140 / 4, "XTL_BUF_EN0"),
	PINCTRL_PIN(0x144 / 4, "XTL_BUF_EN1"),
	PINCTRL_PIN(0x148 / 4, "CLK_32K"),
	PINCTRL_PIN(0x14c / 4, "AUD_SCLK"),
	PINCTRL_PIN(0x150 / 4, "AUD_ADD0"),
	PINCTRL_PIN(0x154 / 4, "AUD_ADSYNC"),
	PINCTRL_PIN(0x158 / 4, "AUD_DAD1"),
	PINCTRL_PIN(0x15c / 4, "AUD_DAD0"),
	PINCTRL_PIN(0x160 / 4, "AUD_DASYNC"),
	PINCTRL_PIN(0x164 / 4, "ADI_D"),
	PINCTRL_PIN(0x168 / 4, "ADI_SYNC"),
	PINCTRL_PIN(0x16c / 4, "ADI_SCLK"),
	PINCTRL_PIN(0x170 / 4, "LCM_RSTN"),
	PINCTRL_PIN(0x174 / 4, "DSI_TE"),
	PINCTRL_PIN(0x178 / 4, "MTDO_ARM"),
	PINCTRL_PIN(0x17c / 4, "MTDI_ARM"),
	PINCTRL_PIN(0x180 / 4, "MTCK_ARM"),
	PINCTRL_PIN(0x184 / 4, "MTMS_ARM"),
	PINCTRL_PIN(0x188 / 4, "MTRST_N_ARM"),
	PINCTRL_PIN(0x18c / 4, "DTDO_LTE"),
	PINCTRL_PIN(0x190 / 4, "DTDI_LTE"),
	PINCTRL_PIN(0x194 / 4, "DTCK_LTE"),
	PINCTRL_PIN(0x198 / 4, "DTMS_LTE"),
	PINCTRL_PIN(0x19c / 4, "DRTCK_LTE"),
	PINCTRL_PIN(0x1a0 / 4, "NFWPN"),
	PINCTRL_PIN(0x1a4 / 4, "NFRB"),
	PINCTRL_PIN(0x1a8 / 4, "NFCLE"),
	PINCTRL_PIN(0x1ac / 4, "NFALE"),
	PINCTRL_PIN(0x1b0 / 4, "NFREN"),
	PINCTRL_PIN(0x1b4 / 4, "NFD4"),
	PINCTRL_PIN(0x1b8 / 4, "NFD5"),
	PINCTRL_PIN(0x1bc / 4, "NFD6"),
	PINCTRL_PIN(0x1c0 / 4, "NFD7"),
	PINCTRL_PIN(0x1c4 / 4, "NFD10"),
	PINCTRL_PIN(0x1c8 / 4, "NFD11"),
	PINCTRL_PIN(0x1cc / 4, "NFD14"),
	PINCTRL_PIN(0x1d0 / 4, "NFCEN0"),
	PINCTRL_PIN(0x1d4 / 4, "NFWEN"),
	PINCTRL_PIN(0x1d8 / 4, "NFD0"),
	PINCTRL_PIN(0x1dc / 4, "NFD1"),
	PINCTRL_PIN(0x1e0 / 4, "NFD2"),
	PINCTRL_PIN(0x1e4 / 4, "NFD3"),
	PINCTRL_PIN(0x1e8 / 4, "NFD8"),
	PINCTRL_PIN(0x1ec / 4, "NFD9"),
	PINCTRL_PIN(0x1f0 / 4, "NFD12"),
	PINCTRL_PIN(0x1f4 / 4, "NFD13"),
	PINCTRL_PIN(0x1f8 / 4, "NFD15"),
	PINCTRL_PIN(0x1fc / 4, "CCIRD0"),
	PINCTRL_PIN(0x200 / 4, "CCIRD1"),
	PINCTRL_PIN(0x204 / 4, "CMMCLK"),
	PINCTRL_PIN(0x208 / 4, "CMPCLK"),
	PINCTRL_PIN(0x20c / 4, "CMRST0"),
	PINCTRL_PIN(0x210 / 4, "CMRST1"),
	PINCTRL_PIN(0x214 / 4, "CMPD0"),
	PINCTRL_PIN(0x218 / 4, "CMPD1"),
	PINCTRL_PIN(0x21c / 4, "SCL0"),
	PINCTRL_PIN(0x220 / 4, "SDA0"),
	PINCTRL_PIN(0x224 / 4, "SPI2_CSN"),
	PINCTRL_PIN(0x228 / 4, "SPI2_DO"),
	PINCTRL_PIN(0x22c / 4, "SPI2_DI"),
	PINCTRL_PIN(0x230 / 4, "SPI2_CLK"),
	PINCTRL_PIN(0x234 / 4, "SPI0_CSN"),
	PINCTRL_PIN(0x238 / 4, "SPI0_DO"),
	PINCTRL_PIN(0x23c / 4, "SPI0_DI"),
	PINCTRL_PIN(0x240 / 4, "SPI0_CLK"),
	PINCTRL_PIN(0x244 / 4, "MEMS_MIC_CLK0"),
	PINCTRL_PIN(0x248 / 4, "MEMS_MIC_DATA0"),
	PINCTRL_PIN(0x24c / 4, "MEMS_MIC_CLK1"),
	PINCTRL_PIN(0x250 / 4, "MEMS_MIC_DATA1"),
	PINCTRL_PIN(0x254 / 4, "KEYOUT0"),
	PINCTRL_PIN(0x258 / 4, "KEYOUT1"),
	PINCTRL_PIN(0x25c / 4, "KEYOUT2"),
	PINCTRL_PIN(0x260 / 4, "KEYIN0"),
	PINCTRL_PIN(0x264 / 4, "KEYIN1"),
	PINCTRL_PIN(0x268 / 4, "KEYIN2"),
	PINCTRL_PIN(0x26c / 4, "SCL2"),
	PINCTRL_PIN(0x270 / 4, "SDA2"),
	PINCTRL_PIN(0x274 / 4, "CLK_AUX0"),
	PINCTRL_PIN(0x278 / 4, "IIS1DI"),
	PINCTRL_PIN(0x27c / 4, "IIS1DO"),
	PINCTRL_PIN(0x280 / 4, "IIS1CLK"),
	PINCTRL_PIN(0x284 / 4, "IIS1LRCK"),
	PINCTRL_PIN(0x288 / 4, "TRACECLK"),
	PINCTRL_PIN(0x28c / 4, "TRACECTRL"),
	PINCTRL_PIN(0x290 / 4, "TRACEDAT0"),
	PINCTRL_PIN(0x294 / 4, "TRACEDAT1"),
	PINCTRL_PIN(0x298 / 4, "TRACEDAT2"),
	PINCTRL_PIN(0x29c / 4, "TRACEDAT3"),
	PINCTRL_PIN(0x2a0 / 4, "TRACEDAT4"),
	PINCTRL_PIN(0x2a4 / 4, "TRACEDAT5"),
	PINCTRL_PIN(0x2a8 / 4, "TRACEDAT6"),
	PINCTRL_PIN(0x2ac / 4, "TRACEDAT7"),
	PINCTRL_PIN(0x2b0 / 4, "EXTINT0"),
	PINCTRL_PIN(0x2b4 / 4, "EXTINT1"),
	PINCTRL_PIN(0x2b8 / 4, "SCL3"),
	PINCTRL_PIN(0x2bc / 4, "SDA3"),
	PINCTRL_PIN(0x2c0 / 4, "U1TXD"),
	PINCTRL_PIN(0x2c4 / 4, "U1RXD"),
	PINCTRL_PIN(0x2c8 / 4, "U2TXD"),
	PINCTRL_PIN(0x2cc / 4, "U2RXD"),
	PINCTRL_PIN(0x2d0 / 4, "U3TXD"),
	PINCTRL_PIN(0x2d4 / 4, "U3RXD"),
	PINCTRL_PIN(0x2d8 / 4, "U3CTS"),
	PINCTRL_PIN(0x2dc / 4, "U3RTS"),
	PINCTRL_PIN(0x2e0 / 4, "U4TXD"),
	PINCTRL_PIN(0x2e4 / 4, "U4RXD"),
};

static const unsigned int sc9830_i2c2_pins[] = { 0x26c / 4, 0x270 / 4 };
static const char * const sc9830_groups[] = { "i2c2" };
static const char * const sc9830_functions[] = { "i2c2", "gpio" };

static int sc9830_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(sc9830_groups);
}

static const char *sc9830_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	return sc9830_groups[selector];
}

static int sc9830_get_group_pins(struct pinctrl_dev *pctldev,
				 unsigned int selector, const unsigned int **pins,
				unsigned int *num_pins)
{
	if (selector >= ARRAY_SIZE(sc9830_groups))
		return -EINVAL;

	*pins = sc9830_i2c2_pins;
	*num_pins = ARRAY_SIZE(sc9830_i2c2_pins);
	return 0;
}

static const struct pinctrl_ops sc9830_pinctrl_ops = {
	.get_groups_count = sc9830_get_groups_count,
	.get_group_name = sc9830_get_group_name,
	.get_group_pins = sc9830_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_group,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int sc9830_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(sc9830_functions);
}

static const char *sc9830_get_function_name(struct pinctrl_dev *pctldev,
					    unsigned int selector)
{
	return sc9830_functions[selector];
}

static int sc9830_get_function_groups(struct pinctrl_dev *pctldev,
				      unsigned int selector,
				      const char * const **groups,
				      unsigned int *num_groups)
{
	if (selector >= ARRAY_SIZE(sc9830_functions))
		return -EINVAL;

	*groups = sc9830_groups;
	*num_groups = ARRAY_SIZE(sc9830_groups);
	return 0;
}

static int sc9830_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
			  unsigned int group)
{
	struct sc9830_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long flags;
	unsigned int i;
	u32 value, af;

	if (function >= 2 || group)
		return -EINVAL;

	af = function ? SC9830_AF_MASK : 0;
	spin_lock_irqsave(&pctl->lock, flags);
	for (i = 0; i < ARRAY_SIZE(sc9830_i2c2_pins); i++) {
		value = readl(pctl->base + sc9830_i2c2_pins[i] * 4);
		value = (value & ~SC9830_AF_MASK) | af;
		writel(value, pctl->base + sc9830_i2c2_pins[i] * 4);
	}
	spin_unlock_irqrestore(&pctl->lock, flags);
	return 0;
}

static const struct pinmux_ops sc9830_pinmux_ops = {
	.get_functions_count = sc9830_get_functions_count,
	.get_function_name = sc9830_get_function_name,
	.get_function_groups = sc9830_get_function_groups,
	.set_mux = sc9830_set_mux,
	.strict = true,
};

static const struct pinctrl_desc sc9830_pinctrl_desc = {
	.name = "sc9830-pinctrl",
	.pins = sc9830_pins,
	.npins = ARRAY_SIZE(sc9830_pins),
	.pctlops = &sc9830_pinctrl_ops,
	.pmxops = &sc9830_pinmux_ops,
	.owner = THIS_MODULE,
};

static int sc9830_pinctrl_probe(struct platform_device *pdev)
{
	struct pinctrl_dev *pctldev;
	struct sc9830_pinctrl *pctl;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctl->base))
		return PTR_ERR(pctl->base);

	spin_lock_init(&pctl->lock);
	pctldev = devm_pinctrl_register(&pdev->dev, &sc9830_pinctrl_desc, pctl);
	return PTR_ERR_OR_ZERO(pctldev);
}

static const struct of_device_id sc9830_pinctrl_match[] = {
	{ .compatible = "sprd,sc9830-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_pinctrl_match);

static struct platform_driver sc9830_pinctrl_driver = {
	.probe = sc9830_pinctrl_probe,
	.driver = {
		.name = "sc9830-pinctrl",
		.of_match_table = sc9830_pinctrl_match,
	},
};
module_platform_driver(sc9830_pinctrl_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 pin multiplexer");
MODULE_LICENSE("GPL");
