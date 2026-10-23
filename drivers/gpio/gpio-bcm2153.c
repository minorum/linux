// SPDX-License-Identifier: GPL-2.0-only
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define BCM2153_GPIO_MODE 0x00
#define BCM2153_GPIO_DATA 0x10
#define BCM2153_GPIO_INPUT 0x18
#define BCM2153_GPIO_MASK0 0x30
#define BCM2153_GPIO_MASK1 0x34
#define BCM2153_GPIO_EDGE 0x40

struct bcm2153_gpio {
	struct gpio_chip chip;
	void __iomem *base;
	raw_spinlock_t lock;
};

static void bcm2153_gpio_set_mode(struct bcm2153_gpio *gpio, unsigned int pin, u32 mode)
{
	u32 modes[2], edges[2], masks[2];
	unsigned int bank = pin / 32;
	unsigned int half, offset;
	unsigned int shift = pin % 16 * 2;

	for (half = 0; half < 2; half++) {
		offset = bank * 8 + half * 4;
		modes[half] = readl(gpio->base + BCM2153_GPIO_MODE + offset);
		edges[half] = readl(gpio->base + BCM2153_GPIO_EDGE + offset);
	}
	half = pin % 32 / 16;
	modes[half] = (modes[half] & ~(3U << shift)) | (mode << shift);
	for (half = 0; half < 2; half++) {
		offset = bank * 8 + half * 4;
		writel(modes[half], gpio->base + BCM2153_GPIO_MODE + offset);
		masks[0] = readl(gpio->base + BCM2153_GPIO_MASK0);
		masks[1] = readl(gpio->base + BCM2153_GPIO_MASK1);
		writel(0, gpio->base + BCM2153_GPIO_MASK0);
		writel(0, gpio->base + BCM2153_GPIO_MASK1);
		writel(edges[half], gpio->base + BCM2153_GPIO_EDGE + offset);
		writel(masks[0], gpio->base + BCM2153_GPIO_MASK0);
		writel(masks[1], gpio->base + BCM2153_GPIO_MASK1);
	}
}

static void bcm2153_gpio_write(struct bcm2153_gpio *gpio, unsigned int pin, int value)
{
	void __iomem *reg = gpio->base + BCM2153_GPIO_DATA + pin / 32 * 4;
	u32 data = readl(reg);

	writel(value ? data | BIT(pin % 32) : data & ~BIT(pin % 32), reg);
}

static int bcm2153_gpio_set(struct gpio_chip *chip, unsigned int pin, int value)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	bcm2153_gpio_write(gpio, pin, value);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
	return 0;
}

static int bcm2153_gpio_direction_output(struct gpio_chip *chip,
					unsigned int pin, int value)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	bcm2153_gpio_write(gpio, pin, value);
	bcm2153_gpio_set_mode(gpio, pin, 2);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
	return 0;
}

static int bcm2153_gpio_direction_input(struct gpio_chip *chip, unsigned int pin)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	bcm2153_gpio_set_mode(gpio, pin, 0);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
	return 0;
}

static int bcm2153_gpio_get_direction(struct gpio_chip *chip, unsigned int pin)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	u32 mode = readl(gpio->base + BCM2153_GPIO_MODE + pin / 16 * 4);

	switch ((mode >> (pin % 16 * 2)) & 3) {
	case 0:
	case 1:
		return GPIO_LINE_DIRECTION_IN;
	case 2:
		return GPIO_LINE_DIRECTION_OUT;
	default:
		return -EOPNOTSUPP;
	}
}

static int bcm2153_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	unsigned int offset;
	int ret = bcm2153_gpio_get_direction(chip, pin);

	if (ret < 0)
		return ret;
	offset = ret == GPIO_LINE_DIRECTION_IN ? BCM2153_GPIO_INPUT : BCM2153_GPIO_DATA;
	return !!(readl(gpio->base + offset + pin / 32 * 4) & BIT(pin % 32));
}

static int bcm2153_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2153_gpio *gpio;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;
	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);
	raw_spin_lock_init(&gpio->lock);
	gpio->chip.label = dev_name(dev);
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = 64;
	gpio->chip.get_direction = bcm2153_gpio_get_direction;
	gpio->chip.direction_input = bcm2153_gpio_direction_input;
	gpio->chip.direction_output = bcm2153_gpio_direction_output;
	gpio->chip.get = bcm2153_gpio_get;
	gpio->chip.set = bcm2153_gpio_set;
	return devm_gpiochip_add_data(dev, &gpio->chip, gpio);
}

static const struct of_device_id bcm2153_gpio_of_match[] = {
	{ .compatible = "brcm,bcm2153-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2153_gpio_of_match);

static struct platform_driver bcm2153_gpio_driver = {
	.probe = bcm2153_gpio_probe,
	.driver = {
		.name = "bcm2153-gpio",
		.of_match_table = bcm2153_gpio_of_match,
	},
};
module_platform_driver(bcm2153_gpio_driver);

MODULE_DESCRIPTION("Broadcom BCM2153 GPIO");
MODULE_LICENSE("GPL");
