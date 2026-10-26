// SPDX-License-Identifier: GPL-2.0-only
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define BCM2153_GPIO_MODE 0x00
#define BCM2153_GPIO_DATA 0x10
#define BCM2153_GPIO_INPUT 0x18
#define BCM2153_GPIO_MASK0 0x30
#define BCM2153_GPIO_MASK1 0x34
#define BCM2153_GPIO_ACK 0x38
#define BCM2153_GPIO_EDGE 0x40
#define BCM2153_GPIO_PENDING 0x50

struct bcm2153_gpio {
	struct gpio_chip chip;
	void __iomem *base;
	raw_spinlock_t lock;
	unsigned int parent_irq;
};

static void bcm2153_gpio_set_mode(struct bcm2153_gpio *gpio, unsigned int pin,
				  u32 mode, u32 trigger)
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
	if (mode == 1)
		edges[half] = (edges[half] & ~(3U << shift)) | (trigger << shift);
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
	bcm2153_gpio_set_mode(gpio, pin, 2, 0);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
	return 0;
}

static int bcm2153_gpio_direction_input(struct gpio_chip *chip, unsigned int pin)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	bcm2153_gpio_set_mode(gpio, pin, 0, 0);
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

static void bcm2153_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	void __iomem *reg = gpio->base + BCM2153_GPIO_ACK + d->hwirq / 32 * 4;
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel(readl(reg) | BIT(d->hwirq % 32), reg);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static void bcm2153_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	void __iomem *reg = gpio->base + BCM2153_GPIO_MASK0 + d->hwirq / 32 * 4;
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel(readl(reg) & ~BIT(d->hwirq % 32), reg);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
	gpiochip_disable_irq(chip, d->hwirq);
}

static void bcm2153_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	void __iomem *reg = gpio->base + BCM2153_GPIO_MASK0 + d->hwirq / 32 * 4;
	unsigned long flags;

	gpiochip_enable_irq(chip, d->hwirq);
	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel(readl(reg) | BIT(d->hwirq % 32), reg);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static int bcm2153_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;
	u32 trigger;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		trigger = 1;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trigger = 2;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		trigger = 3;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&gpio->lock, flags);
	bcm2153_gpio_set_mode(gpio, d->hwirq, 1, trigger);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
	irq_set_handler_locked(d, handle_edge_irq);
	return 0;
}

static const struct irq_chip bcm2153_gpio_irq_chip = {
	.name = "bcm2153-gpio",
	.irq_ack = bcm2153_gpio_irq_ack,
	.irq_mask = bcm2153_gpio_irq_mask,
	.irq_unmask = bcm2153_gpio_irq_unmask,
	.irq_set_type = bcm2153_gpio_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_SET_TYPE_MASKED,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void bcm2153_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);
	struct irq_chip *parent = irq_desc_get_chip(desc);
	unsigned long pending;
	unsigned int bank, bit;
	bool active;

	chained_irq_enter(parent, desc);
	do {
		active = false;
		for (bank = 0; bank < 2; bank++) {
			pending = readl(gpio->base + BCM2153_GPIO_PENDING + bank * 4);
			pending &= readl(gpio->base + BCM2153_GPIO_MASK0 + bank * 4);
			active |= !!pending;
			for_each_set_bit(bit, &pending, 32)
				generic_handle_domain_irq(chip->irq.domain, bank * 32 + bit);
		}
	} while (active);
	chained_irq_exit(parent, desc);
}

static int bcm2153_gpio_irq_init_hw(struct gpio_chip *chip)
{
	struct bcm2153_gpio *gpio = gpiochip_get_data(chip);

	writel(0, gpio->base + BCM2153_GPIO_MASK0);
	writel(0, gpio->base + BCM2153_GPIO_MASK1);
	return 0;
}

static int bcm2153_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2153_gpio *gpio;
	struct gpio_irq_chip *girq;
	int irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

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
	gpio->parent_irq = irq;
	girq = &gpio->chip.irq;
	gpio_irq_chip_set_chip(girq, &bcm2153_gpio_irq_chip);
	girq->parent_handler = bcm2153_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = &gpio->parent_irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->init_hw = bcm2153_gpio_irq_init_hw;
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
