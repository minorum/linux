// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

struct sc2723_regulator {
	struct regulator_desc desc;
	unsigned int selector;
};

struct sc2723_cpu_regulator {
	struct regulator_desc desc;
	bool linear;
	bool fault;
};

static int sc2723_cpu_voltage(bool linear, unsigned int selector)
{
	static const int bases[] = {
		1100000, 700000, 800000, 900000,
		1000000, 650000, 1200000, 1300000,
	};

	if (selector >= (linear ? 1024 : 256))
		return -EINVAL;
	if (linear)
		return 600000 + selector * 3125;
	return bases[selector >> 5] + (selector & 31) * 3125;
}

static int sc2723_cpu_select(bool linear, int minimum, int maximum)
{
	int selector, voltage, best = -EINVAL, best_voltage = 0;

	for (selector = 0; selector < (linear ? 1024 : 256); selector++) {
		voltage = sc2723_cpu_voltage(linear, selector);
		if (voltage >= minimum && voltage <= maximum &&
		    (best < 0 || voltage < best_voltage)) {
			best = selector;
			best_voltage = voltage;
		}
	}
	return best;
}

static int sc2723_cpu_get_selector(struct regulator_dev *rdev)
{
	struct sc2723_cpu_regulator *cpu = rdev_get_drvdata(rdev);
	unsigned int mode, power, selector;
	int ret;

	if (cpu->fault)
		return -EIO;
	ret = regmap_read(rdev->regmap, 0x918, &mode);
	if (ret)
		return ret;
	if (!!(mode & BIT(14)) != cpu->linear)
		return -EIO;
	ret = regmap_read(rdev->regmap, 0x810, &power);
	if (ret)
		return ret;
	if (power & BIT(10))
		return -EIO;
	ret = regmap_read(rdev->regmap, 0xa04, &selector);
	if (ret)
		return ret;
	return selector & rdev->desc->vsel_mask;
}

static int sc2723_cpu_set_selector(struct regulator_dev *rdev, unsigned int selector)
{
	struct sc2723_cpu_regulator *cpu = rdev_get_drvdata(rdev);
	int voltage_now, target, voltage, next, ret;
	unsigned int readback;

	if (cpu->fault)
		return -EIO;
	target = sc2723_cpu_voltage(cpu->linear, selector);
	if (target < 0)
		return target;
	ret = sc2723_cpu_get_selector(rdev);
	if (ret < 0)
		goto fault;
	voltage_now = sc2723_cpu_voltage(cpu->linear, ret);
	while (voltage_now != target) {
		voltage = voltage_now < target ? min(target, voltage_now + 25000) :
			max(target, voltage_now - 25000);
		next = sc2723_cpu_select(cpu->linear, voltage,
					 voltage_now < target ? voltage : voltage_now - 1);
		if (next < 0) {
			ret = next;
			goto fault;
		}
		voltage = sc2723_cpu_voltage(cpu->linear, next);
		ret = regmap_update_bits(rdev->regmap, 0xa04,
					 rdev->desc->vsel_mask, next);
		if (ret)
			goto fault;
		if (voltage > voltage_now)
			udelay(DIV_ROUND_UP(voltage - voltage_now, 5000));
		ret = regmap_read(rdev->regmap, 0xa04, &readback);
		if (ret)
			goto fault;
		if ((readback & rdev->desc->vsel_mask) != next) {
			ret = -EIO;
			goto fault;
		}
		voltage_now = voltage;
	}
	return 0;

fault:
	cpu->fault = true;
	return ret;
}

static int sc2723_cpu_list_voltage(struct regulator_dev *rdev, unsigned int selector)
{
	struct sc2723_cpu_regulator *cpu = rdev_get_drvdata(rdev);

	return sc2723_cpu_voltage(cpu->linear, selector);
}

static int sc2723_cpu_init(struct regulator_dev *rdev, struct regulator_config *config)
{
	struct sc2723_cpu_regulator *cpu = rdev_get_drvdata(rdev);
	struct regulation_constraints *constraints = rdev->constraints;
	int selector, voltage;

	if (!constraints->always_on || !constraints->boot_on || constraints->uV_offset ||
	    constraints->min_uV <= 0 || constraints->max_uV < constraints->min_uV)
		return -EINVAL;
	selector = sc2723_cpu_get_selector(rdev);
	if (selector < 0)
		return selector;
	voltage = sc2723_cpu_voltage(cpu->linear, selector);
	if (voltage < constraints->min_uV || voltage > constraints->max_uV)
		return -EINVAL;
	constraints->apply_uV = false;
	return 0;
}

static int sc2723_cpu_enable(struct regulator_dev *rdev)
{
	int ret = regulator_is_enabled_regmap(rdev);

	return ret > 0 ? 0 : ret ?: -EPERM;
}

static const struct regulator_ops sc2723_cpu_ops = {
	.enable = sc2723_cpu_enable,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = sc2723_cpu_list_voltage,
	.map_voltage = regulator_map_voltage_iterate,
	.get_voltage_sel = sc2723_cpu_get_selector,
	.set_voltage_sel = sc2723_cpu_set_selector,
};

static int sc2723_cpu_register(struct device *dev, struct regulator_config *config)
{
	struct sc2723_cpu_regulator *cpu;
	struct regulator_dev *rdev;
	struct device_node *node;
	unsigned int mode, power;
	int ret;

	node = of_get_child_by_name(dev->of_node, "vddarm");
	if (!node)
		return 0;
	ret = of_device_is_available(node);
	of_node_put(node);
	if (!ret)
		return 0;
	ret = regmap_read(config->regmap, 0x918, &mode);
	if (ret)
		return ret;
	ret = regmap_read(config->regmap, 0x810, &power);
	if (ret)
		return ret;
	if (power & BIT(10))
		return -EPERM;
	cpu = devm_kzalloc(dev, sizeof(*cpu), GFP_KERNEL);
	if (!cpu)
		return -ENOMEM;
	cpu->linear = !!(mode & BIT(14));
	cpu->desc = (struct regulator_desc) {
		.name = "vddarm",
		.of_match = "vddarm",
		.init_cb = sc2723_cpu_init,
		.id = 2,
		.ops = &sc2723_cpu_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = cpu->linear ? 1024 : 256,
		.enable_reg = 0x810,
		.enable_mask = BIT(10),
		.enable_is_inverted = true,
		.vsel_reg = 0xa04,
		.vsel_mask = cpu->linear ? GENMASK(9, 0) : GENMASK(7, 0),
	};
	config->driver_data = cpu;
	rdev = devm_regulator_register(dev, &cpu->desc, config);
	return PTR_ERR_OR_ZERO(rdev);
}

static int sc2723_rf_selector(unsigned int code, unsigned int bits,
			      int target, int minimum, int step)
{
	int delta;

	if (code >= BIT(bits))
		return -EINVAL;
	delta = (int)code - (int)BIT(bits - 1);
	return DIV_ROUND_UP(target - minimum + delta * step, step);
}

static int sc2723_rf_is_enabled(struct regulator_dev *rdev)
{
	struct sc2723_regulator *regulator = rdev_get_drvdata(rdev);
	unsigned int selector;
	int ret;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret <= 0)
		return ret;
	ret = regmap_read(rdev->regmap, rdev->desc->vsel_reg, &selector);
	if (ret)
		return ret;
	if ((selector & rdev->desc->vsel_mask) != regulator->selector)
		return -EINVAL;
	return 1;
}

static int sc2723_rf_enable(struct regulator_dev *rdev)
{
	struct sc2723_regulator *regulator = rdev_get_drvdata(rdev);
	int ret, cleanup;

	ret = sc2723_rf_is_enabled(rdev);
	if (ret)
		return ret < 0 ? ret : 0;
	ret = regmap_update_bits(rdev->regmap, rdev->desc->vsel_reg,
				 rdev->desc->vsel_mask, regulator->selector);
	if (ret)
		return ret;
	ret = regulator_enable_regmap(rdev);
	if (ret) {
		cleanup = regulator_disable_regmap(rdev);
		if (cleanup)
			dev_err(&rdev->dev, "Supply disable failed: %d\n", cleanup);
	}
	return ret;
}

static const struct regulator_ops sc2723_emmc_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc sc2723_emmc_descs[] = {
	{
		.name = "vddgen0",
		.of_match = "vddgen0",
		.id = 3,
		.ops = &sc2723_emmc_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 1800000,
		.enable_reg = 0x810,
		.enable_mask = BIT(4),
		.enable_is_inverted = true,
		.vsel_reg = 0x820,
		.vsel_mask = GENMASK(6, 0),
	},
	{
		.name = "vddemmccore",
		.of_match = "vddemmccore",
		.id = 4,
		.ops = &sc2723_emmc_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 3000000,
		.enable_reg = 0x810,
		.enable_mask = BIT(7),
		.enable_is_inverted = true,
		.vsel_reg = 0x834,
		.vsel_mask = GENMASK(15, 8),
	},
};

static int sc2723_emmc_register(struct device *dev, struct regulator_config *config)
{
	static const unsigned int selectors[] = { 104, 180 << 8 };
	static const unsigned int software_select[] = { BIT(13), BIT(7) };
	struct regulator_dev *rdev;
	struct device_node *node;
	unsigned int power, source, selector;
	int i, ret, count = 0;

	for (i = 0; i < ARRAY_SIZE(sc2723_emmc_descs); i++) {
		const struct regulator_desc *desc = &sc2723_emmc_descs[i];

		node = of_get_child_by_name(dev->of_node, desc->of_match);
		ret = of_device_is_available(node);
		of_node_put(node);
		if (!ret)
			continue;
		ret = regmap_read(config->regmap, 0x810, &power);
		if (ret)
			return ret;
		ret = regmap_read(config->regmap, 0x8c0, &source);
		if (ret)
			return ret;
		ret = regmap_read(config->regmap, desc->vsel_reg, &selector);
		if (ret)
			return ret;
		if ((power & desc->enable_mask) || (source & software_select[i]) ||
		    (selector & desc->vsel_mask) != selectors[i])
			return dev_err_probe(dev, -EINVAL, "eMMC boot calibration differs\n");
		rdev = devm_regulator_register(dev, desc, config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "eMMC registration failed\n");
		count++;
	}
	return count;
}

static const struct regulator_ops sc2723_rf_ops = {
	.enable = sc2723_rf_enable,
	.disable = regulator_disable_regmap,
	.is_enabled = sc2723_rf_is_enabled,
};

static const struct regulator_desc sc2723_rf_descs[] = {
	{
		.name = "vddcon",
		.of_match = "vddcon",
		.ops = &sc2723_rf_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 1600000,
		.enable_reg = 0x814,
		.enable_mask = BIT(13),
		.enable_is_inverted = true,
		.vsel_reg = 0xa18,
		.vsel_mask = GENMASK(9, 0),
	},
	{
		.name = "vddwifipa",
		.of_match = "vddwifipa",
		.id = 1,
		.ops = &sc2723_rf_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 3300000,
		.enable_reg = 0x814,
		.enable_mask = BIT(11),
		.enable_is_inverted = true,
		.vsel_reg = 0x818,
		.vsel_mask = GENMASK(7, 0),
	},
	{
		.name = "vddsim2",
		.of_match = "vddsim2",
		.id = 2,
		.ops = &sc2723_rf_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 1,
		.fixed_uV = 3000000,
		.enable_reg = 0x814,
		.enable_mask = BIT(3),
		.enable_is_inverted = true,
		.vsel_reg = 0x82c,
		.vsel_mask = GENMASK(15, 8),
	},
};

static const struct regulator_desc sc2723_kpled_desc = {
	.name = "vddkpled",
	.of_match = "vddkpled",
	.id = 6,
	.ops = &sc2723_rf_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = 1,
	.fixed_uV = 3300000,
	.enable_reg = 0x8f4,
	.enable_mask = BIT(8),
	.enable_is_inverted = true,
	.vsel_reg = 0x8f4,
	.vsel_mask = GENMASK(7, 0),
};

static int sc2723_kpled_register(struct device *dev, struct regulator_config *config)
{
	static const char * const names[] = { "calibration_state", "vddkpled" };
	struct device_node *node;
	struct sc2723_regulator *regulator;
	struct regulator_dev *rdev;
	struct nvmem_cell *cell;
	unsigned int power;
	size_t length;
	u8 trim[2], *data;
	int i, ret = 0;

	node = of_get_child_by_name(dev->of_node, "vddkpled");
	if (!of_device_is_available(node))
		goto put_node;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		cell = of_nvmem_cell_get(node, names[i]);
		if (IS_ERR(cell)) {
			ret = PTR_ERR(cell);
			goto put_node;
		}
		data = nvmem_cell_read(cell, &length);
		nvmem_cell_put(cell);
		if (IS_ERR(data)) {
			ret = PTR_ERR(data);
			goto put_node;
		}
		if (length != 1 || data[0] >= (i ? 64 : 1))
			ret = -EINVAL;
		else
			trim[i] = data[0];
		kfree(data);
		if (ret)
			goto put_node;
	}
	regulator = devm_kzalloc(dev, sizeof(*regulator), GFP_KERNEL);
	if (!regulator) {
		ret = -ENOMEM;
		goto put_node;
	}
	regulator->desc = sc2723_kpled_desc;
	regulator->selector = sc2723_rf_selector(trim[1], 6, 3300000, 1200000, 10000);
	ret = regmap_read(config->regmap, 0x8f4, &power);
	if (ret)
		goto put_node;
	if (!(power & BIT(8)) && (power & 0xff) != regulator->selector) {
		ret = -EINVAL;
		goto put_node;
	}
	config->driver_data = regulator;
	rdev = devm_regulator_register(dev, &regulator->desc, config);
	config->driver_data = NULL;
	ret = IS_ERR(rdev) ? PTR_ERR(rdev) : 1;
put_node:
	of_node_put(node);
	return ret;
}

static int sc2723_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { .dev = dev };
	struct sc2723_regulator *regulators;
	struct regulator_dev *rdev;
	struct device_node *sim2;
	u32 code, power, selector;
	u8 state;
	int i, ret, registered, count = ARRAY_SIZE(sc2723_rf_descs) - 1;

	config.regmap = dev_get_regmap(dev->parent, NULL);
	if (!config.regmap)
		return -ENODEV;
	registered = sc2723_kpled_register(dev, &config);
	if (registered < 0)
		return dev_err_probe(dev, registered, "Panel supply registration failed\n");
	ret = sc2723_emmc_register(dev, &config);
	if (ret < 0)
		return ret;
	ret += registered;
	if (ret && !of_property_present(dev->of_node, "nvmem-cells")) {
		if (of_get_available_child_count(dev->of_node) != ret)
			return -EINVAL;
		return 0;
	}
	ret = nvmem_cell_read_u8(dev, "calibration_state", &state);
	if (ret)
		return dev_err_probe(dev, ret, "Calibration state unavailable\n");
	if (state)
		return dev_err_probe(dev, -EINVAL, "Factory calibration invalid\n");
	sim2 = of_get_child_by_name(dev->of_node, "vddsim2");
	if (of_device_is_available(sim2))
		count++;
	of_node_put(sim2);
	regulators = devm_kcalloc(dev, count, sizeof(*regulators), GFP_KERNEL);
	if (!regulators)
		return -ENOMEM;
	for (i = 0; i < count; i++) {
		regulators[i].desc = sc2723_rf_descs[i];
		ret = nvmem_cell_read_variable_le_u32(dev, sc2723_rf_descs[i].name,
						      &code);
		if (ret)
			return dev_err_probe(dev, ret, "RF calibration unavailable\n");
		if (i == 2) {
			if (code >= 64)
				return -EINVAL;
			ret = (180 + ((int)code - 32) * 3000 / 1800) << 8;
		} else {
			ret = sc2723_rf_selector(code, i ? 6 : 7,
						 sc2723_rf_descs[i].fixed_uV,
						 i ? 1200000 : 600000, i ? 10000 : 3125);
		}
		if (ret < 0 || ret > sc2723_rf_descs[i].vsel_mask)
			return dev_err_probe(dev, -EINVAL, "RF calibration out of range\n");
		regulators[i].selector = ret;
	}
	ret = regmap_read(config.regmap, 0x814, &power);
	if (ret)
		return ret;
	for (i = 0; i < count; i++) {
		if (power & sc2723_rf_descs[i].enable_mask)
			continue;
		ret = regmap_read(config.regmap, sc2723_rf_descs[i].vsel_reg, &selector);
		if (ret)
			return ret;
		if ((selector & sc2723_rf_descs[i].vsel_mask) != regulators[i].selector)
			return dev_err_probe(dev, -EINVAL, "RF voltage already differs\n");
	}
	for (i = 0; i < count; i++) {
		config.driver_data = &regulators[i];
		rdev = devm_regulator_register(dev, &regulators[i].desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "RF registration failed\n");
	}
	return sc2723_cpu_register(dev, &config);
}

static const struct of_device_id sc2723_regulator_match[] = {
	{ .compatible = "sprd,sc2723-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2723_regulator_match);

static struct platform_driver sc2723_regulator_driver = {
	.driver = {
		.name = "sc2723-regulator",
		.of_match_table = sc2723_regulator_match,
	},
	.probe = sc2723_regulator_probe,
};
module_platform_driver(sc2723_regulator_driver);

MODULE_DESCRIPTION("SC2723 CPU and calibrated peripheral regulators");
MODULE_LICENSE("GPL");
