// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

enum sc9830_clocks { CPU, MPLL, INTERMEDIATE, AXI, NUM_CLOCKS };

struct sc9830_cpufreq {
	struct device *cpu_dev;
	struct clk_bulk_data clocks[NUM_CLOCKS];
	struct regulator *arm;
	void __iomem *ap;
	cpumask_t cpus;
	u32 fixed_mux, fixed_div;
	int delta;
	bool fault;
};

static struct sc9830_cpufreq *sc9830_policy;
static struct platform_device *sc9830_device;

static int sc9830_raw_voltage(int delta, unsigned long nominal)
{
	int offset;

	if (delta < -96 || delta > 127 || nominal < 900000 || nominal > 1100000)
		return -EINVAL;
	offset = delta * (int)(nominal / 1000) / 900;
	return (int)nominal + offset * 3125;
}

static int sc9830_opp_voltage(struct sc9830_cpufreq *d, unsigned long rate)
{
	struct dev_pm_opp *opp;
	unsigned long nominal;
	int expected;

	switch (rate) {
	case 768000000:
	case 900000000:
		expected = 900000;
		break;
	case 1200000000:
	case 1350000000:
		expected = 1000000;
		break;
	case 1500000000:
		expected = 1100000;
		break;
	default:
		return -EINVAL;
	}
	opp = dev_pm_opp_find_freq_exact(d->cpu_dev, rate, true);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	nominal = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	if (nominal != expected)
		return -EINVAL;
	return sc9830_raw_voltage(d->delta, nominal);
}

static int sc9830_verify_clock(struct sc9830_cpufreq *d, unsigned long rate,
			       unsigned int parent, unsigned int divisor)
{
	u32 mux = readl(d->ap + 0x54), div = readl(d->ap + 0x38);
	unsigned long cpu_rate = clk_get_rate(d->clocks[CPU].clk);
	unsigned long axi_rate = clk_get_rate(d->clocks[AXI].clk);

	if (readl(d->ap + 0x54) != mux || readl(d->ap + 0x38) != div)
		return -EAGAIN;
	if ((mux & ~GENMASK(2, 0)) != d->fixed_mux ||
	    (div & ~GENMASK(10, 8)) != d->fixed_div ||
	    FIELD_GET(GENMASK(2, 0), mux) != parent ||
	    FIELD_GET(GENMASK(10, 8), div) + 1 != divisor ||
	    cpu_rate != rate || axi_rate != DIV_ROUND_UP(rate, divisor) ||
	    axi_rate > 675000000)
		return -EIO;
	return 0;
}

static int sc9830_set_clocks(struct sc9830_cpufreq *d, unsigned long rate)
{
	struct clk *cpu = d->clocks[CPU].clk, *mpll = d->clocks[MPLL].clk;
	unsigned int old_div = FIELD_GET(GENMASK(10, 8), readl(d->ap + 0x38)) + 1;
	unsigned int divisor = DIV_ROUND_UP(rate, 675000000);
	int ret;

	ret = clk_set_parent(cpu, d->clocks[INTERMEDIATE].clk);
	if (ret)
		return ret;
	ret = sc9830_verify_clock(d, 768000000, 2, old_div);
	if (ret)
		return ret;
	ret = clk_set_rate(d->clocks[AXI].clk, 768000000 / divisor);
	if (ret)
		return ret;
	ret = sc9830_verify_clock(d, 768000000, 2, divisor);
	if (ret)
		return ret;
	if (rate == 768000000)
		return 0;
	ret = clk_set_rate(mpll, rate);
	if (ret)
		return ret;
	if (clk_get_rate(mpll) != rate)
		return -EIO;
	ret = clk_set_parent(cpu, mpll);
	if (ret)
		return ret;
	return sc9830_verify_clock(d, rate, 6, divisor);
}

static int sc9830_set_voltage(struct sc9830_cpufreq *d, int voltage)
{
	int ret = regulator_set_voltage(d->arm, voltage, voltage);

	if (ret)
		return ret;
	return regulator_get_voltage(d->arm) == voltage ? 0 : -EIO;
}

static int sc9830_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct sc9830_cpufreq *d = policy->driver_data;
	unsigned long rate = (unsigned long)policy->freq_table[index].frequency * 1000;
	unsigned long old_rate;
	u32 mux, div;
	int voltage, old_voltage, required, ret;

	if (d->fault)
		return -EIO;
	voltage = sc9830_opp_voltage(d, rate);
	if (voltage < 0)
		return voltage;
	old_rate = clk_get_rate(d->clocks[CPU].clk);
	required = sc9830_opp_voltage(d, old_rate);
	old_voltage = regulator_get_voltage(d->arm);
	if (required < 0 || old_voltage < required || !clk_get_rate(d->clocks[MPLL].clk)) {
		ret = -EIO;
		goto fault;
	}
	mux = readl(d->ap + 0x54);
	div = readl(d->ap + 0x38);
	ret = sc9830_verify_clock(d, old_rate, FIELD_GET(GENMASK(2, 0), mux),
				   FIELD_GET(GENMASK(10, 8), div) + 1);
	if (ret)
		goto fault;
	if (voltage > old_voltage) {
		ret = sc9830_set_voltage(d, voltage);
		if (ret)
			goto fault;
	}
	ret = sc9830_set_clocks(d, rate);
	if (ret)
		goto fault;
	if (voltage < old_voltage) {
		ret = sc9830_set_voltage(d, voltage);
		if (ret)
			goto fault;
	}
	return 0;

fault:
	d->fault = true;
	dev_err(d->cpu_dev, "DVFS stopped after uncertain transition: %d\n", ret);
	return ret;
}

static int sc9830_cpufreq_init(struct cpufreq_policy *policy)
{
	struct sc9830_cpufreq *d = sc9830_policy;
	int ret;

	if (!d || !cpumask_test_cpu(policy->cpu, &d->cpus))
		return -ENODEV;
	ret = dev_pm_opp_init_cpufreq_table(d->cpu_dev, &policy->freq_table);
	if (ret)
		return ret;
	cpumask_copy(policy->cpus, &d->cpus);
	policy->driver_data = d;
	policy->clk = d->clocks[CPU].clk;
	policy->cpuinfo.transition_latency = 1000000;
	policy->suspend_freq = dev_pm_opp_get_suspend_opp_freq(d->cpu_dev) / 1000;
	return 0;
}

static void sc9830_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct sc9830_cpufreq *d = policy->driver_data;

	dev_pm_opp_free_cpufreq_table(d->cpu_dev, &policy->freq_table);
}

static struct cpufreq_driver sc9830_cpufreq_driver = {
	.name = "sc9830-cpufreq",
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK | CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sc9830_target,
	.get = cpufreq_generic_get,
	.init = sc9830_cpufreq_init,
	.exit = sc9830_cpufreq_exit,
	.suspend = cpufreq_generic_suspend,
};

static int sc9830_read_cell(struct device_node *node, const char *name, u32 *value)
{
	struct nvmem_cell *cell;
	u8 *data;
	size_t length, i;

	cell = of_nvmem_cell_get(node, name);
	if (IS_ERR(cell))
		return PTR_ERR(cell);
	data = nvmem_cell_read(cell, &length);
	nvmem_cell_put(cell);
	if (IS_ERR(data))
		return PTR_ERR(data);
	if (!length || length > sizeof(*value)) {
		kfree(data);
		return -EINVAL;
	}
	*value = 0;
	for (i = 0; i < length; i++)
		*value |= (u32)data[i] << (8 * i);
	kfree(data);
	return 0;
}

static int sc9830_cpufreq_probe(struct platform_device *pdev)
{
	static const char * const clock_names[] = { "cpu", "mpll", "intermediate", "axi" };
	struct device_node *node __free(device_node) = NULL;
	struct device_node *table __free(device_node) = NULL;
	struct sc9830_cpufreq *d;
	struct regmap *aon;
	struct resource resource;
	unsigned long rate = 0;
	u32 state, code, additional, chip;
	int i, ret, voltage;

	if (sc9830_policy)
		return -EBUSY;
	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->cpu_dev = get_cpu_device(0);
	if (!d->cpu_dev)
		return -EPROBE_DEFER;
	table = of_parse_phandle(d->cpu_dev->of_node, "operating-points-v2", 0);
	if (!of_device_is_compatible(table, "operating-points-v2-sprd-sc9830"))
		return -ENODEV;
	ret = sc9830_read_cell(table, "calibration_state", &state);
	if (ret)
		return ret;
	ret = sc9830_read_cell(table, "vddarm", &code);
	if (ret)
		return ret;
	ret = sc9830_read_cell(table, "additional", &additional);
	if (ret)
		return ret;
	if (state || code > 127 || additional > 3)
		return -EINVAL;
	d->delta = (int)code - 64 + 32 * ((int)additional - 1);
	aon = syscon_regmap_lookup_by_compatible("sprd,sc9830-aon-clk");
	if (IS_ERR(aon))
		return PTR_ERR(aon);
	ret = regmap_read(aon, 0xfc, &chip);
	if (ret)
		return ret;
	if (chip != 0x6b4c5300)
		return -ENODEV;
	node = of_find_compatible_node(NULL, NULL, "sprd,sc9830-apahb-gate");
	if (!node)
		return -ENODEV;
	ret = of_address_to_resource(node, 0, &resource);
	if (ret)
		return ret;
	if (resource_size(&resource) < 0x58)
		return -EINVAL;
	d->ap = of_iomap(node, 0);
	if (!d->ap)
		return -ENOMEM;
	d->fixed_mux = readl(d->ap + 0x54) & ~GENMASK(2, 0);
	d->fixed_div = readl(d->ap + 0x38) & ~GENMASK(10, 8);
	if (d->fixed_div & GENMASK(6, 4)) {
		ret = -EINVAL;
		goto unmap;
	}
	for (i = 0; i < NUM_CLOCKS; i++)
		d->clocks[i].id = clock_names[i];
	ret = clk_bulk_get(d->cpu_dev, NUM_CLOCKS, d->clocks);
	if (ret)
		goto unmap;
	d->arm = regulator_get_optional(d->cpu_dev, "cpu");
	if (IS_ERR(d->arm)) {
		ret = PTR_ERR(d->arm);
		goto put_clocks;
	}
	ret = dev_pm_opp_of_get_sharing_cpus(d->cpu_dev, &d->cpus);
	if (ret)
		goto put_regulator;
	if (!cpumask_equal(&d->cpus, cpu_possible_mask)) {
		ret = -EINVAL;
		goto put_regulator;
	}
	ret = dev_pm_opp_of_cpumask_add_table(&d->cpus);
	if (ret)
		goto put_regulator;
	if (dev_pm_opp_get_opp_count(d->cpu_dev) != 5) {
		ret = -EINVAL;
		goto remove_opps;
	}
	for (i = 0; i < 5; i++, rate++) {
		struct dev_pm_opp *opp = dev_pm_opp_find_freq_ceil(d->cpu_dev, &rate);

		if (IS_ERR(opp)) {
			ret = PTR_ERR(opp);
			goto remove_opps;
		}
		dev_pm_opp_put(opp);
		voltage = sc9830_opp_voltage(d, rate);
		if (voltage < 0 || regulator_is_supported_voltage(d->arm, voltage, voltage) <= 0) {
			ret = -EINVAL;
			goto remove_opps;
		}
	}
	rate = clk_get_rate(d->clocks[CPU].clk);
	voltage = sc9830_opp_voltage(d, rate);
	if (voltage < 0 || regulator_get_voltage(d->arm) < voltage ||
	    clk_get_rate(d->clocks[INTERMEDIATE].clk) != 768000000 ||
	    !clk_get_rate(d->clocks[MPLL].clk)) {
		ret = -EINVAL;
		goto remove_opps;
	}
	state = FIELD_GET(GENMASK(2, 0), readl(d->ap + 0x54));
	if (state != 2 && state != 6) {
		ret = -EINVAL;
		goto remove_opps;
	}
	ret = sc9830_verify_clock(d, rate, state,
		FIELD_GET(GENMASK(10, 8), readl(d->ap + 0x38)) + 1);
	if (ret)
		goto remove_opps;
	ret = clk_rate_exclusive_get(d->clocks[MPLL].clk);
	if (ret)
		goto remove_opps;
	ret = regulator_enable(d->arm);
	if (ret)
		goto put_exclusive;
	platform_set_drvdata(pdev, d);
	sc9830_policy = d;
	ret = cpufreq_register_driver(&sc9830_cpufreq_driver);
	if (!ret)
		return 0;
	sc9830_policy = NULL;
	regulator_disable(d->arm);
put_exclusive:
	clk_rate_exclusive_put(d->clocks[MPLL].clk);
remove_opps:
	dev_pm_opp_of_cpumask_remove_table(&d->cpus);
put_regulator:
	regulator_put(d->arm);
put_clocks:
	clk_bulk_put(NUM_CLOCKS, d->clocks);
unmap:
	iounmap(d->ap);
	return ret;
}

static void sc9830_cpufreq_remove(struct platform_device *pdev)
{
	struct sc9830_cpufreq *d = platform_get_drvdata(pdev);

	cpufreq_unregister_driver(&sc9830_cpufreq_driver);
	sc9830_policy = NULL;
	regulator_disable(d->arm);
	clk_rate_exclusive_put(d->clocks[MPLL].clk);
	dev_pm_opp_of_cpumask_remove_table(&d->cpus);
	regulator_put(d->arm);
	clk_bulk_put(NUM_CLOCKS, d->clocks);
	iounmap(d->ap);
}

static int sc9830_cpufreq_suspend(struct device *dev)
{
	struct sc9830_cpufreq *d = dev_get_drvdata(dev);

	return d->fault ? -EIO : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc9830_pm_ops, sc9830_cpufreq_suspend, NULL);

static struct platform_driver sc9830_platform_driver = {
	.probe = sc9830_cpufreq_probe,
	.remove = sc9830_cpufreq_remove,
	.driver = {
		.name = "sc9830-cpufreq",
		.pm = pm_sleep_ptr(&sc9830_pm_ops),
	},
};

static int __init sc9830_cpufreq_register(void)
{
	int ret;

	if (!of_machine_is_compatible("sprd,sc9830"))
		return -ENODEV;
	ret = platform_driver_register(&sc9830_platform_driver);
	if (ret)
		return ret;
	sc9830_device = platform_device_register_simple("sc9830-cpufreq", -1, NULL, 0);
	if (!IS_ERR(sc9830_device))
		return 0;
	platform_driver_unregister(&sc9830_platform_driver);
	return PTR_ERR(sc9830_device);
}
module_init(sc9830_cpufreq_register);

static void __exit sc9830_cpufreq_unregister(void)
{
	platform_device_unregister(sc9830_device);
	platform_driver_unregister(&sc9830_platform_driver);
}
module_exit(sc9830_cpufreq_unregister);

MODULE_DESCRIPTION("SC9830 calibrated shared-cluster CPU frequency control");
MODULE_LICENSE("GPL");
