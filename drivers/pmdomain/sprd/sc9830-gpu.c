// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define GPU_CFG		0x20
#define GPU_FORCE_OFF	BIT(25)
#define GPU_AUTO_OFF	BIT(24)
#define GPU_STATUS	0xbc
#define GPU_STATE	GENMASK(27, 24)
#define GPU_OFF		7
#define GPU_CLOCK	0x4
#define GPU_SET		0x1000
#define GPU_CLEAR	0x2000

struct sc9830_gpu {
	struct generic_pm_domain genpd;
	struct device *dev;
	struct regmap *pmu;
	struct regmap *clock;
	struct clk *bus;
	struct clk *source;
	struct regulator *supply;
	u32 saved;
	bool saved_valid;
	bool bus_enabled;
	bool source_enabled;
	bool supply_enabled;
	bool supply_uncertain;
	bool provider_added;
	bool faulted;
};

static int sc9830_gpu_state(struct sc9830_gpu *gpu, unsigned int *state)
{
	unsigned int sample[3];
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sample); i++) {
		ret = regmap_read(gpu->pmu, GPU_STATUS, &sample[i]);
		if (ret)
			return ret;
		sample[i] = FIELD_GET(GPU_STATE, sample[i]);
	}
	if (sample[0] != sample[1] || sample[1] != sample[2])
		return -EAGAIN;
	*state = sample[0];
	return 0;
}

static int sc9830_gpu_wait(struct sc9830_gpu *gpu, unsigned int target)
{
	unsigned int state;
	int ret, poll;

	poll = read_poll_timeout(sc9830_gpu_state, ret,
				 ret != -EAGAIN && (ret || state == target),
				 50, 3000000, false, gpu, &state);
	return poll ?: ret;
}

static int sc9830_gpu_start(struct sc9830_gpu *gpu, bool unwind_supply)
{
	bool supply_started = false;
	int ret, rollback;

	if (gpu->supply && !gpu->supply_enabled) {
		ret = regulator_enable(gpu->supply);
		if (ret)
			return ret;
		gpu->supply_enabled = true;
		supply_started = true;
	}
	if (!gpu->source_enabled) {
		ret = clk_enable(gpu->source);
		if (ret) {
			if (supply_started && unwind_supply) {
				rollback = regulator_disable(gpu->supply);
				if (rollback) {
					gpu->supply_uncertain = true;
					dev_err(gpu->dev, "GPU supply rollback failed: %d\n",
						rollback);
				} else {
					gpu->supply_enabled = false;
				}
			}
			return ret;
		}
		gpu->source_enabled = true;
		usleep_range(1000, 1100);
	}
	ret = regmap_write(gpu->pmu, GPU_CFG + GPU_CLEAR, GPU_FORCE_OFF);
	if (ret)
		return ret;
	usleep_range(100, 200);
	if (!gpu->bus_enabled) {
		ret = clk_enable(gpu->bus);
		if (ret)
			return ret;
		gpu->bus_enabled = true;
	}
	ret = sc9830_gpu_wait(gpu, 0);
	if (ret)
		return ret;
	if (gpu->saved_valid)
		return regmap_write(gpu->clock, GPU_CLOCK, gpu->saved);
	return 0;
}

static int sc9830_gpu_power_on(struct generic_pm_domain *domain)
{
	struct sc9830_gpu *gpu = container_of(domain, struct sc9830_gpu, genpd);
	int ret;

	if (READ_ONCE(gpu->faulted))
		return -EIO;
	ret = sc9830_gpu_start(gpu, true);
	if (ret) {
		WRITE_ONCE(gpu->faulted, true);
		dev_err(gpu->dev, "GPU power-on failed: %d\n", ret);
	}
	return ret;
}

static int sc9830_gpu_stop(struct sc9830_gpu *gpu)
{
	int ret;

	if (gpu->bus_enabled) {
		clk_disable(gpu->bus);
		gpu->bus_enabled = false;
	}
	ret = regmap_write(gpu->pmu, GPU_CFG + GPU_SET, GPU_FORCE_OFF);
	if (!ret)
		ret = sc9830_gpu_wait(gpu, GPU_OFF);
	if (ret)
		return ret;
	if (gpu->source_enabled) {
		clk_disable(gpu->source);
		gpu->source_enabled = false;
	}
	if (gpu->supply_uncertain)
		return -EIO;
	if (gpu->supply_enabled) {
		ret = regulator_disable(gpu->supply);
		if (ret) {
			gpu->supply_uncertain = true;
			WRITE_ONCE(gpu->faulted, true);
			dev_err(gpu->dev, "GPU supply release failed: %d\n", ret);
			return ret;
		}
		gpu->supply_enabled = false;
	}
	return 0;
}

static int sc9830_gpu_power_off(struct generic_pm_domain *domain)
{
	struct sc9830_gpu *gpu = container_of(domain, struct sc9830_gpu, genpd);
	int ret, rollback;

	if (READ_ONCE(gpu->faulted))
		return -EIO;
	ret = regmap_read(gpu->clock, GPU_CLOCK, &gpu->saved);
	if (ret)
		return ret;
	gpu->saved_valid = true;
	ret = sc9830_gpu_stop(gpu);
	if (!ret || READ_ONCE(gpu->faulted))
		return ret;
	rollback = sc9830_gpu_start(gpu, true);
	if (rollback) {
		WRITE_ONCE(gpu->faulted, true);
		dev_err(gpu->dev, "GPU power-off rollback failed: %d\n", rollback);
	}
	return ret;
}

static int sc9830_gpu_start_dev(struct device *dev)
{
	struct generic_pm_domain *domain = pd_to_genpd(dev->pm_domain);
	struct sc9830_gpu *gpu = container_of(domain, struct sc9830_gpu, genpd);
	int ret;

	if (READ_ONCE(gpu->faulted))
		return -EIO;
	ret = sc9830_gpu_wait(gpu, 0);
	if (ret)
		WRITE_ONCE(gpu->faulted, true);
	return ret;
}

static int sc9830_gpu_init_state(struct sc9830_gpu *gpu, unsigned int state)
{
	int ret, rollback;

	if (state != GPU_OFF && state != 0)
		return -EBUSY;
	ret = regmap_update_bits(gpu->pmu, GPU_CFG,
				 GPU_FORCE_OFF | GPU_AUTO_OFF | GENMASK(23, 0),
				 (state == GPU_OFF ? GPU_FORCE_OFF : 0) | 0x010101);
	if (!ret && state == 0)
		ret = sc9830_gpu_start(gpu, false);
	if (!ret)
		return 0;
	rollback = sc9830_gpu_stop(gpu);
	if (rollback)
		dev_err(gpu->dev, "GPU handover rollback failed: %d\n", rollback);
	return ret;
}

static void sc9830_gpu_remove_provider(void *data)
{
	struct sc9830_gpu *gpu = data;
	int ret;

	if (gpu->provider_added)
		of_genpd_del_provider(gpu->dev->of_node);
	ret = pm_genpd_remove(&gpu->genpd);
	if (ret) {
		dev_err(gpu->dev, "GPU domain removal failed: %d\n", ret);
		return;
	}
	if (gpu->bus_enabled || gpu->source_enabled || gpu->supply_enabled) {
		ret = sc9830_gpu_stop(gpu);
		if (ret)
			dev_err(gpu->dev, "GPU cleanup failed: %d\n", ret);
	}
}

static int sc9830_gpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc9830_gpu *gpu;
	unsigned int state;
	int ret, poll, rollback;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;
	gpu->dev = dev;
	gpu->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,pmu");
	if (IS_ERR(gpu->pmu))
		return dev_err_probe(dev, PTR_ERR(gpu->pmu), "PMU regmap\n");
	gpu->clock = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,gpu-clk");
	if (IS_ERR(gpu->clock))
		return dev_err_probe(dev, PTR_ERR(gpu->clock), "GPU clock regmap\n");
	gpu->bus = devm_clk_get_prepared(dev, "bus");
	if (IS_ERR(gpu->bus))
		return dev_err_probe(dev, PTR_ERR(gpu->bus), "GPU bus clock\n");
	gpu->source = devm_clk_get_prepared(dev, "source");
	if (IS_ERR(gpu->source))
		return dev_err_probe(dev, PTR_ERR(gpu->source), "GPU source clock\n");
	poll = read_poll_timeout(sc9830_gpu_state, ret, ret != -EAGAIN,
				 50, 3000000, false, gpu, &state);
	if (poll || ret)
		return dev_err_probe(dev, poll ?: ret, "unstable GPU domain\n");
	if (of_property_present(dev->of_node, "domain-supply")) {
		gpu->supply = devm_regulator_get_optional(dev, "domain");
		if (IS_ERR(gpu->supply))
			return dev_err_probe(dev, PTR_ERR(gpu->supply),
					     "GPU domain supply\n");
	}
	ret = sc9830_gpu_init_state(gpu, state);
	if (ret)
		return ret;
	gpu->genpd.name = "sc9830-gpu";
	gpu->genpd.power_on = sc9830_gpu_power_on;
	gpu->genpd.power_off = sc9830_gpu_power_off;
	gpu->genpd.dev_ops.start = sc9830_gpu_start_dev;
	ret = pm_genpd_init(&gpu->genpd, NULL, state == GPU_OFF);
	if (ret) {
		if (gpu->bus_enabled || gpu->source_enabled || gpu->supply_enabled) {
			rollback = sc9830_gpu_stop(gpu);
			if (rollback)
				dev_err(dev, "GPU registration rollback failed: %d\n", rollback);
		}
		return ret;
	}
	ret = devm_add_action_or_reset(dev, sc9830_gpu_remove_provider, gpu);
	if (ret)
		return ret;
	ret = of_genpd_add_provider_simple(dev->of_node, &gpu->genpd);
	if (!ret)
		gpu->provider_added = true;
	return ret;
}

static const struct of_device_id sc9830_gpu_match[] = {
	{ .compatible = "sprd,sc9830-gpu-pwrc" },
	{ }
};

static struct platform_driver sc9830_gpu_driver = {
	.probe = sc9830_gpu_probe,
	.driver = {
		.name = "sc9830-gpu-pwrc",
		.of_match_table = sc9830_gpu_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sc9830_gpu_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 GPU power domain");
MODULE_LICENSE("GPL");
