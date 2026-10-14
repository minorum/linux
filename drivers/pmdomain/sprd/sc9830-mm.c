// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>

#define MM_TOP_CFG		0x1c
#define MM_FORCE_SHUTDOWN	BIT(25)
#define MM_POWER_STATUS		0xbc
#define MM_STATE_MASK		GENMASK(31, 28)
#define MM_STATE_OFF		7
#define MM_AON_EB		BIT(25)
#define MM_CKG_EB		BIT(6)
#define MM_AXI_CKG		(BIT(7) | BIT(8))
#define MM_CKG_FIRST		0x20
#define MM_CKG_COUNT		11
#define MM_SET			0x1000
#define MM_CLEAR			0x2000
#define MM_POLL_US		50
#define MM_TIMEOUT_US		3000000

struct sc9830_mm {
	struct generic_pm_domain genpd;
	struct device *dev;
	struct regmap *pmu;
	struct regmap *aon;
	struct regmap *ahb;
	struct regmap *clk;
	/* Initial context. */
	struct mutex context_lock;
	u32 saved[MM_CKG_COUNT];
	bool saved_valid;
	bool faulted;
};

static int sc9830_mm_state(struct sc9830_mm *mm, unsigned int *state)
{
	unsigned int sample[3];
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sample); i++) {
		ret = regmap_read(mm->pmu, MM_POWER_STATUS, &sample[i]);
		if (ret)
			return ret;
		sample[i] = FIELD_GET(MM_STATE_MASK, sample[i]);
	}
	if (sample[0] != sample[1] || sample[1] != sample[2])
		return -EAGAIN;
	*state = sample[0];
	return 0;
}

static int sc9830_mm_wait(struct sc9830_mm *mm, unsigned int target)
{
	unsigned int state;
	int ret, poll;

	poll = read_poll_timeout(sc9830_mm_state, ret,
				 ret != -EAGAIN && (ret || state == target),
				 MM_POLL_US, MM_TIMEOUT_US, false, mm, &state);
	return poll ?: ret;
}

static int sc9830_mm_gates(struct sc9830_mm *mm)
{
	int ret;

	ret = regmap_write(mm->aon, MM_SET, MM_AON_EB);
	if (ret)
		return ret;
	return regmap_write(mm->ahb, MM_SET, MM_CKG_EB);
}

static int sc9830_mm_save(struct sc9830_mm *mm)
{
	u32 saved[MM_CKG_COUNT];
	int i, ret;

	for (i = 0; i < MM_CKG_COUNT; i++) {
		ret = regmap_read(mm->clk, MM_CKG_FIRST + 4 * i, &saved[i]);
		if (ret)
			return ret;
	}
	memcpy(mm->saved, saved, sizeof(saved));
	mm->saved_valid = true;
	return 0;
}

static int sc9830_mm_restore(struct sc9830_mm *mm)
{
	int i, ret;

	for (i = 0; i < MM_CKG_COUNT; i++) {
		ret = regmap_write(mm->clk, MM_CKG_FIRST + 4 * i, mm->saved[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int sc9830_mm_start(struct sc9830_mm *mm)
{
	int ret;

	ret = regmap_write(mm->pmu, MM_TOP_CFG + MM_CLEAR,
			   MM_FORCE_SHUTDOWN);
	if (ret)
		return ret;
	ret = sc9830_mm_wait(mm, 0);
	if (ret)
		return ret;
	ret = sc9830_mm_gates(mm);
	if (ret)
		return ret;
	ret = regmap_update_bits(mm->ahb, 8, MM_AXI_CKG, MM_AXI_CKG);
	if (ret)
		return ret;
	ret = regmap_update_bits(mm->clk, MM_CKG_FIRST, 3, 3);
	if (ret)
		return ret;
	if (mm->saved_valid)
		return sc9830_mm_restore(mm);
	return sc9830_mm_save(mm);
}

static int sc9830_mm_power_on(struct generic_pm_domain *genpd)
{
	struct sc9830_mm *mm = container_of(genpd, struct sc9830_mm, genpd);
	int ret, rollback;

	if (READ_ONCE(mm->faulted))
		return -EIO;
	ret = sc9830_mm_start(mm);
	if (!ret)
		return 0;
	rollback = regmap_write(mm->aon, MM_CLEAR, MM_AON_EB);
	if (!rollback)
		rollback = regmap_write(mm->pmu, MM_TOP_CFG + MM_SET,
					MM_FORCE_SHUTDOWN);
	if (!rollback)
		rollback = sc9830_mm_wait(mm, MM_STATE_OFF);
	if (rollback) {
		WRITE_ONCE(mm->faulted, true);
		dev_err(mm->dev, "power-on rollback failed: %d\n", rollback);
	}
	return ret;
}

static int sc9830_mm_power_off(struct generic_pm_domain *genpd)
{
	struct sc9830_mm *mm = container_of(genpd, struct sc9830_mm, genpd);
	int ret, rollback;

	if (READ_ONCE(mm->faulted))
		return -EIO;
	ret = sc9830_mm_wait(mm, 0);
	if (ret) {
		WRITE_ONCE(mm->faulted, true);
		return ret;
	}
	ret = sc9830_mm_gates(mm);
	if (ret)
		return ret;
	ret = sc9830_mm_save(mm);
	if (ret)
		return ret;
	ret = regmap_write(mm->aon, MM_CLEAR, MM_AON_EB);
	if (!ret)
		ret = regmap_write(mm->pmu, MM_TOP_CFG + MM_SET,
				   MM_FORCE_SHUTDOWN);
	if (!ret)
		ret = sc9830_mm_wait(mm, MM_STATE_OFF);
	if (!ret)
		return 0;
	rollback = sc9830_mm_start(mm);
	if (rollback) {
		WRITE_ONCE(mm->faulted, true);
		dev_err(mm->dev, "power-off rollback failed: %d\n", rollback);
	}
	return ret;
}

static int sc9830_mm_start_dev(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct sc9830_mm *mm = container_of(genpd, struct sc9830_mm, genpd);
	unsigned int aon, ahb;
	int ret;

	mutex_lock(&mm->context_lock);
	if (READ_ONCE(mm->faulted)) {
		ret = -EIO;
		goto fault;
	}
	ret = sc9830_mm_wait(mm, 0);
	if (ret)
		goto fault;
	if (mm->saved_valid) {
		ret = sc9830_mm_gates(mm);
		goto fault;
	}
	ret = regmap_read(mm->aon, 0, &aon);
	if (ret)
		goto fault;
	ret = regmap_write(mm->aon, MM_SET, MM_AON_EB);
	if (ret)
		goto fault;
	ret = regmap_read(mm->ahb, 0, &ahb);
	if (ret)
		goto fault;
	ret = regmap_write(mm->ahb, MM_SET, MM_CKG_EB);
	if (ret)
		goto fault;
	if (!(aon & MM_AON_EB) && !(ahb & MM_CKG_EB)) {
		ret = regmap_update_bits(mm->ahb, 8, MM_AXI_CKG, MM_AXI_CKG);
		if (ret)
			goto fault;
		ret = regmap_update_bits(mm->clk, MM_CKG_FIRST, 3, 3);
		if (ret)
			goto fault;
	}
	ret = sc9830_mm_save(mm);
	if (!ret)
		ret = sc9830_mm_start(mm);
fault:
	if (ret)
		WRITE_ONCE(mm->faulted, true);
	mutex_unlock(&mm->context_lock);
	return ret;
}

static int sc9830_mm_stop_dev(struct device *dev)
{
	return 0;
}

static void sc9830_mm_remove_provider(void *data)
{
	struct sc9830_mm *mm = data;

	of_genpd_del_provider(mm->dev->of_node);
	pm_genpd_remove(&mm->genpd);
}

static int sc9830_mm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc9830_mm *mm;
	unsigned int state, aon;
	int ret, poll;

	mm = devm_kzalloc(dev, sizeof(*mm), GFP_KERNEL);
	if (!mm)
		return -ENOMEM;
	mm->dev = dev;
	mutex_init(&mm->context_lock);
	mm->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,pmu");
	if (IS_ERR(mm->pmu))
		return dev_err_probe(dev, PTR_ERR(mm->pmu), "PMU regmap\n");
	mm->aon = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,aon");
	if (IS_ERR(mm->aon))
		return dev_err_probe(dev, PTR_ERR(mm->aon), "AON regmap\n");
	mm->ahb = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,mm-ahb");
	if (IS_ERR(mm->ahb))
		return dev_err_probe(dev, PTR_ERR(mm->ahb), "MM AHB regmap\n");
	mm->clk = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,mm-clk");
	if (IS_ERR(mm->clk))
		return dev_err_probe(dev, PTR_ERR(mm->clk), "MM clock regmap\n");
	poll = read_poll_timeout(sc9830_mm_state, ret, ret != -EAGAIN,
				 MM_POLL_US, MM_TIMEOUT_US, false, mm, &state);
	if (poll || ret)
		return dev_err_probe(dev, poll ?: ret, "unstable MM domain\n");
	if (state != MM_STATE_OFF)
		return dev_err_probe(dev, -EBUSY, "MM domain is not unowned\n");
	ret = regmap_read(mm->aon, 0, &aon);
	if (ret)
		return ret;
	if (aon & MM_AON_EB)
		return dev_err_probe(dev, -EBUSY, "inherited MM gate enabled\n");
	mm->genpd.name = "sc9830-mm";
	mm->genpd.power_on = sc9830_mm_power_on;
	mm->genpd.power_off = sc9830_mm_power_off;
	mm->genpd.dev_ops.start = sc9830_mm_start_dev;
	mm->genpd.dev_ops.stop = sc9830_mm_stop_dev;
	ret = pm_genpd_init(&mm->genpd, NULL, true);
	if (ret)
		return ret;
	ret = of_genpd_add_provider_simple(dev->of_node, &mm->genpd);
	if (ret) {
		pm_genpd_remove(&mm->genpd);
		return ret;
	}
	return devm_add_action_or_reset(dev, sc9830_mm_remove_provider, mm);
}

static const struct of_device_id sc9830_mm_match[] = {
	{ .compatible = "sprd,sc9830-mm-pwrc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc9830_mm_match);

static struct platform_driver sc9830_mm_driver = {
	.probe = sc9830_mm_probe,
	.driver = {
		.name = "sc9830-mm-pwrc",
		.of_match_table = sc9830_mm_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sc9830_mm_driver);

MODULE_DESCRIPTION("Spreadtrum SC9830 multimedia power domain");
MODULE_LICENSE("GPL");
