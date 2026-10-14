// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <asm/memory.h>

#include "entry.c"
#include "resources.c"
#include "sram.c"
#include "analog.c"
#include "guard.c"

struct sc9830_retention_platform {
	struct sc9830_retention_entry entry;
	struct sc9830_retention_resources maps;
	struct sc9830_retention_sram sram;
	struct sc9830_retention_analog analog;
	struct device *pmic;
	bool active;
	bool available;
};

static struct sc9830_retention_platform *sc9830_platform;
static DEFINE_MUTEX(sc9830_analog_lock);
static bool sc9830_trial;

static int __init sc9830_retention_trial(char *value)
{
	return kstrtobool(value, &sc9830_trial);
}
early_param("sc9830_retention_trial", sc9830_retention_trial);

static void sc9830_retention_end_locked(void)
{
	struct sc9830_retention_platform *state = sc9830_platform;
	int ret;

	if (!state->active)
		return;
	ret = sc9830_retention_sram_restore(&state->sram);
	ret |= sc9830_retention_analog_restore(&state->analog);
	if (ret)
		sc9830_retention_recovery();
	if (sc9830_retention_sram_release(&state->sram))
		sc9830_retention_recovery();
	state->entry.status = NULL;
	state->active = false;
}

static int sc9830_retention_begin_locked(suspend_state_t target)
{
	struct sc9830_retention_platform *state = sc9830_platform;
	phys_addr_t resume = __pa_symbol(cpu_resume);
	u32 ticks;
	int ret;

	if (target == PM_SUSPEND_TO_IDLE)
		return 0;
	if (target != PM_SUSPEND_MEM || state->active ||
	    resume > U32_MAX || !resume || (resume & 3))
		return -EINVAL;
	if (!state->available)
		return -ENODEV;
	ret = sc9830_retention_guard_check(state->analog.map, &ticks);
	if (ret)
		return ret;
	state->active = true;
	ret = sc9830_retention_analog_prepare(&state->analog);
	if (ret) {
		if (state->analog.owned)
			sc9830_retention_recovery();
		goto end;
	}
	ret = sc9830_retention_sram_stage(&state->sram, resume, 1000000);
	if (ret) {
		if (state->sram.installed)
			sc9830_retention_recovery();
		goto end;
	}
	state->entry.status = state->sram.base + 0x400 + state->sram.status;
	return 0;
end:
	sc9830_retention_end_locked();
	return ret;
}

static void sc9830_retention_platform_end(void)
{
	mutex_lock(&sc9830_analog_lock);
	sc9830_retention_end_locked();
	mutex_unlock(&sc9830_analog_lock);
}

static int sc9830_retention_begin(suspend_state_t target)
{
	int ret;

	mutex_lock(&sc9830_analog_lock);
	ret = sc9830_retention_begin_locked(target);
	mutex_unlock(&sc9830_analog_lock);
	return ret;
}

static int sc9830_retention_prepare(void)
{
	u32 ticks;
	int ret;

	mutex_lock(&sc9830_analog_lock);
	ret = sc9830_platform->available ?
		sc9830_retention_guard_check(sc9830_platform->analog.map, &ticks) : -ENODEV;
	mutex_unlock(&sc9830_analog_lock);
	return ret;
}

static int sc9830_retention_enter(suspend_state_t target)
{
	struct sc9830_retention_platform *state = sc9830_platform;
	u32 chip;
	int ret;

	if (target != PM_SUSPEND_MEM || !state->active)
		return -EINVAL;
	ret = sc9830_retention_sram_verify(&state->sram);
	if (ret)
		return ret;
	chip = readl(state->entry.context.aon + 0xfc);
	if (chip == 0x8300a001 && !(readl(state->entry.policy.pmu + 0x5c) & BIT(24)))
		return -EBUSY;
	ret = sc9830_retention_execute(&state->entry);
	pr_info("SC9830_RETENTION_RETURN ret=%d status=%u\n",
		ret, readl(state->entry.status));
	return ret;
}

static const struct platform_suspend_ops sc9830_retention_ops = {
	.valid = suspend_valid_only_mem,
	.begin = sc9830_retention_begin,
	.prepare = sc9830_retention_prepare,
	.enter = sc9830_retention_enter,
	.end = sc9830_retention_platform_end,
};

static void sc9830_retention_register(struct work_struct *work)
{
	if (!READ_ONCE(sc9830_platform->available))
		return;
	suspend_set_ops(&sc9830_retention_ops);
	if (READ_ONCE(sc9830_platform->available))
		pr_info("SC9830_RETENTION_TRIAL_READY\n");
}

static DECLARE_WORK(sc9830_retention_registration, sc9830_retention_register);

static int sc9830_retention_platform_probe(struct platform_device *pdev)
{
	static const char * const names[] = {
		"sprd,ap-clk", "sprd,ap-ahb", "sprd,ap-apb", "sprd,aon",
		"sprd,pub", "sprd,pmu", "sprd,dmc", "sprd,phy",
	};
	struct sc9830_retention_platform *state;
	struct device_node *node = pdev->dev.of_node, *provider;
	struct device_link *link;
	struct resource resources[ARRAY_SIZE(names)];
	unsigned int i;
	int ret = -EINVAL;

	if (!sc9830_trial || !of_machine_is_compatible("samsung,j3xnlte") ||
	    !of_device_is_available(node))
		return -ENODEV;
	if (sc9830_platform)
		return -EBUSY;
	state = kzalloc_obj(*state);
	if (!state)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		provider = of_parse_phandle(node, names[i], 0);
		if (!provider) {
			ret = -EINVAL;
			goto free;
		}
		ret = of_address_to_resource(provider, i == 4 ? 1 : 0, &resources[i]);
		of_node_put(provider);
		if (ret)
			goto free;
	}
	provider = of_parse_phandle(node, "sprd,pmic", 0);
	if (!provider) {
		ret = -EINVAL;
		goto free;
	}
	state->pmic = bus_find_device_by_of_node(&spi_bus_type, provider);
	of_node_put(provider);
	ret = -EPROBE_DEFER;
	if (!state->pmic)
		goto free;
	if (!of_device_is_compatible(state->pmic->of_node, "sprd,sc2723"))
		goto put_pmic;
	ret = sc9830_retention_resources_map(&state->maps, resources, ARRAY_SIZE(resources));
	if (ret)
		goto put_pmic;
	device_lock(state->pmic);
	if (!device_is_bound(state->pmic) ||
	    READ_ONCE(state->pmic->links.status) != DL_DEV_DRIVER_BOUND) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	state->analog.map = dev_get_regmap(state->pmic, NULL);
	if (!state->analog.map) {
		ret = -ENODEV;
		goto unlock;
	}
	link = device_link_add(&pdev->dev, state->pmic, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link) {
		ret = -ENOMEM;
		goto unlock;
	}
	device_unlock(state->pmic);
	state->entry.context.ap = state->maps.base[0];
	state->entry.context.ahb = state->maps.base[1];
	state->entry.context.apb = state->maps.base[2];
	state->entry.context.aon = state->maps.base[3];
	state->entry.context.pub = state->maps.base[4];
	state->entry.policy.pmu = state->maps.base[5];
	state->entry.policy.dmc = state->maps.base[6];
	state->entry.policy.phy = state->maps.base[7];
	state->entry.address = 0x400;
	platform_set_drvdata(pdev, state);
	state->available = true;
	sc9830_platform = state;
	schedule_work(&sc9830_retention_registration);
	return 0;
unlock:
	device_unlock(state->pmic);
	sc9830_retention_resources_release(&state->maps);
put_pmic:
	put_device(state->pmic);
free:
	kfree(state);
	return ret;
}

static void sc9830_retention_platform_remove(struct platform_device *pdev)
{
	struct sc9830_retention_platform *state = platform_get_drvdata(pdev);
	struct device *pmic;

	mutex_lock(&sc9830_analog_lock);
	if (state->active)
		sc9830_retention_recovery();
	state->available = false;
	state->analog.map = NULL;
	pmic = state->pmic;
	state->pmic = NULL;
	mutex_unlock(&sc9830_analog_lock);
	put_device(pmic);
}

static const struct of_device_id sc9830_retention_match[] = {
	{ .compatible = "sprd,sc9830-suspend" },
	{ }
};

static struct platform_driver sc9830_retention_driver = {
	.probe = sc9830_retention_platform_probe,
	.remove = sc9830_retention_platform_remove,
	.driver = {
		.name = "sc9830-suspend",
		.of_match_table = sc9830_retention_match,
		.suppress_bind_attrs = true,
	},
};

static int __init sc9830_retention_platform_init(void)
{
	return sc9830_trial ? platform_driver_register(&sc9830_retention_driver) : 0;
}
late_initcall(sc9830_retention_platform_init);

