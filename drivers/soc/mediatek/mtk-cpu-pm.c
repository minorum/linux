// SPDX-License-Identifier: GPL-2.0-only
#include <linux/arm-smccc.h>
#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/psci.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#include <uapi/linux/psci.h>

struct mtk_cpu_pm {
	struct device *dev;
	struct hlist_node node;
	int state;
	unsigned long failed;
	u64 mpidr[];
};

static int mtk_cpu_pm_prepare(unsigned int cpu, struct hlist_node *node)
{
	struct mtk_cpu_pm *pm = container_of(node, struct mtk_cpu_pm, node);
	unsigned int index = pm->mpidr[cpu] >> 8;

	return test_bit(index, &pm->failed) ? -EIO : 0;
}

static int mtk_cpu_pm_dead(unsigned int cpu, struct hlist_node *node)
{
	struct mtk_cpu_pm *pm = container_of(node, struct mtk_cpu_pm, node);
	unsigned int index = pm->mpidr[cpu] >> 8;
	struct arm_smccc_res result;
	int ret;

	ret = psci_ops.affinity_info(pm->mpidr[cpu], 0);
	if (ret != PSCI_0_2_AFFINITY_LEVEL_OFF) {
		dev_err(pm->dev, "CPU%u affinity state %d prevents power-down completion\n",
			cpu, ret);
		set_bit(index, &pm->failed);
		return 0;
	}

	arm_smccc_smc(MTK_SIP_POWER_DOWN_CORE, index, 0, 0, 0, 0, 0, 0, &result);
	if (result.a0) {
		dev_err(pm->dev, "CPU%u power-down completion failed: %lx\n", cpu, result.a0);
		set_bit(index, &pm->failed);
	}

	return 0;
}

static void mtk_cpu_pm_remove(void *data)
{
	struct mtk_cpu_pm *pm = data;

	cpuhp_state_remove_instance_nocalls(pm->state, &pm->node);
	cpuhp_remove_multi_state(pm->state);
}

static int mtk_cpu_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_cpu_pm *pm;
	struct device_node *node;
	unsigned int cpu;
	u64 hwid;
	int ret;

	if (!psci_ops.affinity_info)
		return dev_err_probe(dev, -ENODEV, "PSCI affinity-info unavailable\n");

	pm = devm_kzalloc(dev, struct_size(pm, mpidr, nr_cpu_ids), GFP_KERNEL);
	if (!pm)
		return -ENOMEM;
	pm->dev = dev;

	for_each_possible_cpu(cpu) {
		node = of_get_cpu_node(cpu, NULL);
		if (!node)
			return dev_err_probe(dev, -ENODEV, "CPU%u node unavailable\n", cpu);
		hwid = of_get_cpu_hwid(node, 0);
		of_node_put(node);
		if (hwid & ~GENMASK_ULL(10, 8))
			return dev_err_probe(dev, -EINVAL, "Unsupported CPU%u hardware ID %llx\n",
					     cpu, hwid);
		pm->mpidr[cpu] = hwid;
	}

	pm->state = cpuhp_setup_state_multi(CPUHP_BP_PREPARE_DYN,
					    "soc/mediatek/cpu-pm:prepare",
					    mtk_cpu_pm_prepare, mtk_cpu_pm_dead);
	if (pm->state < 0)
		return dev_err_probe(dev, pm->state, "Failed to register CPU hotplug state\n");

	ret = cpuhp_state_add_instance_nocalls(pm->state, &pm->node);
	if (ret) {
		cpuhp_remove_multi_state(pm->state);
		return dev_err_probe(dev, ret, "Failed to register CPU hotplug instance\n");
	}

	return devm_add_action_or_reset(dev, mtk_cpu_pm_remove, pm);
}

static const struct of_device_id mtk_cpu_pm_of_match[] = {
	{ .compatible = "mediatek,mt6785-cpu-pm" },
	{ }
};

static struct platform_driver mtk_cpu_pm_driver = {
	.probe = mtk_cpu_pm_probe,
	.driver = {
		.name = "mtk-cpu-pm",
		.of_match_table = mtk_cpu_pm_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mtk_cpu_pm_driver);

MODULE_DESCRIPTION("MediaTek MT6785 CPU power-down completion");
MODULE_LICENSE("GPL");
