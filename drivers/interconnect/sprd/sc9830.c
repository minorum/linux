// SPDX-License-Identifier: GPL-2.0-only
#include <linux/interconnect-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <dt-bindings/interconnect/sprd,sc9830.h>

static const u32 sc9830_qos_regs[] = { 0x25c, 0x264, 0x26c, 0x4b4 };
static const u32 sc9830_qos_profile[] = {
	0x08000020, 0x08000100, 0x10000040, 0x100,
};

struct sc9830_icc {
	struct icc_provider provider;
	struct icc_node *master;
	struct regmap *dmc;
	u32 saved[ARRAY_SIZE(sc9830_qos_regs)];
	bool owned;
	bool faulted;
};

static u32 sc9830_qos_mask(unsigned int index)
{
	return index == 3 ? 0x3ff : ~0U;
}

static int sc9830_qos_read(struct sc9830_icc *icc, u32 *values)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(sc9830_qos_regs); i++) {
		ret = regmap_read(icc->dmc, sc9830_qos_regs[i], &values[i]);
		if (ret)
			return ret;
		values[i] &= sc9830_qos_mask(i);
	}
	return 0;
}

static int sc9830_qos_check(struct sc9830_icc *icc, const u32 *expected)
{
	u32 values[ARRAY_SIZE(sc9830_qos_regs)];
	int ret = sc9830_qos_read(icc, values);

	if (ret)
		return ret;
	return memcmp(values, expected, sizeof(values)) ? -EIO : 0;
}

static int sc9830_qos_write(struct sc9830_icc *icc, const u32 *values)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_write(icc->dmc, sc9830_qos_regs[i], values[i]);
		if (ret)
			return ret;
	}
	return regmap_update_bits(icc->dmc, sc9830_qos_regs[3], 0x3ff,
				  values[3]);
}

static int sc9830_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct sc9830_icc *icc = container_of(src->provider,
						  struct sc9830_icc, provider);
	bool demand = icc->master->avg_bw || icc->master->peak_bw;
	int ret;

	if (icc->faulted)
		return -EIO;
	if (src->provider != dst->provider)
		return -EINVAL;
	if (!icc->owned && !demand)
		return 0;
	if (icc->owned) {
		ret = sc9830_qos_check(icc, sc9830_qos_profile);
		if (ret)
			goto fault;
		if (demand)
			return 0;
		ret = sc9830_qos_write(icc, icc->saved);
		if (ret)
			goto fault;
		ret = sc9830_qos_check(icc, icc->saved);
		if (ret)
			goto fault;
		icc->owned = false;
		return 0;
	}
	ret = sc9830_qos_read(icc, icc->saved);
	if (ret)
		return ret;
	ret = sc9830_qos_check(icc, icc->saved);
	if (ret)
		goto fault;
	icc->owned = true;
	ret = sc9830_qos_write(icc, sc9830_qos_profile);
	if (ret)
		goto fault;
	ret = sc9830_qos_check(icc, sc9830_qos_profile);
	if (!ret)
		return 0;
fault:
	icc->faulted = true;
	dev_err(icc->provider.dev, "DMC QoS state uncertain; ownership retained\n");
	return ret;
}

static int sc9830_icc_aggregate(struct icc_node *node, u32 tag, u32 avg,
				u32 peak, u32 *agg_avg, u32 *agg_peak)
{
	if (check_add_overflow(*agg_avg, avg, agg_avg))
		*agg_avg = U32_MAX;
	*agg_peak = max(*agg_peak, peak);
	return 0;
}

static int sc9830_icc_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	*avg = 0;
	*peak = 0;
	return 0;
}

static int sc9830_icc_idle(struct regmap *pmu)
{
	u32 state;
	unsigned int i;
	int ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_read(pmu, 0xbc, &state);
		if (ret)
			return ret;
		if ((state >> 28) != 7)
			return -EBUSY;
	}
	return 0;
}

static int sc9830_icc_probe(struct platform_device *pdev)
{
	static const struct regmap_config config = {
		.reg_bits = 32,
		.val_bits = 32,
		.reg_stride = 4,
		.max_register = 0x4b4,
		.fast_io = true,
	};
	static const char *const names[] = { "dcam", "ddr" };
	struct device *dev = &pdev->dev;
	struct icc_onecell_data *data;
	struct icc_provider *provider;
	struct sc9830_icc *icc;
	struct regmap *pmu;
	void __iomem *base;
	unsigned int i;
	int ret;

	icc = devm_kzalloc(dev, sizeof(*icc), GFP_KERNEL);
	data = devm_kzalloc(dev, struct_size(data, nodes, 2), GFP_KERNEL);
	if (!icc || !data)
		return -ENOMEM;
	pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,pmu");
	if (IS_ERR(pmu))
		return PTR_ERR(pmu);
	ret = sc9830_icc_idle(pmu);
	if (ret)
		return dev_err_probe(dev, ret, "MM domain is not idle\n");
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	icc->dmc = devm_regmap_init_mmio(dev, base, &config);
	if (IS_ERR(icc->dmc))
		return PTR_ERR(icc->dmc);
	provider = &icc->provider;
	provider->dev = dev;
	provider->set = sc9830_icc_set;
	provider->aggregate = sc9830_icc_aggregate;
	provider->get_bw = sc9830_icc_get_bw;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = data;
	icc_provider_init(provider);
	data->num_nodes = 2;
	for (i = 0; i < data->num_nodes; i++) {
		data->nodes[i] = icc_node_create_dyn();
		if (IS_ERR(data->nodes[i])) {
			ret = PTR_ERR(data->nodes[i]);
			goto nodes;
		}
		ret = icc_node_set_name(data->nodes[i], provider, names[i]);
		if (ret) {
			icc_node_destroy(data->nodes[i]->id);
			goto nodes;
		}
		if (i == MASTER_DCAM)
			icc->master = data->nodes[i];
		icc_node_add(data->nodes[i], provider);
	}
	ret = icc_link_create(data->nodes[MASTER_DCAM], data->nodes[SLAVE_DDR]->id);
	if (ret)
		goto nodes;
	platform_set_drvdata(pdev, icc);
	ret = icc_provider_register(provider);
	if (!ret)
		return 0;
nodes:
	icc_nodes_remove(provider);
	return ret;
}

static const struct of_device_id sc9830_icc_match[] = {
	{ .compatible = "sprd,sc9830-dcam-interconnect" },
	{ }
};

static struct platform_driver sc9830_icc_driver = {
	.probe = sc9830_icc_probe,
	.driver = {
		.name = "sc9830-dcam-interconnect",
		.of_match_table = sc9830_icc_match,
		.sync_state = icc_sync_state,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sc9830_icc_driver);
