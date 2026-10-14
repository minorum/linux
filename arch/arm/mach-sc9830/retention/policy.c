// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/io.h>

struct sc9830_retention_policy {
	void __iomem *pmu;
	void __iomem *dmc;
	void __iomem *phy;
	u32 wake_por;
	u32 dmc_mode;
	bool saved;
};

static int sc9830_retention_policy_restore(struct sc9830_retention_policy *policy)
{
	u32 value;
	int ret = 0;

	if (!policy->saved)
		return 0;
	value = readl(policy->dmc + 0x30);
	writel((value & ~3U) | policy->dmc_mode, policy->dmc + 0x30);
	if ((readl(policy->dmc + 0x30) & 3) != policy->dmc_mode)
		ret = -EIO;
	writel(policy->wake_por, policy->pmu + 0x60);
	if (readl(policy->pmu + 0x60) != policy->wake_por)
		ret = -EIO;
	if (!ret)
		policy->saved = false;
	return ret;
}

static int sc9830_retention_policy_prepare(struct sc9830_retention_policy *policy)
{
	u32 value, state;
	unsigned int i;

	if (policy->saved)
		return -EBUSY;
	for (i = 0; i < 3; i++) {
		state = readl(policy->pmu + 0xbc);
		if ((state >> 28) != 7)
			return -EBUSY;
	}
	value = readl(policy->pmu + 0xc4);
	state = readl(policy->pmu + 0xc4);
	if ((value | state) & 0x0f000000 || !(readl(policy->phy + 0x184) & 1))
		return -EBUSY;
	value = readl(policy->dmc + 0x30);
	if ((value & 3) != 2)
		return -EBUSY;
	policy->dmc_mode = value & 3;
	policy->wake_por = readl(policy->pmu + 0x60);
	policy->saved = true;
	writel(1, policy->pmu + 0x60);
	if (readl(policy->pmu + 0x60) != 1)
		goto rollback;
	writel((value & ~3U) | 1, policy->dmc + 0x30);
	if (readl(policy->dmc + 0x30) != ((value & ~3U) | 1))
		goto rollback;
	return 0;
rollback:
	sc9830_retention_policy_restore(policy);
	return -EIO;
}
