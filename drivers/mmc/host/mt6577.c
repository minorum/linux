// SPDX-License-Identifier: GPL-2.0-only
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#define MSDC_CFG		0x00
#define CFG_MODE	BIT(0)
#define CFG_CKPDN	BIT(1)
#define CFG_RST		BIT(2)
#define CFG_PIO		BIT(3)
#define CFG_CKSTB	BIT(7)
#define CFG_CKDIV	GENMASK(15, 8)
#define CFG_CKMOD	GENMASK(17, 16)
#define MSDC_IOCON	0x04
#define IOCON_DDLSEL	BIT(3)
#define MSDC_PS		0x08
#define PS_CDEN		BIT(0)
#define PS_DAT0		BIT(16)
#define MSDC_INT	0x0c
#define INT_CMDRDY	BIT(8)
#define INT_CMDTMO	BIT(9)
#define INT_RSPCRCERR	BIT(10)
#define INT_XFER_COMPL	BIT(12)
#define INT_DATTMO	BIT(14)
#define INT_DATCRCERR	BIT(15)
#define CMD_EVENTS	(INT_CMDRDY | INT_CMDTMO | INT_RSPCRCERR)
#define MSDC_INTEN	0x10
#define MSDC_FIFOCS	0x14
#define FIFO_RXCNT	GENMASK(7, 0)
#define FIFO_TXCNT	GENMASK(23, 16)
#define FIFO_CLR		BIT(31)
#define MSDC_TXDATA	0x18
#define MSDC_RXDATA	0x1c
#define SDC_CFG		0x30
#define SDC_BUSWIDTH	GENMASK(17, 16)
#define SDC_DTOC		GENMASK(31, 24)
#define SDC_SDIO		BIT(19)
#define SDC_SDIOIDE	BIT(20)
#define SDC_CMD		0x34
#define CMD_RESPONSE	GENMASK(9, 7)
#define CMD_DATA_TYPE	GENMASK(12, 11)
#define CMD_WRITE	BIT(13)
#define CMD_STOP		BIT(14)
#define CMD_BLOCK_LEN	GENMASK(27, 16)
#define SDC_ARG		0x38
#define SDC_STS		0x3c
#define STS_CMDBUSY	BIT(1)
#define SDC_RESP0	0x40
#define SDC_BLK_NUM	0x50
#define MSDC_DMA_SA	0x90
#define MSDC_DMA_CTRL	0x98
#define DMA_START	BIT(0)
#define DMA_STOP		BIT(1)
#define DMA_LASTBUF	BIT(10)
#define DMA_BURST	GENMASK(14, 12)
#define DMA_SIZE		GENMASK(31, 16)
#define MSDC_DMA_CFG	0x9c
#define DMA_ACTIVE	BIT(0)
#define MSDC_PATCH0	0xb0
#define MSDC_PAD_CTRL0	0xe0
#define MSDC_PAD_CTRL1	0xe4
#define MSDC_PAD_CTRL2	0xe8
#define PAD_DRVN		GENMASK(2, 0)
#define PAD_DRVP		GENMASK(6, 4)
#define PAD_PD		BIT(16)
#define PAD_PU		BIT(17)
#define PAD_SMT		BIT(18)
#define PAD_IES		BIT(19)
#define MSDC_PAD_TUNE	0xec
#define MSDC_RDDLY0	0xf0
#define MSDC_RDDLY1	0xf4
#define MSDC_ECO_VER	0x104
#define FIFO_SIZE	128
#define MAX_REQUEST	32768

struct mt6577_host {
	void __iomem *base;
	unsigned long rate;
	u8 *buffer;
	dma_addr_t dma_addr;
	struct clk *clk;
	bool clock_enabled;
	bool dma_enabled;
	int clock_error;
};

static void mt6577_disable_clock(void *arg)
{
	struct mt6577_host *host = arg;

	if (host->clock_enabled) {
		clk_disable_unprepare(host->clk);
		host->clock_enabled = false;
	}
}

static int mt6577_reset(struct mt6577_host *host)
{
	u32 val;
	int ret;

	writel(readl(host->base + MSDC_CFG) | CFG_RST, host->base + MSDC_CFG);
	ret = readl_poll_timeout(host->base + MSDC_CFG, val,
				 !(val & CFG_RST), 1, 20000);
	if (ret)
		return ret;
	writel(FIFO_CLR, host->base + MSDC_FIFOCS);
	ret = readl_poll_timeout(host->base + MSDC_FIFOCS, val,
				 !(val & FIFO_CLR), 1, 20000);
	if (ret)
		return ret;
	writel(0, host->base + MSDC_INTEN);
	writel(readl(host->base + MSDC_INT), host->base + MSDC_INT);
	return 0;
}

static int mt6577_init_hw(struct mt6577_host *host)
{
	u32 val;
	int ret;

	if (readl(host->base + MSDC_ECO_VER) != 4)
		return -ENODEV;
	val = readl(host->base + MSDC_CFG) & ~CFG_CKPDN;
	writel(val | CFG_MODE | CFG_PIO, host->base + MSDC_CFG);
	ret = mt6577_reset(host);
	if (ret)
		return ret;
	val = PAD_SMT | PAD_IES | FIELD_PREP(PAD_DRVN, 4) |
		FIELD_PREP(PAD_DRVP, 4);
	writel(val | PAD_PD, host->base + MSDC_PAD_CTRL0);
	writel(val | PAD_PU, host->base + MSDC_PAD_CTRL1);
	writel(val | PAD_PU, host->base + MSDC_PAD_CTRL2);
	writel(readl(host->base + MSDC_PS) & ~PS_CDEN, host->base + MSDC_PS);
	writel(0, host->base + MSDC_PAD_TUNE);
	writel(0, host->base + MSDC_IOCON);
	writel(IOCON_DDLSEL, host->base + MSDC_IOCON);
	writel(0, host->base + MSDC_RDDLY0);
	writel(0, host->base + MSDC_RDDLY1);
	writel(0x403c004f, host->base + MSDC_PATCH0);
	val = readl(host->base + SDC_CFG) & ~(SDC_BUSWIDTH | SDC_SDIOIDE);
	writel(val | SDC_SDIO, host->base + SDC_CFG);
	return 0;
}

static int mt6577_command(struct mt6577_host *host, struct mmc_command *cmd,
			  struct mmc_data *data)
{
	u32 raw = cmd->opcode, response = 0, val;
	int ret, i;

	ret = readl_poll_timeout(host->base + SDC_STS, val,
				 !(val & STS_CMDBUSY), 1, 20000);
	if (ret)
		return ret;
	if (cmd->flags & MMC_RSP_136)
		response = 2;
	else if (cmd->flags & MMC_RSP_BUSY)
		response = 7;
	else if (cmd->flags & MMC_RSP_CRC)
		response = 1;
	else if (cmd->flags & MMC_RSP_PRESENT)
		response = 3;
	raw |= FIELD_PREP(CMD_RESPONSE, response);
	if (data) {
		raw |= FIELD_PREP(CMD_BLOCK_LEN, data->blksz) |
		       FIELD_PREP(CMD_DATA_TYPE, data->blocks > 1 ? 2 : 1);
		if (data->flags & MMC_DATA_WRITE)
			raw |= CMD_WRITE;
	}
	if (cmd->opcode == MMC_STOP_TRANSMISSION)
		raw |= CMD_STOP;
	writel(data ? data->blocks : 0, host->base + SDC_BLK_NUM);
	writel(cmd->arg, host->base + SDC_ARG);
	writel(raw, host->base + SDC_CMD);
	ret = readl_poll_timeout(host->base + MSDC_INT, val,
				 val & CMD_EVENTS, 1, 1000000);
	writel(val & CMD_EVENTS, host->base + MSDC_INT);
	if (ret || val & INT_CMDTMO)
		return -ETIMEDOUT;
	if (val & INT_RSPCRCERR)
		return -EILSEQ;
	if (cmd->flags & MMC_RSP_136) {
		for (i = 0; i < 4; i++)
			cmd->resp[i] = readl(host->base + SDC_RESP0 + (3 - i) * 4);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->base + SDC_RESP0);
	}
	if (cmd->flags & MMC_RSP_BUSY)
		return readl_poll_timeout(host->base + MSDC_PS, val,
					  val & PS_DAT0, 10, 1000000);
	return 0;
}

static int mt6577_transfer(struct mt6577_host *host, struct mmc_data *data)
{
	unsigned int remaining = data->blocks * data->blksz;
	bool write = data->flags & MMC_DATA_WRITE;
	ktime_t deadline = ktime_add_ms(ktime_get(), 1000);
	u8 *ptr = host->buffer;
	u32 status, count;

	for (;;) {
		status = readl(host->base + MSDC_INT);
		if (status & INT_DATTMO)
			return -ETIMEDOUT;
		if (status & INT_DATCRCERR)
			return -EILSEQ;
		count = readl(host->base + MSDC_FIFOCS);
		count = write ? FIELD_GET(FIFO_TXCNT, count) : FIELD_GET(FIFO_RXCNT, count);
		if (count > FIFO_SIZE)
			return -EIO;
		if (write)
			count = FIFO_SIZE - count;
		count = min(count, remaining);
		while (count >= 4) {
			if (write)
				writel(*(u32 *)ptr, host->base + MSDC_TXDATA);
			else
				*(u32 *)ptr = readl(host->base + MSDC_RXDATA);
			ptr += 4;
			remaining -= 4;
			count -= 4;
		}
		if (status & INT_XFER_COMPL) {
			writel(INT_XFER_COMPL, host->base + MSDC_INT);
			return remaining ? -EIO : 0;
		}
		if (ktime_after(ktime_get(), deadline))
			return -ETIMEDOUT;
		cpu_relax();
		cond_resched();
	}
}

static void mt6577_set_ios(struct mmc_host *mmc, struct mmc_ios *ios);

static int mt6577_dma_transfer(struct mt6577_host *host, struct mmc_data *data)
{
	u32 val, ctrl;
	int ret, stop;

	writel(readl(host->base + MSDC_CFG) & ~CFG_PIO, host->base + MSDC_CFG);
	writel(lower_32_bits(host->dma_addr), host->base + MSDC_DMA_SA);
	ctrl = FIELD_PREP(DMA_SIZE, data->blocks * data->blksz) |
	       FIELD_PREP(DMA_BURST, 6) | DMA_LASTBUF;
	dma_wmb();
	writel(ctrl | DMA_START, host->base + MSDC_DMA_CTRL);
	ret = readl_poll_timeout(host->base + MSDC_INT, val,
				 val & (INT_XFER_COMPL | INT_DATTMO | INT_DATCRCERR),
				 1, 1000000);
	if (val & INT_DATTMO)
		ret = -ETIMEDOUT;
	else if (val & INT_DATCRCERR)
		ret = -EILSEQ;
	writel(ctrl | DMA_STOP, host->base + MSDC_DMA_CTRL);
	stop = readl_poll_timeout(host->base + MSDC_DMA_CFG, val,
				  !(val & DMA_ACTIVE), 1, 20000);
	if (stop) {
		clk_disable_unprepare(host->clk);
		host->clock_enabled = false;
		host->clock_error = -EIO;
		host->dma_enabled = false;
		return stop;
	}
	dma_rmb();
	writel(readl(host->base + MSDC_CFG) | CFG_PIO, host->base + MSDC_CFG);
	if (ret)
		host->dma_enabled = false;
	return ret;
}

static void mt6577_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mt6577_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	unsigned int length = 0;
	bool dma = host->dma_enabled;
	int ret;

	if (host->clock_error) {
		mrq->cmd->error = host->clock_error;
		goto done;
	}
	if (data) {
		if (!data->blksz || !data->blocks || data->blksz % 4 ||
		    data->blksz > 512 || data->blocks > MAX_REQUEST / data->blksz) {
			mrq->cmd->error = -EINVAL;
			goto done;
		}
		length = data->blocks * data->blksz;
		if (data->flags & MMC_DATA_WRITE &&
		    sg_copy_to_buffer(data->sg, data->sg_len, host->buffer, length) != length) {
			mrq->cmd->error = -EINVAL;
			goto done;
		}
	}
	ret = mt6577_reset(host);
	if (ret) {
		mrq->cmd->error = ret;
		goto done;
	}
	if (mrq->sbc) {
		mrq->sbc->error = mt6577_command(host, mrq->sbc, NULL);
		if (mrq->sbc->error)
			goto done;
	}
	if (data && host->dma_enabled)
		writel(readl(host->base + MSDC_CFG) & ~CFG_PIO, host->base + MSDC_CFG);
	mrq->cmd->error = mt6577_command(host, mrq->cmd, data);
	if (mrq->cmd->error)
		goto done;
	if (data) {
		data->error = host->dma_enabled ? mt6577_dma_transfer(host, data) :
			      mt6577_transfer(host, data);
		if (!data->error && data->flags & MMC_DATA_READ &&
		    sg_copy_from_buffer(data->sg, data->sg_len, host->buffer, length) != length)
			data->error = -EINVAL;
		if (!data->error)
			data->bytes_xfered = length;
		if (mrq->stop && !host->clock_error)
			mrq->stop->error = mt6577_command(host, mrq->stop, NULL);
		if (!data->error && data->flags & MMC_DATA_WRITE)
			data->error = readl_poll_timeout(host->base + MSDC_PS, ret,
							 ret & PS_DAT0, 10, 1000000);
		if (data->error && dma && host->clock_enabled) {
			host->dma_enabled = false;
			mt6577_set_ios(mmc, &mmc->ios);
			dev_warn(mmc_dev(mmc), "DMA error %d, using PIO at %u Hz\n",
				 data->error, mmc->actual_clock);
		}
	}
done:
	mmc_request_done(mmc, mrq);
}

static void mt6577_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mt6577_host *host = mmc_priv(mmc);
	u32 cfg, val, div, clock;

	if (!host->clock_enabled)
		return;
	if ((ios->timing != MMC_TIMING_LEGACY && ios->timing != MMC_TIMING_MMC_HS) ||
	    ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330) {
		host->clock_error = -EINVAL;
		return;
	}
	val = readl(host->base + SDC_CFG) & ~SDC_BUSWIDTH;
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		val |= FIELD_PREP(SDC_BUSWIDTH, 1);
	writel(val | SDC_DTOC, host->base + SDC_CFG);
	cfg = readl(host->base + MSDC_CFG) & ~CFG_CKPDN;
	writel(cfg, host->base + MSDC_CFG);
	mmc->actual_clock = 0;
	if (!ios->clock)
		return;
	clock = host->dma_enabled ? ios->clock : min(ios->clock, 12000000U);
	div = DIV_ROUND_UP(host->rate, 4 * clock);
	if (!div || div > FIELD_MAX(CFG_CKDIV)) {
		host->clock_error = -EINVAL;
		return;
	}
	cfg &= ~(CFG_CKMOD | CFG_CKDIV);
	cfg |= CFG_MODE | CFG_PIO | FIELD_PREP(CFG_CKDIV, div);
	writel(cfg, host->base + MSDC_CFG);
	host->clock_error = readl_poll_timeout(host->base + MSDC_CFG, val,
					       val & CFG_CKSTB, 1, 1000000);
	if (host->clock_error)
		return;
	writel(cfg | CFG_CKPDN, host->base + MSDC_CFG);
	mmc->actual_clock = host->rate / (4 * div);
}

static const struct mmc_host_ops mt6577_ops = {
	.request = mt6577_request,
	.set_ios = mt6577_set_ios,
};

static int mt6577_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct mt6577_host *host;
	int ret;

	mmc = devm_mmc_alloc_host(&pdev->dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;
	host = mmc_priv(mmc);
	host->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->base))
		return PTR_ERR(host->base);
	host->clk = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(host->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(host->clk), "Missing source clock\n");
	ret = clk_prepare_enable(host->clk);
	if (ret)
		return ret;
	host->clock_enabled = true;
	ret = devm_add_action_or_reset(&pdev->dev, mt6577_disable_clock, host);
	if (ret)
		return ret;
	host->rate = clk_get_rate(host->clk);
	if (!host->rate)
		return -EINVAL;
	if (!dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)))
		host->buffer = dmam_alloc_coherent(&pdev->dev, MAX_REQUEST,
						   &host->dma_addr, GFP_KERNEL);
	host->dma_enabled = !!host->buffer;
	if (!host->buffer)
		host->buffer = devm_kmalloc(&pdev->dev, MAX_REQUEST, GFP_KERNEL);
	if (!host->buffer)
		return -ENOMEM;
	mmc->ops = &mt6577_ops;
	mmc->f_min = DIV_ROUND_UP(host->rate, 4 * 255);
	mmc->f_max = 52000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps2 = MMC_CAP2_NO_SD | MMC_CAP2_NO_SDIO;
	mmc->max_segs = 16;
	mmc->max_seg_size = MAX_REQUEST;
	mmc->max_req_size = MAX_REQUEST;
	mmc->max_blk_count = MAX_REQUEST / 512;
	mmc->max_blk_size = 512;
	mmc->max_busy_timeout = 1000;
	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;
	if (!(mmc->caps & MMC_CAP_NONREMOVABLE) || mmc->caps & MMC_CAP_8_BIT_DATA)
		return -EINVAL;
	mmc->f_max = min(mmc->f_max, 52000000U);
	ret = mt6577_init_hw(host);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Cannot initialize controller\n");
	platform_set_drvdata(pdev, mmc);
	dev_info(&pdev->dev, "%s transfers\n", host->dma_enabled ? "DMA" : "PIO");
	return mmc_add_host(mmc);
}

static void mt6577_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct mt6577_host *host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	if (host->clock_enabled)
		writel(0, host->base + MSDC_INTEN);
	mt6577_disable_clock(host);
}

static const struct of_device_id mt6577_match[] = {
	{ .compatible = "mediatek,mt6577-mmc" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6577_match);

static struct platform_driver mt6577_driver = {
	.probe = mt6577_probe,
	.remove = mt6577_remove,
	.driver = {
		.name = "mt6577-mmc",
		.of_match_table = mt6577_match,
	},
};
module_platform_driver(mt6577_driver);

MODULE_DESCRIPTION("MediaTek MT6577 eMMC host");
MODULE_LICENSE("GPL");
