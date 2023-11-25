/*
 * Realtek Semiconductor Corp.
 *
 * Memory power control driver
 *
 * Copyright (C) 2017      Wei WANG (wei_wang@realsil.com.cn)
 */

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>

#include <dt-bindings/reset/rts-sysmem.h>

struct rts_sysmem_regs {
	u32 resvd[16];
	u32 sys_isp_mem_sd;
	u32 sys_video_mem_sd;
	u32 sys_mem_sd;
#define SDIO2_MEM_SD			0x30000
#define H265_MEM_SD			0x2000
#define NAND_MEM_SD			0x300
#define ETH_MEM_SD			0x80
#define CIPHER_MEM_SD			0x40
#define AUDIO_MEM_SD			0x20
#define H264_MEM_SD			0x18
#define U2DEV_MEM_SD			0x04
#define SDIO1_MEM_SD			0x03
};

struct rts_sysmem_data {
	struct mutex				lock;
	struct rts_sysmem_regs __iomem	*regs;
	struct reset_controller_dev	rcdev;
};

#define SYS_ISP_MEM_ALL_MASK	0xFFFFFFFF
#define SYS_VIDEO_MEM_ALL_MASK	0xFFFFFFFF

#define RTS_REG_SET(addr, mask)				\
do {							\
	u32 val;					\
	val = readl((addr));				\
	val |= (mask);					\
	writel(val, (addr));				\
} while (0)

#define RTS_REG_CLR(addr, mask)				\
do {							\
	u32 val;					\
	val = readl((addr));				\
	val &= ~(mask);					\
	writel(val, (addr));				\
} while (0)

static int rts_sysmem_assert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct rts_sysmem_data *rdata = container_of(rcdev,
						     struct rts_sysmem_data,
						     rcdev);

	struct rts_sysmem_regs *regs = rdata->regs;

	mutex_lock(&rdata->lock);

	switch (id) {
	case SYS_ISP_MEM:
		RTS_REG_CLR(&regs->sys_isp_mem_sd, SYS_ISP_MEM_ALL_MASK);
		break;

	case SYS_VIDEO_MEM:
		RTS_REG_CLR(&regs->sys_video_mem_sd, SYS_VIDEO_MEM_ALL_MASK);
		break;

	case SYS_MEM_SD_NAND_SPIC:
		RTS_REG_CLR(&regs->sys_mem_sd, NAND_MEM_SD);
		break;

	case SYS_MEM_SD_ETH:
		RTS_REG_CLR(&regs->sys_mem_sd, ETH_MEM_SD);
		break;

	case SYS_MEM_SD_CIPHER:
		RTS_REG_CLR(&regs->sys_mem_sd, CIPHER_MEM_SD);
		break;

	case SYS_MEM_SD_AUDIO:
		RTS_REG_CLR(&regs->sys_mem_sd, AUDIO_MEM_SD);
		break;

	case SYS_MEM_SD_H264:
		RTS_REG_CLR(&regs->sys_mem_sd, H264_MEM_SD);
		break;

	case SYS_MEM_SD_H265:
		RTS_REG_CLR(&regs->sys_mem_sd, H265_MEM_SD);
		break;

	case SYS_MEM_SD_U2DEV:
		RTS_REG_CLR(&regs->sys_mem_sd, U2DEV_MEM_SD);
		break;

	case SYS_MEM_SD_SDIO1:
		RTS_REG_CLR(&regs->sys_mem_sd, SDIO1_MEM_SD);
		break;

	case SYS_MEM_SD_SDIO2:
		RTS_REG_CLR(&regs->sys_mem_sd, SDIO2_MEM_SD);
		break;

	default:
		pr_info("ERROR: invalid sys mem id %ld\n", id);
		break;
	}

	mutex_unlock(&rdata->lock);

	return 0;
}

static int rts_sysmem_deassert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct rts_sysmem_data *rdata = container_of(rcdev,
						     struct rts_sysmem_data,
						     rcdev);

	struct rts_sysmem_regs *regs = rdata->regs;

	mutex_lock(&rdata->lock);

	switch (id) {
	case SYS_ISP_MEM:
		RTS_REG_SET(&regs->sys_isp_mem_sd, SYS_ISP_MEM_ALL_MASK);
		break;

	case SYS_VIDEO_MEM:
		RTS_REG_SET(&regs->sys_video_mem_sd, SYS_VIDEO_MEM_ALL_MASK);
		break;

	case SYS_MEM_SD_NAND_SPIC:
		RTS_REG_SET(&regs->sys_mem_sd, NAND_MEM_SD);
		break;

	case SYS_MEM_SD_ETH:
		RTS_REG_SET(&regs->sys_mem_sd, ETH_MEM_SD);
		break;

	case SYS_MEM_SD_CIPHER:
		RTS_REG_SET(&regs->sys_mem_sd, CIPHER_MEM_SD);
		break;

	case SYS_MEM_SD_AUDIO:
		RTS_REG_SET(&regs->sys_mem_sd, AUDIO_MEM_SD);
		break;

	case SYS_MEM_SD_H264:
		RTS_REG_SET(&regs->sys_mem_sd, H264_MEM_SD);
		break;

	case SYS_MEM_SD_H265:
		RTS_REG_SET(&regs->sys_mem_sd, H265_MEM_SD);
		break;

	case SYS_MEM_SD_U2DEV:
		RTS_REG_SET(&regs->sys_mem_sd, U2DEV_MEM_SD);
		break;

	case SYS_MEM_SD_SDIO1:
		RTS_REG_SET(&regs->sys_mem_sd, SDIO1_MEM_SD);
		break;

	case SYS_MEM_SD_SDIO2:
		RTS_REG_SET(&regs->sys_mem_sd, SDIO2_MEM_SD);
		break;

	default:
		pr_info("ERROR: invalid sys mem id %ld\n", id);
		break;
	}

	mutex_unlock(&rdata->lock);

	return 0;
}

static const struct reset_control_ops rlx_sysmem_ops = {
	.assert		= rts_sysmem_assert,
	.deassert	= rts_sysmem_deassert,
};

static int rts_sysmem_probe(struct platform_device *pdev)
{
	struct rts_sysmem_data *rdata;
	struct resource *res;

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	mutex_init(&rdata->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rdata->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rdata->regs))
		return PTR_ERR(rdata->regs);

	rdata->rcdev.owner = THIS_MODULE;
	rdata->rcdev.nr_resets = SYS_MEM_MAX;
	rdata->rcdev.ops = &rlx_sysmem_ops;
	rdata->rcdev.of_node = pdev->dev.of_node;

	return devm_reset_controller_register(&pdev->dev, &rdata->rcdev);
}

static const struct of_device_id rts_sysmem_dt_ids[] = {
	 { .compatible = "realtek,rts3903-sysmem", },
	 { /* sentinel */ },
};

static struct platform_driver rts_sysmem_driver = {
	.probe	= rts_sysmem_probe,
	.driver = {
		.name		= "rts-sysmem",
		.of_match_table	= of_match_ptr(rts_sysmem_dt_ids),
	},
};

builtin_platform_driver(rts_sysmem_driver);
