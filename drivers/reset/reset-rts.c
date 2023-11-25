/*
 * Realtek Semiconductor Corp.
 *
 * force reset driver
 *
 * Copyright (C) 2014      Wei WANG (wei_wang@realsil.com.cn)
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>

#include <dt-bindings/reset/rts-resets.h>

#define FORCE_BUS_SD2_RESET		0x200
#define FORCE_BUS_VIDEO_RESET           0x100
#define FORCE_BUS_SD1_RESET              0x80
#define FORCE_BUS_I2S_RESET             0x40
#define FORCE_BUS_U2DEV_RESET           0x20
#define FORCE_BUS_U2HOST_RESET          0x10

#define FORCE_RTC32K_RESET              0x08
#define FORCE_U2DEV_UTMI_RESET          0x04
#define FORCE_U2HOST_UTMI_RESET         0x02
#define FORCE_MCU_CLK_RESET             0x01

#define FORCE_SD2_CLK_ASYNC_RESET	0x8000
#define FORCE_H265_CCLK_ASYNC_RESET	0x4000
#define FORCE_H265_BCLK_ASYNC_RESET	0x2000
#define FORCE_H265_ACLK_ASYNC_RESET	0x1000
#define FORCE_CODEC_CLK_ASYNC_RESET     0x800
#define FORCE_SPDIF_CLK_ASYNC_RESET     0x400
#define FORCE_H264_ASYNC_RESET          0x200
#define FORCE_I2C_CLK_ASYNC_RESET       0x100
#define FORCE_UART_CLK_ASYNC_RESET      0x080
#define FORCE_ETHERNET_CLK_ASYNC_RESET  0x040
#define FORCE_SD1_CLK_ASYNC_RESET        0x020
#define FORCE_CIPHER_CLK_ASYNC_RESET    0x010
#define FORCE_I2S_CLK_ASYNC_RESET       0x008
#define FORCE_ISP_CLK_ASYNC_RESET       0x004
#define FORCE_JPG_CLK_ASYNC_RESET       0x002
#define FORCE_MIPI_CLK_ASYNC_RESET      0x001

struct rts_force_reset_regs {
	u32 force_reg_reset;
	u32 force_reg_reset_fwc;
	u32 force_reg_async_reset;
};

struct rts_reset_data {
	struct mutex				lock;
	struct rts_force_reset_regs __iomem	*regs;
	struct reset_controller_dev	rcdev;
};

#define ALL_MASK	0xFFFFFFFF

#define RTS_FRR_SET(addr, mask)				\
do {							\
	u32 val;					\
	val = readl((addr));				\
	val |= (mask);					\
	writel(val, (addr));				\
} while (0)

#define RTS_FRR_CLR(addr, mask)				\
do {							\
	u32 val;					\
	val = readl((addr));				\
	val &= ~(mask);					\
	writel(val, (addr));				\
} while (0)

#define RTS_FORCE_RESET_AUTO(addr, mask)	RTS_FRR_SET(addr, mask)

#define RTS_FORCE_RESET(addr, mask)			\
do {							\
	RTS_FRR_SET(addr, mask);			\
	RTS_FRR_CLR(addr, mask);			\
} while (0)

static int rts_sys_force_reset(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct rts_reset_data *rdata = container_of(rcdev,
						     struct rts_reset_data,
						     rcdev);

	struct rts_force_reset_regs *regs = rdata->regs;

	mutex_lock(&rdata->lock);

	switch (id) {
	case FORCE_RESET_VIDEO:
		RTS_FORCE_RESET_AUTO(&regs->force_reg_reset,
			FORCE_BUS_VIDEO_RESET);
		break;

	case FORCE_RESET_H264:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H264_ASYNC_RESET);
		break;

	case FORCE_RESET_JPG:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_JPG_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_MIPI:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_MIPI_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_SDIO1:
		RTS_FORCE_RESET_AUTO(&regs->force_reg_reset,
			FORCE_BUS_SD1_RESET);
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_SD1_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_SDIO2:
		RTS_FORCE_RESET_AUTO(&regs->force_reg_reset,
			FORCE_BUS_SD2_RESET);
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_SD2_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_CIPHER:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_CIPHER_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_CODEC:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
				FORCE_CODEC_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_I2S:
		RTS_FORCE_RESET_AUTO(&regs->force_reg_reset,
			FORCE_BUS_I2S_RESET);
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
				FORCE_I2S_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_SPDIF:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
				FORCE_SPDIF_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_I2C:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_I2C_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_U2DEV:
		RTS_FORCE_RESET_AUTO(&regs->force_reg_reset,
			FORCE_BUS_U2DEV_RESET);
		RTS_FORCE_RESET(&regs->force_reg_reset_fwc,
			FORCE_U2DEV_UTMI_RESET);
		break;

	case FORCE_RESET_U2HOST:
		RTS_FORCE_RESET_AUTO(&regs->force_reg_reset,
			FORCE_BUS_U2HOST_RESET);
		RTS_FORCE_RESET(&regs->force_reg_reset_fwc,
			FORCE_U2HOST_UTMI_RESET);
		break;

	case FORCE_RESET_MCU:
		RTS_FORCE_RESET(&regs->force_reg_reset_fwc,
			FORCE_MCU_CLK_RESET);
		break;

	case FORCE_RESET_ISP:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_ISP_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_MCU_PREPARE:
		RTS_FRR_SET(&regs->force_reg_reset_fwc, FORCE_MCU_CLK_RESET);
		break;

	case FORCE_RESET_MCU_DONE:
		RTS_FRR_CLR(&regs->force_reg_reset_fwc, FORCE_MCU_CLK_RESET);
		break;

	case FORCE_RESET_UART:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_UART_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_ETHERNET:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_ETHERNET_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_AXI:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H265_ACLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_BPU:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H265_BCLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_CORE:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H265_CCLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265:
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H265_ACLK_ASYNC_RESET);
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H265_BCLK_ASYNC_RESET);
		RTS_FORCE_RESET(&regs->force_reg_async_reset,
			FORCE_H265_CCLK_ASYNC_RESET);
		break;

	default:
		pr_info("ERROR: invalid reset model %ld\n", id);
		break;
	}

	mutex_unlock(&rdata->lock);

	return 0;
}

static int rts_sys_reset_assert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct rts_reset_data *rdata = container_of(rcdev,
						     struct rts_reset_data,
						     rcdev);

	struct rts_force_reset_regs *regs = rdata->regs;

	mutex_lock(&rdata->lock);

	switch (id) {
	case FORCE_RESET_ETHERNET:
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_ETHERNET_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_AXI:
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_H265_ACLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_BPU:
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_H265_BCLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_CORE:
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_H265_CCLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265:
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_H265_ACLK_ASYNC_RESET);
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_H265_BCLK_ASYNC_RESET);
		RTS_FRR_CLR(&regs->force_reg_async_reset,
			FORCE_H265_CCLK_ASYNC_RESET);
		break;

	default:
		pr_info("ERROR: invalid assert model %ld\n", id);
		break;
	}

	mutex_unlock(&rdata->lock);

	return 0;
}

static int rts_sys_reset_deassert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct rts_reset_data *rdata = container_of(rcdev,
						     struct rts_reset_data,
						     rcdev);

	struct rts_force_reset_regs *regs = rdata->regs;

	mutex_lock(&rdata->lock);

	switch (id) {
	case FORCE_RESET_ETHERNET:
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_ETHERNET_CLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_AXI:
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_H265_ACLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_BPU:
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_H265_BCLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265_CORE:
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_H265_CCLK_ASYNC_RESET);
		break;

	case FORCE_RESET_H265:
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_H265_ACLK_ASYNC_RESET);
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_H265_BCLK_ASYNC_RESET);
		RTS_FRR_SET(&regs->force_reg_async_reset,
			FORCE_H265_CCLK_ASYNC_RESET);
		break;

	default:
		pr_info("ERROR: invalid deassert model %ld\n", id);
		break;
	}

	mutex_unlock(&rdata->lock);

	return 0;
}

static const struct reset_control_ops rlx_reset_ops = {
	.reset		= rts_sys_force_reset,
	.assert		= rts_sys_reset_assert,
	.deassert	= rts_sys_reset_deassert,
};

static void rts_force_reset_hw_init(struct rts_force_reset_regs *regs)
{
	RTS_FRR_SET(&regs->force_reg_reset_fwc, FORCE_U2DEV_UTMI_RESET |
		FORCE_U2HOST_UTMI_RESET | FORCE_MCU_CLK_RESET);
	RTS_FRR_SET(&regs->force_reg_async_reset, ALL_MASK &
		~(FORCE_UART_CLK_ASYNC_RESET | FORCE_I2C_CLK_ASYNC_RESET));
}

static int rts_reset_probe(struct platform_device *pdev)
{
	struct rts_reset_data *rdata;
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
	rdata->rcdev.nr_resets = FORCE_RESET_MAX;
	rdata->rcdev.ops = &rlx_reset_ops;
	rdata->rcdev.of_node = pdev->dev.of_node;

	rts_force_reset_hw_init(rdata->regs);

	return devm_reset_controller_register(&pdev->dev, &rdata->rcdev);
}

static int rts_reset_remove(struct platform_device *pdev)
{
	struct rts_reset_data *rdata = platform_get_drvdata(pdev);

	reset_controller_unregister(&rdata->rcdev);

	return 0;
}

static const struct of_device_id rts_reset_dt_ids[] = {
	 { .compatible = "realtek,rts3903-reset", },
	 { /* sentinel */ },
};

static struct platform_driver rts_reset_driver = {
	.probe	= rts_reset_probe,
	.remove	= rts_reset_remove,
	.driver = {
		.name		= "rts-reset",
		.of_match_table	= of_match_ptr(rts_reset_dt_ids),
	},
};

builtin_platform_driver(rts_reset_driver);

