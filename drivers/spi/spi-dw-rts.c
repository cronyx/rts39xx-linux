/*
 * SSI interface driver for DW SPI Core
 *
 * Copyright (c) 2018, Realtek semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_rts"

struct dw_spi_rts {
	struct dw_spi  dws;
	struct clk     *clk;
};

static int dw_spi_rts_probe(struct platform_device *pdev)
{
	struct dw_spi_rts *dwsrts;
	struct dw_spi *dws;
	struct resource *mem;
	int ret;
	int num_cs;

	dwsrts = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_rts),
			GFP_KERNEL);
	if (!dwsrts)
		return -ENOMEM;

	dws = &dwsrts->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return dws->irq; /* -ENXIO */
	}

	dwsrts->clk = devm_clk_get(&pdev->dev, "xb2_ck");
	if (IS_ERR(dwsrts->clk))
		return PTR_ERR(dwsrts->clk);
	ret = clk_prepare_enable(dwsrts->clk);
	if (ret)
		return ret;

	dws->bus_num = pdev->id;

	dws->max_freq = clk_get_rate(dwsrts->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width",
		&dws->reg_io_width);

	num_cs = 4;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	dws->num_cs = num_cs;

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, dwsrts);
	return 0;

out:
	clk_disable_unprepare(dwsrts->clk);
	return ret;
}

static int dw_spi_rts_remove(struct platform_device *pdev)
{
	struct dw_spi_rts *dwsrts = platform_get_drvdata(pdev);

	clk_disable_unprepare(dwsrts->clk);
	dw_spi_remove_host(&dwsrts->dws);

	return 0;
}

static const struct of_device_id dw_spi_rts_of_match[] = {
	{ .compatible = "realtek,dw-apb-ssi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_rts_of_match);

static struct platform_driver dw_spi_rts_driver = {
	.probe		= dw_spi_rts_probe,
	.remove		= dw_spi_rts_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_rts_of_match,
	},
};
module_platform_driver(dw_spi_rts_driver);

MODULE_AUTHOR("Keent <keent_zhuo@realsil.com.cn>");
MODULE_DESCRIPTION("SSI interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
