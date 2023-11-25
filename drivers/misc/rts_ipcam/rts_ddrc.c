/* Driver for Realtek controller ddr
 *
 * Copyright(c) 2014 Realtek Semiconductor Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author:
 *   Keent <keent_zhuo@realsil.com.cn>
 *   No. 128, West Shenhu Road, Suzhou Industry Park, Suzhou, China
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/cdev.h>
#include "rts_ddrc.h"

/* DDR controller register offsets */
#define DRR					0x0010

/*DDR phy register offset*/
#define SSC2			0x24
#define SSC3			0x28

/* DRR bits */
#define tRFC	0
#define tREF	8

#define REF_PLL					25000000

struct rts_ddrc {
	struct device				*dev;
	void __iomem		*cregs;	/*ddr controller register*/
	void __iomem		*pregs;	/*ddr phy register*/
	u32 tref;
};

static ssize_t ddrc_tref_show(struct device *_dev,
			    struct device_attribute *attr, char *buf)
{
	u32 tref = 0;

	struct rts_ddrc *ddrc = dev_get_drvdata(_dev);

	if (ddrc->dev == _dev)
		tref = ddrc->tref;

	return sprintf(buf, "%d\n", tref);
}

DEVICE_ATTR(ddrc_tref, 0644, ddrc_tref_show, NULL);

static const struct attribute *ddrc_attrs[] = {
	&dev_attr_ddrc_tref.attr,
	NULL,
};

static const struct attribute_group ddrc_attr_group = {
	.attrs = (struct attribute **) ddrc_attrs,
};

static inline u32 rts_readl_c(struct rts_ddrc *rddr, u32 reg)
{
	return readl(rddr->cregs + reg);
}

static inline void rts_writel_c(struct rts_ddrc *rddr, u32 reg, u32 val)
{
	writel(val, rddr->cregs + reg);
}

static inline u32 rts_readl_p(struct rts_ddrc *rddr, u32 reg)
{
	return readl(rddr->pregs + reg);
}

static inline void rts_writel_p(struct rts_ddrc *rddr, u32 reg, u32 val)
{
	writel(val, rddr->pregs + reg);
}

static u64 integer_div(u64 dividend, u64 divisor)
{
	u64 res;

	res = 0;

	if (divisor == 0) {
		pr_err("wrong divisor\n");
		return 0;
	}

	if (dividend < divisor)
		return 0;

	while (dividend > divisor) {
		dividend -= divisor;
		res++;
	}

	if (dividend == divisor)
		res++;

	return res;
}

static int get_dram_clock(struct rts_ddrc *rddr)
{

	u64 dpi_n_code, dpi_f_code;
	u64 clock;

	dpi_f_code = rts_readl_p(rddr, SSC2) & 0x7ff;
	dpi_n_code = rts_readl_p(rddr, SSC3) & 0xff;

	clock = integer_div(REF_PLL * dpi_f_code * 2, 2048);

	clock = integer_div((clock + REF_PLL * (dpi_n_code + 2) * 2 + 500000),
			    1000000);
	return (int)clock;

}

static struct rts_ddrc *_rts_ddrc;

static int ddrc_notifier(struct notifier_block *nb, unsigned long event,
void *data)
{
	int retval = 0;
	int ddr_clock;
	int pctl_period_ps;
	u32 drr_val_temp;
	u32 drr_tref_val_temp;

	/*get ddr clock*/
	ddr_clock = get_dram_clock(_rts_ddrc);

	switch (ddr_clock) {
	case 800:
		pctl_period_ps = 5000;
		break;
	case 1066:
		pctl_period_ps = 3750;
		break;
	case 1333:
		pctl_period_ps = 3000;
		break;
	case 1600:
		pctl_period_ps = 2500;
		break;
	default:
		retval = -EINVAL;
		break;
	}

	drr_val_temp = rts_readl_c(_rts_ddrc, DRR);
	drr_tref_val_temp = ((u16)(drr_val_temp >> 8) + 0x100) * pctl_period_ps;

	_rts_ddrc->tref = drr_tref_val_temp;

	/*low event and current value is high, need to change*/
	if ((event == 0) && (_rts_ddrc->tref == 30000000)) {
		dev_info(_rts_ddrc->dev, "low event and current value is high\n");
		_rts_ddrc->tref = 60000000;
		drr_val_temp = (drr_val_temp & 0xff0000ff) |
			((_rts_ddrc->tref / pctl_period_ps - 0x100) << 8);
		rts_writel_c(_rts_ddrc, DRR, drr_val_temp);

		return retval;
	}
	/*high event and current value is low, need to change*/
	if ((event == 1) && (_rts_ddrc->tref == 60000000)) {
		dev_info(_rts_ddrc->dev, "high event and current value is low\n");
		_rts_ddrc->tref = 30000000;
		drr_val_temp = (drr_val_temp & 0xff0000ff) |
		((_rts_ddrc->tref / pctl_period_ps - 0x100) << 8);
		rts_writel_c(_rts_ddrc, DRR, drr_val_temp);

		return retval;
	}

	return retval;
}

static struct notifier_block ddrc_nb = {
	.notifier_call = ddrc_notifier, }
;

static const struct of_device_id rts_ddrc_dt_ids[] = {
	{ .compatible = "realtek,rts3903-ddrc", },
	{ }
};
MODULE_DEVICE_TABLE(of, rts_ddrc_dt_ids);

static int rts_ddrc_probe(struct platform_device *pdev)
{
	struct rts_ddrc *rddr;
	struct resource *res;
	int err;

	rddr = devm_kzalloc(&pdev->dev, sizeof(*rddr), GFP_KERNEL);
	if (!rddr)
		return -ENOMEM;

	rddr->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource provided");
		return -ENXIO;
	}

	rddr->cregs = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(rddr->cregs))
		return PTR_ERR(rddr->cregs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource provided");
		return -ENXIO;
	}

	rddr->pregs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rddr->pregs))
		return PTR_ERR(rddr->pregs);

	platform_set_drvdata(pdev, rddr);

	err = sysfs_create_group(&rddr->dev->kobj,
			&ddrc_attr_group);
	if (err < 0)
		return err;

	 thermal_event_register(&ddrc_nb);

	_rts_ddrc = rddr;

	return 0;
}

static int rts_ddrc_remove(struct platform_device *pdev)
{

	sysfs_remove_group(&pdev->dev.kobj, &ddrc_attr_group);

	_rts_ddrc = NULL;

	return 0;
}

static struct platform_driver rts_ddrc_driver = {
	.driver = {
		.name = "ddrc_platform",
		.of_match_table = rts_ddrc_dt_ids,
	},
	.probe = rts_ddrc_probe,
	.remove = rts_ddrc_remove,
};
module_platform_driver(rts_ddrc_driver);
