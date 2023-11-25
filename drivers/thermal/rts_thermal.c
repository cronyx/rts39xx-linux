/* Driver for Realtek thermal
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
 *   Peter Sun <peter_sun@realsil.com.cn>
 *   No. 128, West Shenhu Road, Suzhou Industry Park, Suzhou, China
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#define SYS_TM_CTRL 0x0
#define SYS_TM_CFG0 0x4
#define SYS_TM_CFG1 0x8
#define SYS_TM_CFG2 0xc
#define SYS_TM_CFG3 0x10
#define SYS_TM_CFG4 0x14
#define SYS_TM_CFG5 0x18
#define SYS_TM_CFG6 0x1c
#define SYS_TM_OUT 0x20
#define SYS_TM_DEBUG 0x24
#define SYS_TM_OUT_D 0x28
#define SYS_TM_CT_HIGH 0x2c
#define SYS_TM_CT_LOW 0x30
#define SYS_TM_CT_INT_EN 0x34
#define SYS_TM_CT_INT_FLAG 0x38

#define RTS_HYSTERESIS 500

#define CENTIGRADE_TO_K(temp) (temp + 27315)
#define K_TO_CENTIGRADE(temp) (temp - 27315)

enum {
	TYPE_RLE0745 = 1,
	TYPE_RTS3901 = 2,
	TYPE_RTS3903 = 3,

	TYPE_FPGA = (1 << 16),
};

enum ddr_state {
	DDR_STATUS_LOW,
	DDR_STATUS_HIGH,
};

#define RTS_SOC_CAM_HW_ID(type)		((type) & 0xff)

struct blocking_notifier_head ddrc_trigger;

struct rts_trip_point {
	unsigned long temp;
	enum thermal_trip_type type;
};

struct rts_thsens_platform_data {
	struct rts_trip_point trip_points[THERMAL_MAX_TRIPS];
	int num_trips;
};

struct rts_thermal_zone {
	struct thermal_zone_device *therm_dev;
	struct delayed_work therm_work;
	struct work_struct init_work;
	struct rts_thsens_platform_data *trip_tab;
	unsigned int cur_index;
	void __iomem *base;
	u32 thermal_k[18];
	int devtype;
};

static u32 actual_k[] = {
	26815, 27315, 27815, 28315, 28815, 29315, 29815, 30315,
	30815, 31315, 31815, 32315, 32815, 33315, 33815, 34315,
	34815, 35315
};

static inline uint32_t rts_thermal_reg_read(struct rts_thermal_zone *zone,
	uint32_t reg)
{
	return readl(zone->base + reg);
}


static inline void rts_thermal_reg_write(struct rts_thermal_zone *zone,
	uint32_t reg, uint32_t val)
{
	writel(val, zone->base + reg);
}

int thermal_event_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ddrc_trigger, nb);
}
EXPORT_SYMBOL(thermal_event_register);

u32 temp2code(struct rts_thermal_zone *zone, u32 actk)
{
	int i;
	int x, x1, y1, x2, y2;
	u32 thk;

	for (i = 0; i < ARRAY_SIZE(actual_k)-1; i++)
		if (actk <= actual_k[i])
			break;

	if (i > 0)
		i--;

	x = actk;
	x1 = actual_k[i];
	y1 = zone->thermal_k[i];
	x2 = actual_k[i+1];
	y2 = zone->thermal_k[i+1];
	thk = (x-x1)*(y2-y1)/(x2-x1) + y1;
	if (RTS_SOC_CAM_HW_ID(zone->devtype) == TYPE_RTS3903)
		thk -= 20000;

	return	thk;
}

u32 code2temp(struct rts_thermal_zone *zone, u32 code)
{
	int i;
	int x, x1, y1, x2, y2;
	u32 thk;

	if (RTS_SOC_CAM_HW_ID(zone->devtype) == TYPE_RTS3903)
		code += 20000;

	thk = code;

	for (i = 0; i < ARRAY_SIZE(zone->thermal_k)-1; i++)
		if (thk <= zone->thermal_k[i])
			break;

	if (i > 0)
		i--;

	x = thk;
	x1 = zone->thermal_k[i];
	y1 = actual_k[i];
	x2 = zone->thermal_k[i+1];
	y2 = actual_k[i+1];

	thk = (x-x1)*(y2-y1)/(x2-x1) + y1;

	return	thk;
}


/* Callback to get current temperature */
static int rts_sys_get_temp(struct thermal_zone_device *thermal,
		int *temp)
{
	struct rts_thermal_zone *pzone = thermal->devdata;
	unsigned long code = rts_thermal_reg_read(pzone, SYS_TM_OUT);

	*temp = code2temp(pzone, code);

	return 0;
}

/* Callback to get trip point type */
static int rts_sys_get_trip_type(struct thermal_zone_device *thermal,
		int trip, enum thermal_trip_type *type)
{
	struct rts_thermal_zone *pzone = thermal->devdata;
	struct rts_thsens_platform_data *ptrips = pzone->trip_tab;

	if (trip >= ptrips->num_trips)
		return -EINVAL;

	*type = ptrips->trip_points[trip].type;

	return 0;
}

/* Callback to get trip point temperature */
static int rts_sys_get_trip_temp(struct thermal_zone_device *thermal,
		int trip, int *temp)
{
	struct rts_thermal_zone *pzone = thermal->devdata;
	struct rts_thsens_platform_data *ptrips = pzone->trip_tab;

	if (trip >= ptrips->num_trips)
		return -EINVAL;

	*temp = ptrips->trip_points[trip].temp;

	return 0;
}

/* Callback to get critical trip point temperature */
static int rts_sys_get_crit_temp(struct thermal_zone_device *thermal,
		int *temp)
{

	struct rts_thermal_zone *pzone = thermal->devdata;
	struct rts_thsens_platform_data *ptrips = pzone->trip_tab;
	int i;

	for (i = ptrips->num_trips - 1; i >= 0; i--) {
		if (ptrips->trip_points[i].type == THERMAL_TRIP_CRITICAL) {
			*temp = ptrips->trip_points[i].temp;
			return 0;
		}
	}

	return -EINVAL;
}

static struct thermal_zone_device_ops thdev_ops = {
	.get_temp = rts_sys_get_temp,
	.get_trip_type = rts_sys_get_trip_type,
	.get_trip_temp = rts_sys_get_trip_temp,
	.get_crit_temp = rts_sys_get_crit_temp,
};

static void rts_thermal_update_config(struct rts_thermal_zone *pzone,
		unsigned int idx)
{
	unsigned int dft_low, dft_high;

	pzone->cur_index = idx;

	dft_low = temp2code(pzone, pzone->trip_tab->trip_points[idx].temp);
	dft_high = temp2code(pzone, pzone->trip_tab->trip_points[idx+1].temp);

	dft_low = (dft_low > RTS_HYSTERESIS) ? (dft_low - RTS_HYSTERESIS) : 0;
	dft_high += RTS_HYSTERESIS;

	rts_thermal_reg_write(pzone, SYS_TM_CT_LOW, dft_low);
	rts_thermal_reg_write(pzone, SYS_TM_CT_HIGH, dft_high);
	rts_thermal_reg_write(pzone, SYS_TM_CT_INT_FLAG, 1);

}

static irqreturn_t rts_thermal_irq_handler(int irq, void *irq_data)
{
	struct rts_thermal_zone *pzone = irq_data;
	struct rts_thsens_platform_data *ptrips = pzone->trip_tab;
	unsigned int idx = pzone->cur_index;
	unsigned int code;
	int flags;
	u32 highc, lowc;

	flags = rts_thermal_reg_read(pzone, SYS_TM_CT_INT_FLAG);
	if (flags == 0)
		return IRQ_NONE;

	flags = rts_thermal_reg_read(pzone, SYS_TM_CT_INT_EN);
	if (flags == 0)
		return IRQ_NONE;

	rts_thermal_reg_write(pzone, SYS_TM_CT_INT_EN, 0);
	rts_thermal_reg_write(pzone, SYS_TM_CT_INT_FLAG, 1);

	if (idx < ptrips->num_trips - 1) {

		code = rts_thermal_reg_read(pzone, SYS_TM_OUT);

		dev_dbg(&pzone->therm_dev->device,
			"current code is 0x%x\n", code);

		lowc = pzone->trip_tab->trip_points[idx].temp;
		highc = pzone->trip_tab->trip_points[idx+1].temp;

		if (code < lowc) {
			if (idx > 0)
				idx -= 1;
		} else if (code > highc) {
			if (idx < ptrips->num_trips - 2)
				idx++;
		}

		rts_thermal_update_config(pzone, idx);

		dev_dbg(&pzone->therm_dev->device,
			"thermal set to idx %d\n", idx);

	}

	schedule_delayed_work(&pzone->therm_work,
	      msecs_to_jiffies(300));

	return IRQ_HANDLED;
}

static void rts_ddrc_notify(struct rts_thermal_zone *pzone)
{
	int events;
	unsigned long code = rts_thermal_reg_read(pzone, SYS_TM_OUT);

	code = code2temp(pzone, code);

	if (code < pzone->trip_tab->trip_points[0].temp)
		events =  DDR_STATUS_LOW;
	else if (code > pzone->trip_tab->trip_points[1].temp)
		events =  DDR_STATUS_HIGH;
	else
		return;

	blocking_notifier_call_chain(&ddrc_trigger, events, NULL);
}

static void rts_thermal_work(struct work_struct *work)
{
	struct rts_thermal_zone *pzone;

	pzone = container_of((struct delayed_work *)work,
		struct rts_thermal_zone, therm_work);

	thermal_zone_device_update(pzone->therm_dev, THERMAL_EVENT_UNSPECIFIED);
	dev_dbg(&pzone->therm_dev->device, "thermal work finished\n");

	rts_ddrc_notify(pzone);

	rts_thermal_reg_write(pzone, SYS_TM_CT_INT_EN, 1);
}

static void rts_init_thermal(struct work_struct *work)
{
	struct rts_thermal_zone *pzone;

	pzone = container_of(work, struct rts_thermal_zone, init_work);

	if (RTS_SOC_CAM_HW_ID(pzone->devtype) != TYPE_RTS3903) {
		rts_thermal_reg_write(pzone, SYS_TM_CFG5, 0x20);
		rts_thermal_reg_write(pzone, SYS_TM_CFG6, 0x5e39ac);
		rts_thermal_reg_write(pzone, SYS_TM_CTRL, 0x36);
		rts_thermal_reg_write(pzone, SYS_TM_CTRL, 0x37);
	} else {
		rts_thermal_reg_write(pzone, SYS_TM_CTRL, 0x17);
	}

	mdelay(200);

	rts_thermal_update_config(pzone, 0);
	rts_thermal_reg_write(pzone, SYS_TM_CT_INT_EN, 1);
}

static const struct of_device_id rlx_tm_match[] = {
	{
		.compatible = "realtek,rle0745-fpga-tm",
		.data = (void *)(TYPE_RLE0745 | TYPE_FPGA),
	}, {
		.compatible = "realtek,rlx0745-tm",
		.data = (void *)(TYPE_RLE0745),
	}, {
		.compatible = "realtek,rts3901-fpga-tm",
		.data = (void *)(TYPE_RTS3901 | TYPE_FPGA),
	}, {
		.compatible = "realtek,rts3901-tm",
		.data = (void *)(TYPE_RTS3901),
	}, {
		.compatible = "realtek,rts3903-fpga-tm",
		.data = (void *)(TYPE_RTS3903 | TYPE_FPGA),
	}, {
		.compatible = "realtek,rts3903-tm",
		.data = (void *)(TYPE_RTS3903),
	},
	{}
};
MODULE_DEVICE_TABLE(of, rlx_tm_match);

static struct rts_thsens_platform_data*
		rts_thermal_parse_dt(struct platform_device *pdev)
{
	struct rts_thsens_platform_data *ptrips;
	struct device_node *np = pdev->dev.of_node;
	char prop_name[32];
	s32 tmp_data;
	int i;

	ptrips = devm_kzalloc(&pdev->dev, sizeof(*ptrips), GFP_KERNEL);
	if (!ptrips)
		return NULL;

	if (of_property_read_u32(np, "num-trips", &tmp_data))
		goto err_parse_dt;
	if (tmp_data > THERMAL_MAX_TRIPS)
		goto err_parse_dt;
	ptrips->num_trips = tmp_data;

	if (ptrips->num_trips == 0) {
		ptrips->trip_points[0].temp = CENTIGRADE_TO_K(0);
		ptrips->trip_points[0].type = THERMAL_TRIP_HOT;
		ptrips->trip_points[1].temp = CENTIGRADE_TO_K(80);
		ptrips->trip_points[1].type = THERMAL_TRIP_HOT;
	} else {
		for (i = 0; i < ptrips->num_trips; i++) {
			sprintf(prop_name, "trip%d-temp", i);

			if (of_property_read_s32(np, prop_name, &tmp_data))
				goto err_parse_dt;

			ptrips->trip_points[i].temp = CENTIGRADE_TO_K(tmp_data);
			ptrips->trip_points[i].type = THERMAL_TRIP_HOT;

		}
	}
	return ptrips;

err_parse_dt:
	dev_err(&pdev->dev, "Parsing device tree data error.\n");
	return NULL;
}

static int rts_thermal_probe(struct platform_device *pdev)
{
	struct rts_thermal_zone *pzone = NULL;
	struct resource *mem;
	struct rts_thsens_platform_data *ptrips;
	int irq;
	int ret;
	const struct of_device_id *of_id;

	of_id = of_match_device(rlx_tm_match, &pdev->dev);
	if (!of_id)
		return -ENODEV;

	pzone = devm_kzalloc(&pdev->dev, sizeof(*pzone), GFP_KERNEL);
	if (!pzone)
		return -ENOMEM;

	pzone->devtype = (int)of_id->data;

	of_property_read_u32_array(pdev->dev.of_node, "thermal_k",
					  pzone->thermal_k,
					  ARRAY_SIZE(pzone->thermal_k));
	ptrips = rts_thermal_parse_dt(pdev);
	if (!ptrips)
		return -EINVAL;

	pzone->trip_tab = ptrips;

	INIT_DELAYED_WORK(&pzone->therm_work, rts_thermal_work);
	INIT_WORK(&pzone->init_work, rts_init_thermal);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
	}

	BLOCKING_INIT_NOTIFIER_HEAD(&ddrc_trigger);

	pzone->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(pzone->base))
		return PTR_ERR(pzone->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Get IRQ failed.\n");
		ret = irq;
		goto failed;
	}
	ret = devm_request_irq(&pdev->dev, irq, rts_thermal_irq_handler,
		IRQF_SHARED, "rts_thermal", pzone);

	pzone->therm_dev = thermal_zone_device_register("rts_thermal_zone",
		ptrips->num_trips, 0, pzone, &thdev_ops, NULL, 0, 0);

	if (IS_ERR(pzone->therm_dev)) {
		dev_err(&pdev->dev, "Register thermal zone device failed.\n");
		ret = PTR_ERR(pzone->therm_dev);
		goto failed;
	}
	dev_info(&pzone->therm_dev->device, "Thermal zone device registered.\n");

	platform_set_drvdata(pdev, pzone);

	schedule_work(&pzone->init_work);
failed:

	return ret;
}

static int rts_thermal_remove(struct platform_device *pdev)
{
	struct rts_thermal_zone *pzone = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(pzone->therm_dev);
	cancel_delayed_work_sync(&pzone->therm_work);

	return 0;
}

static struct platform_driver rts_thermal_driver = {
	.driver = {
		.name = "rts-thermal",
		.of_match_table = rlx_tm_match,
	},
	.probe = rts_thermal_probe,
	.remove = rts_thermal_remove,
};

static int __init rts_thermal_init(void)
{
	return platform_driver_register(&rts_thermal_driver);
}
fs_initcall(rts_thermal_init);

static void __exit rts_thermal_exit(void)
{
	platform_driver_unregister(&rts_thermal_driver);
}
module_exit(rts_thermal_exit);
