#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clocksource.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/rts_xb2.h>

#define RTS3901_RTC_EN 0
#define RTS3901_RTC_TIME_LOAD 0x4
#define RTS3901_RTC_DATE_LOAD 0x8
#define RTS3901_RTC_LOAD 0xc
#define RTS3901_RTC_ALARM_EN 0x10
#define RTS3901_RTC_ALARM0_TIME 0x0014
#define RTS3901_RTC_ALARM0_DATE 0x0018
#define RTS3901_RTC_ALARM1_TIME 0x001C
#define RTS3901_RTC_ALARM1_DATE 0x0020
#define RTS3901_RTC_ALARM2_TIME 0x0024
#define RTS3901_RTC_ALARM2_DATE 0x0028
#define RTS3901_RTC_ALARM3_TIME 0x002C
#define RTS3901_RTC_ALARM3_DATE 0x0030
#define RTS3901_RTC_TIME_NOW 0x0034
#define RTS3901_RTC_DATE_NOW 0x0038
#define RTS3901_RTC_LEAP_YEAR 0x003C

#define RTS3903_RTC_CFG_EN	0x0000
#define RTS3903_RTC_CNT_RST	0x0004
#define RTS3903_RTC_CNT_SET	0x0008
#define RTS3903_RTC_CLK32K_SEL 0x000C
#define RTS3903_RTC_XTAL_CFG	0x0010
#define RTS3903_RTC_XTAL_STATE	0x0014
#define RTS3903_RTC_C_CNT		0x0018
#define RTS3903_RTC_C_CNT_SYNC	0x001C
#define RTS3903_RTC_S_CNT		0x0020
#define RTS3903_RTC_SAMPLE_CTRL	0x0024
#define RTS3903_RTC_ALARM_EN	0x0040
#define RTS3903_RTC_ALARM0_SECOND	0x0044
#define RTS3903_RTC_ALARM1_SECOND	0x0048
#define RTS3903_RTC_ALARM2_SECOND	0x004C
#define RTS3903_RTC_ALARM3_SECOND	0x0050

#define ALARM0_ENABLE 0
#define ALARM1_ENABLE 1
#define ALARM2_ENABLE 2
#define ALARM3_ENABLE 3

#define SEC_VALUE(x) (x & 0x3f)
#define MIN_VALUE(x) ((x & 0x3f00) >> 8)
#define HOUR_VALUE(x) ((x & 0x1f0000) >> 16)
#define WEEK_VALUE(x) ((x & 0x7000000) >> 24)

#define DAY_VALUE(x) (x & 0x1f)
#define MON_VALUE(x) ((x & 0xf00) >> 8)
#define YEAR_VALUE(x) ((x & 0x7f0000) >> 16)
#define CEN_VALUE(x) ((x & 0x7f000000) >> 24)

#define TIME_REG(weekday, hour, min, sec) (((weekday & 0x7f)<<24) | \
	((hour&0x1f) << 16) | ((min & 0x3f) << 8) | (sec & 0x3f))

#define DATE_REG(cen, year, mon, day) (((cen & 0x7f)<<24) | \
	((year & 0x7f) << 16) | ((mon & 0xf) << 8) | (day & 0x1f))

enum {
	TYPE_RLE0745 = 1,
	TYPE_RTS3901 = 2,
	TYPE_RTS3903 = 3,

	TYPE_FPGA = (1 << 16),
};

#define RTS_SOC_CAM_HW_ID(type)		((type) & 0xff)

struct rts_rtc {
	struct resource *mem;
	void __iomem *base;
	struct rtc_device *rtc;
	int irq;
	spinlock_t lock;
	int devtype;
	int xtal_flag;
};

int rts_set_time_xtal(struct rts_rtc *rtc, u32 sec)
{
	writel(1, rtc->base + RTS3903_RTC_CFG_EN);
	msleep(500);

	writel(1, rtc->base + RTS3903_RTC_CLK32K_SEL);
	writel(1, rtc->base + RTS3903_RTC_CNT_RST);
	writel(0, rtc->base + RTS3903_RTC_CNT_RST);

	writel(sec, rtc->base + RTS3903_RTC_CNT_SET);
	writel(0, rtc->base + RTS3903_RTC_CNT_SET);
	writel(0, rtc->base + RTS3903_RTC_CLK32K_SEL);
	writel(0, rtc->base + RTS3903_RTC_CFG_EN);

	return 0;
}

int rts_set_time_nonxtal(struct rts_rtc *rtc, u32 sec)
{
	writel(1, rtc->base + RTS3903_RTC_CFG_EN);
	msleep(500);

	writel(0, rtc->base + RTS3903_RTC_CLK32K_SEL);
	writel(1, rtc->base + RTS3903_RTC_CNT_RST);
	writel(0, rtc->base + RTS3903_RTC_CNT_RST);

	writel(sec, rtc->base + RTS3903_RTC_CNT_SET);
	writel(0, rtc->base + RTS3903_RTC_CNT_SET);
	writel(1, rtc->base + RTS3903_RTC_CLK32K_SEL);
	writel(0, rtc->base + RTS3903_RTC_CFG_EN);

	return 0;
}

int rts_set_time(struct rts_rtc *rtc, u32 sec, int xtal)
{
	int ret;

	if (xtal)
		ret = rts_set_time_xtal(rtc, sec);
	else
		ret = rts_set_time_nonxtal(rtc, sec);

	return ret;
}

static inline uint32_t rts_rtc_reg_read(struct rts_rtc *rtc, size_t reg)
{
	return readl(rtc->base + reg);
}

static inline void rts_rtc_reg_write(struct rts_rtc *rtc, size_t reg,
	uint32_t val)
{
	writel(val, rtc->base + reg);
}

static inline void rts_rtc_reg_setbit(struct rts_rtc *rtc, size_t reg,
	uint32_t bit)
{
	set_bit(bit, rtc->base + reg);
}

static inline void rts_rtc_reg_clearbit(struct rts_rtc *rtc, size_t reg,
	uint32_t bit)
{
	clear_bit(bit, rtc->base + reg);
}

static int rts_rtc_read_time(struct device *dev, struct rtc_time *rtc_tm)
{
	uint32_t time, date;
	ulong flags;

	struct platform_device *pdev = to_platform_device(dev);
	struct rts_rtc *pdata = platform_get_drvdata(pdev);


	if (RTS_SOC_CAM_HW_ID(pdata->devtype) == TYPE_RTS3903) {
		time = readl(pdata->base + RTS3903_RTC_C_CNT);
		rtc_time_to_tm(time, rtc_tm);
	} else {
		spin_lock_irqsave(&pdata->lock, flags);

		time =	readl(pdata->base + RTS3901_RTC_TIME_NOW);
		date =  readl(pdata->base + RTS3901_RTC_DATE_NOW);

		spin_unlock_irqrestore(&pdata->lock, flags);

		rtc_tm->tm_min  = MIN_VALUE(time);
		rtc_tm->tm_hour = HOUR_VALUE(time);
		rtc_tm->tm_sec  = SEC_VALUE(time);
		rtc_tm->tm_wday = WEEK_VALUE(time);
		rtc_tm->tm_mday = DAY_VALUE(date);
		rtc_tm->tm_mon  = MON_VALUE(date) - 1;
		rtc_tm->tm_year =
			YEAR_VALUE(date) + CEN_VALUE(date) * 100 - 1900;
	}

	dev_dbg(dev, "read time %04d.%02d.%02d %02d:%02d:%02d\n",
		 1900 + rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
		 rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);

	return rtc_valid_tm(rtc_tm);
}

static int rts_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rts_rtc *pdata = platform_get_drvdata(pdev);
	uint32_t century, year, time, date;
	ulong flags, sec;

	if ((RTS_SOC_CAM_HW_ID(pdata->devtype) == TYPE_RTS3901) ||
		 (RTS_SOC_CAM_HW_ID(pdata->devtype) == TYPE_RLE0745)) {

		century = (tm->tm_year + 1900) / 100;
		year = tm->tm_year % 100;

		time = TIME_REG(tm->tm_wday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);
		date = DATE_REG(century, year,
			(tm->tm_mon + 1), tm->tm_mday);

		dev_dbg(dev,
			"set time %04d.%02d.%02d %02d:%02d:%02d 0x%x 0x%x\n",
			 1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
			 tm->tm_hour, tm->tm_min, tm->tm_sec, time, date);

		spin_lock_irqsave(&pdata->lock, flags);

		rts_rtc_reg_write(pdata, RTS3901_RTC_TIME_LOAD, time);
		rts_rtc_reg_write(pdata, RTS3901_RTC_DATE_LOAD, date);
		rts_rtc_reg_write(pdata, RTS3901_RTC_LOAD, 0xff);

		spin_unlock_irqrestore(&pdata->lock, flags);

	} else {
		rtc_tm_to_time(tm, &sec);
#ifdef CONFIG_RTC_EXTERN_XTAL
		if (pdata->xtal_flag == 1) /*external xtal*/
			rts_set_time(pdata, sec, 1);
		else /*internal xtal*/
			rts_set_time(pdata, sec, 0);
#else
		rts_set_time(pdata, sec, 0);
#endif
	}

	return 0;
}

static int rts_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rts_rtc *pdata = platform_get_drvdata(pdev);
	uint32_t cen, year, time, date;
	struct rtc_time *rtc_tm = &(alrm->time);
	ulong flags, sec;

	if (RTS_SOC_CAM_HW_ID(pdata->devtype) == TYPE_RTS3903) {
		rtc_tm_to_time(rtc_tm, &sec);
		rts_rtc_reg_write(pdata, RTS3903_RTC_ALARM3_SECOND, sec);
	} else {

		cen = (rtc_tm->tm_year + 1900) / 100;
		year = rtc_tm->tm_year % 100;

		rtc_tm->tm_wday = 1 << (rtc_tm->tm_wday - 1);

		time = TIME_REG(rtc_tm->tm_wday, rtc_tm->tm_hour,
			rtc_tm->tm_min, rtc_tm->tm_sec);
		date = DATE_REG(cen,
			year, (rtc_tm->tm_mon + 1), rtc_tm->tm_hour);

		dev_dbg(dev, "set alarm %04d.%02d.%02d %02d:%02d:%02d %x %x\n",
			1900 + rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
			rtc_tm->tm_hour, rtc_tm->tm_min,
			rtc_tm->tm_sec, time, date);

		spin_lock_irqsave(&pdata->lock, flags);

		rts_rtc_reg_write(pdata, RTS3901_RTC_ALARM3_TIME, time);
		rts_rtc_reg_write(pdata, RTS3901_RTC_ALARM3_DATE, date);

		spin_unlock_irqrestore(&pdata->lock, flags);
	}

	return 0;
}

static int rts_rtc_get_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	ulong flags;
	struct platform_device *pdev = to_platform_device(dev);
	struct rts_rtc *pdata = platform_get_drvdata(pdev);
	uint32_t time, date;
	struct rtc_time *rtc_tm = &(alrm->time);

	if (RTS_SOC_CAM_HW_ID(pdata->devtype) == TYPE_RTS3903) {
		time =  readl(pdata->base + RTS3903_RTC_ALARM3_SECOND);
		alrm->enabled = rts_rtc_reg_read(pdata, RTS3903_RTC_ALARM_EN)
			& (1 << ALARM3_ENABLE);
		rtc_time_to_tm(time, rtc_tm);
	} else {
		spin_lock_irqsave(&pdata->lock, flags);
		time =	readl(pdata->base + RTS3901_RTC_ALARM3_TIME);
		date =  readl(pdata->base + RTS3901_RTC_ALARM3_DATE);

		alrm->enabled = rts_rtc_reg_read(pdata, RTS3901_RTC_ALARM_EN)
			& ALARM3_ENABLE;
		spin_unlock_irqrestore(&pdata->lock, flags);

		rtc_tm->tm_min  = MIN_VALUE(time);
		rtc_tm->tm_hour = HOUR_VALUE(time);
		rtc_tm->tm_sec  = SEC_VALUE(time);
		rtc_tm->tm_wday = WEEK_VALUE(time);
		rtc_tm->tm_mday = DAY_VALUE(date);
		rtc_tm->tm_mon  = MON_VALUE(date) - 1;
		rtc_tm->tm_year = YEAR_VALUE(date) +
			CEN_VALUE(date) * 100 - 1900;
	}

	dev_dbg(dev, "get alarm %04d.%02d.%02d %02d:%02d:%02d\n",
		 1900 + rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
		 rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);

	alrm->pending = 0;

	return 0;
}

static irqreturn_t rts_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rts_rtc *pdata = platform_get_drvdata(pdev);
	unsigned long events = 0;

	events = RTC_IRQF | RTC_AF;
	if (likely(pdata->rtc))
		rtc_update_irq(pdata->rtc, 1, events);

	return events ? IRQ_HANDLED : IRQ_NONE;
}

static int rts_rtc_alarm_irq_enable(struct device *dev,
	unsigned int enabled)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rts_rtc *pdata = platform_get_drvdata(pdev);
	ulong flags;

	if (pdata->irq <= 0)
		return -EINVAL;

	if (RTS_SOC_CAM_HW_ID(pdata->devtype) == TYPE_RTS3903) {
		rts_rtc_reg_setbit(pdata,
			RTS3903_RTC_ALARM_EN, ALARM3_ENABLE);
	} else {
		spin_lock_irqsave(&pdata->lock, flags);
		if (enabled)
			rts_rtc_reg_setbit(pdata,
				RTS3901_RTC_ALARM_EN, ALARM3_ENABLE);
		else
			rts_rtc_reg_clearbit(pdata,
				RTS3901_RTC_ALARM_EN, ALARM3_ENABLE);
		spin_unlock_irqrestore(&pdata->lock, flags);
	}

	return 0;
}

static const struct rtc_class_ops rts_rtc_ops = {
	.read_time	= rts_rtc_read_time,
	.set_time	= rts_rtc_set_time,
	.read_alarm	= rts_rtc_get_alarm,
	.set_alarm	= rts_rtc_set_alarm,
	.alarm_irq_enable = rts_rtc_alarm_irq_enable,
};

static struct rts_rtc *rtc_data;
static cycle_t rtc_cs_read(struct clocksource *cs)
{
	uint32_t time, date;
	ulong flags;
	struct rtc_time rtc_tm;
	unsigned long rtc_cyc;

	if (RTS_SOC_CAM_HW_ID(rtc_data->devtype) == TYPE_RTS3903)
		return readl(rtc_data->base + RTS3903_RTC_C_CNT);

	spin_lock_irqsave(&rtc_data->lock, flags);

	time =	readl(rtc_data->base + RTS3901_RTC_TIME_NOW);
	date =  readl(rtc_data->base + RTS3901_RTC_DATE_NOW);

	spin_unlock_irqrestore(&rtc_data->lock, flags);

	rtc_tm.tm_min  = MIN_VALUE(time);
	rtc_tm.tm_hour = HOUR_VALUE(time);
	rtc_tm.tm_sec  = SEC_VALUE(time);
	rtc_tm.tm_wday = WEEK_VALUE(time);
	rtc_tm.tm_mday = DAY_VALUE(date);
	rtc_tm.tm_mon  = MON_VALUE(date) - 1;
	rtc_tm.tm_year =
		YEAR_VALUE(date) + CEN_VALUE(date) * 100 - 1900;

	rtc_tm_to_time(&rtc_tm, &rtc_cyc);

	return rtc_cyc;
}

static struct clocksource clocksource_rtc = {
	.name		= "RTC-RTS",
	.read		= rtc_cs_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static const struct of_device_id rlx_rtc_match[] = {
	{
		.compatible = "realtek,rle0745-fpga-rtc",
		.data = (void *)(TYPE_RLE0745 | TYPE_FPGA),
	}, {
		.compatible = "realtek,rlx0745-rtc",
		.data = (void *)(TYPE_RLE0745),
	}, {
		.compatible = "realtek,rts3901-fpga-rtc",
		.data = (void *)(TYPE_RTS3901 | TYPE_FPGA),
	}, {
		.compatible = "realtek,rts3901-rtc",
		.data = (void *)(TYPE_RTS3901),
	}, {
		.compatible = "realtek,rts3903-fpga-rtc",
		.data = (void *)(TYPE_RTS3903 | TYPE_FPGA),
	}, {
		.compatible = "realtek,rts3903-rtc",
		.data = (void *)(TYPE_RTS3903),
	},
	{}
};
MODULE_DEVICE_TABLE(of, rlx_rtc_match);

static int rts_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct rts_rtc *rtc;
	const struct of_device_id *of_id;
	struct rtc_time tm;
	u32 sec;
	u32 counter_o, counter_t;
	of_id = of_match_device(rlx_rtc_match, &pdev->dev);

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;
	rtc->devtype = (int)of_id->data;

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq < 0) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform irq\n");
		goto err_free;
	}
	rtc->irq = rts_xb2_to_irq(rtc->irq);
	rtc->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rtc->mem) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform mmio memory\n");
		goto err_free;
	}
	rtc->mem = request_mem_region(rtc->mem->start,
		resource_size(rtc->mem), pdev->name);
	if (!rtc->mem) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		goto err_free;
	}
	rtc->base = ioremap_nocache(rtc->mem->start,
		resource_size(rtc->mem));
	if (!rtc->base) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		goto err_release_mem_region;
	}
	spin_lock_init(&rtc->lock);

	platform_set_drvdata(pdev, rtc);

	device_init_wakeup(&pdev->dev, 1);
	rtc->rtc = rtc_device_register(pdev->name,
		&pdev->dev, &rts_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		dev_err(&pdev->dev, "Failed to register rtc device: %d\n", ret);
		goto err_iounmap;
	}
	ret = request_irq(rtc->irq,
		rts_rtc_interrupt, IRQF_SHARED, pdev->name, pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request rtc irq: %d\n", ret);
		goto err_unregister_rtc;
	}

	if ((RTS_SOC_CAM_HW_ID(rtc->devtype) == TYPE_RTS3901) ||
		(RTS_SOC_CAM_HW_ID(rtc->devtype) == TYPE_RLE0745))
		rts_rtc_reg_write(rtc, RTS3901_RTC_EN, 1);
	else {
		counter_o = readl(rtc->base + RTS3903_RTC_S_CNT);
		msleep(20);
		counter_t = readl(rtc->base + RTS3903_RTC_S_CNT);
		if (counter_o != counter_t)
			rtc->xtal_flag = 1;
		else
			rtc->xtal_flag = 0;
		sec  = readl(rtc->base + RTS3903_RTC_C_CNT);
		rtc_time_to_tm(sec, &tm);
		rts_rtc_set_time(&pdev->dev, &tm);
	}

	/* register clocksource */
	rtc_data = rtc;
	clocksource_rtc.rating = 100;
	clocksource_register_hz(&clocksource_rtc, 1);

	return 0;

err_unregister_rtc:
	rtc_device_unregister(rtc->rtc);
err_iounmap:
	platform_set_drvdata(pdev, NULL);
	iounmap(rtc->base);
err_release_mem_region:
	release_mem_region(rtc->mem->start, resource_size(rtc->mem));
err_free:
	kfree(rtc);

	return ret;
}

static int rts_rtc_remove(struct platform_device *pdev)
{
	struct rts_rtc *rtc = platform_get_drvdata(pdev);

	free_irq(rtc->irq, rtc);

	rtc_device_unregister(rtc->rtc);

	iounmap(rtc->base);
	release_mem_region(rtc->mem->start, resource_size(rtc->mem));

	kfree(rtc);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver rts_rtc_driver = {
	.probe	 = rts_rtc_probe,
	.remove	 = rts_rtc_remove,
	.driver	 = {
		.name  = "rts-rtc",
		.of_match_table = rlx_rtc_match,
	},
};

module_platform_driver(rts_rtc_driver);
