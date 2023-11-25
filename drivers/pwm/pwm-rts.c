/* Driver for Realtek PWM
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

#define NUM_PWM 4

#define XB2_PWM_EN 0
#define XB2_PWM_KEEP 4
#define XB2_PWM_HIGH 8
#define XB2_PWM_LOW 0xc
#define XB2_PWM_FLAG 0x10
#define XB2_PWM_NUM 0x14
#define XB2_PWM_CNT 0x1c

struct pwm_desc {
	struct device *dev;
	struct pwm_device *pwm;
	int duty_ns;
	int period_ns;
};

struct rtsx_pwm_chip {
	struct pwm_chip chip;
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;
	struct device *dev;
	u32 busclk;
	struct class *pwm_class;
	struct pwm_desc desc[4];
	void __iomem *mmio_base;
};

static ssize_t duty_ns_show(struct device *_dev,
			    struct device_attribute *attr, char *buf)
{
	u32 duty = 0, i;

	struct rtsx_pwm_chip *pwm = dev_get_drvdata(_dev);

	for (i = 0; i < NUM_PWM; i++)
		if (pwm->desc[i].dev == _dev) {
			if (pwm->desc[i].pwm == 0)
				return -EIO;
			duty = pwm->desc[i].duty_ns;
		}

	return sprintf(buf, "%d\n", duty);
}

static ssize_t duty_ns_store(struct device *_dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u32 i;
	int err;
	long val;

	struct rtsx_pwm_chip *pwm = dev_get_drvdata(_dev);

	err = kstrtol(buf, 10, &val);
	if (err)
		return err;

	if (val >= 0x40000000)
		return -ERANGE;

	for (i = 0; i < NUM_PWM; i++)
		if (pwm->desc[i].dev == _dev) {
			if (pwm->desc[i].pwm == 0)
				return -EIO;
			pwm->desc[i].duty_ns = val;
		}

	return count;
}

DEVICE_ATTR(duty_ns, 0644, duty_ns_show, duty_ns_store);

static ssize_t period_ns_show(struct device *_dev,
			    struct device_attribute *attr, char *buf)
{
	u32 period = 0, i;

	struct rtsx_pwm_chip *pwm = dev_get_drvdata(_dev);

	for (i = 0; i < NUM_PWM; i++)
		if (pwm->desc[i].dev == _dev) {
			if (pwm->desc[i].pwm == 0)
				return -EIO;
			period = pwm->desc[i].period_ns;
		}

	return sprintf(buf, "%d\n", period);
}

static ssize_t period_ns_store(struct device *_dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u32 i;
	int err;
	long val;

	struct rtsx_pwm_chip *pwm = dev_get_drvdata(_dev);

	err = kstrtol(buf, 10, &val);
	if (err)
		return err;

	if (val >= 0x40000000)
		return -ERANGE;

	for (i = 0; i < NUM_PWM; i++)
		if (pwm->desc[i].dev == _dev) {
			if (pwm->desc[i].pwm == 0)
				return -EIO;
			pwm->desc[i].period_ns = val;
		}

	return count;
}

DEVICE_ATTR(period_ns, 0644, period_ns_show, period_ns_store);

static ssize_t enable_store(struct device *_dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u32 i;
	int err;
	long val;

	struct rtsx_pwm_chip *pwm = dev_get_drvdata(_dev);

	err = kstrtol(buf, 10, &val);
	if (err)
		return err;

	for (i = 0; i < NUM_PWM; i++)
		if (pwm->desc[i].dev == _dev) {
			if (pwm->desc[i].pwm == 0)
				return -EIO;

			if (val) {
				pwm_config(pwm->desc[i].pwm,
					pwm->desc[i].duty_ns,
					pwm->desc[i].period_ns);
				pwm_enable(pwm->desc[i].pwm);

			} else {
				pwm_disable(pwm->desc[i].pwm);
			}
		}

	return count;
}

DEVICE_ATTR(enable, 0644, NULL, enable_store);

static ssize_t request_store(struct device *_dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u32 i;
	long val;
	int err;
	struct pwm_device *pdev;
	struct rtsx_pwm_chip *pwm = dev_get_drvdata(_dev);

	err = kstrtol(buf, 10, &val);
	if (err)
		return err;

	for (i = 0; i < NUM_PWM; i++) {
		if (pwm->desc[i].dev == _dev) {
			if (val) {
				if (pwm->desc[i].pwm == 0) {
					pdev = pwm_request(i, NULL);
					if (IS_ERR(pdev)) {
						dev_err(_dev,
							"failed to request\n");
						return PTR_ERR(pdev);
					}
					pwm->desc[i].pwm = pdev;
				} else {
					return -EBUSY;
				}

			} else {
				pwm_free(pwm->desc[i].pwm);
				pwm->desc[i].pwm = 0;
			}
		}
	}

	return count;
}

DEVICE_ATTR(request, 0644, NULL, request_store);

static const struct attribute *pwm_attrs[] = {
	&dev_attr_request.attr,
	&dev_attr_duty_ns.attr,
	&dev_attr_period_ns.attr,
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group pwm_attr_group = {
	.attrs = (struct attribute **) pwm_attrs,
};

static inline struct rtsx_pwm_chip *to_rtsx_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct rtsx_pwm_chip, chip);
}

static inline u32 pwm_readl(struct rtsx_pwm_chip *chip, unsigned int num,
			    int offset)
{
	return readl(chip->mmio_base + (offset + num * 0x40));
}

static inline void pwm_writel(struct rtsx_pwm_chip *chip, unsigned int num,
			      int offset, unsigned int val)
{
	writel(val, chip->mmio_base + offset + num * 0x40);
}

#define NS_IN_HZ (1000000000UL)

static int rtsx_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			   int duty_ns, int period_ns)
{
	struct rtsx_pwm_chip *pc = to_rtsx_pwm_chip(chip);
	unsigned long offset;
	unsigned long high, total, low;
	unsigned long tin_ns;

	offset = pwm->hwpwm;

	tin_ns = NS_IN_HZ / pc->busclk;

	high = duty_ns / tin_ns;
	total = period_ns / tin_ns;
	low = total - high;

	writel(high, pc->mmio_base + offset * 0x40 + XB2_PWM_HIGH);
	writel(low, pc->mmio_base + offset * 0x40 + XB2_PWM_LOW);

	return 0;
}

static int rtsx_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rtsx_pwm_chip *pc = to_rtsx_pwm_chip(chip);

	pwm_writel(pc, pwm->hwpwm, XB2_PWM_EN, 1);

	return 0;
}

static void rtsx_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rtsx_pwm_chip *pc = to_rtsx_pwm_chip(chip);

	pwm_writel(pc, pwm->hwpwm, XB2_PWM_EN, 0);
}

static const struct pwm_ops rtsx_pwm_ops = {
	.config = rtsx_pwm_config,
	.enable = rtsx_pwm_enable,
	.disable = rtsx_pwm_disable,
	.owner = THIS_MODULE,
};

static const struct of_device_id rts_pwm_dt_ids[] = {
	{ .compatible = "realtek,rts3903-pwm", },
	{ }
};
MODULE_DEVICE_TABLE(of, rts_pwm_dt_ids);

static int rtsx_pwm_probe(struct platform_device *pdev)
{
	struct rtsx_pwm_chip *pwm;
	struct resource *r;
	struct clk *clk;
	int ret;
	int i;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->dev = &pdev->dev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pwm->mmio_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(pwm->mmio_base))
		return PTR_ERR(pwm->mmio_base);

	platform_set_drvdata(pdev, pwm);

	clk = clk_get(&pdev->dev, "xb2_ck");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get xb2 clock\n");
		return PTR_ERR(clk);
	}
	pwm->busclk = clk_get_rate(clk);
	clk_put(clk);

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &rtsx_pwm_ops;
	pwm->chip.base = -1;
	pwm->chip.npwm = NUM_PWM;

	pwm->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pwm->pinctrl))
		return PTR_ERR(pwm->pinctrl);

	pwm->default_state = pinctrl_lookup_state(pwm->pinctrl,
						  PINCTRL_STATE_DEFAULT);

	/* Allow pins to be muxed in and configured */
	if (IS_ERR(pwm->default_state)) {
		dev_err(&pdev->dev, "could not get default status\n");
		return PTR_ERR(pwm->default_state);
	}

	ret = pinctrl_select_state(pwm->pinctrl, pwm->default_state);
	if (ret) {
		dev_err(&pdev->dev, "could not set default pins\n");
		return ret;
	}

	writel(1, pwm->mmio_base + XB2_PWM_KEEP);
	writel(1, pwm->mmio_base + 0x40 + XB2_PWM_KEEP);
	writel(1, pwm->mmio_base + 0x80 + XB2_PWM_KEEP);
	writel(1, pwm->mmio_base + 0xc0 + XB2_PWM_KEEP);

	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		return ret;
	}

	pwm->pwm_class = class_create(THIS_MODULE, "settings");
	if (IS_ERR(pwm->pwm_class))
		return PTR_ERR(pwm->pwm_class);

	for (i = 0; i < NUM_PWM; i++) {
		if (i <= pwm->chip.npwm) {
			pwm->desc[i].pwm = pwm->chip.pwms + i;
		}
		pwm->desc[i].dev = device_create(pwm->pwm_class, &pdev->dev,
			MKDEV(0, 0), (void *)pwm, "pwm%d", i);
		if (IS_ERR(pwm->desc[i].dev))
			return PTR_ERR(pwm->desc[i].dev);
		ret = sysfs_create_group(&pwm->desc[i].dev->kobj,
			&pwm_attr_group);
		if (ret < 0)
			return ret;

		if(i == 0)
		{
			rtsx_pwm_config(&pwm->chip, pwm->desc[i].pwm, 1000000, 1000000);
			rtsx_pwm_enable(&pwm->chip, pwm->desc[i].pwm);
		}
	}
	return 0;
}

static int rtsx_pwm_remove(struct platform_device *pdev)
{
	struct pinctrl_state *sleep_state;
	int ret = 0;

	struct rtsx_pwm_chip *pwm = platform_get_drvdata(pdev);
	int i;

	if (WARN_ON(!pwm))
		return -ENODEV;

	for (i = 0; i < NUM_PWM; i++)
		pwm_writel(pwm, i, XB2_PWM_EN, 0);

	ret = pwmchip_remove(&pwm->chip);
	if (ret)
		return ret;

	for (i = 0; i < NUM_PWM; i++) {
		sysfs_remove_group(&pwm->desc[i].dev->kobj, &pwm_attr_group);
		device_unregister(pwm->desc[i].dev);
	}

	class_destroy(pwm->pwm_class);

	sleep_state = pinctrl_lookup_state(pwm->pinctrl, PINCTRL_STATE_SLEEP);

	/* Allow pins to be muxed in and configured */
	if (IS_ERR(sleep_state)) {
		dev_err(&pdev->dev, "could not get sleep status\n");
		return PTR_ERR(sleep_state);
	}

	ret = pinctrl_select_state(pwm->pinctrl, sleep_state);
	if (ret)
		dev_err(&pdev->dev, "could not set sleep status\n");

	return ret;

}

static struct platform_driver pwm_driver = {
	.driver = {
		.name = "pwm_platform",
		.of_match_table = rts_pwm_dt_ids,
	},
	.probe = rtsx_pwm_probe,
	.remove = rtsx_pwm_remove,
};
module_platform_driver(pwm_driver);
