/* Driver for Realtek ipcam pinctrl
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

#include <linux/bitmap.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include "pinctrl-utils.h"

#define MODULE_NAME "pinctrl-rts"

#define XB2_GPIO_OE 0
#define XB2_GPIO_PULLCTRL 4
#define XB2_GPIO_O 8
#define XB2_GPIO_INT_EN 0xC
#define XB2_GPIO_INT_FLAG 0x10
#define XB2_GPIO_DRV_SEL 0x14
#define XB2_GPIO_SR_CTRL 0x18

#define XB2_SHARE_GPIO_SELECT 0x004c
#define XB2_SHARE_GPIO_OE 0x0050
#define XB2_SHARE_GPIO_PULLCTRL 0x0054
#define XB2_SHARE_GPIO_O 0x0058
#define XB2_SHARE_GPIO_INT_EN 0x005c
#define XB2_SHARE_GPIO_INT_FLAG 0x0060
#define XB2_SHARE_GPIO_DRV_SEL 0x0064
#define XB2_SHARE_GPIO_SR_CTRL 0x0068
#define PWM_GPIO_ETN_LED_SEL 0x006c
#define XB2_SHARE_GPIO_PULLCTRL1 0x0070
#define XB2_SHARE_GPIO_INT_EN1 0x0074
#define XB2_SHARE_GPIO_INT_FLAG1 0x0078

#define XB2_VIDEO_SHARE_GPIO_SELECT 0x007C
#define XB2_VIDEO_SHARE_GPIO_OE 0x0080
#define XB2_VIDEO_SHARE_GPIO_PULLCTRL 0x0084
#define XB2_VIDEO_SHARE_GPIO_O 0x0088
#define XB2_VIDEO_SHARE_GPIO_INT_EN 0x008C
#define XB2_VIDEO_SHARE_GPIO_INT_FLAG 0x0090
#define XB2_VIDEO_SHARE_GPIO_DRV_SEL 0x0094
#define XB2_VIDEO_SHARE_GPIO_SR_CTRL 0x0098

#define RTS_PINRANGE(a, b, c)	{ .gpio_base = a, .pin_base = b, .pins = c }
#define RTS_GETFIELD(val, width, offset)	\
			((val >> offset) & ((1 << width) - 1))
#define RTS_SETFIELD(reg, field, width, offset)	((reg & \
			(~(((1 << width) - 1) <<	offset))) \
			| ((field & ((1 << width) - 1)) << offset))

#define GPIO_FUNC_SELECT 0
#define UART_FUNC_SELECT 1
#define PWM_FUNC_SELECT 2
#define DMIC_FUNC_SELECT 3
#define USBD_FUNC_SELECT 4
#define ETNLED_FUNC_SELECT 5
#define USBHST_FUNC_SELECT 6
#define MCU_FUNC_SELECT 7

#define GPIO_GROUP_SELECT 0
#define UART0_GROUP_SELECT 1
#define UART1_GROUP_SELECT 2
#define UART2_GROUP_SELECT 3
#define PWM0_GROUP_SELECT 4
#define PWM1_GROUP_SELECT 5
#define PWM2_GROUP_SELECT 6
#define PWM3_GROUP_SELECT 7
#define DMIC_GROUP_SELECT 8
#define USBD_GROUP_SELECT 9
#define ETNLED0_GROUP_SELECT 10
#define ETNLED1_GROUP_SELECT 11
#define ETNLED2_GROUP_SELECT 12
#define USBHST_GROUP_SELECT 13
#define MCU_GROUP_SELECT 14

#define RTS_SOC_CAM_HW_ID(type)		((int)(type) & 0xff)
#define RTS_MAX_NGPIO	64

enum {
	TYPE_RLE0745 = 1,
	TYPE_RTS3901 = 2,
	TYPE_RTS3903 = 3,
	TYPE_RTS3913 = 4,

	TYPE_FPGA = (1 << 16),
};


struct rts_pinctrl {
	struct gpio_chip *gpio_chip;
	struct device *dev;
	unsigned int *irq_type;
	struct pinctrl_dev *rtspctl_dev;
	struct irq_domain *irq_domain;
	spinlock_t irq_lock;
	void __iomem *addr;
	int irq;
	u64 pinsmask;
	int devt;
};

struct rts_pin_group {
	const char *name;
	const unsigned int *pins;
	const unsigned int num_pins;
};

struct rts_func {
	const char *name;
	const char *const *groups;
	const unsigned int num_groups;
};

struct rts_pinpair {
	unsigned int gpio_base;
	unsigned int pin_base;
	unsigned int pins;
};

static struct lock_class_key gpio_lock_class;

struct pinctrl_pin_desc rts_gpio_pins[] = {
	PINCTRL_PIN(0, "GPIO0"),
	PINCTRL_PIN(1, "GPIO1"),
	PINCTRL_PIN(2, "GPIO2"),
	PINCTRL_PIN(3, "GPIO3"),
	PINCTRL_PIN(4, "GPIO4"),
	PINCTRL_PIN(5, "MDIOGPIO5"),
	PINCTRL_PIN(6, "MDIOGPIO6"),
	PINCTRL_PIN(7, "GPIO7"),
	PINCTRL_PIN(8, "PWMGPIO0"),
	PINCTRL_PIN(9, "PWMGPIO1"),
	PINCTRL_PIN(10, "PWMGPIO2"),
	PINCTRL_PIN(11, "PWMGPIO3"),
	PINCTRL_PIN(12, "UART2TXD"),
	PINCTRL_PIN(13, "UART2RXD"),
	PINCTRL_PIN(14, "UART1TXD"),
	PINCTRL_PIN(15, "UART1RXD"),
	PINCTRL_PIN(16, "UART0TXD"),
	PINCTRL_PIN(17, "UART0RXD"),
	PINCTRL_PIN(18, "UART0CTS"),
	PINCTRL_PIN(19, "UART0RTS"),
	PINCTRL_PIN(20, "DMICDATA"),
	PINCTRL_PIN(21, "DMICCLK"),
	PINCTRL_PIN(22, "USBDVBUS"),
	PINCTRL_PIN(23, "AUDIO_MCLK"),
	PINCTRL_PIN(24, "AUDIO_SCK"),
	PINCTRL_PIN(25, "AUDIO_SD_IN"),
	PINCTRL_PIN(26, "AUDIO_SD_OUT"),
	PINCTRL_PIN(27, "AUDIO_WS"),
	PINCTRL_PIN(28, "AUDIO_PDM_OUT"),
	PINCTRL_PIN(29, "AUDIO_CLK_PDM"),
	PINCTRL_PIN(30, "AUDIO_SPDIF_OUT"),
	PINCTRL_PIN(31, "HST_PORT_OVERCURRENT"),
	PINCTRL_PIN(32, "HST_PORT_PWR_EN"),
	PINCTRL_PIN(33, "PIXDIN_EXT_0"),
	PINCTRL_PIN(34, "PIXDIN_EXT_1"),
	PINCTRL_PIN(35, "PIXDIN_0"),
	PINCTRL_PIN(36, "PIXDIN_1"),
	PINCTRL_PIN(37, "PIXDIN_2"),
	PINCTRL_PIN(38, "PIXDIN_3"),
	PINCTRL_PIN(39, "HSYNC"),
	PINCTRL_PIN(40, "VSYNC"),
};

static const unsigned int gpio_pins[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
	14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
	26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40
};

static const unsigned int pwm0_pins[] = {
	8
};

static const unsigned int pwm1_pins[] = {
	9
};

static const unsigned int pwm2_pins[] = {
	10
};

static const unsigned int pwm3_pins[] = {
	11
};

static const unsigned int uart0_pins[] = {
	16, 17, 18, 19
};

static const unsigned int uart1_pins[] = {
	14, 15
};

static const unsigned int uart2_pins[] = {
	12, 13
};

static const unsigned int dmic_pins[] = {
	20, 21
};

static const unsigned int usbd_pins[] = {
	22
};

static const unsigned int etnled0_pins[] = {
	9
};

static const unsigned int etnled1_pins[] = {
	10
};

static const unsigned int etnled2_pins[] = {
	11
};

static const unsigned usbhst_pins[] = {
	31, 32
};

static const unsigned mcu_pins[] = {
	33, 34, 35, 36, 37, 38, 39, 40
};


static const struct rts_pin_group rts_pin_groups[] = {
	{
	 .name = "gpiogrp",
	 .pins = gpio_pins,
	 .num_pins = ARRAY_SIZE(gpio_pins),
	 },
	{
	 .name = "uart0grp",
	 .pins = uart0_pins,
	 .num_pins = ARRAY_SIZE(uart0_pins),
	 },
	{
	 .name = "uart1grp",
	 .pins = uart1_pins,
	 .num_pins = ARRAY_SIZE(uart1_pins),
	 },
	{
	 .name = "uart2grp",
	 .pins = uart2_pins,
	 .num_pins = ARRAY_SIZE(uart2_pins),
	 },
	{
	 .name = "pwm0grp",
	 .pins = pwm0_pins,
	 .num_pins = ARRAY_SIZE(pwm0_pins),
	 },
	{
	 .name = "pwm1grp",
	 .pins = pwm1_pins,
	 .num_pins = ARRAY_SIZE(pwm1_pins),
	 },
	{
	 .name = "pwm2grp",
	 .pins = pwm2_pins,
	 .num_pins = ARRAY_SIZE(pwm2_pins),
	 },
	{
	 .name = "pwm3grp",
	 .pins = pwm3_pins,
	 .num_pins = ARRAY_SIZE(pwm3_pins),
	 },
	{
	 .name = "dmicgrp",
	 .pins = dmic_pins,
	 .num_pins = ARRAY_SIZE(dmic_pins),
	 },
	{
	 .name = "usbdgrp",
	 .pins = usbd_pins,
	 .num_pins = ARRAY_SIZE(usbd_pins),
	 },
	{
	 .name = "etnled0grp",
	 .pins = etnled0_pins,
	 .num_pins = ARRAY_SIZE(etnled0_pins),
	},
	{
	 .name = "etnled1grp",
	 .pins = etnled1_pins,
	 .num_pins = ARRAY_SIZE(etnled1_pins),
	},
	{
	 .name = "etnled2grp",
	 .pins = etnled2_pins,
	 .num_pins = ARRAY_SIZE(etnled2_pins),
	},
	{
	 .name = "usbhstgrp",
	 .pins = usbhst_pins,
	 .num_pins = ARRAY_SIZE(usbhst_pins),
	},
	{
	 .name = "mcugrp",
	 .pins = mcu_pins,
	 .num_pins = ARRAY_SIZE(mcu_pins),
	},
};

static const char *const gpiogrps[] = { "gpiogrp" };
static const char *const uartgrps[] = { "uart0grp", "uart1grp", "uart2grp" };

static const char *const pwmgrps[] = {
	"pwm0grp", "pwm1grp", "pwm2grp", "pwm3grp"
};
static const char *const dmicgrps[] = { "dmicgrp" };
static const char *const usbdgrps[] = { "usbdgrp" };
static const char *const etnledgrps[] = {
	"etnled0grp", "etnled1grp", "etnled2grp"
};
static const char *const usbhstgrps[] = { "usbhstgrp" };
static const char *const mcugrps[] = { "mcugrp" };


static const struct rts_func rts_functions[] = {
	{
	 .name = "gpiofunc",
	 .groups = gpiogrps,
	 .num_groups = ARRAY_SIZE(gpiogrps),
	 },
	{
	 .name = "uartfunc",
	 .groups = uartgrps,
	 .num_groups = ARRAY_SIZE(uartgrps),
	 },
	{
	 .name = "pwmfunc",
	 .groups = pwmgrps,
	 .num_groups = ARRAY_SIZE(pwmgrps),
	 },
	{
	 .name = "dmicfunc",
	 .groups = dmicgrps,
	 .num_groups = ARRAY_SIZE(dmicgrps),
	 },
	{
	 .name = "usbdfunc",
	 .groups = usbdgrps,
	 .num_groups = ARRAY_SIZE(usbdgrps),
	 },
	{
	 .name = "etnledfunc",
	 .groups = etnledgrps,
	 .num_groups = ARRAY_SIZE(etnledgrps),
	},
	{
	 .name = "usbhstfunc",
	 .groups = usbhstgrps,
	 .num_groups = ARRAY_SIZE(usbhstgrps),
	},
	{
	 .name = "mcufunc",
	 .groups = mcugrps,
	 .num_groups = ARRAY_SIZE(mcugrps),
	},
};

static struct rts_pinpair rts_pintable_3901[] = {
	RTS_PINRANGE(0, 0, 23),
};

static struct rts_pinpair rts_pintable_3903[] = {
	RTS_PINRANGE(0, 0, 31),
};

static struct rts_pinpair rts_pintable_3913[] = {
	RTS_PINRANGE(0, 0, 41),
};

static void rts_gpio_set_field(void __iomem *reg,
			       unsigned int field, unsigned int width,
			       unsigned int offset)
{
	unsigned int val = readl(reg);

	val = RTS_SETFIELD(val, field, width, offset);
	writel(val, reg);
}

static unsigned int rts_gpio_get_field(void __iomem *reg,
				       unsigned int width, unsigned int offset)
{
	unsigned int val = readl(reg);

	return RTS_GETFIELD(val, width, offset);
}

static int rts_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	struct rts_pinctrl *rtspc = gpiochip_get_data(chip);
	int res = pinctrl_request_gpio(chip->base + offset);
	if (!res) {
		if (offset >= 33)
			set_bit(offset - 33,
				rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		else if (offset >= 8)
			set_bit(offset - 8,
				rtspc->addr + XB2_SHARE_GPIO_SELECT);
	}

	return res;
}

static void rts_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	struct rts_pinctrl *rtspc = gpiochip_get_data(chip);
	if (offset >= 33)
		clear_bit(offset - 33,
			rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
	else if (offset >= 8)
		clear_bit(offset - 8, rtspc->addr + XB2_SHARE_GPIO_SELECT);
	pinctrl_free_gpio(chip->base + offset);

}

static int rts_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int rts_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct rts_pinctrl *rtspc = gpiochip_get_data(chip);

	if (offset < 8)
		return rts_gpio_get_field(rtspc->addr + XB2_GPIO_O,
			1, offset);
	else if (offset < 33)
		return rts_gpio_get_field(rtspc->addr + XB2_SHARE_GPIO_O,
			1, offset-8);
	else
		return rts_gpio_get_field(
			rtspc->addr + XB2_VIDEO_SHARE_GPIO_O, 1, offset-33);
}

static void rts_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct rts_pinctrl *rtspc = gpiochip_get_data(chip);

	if (value)
		if (offset < 8)
			set_bit(offset, rtspc->addr + XB2_GPIO_O);
		else if (offset < 33)
			set_bit(offset-8, rtspc->addr + XB2_SHARE_GPIO_O);
		else
			set_bit(offset-33,
				rtspc->addr + XB2_VIDEO_SHARE_GPIO_O);
	else
		if (offset < 8)
			clear_bit(offset, rtspc->addr + XB2_GPIO_O);
		else if (offset < 33)
			clear_bit(offset-8, rtspc->addr + XB2_SHARE_GPIO_O);
		else
			clear_bit(offset-33,
				rtspc->addr + XB2_VIDEO_SHARE_GPIO_O);

}

static int rts_gpio_direction_output(struct gpio_chip *chip,
				     unsigned int offset, int value)
{
	rts_gpio_set(chip, offset, value);
	return pinctrl_gpio_direction_output(chip->base + offset);
}

static int rts_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{

	struct rts_pinctrl *rtspc = gpiochip_get_data(chip);

	return irq_linear_revmap(rtspc->irq_domain, offset);
}

static struct gpio_chip rts_gpio_chip = {
	.label = MODULE_NAME,
	.owner = THIS_MODULE,
	.request = rts_gpio_request,
	.free = rts_gpio_free,
	.direction_input = rts_gpio_direction_input,
	.direction_output = rts_gpio_direction_output,
	.get = rts_gpio_get,
	.set = rts_gpio_set,
	.to_irq = rts_gpio_to_irq,
	.base = 0,
	.ngpio = RTS_MAX_NGPIO,
	.can_sleep = 0,
};

static int rts_rtspctl_get_groups_count(struct pinctrl_dev *rtspctldev)
{
	return ARRAY_SIZE(rts_pin_groups);
}

static const char *rts_rtspctl_get_group_name(struct pinctrl_dev *rtspctldev,
					      unsigned int selector)
{
	return rts_pin_groups[selector].name;
}

static int rts_rtspctl_get_group_pins(struct pinctrl_dev *rtspctldev,
		      unsigned int selector,
		      const unsigned int **pins, unsigned int *num_pins)
{
	*pins = rts_pin_groups[selector].pins;
	*num_pins = rts_pin_groups[selector].num_pins;

	return 0;
}

static void rts_rtspctl_pin_dbg_show(struct pinctrl_dev *rtspctldev,
				     struct seq_file *s, unsigned int offset)
{

}

static const struct pinctrl_ops rts_rtspctl_ops = {
	.get_groups_count = rts_rtspctl_get_groups_count,
	.get_group_name = rts_rtspctl_get_group_name,
	.get_group_pins = rts_rtspctl_get_group_pins,
	.pin_dbg_show = rts_rtspctl_pin_dbg_show,
#ifdef CONFIG_OF
	.dt_node_to_map	= pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
#endif

};

static int rts_pmx_get_functions_count(struct pinctrl_dev *rtspctldev)
{
	return ARRAY_SIZE(rts_functions);
}

static const char *rts_pmx_get_function_name(struct pinctrl_dev *rtspctldev,
					     unsigned int selector)
{
	return rts_functions[selector].name;
}

static int rts_pmx_get_function_groups(struct pinctrl_dev *rtspctldev,
				       unsigned int selector,
				       const char *const **groups,
				       unsigned int *const num_groups)
{
	/* every pin can do every function */
	*groups = rts_functions[selector].groups;
	*num_groups = rts_functions[selector].num_groups;

	return 0;
}

static int rts_pmx_enable(struct pinctrl_dev *rtspctldev,
	unsigned int func_selector, unsigned int group_selector)
{

	struct rts_pinctrl *rtspc = pinctrl_dev_get_drvdata(rtspctldev);

	switch (func_selector) {
	case PWM_FUNC_SELECT:
		if (group_selector == PWM0_GROUP_SELECT)
			clear_bit(0, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		else if (group_selector == PWM1_GROUP_SELECT)
			clear_bit(1, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		else if (group_selector == PWM2_GROUP_SELECT)
			clear_bit(2, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		else
			clear_bit(3, rtspc->addr + XB2_SHARE_GPIO_SELECT);

		break;
	case UART_FUNC_SELECT:
		if (group_selector == UART0_GROUP_SELECT) {
			clear_bit(8, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			clear_bit(9, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			clear_bit(10, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			clear_bit(11, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		} else if (group_selector == UART1_GROUP_SELECT) {
			clear_bit(6, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			clear_bit(7, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		} else {
			clear_bit(4, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			clear_bit(5, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		}
		break;
	case DMIC_FUNC_SELECT:
		clear_bit(12, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		clear_bit(13, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		break;
	case USBD_FUNC_SELECT:
		clear_bit(14, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		break;
	case ETNLED_FUNC_SELECT:
		if (group_selector == ETNLED0_GROUP_SELECT) {
			set_bit(0, rtspc->addr + PWM_GPIO_ETN_LED_SEL);
			set_bit(1, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			set_bit(1, rtspc->addr + XB2_SHARE_GPIO_OE);
		} else if (group_selector == ETNLED1_GROUP_SELECT) {
			set_bit(1, rtspc->addr + PWM_GPIO_ETN_LED_SEL);
			set_bit(2, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			set_bit(2, rtspc->addr + XB2_SHARE_GPIO_OE);
		} else {
			set_bit(2, rtspc->addr + PWM_GPIO_ETN_LED_SEL);
			set_bit(3, rtspc->addr + XB2_SHARE_GPIO_SELECT);
			set_bit(3, rtspc->addr + XB2_SHARE_GPIO_OE);
		}
		break;
	case USBHST_FUNC_SELECT:
		clear_bit(23, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		clear_bit(24, rtspc->addr + XB2_SHARE_GPIO_SELECT);
		break;
	case MCU_FUNC_SELECT:
		clear_bit(0, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(1, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(2, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(3, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(4, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(5, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(6, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		clear_bit(7, rtspc->addr + XB2_VIDEO_SHARE_GPIO_SELECT);
		break;

	default:
		dev_err(rtspc->dev, "not known function selector\n");
		break;
	}
	return 0;
}

int rts_gpio_config_set(struct rts_pinctrl *rtspc, unsigned int pin,
			unsigned int config, unsigned int value)
{
	switch (config) {
	case RTS_PINCONFIG_PULL_NONE:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr + XB2_GPIO_PULLCTRL,
					   0, 2, pin << 1);
		else if (pin < 23)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_PULLCTRL, 0, 2,
					   (pin - 8) << 1);
		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_PULLCTRL1, 0, 2,
					   (pin - 23) << 1);
		else
			rts_gpio_set_field(rtspc->addr
					   + XB2_VIDEO_SHARE_GPIO_PULLCTRL,
					   0, 2, (pin - 33) << 1);

		break;
	case RTS_PINCONFIG_PULL_DOWN:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr
					   + XB2_GPIO_PULLCTRL, 1, 2, pin << 1);
		else if (pin < 23)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_PULLCTRL,
					   1, 2, (pin - 8) << 1);
		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_PULLCTRL1, 1, 2,
					   (pin - 23) << 1);
		else
			rts_gpio_set_field(rtspc->addr
					   + XB2_VIDEO_SHARE_GPIO_PULLCTRL,
					   1, 2, (pin - 33) << 1);
		break;
	case RTS_PINCONFIG_PULL_UP:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr
					   + XB2_GPIO_PULLCTRL, 2, 2, pin << 1);
		else if (pin < 23)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_PULLCTRL,
					   2, 2, (pin - 8) << 1);
		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_PULLCTRL1, 2, 2,
					   (pin - 23) << 1);
		else
			rts_gpio_set_field(rtspc->addr
					   + XB2_VIDEO_SHARE_GPIO_PULLCTRL,
					   2, 2, (pin - 33) << 1);

		break;
	case RTS_PIN_CONFIG_DRIVE_STRENGTH:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr
					   + XB2_GPIO_DRV_SEL, value, 1, pin);

		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_DRV_SEL,
					   value, 1, (pin - 8));
		else
			rts_gpio_set_field(rtspc->addr
					   + XB2_VIDEO_SHARE_GPIO_DRV_SEL,
					   value, 1, (pin - 33));

		break;
	case RTS_PIN_CONFIG_SLEW_RATE:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr
					   + XB2_GPIO_SR_CTRL, value, 1, pin);
		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr
					   + XB2_SHARE_GPIO_SR_CTRL,
					   value, 1, (pin - 8));
		else
			rts_gpio_set_field(rtspc->addr
					   + XB2_VIDEO_SHARE_GPIO_SR_CTRL,
					   value, 1, (pin - 33));
		break;
	case RTS_PINCONFIG_INPUT:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr
					   + XB2_GPIO_OE, 0, 1, pin);
		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr + XB2_SHARE_GPIO_OE,
					   0, 1, (pin - 8));
		else
			rts_gpio_set_field(rtspc->addr +
				XB2_VIDEO_SHARE_GPIO_OE, 0, 1, (pin - 33));
		break;
	case RTS_PINCONFIG_OUTPUT:
		if (pin < 8)
			rts_gpio_set_field(rtspc->addr
					   + XB2_GPIO_OE, 1, 1, pin);
		else if (pin < 33)
			rts_gpio_set_field(rtspc->addr + XB2_SHARE_GPIO_OE,
					   1, 1, (pin - 8));
		else
			rts_gpio_set_field(
				rtspc->addr + XB2_VIDEO_SHARE_GPIO_OE,
				1, 1, (pin - 33));
		break;
	default:
		dev_err(rtspc->dev, "illegal configuration requested\n");
		return -EINVAL;
	}

	return 0;
}

static int rts_pmx_gpio_set_direction(struct pinctrl_dev *rtspctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int offset, bool input)
{
	int ret;
	struct rts_pinctrl *rtspc = pinctrl_dev_get_drvdata(rtspctldev);

	int config = input ? RTS_PINCONFIG_INPUT : RTS_PINCONFIG_OUTPUT;

	ret = rts_gpio_config_set(rtspc, offset, config, 0);

	return 0;
}

static int rts_pmx_request(struct pinctrl_dev *rtspctldev, unsigned int offset)
{
	struct rts_pinctrl *rtspc = pinctrl_dev_get_drvdata(rtspctldev);

	if (rtspc->devt == TYPE_RTS3913) {
		if (offset > 40)
			return -EINVAL;
	} else if (rtspc->devt == TYPE_RTS3903) {
		if (offset > 30)
			return -EINVAL;
	} else {
		if (offset > 22)
			return -EINVAL;
	}
	if (rtspc->pinsmask & (1 << offset)) {
		return -EBUSY;
	} else {
		rtspc->pinsmask |= (1 << offset);
		return 0;
	}
}

static int rts_pmx_free(struct pinctrl_dev *rtspctldev, unsigned int offset)
{
	struct rts_pinctrl *rtspc = pinctrl_dev_get_drvdata(rtspctldev);

	if (rtspc->pinsmask & (1 << offset)) {
		rtspc->pinsmask &= ~(1 << offset);
		return 0;
	} else {
		return -EINVAL;
	}
}

static const struct pinmux_ops rts_pmx_ops = {
	.request = rts_pmx_request,
	.free = rts_pmx_free,
	.get_functions_count = rts_pmx_get_functions_count,
	.get_function_name = rts_pmx_get_function_name,
	.get_function_groups = rts_pmx_get_function_groups,
	.set_mux = rts_pmx_enable,
	.gpio_set_direction = rts_pmx_gpio_set_direction,
};

int rts_pin_config_set(struct rts_pinctrl *rtspc,
	unsigned int pin, unsigned int param)
{
	int ret;
	unsigned int config = param & 0xffff;
	unsigned int value = param >> 16;

	ret = rts_gpio_config_set(rtspc, pin, config, value);

	return ret;
}

static int rts_pinconf_get(struct pinctrl_dev *rtspctldev,
	unsigned int pin, unsigned long *config)
{
	return -ENOTSUPP;
}

static int rts_pinconf_set(struct pinctrl_dev *rtspctldev,
			       unsigned int pin,
			       unsigned long *configs,
			       unsigned int num_configs)
{
	int ret, i;
	struct rts_pinctrl *rtspc;

	rtspc = pinctrl_dev_get_drvdata(rtspctldev);

	for (i = 0; i < num_configs; i++) {
		ret = rts_pin_config_set(rtspc, pin, configs[i]);
		if (ret)
			return ret;
	}

	return 0;
}

/* set the pin config settings for a specified pin group */
static int rts_pinconf_group_set(struct pinctrl_dev *pctldev,
				unsigned int group, unsigned long *configs,
				unsigned int num_configs)
{
	const unsigned int *pins;
	unsigned int cnt;

	pins = rts_pin_groups[group].pins;
	for (cnt = 0; cnt < rts_pin_groups[group].num_pins; cnt++)
		rts_pinconf_set(pctldev, pins[cnt], configs, num_configs);

	return 0;
}

static const struct pinconf_ops rts_pinconf_ops = {
	.pin_config_get = rts_pinconf_get,
	.pin_config_set = rts_pinconf_set,
	.pin_config_group_set = rts_pinconf_group_set,
};

static struct pinctrl_desc rts_pinctrl_desc = {
	.name = MODULE_NAME,
	.pins = rts_gpio_pins,
	.npins = ARRAY_SIZE(rts_gpio_pins),
	.pctlops = &rts_rtspctl_ops,
	.pmxops = &rts_pmx_ops,
	.confops = &rts_pinconf_ops,
	.owner = THIS_MODULE,
};

static void rts_gpio_irq_enable(struct irq_data *data)
{
	struct rts_pinctrl *rtspc = irq_data_get_irq_chip_data(data);
	unsigned char gpio = (unsigned char)irqd_to_hwirq(data);
	unsigned long flags;
	u32 t;

	spin_lock_irqsave(&rtspc->irq_lock, flags);
	if (gpio < 8) {
		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_RISING
		    || rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH)
			set_bit(gpio + 16, rtspc->addr + XB2_GPIO_INT_EN);

		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_FALLING ||
		    rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH)
			set_bit(gpio, rtspc->addr + XB2_GPIO_INT_EN);
	} else if (gpio < 23) {
		t = gpio - 8;

		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_RISING
		    || rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH)
			set_bit(t + 16, rtspc->addr + XB2_SHARE_GPIO_INT_EN);
		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_FALLING ||
		    rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH)
			set_bit(t,
				rtspc->addr + XB2_SHARE_GPIO_INT_EN);
	} else if (gpio < 33) {
		t = gpio - 23;
		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_RISING
		    || rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH) {
			set_bit(t + 16, rtspc->addr + XB2_SHARE_GPIO_INT_EN1);

		}
		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_FALLING ||
		    rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH) {
			set_bit(t,
				rtspc->addr + XB2_SHARE_GPIO_INT_EN1);
		}
	} else {
		t = gpio - 33;
		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_RISING
		    || rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH) {
			set_bit(t + 16,
				rtspc->addr + XB2_VIDEO_SHARE_GPIO_INT_EN);

		}
		if (rtspc->irq_type[gpio] ==
		    IRQ_TYPE_EDGE_FALLING ||
		    rtspc->irq_type[gpio] == IRQ_TYPE_EDGE_BOTH) {
			set_bit(t,
				rtspc->addr + XB2_VIDEO_SHARE_GPIO_INT_EN);
		}
	}

	spin_unlock_irqrestore(&rtspc->irq_lock, flags);
}

static void rts_gpio_irq_disable(struct irq_data *data)
{
	struct rts_pinctrl *rtspc = irq_data_get_irq_chip_data(data);
	unsigned char gpio = (unsigned char)irqd_to_hwirq(data);
	unsigned long flags;

	spin_lock_irqsave(&rtspc->irq_lock, flags);
	if (gpio < 8) {
		clear_bit(gpio, rtspc->addr + XB2_GPIO_INT_EN);
		clear_bit(gpio + 16, rtspc->addr + XB2_GPIO_INT_EN);
	} else if (gpio < 23) {
		gpio -= 8;
		clear_bit(gpio, rtspc->addr + XB2_SHARE_GPIO_INT_EN);
		clear_bit(gpio + 16, rtspc->addr + XB2_SHARE_GPIO_INT_EN);
	} else if (gpio < 33) {
		gpio -= 23;
		clear_bit(gpio, rtspc->addr + XB2_SHARE_GPIO_INT_EN1);
		clear_bit(gpio + 16, rtspc->addr + XB2_SHARE_GPIO_INT_EN1);
	} else {
		gpio -= 33;
		clear_bit(gpio, rtspc->addr + XB2_VIDEO_SHARE_GPIO_INT_EN);
		clear_bit(gpio + 16,
			rtspc->addr + XB2_VIDEO_SHARE_GPIO_INT_EN);
	}

	spin_unlock_irqrestore(&rtspc->irq_lock, flags);
}

static int rts_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct rts_pinctrl *rtspc = irq_data_get_irq_chip_data(data);
	unsigned char gpio = (unsigned char)irqd_to_hwirq(data);

	switch (type) {
	case IRQ_TYPE_NONE:
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
		rtspc->irq_type[gpio] = type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct irq_chip rts_gpio_irq_chip = {
	.name = MODULE_NAME,
	.irq_enable = rts_gpio_irq_enable,
	.irq_disable = rts_gpio_irq_disable,
	.irq_set_type = rts_gpio_irq_set_type,
};

static irqreturn_t rts_irq_handler(int irq, void *pc)
{
	unsigned long offset;
	struct rts_pinctrl *rtspc = (struct rts_pinctrl *)pc;
	unsigned long val1 = 0, val2 = 0, val3 = 0, val4 = 0;
	int irqno = 0;

	val1 = readl(rtspc->addr + XB2_GPIO_INT_FLAG);
	val2 = readl(rtspc->addr + XB2_SHARE_GPIO_INT_FLAG);
	if (rtspc->devt == TYPE_RTS3903) {
		val3 = readl(rtspc->addr + XB2_SHARE_GPIO_INT_FLAG1);
	} else if (rtspc->devt == TYPE_RTS3913) {
		val3 = readl(rtspc->addr + XB2_SHARE_GPIO_INT_FLAG1);
		val4 = readl(rtspc->addr + XB2_VIDEO_SHARE_GPIO_INT_FLAG);
	}

	val1 &= 0x00ff00ff;
	val2 &= 0x7fff7fff;
	if (rtspc->devt == TYPE_RTS3903) {
		val3 &= 0x00ff00ff;
	} else if (rtspc->devt == TYPE_RTS3913) {
		val3 &= 0x03ff03ff;
		val4 &= 0x00ff00ff;
	}

	writel(val1, rtspc->addr + XB2_GPIO_INT_FLAG);
	writel(val2, rtspc->addr + XB2_SHARE_GPIO_INT_FLAG);

	if (rtspc->devt == TYPE_RTS3903) {
		writel(val3, rtspc->addr + XB2_SHARE_GPIO_INT_FLAG1);
	}
	else if (rtspc->devt == TYPE_RTS3913) {
		writel(val3, rtspc->addr + XB2_SHARE_GPIO_INT_FLAG1);
		writel(val4, rtspc->addr + XB2_VIDEO_SHARE_GPIO_INT_FLAG);
	}

	if (val1 == 0 && val2 == 0 && val3 == 0 && val4 == 0)
		return IRQ_NONE;

	for_each_set_bit(offset, (const unsigned long *)&val1, 32) {
		irqno = offset;
		if (irqno >= 16)
			irqno -= 16;
		irqno = irq_linear_revmap(rtspc->irq_domain, irqno);
		if (irqno)
			generic_handle_irq(irqno);
	}

	for_each_set_bit(offset, (const unsigned long *)&val2, 32) {
		irqno = offset;
		if (irqno >= 16)
			irqno -= 16;
		irqno += 8;
		irqno = irq_linear_revmap(rtspc->irq_domain, irqno);
		if (irqno)
			generic_handle_irq(irqno);
	}

	if (rtspc->devt == TYPE_RTS3903)
		for_each_set_bit(offset, (const unsigned long *)&val3, 32) {
			irqno = offset;
			if (irqno >= 16)
				irqno -= 16;
			irqno += 23;
			irqno = irq_linear_revmap(rtspc->irq_domain, irqno);
			if (irqno)
				generic_handle_irq(irqno);
		}

	if (rtspc->devt == TYPE_RTS3913) {
		for_each_set_bit(offset, (const unsigned long *)&val3, 32) {
			irqno = offset;
			if (irqno >= 16)
				irqno -= 16;
			irqno += 23;
			irqno = irq_linear_revmap(rtspc->irq_domain, irqno);
			if (irqno)
				generic_handle_irq(irqno);
		}

		for_each_set_bit(offset, (const unsigned long *)&val4, 32) {
			irqno = offset;
			if (irqno >= 16)
				irqno -= 16;
			irqno += 33;
			irqno = irq_linear_revmap(rtspc->irq_domain, irqno);
			if (irqno)
				generic_handle_irq(irqno);
		}
	}

	return IRQ_HANDLED;
}

static const struct of_device_id rts_pinctrl_match[] = {
	{
		.compatible = "realtek,rts3903-pinctrl",
		.data = (void *)(TYPE_RTS3903),
	}, {
		.compatible = "realtek,rts3913-pinctrl",
		.data = (void *)(TYPE_RTS3913),
	},
	{}
};
MODULE_DEVICE_TABLE(of, rts_pinctrl_match);

static int rts_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rts_pinctrl *rtspc;
	struct resource *res;
	int err, i, len;
	struct rts_pinpair *pp;
	u32 gpio_numbers;

	const struct of_device_id *of_id;

	of_id = of_match_device(rts_pinctrl_match, &pdev->dev);

	rtspc = devm_kzalloc(dev, sizeof(*rtspc), GFP_KERNEL);
	if (!rtspc)
		return -ENOMEM;

	err = of_property_read_u32(dev->of_node, "gpio-numbers", &gpio_numbers);
	if (err < 0) {
		dev_err(dev, "failed to get gpio numbers:%d\n", err);
		devm_kfree(dev, rtspc);
		return err;
	}
	if (gpio_numbers > RTS_MAX_NGPIO)
		gpio_numbers = RTS_MAX_NGPIO;
	rtspc->irq_type = kcalloc(gpio_numbers, sizeof(int), GFP_KERNEL);
	if (rtspc->irq_type == NULL) {
		devm_kfree(dev, rtspc);
		return -ENOMEM;
	}

	rtspc->devt = RTS_SOC_CAM_HW_ID(of_id->data);
	platform_set_drvdata(pdev, rtspc);
	rtspc->dev = dev;

	spin_lock_init(&rtspc->irq_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	rtspc->addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(rtspc->addr))
		return PTR_ERR(rtspc->addr);

	rtspc->irq = platform_get_irq(pdev, 0);
	if (rtspc->irq < 0) {
		dev_err(dev, "irqs not supported\n");
		return rtspc->irq;
	}

	rtspc->irq_domain = irq_domain_add_linear(NULL, gpio_numbers,
						  &irq_domain_simple_ops, NULL);
	if (!rtspc->irq_domain) {
		dev_err(dev, "could not create IRQ domain\n");
		return -ENOMEM;
	}

	for (i = 0; i < gpio_numbers; i++) {
		int gpioirq = irq_create_mapping(rtspc->irq_domain, i);

		irq_set_lockdep_class(gpioirq, &gpio_lock_class);
		irq_set_chip_and_handler(gpioirq, &rts_gpio_irq_chip,
					 handle_simple_irq);
		irq_set_chip_data(gpioirq, rtspc);
	}

	err = request_irq(rtspc->irq, rts_irq_handler,
			  IRQF_SHARED, "gpio", (void *)rtspc);
	if (err) {
		dev_err(dev, "failure requesting irq %i\n", rtspc->irq);
		return err;
	}
	dev_info(&pdev->dev, "rtspc registered with IRQs\n");

	rts_gpio_chip.ngpio = gpio_numbers;
	rtspc->gpio_chip = &rts_gpio_chip;
	rtspc->gpio_chip->of_node = dev->of_node;
	rtspc->gpio_chip->label = dev_name(dev);
	rtspc->gpio_chip->parent = dev;

	err = gpiochip_add_data(rtspc->gpio_chip, rtspc);
	if (err) {
		dev_err(dev, "failed to add gpiochip\n");
		return err;
	}

	rtspc->rtspctl_dev = pinctrl_register(&rts_pinctrl_desc, dev, rtspc);
	if (!rtspc->rtspctl_dev)
		goto remove_gpiochip;

	if (rtspc->devt == TYPE_RTS3903) {
		pp = rts_pintable_3903;
		len = ARRAY_SIZE(rts_pintable_3903);
	} else 	if (rtspc->devt == TYPE_RTS3913) {
		pp = rts_pintable_3913;
		len = ARRAY_SIZE(rts_pintable_3913);
	} else {
		pp = rts_pintable_3901;
		len = ARRAY_SIZE(rts_pintable_3901);
	}

	for (i = 0; i < len; i++) {
		err = gpiochip_add_pin_range(rtspc->gpio_chip,
			     pdev->name, pp[i].gpio_base,
			     pp[i].pin_base, pp[i].pins);
		if (err)
			goto remove_gpiochip;
	}

	return 0;

remove_gpiochip:
	dev_info(&pdev->dev, "Remove gpiochip\n");
	gpiochip_remove(rtspc->gpio_chip);

	kfree(rtspc->irq_type);
	devm_kfree(dev, rtspc);

	return err;

}

static int rts_pinctrl_remove(struct platform_device *pdev)
{
	struct rts_pinctrl *rtspc = platform_get_drvdata(pdev);

	free_irq(rtspc->irq, rtspc);

	pinctrl_unregister(rtspc->rtspctl_dev);

	gpiochip_remove(rtspc->gpio_chip);

	kfree(rtspc->irq_type);
	devm_kfree(&pdev->dev, rtspc);

	return 0;
}

static struct platform_driver rts_pinctrl_driver = {
	.probe = rts_pinctrl_probe,
	.remove = rts_pinctrl_remove,
	.driver = {
		   .name = "pinctrl_platform",
		   .owner = THIS_MODULE,
		   .of_match_table = rts_pinctrl_match,
	},
};

static int __init rts_pinctrl_init(void)
{
	return platform_driver_register(&rts_pinctrl_driver);
}
postcore_initcall(rts_pinctrl_init);

static void __exit rts_pinctrl_exit(void)
{
	platform_driver_unregister(&rts_pinctrl_driver);
}
module_exit(rts_pinctrl_exit);
