/*
 * Realtek Semiconductor Corp.
 *
 * bsp/gpio.h:
 *
 * Copyright (C) 2014 Wei WANG (wei_wang@realsil.com.cn)
 */
#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H

enum rts_pinconf_pull {
	RTS_PINCONFIG_DUMMY_OP0,
	RTS_PINCONFIG_PULL_NONE,
	RTS_PINCONFIG_DUMMY_OP1,
	RTS_PINCONFIG_PULL_DOWN,
	RTS_PINCONFIG_DUMMY_OP2,
	RTS_PINCONFIG_PULL_UP,
	RTS_PIN_CONFIG_DRIVE_STRENGTH,
	RTS_PIN_CONFIG_SLEW_RATE,
	RTS_PINCONFIG_INPUT,
	RTS_PINCONFIG_OUTPUT,
};

#ifdef CONFIG_GPIOLIB
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep
#define gpio_to_irq __gpio_to_irq
#else
int gpio_request(unsigned int gpio, const char *label);
void gpio_free(unsigned int gpio);
int gpio_direction_input(unsigned int gpio);
int gpio_direction_output(unsigned int gpio, int value);
int gpio_get_value(unsigned int gpio);
void gpio_set_value(unsigned int gpio, int value);
#endif

#include <asm-generic/gpio.h>		/* cansleep wrappers */


#endif /* _BSP_GPIO_H */
