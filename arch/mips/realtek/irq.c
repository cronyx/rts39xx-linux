/*
 * Realtek Semiconductor Corp.
 *
 * bsp/irq.c
 *     platform interrupt init and handler code
 *     using device tree framework
 *
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 */
#include <linux/irqchip.h>

void __init plat_irq_init(void)
{
	irqchip_init();
}
