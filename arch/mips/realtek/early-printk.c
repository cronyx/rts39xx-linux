/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2015, Tony Wu <tonywu@realtek.com>
 */

#include <linux/io.h>
#include <linux/serial_reg.h>
#include <asm/addrspace.h>

#include "bspchip.h"

static __iomem void *uart_base = (__iomem void *) KSEG1ADDR(BSP_UART1_VADDR);

static inline void uart_w32(u32 val, unsigned reg)
{
	__raw_writel(val, uart_base + (reg << 2));
}

static inline u32 uart_r32(unsigned reg)
{
	return __raw_readl(uart_base + (reg << 2));
}

void early_uart_init(void)
{
	uart_w32(0x80, UART_LCR);
	uart_w32(0x0, UART_IER);
	/*
	 *  baud rate = (serial clock freq) / (16 * divisor)
	 *  baud rate = 57600
	 *  serial clock freq = 25MHz
	 */
	uart_w32(0x1b, UART_TX);
	uart_w32(0x3, UART_LCR);
	__raw_writel(CLK_ENABLE|UART_SELECT_USBPLL_5|CLOCK_SELECT_DIV2, UART_CLK_CFG_REG);
	__raw_writel(UART1_TX_EN, UART_TX_EN_REG);
}

void prom_putchar(unsigned char ch)
{
	while ((uart_r32(UART_LSR) & UART_LSR_THRE) == 0)
		;
	uart_w32(ch, UART_TX);
}
