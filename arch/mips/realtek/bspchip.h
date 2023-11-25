/*
 * Realtek Semiconductor Corp.
 *
 * bsp/bspchip.h:
 *   bsp chip address and IRQ mapping file
 *
 * Copyright (C) 2006-2015 Tony Wu (tonywu@realtek.com)
 */

#ifndef _BSPCHIP_H_
#define _BSPCHIP_H_

#include <linux/version.h>

#include <asm/mach-generic/irq.h>

/*
 * FREQ
 */
#define BSP_PLL_FREQ		25000000	   /* 25 MHz */
#define BSP_CPU_FREQ		(2 * BSP_PLL_FREQ) /* 50 MHz */

/*
 * Clock
 */
#define SYSBASE_VA			0xb8860000
#define SYSBASE_REG(x)			((void __iomem *)(x) + SYSBASE_VA)

#define CLK_CHANGE		SYSBASE_REG(0x1000)
#define BUS_CLK_CHANGE			0x7
#define XB2_CLK_CHANGE			0x6
#define CPU_CLK_CHANGE			0x5
#define DRAM_CLK_CHANGE		0x4
#define CLK_CHANGE_NULL			0

#define FORCE_REG_RESET_FWC		SYSBASE_REG(0x04)
#define FORCE_REG_ASYNC_RESET		SYSBASE_REG(0X08)
#define RST_LOAD_MODE_REG		SYSBASE_REG(0X28)

#define LEGACY_SPI_NOR			0x0
#define RST_LOAD			0x2

#define WORK_MODE			8
#define WORK_MODE_MASK			(0x3 << 8)
#define RST_LOAD_MODE			4
#define RST_LOAD_MODE_MASK		(0xf << 4)

#define RST_LOAD_SPI_NOR		0x0
#define RST_LOAD_SPI_NAND		0x1
#define RST_LOAD_EMMC			0x2
#define RST_LOAD_SPI_NOR_JTAG		0x3
#define RST_LOAD_SPI_NOR_DOWNLOAD	0x4
#define RST_LOAD_SPI_NAND_DOWNLOAD	0x5
#define RST_LOAD_EMMC_DOWNLOAD		0x6
#define RST_LOAD_SPI_NAND_JTAG		0x8
#define RST_LOAD_MDIO			0x9
#define RST_LOAD_EMMC_DBGROM		0xa
#define RST_LOAD_SPI_NOR_EXTCLK		0xb
#define RST_LOAD_EMMC_JTAG		0xf

#define UART_CLK_LP_EN			SYSBASE_REG(0x1004)
#define MCU_CLK_CFG_REG		SYSBASE_REG(0x1008)
#define DRAM_CLK_CFG_REG		SYSBASE_REG(0x100c)
#define CPU_CLK_CFG_REG		SYSBASE_REG(0x1010)
#define XB2_CLK_CFG_REG		SYSBASE_REG(0x1014)
#define BUS_CLK_CFG_REG		SYSBASE_REG(0x1018)
#define I2S_CLK_CFG_REG		SYSBASE_REG(0x101c)
#define CIPHER_CLK_CFG_REG		SYSBASE_REG(0x1020)
#define ETHERNET_CLK_CFG_REG		SYSBASE_REG(0x1024)
#define UART_CLK_CFG_REG		SYSBASE_REG(0x1028)
#define I2C_CLK_CFG_REG		SYSBASE_REG(0x102c)
#define H264_CLK_CFG_REG		SYSBASE_REG(0x1030)
#define RC_CLK_CFG			SYSBASE_REG(0x1034)
#define RC_OSC_POW_CFG			SYSBASE_REG(0x1038)
#define RTC32K_DIV_CFG0		SYSBASE_REG(0x103c)
#define RTC32K_DIV_CFG1		SYSBASE_REG(0x1040)
#define RTC32K_DIV_CFG2		SYSBASE_REG(0x1044)
#define RTC_CLK_CFG			SYSBASE_REG(0x1048)
#define USBPHY_CLK_CFG			SYSBASE_REG(0x104c)
#define JPEG_CLK_CFG_REG		SYSBASE_REG(0x1050)
#define PLL_BYPASS_CFG_REG		SYSBASE_REG(0x1054)
#define ISP_SCAN_CLK_CFG_REG		SYSBASE_REG(0x1058)
#define MIPI_SCAN_CLK_CFG_REG		SYSBASE_REG(0x105c)
#define SPDIF_CLK_CFG_REG		SYSBASE_REG(0x1060)
#define PLATFORM_CONFIGURATION		SYSBASE_REG(0x1064)
#define CODEC_CLK_CFG_REG		SYSBASE_REG(0x1068)
#define VIDEO_CLK_SEL_REG		SYSBASE_REG(0x1068)

#define CLK_ENABLE			0x1000000
#define UART_SELECT_USBPLL_5    0x0
#define UART_SELECT_PLL0_5    0x01
#define UART_SELECT_PLL1_5    0x02

#define DRAMC_SELECT_USBPLL_2	0
#define DRAMC_SELECT_PLL0_2	0x01
#define DRAMC_SELECT_PLL1_3	0x02
#define DRAMC_SELECT_PLL1_2	0x03

#define CPU_SELECT_USBPLL_3	0
#define CPU_SELECT_PLL0_3	0x01
#define CPU_SELECT_PLL0_2	0x02
#define CPU_SELECT_PLL1_2	0x03

#define XB2_SELECT_USBPLL_2	0
#define XB2_SELECT_PLL0_5	0x01
#define XB2_SELECT_PLL1_2	0x02

#define BUS_SELECT_USBPLL_2	0
#define BUS_SELECT_PLL0_5	0x01
#define BUS_SELECT_PLL1_2	0x02

#define CLOCK_SELECT_DIV1		(0x0<<2)
#define CLOCK_SELECT_DIV2		(0x1<<2)
#define CLOCK_SELECT_DIV4		(0x2<<2)
#define CLOCK_SELECT_DIV6		(0x3<<2)
#define CLOCK_SELECT_DIV8		(0x4<<2)
#define CLOCK_SELECT_DIV10		(0x5<<2)
#define CLOCK_SELECT_DIV12		(0x6<<2)
#define CLOCK_SELECT_DIV14		(0x7<<2)

/*XB2*/
#define XB2_VA				0xB8870000
#define	XB2BASE_REG(x)			((x) + XB2_VA)
#define	UART_TX_EN_REG			XB2BASE_REG(0x54)
#define UART0_TX_EN			0x01
#define UART1_TX_EN			(0x01<<8)
#define UART2_TX_EN			(0x01<<16)

/*
 * DWAPB UART
 */
#define BSP_UART0_VADDR		0xb8810000UL
#define BSP_UART0_BAUD		57600
#define BSP_UART0_FREQ		24000000
#define BSP_UART0_USR		(BSP_UART0_VADDR + 0x7c)
#define BSP_UART0_FCR		(BSP_UART0_VADDR + 0x08)
#define BSP_UART0_PADDR		0x18810000L
#define BSP_UART0_PSIZE		0x100

#define BSP_UART1_VADDR		0xb8810100UL
#define BSP_UART1_BAUD		115200//57600
#define BSP_UART1_FREQ		24000000
#define BSP_UART1_USR		(BSP_UART1_VADDR + 0x7c)
#define BSP_UART1_FCR		(BSP_UART1_VADDR + 0x08)
#define BSP_UART1_PADDR		0x18810100L
#define BSP_UART1_PSIZE		0x100

#define BSP_UART2_VADDR		0xb8810200UL
#define BSP_UART2_BAUD		57600
#define BSP_UART2_FREQ		24000000
#define BSP_UART2_USR		(BSP_UART2_VADDR + 0x7c)
#define BSP_UART2_FCR		(BSP_UART2_VADDR + 0x08)
#define BSP_UART2_PADDR		0x18810200L
#define BSP_UART2_PSIZE		0x100

/*
 * DWAPB Timer
 */
#define BSP_TIMER_FREQ		(BSP_PLL_FREQ)
#define BSP_TIMER_VADDR		0xbfb01000UL
#define BSP_TIMER_TLCR		(BSP_TIMER_VADDR + 0x00)
#define BSP_TIMER_TCVR		(BSP_TIMER_VADDR + 0x04)
#define BSP_TIMER_TCR		(BSP_TIMER_VADDR + 0x08)
#define BSP_TIMER_EOI		(BSP_TIMER_VADDR + 0x0c)

/*
 * LS GMAC
 */
#define BSP_GMAC_PADDR		0x1b007000UL
#define BSP_GMAC_PSIZE		0x00000400UL

/*
 * LS PCIe
 */
#define BSP_PCIE_RC_CFG		0xbb000000UL
#define BSP_PCIE_EP_CFG		0xb9010000UL
#define BSP_PCIE_IO_PADDR	0x19200000UL
#define BSP_PCIE_IO_PSIZE	0x00200000UL
#define BSP_PCIE_MEM_PADDR	0x19400000UL
#define BSP_PCIE_MEM_PSIZE	0x00c00000UL

/*
 * LS USB3
 */
#define BSP_XHCI_PADDR		0x18000000UL
#define BSP_XHCI_PSIZE		0x00100000UL

/*
 * LS SMU
 */
#define BSP_SMU_VADDR		0xbb007800UL
#define BSP_SMU_PADDR		0x1b007800UL
#define BSP_SMU_PSIZE		0x00000800UL

/*
 * Register access macro
 */
#ifndef REG32
#define REG32(reg)	(*(volatile unsigned int   *)(reg))
#endif
#ifndef REG16
#define REG16(reg)	(*(volatile unsigned short *)(reg))
#endif
#ifndef REG08
#define REG08(reg)	(*(volatile unsigned char  *)(reg))
#endif

/* ETHERNET_CLK_CFG_REG */
#define AFE_POW_STATE			(1 << 18)
#define CLK_EN_ETN_250M			(1 << 17)
#define ETH_EPHY_RST_N			(1 << 14)
#define ETH_EPHY_ADDR			(1 << 9)

/* USBPHY_CLK_CFG_REG */
#define USBPHY_HOST_CLK_EN		(1 << 1)
#define USBPHY_DEV_CLK_EN		(1 << 0)

/* pll clk reg */
#define BSP_CLK_PLL0_BASE		SYSBASE_REG(0x4100)
#define BSP_CLK_PLL1_BASE		SYSBASE_REG(0x4200)
#define BSP_CLK_PLL2_BASE		SYSBASE_REG(0x4300)
#define BSP_CLK_PLL3_BASE		SYSBASE_REG(0x4400)

#define xb2flush()				\
	__asm__ __volatile__(			\
		".set	push\n\t"		\
		".set	noreorder\n\t"		\
		"sync\n\t"			\
		"lw	$0,%0\n\t"		\
		"nop\n\t"			\
		".set	pop"			\
		: /* no output */		\
		: "m" (*(int *)CKSEG1ADDR(0x18800000)) \
		: "memory")

#endif   /* _BSPCHIP_H */
