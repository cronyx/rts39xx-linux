#ifndef _DT_BINDINGS_INTERRUPT_CONTROLLER_RTS3913_H
#define _DT_BINDINGS_INTERRUPT_CONTROLLER_RTS3913_H

/*
 * IRQ Controller
 */
//#define BSP_IRQ_CPU_BASE	0
#define BSP_IRQ_CPU_NUM		8
#define BSP_IRQ_LOPI_NUM	8

/*
 * IRQ Mapping
 *
 * If there is GIC, IRQs are routed through GIC via software
 * If not, IRQs are hardwired via ICTL
 */
#ifndef CONFIG_IRQ_GIC

#define BSP_IRQ_ENCPY			(3) /* AES&DES&3DES*/
#define BSP_IRQ_DMA			(4) /* DMA controller */
#define BSP_IRQ_SPI			(5) /* DDR & SPI controller */
#define BSP_IRQ_OTHERS			(6) /* XB2/I2C host */
#define BSP_IRQ_USBDEV			(7) /* USB device */

/*
 * LOPI IRQ offset
 */
#define BSP_IRQ_SD			(0) /* SD controller */
#define BSP_IRQ_ETHERNET		(1) /* ethernet */
#define BSP_IRQ_USBHOST			(2) /* USB host */
#define BSP_IRQ_H265			(3) /* H264 controller */
#define BSP_IRQ_MCU8051			(4) /* DW8051&ISP&JPEG */
#define BSP_IRQ_I2S			(5) /* MIC&SPK&I2S */
#define BSP_IRQ_COMPARE			(6) /* cpu timer */
#define BSP_IRQ_TIMER			(7) /* xb2 timer */

#endif

/*
 * XB2 IRQ offset
 */
#define MDIO_RD_DONE_INT_IRQ		10
#define MDIO_WR_DONE_INT_IRQ		9
#define SARADC_DONE_INT_IRQ		8
#define RTC_ALARM3_INT_IRQ		7
#define RTC_ALARM2_INT_IRQ		6
#define RTC_ALARM1_INT_IRQ		5
#define RTC_ALARM0_INT_IRQ		4
#define PWM3_DONE_INT_IRQ		3
#define PWM2_DONE_INT_IRQ		2
#define PWM1_DONE_INT_IRQ		1
#define PWM0_DONE_INT_IRQ		0

#endif
