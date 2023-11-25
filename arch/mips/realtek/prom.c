/*
 * Realtek Semiconductor Corp.
 *
 * bsp/prom.c
 *     bsp early initialization code
 *
 * Copyright (C) 2006-2015 Tony Wu (tonywu@realtek.com)
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/prom.h>

#include <asm/fw/fw.h>

const char *get_system_type(void)
{
	return "Formosa";
}

void __init prom_free_prom_memory(void)
{
	return;
}

/* Do basic initialization */
void __init prom_init(void)
{
	extern void plat_smp_init(void);
	extern void early_uart_init(void);

	fw_init_cmdline();

#ifdef CONFIG_SERIAL_8250_CONSOLE
	if (!strstr(arcs_cmdline, "console=")) {
		strlcat(arcs_cmdline, " console=ttyS0,57600",
			COMMAND_LINE_SIZE);
	}
#endif

	mips_set_machine_name("Sheipa Platform");

#ifdef CONFIG_EARLY_PRINTK
	early_uart_init();
#endif

#ifdef CONFIG_SMP
	plat_smp_init();
#endif
}
