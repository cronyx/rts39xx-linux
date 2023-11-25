/*
 * Realtek Semiconductor Corp.
 *
 * bsp/setup.c
 *     bsp interrupt initialization and handler code
 *
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 */
#include <linux/of_fdt.h>

#include <asm/prom.h>
#include <asm/time.h>
#include <asm/reboot.h>
#include <asm/fw/fw.h>
#include <asm/taroko-mmcr.h>

#include "bspchip.h"

#ifdef CONFIG_RTSX_WATCHDOG
extern void reboot_by_wdt(int new_timeout);
#else
#define reboot_by_wdt(new_timeout)
#endif

extern char __dtb_start[];
extern char __appended_dtb[];

static void plat_machine_restart(char *command)
{
	local_irq_disable();
	reboot_by_wdt(1);

	while (1) ;
}

static void plat_machine_halt(void)
{
	printk("System halted.\n");
	while(1);
}

#ifdef CONFIG_CPU_HAS_L2C
static void plat_l2c_enable(void)
{
	if (IS_ENABLED(CONFIG_SMP))
		MMCR_REG32(L2C_CTRL) = MMCR_REG32(L2C_CTRL) | 0x1;
	else {
		change_c0_cctl1(CCTL_L2CON, 1);
		change_c0_cctl1(CCTL_L2COFF, 0);
	}
}
#else
static void plat_l2c_enable(void)
{
}
#endif

static int memory_dtb;

static int __init early_init_dt_find_memory(unsigned long node,
				const char *uname, int depth, void *data)
{
	if (depth == 1 && !strcmp(uname, "memory@0"))
		memory_dtb = 1;

	return 0;
}

/* callback function */
void __init plat_mem_setup(void)
{
	unsigned long rts_memsize;

	/* define io/mem region */
	ioport_resource.start = 0x14000000;
	ioport_resource.end = 0x1fffffff;

	iomem_resource.start = 0x14000000;
	iomem_resource.end = 0x1fffffff;

	/* set reset vectors */
	_machine_restart = plat_machine_restart;
	_machine_halt = plat_machine_halt;
	pm_power_off = plat_machine_halt;

#ifdef CONFIG_BUILTIN_DTB
	/*
	 * Load the builtin devicetree. This causes the chosen node to be
	 * parsed resulting in our memory appearing
	 */
#ifdef CONFIG_MIPS_RAW_APPENDED_DTB
	__dt_setup_arch(&__appended_dtb);
#else
	__dt_setup_arch(&__dtb_start);
#endif
#else
	/*
	 * U-Boot will pass $a0 and $a1 to kernel. $a0 is the magic value, -2,
	 * to check whether $a1 is the dtb address or not.
	 */
	if (fw_arg0 == -2)
		__dt_setup_arch((void *)KSEG0ADDR(fw_arg1));
#endif

	rts_memsize = fw_getenvl("memsize");
	if (rts_memsize > 0) {
#define RTS_LOWMEM	(256 << 20)
		if (rts_memsize <= RTS_LOWMEM)
			add_memory_region(0, rts_memsize, BOOT_MEM_RAM);
		else {
			add_memory_region(0, RTS_LOWMEM, BOOT_MEM_RAM);
			if (IS_ENABLED(CONFIG_HIGHMEM)) {
#define RTS_HIGHMEM_START	(0x90000000)
				add_memory_region(RTS_HIGHMEM_START,
						(rts_memsize - RTS_LOWMEM),
						BOOT_MEM_RAM);
#undef RTS_HIGHMEM_START
			}
		}
#undef RTS_LOWMEM
	} else {
		of_scan_flat_dt(early_init_dt_find_memory, NULL);
		if (memory_dtb)
			of_scan_flat_dt(early_init_dt_scan_memory, NULL);
		else {
			/*
			 * Memory region is shown as follows:
			 *
			 * 0x00000000 - 0x14000000 - 0x20000000 => ....
			 *      dram (320MB)      mmio        highmem
			 */
#define SZ_LOWMEM	(320 << 20)
			if (cpu_mem_size <= SZ_LOWMEM)
				add_memory_region(0,
					cpu_mem_size, BOOT_MEM_RAM);
			else {
				add_memory_region(0, SZ_LOWMEM, BOOT_MEM_RAM);
				if (IS_ENABLED(CONFIG_HIGHMEM)) {
					add_memory_region(HIGHMEM_START,
						(cpu_mem_size - SZ_LOWMEM),
						BOOT_MEM_RAM);
				}
			}
#undef SZ_LOWMEM
		}
	}

	plat_l2c_enable();
}
