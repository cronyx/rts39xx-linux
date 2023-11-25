/*
 * Realtek Semiconductor Corp.
 *
 * bsp/smp.c
 *     bsp SMP initialization code
 *
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 */
#include <linux/smp.h>
#include <linux/interrupt.h>

#include <asm/smp-ops.h>
#include <asm/taroko-mmcr.h>

#include "bspchip.h"

/*
 * Called in kernel/smp-*.c to do secondary CPU initialization.
 *
 * All platform-specific initialization should be implemented in
 * this function.
 *
 * Known SMP callers are:
 *     kernel/smp-cmp.c
 *
 * This function is called by secondary CPUs.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
void __cpuinit plat_smp_init_secondary(void)
#else
void plat_smp_init_secondary(void)
#endif
{
	change_c0_status(ST0_IM, 0xff00);
	change_lxc0_estatus(EST0_IM, 0xff0000);
}

/*
 * Called in bsp/setup.c to initialize SMP operations
 *
 * Depends on SMP type, plat_smp_init calls corresponding
 * SMP operation initializer in arch/mips/kernel
 *
 * Known SMP types are:
 *     CONFIG_SMP_CMP
 */
void __init plat_smp_init(void)
{
	mmcr_init_core(0);
	register_cmp_smp_ops();
}
