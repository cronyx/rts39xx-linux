/*
 * Realtek Semiconductor Corp.
 *
 * bsp/flash.c
 *     flash setup code
 *
 * Copyright (C) 2015 Tony Wu (tonywu@realtek.com)
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <mtd/mtd-abi.h>

#include "bspchip.h"

unsigned int *plat_spi_base = (unsigned int *)BSP_SPIC_PADDR;

static struct mtd_partition flash_partitions[] = {
	{
		.name =         "boot",
		.offset =       0x0,
		.size =         0x100000,
	},
	{
		.name =         "rootfs",
		.offset =       0x100000,
		.size =         0x100000,
	},
};

static struct physmap_flash_data flash_data = {
	.width          = 4,
	.nr_parts       = ARRAY_SIZE(flash_partitions),
	.parts          = flash_partitions
};

static struct resource flash_resource = {
	.start          = BSP_FLASH_MAPBASE,
	.end            = BSP_FLASH_MAPBASE + BSP_FLASH_MAPSIZE - 1,
	.flags          = IORESOURCE_MEM
};

static struct platform_device flash_device = {
	.name           = "physmap-flash",
	.id             = 0,
	.dev            = {
	        .platform_data  = &flash_data,
	},
	.num_resources  = 1,
	.resource       = &flash_resource,
};

static int __init plat_flash_init(void)
{
	int err;

	err = platform_device_register(&flash_device);
	if (err)
		return err;

	return 0;
}
arch_initcall(plat_flash_init);
