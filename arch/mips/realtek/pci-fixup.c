/*
 * Realtek Semiconductor Corp.
 *
 * bsp/pci-fixup.c
 *
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 */
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/kernel.h>

#include <asm/io.h>

#include "bspchip.h"

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	unsigned int val;
	int rc_payload_size, ep_payload_size;

	val = REG32(0xbb004000);
	if (val != 0x3)
		return 0;

	val = REG32(BSP_PCIE_EP_CFG+0x78);
	printk("INFO: Address %lx = 0x%x\n", BSP_PCIE_EP_CFG + 0x78, val);

	rc_payload_size = REG32(BSP_PCIE_RC_CFG + 0x74) & 0x7;
	ep_payload_size = REG32(BSP_PCIE_EP_CFG + 0x74) & 0x7;
	REG32(BSP_PCIE_EP_CFG + 0x78) = 0x105010 | (rc_payload_size <= ep_payload_size ?
					rc_payload_size: ep_payload_size) << 5;
	printk("INFO: Set PCIE payload to 128\n");

	val = REG32(BSP_PCIE_EP_CFG+0x78);
	printk("INFO: Address %lx = 0x%x\n",BSP_PCIE_EP_CFG + 0x78, val);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
#else
int pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
#endif
{
	int irq;

	printk("INFO: map_irq: slot=%d, pin=%d\n", slot, pin);

	if (pin == 0)
		return -1;

	irq = BSP_IRQ_PCIE - 1 + pin;
	return irq;
}
