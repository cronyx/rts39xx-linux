core-y   += arch/mips/realtek/
cflags-y += -I$(srctree)/arch/mips/realtek/
load-y   += 0x80010000

obj-y := prom.o timer.o setup.o irq.o
obj-$(CONFIG_PCI) += pci.o pci-fixup.o pci-ops.o
obj-$(CONFIG_SMP) += smp.o
obj-$(CONFIG_EARLY_PRINTK) += early-printk.o
obj-$(CONFIG_MTD_SPI) += flash.o
obj-$(CONFIG_OF) += of.o
