#
# This file is subject to the terms and conditions of the GNU General Public
# License.
#
# Adapted for MIPS Pete Popov, Dan Malek
#
# Copyright (C) 1994 by Linus Torvalds
# Adapted for PowerPC by Gary Thomas
# modified by Cort (cort@cs.nmt.edu)
#
# Copyright (C) 2009 Lemote Inc. & DSLab, Lanzhou University
# Author: Wu Zhangjin <wuzhangjin@gmail.com>
#

include $(srctree)/arch/mips/Kbuild.platforms

# set the default size of the mallocing area for decompressing
BOOT_HEAP_SIZE := 0x400000

# Disable Function Tracer
KBUILD_CFLAGS := $(shell echo $(KBUILD_CFLAGS) | sed -e "s/-pg//")

KBUILD_CFLAGS := $(filter-out -fstack-protector, $(KBUILD_CFLAGS))

KBUILD_CFLAGS := $(LINUXINCLUDE) $(KBUILD_CFLAGS) -D__KERNEL__ \
	-DBOOT_HEAP_SIZE=$(BOOT_HEAP_SIZE) -D"VMLINUX_LOAD_ADDRESS_ULL=$(VMLINUX_LOAD_ADDRESS)ull"

KBUILD_AFLAGS := $(LINUXINCLUDE) $(KBUILD_AFLAGS) -D__ASSEMBLY__ \
	-DBOOT_HEAP_SIZE=$(BOOT_HEAP_SIZE) \
	-DKERNEL_ENTRY=$(VMLINUX_ENTRY_ADDRESS)

# decompressor objects (linked with vmlinuz)
vmlinuzobjs-y := $(obj)/head.o $(obj)/decompress.o $(obj)/string.o

ifdef CONFIG_DEBUG_ZBOOT
vmlinuzobjs-$(CONFIG_DEBUG_ZBOOT)		   += $(obj)/dbg.o
vmlinuzobjs-$(CONFIG_SYS_SUPPORTS_ZBOOT_UART16550) += $(obj)/uart-16550.o
vmlinuzobjs-$(CONFIG_SYS_SUPPORTS_ZBOOT_UART_PROM) += $(obj)/uart-prom.o
vmlinuzobjs-$(CONFIG_MIPS_ALCHEMY)		   += $(obj)/uart-alchemy.o
vmlinuzobjs-$(CONFIG_ATH79)			   += $(obj)/uart-ath79.o
vmlinuzobjs-$(CONFIG_REALTEK_RTS_COMMON)	   += $(obj)/uart-rts.o
endif

extra-y += uart-ath79.c
$(obj)/uart-ath79.c: $(srctree)/arch/mips/ath79/early_printk.c
	$(call cmd,shipped)

extra-y += uart-rts.c
$(obj)/uart-rts.c: $(srctree)/arch/mips/realtek/common/early-printk.c
	$(call cmd,shipped)

vmlinuzobjs-y += $(obj)/ashldi3.o $(obj)/bswapsi.o

extra-y += ashldi3.c bswapsi.c
$(obj)/ashldi3.o $(obj)/bswapsi.o: KBUILD_CFLAGS += -I$(srctree)/arch/mips/lib
$(obj)/ashldi3.c $(obj)/bswapsi.c: $(obj)/%.c: $(srctree)/arch/mips/lib/%.c
	$(call cmd,shipped)

targets := $(notdir $(vmlinuzobjs-y))

targets += vmlinux.bin
OBJCOPYFLAGS_vmlinux.bin := $(OBJCOPYFLAGS) -O binary -R .comment -S
$(obj)/vmlinux.bin: $(KBUILD_IMAGE) FORCE
	$(call if_changed,objcopy)

tool_$(CONFIG_KERNEL_GZIP)    = gzip
tool_$(CONFIG_KERNEL_BZIP2)   = bzip2
tool_$(CONFIG_KERNEL_LZ4)     = lz4
tool_$(CONFIG_KERNEL_LZMA)    = lzma
tool_$(CONFIG_KERNEL_LZO)     = lzo
tool_$(CONFIG_KERNEL_XZ)      = xzkern

targets += vmlinux.bin.z
$(obj)/vmlinux.bin.z: $(obj)/vmlinux.bin FORCE
	$(call if_changed,$(tool_y))

targets += piggy.o dummy.o
OBJCOPYFLAGS_piggy.o := --add-section=.image=$(obj)/vmlinux.bin.z \
			--set-section-flags=.image=contents,alloc,load,readonly,data
$(obj)/piggy.o: $(obj)/dummy.o $(obj)/vmlinux.bin.z FORCE
	$(call if_changed,objcopy)

# Calculate the load address of the compressed kernel image
hostprogs-y := calc_vmlinuz_load_addr

ifneq ($(zload-y),)
VMLINUZ_LOAD_ADDRESS := $(zload-y)
else
VMLINUZ_LOAD_ADDRESS = $(shell $(obj)/calc_vmlinuz_load_addr \
		$(obj)/vmlinux.bin $(VMLINUX_LOAD_ADDRESS))
endif

ifeq ($(CONFIG_SOC_SHEIPA),y)
UIMAGE_LOADADDR = $(VMLINUZ_LOAD_ADDRESS)
endif

vmlinuzobjs-y += $(obj)/piggy.o

quiet_cmd_zld = LD      $@
      cmd_zld = $(LD) $(LDFLAGS) -Ttext $(VMLINUZ_LOAD_ADDRESS) -T $< $(vmlinuzobjs-y) -o $@
quiet_cmd_strip = STRIP	  $@
      cmd_strip = $(STRIP) -s $@
vmlinuz: $(src)/ld.script $(vmlinuzobjs-y) $(obj)/calc_vmlinuz_load_addr
	$(call cmd,zld)
	$(call cmd,strip)

#
# Some DECstations need all possible sections of an ECOFF executable
#
ifdef CONFIG_MACH_DECSTATION
  e2eflag := -a
endif

# elf2ecoff can only handle 32bit image
hostprogs-y += ../elf2ecoff

ifdef CONFIG_32BIT
	VMLINUZ = vmlinuz
else
	VMLINUZ = vmlinuz.32
endif

quiet_cmd_32 = OBJCOPY $@
      cmd_32 = $(OBJCOPY) -O $(32bit-bfd) $(OBJCOPYFLAGS) $< $@
vmlinuz.32: vmlinuz
	$(call cmd,32)

quiet_cmd_ecoff = ECOFF	  $@
      cmd_ecoff = $< $(VMLINUZ) $@ $(e2eflag)
vmlinuz.ecoff: $(obj)/../elf2ecoff $(VMLINUZ)
	$(call cmd,ecoff)

OBJCOPYFLAGS_vmlinuz.bin := $(OBJCOPYFLAGS) -O binary
vmlinuz.bin: vmlinuz
	$(call cmd,objcopy)

OBJCOPYFLAGS_vmlinuz.srec := $(OBJCOPYFLAGS) -S -O srec
vmlinuz.srec: vmlinuz
	$(call cmd,objcopy)

ifeq ($(CONFIG_SOC_SHEIPA),y)

append_dtb-$(CONFIG_DTB_RTS3903_EVB)          := rts3903_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3904_EVB)          := rts3904_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3903N_EVB)         := rts3903n_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3904N_EVB)         := rts3904n_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3903N_PTZ)         := rts3903n_ptz.dtb
append_dtb-$(CONFIG_DTB_RTS3913_FPGA)         := rts3913_fpga.dtb
append_dtb-$(CONFIG_DTB_RTS3913N_EVB)         := rts3913n_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3914N_EVB)         := rts3914n_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3913L_EVB)         := rts3913l_evb.dtb
append_dtb-$(CONFIG_DTB_RTS3903N_AONI)         := rts3903n_aoni.dtb

to_append.dtb: $(srctree)/arch/mips/boot/dts/realtek/$(append_dtb-y) FORCE
	@echo "shipping $(append_dtb-y)"
	$(call cmd,shipped)

vmlinuz.bin_w_dtb: vmlinuz.bin to_append.dtb FORCE
	cat $(filter-out FORCE,$^) > $@

vmlinuz.img: $(if $(CONFIG_MIPS_RAW_APPENDED_DTB),vmlinuz.bin_w_dtb,vmlinuz.bin)
	$(call if_changed,uimage,none)

endif

clean-files := $(objtree)/vmlinuz $(objtree)/vmlinuz.{32,ecoff,bin,srec} $(objtree)/vmlinuz.bin $(objtree)/vmlinuz.img $(objtree)/to_append.dtb $(objtree)/vmlinuz.bin_w_dtb
