#
# Makefile for the Linux/MIPS kernel FPU emulation.
#

obj-$(CONFIG_CPU_HAS_EMU)	+= cp1emu.o ieee754dp.o ieee754sp.o ieee754.o \
	   dp_div.o dp_mul.o dp_sub.o dp_add.o dp_fsp.o dp_cmp.o dp_simple.o \
	   dp_tint.o dp_fint.o dp_maddf.o dp_2008class.o dp_fmin.o dp_fmax.o \
	   sp_div.o sp_mul.o sp_sub.o sp_add.o sp_fdp.o sp_cmp.o sp_simple.o \
	   sp_tint.o sp_fint.o sp_maddf.o sp_2008class.o sp_fmin.o sp_fmax.o \
	   dsemul.o

lib-$(CONFIG_CPU_HAS_EMU)	+= ieee754d.o \
	   dp_tlong.o dp_flong.o dp_sqrt.o \
	   sp_tlong.o sp_flong.o sp_sqrt.o

obj-$(CONFIG_DEBUG_FS) += me-debugfs.o
