dtb-$(CONFIG_DTB_RTS3903_EVB)          := rts3903_evb.dtb
dtb-$(CONFIG_DTB_RTS3904_EVB)          := rts3904_evb.dtb
dtb-$(CONFIG_DTB_RTS3903N_EVB)         := rts3903n_evb.dtb
dtb-$(CONFIG_DTB_RTS3904N_EVB)         := rts3904n_evb.dtb
dtb-$(CONFIG_DTB_RTS3903N_PTZ)         := rts3903n_ptz.dtb
dtb-$(CONFIG_DTB_RTS3913_FPGA)         := rts3913_fpga.dtb
dtb-$(CONFIG_DTB_RTS3913N_EVB)         := rts3913n_evb.dtb
dtb-$(CONFIG_DTB_RTS3914N_EVB)         := rts3914n_evb.dtb
dtb-$(CONFIG_DTB_RTS3913L_EVB)         := rts3913l_evb.dtb
dtb-$(CONFIG_DTB_RTS3903N_AONI)         := rts3903n_aoni.dtb

ifeq ($(CONFIG_MIPS_NO_APPENDED_DTB),y)
obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))
endif

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
