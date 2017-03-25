
ccflags-y += -I$(srctree)/drivers/misc/mediatek/usb_c/inc

obj-$(CONFIG_DUAL_ROLE_USB_INTF)	+= tcpci_dual_role.o
obj-$(CONFIG_TCPC_RT1711H)		+= tcpc_rt1711h.o
obj-$(CONFIG_TCPC_RT1711)		+= tcpc_rt1711.o
obj-$(CONFIG_TCPC_CLASS)		+= tcpci_core.o tcpci_typec.o\
						tcpci_alert.o\
						tcpci_timer.o tcpm.o\
						pd_dbg_info.o tcpc_usb_cb.o
