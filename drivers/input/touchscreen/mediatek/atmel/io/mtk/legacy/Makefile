

ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/atmel/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/atmel/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/atmel/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y += io.o


KBUILD_CFLAGS   += -DCONFIG_MXT_PLUGIN_SUPPORT -DCONFIG_MXT_PLIGIN_CAL -DCONFIG_MXT_PLIGIN_AC -DCONFIG_MXT_PLIGIN_PI -DCONFIG_MXT_PLIGIN_MISC -DCONFIG_MXT_PLIGIN_CLIP -DCONFIG_MXT_PLIGIN_WDG

#ifdef CONFIG_MXT_DEBUG_ENABLE
#KBUILD_CFLAGS	+= -DDEBUG
#endif

#MAKEFLAGS += --include-dir=

