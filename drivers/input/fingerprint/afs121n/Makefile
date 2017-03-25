# MicroArray Fprint Driver Code
# MAKEFILE
# Date: 2016-09-10
# Version: v4.1.2
# Author: czl&cpy&guq
# Contact: [czl|cpy|guq]@microarray.com.cn

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)

#ifeq ($(CONFIG_MICROARRAY_FP),y)

#ifeq ($(XXXX_TEE), y)
#obj-y	+= madev_tee.o mtk-settings.o
#else
#obj-y	+= madev.o mtk-settings.o
#endif

#endif

obj-y += madev_tee.o mtk-settings.o

