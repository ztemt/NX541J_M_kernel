#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat

obj-$(CONFIG_NUBIA_GENERAL_CHARGE_COMMON) += general_charger.o general_battery.o 

obj-$(CONFIG_TI_BQ27520_FG_SUPPORT) += bq27520_battery.o

obj-$(CONFIG_TI_BQ25890_CHARGER_SUPPORT) += bq25890_charger.o

obj-$(CONFIG_TI_BQ25896_CHARGER_SUPPORT) += bq25896_charger.o
