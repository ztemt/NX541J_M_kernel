/*
 *  Richtek Charger Interface for Mediatek
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  ShuFanLee <shufan_lee@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __MTK_CHARGER_INTF_H
#define __MTK_CHARGER_INTF_H

#include <mt-plat/charging.h>
#include <linux/types.h>


/* MTK charger interface */
extern int(*mtk_charger_intf[CHARGING_CMD_NUMBER])(void *data);

/*
 * The following interface are not related to charger
 * They are implemented in mtk_charger_intf.c
 */
extern int mtk_charger_sw_init(void *data);
extern int mtk_charger_set_hv_threshold(void *data);
extern int mtk_charger_get_hv_status(void *data);
extern int mtk_charger_get_battery_status(void *data);
extern int mtk_charger_get_charger_det_status(void *data);
extern int mtk_charger_get_charger_type(void *data);
extern int mtk_charger_get_is_pcm_timer_trigger(void *data);
extern int mtk_charger_set_platform_reset(void *data);
extern int mtk_charger_get_platform_boot_mode(void *data);
extern int mtk_charger_set_power_off(void *data);
extern int mtk_charger_get_power_source(void *data);
extern int mtk_charger_get_csdac_full_flag(void *data);
extern int mtk_charger_set_error_state(void *data);
extern int mtk_charger_diso_init(void *data);
extern int mtk_charger_get_diso_state(void *data);
extern int mtk_charger_set_vbus_ovp_en(void *data);
extern int mtk_charger_get_bif_vbat(void *data);
extern int mtk_charger_set_chrind_ck_pdn(void *data);
extern int mtk_charger_get_bif_tbat(void *data);
extern int mtk_charger_set_dp(void *data);
extern int mtk_charger_get_bif_is_exist(void *data);

#endif /* __MTK_CHARGER_INTF_H */
