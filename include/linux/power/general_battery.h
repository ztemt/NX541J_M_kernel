#ifndef __BATTERYstaticH__
#define __BATTERYstaticH__

//#include <mach/mt_typedefs.h>
/* ============================================================ */
/* ENUM */
/* ============================================================ */
typedef enum
{
    BATTERY_COMMON_CMD_HW_INIT,
    BATTERY_COMMON_CMD_HW_RESET,	/* FGADC_Reset_SW_Parameter */
	BATTERY_COMMON_CMD_DUMP_REGISTER,

    BATTERY_COMMON_CMD_GET_BATT_VOLT,
    BATTERY_COMMON_CMD_GET_BATT_TEMP,
    BATTERY_COMMON_CMD_GET_BATT_CURRENT,
	BATTERY_COMMON_CMD_GET_BATT_SOC,
    BATTERY_COMMON_CMD_GET_BATT_RMC,
	BATTERY_COMMON_CMD_GET_BATT_FCC,
    BATTERY_COMMON_CMD_GET_BATT_DCC,
	BATTERY_COMMON_CMD_SET_BATT_TEMP,

    BATTERY_COMMON_CMD_UPDATE_BATT_PROFILE,
    BATTERY_COMMON_CMD_GET_CHGR_VOLT,
	BATTERY_COMMON_CMD_GET_CYCLE_COUNT,
    BATTERY_COMMOM_CMD_NUMBER
} BATTERY_COMMON_CTRL_CMD;

typedef struct {
        int t_batt;
        int r_batt;
} BATT_TEMPERATURE_T;

typedef int (*fg_ctrl_fn) (BATTERY_COMMON_CTRL_CMD cmd, void *data);


#define BATTERY_MAX_FLOAT_VOLTAGE			4400
#define BATTERY_CUTOFF_VOLTAGE				3400

#endif