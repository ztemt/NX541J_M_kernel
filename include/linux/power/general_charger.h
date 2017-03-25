#ifndef __CHARGER_COMMON_H__
#define __CHARGER_COMMON_H__

/******************************************************************************
 * Charging Parameter Define.
 *****************************************************************************/
#define USB_INVALID_CHARGER_CURRENT			100
#define USB_SDP_LIMIT_MAX_CURRENT			500
#define USB_DCP_LIMIT_MAX_CURRENT			2000
#define USB_HVDCP_LIMIT_MAX_CURRENT         2000
#define USB_MAX_CHARGE_CURRENT				2800

#define PARALLEL_LIMT_CURRENT_PERCENTAGE    50
#define PARALLEL_AICL_CURRENT_PERCENTAGE    50

#define BATTERY_RECHAEGE_THRESHOLD			4250

/** 0.1 degree   eg:470 = 47C */
#define CHG_TEMP_MAX    	                700
#define CHG_TEMP_HOT    	                500
#define CHG_TEMP_WARM   	                450
#define CHG_TEMP_GOOD   	                230
#define CHG_TEMP_COOL   	                100
#define CHG_TEMP_COLD   	                -50
#define CHG_TEMP_MIN   		                -200

#define BATT_COLD_CURRENT                   0
#define BATT_GOOD_CURRENT                   2800
#define BATT_COOL_CURRENT                   1200
#define BATT_WARM_CURRENT                   1200
#define BATT_HOT_CURRENT                    0

/** Charger Control Command **/
typedef enum
{
    CHARGER_COMMON_CMD_INIT,
    CHARGER_COMMON_CMD_DUMP_REGISTER,
    CHARGER_COMMON_CMD_ENABLE,
    CHARGER_COMMON_CMD_SET_CV_VOLTAGE,
    CHARGER_COMMON_CMD_GET_CHARGING_CURRENT,
    CHARGER_COMMON_CMD_SET_CHARGING_CURRENT,
    CHARGER_COMMON_CMD_GET_CHARGING_STATUS,
    CHARGER_COMMON_CMD_GET_INPUT_CURRENT,
    CHARGER_COMMON_CMD_SET_INPUT_CURRENT,
    CHARGER_COMMON_CMD_GET_BATTERY_VOLTAGE,
    CHARGER_COMMON_CMD_GET_BATTERY_PRESENT,
    CHARGER_COMMON_CMD_GET_CHARGER_VOLTAGE,
    CHARGER_COMMON_CMD_GET_CHARGER_PRESENT,
    CHARGER_COMMON_CMD_GET_CHARGER_TYPE,
    CHARGER_COMMON_CMD_SET_HIZ_SWTCHR,
    CHARGER_COMMON_CMD_SET_OTG_STATUS,
    CHARGER_COMMON_CMD_GET_OTG_STATUS,
    CHARGER_COMMON_CMD_NUMBER
} CHARGER_COMMON_CTRL_CMD;

typedef int (*chg_ctrl_fn) (CHARGER_COMMON_CTRL_CMD cmd, void *data);

typedef enum
{
    USB_INVALID_CHARGER = 0,
    USB_SDP_CHARGER,		/* USB : 450mA */
    USB_CDP_CHARGER,
    USB_DCP_CHARGER,
    USB_HVDCP_CHARGER,
    USB_UNKNOW_CHARGER,
    USB_DIVIDER_CHARGER,
} CHARGER_COMMON_TYPE;

/*****************************************************************************
 *  Log
 ****************************************************************************/
#define BAT_LOG_CRTI 1
#define BAT_LOG_FULL 2
#define BAT_LOG_DEBG 3

#define BAT_DEG(num, fmt, args...) \
do {\
	switch (num) {\
		case BAT_LOG_CRTI:\
			pr_err(fmt, ##args); \
			break;\
		case BAT_LOG_FULL:\
		case BAT_LOG_DEBG:\
		default:\
			pr_debug(fmt, ##args); \
			break;\
	} \
} while (0)

enum general_batt_status
{
    BATT_STATUS_COLD = 0,
    BATT_STATUS_COOL,
    BATT_STATUS_GOOD,
    BATT_STATUS_WARM,
    BATT_STATUS_HOT,
    BATT_STATUS_UNKNOW,
};

struct batt_status_map
{
    int low_temp;
    int high_temp;
    enum general_batt_status batt_st;
    int batt_current;
};

enum general_charge_status {
	CHARGE_STATE_NO_CHG	= 0,
	CHARGE_STATE_PRE_CHG	= 1,
	CHARGE_STATE_FAST_CHG	= 2,
	CHARGE_STATE_CHG_DONE	= 3,
};

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	USBOTG	= BIT(3),
	SOC	= BIT(4),
	RECHARGE = BIT(5),
};

#define ENABLE			1
#define DISABLE			0

#define INIT_VAL_INVILD		-511	

extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
extern int hw_charging_pmic_get_charger_type(void);
#endif