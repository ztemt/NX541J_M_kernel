#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>
#include <linux/power/general_battery.h>
#include <linux/power/general_charger.h>
#include <mt-plat/upmu_common.h>

/** ============================================================  */
/** Global variable */
/** ============================================================ */
int Enable_BATDRV_LOG = BAT_LOG_CRTI;

#ifdef CONFIG_TI_BQ25890_CHARGER_SUPPORT
static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_UNKNOWN,	/* bit 0 */
	POWER_SUPPLY_TYPE_USB,		/* bit 1 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_HVDCP,/* bit 4 */
	POWER_SUPPLY_TYPE_USB,		/* bit 5 error case, report SDP */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 6 */
};
#else
static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_UNKNOWN,	/* bit 0 */
	POWER_SUPPLY_TYPE_USB,		/* bit 1 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,  /* bit 4 */
	POWER_SUPPLY_TYPE_USB,		/* bit 5 error case, report SDP */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 6 */
};
#endif

static struct batt_status_map batt_temp_map[] =
{
    { CHG_TEMP_MIN,  CHG_TEMP_COLD, BATT_STATUS_COLD, BATT_COLD_CURRENT },		/** -20 degree to -5 degree*/
    { CHG_TEMP_COLD, CHG_TEMP_COOL, BATT_STATUS_COOL, BATT_COOL_CURRENT }, 		/** -5 degree to 10 degree*/
    { CHG_TEMP_COOL, CHG_TEMP_WARM, BATT_STATUS_GOOD, BATT_GOOD_CURRENT }, 		/** 10 degree to 45 degree*/
    { CHG_TEMP_WARM, CHG_TEMP_HOT,  BATT_STATUS_WARM, BATT_WARM_CURRENT }, 		/** 45 degree to 50 degree*/
    { CHG_TEMP_HOT,  CHG_TEMP_MAX,  BATT_STATUS_HOT,  BATT_HOT_CURRENT  }, 		/** 50 degree to 70 degree*/
};

/** USB properties */
static enum power_supply_property general_charger_usb_props[] =
{
    POWER_SUPPLY_PROP_TYPE,
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_OTG,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_DETECTION,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static char *usb_supplied_to[] = {
	"usb",
};

/***********************************************************************
 * struct - device information
 * @dev:					device pointer to access the parent
 * @monitor_wake_lock:		lock to polling ntc operator
 * @batt_health:			indicates that battery health
 * @batt_present:			present status of battery
 * @batt_voltage:			present voltage of battery
 * @batt_temp:				present temperature of battery
 * @batt_soc:				capacity of the battery reported
 * @usb_present:			present status of USB
 * @usb_chgr_type:			the type of charger
 * @usb_psy:				power supply to export
 *					information to userspace
 * @batt_psy:				power supply to export
 *					information to userspace
 ***********************************************************************/
struct general_charger
{
    struct device 		*dev;
    struct wake_lock	monitor_wake_lock;

    unsigned int		batt_health;
    unsigned int 		batt_present;
    unsigned int		batt_voltage;
    unsigned int		batt_temp;
	unsigned int		batt_soc;
	unsigned int		usb_present;
    unsigned int 		usb_chgr_type;
	unsigned int		usb_chgr_voltage;
	unsigned int		chg_disabled;
    unsigned int		chg_status;
	unsigned int		chg_done;
    unsigned int		max_chg_current;
	unsigned int		usb_chg_current;
    unsigned int		otg_enabled;
	unsigned int		thermal_current;
    bool        		need_update;

	chg_ctrl_fn			main_chgr;
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    bool                parallel_enabled;
	chg_ctrl_fn			parallel_chgr;
    struct mutex        parallel_lock;
    struct wake_lock	parallel_check;
    struct delayed_work parallel_en_work;
#endif
    struct dentry              *debug_root;
    int					chgr_stat_gpio;
    int					chgr_stat_irq;

#ifdef CONFIG_OF
    int                 usb_sdp_limit_max_current;
    int                 usb_dcp_limit_max_current;
    int                 usb_hvdcp_limit_max_current;
    int                 usb_max_charge_current;

    int                 battery_recharge_threshold;
#endif
    struct power_supply	*batt_psy;
    struct power_supply usb_psy;

    struct delayed_work	chgr_monitor_work;
};

static int dump_control_interface(CHARGER_COMMON_CTRL_CMD cmd, void *data)
{
   BAT_DEG(BAT_LOG_CRTI,"Charger controller error.\n");
   return 0;
};

static void general_charger_hw_init(struct general_charger *info)
{
	int ret;
    union power_supply_propval val;
    struct power_supply	*power_psy = power_supply_get_by_name("main");

    if(power_psy){
        ret = power_psy->get_property(power_psy, POWER_SUPPLY_PROP_CONTROL_FUNCTION, &val);
        info->main_chgr = (chg_ctrl_fn)val.int64val;
    }
    if(!info->main_chgr)
        info->main_chgr = dump_control_interface;
    info->main_chgr(CHARGER_COMMON_CMD_INIT, NULL);
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    power_psy = power_supply_get_by_name("parallel");

    if(power_psy){
        ret = power_psy->get_property(power_psy, POWER_SUPPLY_PROP_CONTROL_FUNCTION, &val);
        info->parallel_chgr = (chg_ctrl_fn)val.int64val;
    }
    if(!info->parallel_chgr)
        info->parallel_chgr = dump_control_interface;
    info->parallel_chgr(CHARGER_COMMON_CMD_INIT, NULL);
#endif
    return;
}

static void general_charger_dump_hw_register(struct general_charger *info)
{
    info->main_chgr(CHARGER_COMMON_CMD_DUMP_REGISTER, NULL);
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    if(info->parallel_enabled) info->parallel_chgr(CHARGER_COMMON_CMD_DUMP_REGISTER, NULL);
#endif
}

static void general_charger_set_usb_connected(int online)
{
	BAT_DEG(BAT_LOG_FULL,"Set usb connect:0x%x.\n", online);
    if (online)
		mt_usb_connect();
	else
		mt_usb_disconnect();
}

#define BATT_TEMP_DELTA    20
static int general_charger_get_battery_status(const struct batt_status_map *pts,
        uint32_t tablesize, int input, int *batt_temp_status)
{
    static int current_index = 0;
    static int init_status = 1;

    if ( pts == NULL || batt_temp_status == NULL)
        return BATT_STATUS_UNKNOW;

	if(input > CHG_TEMP_MAX || input < CHG_TEMP_MIN)
		return BATT_STATUS_UNKNOW;

    if(init_status)
    {
        while (current_index < tablesize)
        {
            if ( (pts[current_index].low_temp <= input) && (input <= pts[current_index].high_temp) )
                break;
            else
                current_index++;
        }
        init_status = 0;
        BAT_DEG(BAT_LOG_CRTI,"-First-input=%d  current_index=%d \n",input, current_index);
    }
    else
    {
        if(input < (pts[current_index].low_temp - BATT_TEMP_DELTA))
            current_index--;
        else if(input > pts[current_index].high_temp)
            current_index++;
    }

    if(current_index < 0)
        *batt_temp_status = BATT_STATUS_COLD;
    else if(current_index >= tablesize)
        *batt_temp_status = BATT_STATUS_HOT;
    else
        *batt_temp_status = pts[current_index].batt_st;

    BAT_DEG(BAT_LOG_CRTI,"input=%d index=%d, batt_temp_status=%d\n", input, current_index, *batt_temp_status);

    return current_index;
}

static int general_charger_set_charging_enable(struct general_charger *info, int reason,
					int enable)
{
	int reg, rc = 0;
	int disabled = info->chg_disabled;

	if (enable)
		disabled &= ~reason;
	else
		disabled |= reason;

	if (!!info->chg_disabled == !!disabled)
		goto skip;

	reg = !!disabled ? DISABLE : ENABLE;
	rc = info->main_chgr(CHARGER_COMMON_CMD_ENABLE, &reg);
skip:
	info->chg_disabled = disabled;
    return rc;
}

static void general_charger_set_hiz_mode(struct general_charger *info, int enable)
{
	int reg = enable;
	info->main_chgr(CHARGER_COMMON_CMD_SET_HIZ_SWTCHR, &reg);
}

static void general_charger_get_charger_detection(struct general_charger *info)
{
	enum power_supply_type usb_supply_type;
	int usb_chgr_type = info->main_chgr(CHARGER_COMMON_CMD_GET_CHARGER_TYPE, NULL);

	BAT_DEG(BAT_LOG_FULL,"Get register value :0x%x .\n", usb_chgr_type);
	usb_supply_type = usb_type_enum[usb_chgr_type];

	if(info->usb_chgr_type ^ usb_chgr_type) {
		if( usb_supply_type == POWER_SUPPLY_TYPE_USB ) {
			BAT_DEG(BAT_LOG_CRTI,"SDP detected, set usb limit currtent to 500mA\n");
            info->max_chg_current = info->usb_sdp_limit_max_current;
        }
        else if( usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP ) {
            BAT_DEG(BAT_LOG_CRTI,"CDP detected, set usb limit currtent to 2000mA\n");
            info->max_chg_current = info->usb_dcp_limit_max_current;
        }
        else if ( usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP ){
            BAT_DEG(BAT_LOG_CRTI,"DCP detected, set usb limit currtent to 2000mA\n");
            info->max_chg_current = info->usb_dcp_limit_max_current;
        }
        else if ( usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
            BAT_DEG(BAT_LOG_CRTI,"HVDCP detected, set usb limit currtent to 2000mA\n");
            info->max_chg_current = info->usb_hvdcp_limit_max_current;
        }
        else {
            BAT_DEG(BAT_LOG_CRTI,"Others detected, set usb limit currtent to 500mA\n");
            info->max_chg_current = info->usb_sdp_limit_max_current;
		}

		info->usb_chgr_type = usb_chgr_type;
		power_supply_set_supply_type(&info->usb_psy, usb_supply_type);
	}
}

#if 0
static int general_charger_set_appropriate_vddmax(struct general_charger *info, int voltage)
{
    return info->main_chgr(CHARGER_COMMON_CMD_SET_CV_VOLTAGE, &voltage);
}
#endif

static int general_charger_get_usb_charge_current(struct general_charger *info)
{
    return info->main_chgr(CHARGER_COMMON_CMD_GET_CHARGING_CURRENT, NULL);
}

static int general_charger_set_usb_charge_current(struct general_charger *info, int current_ma)
{
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    int usb_current= current_ma, parallel_current = 0;
    if(info->parallel_enabled){
        usb_current = current_ma - (current_ma * PARALLEL_AICL_CURRENT_PERCENTAGE) / 100;
        parallel_current = current_ma - usb_current;
    }

    info->main_chgr(CHARGER_COMMON_CMD_SET_CHARGING_CURRENT, &usb_current);
    info->parallel_chgr(CHARGER_COMMON_CMD_SET_CHARGING_CURRENT, &parallel_current);
    BAT_DEG(BAT_LOG_CRTI, "Parallel charge current:%d:%d", usb_current, parallel_current);
#else
    info->main_chgr(CHARGER_COMMON_CMD_SET_CHARGING_CURRENT, &current_ma);
    BAT_DEG(BAT_LOG_CRTI, "Set appropriate current:%d.\n", current_ma);
#endif
    return 0;
}

static int general_charger_set_usb_limit_current(struct general_charger *info, int current_ma)
{
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    int usb_limited_current = current_ma, parallel_limited_current = 0;
    if(info->parallel_enabled){
        parallel_limited_current = current_ma;
    }

    info->main_chgr(CHARGER_COMMON_CMD_SET_INPUT_CURRENT, &usb_limited_current);
    info->parallel_chgr(CHARGER_COMMON_CMD_SET_INPUT_CURRENT, &parallel_limited_current);
    BAT_DEG(BAT_LOG_CRTI, "Parallel usb limit current:%d:%d", usb_limited_current, usb_limited_current);
#else
    info->main_chgr(CHARGER_COMMON_CMD_SET_INPUT_CURRENT, &current_ma);
#endif
    return 0;
}

#if 0
static int general_charger_get_charger_type(struct general_charger *info)
{
    return info->main_chgr(CHARGER_COMMON_CMD_GET_CHARGER_TYPE, NULL);
}
#endif

static int general_charger_set_appropriate_current(struct general_charger *info)
{
	int ret = 0;
	unsigned int chg_current = info->max_chg_current;
	chg_current = min(chg_current, info->thermal_current);
	if(info->usb_chg_current != chg_current) {
		pr_debug("setting usb limit current %d mA\n", chg_current);
		info->usb_chg_current = chg_current;
        general_charger_set_usb_charge_current(info, info->usb_max_charge_current);
		general_charger_set_usb_limit_current(info, chg_current);
	}

    return ret;
}

#if 0
static int general_charger_get_input_current(struct general_charger *info)
{
    return info->main_chgr(CHARGER_COMMON_CMD_GET_INPUT_CURRENT, NULL);
}

static int general_charger_get_charger_present(struct general_charger *info)
{
    return info->main_chgr(CHARGER_COMMON_CMD_GET_CHARGER_PRESENT, NULL);
}
#endif

static int general_charger_get_charging_status(struct general_charger *info)
{
    int iddig_num, iddig_state;
 	enum general_charge_status chg_status;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
#ifdef CONFIG_OF
    iddig_num = of_get_named_gpio(info->dev->of_node, "usb_iddig_detect", 0);
    iddig_state = gpio_get_value(iddig_num);
    if(!iddig_state)
        return POWER_SUPPLY_STATUS_DISCHARGING;
#endif

    if(!info->usb_present && !pmic_get_register_value(PMIC_RGS_CHRDET)) {
  		return POWER_SUPPLY_STATUS_DISCHARGING;
  	}

	chg_status = info->main_chgr(CHARGER_COMMON_CMD_GET_CHARGING_STATUS, NULL);

	if (chg_status == CHARGE_STATE_NO_CHG){
        if(info->usb_present) BAT_DEG(BAT_LOG_CRTI, "usb present, but not charge now. current_now:%d\n", general_charger_get_usb_charge_current(info));
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}	
	else if (chg_status == CHARGE_STATE_CHG_DONE){
		status = POWER_SUPPLY_STATUS_FULL;
	}
	else if((chg_status == CHARGE_STATE_PRE_CHG) || (chg_status == CHARGE_STATE_FAST_CHG)){
		status = POWER_SUPPLY_STATUS_CHARGING;
	}		
  	else {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
  	}

    return status;
}

static int general_charger_set_otg_status(struct general_charger*info, int enable)
{
	int rc = 0;
	int status = enable;
	if(info->otg_enabled != status) {
		if(status == ENABLE){
			info->main_chgr(CHARGER_COMMON_CMD_SET_OTG_STATUS, &status);
		}
		else{
			info->main_chgr(CHARGER_COMMON_CMD_SET_OTG_STATUS, &status);
		}
		info->otg_enabled = status;
	}

	return rc;
}

#if 0
static int general_charger_get_otg_status(struct general_charger*info)
{
	int status;
	status = info->main_chgr(CHARGER_COMMON_CMD_GET_OTG_STATUS, NULL);
    return status;
}
#endif

static int general_charger_get_battery_voltage(struct general_charger*info)
{
    return info->main_chgr(CHARGER_COMMON_CMD_GET_BATTERY_VOLTAGE, NULL);
}

#define R_CHARGER_PULL_HIGH      330
#define R_CHARGER_PULL           39
static int general_charger_get_charger_voltage(struct general_charger*info)
{
    int val;
    int date = 5;

	val = PMIC_IMM_GetOneChannelValue(PMIC_AUX_VCDT_AP, date, 1);
	val =(((R_CHARGER_PULL_HIGH + R_CHARGER_PULL) * 100 * val) / R_CHARGER_PULL) / 100;

	return val;
}

static int general_charger_get_battery_parameters(struct general_charger *info)
{
	union power_supply_propval val = {0,};
	
	if (info->batt_psy == NULL || info->batt_psy < 0)
		info->batt_psy = power_supply_get_by_name("battery");

	if(!info->batt_psy)
		return -1;

	info->batt_voltage = general_charger_get_battery_voltage(info);
    info->usb_chgr_voltage = general_charger_get_charger_voltage(info);

	info->batt_psy->get_property(info->batt_psy,POWER_SUPPLY_PROP_TEMP, &val);
	info->batt_temp = val.intval;
	
	info->batt_psy->get_property(info->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&val);
	info->batt_soc = val.intval;
	
	return 0;
}

static int general_charger_set_battery_parameters(struct general_charger *info)
{
	union power_supply_propval val = {0,};

	if (info->batt_psy == NULL || info->batt_psy < 0)
		info->batt_psy = power_supply_get_by_name("battery");

	if(!info->batt_psy)
		return -1;

    if(info->batt_health == BATT_STATUS_HOT)
        val.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
    else if(info->batt_health == BATT_STATUS_COLD)
        val.intval = POWER_SUPPLY_HEALTH_COLD;
	else if(info->batt_health == BATT_STATUS_WARM)
		val.intval = POWER_SUPPLY_HEALTH_WARM;
    else
        val.intval = POWER_SUPPLY_HEALTH_GOOD;

	info->batt_psy->set_property(info->batt_psy, POWER_SUPPLY_PROP_HEALTH, &val);

	val.intval = info->chg_status;
	info->batt_psy->set_property(info->batt_psy, POWER_SUPPLY_PROP_STATUS, &val);

    power_supply_changed(info->batt_psy);

	return 0;
}

static void general_charger_jeita_setting_checkpoint(struct general_charger *info)
{
    int battery_status = BATT_STATUS_UNKNOW;
    int batt_current = 0;
    int state_index;

    state_index = general_charger_get_battery_status(batt_temp_map,
                  ARRAY_SIZE(batt_temp_map),
                  info->batt_temp,
                  &battery_status);
    if(battery_status != BATT_STATUS_UNKNOW) {
        batt_current = batt_temp_map[state_index].batt_current;
		info->thermal_current = batt_current;
	}
    if(battery_status != info->batt_health)
    {
        BAT_DEG(BAT_LOG_CRTI,"last info->batt_health=%d new_battery_status=%d\n",info->batt_health, battery_status);
        if(batt_current > 0)
        {
            general_charger_set_appropriate_current(info);
			general_charger_set_charging_enable(info, THERMAL, ENABLE);
            BAT_DEG(BAT_LOG_CRTI,"batt_temp=%d batt_status=%d batt_current=%d start charging...\n",info->batt_temp,battery_status,batt_current);
        }
        else
        {
			general_charger_set_charging_enable(info, THERMAL, DISABLE);
            BAT_DEG(BAT_LOG_CRTI,"batt_temp=%d out of rangge,stop charging!\n", info->batt_temp);
        }

        info->need_update = true;
		info->batt_health = battery_status;
		info->chg_status = (info->chg_disabled == 0) ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		general_charger_set_battery_parameters(info);
    }
}

#define MAX_DEBOUNCE_COUNT		5
static void general_charger_recharge_setting_checkpoint(struct general_charger *info)
{
    static int count;

	if(!info->chg_done)
		return;

	if(info->usb_present && info->batt_voltage < BATTERY_RECHAEGE_THRESHOLD){
		count++;
	}

	if(count > MAX_DEBOUNCE_COUNT){
		count = 0;
        info->chg_done = 0;
        info->need_update = true;
		general_charger_set_charging_enable(info, RECHARGE, DISABLE);
		BAT_DEG(BAT_LOG_CRTI,"Recharge start.\n");
		general_charger_set_charging_enable(info, RECHARGE, ENABLE);
	}
} 

static int
general_charger_usb_property_is_writeable(struct power_supply *psy,
        enum power_supply_property psp)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
    case POWER_SUPPLY_PROP_USB_OTG:
    case POWER_SUPPLY_PROP_ONLINE:
        return 1;
    default:
        break;
    }

    return 0;
}

/**
 * general_charger_usb_get_property() - get the usb properties
 * @psy:        pointer to the power_supply structure
 * @psp:        pointer to the power_supply_property structure
 * @val:        pointer to the power_supply_propval union
 *
 * This function gets called when an application tries to get the usb
 * properties by reading the sysfs files.
 * USB properties are online, present and voltage.
 * online:     usb charging is in progress or not
 * present:    presence of the usb
 * voltage:    vbus voltage
 * Returns error code in case of failure else 0(on success)
 */
static int general_charger_usb_set_property(struct power_supply *psy,
        enum power_supply_property psp,
        const union power_supply_propval *val)
{
    struct general_charger *info = container_of(psy, struct general_charger,
                                   usb_psy);

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        if(val->intval == POWER_SUPPLY_STATUS_CHARGING)
        {
            info->chg_done = 0;
            BAT_DEG(BAT_LOG_CRTI, "Resuming charging by battery.\n");
            general_charger_set_charging_enable(info, USER, ENABLE);
        }
        break;
    case POWER_SUPPLY_PROP_USB_OTG:
        general_charger_set_otg_status(info, val->intval);
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        info->batt_health = val->intval;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
        info->usb_present = val->intval;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        info->batt_present = val->intval;
        break;
    case POWER_SUPPLY_PROP_TYPE:
        psy->type = val->intval;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

/**
 * general_charger_usb_get_property() - get the usb properties
 * @psy:        pointer to the power_supply structure
 * @psp:        pointer to the power_supply_property structure
 * @val:        pointer to the power_supply_propval union
 *
 * This function gets called when an application tries to get the usb
 * properties by reading the sysfs files.
 * USB properties are online, present and voltage.
 * online:     usb charging is in progress or not
 * present:    presence of the usb
 * voltage:    vbus voltage
 * Returns error code in case of failure else 0(on success)
 */
static int general_charger_usb_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct general_charger *info = container_of(psy, struct general_charger,
                                   usb_psy);

    switch (psp)
    {
    case POWER_SUPPLY_PROP_USB_OTG:
        val->intval = info->otg_enabled;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        if(info->batt_health == BATT_STATUS_HOT)
            val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
        else if(info->batt_health == BATT_STATUS_COLD)
            val->intval = POWER_SUPPLY_HEALTH_COLD;
		else if(info->batt_health == BATT_STATUS_WARM)
			val->intval = POWER_SUPPLY_HEALTH_WARM;
        else
            val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = info->usb_present;
        break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = general_charger_get_charger_voltage(info);
		break;
	case POWER_SUPPLY_PROP_CHARGE_DETECTION:
		val->intval = info->usb_chgr_type;
		break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = general_charger_get_battery_voltage(info);
        break;
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = info->chg_status;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval  = info->usb_chg_current;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

#define SCHEDULE_MONITOR_SOC_WAIT_MS		1000
static void general_charger_external_power_changed(struct power_supply *psy)
{
    int iddig_num, iddig_state;
    int usb_present;
    struct general_charger *info = container_of(psy, struct general_charger,
                                   usb_psy);
    usb_present = pmic_get_register_value(PMIC_RGS_CHRDET);
#ifdef CONFIG_OF
    iddig_num = of_get_named_gpio(info->dev->of_node, "usb_iddig_detect", 0);
    iddig_state = gpio_get_value(iddig_num);
    if(!iddig_state)
        return;
#endif

    BAT_DEG(BAT_LOG_CRTI,"usb present:%d -> %d, status:%d.\n", info->usb_present, usb_present, info->chg_status);

	if(info->usb_present != usb_present) {
        /** Update the charging status immediately */
        info->need_update = true;
        info->usb_present = usb_present;
        info->chg_status = info->usb_present ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		info->usb_chgr_type = info->usb_present ? info->usb_chgr_type : 0;
        general_charger_set_battery_parameters(info);
		general_charger_set_usb_connected(info->usb_present);
		general_charger_set_charging_enable(info, CURRENT, !!usb_present);
    }

    cancel_delayed_work_sync(&info->chgr_monitor_work);
    schedule_delayed_work(&info->chgr_monitor_work, msecs_to_jiffies(SCHEDULE_MONITOR_SOC_WAIT_MS * 3));
}

#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
static bool general_charger_check_parallel_conditions(struct general_charger * info)
{
    if(info->batt_health != BATT_STATUS_GOOD){
        BAT_DEG(BAT_LOG_CRTI, "Battery is not good, disable parallel charge.\n");
        return false;
    }

    if(info->usb_chgr_type != USB_HVDCP_CHARGER){
        BAT_DEG(BAT_LOG_CRTI, "Charge type not support, skip enable parallel charge.\n");
        return false;
    }

    return true; 
}

static void general_charger_try_enable_parallel_charge(struct general_charger *info)
{
    int parallel_enabled = info->parallel_enabled;
    general_charger_set_usb_limit_current(info, info->usb_hvdcp_limit_max_current);
    general_charger_set_usb_charge_current(info, info->usb_max_charge_current);
    info->parallel_chgr(CHARGER_COMMON_CMD_ENABLE, &parallel_enabled);
}
#endif

#define MONITOR_SOC_WAIT_MS                 10000
static void general_charger_chgr_monitor_work(struct work_struct *work)
{
    int chg_status;
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    bool parallel_enabled = false;    
#endif
    struct delayed_work *dwork = to_delayed_work(work);
    struct general_charger *info = container_of(dwork,
                                   struct general_charger, chgr_monitor_work);

    /** Get battery parameters */
	general_charger_get_battery_parameters(info);

#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    /** Setting parallel charge parameters */
    parallel_enabled = general_charger_check_parallel_conditions(info);
    if(info->parallel_enabled ^ parallel_enabled){
        info->parallel_enabled = parallel_enabled;
        general_charger_try_enable_parallel_charge(info);
    }
    BAT_DEG(BAT_LOG_CRTI,"Parallel charge enabled:%d\n", info->parallel_enabled);
#endif

	/** Update charging information to power supply class. */
    chg_status = general_charger_get_charging_status(info);

    if(info->chg_status != chg_status){
        info->need_update = true;
        info->chg_status = chg_status;
        power_supply_changed(info->batt_psy);
    }

	/** Report charge done */
	if(info->chg_status == POWER_SUPPLY_STATUS_FULL){
		BAT_DEG(BAT_LOG_CRTI,"Charge Done.\n");
		info->chg_done = 1;
	}

	/** Check NTC conditions */
	general_charger_jeita_setting_checkpoint(info);

	/** Check recharge conditions*/
	general_charger_recharge_setting_checkpoint(info);

#ifdef CONFIG_GENERAL_PM_DEBUG
    general_charger_dump_hw_register(info);
#endif

    BAT_DEG(BAT_LOG_CRTI,"s:%d,c:%d,u:%d,e:%d,t:%d,v:%d.\n",
           info->chg_status, info->usb_chg_current, info->usb_present, info->chg_disabled, info->usb_chgr_type, info->usb_chgr_voltage);

    if(info->need_update){
        BAT_DEG(BAT_LOG_CRTI,"Sync information\n");
        power_supply_changed(&info->usb_psy);
        info->need_update = false;
    }

	/** Lock to avoid system suspend when USB plug in.*/
    if((info->usb_present != 1) && wake_lock_active(&info->monitor_wake_lock)) {
        BAT_DEG(BAT_LOG_FULL,"<---- release wakeup lock.\n");
        wake_unlock(&info->monitor_wake_lock);
    }

    if(info->usb_present) schedule_delayed_work(&info->chgr_monitor_work, msecs_to_jiffies(MONITOR_SOC_WAIT_MS));
}

static irqreturn_t general_charger_usbin_interrupt_handler(int irq, void *data)
{
    int usb_present = pmic_get_register_value(PMIC_RGS_CHRDET);
    struct general_charger *info = (struct general_charger *)data;
	BAT_DEG(BAT_LOG_CRTI,"-------------------- usbin hander --------------------\n");

    if (!wake_lock_active(&info->monitor_wake_lock))
    {
        BAT_DEG(BAT_LOG_FULL,"----> handle wakeup lock.\n");
        wake_lock(&info->monitor_wake_lock);
    }
    general_charger_set_usb_connected(usb_present);

	if(usb_present){
		general_charger_get_charger_detection(info);
		general_charger_set_appropriate_current(info);
	}
	else{
		/******        Workaround     ******
			avoid register value be reset.
		************************************/
		general_charger_set_hiz_mode(info, ENABLE);
	}

    return IRQ_HANDLED;
}

#ifdef CONFIG_OF
#define ELEMENTS_ROWS       4
static int general_charger_parse_temp_map_dt(struct general_charger *info,
                struct batt_status_map *map, char *property)
{
    struct device_node *node = info->dev->of_node;
    int total_elements, size;
    struct property *prop;
    const __be32 *data;
    int num, i;

    prop = of_find_property(node, property, &size);
    if (!prop) {
        dev_err(info->dev, "%s missing\n", property);
        return -EINVAL;
    }

    total_elements = size / sizeof(int);
    if (total_elements % ELEMENTS_ROWS) {
        dev_err(info->dev, "%s table not in multiple of %d, total elements = %d\n",
                property, ELEMENTS_ROWS, total_elements);
        return -EINVAL;
    }

    data = prop->value;
    num = total_elements / ELEMENTS_ROWS;

    for (i = 0; i < num; i++) {
        map[i].low_temp =  be32_to_cpup(data++);
        map[i].high_temp =  be32_to_cpup(data++);
        map[i].batt_st =  be32_to_cpup(data++);
        map[i].batt_current =  be32_to_cpup(data++);
    }
    return 0;
}

static int general_charger_parse_dt_parameters(struct general_charger *info)
{
    int rc;
    struct device_node *node = info->dev->of_node;
    if (!node) {
        dev_err(info->dev, "device tree info. missing\n");
        return -EINVAL;
    }

    rc = of_property_read_u32(node, "nubia,usb_sdp_limit_max_current",
                                               &info->usb_sdp_limit_max_current);
    if (rc < 0)
        info->usb_sdp_limit_max_current = -EINVAL;

    rc = of_property_read_u32(node, "nubia,usb_dcp_limit_max_current",
                                               &info->usb_dcp_limit_max_current);
    if (rc < 0)
        info->usb_dcp_limit_max_current = -EINVAL;

    rc = of_property_read_u32(node, "nubia,usb_hvdcp_limit_max_current",
                                               &info->usb_hvdcp_limit_max_current);
    if (rc < 0)
        info->usb_hvdcp_limit_max_current = -EINVAL;

    rc = of_property_read_u32(node, "nubia,usb_max_charge_current",
                                               &info->usb_max_charge_current);
    if (rc < 0)
        info->usb_max_charge_current = -EINVAL;

    rc = of_property_read_u32(node, "nubia,battery_recharge_threshold",
                                               &info->battery_recharge_threshold);
    if (rc < 0)
        info->battery_recharge_threshold = -EINVAL;

    rc = general_charger_parse_temp_map_dt(info, batt_temp_map, "nubia,batt_status_map_table");

    return rc;
}

static int general_charger_init_charge_pins(struct general_charger *info)
{
    int ret;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_0;
    struct pinctrl_state *pinctrl_1;
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    struct pinctrl_state *pinctrl_2;
#endif
    
    pinctrl = devm_pinctrl_get(info->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        pr_err("Cannt find chr_stat pinctrl.\n");
    }

    pinctrl_0 = pinctrl_lookup_state(pinctrl, "chr_stat_cfg");
    if (IS_ERR(pinctrl_0)) {
        ret = PTR_ERR(pinctrl_0);
        pr_err("Cannt find chr_stat pinctrl chr_stat_cfg.\n");
    }
    else{
        pr_err("Success to select chr_stat_cfg.\n");
        pinctrl_select_state(pinctrl, pinctrl_0);
    }

    pinctrl_1 = pinctrl_lookup_state(pinctrl, "chr_enable_cfg");
    if (IS_ERR(pinctrl_1)) {
        ret = PTR_ERR(pinctrl_1);
        pr_err("Cannt find chr_stat pinctrl chr_enable_cfg.\n");
    }
    else{
        pr_err("Success to select chr_enable_cfg.\n");
        pinctrl_select_state(pinctrl, pinctrl_1);
    }

#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    pinctrl_2 = pinctrl_lookup_state(pinctrl, "plr_enable_cfg");
    if (IS_ERR(pinctrl_2)) {
        ret = PTR_ERR(pinctrl_2);
        pr_err("Cannt find chr_stat pinctrl plr_enable_cfg.\n");
    }
    else{
        pr_err("Success to select plr_enable_cfg.\n");
        pinctrl_select_state(pinctrl, pinctrl_2);
    }
#endif
    return ret;
}
#endif

static ssize_t general_charger_debugfs_write(struct file *file, const char *buf, size_t count, loff_t *data)
{
    struct seq_file *s = file->private_data;
    struct general_charger *info = s->private;
    char cmd[16];
    int status = count;

    memset(cmd, 0x00, sizeof(cmd));

    if (copy_from_user(&cmd, buf, min_t(size_t, sizeof(buf) - 1, count))) {
        status = -EFAULT;
        goto out;
    }

    if (!strncmp(cmd, "dump", 6)) {
        general_charger_dump_hw_register(info);
    } else {
        status = -EINVAL;
        goto out;
    }
out:
	return status;
}

static int general_charger_debugfs_status(struct seq_file *m, void *v)
{
	seq_printf(m, "Success.\n");
	return 0;
}

static int general_charger_debugfs_open(struct inode *inode, struct file *file)
{
	struct general_charger *info = inode->i_private;
	return single_open(file, general_charger_debugfs_status, info);
}

static const struct file_operations general_charger_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= general_charger_debugfs_open,
	.read		= seq_read,
    .write      = general_charger_debugfs_write,
};

static void general_charger_create_debugfs_entries(struct general_charger *info)
{

	info->debug_root = debugfs_create_dir("bat_debug", NULL);
	if (!info->debug_root)
		pr_err("Couldn't create debug dir\n");

	if (info->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("command", S_IFREG | S_IWUSR | S_IRUGO,
					  info->debug_root, info,
					  &general_charger_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create lbc_config debug file\n");
	}
}

static int general_charger_probe(struct platform_device *pdev)
{
    int ret;
    unsigned int debounce_enable;
#ifdef CONFIG_OF
    u32 ints[2] = { 0, 0 };
    struct device_node *node;
#endif
    struct general_charger *info =
        kzalloc(sizeof(struct general_charger), GFP_KERNEL);
    if (!info)
        return -ENOMEM;

    /* Get parent data */
    info->dev = &pdev->dev;

    /* Power supply */
    if (info->batt_psy == NULL)
        info->batt_psy = power_supply_get_by_name("battery");

    /** Set Charger HW Init**/
    general_charger_hw_init(info);
	general_charger_set_charging_enable(info, CURRENT, pmic_get_register_value(PMIC_RGS_CHRDET));

    /** Power Supply Base Class */
    info->usb_psy.name = "usb";
    info->usb_psy.type = POWER_SUPPLY_TYPE_USB;
    info->usb_psy.properties = general_charger_usb_props;
    info->usb_psy.num_properties = ARRAY_SIZE(general_charger_usb_props);
    info->usb_psy.get_property = general_charger_usb_get_property;
    info->usb_psy.set_property = general_charger_usb_set_property;
    info->usb_psy.external_power_changed = general_charger_external_power_changed;
    info->usb_psy.property_is_writeable = general_charger_usb_property_is_writeable;
    info->usb_psy.supplied_to	= usb_supplied_to;
    info->usb_psy.num_supplicants	= ARRAY_SIZE(usb_supplied_to);

    /** Create a work queue for the charger */
    wake_lock_init(&info->monitor_wake_lock, WAKE_LOCK_SUSPEND,	"chgr_monitor_lock");
    INIT_DELAYED_WORK(&info->chgr_monitor_work, general_charger_chgr_monitor_work);

    /** Register USB charger class */
    ret = power_supply_register(info->dev, &info->usb_psy);
    if (ret)
    {
        dev_err(info->dev, "failed to register USB charger\n");
        goto free_charger_wq;
    }

    info->usb_present = INIT_VAL_INVILD;
    info->usb_chgr_type = INIT_VAL_INVILD;
	info->batt_temp = INIT_VAL_INVILD;

	/** Initialize interrupt*/
    general_charger_init_charge_pins(info);
#ifdef CONFIG_OF
    node = of_find_compatible_node(NULL, NULL, "mediatek,chr_stat");
    if(node)
    {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        info->chgr_stat_gpio = ints[0];
        debounce_enable = ints[1];
    }

    info->chgr_stat_irq = irq_of_parse_and_map(node, 0);

    ret = devm_request_threaded_irq(info->dev, info->chgr_stat_irq, NULL, general_charger_usbin_interrupt_handler,
                      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                      "stat_irq", info);

	if(ret < 0){
		BAT_DEG(BAT_LOG_CRTI,"request irq failed\n");
		goto free_usb_psy;
	}
#endif

	general_charger_create_debugfs_entries(info);
    general_charger_parse_dt_parameters(info);
    platform_set_drvdata(pdev, info);

	if (!wake_lock_active(&info->monitor_wake_lock)){
		BAT_DEG(BAT_LOG_CRTI,"----> handle wakeup lock.\n");
		wake_lock(&info->monitor_wake_lock);
	}

    info->need_update = true;
    schedule_delayed_work(&info->chgr_monitor_work, msecs_to_jiffies(SCHEDULE_MONITOR_SOC_WAIT_MS));

    return ret;

free_usb_psy:
    power_supply_unregister(&info->usb_psy);
free_charger_wq:
    kfree(info);
	return ret;
}

struct platform_device general_charger_device =
{
    .name = "general_charger",
    .id = -1,
};

struct of_device_id charger_match_table[] = {
        { .compatible = "nubia, general_charger", },
        {},
};

#ifdef CONFIG_PM_SLEEP
static int general_charger_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct general_charger *info = platform_get_drvdata(pdev);
    BAT_DEG(BAT_LOG_CRTI,"general_charger suspend.\n");
    cancel_delayed_work(&info->chgr_monitor_work);
    return 0;
}

static int general_charger_resume(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct general_charger *info = platform_get_drvdata(pdev);
    BAT_DEG(BAT_LOG_CRTI,"general_charger resume.\n");
    /** Update the real charging status next time */
    info->need_update = true;
    schedule_delayed_work(&info->chgr_monitor_work, msecs_to_jiffies(SCHEDULE_MONITOR_SOC_WAIT_MS));
    return 0;
}

static const struct dev_pm_ops general_charger_pm_ops =
{
    .suspend 	= general_charger_suspend,
    .resume 	= general_charger_resume,
};
#endif

static struct platform_driver general_charger_driver =
{
    .probe = general_charger_probe,
    .driver = {
        .name = "general_charger",
    #ifdef CONFIG_PM_SLEEP
        .pm   = &general_charger_pm_ops,
    #endif
        .of_match_table = charger_match_table,
    },
};

static int __init general_charger_init(void)
{
    int ret;

    pr_debug("general_charger_init\n");

    ret = platform_driver_register(&general_charger_driver);
    if (ret)
        BAT_DEG(BAT_LOG_CRTI,"Unable to register driver.\n");

    BAT_DEG(BAT_LOG_CRTI,"Initialization : DONE !!\n");
    return 0;
}

static void __exit general_charger_exit(void)
{
   BAT_DEG(BAT_LOG_CRTI,"General charger driver exit!!\n");
}
module_init(general_charger_init);
module_exit(general_charger_exit);

MODULE_AUTHOR("Oscar Liu");
MODULE_DESCRIPTION("Battery Device Driver");
MODULE_LICENSE("GPL");

