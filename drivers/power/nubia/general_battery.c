/*
 * nubia fg chip driver
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>
#include <mach/upmu_hw.h>
#include <mt-plat/upmu_common.h>
#include <linux/power/general_charger.h>
#include <linux/power/general_battery.h>

#define RBAT_PULL_UP_R		90900
#define RBAT_PULL_UP_VOLT	2800

BATT_TEMPERATURE_T batt_temperature_table[] =
{
    {-20, 483954},
    {-15, 360850},
    {-10, 271697},
    { -5, 206463},
    {  0, 158214},
    {  5, 122259},
    { 10, 95227},
    { 15, 74730},
    { 20, 59065},
    { 25, 47000},
    { 30, 37643},
    { 35, 30334},
    { 40, 24591},
    { 45, 20048},
    { 50, 16433},
    { 55, 13539},
    { 60, 11210},
    { 65, 9328},
    { 70, 7798},
};

struct general_battery
{
    struct device			*dev;
    struct delayed_work		soc_monitor_work;
    struct wake_lock		monitor_wake_lock;

    struct power_supply		batt_psy;
    struct power_supply		*usb_psy;

    unsigned int			calulated_soc;
    unsigned int			batt_health;
    unsigned int			batt_present;
    unsigned int			batt_id;
    unsigned int			batt_temp;
    unsigned int			batt_vol;
    unsigned int			batt_dcc;
    unsigned int            batt_fcc;
    unsigned int            batt_rmc;
    unsigned int			chg_status;
    unsigned int			usb_present;
    unsigned int			cycle_count;
	unsigned int			battery_full_design_capacity;

    bool                    need_update;
    bool                    was_resuming;
    struct timespec			last_soc_change_time;

    int						batt_ma;
    int						soc_int_gpio;
    int						soc_irq;
    fg_ctrl_fn				fg_func;

    struct dentry			*debug_root;
    char*					battery_type;
};

enum power_supply_property battery_batt_props[] =
{
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static int dump_control_interface(BATTERY_COMMON_CTRL_CMD cmd, void *data)
{
   BAT_DEG(BAT_LOG_CRTI,"FG controller error.\n");
   return 0;
};

static int general_battery_hw_init(struct general_battery *chip)
{
    int ret;
    union power_supply_propval val;
    struct power_supply	*fg_psy = power_supply_get_by_name("fg");

    if(fg_psy)
    {
        ret = fg_psy->get_property(fg_psy, POWER_SUPPLY_PROP_CONTROL_FUNCTION, &val);
        chip->fg_func = (fg_ctrl_fn)val.int64val;
    }

    if(!chip->fg_func)
        chip->fg_func = dump_control_interface;

    return chip->fg_func(BATTERY_COMMON_CMD_HW_INIT, NULL);
}

static int general_battery_get_batt_current(struct general_battery *chip)
{
    int batt_ma = chip->fg_func(BATTERY_COMMON_CMD_GET_BATT_CURRENT, NULL);
    chip->batt_ma = batt_ma;
    return batt_ma;
}

static int general_battery_hw_reset(struct general_battery *chip)
{
    BAT_DEG(BAT_LOG_CRTI, "reset fuel gauge.\n");
    return chip->fg_func(BATTERY_COMMON_CMD_HW_RESET, NULL);
}

static int general_battery_update_batt_profile(struct general_battery *chip)
{
    int ret;
    BAT_DEG(BAT_LOG_CRTI, "update battery profile.\n");
    cancel_delayed_work_sync(&chip->soc_monitor_work);
    ret = chip->fg_func(BATTERY_COMMON_CMD_UPDATE_BATT_PROFILE, NULL);
    schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(10000));
    return ret;
}

#ifdef CONFIG_GENERAL_PM_DEBUG
static int general_battery_dump_hw_register(struct general_battery *chip)
{
    return chip->fg_func(BATTERY_COMMON_CMD_DUMP_REGISTER, NULL);
}
#endif

static int general_battery_get_cycle_count(struct general_battery *chip)
{
    int cycle_count = chip->fg_func(BATTERY_COMMON_CMD_GET_CYCLE_COUNT, NULL);
    if(cycle_count != chip->cycle_count)
    {
        chip->need_update = true;
        chip->cycle_count = cycle_count;
    }

    return cycle_count;
}

static int general_battery_get_batt_dcc(struct general_battery *chip)
{
    int value;
    value = chip->fg_func(BATTERY_COMMON_CMD_GET_BATT_DCC, NULL);
    chip->batt_dcc = value;

    return value;
}

static int general_battery_get_batt_rmc(struct general_battery *chip)
{
    int value;
    value = chip->fg_func(BATTERY_COMMON_CMD_GET_BATT_RMC, NULL);
    chip->batt_rmc = value;

    return value;
}

static int general_battery_get_batt_fcc(struct general_battery *chip)
{
    int value;
    value = chip->fg_func(BATTERY_COMMON_CMD_GET_BATT_FCC, NULL);
    chip->batt_fcc = value;

    return value;
}

static int general_battery_thermistor2temp(int res)
{
    int i = 0;
    int res_min = 0, res_max = 0;
    int batt_temp = -200, tmp_min = 0, tmp_max = 0;

    if (res >= batt_temperature_table[0].r_batt)
    {
        batt_temp = -20;
    }
    else if (res <= batt_temperature_table[18].r_batt)
    {
        batt_temp = 70;
    }
    else
    {
        res_min = batt_temperature_table[0].r_batt;
        tmp_min = batt_temperature_table[0].t_batt;

        for (i = 0; i <= 18; i++)
        {
            if (res >= batt_temperature_table[i].r_batt)
            {
                res_max = batt_temperature_table[i].r_batt;
                tmp_max = batt_temperature_table[i].t_batt;
                break;
            }
            else
            {
                res_min = batt_temperature_table[i].r_batt;
                tmp_min = batt_temperature_table[i].t_batt;
            }
        }

        batt_temp = (((res - res_max) * tmp_min) + ((res_min - res) * tmp_max)) / (res_min - res_max);
    }

    return batt_temp;
}

static int general_battery_get_batt_temperature(struct general_battery *chip)
{
    int t_batt = 25;
    long long r_adc_val, r_adc_temp;
    int adc_volt = 5;
    adc_volt = PMIC_IMM_GetOneChannelValue(PMIC_AUX_BATON_AP, adc_volt, 1);

    r_adc_temp = RBAT_PULL_UP_R * (long long)adc_volt;
    do_div(r_adc_temp, (RBAT_PULL_UP_VOLT - adc_volt));

    BAT_DEG(BAT_LOG_FULL,"adc voltage:%d\n", adc_volt);
#ifdef RBAT_PULL_DOWN_R
    r_adc_val = (r_adc_temp * RBAT_PULL_DOWN_R);
    do_div(r_adc_val, abs(RBAT_PULL_DOWN_R - r_adc_temp));
#else
    r_adc_val = r_adc_temp;
#endif

    t_batt = general_battery_thermistor2temp(r_adc_val);

    chip->batt_temp = t_batt * 10;

    return t_batt;
}

static int general_battery_set_batt_temperature(struct general_battery *chip)
{
    int ret = 0;
    int temp = chip->batt_temp;
    chip->fg_func(BATTERY_COMMON_CMD_SET_BATT_TEMP, (void*)&temp);

    return ret;
}

static int general_battery_set_charge_enable(struct general_battery *chip)
{
    union power_supply_propval val = {0,};

    if (chip->usb_psy == NULL || chip->usb_psy < 0)
        chip->usb_psy = power_supply_get_by_name("usb");

    if(!chip->usb_psy)
        return -1;
    val.intval = POWER_SUPPLY_STATUS_CHARGING;

    chip->usb_psy->set_property(chip->usb_psy, POWER_SUPPLY_PROP_STATUS, &val);
    chip->chg_status = val.intval;

    power_supply_changed(&chip->batt_psy);

    return 0;
}

static int bound_soc(int soc)
{
    soc = max(0, soc);
    soc = min(100, soc);
    return soc;
}

static int calculate_delta_time(struct timespec *time_stamp, int *delta_time_s)
{
	struct timespec now_time;

	/* default to delta time = 0 if anything fails */
	*delta_time_s = 0;

	get_monotonic_boottime(&now_time);

	*delta_time_s = (now_time.tv_sec - time_stamp->tv_sec);

	/* remember this time */
	*time_stamp = now_time;
	return 0;
}

#define SOC_CHANGE_PER_SEC          60
static int general_battery_get_batt_soc(struct general_battery *chip)
{
	int delta_time = 0;
    int soc_changed;
    int time_since_last_change_sec;
    struct timespec last_change_time;
    int usb_present = chip->usb_present;
    int last_batt_soc = chip->calulated_soc;
    int batt_soc = chip->fg_func(BATTERY_COMMON_CMD_GET_BATT_SOC, NULL);

    /** Recharge Issue */
    if( usb_present && last_batt_soc==100 && (batt_soc==99 || batt_soc==98) )
    {
        batt_soc = 100;
        general_battery_set_charge_enable(chip);
        BAT_DEG(BAT_LOG_CRTI,"Recharge:set soc to 100\n");
    }

    /** Charge Done Issue */
    if( chip->chg_status == POWER_SUPPLY_STATUS_FULL
            && (batt_soc >= 95 && batt_soc < 100))
    {
        batt_soc = 100;
        BAT_DEG(BAT_LOG_CRTI,"Charge Done:set soc to 100\n");
    }

    /** Power-off Issue */
    if(last_batt_soc != 0 && batt_soc == 0 && !usb_present)
    {
        if(chip->batt_vol > BATTERY_CUTOFF_VOLTAGE && chip->batt_vol < BATTERY_MAX_FLOAT_VOLTAGE)
        {
            batt_soc = 1;
            pr_info("batt_vol is above 3400,set soc to 1 \n");
        }
    }

    last_change_time = chip->last_soc_change_time;
	calculate_delta_time(&last_change_time, &time_since_last_change_sec);

	delta_time = time_since_last_change_sec/ SOC_CHANGE_PER_SEC;

	if(delta_time < 0) delta_time = 0;

    soc_changed = min(1, delta_time);

    BAT_DEG(BAT_LOG_CRTI,"soc_changed:%d, interval_sec:%d\n", soc_changed, time_since_last_change_sec);

    if(last_batt_soc >= 0)
    {
        /** Charging */
        if( last_batt_soc < batt_soc && chip->batt_ma <= 0 )
            /** Replace the soc when resuming device */
            last_batt_soc = chip->was_resuming ? batt_soc : last_batt_soc + soc_changed;
        /** Discharing */
        else if( last_batt_soc > batt_soc && chip->batt_ma > 0 )
            /** Replace the soc when resuming device */
            last_batt_soc = chip->was_resuming ? batt_soc : last_batt_soc - soc_changed;
        chip->was_resuming = false;
    }
    else{
        /**Report the soc when bootup*/
        last_batt_soc = batt_soc;
    }

    if(chip->calulated_soc != last_batt_soc)
    {
        chip->need_update = true;
        chip->calulated_soc = bound_soc(last_batt_soc);
        chip->last_soc_change_time = last_change_time;
    }

    BAT_DEG(BAT_LOG_CRTI,"Raw soc:%d, Calulated soc:%d, time since last change:%ld \n", batt_soc, chip->calulated_soc, chip->last_soc_change_time.tv_sec);
    return chip->calulated_soc;
}

#if 0
static int general_battery_get_batt_present()
{
    int batt_present = 0;
    if (pmic_get_register_value(MT6351_PMIC_BATON_TDET_EN))
    {
        pmic_set_register_value(MT6351_PMIC_BATON_TDET_EN, 1);
        pmic_set_register_value(MT6351_PMIC_RG_BATON_EN, 1);
        batt_present = pmic_get_register_value(MT6351_PMIC_RGS_BATON_UNDET);
    }

    return batt_present;
}
#endif

static int general_battery_get_batt_voltage(struct general_battery *chip)
{
    int batt_vol = chip->fg_func(BATTERY_COMMON_CMD_GET_BATT_VOLT, NULL);

    chip->batt_vol = batt_vol;

    return batt_vol;
}

static int general_battery_get_usb_parameters(struct general_battery *chip)
{
    int chg_status, batt_health, usb_present;
    union power_supply_propval val = {0,};

    if (chip->usb_psy == NULL || chip->usb_psy < 0)
        chip->usb_psy = power_supply_get_by_name("usb");

    if(!chip->usb_psy)
        return -1;

    chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_STATUS, &val);
    chg_status = val.intval;

    if(chip->chg_status != chg_status){
        chip->chg_status = chg_status;
        chip->need_update = true;
    }

    chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_HEALTH, &val);
    batt_health = val.intval;
    if(chip->batt_health != batt_health){
        chip->batt_health = batt_health;
        chip->need_update = true;
    }

    usb_present = pmic_get_register_value(PMIC_RGS_CHRDET);
    if(chip->usb_present != usb_present){
        chip->usb_present = usb_present;
        chip->need_update = true;
    }
    BAT_DEG(BAT_LOG_CRTI,"s:%d, h:%d. u:%d\n", chip->chg_status, chip->batt_health, chip->usb_present);
    return 0;
}

static int general_battery_set_batt_property(struct power_supply *psy,
        enum power_supply_property psp,
        const union power_supply_propval *val)
{
    struct general_battery *chip = container_of(psy, struct general_battery,
                                   batt_psy);
    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        chip->chg_status = val->intval;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        chip->batt_health = val->intval;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int general_battery_get_batt_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct general_battery *chip = container_of(psy, struct general_battery,
                                   batt_psy);
    switch (psp)
    {
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = 1;
        break;
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = chip->chg_status;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        val->intval = chip->cycle_count;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        val->intval = 4350 * 1000;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = general_battery_get_batt_voltage(chip);
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = chip->batt_health;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = general_battery_get_batt_current(chip);
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        val->intval = chip->batt_dcc;
        break;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        val->intval = chip->batt_rmc;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = chip->battery_full_design_capacity;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = chip->calulated_soc;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval =  chip->batt_temp;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

#define MONITOR_SOC_WAIT_THRESHOLDS         15
#define MONITOR_SOC_WAIT_MS                 30000
#define MONITOR_SOC_WAIT_FAST_MS            10000
#define SCHEDULE_MONITOR_SOC_WAIT_MS		1000
static void general_battery_soc_monitor_work(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct general_battery *chip = container_of(dwork,
                                   struct general_battery, soc_monitor_work);
    /** Update battery informations */
    general_battery_get_cycle_count(chip);
    general_battery_get_batt_voltage(chip);
    general_battery_get_batt_temperature(chip);
    general_battery_get_batt_current(chip);
    general_battery_get_batt_dcc(chip);
    general_battery_get_batt_fcc(chip);
    general_battery_get_batt_rmc(chip);

    if(chip->batt_fcc <= 0){
        general_battery_hw_reset(chip);
        BAT_DEG(BAT_LOG_CRTI, "battery fcc or rmc error, reset fuel gauge.\n");
    }
    /** Get usb informations */
    general_battery_get_usb_parameters(chip);

    /** Set fuel gauge temperature */
    general_battery_set_batt_temperature(chip);

    /** Update battery soc */
    general_battery_get_batt_soc(chip);
#ifdef CONFIG_GENERAL_PM_DEBUG
	general_battery_dump_hw_register(chip);
#endif
    if(wake_lock_active(&chip->monitor_wake_lock) && chip->batt_vol > BATTERY_CUTOFF_VOLTAGE)
    {
        BAT_DEG(BAT_LOG_FULL,"---- release wakeup lock.\n");
        wake_unlock(&chip->monitor_wake_lock);
    }

    if(chip->need_update){
        BAT_DEG(BAT_LOG_CRTI,"Sync information\n");
        power_supply_changed(&chip->batt_psy);
        chip->need_update = false;
    }

    if(chip->usb_present || chip->calulated_soc < MONITOR_SOC_WAIT_THRESHOLDS)
        schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(MONITOR_SOC_WAIT_FAST_MS));
    else
        schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(MONITOR_SOC_WAIT_MS));

    BAT_DEG(BAT_LOG_CRTI,"s:%d,v:%d,t:%d,u:%d,c:%d,fcc:%d, rmc:%d, dcc:%d.\n",
                    chip->calulated_soc, chip->batt_vol, chip->batt_temp, chip->usb_present, chip->batt_ma, chip->batt_fcc, chip->batt_rmc, chip->batt_dcc);
}

static irqreturn_t general_battery_soc_interrupt_handler(int irq, void *data)
{
    struct general_battery *chip = (struct general_battery *)data;

    if (!wake_lock_active(&chip->monitor_wake_lock))
    {
        BAT_DEG(BAT_LOG_FULL,"---- handle wakeup lock.\n");
        wake_lock(&chip->monitor_wake_lock);
    }

    schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(SCHEDULE_MONITOR_SOC_WAIT_MS));

    return IRQ_HANDLED;
}

static ssize_t general_battery_debugfs_write(struct file *file, const char *buf, size_t count, loff_t *data)
{
    struct seq_file *s = file->private_data;
    struct general_battery *chip = s->private;
    char cmd[16];
    int status = count;

    memset(cmd, 0x00, sizeof(cmd));

    if (copy_from_user(&cmd, buf, min_t(size_t, sizeof(buf) - 1, count))) {
        status = -EFAULT;
        goto out;
    }

    if (!strncmp(cmd, "update", 6)) {
        general_battery_update_batt_profile(chip);
    } else if (!strncmp(cmd, "reset", 5)) {
        general_battery_hw_reset(chip);
    } else {
        status = -EINVAL;
        goto out;
    }
out:
	return status;
}

static int general_battery_debugfs_status(struct seq_file *m, void *v)
{
	seq_printf(m, "Success.\n");
	return 0;
}

static int general_battery_debugfs_open(struct inode *inode, struct file *file)
{
	struct general_battery *chip = inode->i_private;
	return single_open(file, general_battery_debugfs_status, chip);
}

static const struct file_operations general_battery_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= general_battery_debugfs_open,
	.read		= seq_read,
    .write      = general_battery_debugfs_write,
};

static void general_battery_create_debugfs_entries(struct general_battery *chip)
{

	chip->debug_root = debugfs_create_dir("bat_debug", NULL);
	if (!chip->debug_root)
		pr_err("Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("command", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &general_battery_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create lbc_config debug file\n");
	}
}

static int general_battery_parse_dt_parameters(struct general_battery *chip)
{
    int rc;
    struct device_node *node = chip->dev->of_node;
    if (!node) {
        dev_err(chip->dev, "device tree chip. missing\n");
        return -EINVAL;
    }

    rc = of_property_read_u32(node, "nubia,battery_full_design_capacity",
                                               &chip->battery_full_design_capacity);
    if (rc < 0)
        chip->battery_full_design_capacity = -EINVAL;

    return rc;
}

static int general_battery_probe(struct platform_device *pdev)
{
    int ret;
    unsigned int debounce_enable;
#ifdef CONFIG_OF
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_cfg;
    u32 ints[2];
    struct device_node *node;
#endif
    struct general_battery *chip =
        kzalloc(sizeof(struct general_battery), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    /** Get parent data */
    chip->dev = &pdev->dev;
    /** Power supply */
    if (chip->usb_psy == NULL)
        chip->usb_psy = power_supply_get_by_name("usb");

    chip->calulated_soc = -1;

    general_battery_hw_init(chip);

    /** Battery supply */
    chip->batt_psy.name = "battery";
    chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
    chip->batt_psy.properties = battery_batt_props;
    chip->batt_psy.num_properties = ARRAY_SIZE(battery_batt_props);
    chip->batt_psy.get_property = general_battery_get_batt_property;
    chip->batt_psy.set_property = general_battery_set_batt_property;

    /** Register USB charger class */
    ret = power_supply_register(chip->dev, &chip->batt_psy);
    if (ret)
    {
        dev_err(chip->dev, "failed to register USB charger\n");
        goto free_mem;
    }

    /** Create a work queue for the charger */
    wake_lock_init(&chip->monitor_wake_lock, WAKE_LOCK_SUSPEND,	"soc_monitor_lock");
    INIT_DELAYED_WORK(&chip->soc_monitor_work, general_battery_soc_monitor_work);

#ifdef CONFIG_OF
    pinctrl = devm_pinctrl_get(chip->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        pr_err("Cannt find chr_stat pinctrl.\n");
    }

    pinctrl_cfg = pinctrl_lookup_state(pinctrl, "fg_stat_cfg");
    if (IS_ERR(pinctrl_cfg)) {
        ret = PTR_ERR(pinctrl_cfg);
        pr_err("Cannt find chr_stat pinctrl fg_stat_cfg.\n");
    }
    else{
        pr_err("Success to select fg_stat_cfg.\n");
        pinctrl_select_state(pinctrl, pinctrl_cfg);
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,ext_fg_stat");
    if(node)
    {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        chip->soc_int_gpio = ints[0];
        debounce_enable = ints[1];
    }
#if defined(CONFIG_MTK_LEGACY)
    mt_gpio_set_debounce(chip->soc_int_gpio, debounce_enable);
#else
    gpio_request(chip->soc_int_gpio, "soc_irq");
#endif
    chip->soc_irq = irq_of_parse_and_map(node, 0);
    ret = devm_request_threaded_irq(chip->dev, chip->soc_irq, NULL, general_battery_soc_interrupt_handler,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                    "soc_irq", chip);
    enable_irq_wake(chip->soc_irq);
    if(ret < 0)
    {
        BAT_DEG(BAT_LOG_CRTI,"request irq failed\n");
        goto free_batt_psy;
    }
#endif

	general_battery_parse_dt_parameters(chip);
    general_battery_create_debugfs_entries(chip);
    platform_set_drvdata(pdev, chip);

    schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(SCHEDULE_MONITOR_SOC_WAIT_MS));
    BAT_DEG(BAT_LOG_CRTI,"General battery probe done.\n");
    return ret;
free_batt_psy:
    power_supply_unregister(&chip->batt_psy);
free_mem:
    kfree(chip);
    return ret;
}

struct platform_device general_battery_device =
{
    .name = "general_battery",
    .id = -1,
};

struct of_device_id battery_match_table[] = {
        { .compatible = "nubia, general_battery", },
        {},
};

#ifdef CONFIG_PM_SLEEP
static int general_battery_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct general_battery *chip = platform_get_drvdata(pdev);
    BAT_DEG(BAT_LOG_CRTI,"general_battery suspend.\n");
    cancel_delayed_work(&chip->soc_monitor_work);
    return 0;
}

static int general_battery_resume(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct general_battery *chip = platform_get_drvdata(pdev);
    BAT_DEG(BAT_LOG_CRTI,"general_battery resume.\n");
    chip->was_resuming = true;
    schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(SCHEDULE_MONITOR_SOC_WAIT_MS));
    return 0;
}

static const struct dev_pm_ops general_battery_pm_ops =
{
    .suspend 	= general_battery_suspend,
    .resume 	= general_battery_resume,
};

#endif

static struct platform_driver general_battery_driver =
{
    .probe = general_battery_probe,
    .driver = {
        .name = "general_battery",
#ifdef CONFIG_PM_SLEEP
        .pm	  = &general_battery_pm_ops,
#endif
        .of_match_table = battery_match_table,
    },
};

static int __init general_battery_init(void)
{
    int ret;

    pr_debug("general_charger_init\n");

    ret = platform_driver_register(&general_battery_driver);
    if (ret)
        BAT_DEG(BAT_LOG_CRTI,"Unable to register driver.\n");

    BAT_DEG(BAT_LOG_CRTI,"Initialization : DONE !!\n");
    return 0;
}

static void __exit general_battery_exit(void)
{
}
module_init(general_battery_init);
module_exit(general_battery_exit);

MODULE_AUTHOR("Oscar Liu");
MODULE_DESCRIPTION("Battery Device Driver");
MODULE_LICENSE("GPL");