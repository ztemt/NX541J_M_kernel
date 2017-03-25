#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include "bq25896_charger.h"
#include <linux/power/general_charger.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/mt_sleep.h>
#if defined(CONFIG_MTK_FPGA)
#else
#ifdef CONFIG_OF
#else
#include <cust_i2c.h>
#endif
#endif

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
static struct i2c_client *new_client;
static const struct i2c_device_id bq25896_i2c_id[] = { {"bq25896", 0}, {} };

unsigned char bq25896_reg[BQ25896_REG_NUM] = { 0 };
static DEFINE_MUTEX(bq25896_i2c_access);

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

/* ============================================================ */
/* Static Function */
/* ============================================================ */
static void bq25896_set_en_hiz(unsigned int val);
static void bq25896_set_en_ilim(unsigned int val);
static void bq25896_dump_register(void);
static void bq25896_chg_en(unsigned int val);
static void bq25896_set_vreg(unsigned int val);
static unsigned int bq25896_get_ichg(void);
static unsigned int bq25896_get_vbus_state(void);
static void bq25896_set_iinlim(unsigned int val);
//static void bq25896_set_vindpm(unsigned int val);
static void bq25896_en_chg_timer(unsigned int val);
static void bq25896_dump_register(void);
static unsigned int bq25896_get_vbus(void);

static unsigned int charging_error;
static unsigned int bq25896_charger_get_error_state(void);
static void bq25896_set_ichg(unsigned int val);
static int bq25896_get_otg_status(void);
static void bq25896_set_otg_status(unsigned int val);
static unsigned int bq25896_get_charger_online(void);
static unsigned int bq25896_get_vbat(void);
/* ============================================================ // */

/**********************************************************
  *
  *   [I2C Function For Read/Write bq25896]
  *
  *********************************************************/
unsigned int bq25896_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char cmd_buf[1] = { 0x00 };
    char readData = 0;
    int ret = 0;

    mutex_lock(&bq25896_i2c_access);

    /* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
    new_client->ext_flag =
    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
    if (ret < 0)
    {
        /* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
        new_client->ext_flag = 0;
        mutex_unlock(&bq25896_i2c_access);

        return 0;
    }

    readData = cmd_buf[0];
    *returnData = readData;

    /* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
    new_client->ext_flag = 0;
    mutex_unlock(&bq25896_i2c_access);

    return 1;
}

unsigned int bq25896_write_byte(unsigned char cmd, unsigned char writeData)
{
    char write_data[2] = { 0 };
    int ret = 0;

    mutex_lock(&bq25896_i2c_access);

    write_data[0] = cmd;
    write_data[1] = writeData;

    new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0)
    {
        new_client->ext_flag = 0;
        mutex_unlock(&bq25896_i2c_access);
        return 0;
    }

    new_client->ext_flag = 0;
    mutex_unlock(&bq25896_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq25896_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
unsigned char SHIFT)
{
    unsigned char bq25896_reg = 0;
    unsigned int ret = 0;

    ret = bq25896_read_byte(RegNum, &bq25896_reg);

    BAT_DEG(BAT_LOG_FULL, "[bq25896_read_interface] Reg[%x]=0x%x\n", RegNum, bq25896_reg);

    bq25896_reg &= (MASK << SHIFT);
    *val = (bq25896_reg >> SHIFT);

    BAT_DEG(BAT_LOG_FULL, "[bq25896_read_interface] val=0x%x\n", *val);

    return ret;
}

unsigned int bq25896_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char bq25896_reg = 0;
    unsigned int ret = 0;

    ret = bq25896_read_byte(RegNum, &bq25896_reg);
    BAT_DEG(BAT_LOG_FULL, "[bq25896_config_interface] Reg[%x]=0x%x\n", RegNum, bq25896_reg);

    bq25896_reg &= ~(MASK << SHIFT);
    bq25896_reg |= (val << SHIFT);

    ret = bq25896_write_byte(RegNum, bq25896_reg);
    BAT_DEG(BAT_LOG_FULL, "[bq25896_config_interface] write Reg[%x]=0x%x\n", RegNum,
    bq25896_reg);

    /* Check */
    /* bq25896_read_byte(RegNum, &bq25896_reg); */
    /* printk("[bq25896_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq25896_reg); */

    return ret;
}

static unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
const unsigned int val)
{
    unsigned int i;

    BAT_DEG(BAT_LOG_FULL, "array_size = %d \r\n", array_size);

    for (i = 0; i < array_size; i++)
    {
        if (val == *(parameter + i))
            return i;
    }

    BAT_DEG(BAT_LOG_CRTI, "NO register value match \r\n");
    /* TODO: ASSERT(0);    // not find the value */
    return 0;
}

static unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
    if (val < array_size)
    {
        return parameter[val];
    }
    else
    {
        BAT_DEG(BAT_LOG_CRTI, "Can't find the parameter \r\n");
        return parameter[0];
    }
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
unsigned int level)
{
    unsigned int i;
    unsigned int max_value_in_last_element;

    if (pList[0] < pList[1])
        max_value_in_last_element = 1;
    else
        max_value_in_last_element = 0;

    if (max_value_in_last_element == 1)
    {
        for (i = (number - 1); i != 0; i--)  	/* max value in the last element */
        {
            if (pList[i] <= level)
            {
                BAT_DEG(2, "zzf_%d<=%d     i=%d\n", pList[i], level, i);
                return pList[i];
            }
        }

        BAT_DEG(BAT_LOG_CRTI, "Can't find closest level \r\n");
        return pList[0];
        /* return CHARGE_CURRENT_0_00_MA; */
    }
    else
    {
        for (i = 0; i < number; i++)  	/* max value in the first element */
        {
            if (pList[i] <= level)
                return pList[i];
        }

        BAT_DEG(BAT_LOG_CRTI, "Can't find closest level \r\n");
        return pList[number - 1];
        /* return CHARGE_CURRENT_0_00_MA; */
    }
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */
static void bq25896_set_en_hiz(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON0),
    (unsigned char) (val),
    (unsigned char) (CON0_EN_HIZ_MASK),
    (unsigned char) (CON0_EN_HIZ_SHIFT)
                                  );
}

static void bq25896_set_en_ilim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON0),
    (unsigned char) (val),
    (unsigned char) (CON0_EN_ILIM_MASK),
    (unsigned char) (CON0_EN_ILIM_SHIFT)
                                  );
}

static void bq25896_set_iinlim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON0),
    (val),
    (unsigned char) (CON0_IINLIM_MASK),
    (unsigned char) (CON0_IINLIM_SHIFT)
                                  );
}

static unsigned int bq25896_get_iinlim(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON0),
    (&val),
    (unsigned char) (CON0_IINLIM_MASK), (unsigned char) (CON0_IINLIM_SHIFT)
                                );
    return val;
}

/* CON1---------------------------------------------------- */
static void bq25896_ADC_start(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON2),
    (unsigned char) (val),
    (unsigned char) (CON2_CONV_START_MASK),
    (unsigned char) (CON2_CONV_START_SHIFT)
                                  );
}

/* CON2---------------------------------------------------- */
static void bq25896_set_ico_en_start(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON2),
    (unsigned char) (val),
    (unsigned char) (CON2_ICO_EN_MASK),
    (unsigned char) (CON2_ICO_EN_RATE_SHIFT)
                                  );
}

/* CON3---------------------------------------------------- */
static void bq25896_set_otg_status(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON3),
    (val),
    (unsigned char) (CON3_OTG_CONFIG_MASK),
    (unsigned char) (CON3_OTG_CONFIG_SHIFT)
                                  );

}

static void bq25896_chg_en(unsigned int val)
{
    unsigned int ret = 0;


    ret = bq25896_config_interface((unsigned char) (BQ25896_CON3),
    (val),
    (unsigned char) (CON3_CHG_CONFIG_MASK),
    (unsigned char) (CON3_CHG_CONFIG_SHIFT)
                                  );

}

static unsigned int bq25896_get_chg_en(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON3),
	(&val),
    (unsigned char) (CON3_CHG_CONFIG_MASK),
    (unsigned char) (CON3_CHG_CONFIG_SHIFT)
                                );
    return val;
}


static void bq25896_set_sys_min(unsigned int val)
{
    unsigned int ret = 0;


    ret = bq25896_config_interface((unsigned char) (BQ25896_CON3),
    (val),
    (unsigned char) (CON3_SYS_V_LIMIT_MASK),
    (unsigned char) (CON3_SYS_V_LIMIT_SHIFT)
                                  );

}


static int bq25896_get_otg_status(void)
{
	int ret = STATUS_OK;
	unsigned char val;
    
    ret = bq25896_read_interface(	    (unsigned char)(BQ25896_CON3), 
										(unsigned char*)(&val), 
										(unsigned char)(CON3_OTG_CONFIG_MASK), 
										(unsigned char)(CON3_OTG_CONFIG_SHIFT)
									);
    return val;
}

/* CON4---------------------------------------------------- */
#ifdef BQ25896_PUMPX_UP
static void bq25896_en_pumpx(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON4),
    (unsigned char) (val),
    (unsigned char) (CON4_EN_PUMPX_MASK),
    (unsigned char) (CON4_EN_PUMPX_SHIFT)
                                  );
}
#endif

static void bq25896_set_ichg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON4),
    (unsigned char) (val),
    (unsigned char) (CON4_ICHG_MASK), (unsigned char) (CON4_ICHG_SHIFT)
                                  );
}

static unsigned int bq25896_get_reg_ichg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON4),
    (&val),
    (unsigned char) (CON4_ICHG_MASK), (unsigned char) (CON4_ICHG_SHIFT)
                                );
    return val;
}

/* CON5---------------------------------------------------- */
static void bq25896_set_iprechg(unsigned int val)
{
    unsigned int ret = 0;


    ret = bq25896_config_interface((unsigned char) (BQ25896_CON5),
    (val),
    (unsigned char) (CON5_IPRECHG_MASK),
    (unsigned char) (CON5_IPRECHG_SHIFT)
                                  );

}

static void bq25896_set_iterml(unsigned int val)
{
    unsigned int ret = 0;


    ret = bq25896_config_interface((unsigned char) (BQ25896_CON5),
    (val),
    (unsigned char) (CON5_ITERM_MASK), (unsigned char) (CON5_ITERM_SHIFT)
                                  );

}

/* CON6---------------------------------------------------- */
static void bq25896_set_vreg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON6),
    (unsigned char) (val),
    (unsigned char) (CON6_2XTMR_EN_MASK),
    (unsigned char) (CON6_2XTMR_EN_SHIFT)
                                  );
}

static void bq25896_set_batlowv(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON6),
    (unsigned char) (val),
    (unsigned char) (CON6_BATLOWV_MASK),
    (unsigned char) (CON6_BATLOWV_SHIFT)
                                  );
}

static void bq25896_set_vrechg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON6),
    (unsigned char) (val),
    (unsigned char) (CON6_VRECHG_MASK),
    (unsigned char) (CON6_VRECHG_SHIFT)
                                  );
}

/* CON7---------------------------------------------------- */
static void bq25896_en_term_chg(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON7),
    (unsigned char) (val),
    (unsigned char) (CON7_EN_TERM_CHG_MASK),
    (unsigned char) (CON7_EN_TERM_CHG_SHIFT)
                                  );
}

static void bq25896_set_wd_timer(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON7),
    (unsigned char) (val),
    (unsigned char) (CON7_WTG_TIM_SET_MASK),
    (unsigned char) (CON7_WTG_TIM_SET_SHIFT)
                                  );
}

static  void bq25896_en_chg_timer(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON7),
    (unsigned char) (val),
    (unsigned char) (CON7_EN_TIMER_MASK),
    (unsigned char) (CON7_EN_TIMER_SHIFT)
                                  );
}

static void bq25896_set_chg_timer(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON7),
    (unsigned char) (val),
    (unsigned char) (CON7_SET_CHG_TIM_MASK),
    (unsigned char) (CON7_SET_CHG_TIM_SHIFT)
                                  );
}

/* CON8--------------------------------------------------- */
static void bq25896_set_thermal_regulation(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON8),
    (unsigned char) (val),
    (unsigned char) (CON8_TREG_MASK), (unsigned char) (CON8_TREG_SHIFT)
                                  );
}

static void bq25896_set_vbat_clamp(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON8),
    (unsigned char) (val),
    (unsigned char) (CON8_VCLAMP_MASK),
    (unsigned char) (CON8_VCLAMP_SHIFT)
                                  );
}

static void bq25896_set_vbat_ir_compensation(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CON8),
    (unsigned char) (val),
    (unsigned char) (CON8_BAT_COMP_MASK),
    (unsigned char) (CON8_BAT_COMP_SHIFT)
                                  );
}

/* CON9---------------------------------------------------- */
#ifdef BQ25896_PUMPX_UP
static void bq25896_pumpx_up(unsigned int val)
{
    unsigned int ret = 0;

    bq25896_en_pumpx(1);
    if (val == 1)
    {
        ret = bq25896_config_interface((unsigned char) (BQ25896_CON9),
        (unsigned char) (1),
        (unsigned char) (CON9_PUMPX_UP),
        (unsigned char) (CON9_PUMPX_UP_SHIFT)
                                      );
    }
    else
    {
        ret = bq25896_config_interface((unsigned char) (BQ25896_CON9),
        (unsigned char) (1),
        (unsigned char) (CON9_PUMPX_DN),
        (unsigned char) (CON9_PUMPX_DN_SHIFT)
                                      );
    }
    /* Input current limit = 1500 mA, changes after port detection*/
    bq25896_set_iinlim(0x1E);
    /* CC mode current = 2048 mA*/
    bq25896_set_ichg(0x20);
    msleep(3000);
}
#endif

/* CONA---------------------------------------------------- */
static void bq25896_set_boost_ilim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CONA),
    (unsigned char) (val),
    (unsigned char) (CONA_BOOST_ILIM_MASK),
    (unsigned char) (CONA_BOOST_ILIM_SHIFT)
                                  );
}

static void bq25896_set_boost_vlim(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_CONA),
    (unsigned char) (val),
    (unsigned char) (CONA_BOOST_VLIM_MASK),
    (unsigned char) (CONA_BOOST_VLIM_SHIFT)
                                  );
}

/* CONB---------------------------------------------------- */
static unsigned int bq25896_get_vbus_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONB),
    (&val),
    (unsigned char) (CONB_VBUS_STAT_MASK),
    (unsigned char) (CONB_VBUS_STAT_SHIFT)
                                );
    return val;
}


static unsigned int bq25896_get_chrg_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONB),
    (&val),
    (unsigned char) (CONB_CHRG_STAT_MASK),
    (unsigned char) (CONB_CHRG_STAT_SHIFT)
                                );
    return val;
}

/* CON0C---------------------------------------------------- */
static unsigned int bq25896_get_chrg_fault_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONC),
    (&val),
    (unsigned char) (CONC_CHRG_FAULT_MASK),
    (unsigned char) (CONC_CHRG_FAULT_SHIFT)
                                );
    return val;
}
#ifdef BQ25896_REGISTER_STATE_DUMP
static unsigned int bq25896_get_wdt_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONC),
    (&val),
    (unsigned char) (CONB_WATG_STAT_MASK),
    (unsigned char) (CONB_WATG_STAT_SHIFT)
                                );
    return val;
}

static unsigned int bq25896_get_boost_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONC),
    (&val),
    (unsigned char) (CONB_BOOST_STAT_MASK),
    (unsigned char) (CONB_BOOST_STAT_SHIFT)
                                );
    return val;
}

static unsigned int bq25896_get_bat_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONC),
    (&val),
    (unsigned char) (CONB_BAT_STAT_MASK),
    (unsigned char) (CONB_BAT_STAT_SHIFT)
                                );
    return val;
}
#endif

/* COND */
static void bq25896_set_force_vindpm(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_COND),
    (unsigned char) (val),
    (unsigned char) (COND_FORCE_VINDPM_MASK),
    (unsigned char) (COND_FORCE_VINDPM_SHIFT)
                                  );
}

#if 0
static void bq25896_set_vindpm(unsigned int val)
{
    unsigned int ret = 0;

    ret = bq25896_config_interface((unsigned char) (BQ25896_COND),
    (unsigned char) (val),
    (unsigned char) (COND_VINDPM_MASK),
    (unsigned char) (COND_VINDPM_SHIFT)
                                  );
}
#endif

/* CONDE */
static unsigned int bq25896_get_vbat(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CONE),
    (&val),
    (unsigned char) (CONE_VBAT_MASK), (unsigned char) (CONE_VBAT_SHIFT)
                                );
    return val;
}

/* CON11 */
static unsigned int bq25896_get_vbus(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON11),
    (&val),
    (unsigned char) (CON11_VBUS_MASK), (unsigned char) (CON11_VBUS_SHIFT)
                                );
    return val;
}

static unsigned int bq25896_get_charger_online()
{
	unsigned int ret = 0;
	unsigned char val;
    
	ret = bq25896_read_interface(   (unsigned char)(BQ25896_CON11),
									(unsigned char*)(&val),
									(unsigned char)(CON11_VBUS_GD_MASK),
									(unsigned char)(CON11_VBUS_GD_SHIFT)
								);
	return val;
}

/* CON12 */
static unsigned int bq25896_get_ichg(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON12),
    (&val),
    (unsigned char) (CONB_ICHG_STAT_MASK),
    (unsigned char) (CONB_ICHG_STAT_SHIFT)
                                );
    return val;
}

/* CON13 */
#ifdef BQ25896_REGISTER_STATE_DUMP
static unsigned int bq25896_get_idpm_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON13),
    (&val),
    (unsigned char) (CON13_IDPM_STAT_MASK),
    (unsigned char) (CON13_IDPM_STAT_SHIFT)
                                );
    return val;
}
#endif

static unsigned int bq25896_get_vdpm_state(void)
{
    unsigned int ret = 0;
    unsigned char val = 0;

    ret = bq25896_read_interface((unsigned char) (BQ25896_CON13),
    (&val),
    (unsigned char) (CON13_VDPM_STAT_MASK),
    (unsigned char) (CON13_VDPM_STAT_SHIFT)
                                );
    return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
static void bq25896_dump_register(void)
{
    unsigned char i = 0;
    unsigned char ichg = 0;
    unsigned char ichg_reg = 0;
    unsigned char iinlim = 0;
    unsigned char vbat = 0;
    unsigned char chrg_state = 0;
    unsigned char chr_en = 0;
    unsigned char vbus = 0;
    unsigned char vdpm = 0;
    unsigned char fault = 0;

    bq25896_ADC_start(1);
    for (i = 0; i < BQ25896_REG_NUM; i++)
    {
        bq25896_read_byte(i, &bq25896_reg[i]);
        BAT_DEG(BAT_LOG_CRTI, "[bq25896 reg@][0x%x]=0x%x ", i, bq25896_reg[i]);
    }
    bq25896_ADC_start(1);
    iinlim = bq25896_get_iinlim();
    chrg_state = bq25896_get_chrg_state();
    chr_en = bq25896_get_chg_en();
    ichg_reg = bq25896_get_reg_ichg();
    ichg = bq25896_get_ichg();
    vbat = bq25896_get_vbat();
    vbus = bq25896_get_vbus();
    vdpm = bq25896_get_vdpm_state();
    fault = bq25896_get_chrg_fault_state();
    BAT_DEG(BAT_LOG_CRTI,
    "[PE+]BQ25896 Ichg_reg=%d mA, Iinlin=%d mA, Vbus=%d mV, err=%d",
    ichg_reg * 64, iinlim * 50 + 100, vbus * 100 + 2600, fault);
    BAT_DEG(BAT_LOG_CRTI, "[PE+]BQ25896 Ichg=%d mA, Vbat =%d mV, ChrStat=%d, CHGEN=%d, VDPM=%d\n",
    ichg * 50, vbat * 20 + 2304, chrg_state, chr_en, vdpm);

}

void bq25896_hw_init(void)
{
    bq25896_dump_register();
}

static unsigned int bq25896_charger_hw_init(void *data)
{
    unsigned int status = STATUS_OK;
    bq25896_set_en_ilim(0x00);      //enable ilimit pin
    bq25896_config_interface(BQ25896_CON0, 0x0, 0x03, 6);	/* Disable ilimit Pin */
    bq25896_config_interface(BQ25896_CON0, 0x1C, 0x3F, 0);	/* enable ilimit Pin */
    bq25896_config_interface(BQ25896_CON1, 0x1, 0x01, 0);	/* Vindpm offset  600MV */
    bq25896_config_interface(BQ25896_CON1, 0x0, 0x01, 1);	/* Disable 12V HVDCP */

    bq25896_config_interface(BQ25896_CON2, 0x0, 0x1, 5);	/* boost freq 1.5MHz when OTG_CONFIG=1 */
    bq25896_set_ico_en_start(0x01); //enable ico Algorithm
    bq25896_config_interface(BQ25896_CON2, 0x1, 0x1, 4);	/* enable ico Algorithm -->bear:en */
    bq25896_config_interface(BQ25896_CON2, 0x1, 0x1, 3);	/* enable HV DCP  */
    bq25896_config_interface(BQ25896_CON2, 0x1, 0x1, 2);	/* enable MaxCharge  */
    bq25896_config_interface(BQ25896_CON2, 0x1, 0x1, 1);	/* disable DPDM detection */
    bq25896_config_interface(BQ25896_CON2, 0x1, 0x1, 0);	/* enable auto DPDM detection */

    bq25896_config_interface(BQ25896_CON3, 0x0, 0x1, 6);
	bq25896_config_interface(BQ25896_CON3, 0x0, 0x1, 4);	/* disable change as default */
    bq25896_set_sys_min(0x03);   //System min voltage default 3.5V
    bq25896_config_interface(BQ25896_CON3, 0x3, 0x7, 1);	/* System min voltage default 3.5V */

    /*PreCC mode */
    bq25896_set_iprechg(0x07);  //precharge current default 448mA
    bq25896_set_iterml(0x02);   //termianation current default 128mA
    bq25896_config_interface(BQ25896_CON5, 0x7, 0xF, 4);	/* precharge current default 448mA */
    bq25896_config_interface(BQ25896_CON5, 0x2, 0x7, 0);	/* termianation current default 128mA */
    bq25896_set_batlowv(0x01); //precharge2cc voltage,BATLOWV, 3.0V 
    bq25896_config_interface(BQ25896_CON6, 0x1, 0x1, 1);	/* precharge2cc voltage,BATLOWV, 3.0V */
    bq25896_config_interface(BQ25896_CON6, 0x20, 0x3F, 2);	/* BQ25896_VREG=CV 4.352V (default 4.208V) */
    bq25896_set_vrechg(0x00);
    bq25896_config_interface(BQ25896_CON6, 0x0, 0x1, 0);	/* recharge voltage@VRECHG=CV-100MV */
    bq25896_en_term_chg(0x01); // enable BQ25896_ICHG termination detect
    bq25896_config_interface(BQ25896_CON7, 0x1, 0x1, 7);	/* enable BQ25896_ICHG termination detect */
    bq25896_set_wd_timer(0x00); //disable watchdog sec
    bq25896_set_chg_timer(0x00); //set charging timer 5h
    bq25896_en_chg_timer(0x01); //enable charging timer
    bq25896_config_interface(BQ25896_CON7, 0x00, 0x3, 4);	/* disable  watch dog  secs 0x3 */
    bq25896_config_interface(BQ25896_CON7, 0x1, 0x1, 3);	/* enable charging timer safty timer */
    bq25896_config_interface(BQ25896_CON7, 0x0, 0x3, 1);	/* charging timer 5h */
    bq25896_config_interface(BQ25896_CON7, 0x1, 0x1, 0);	/* JEITA_ISet : 20% x BQ25896_ICHG */

    bq25896_set_vbat_clamp(0x00);
    bq25896_set_vbat_ir_compensation(0x00);
    bq25896_set_thermal_regulation(0x03);   //thermal 120 as default
    bq25896_config_interface(BQ25896_CON8, 0x3, 0x7, 5);	/* enable ir_comp_resistance */
    bq25896_config_interface(BQ25896_CON8, 0x0, 0x7, 2);	/* enable ir_comp_vdamp */
    bq25896_config_interface(BQ25896_CON8, 0x3, 0x3, 0);	/* thermal 120 default */

    bq25896_config_interface(BQ25896_CON9, 0x0, 0x1, 4);	/* JEITA_VSET: BQ25896_VREG-200mV */

    bq25896_set_boost_ilim(0x01);   //boost current limit 0.5A
    bq25896_set_boost_vlim(0x04);   //boost voltagte 4.998V default
    bq25896_config_interface(BQ25896_CONA, 0x7, 0xF, 4);	/* boost voltagte 4.998V default */
    bq25896_config_interface(BQ25896_CONA, 0x3, 0x7, 0);	/* boost current limit 0.5A */

    /*Vbus current limit */
    bq25896_set_force_vindpm(0x00);     //vindpm vth 0:relative 1:absolute
    //bq25896_set_vindpm(0x15);
    bq25896_config_interface(BQ25896_COND, 0x0, 0x1, 7);	/* vindpm vth 0:relative 1:absolute */
    //bq25896_config_interface(BQ25896_COND, 0x15, 0x7F, 0);	/* absolute VINDPM = 2.6 + code x 0.1 =4.6V;K2 24261 4.452V */
    return status;
}


static unsigned int bq25896_charger_dump_register(void *data)
{
    unsigned int status = STATUS_OK;

    BAT_DEG(BAT_LOG_FULL, "charging_dump_register\r\n");
    bq25896_dump_register();

    return status;
}

static unsigned int bq25896_charger_enable(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int enable = *(unsigned int *) (data);

    if (1 == enable)
    {
        /* bq25896_config_interface(BQ25896_CON3, 0x1, 0x1, 4); //enable charging */
        bq25896_set_en_hiz(0x0);
        bq25896_chg_en(enable);
    }
    else
    {
        /* bq25896_config_interface(BQ25896_CON3, 0x0, 0x1, 4); //enable charging */
        bq25896_chg_en(enable);
        if (bq25896_charger_get_error_state())
            BAT_DEG(BAT_LOG_CRTI, "[charging_enable] under test mode: disable charging\n");
    }

    return status;
}

static unsigned int bq25896_charger_set_cv_voltage(void *data)
{
    unsigned int status;
    unsigned short array_size;
    unsigned int set_cv_voltage;
    unsigned short register_value;
    /*static kal_int16 pre_register_value; */

    array_size = GETARRAYNUM(BQ25896_VREG);
    status = STATUS_OK;
    /*pre_register_value = -1; */
    BAT_DEG(BAT_LOG_CRTI, "charging_set_cv_voltage set_cv_voltage=%d\n",
    *(unsigned int *) data);
    set_cv_voltage = bmt_find_closest_level(BQ25896_VREG, array_size, *(unsigned int *) data);
    //battery_set_cv_voltage(set_cv_voltage);
    register_value =
    charging_parameter_to_value(BQ25896_VREG, GETARRAYNUM(BQ25896_VREG), set_cv_voltage);
    BAT_DEG(BAT_LOG_FULL, "charging_set_cv_voltage register_value=0x%x\n", register_value);
    bq25896_set_vreg(register_value);

    return status;
}


static unsigned int bq25896_charger_get_charging_current(void *data)
{
    unsigned int ibatt;
    unsigned int array_size;
    /*unsigned char reg_value; */
    unsigned int val;

    /*Get current level */
    array_size = GETARRAYNUM(BQ25896_ICHG);
    val = bq25896_get_ichg();

    ibatt = charging_value_to_parameter(BQ25896_ICHG, array_size, val);
    return ibatt;
}

static unsigned int bq25896_charger_get_charger_voltage(void *data)
{
	unsigned char reg;
	reg = bq25896_get_vbus();
	return reg * 100 + 2600;
}


static unsigned int bq25896_charger_set_charging_current(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int set_chr_current;
    unsigned int array_size;
    unsigned int register_value;
    unsigned int current_value = *(unsigned int *) data * 1000;

    array_size = GETARRAYNUM(BQ25896_ICHG);
    set_chr_current = bmt_find_closest_level(BQ25896_ICHG, array_size, current_value);
    register_value = charging_parameter_to_value(BQ25896_ICHG, array_size, set_chr_current);
    bq25896_config_interface(BQ25896_CON4, register_value, 0x80, 1);

    bq25896_set_ichg(register_value);

    return status;
}

static unsigned int bq25896_charger_set_input_current(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int current_value = *(unsigned int *) data * 1000;
    unsigned int set_chr_current;
    unsigned int array_size;
    unsigned int register_value;

    array_size = GETARRAYNUM(BQ25896_IINLIM);
    set_chr_current = bmt_find_closest_level(BQ25896_IINLIM, array_size, current_value);
    register_value = charging_parameter_to_value(BQ25896_IINLIM, array_size, set_chr_current);
    bq25896_set_iinlim(register_value);

    return status;
}

static unsigned int bq25896_charger_get_input_current(void *data)
{
    unsigned int status = STATUS_OK;

    return status;
}

static unsigned int bq25896_charger_get_charger_present(void *data)
{
	return bq25896_get_charger_online();
}

static unsigned int bq25896_charger_get_charging_status(void *data)
{
    unsigned char reg;

    bq25896_read_interface(BQ25896_CONB, &reg, 0x3, 3);	/* BQ25896_ICHG to BAT */
	
	if (reg == 0) /* Ready */
		return CHARGE_STATE_NO_CHG;
	else if (reg == 1) 
		return CHARGE_STATE_PRE_CHG; /* Charge in progress */
	else if (reg ==2)
		return CHARGE_STATE_FAST_CHG;
	else if (reg == 3) /* Charge done */
		return CHARGE_STATE_CHG_DONE;
    
    return CHARGE_STATE_NO_CHG;
}

static unsigned int bq25896_charger_get_battery_voltage(void *data)
{
	unsigned int vbat;
	vbat = bq25896_get_vbat();
    return vbat * 20 + 2304;
}

static unsigned int bq25896_charger_get_charger_type(void *data)
{
    unsigned int type;
	type = bq25896_get_vbus_state();
    return type;
}

static unsigned int bq25896_charger_get_error_state(void)
{
    return charging_error;
}

static unsigned int bq25896_charger_set_otg_status(void *data)
{
	unsigned int enable = *(unsigned int *) (data);
	bq25896_set_otg_status(enable);
    return 0;
}

static unsigned int bq25896_charger_get_otg_status(void *data)
{
	int status = bq25896_get_otg_status();
	return status;
}

static unsigned int(*charging_func[CHARGER_COMMON_CMD_NUMBER]) (void *data);

int bq25896_control_interface(CHARGER_COMMON_CTRL_CMD cmd, void *data)
{
    int status;
    static signed int init = -1;

    if (init == -1)
    {
        init = 0;
        charging_func[CHARGER_COMMON_CMD_INIT] = bq25896_charger_hw_init;
        charging_func[CHARGER_COMMON_CMD_DUMP_REGISTER] = bq25896_charger_dump_register;
        charging_func[CHARGER_COMMON_CMD_ENABLE] = bq25896_charger_enable;
        charging_func[CHARGER_COMMON_CMD_SET_CV_VOLTAGE] = bq25896_charger_set_cv_voltage;
        charging_func[CHARGER_COMMON_CMD_GET_CHARGING_CURRENT] = bq25896_charger_get_charging_current;
        charging_func[CHARGER_COMMON_CMD_SET_CHARGING_CURRENT] = bq25896_charger_set_charging_current;
        charging_func[CHARGER_COMMON_CMD_SET_INPUT_CURRENT] = bq25896_charger_set_input_current;
        charging_func[CHARGER_COMMON_CMD_GET_INPUT_CURRENT] = bq25896_charger_get_input_current;
        charging_func[CHARGER_COMMON_CMD_GET_BATTERY_VOLTAGE] = bq25896_charger_get_battery_voltage;
		charging_func[CHARGER_COMMON_CMD_GET_CHARGER_PRESENT] = bq25896_charger_get_charger_present;
        charging_func[CHARGER_COMMON_CMD_GET_CHARGING_STATUS] = bq25896_charger_get_charging_status;
        charging_func[CHARGER_COMMON_CMD_GET_CHARGER_TYPE] = bq25896_charger_get_charger_type;
		charging_func[CHARGER_COMMON_CMD_SET_OTG_STATUS] = bq25896_charger_set_otg_status;
		charging_func[CHARGER_COMMON_CMD_GET_OTG_STATUS] = bq25896_charger_get_otg_status;
		charging_func[CHARGER_COMMON_CMD_GET_CHARGER_VOLTAGE] = bq25896_charger_get_charger_voltage;
    }

    if (cmd < CHARGER_COMMON_CMD_NUMBER)
    {
        if (charging_func[cmd] != NULL)
            status = charging_func[cmd](data);
        else
        {
            BAT_DEG(BAT_LOG_CRTI, "[chr_control_interface] cmd:%d does not support\n", cmd);
            return STATUS_UNSUPPORTED;
        }
    }
    else
    {
        BAT_DEG(BAT_LOG_CRTI, "[chr_control_interface] cmd:%d is not legal\n", cmd);
        return STATUS_UNSUPPORTED;
    }
    return status;

}

static struct power_supply chgr_psy;
enum power_supply_property bq25896_batt_props[] = {
	POWER_SUPPLY_PROP_CONTROL_FUNCTION,
};

static int bq25896_changer_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	switch (psp) {
    case POWER_SUPPLY_PROP_CONTROL_FUNCTION:
		val->int64val = (int64_t)bq25896_control_interface;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25896_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    BAT_DEG(BAT_LOG_CRTI, "[bq25896_driver_probe]\n");

    new_client = client;

    bq25896_dump_register();
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    chgr_psy.name = "parallel";
#else
	chgr_psy.name = "main";
#endif
	chgr_psy.type = POWER_SUPPLY_TYPE_USB;
	chgr_psy.properties = bq25896_batt_props;
	chgr_psy.num_properties = ARRAY_SIZE(bq25896_batt_props);
	chgr_psy.get_property = bq25896_changer_get_property;
	power_supply_register(&client->dev, &chgr_psy);

    return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq25896 = 0;
static ssize_t show_bq25896_access(struct device *dev, struct device_attribute *attr, char *buf)
{
    BAT_DEG(BAT_LOG_CRTI, "[show_bq25896_access] 0x%x\n", g_reg_value_bq25896);
    return sprintf(buf, "%u\n", g_reg_value_bq25896);
}

static ssize_t store_bq25896_access(struct device *dev, struct device_attribute *attr,
const char *buf, size_t size)
{
    int ret = 0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;

    BAT_DEG(BAT_LOG_CRTI, "[store_bq25896_access]\n");

    if (buf != NULL && size != 0)
    {
        BAT_DEG(BAT_LOG_CRTI, "[store_bq25896_access] buf is %s and size is %zu\n", buf,
        size);
        reg_address = simple_strtoul(buf, &pvalue, 16);
        /*ret = kstrtoul(buf, 16, reg_address); *//* This must be a null terminated string */
        if (size > 3)
        {
            reg_value = simple_strtoul((pvalue + 1), NULL, 16);
            /*ret = kstrtoul(buf + 3, 16, reg_value); */
            BAT_DEG(BAT_LOG_CRTI,
            "[store_bq25896_access] write bq25896 reg 0x%x with value 0x%x !\n",
            reg_address, reg_value);
            ret = bq25896_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {
            ret = bq25896_read_interface(reg_address, &g_reg_value_bq25896, 0xFF, 0x0);
            BAT_DEG(BAT_LOG_CRTI,
            "[store_bq25896_access] read bq25896 reg 0x%x with value 0x%x !\n",
            reg_address, g_reg_value_bq25896);
            BAT_DEG(BAT_LOG_CRTI,
            "[store_bq25896_access] Please use \"cat bq25896_access\" to get value\r\n");
        }
    }
    return size;
}

static DEVICE_ATTR(bq25896_access, 0664, show_bq25896_access, store_bq25896_access);	/* 664 */

static int bq25896_user_space_probe(struct platform_device *dev)
{
    int ret_device_file = 0;

    BAT_DEG(BAT_LOG_CRTI, "******** bq25896_user_space_probe!! ********\n");

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25896_access);

    return 0;
}

struct platform_device bq25896_user_space_device =
{
    .name = "bq25896-user",
    .id = -1,
};

static struct platform_driver bq25896_user_space_driver =
{
    .probe = bq25896_user_space_probe,
    .driver = {
        .name = "bq25896-user",
    },
};

#ifdef CONFIG_OF
static const struct of_device_id bq25896_of_match[] =
{
#ifdef CONFIG_NUBIA_PARALLEL_CHARGER_SUPPORT
    {.compatible = "mediatek,PARALLEL_CHARGER"},
#else    
    {.compatible = "mediatek,SWITHING_CHARGER"},
#endif
    {},
};
#else
static struct i2c_board_info i2c_bq25896 __initdata =
{
    I2C_BOARD_INFO("bq25896", (bq25896_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver bq25896_driver =
{
    .driver = {
        .name = "bq25896",
#ifdef CONFIG_OF
        .of_match_table = bq25896_of_match,
#endif
    },
    .probe = bq25896_driver_probe,
     .id_table = bq25896_i2c_id,
  };

static int __init bq25896_init(void)
{
    int ret = 0;

    /* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
    BAT_DEG(BAT_LOG_CRTI, "[bq25896_init] init start with i2c DTS");
#else
    BAT_DEG(BAT_LOG_CRTI, "[bq25896_init] init start. ch=%d\n", bq25896_BUSNUM);
    i2c_register_board_info(bq25896_BUSNUM, &i2c_bq25896, 1);
#endif
    if (i2c_add_driver(&bq25896_driver) != 0)
    {
        BAT_DEG(BAT_LOG_CRTI,
                    "[bq25896_init] failed to register bq25896 i2c driver.\n");
    }
    else
    {
        BAT_DEG(BAT_LOG_CRTI,
                    "[bq25896_init] Success to register bq25896 i2c driver.\n");
    }

    /* bq25896 user space access interface */
    ret = platform_device_register(&bq25896_user_space_device);
    if (ret)
    {
        BAT_DEG(BAT_LOG_CRTI, "****[bq25896_init] Unable to device register(%d)\n",
                    ret);
        return ret;
    }
    ret = platform_driver_register(&bq25896_user_space_driver);
    if (ret)
    {
        BAT_DEG(BAT_LOG_CRTI, "****[bq25896_init] Unable to register driver (%d)\n",
                    ret);
        return ret;
    }

    return 0;
}

static void __exit bq25896_exit(void)
{
    i2c_del_driver(&bq25896_driver);
}
subsys_initcall(bq25896_init);
module_exit(bq25896_exit);
