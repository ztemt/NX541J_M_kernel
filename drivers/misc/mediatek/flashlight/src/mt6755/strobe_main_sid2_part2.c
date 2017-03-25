/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
#ifdef CONFIG_ZTEMT_SINGLE_FLASHLIGHT
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[strobe_main_sid2_part2.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif


static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	pr_err("sub dummy ioctl");
	return 0;
}

static int sub_strobe_open(void *pArg)
{
	pr_err("sub dummy open");
	return 0;

}

static int sub_strobe_release(void *pArg)
{
	pr_err("sub dummy release");
	return 0;

}

static FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 strobeInit_main_sid2_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}

#else
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */
static struct work_struct workTimeOut;
static int g_timeOutTimeMs=0;
static u32 strobe_Res = 0;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

extern int flashEnable_LM3643_2(void);
extern int flashDisable_LM3643_2(void);
extern int setDuty_LM3643_2(int duty);
extern int FlashIc_Enable(void);
extern int FlashIc_Disable(void);
extern int m_duty2_flash2;
extern int LED2Closeflag_flash2;
//int init_LM3643();



static int FL_Enable_flash2(void)
{
    flashEnable_LM3643_2();
    pr_err("FL_Enable_flash2-");

    return 0;
}

static int FL_Disable_flash2(void)
{
    flashDisable_LM3643_2();

    return 0;
}

static int FL_dim_duty_flash2(kal_uint32 duty)
{
    setDuty_LM3643_2(duty);
    return 0;
}

/*
static int g_lowPowerLevel=LOW_BATTERY_LEVEL_0;
static void lowPowerCB(LOW_BATTERY_LEVEL lev)
{
	g_lowPowerLevel=lev;
}*/

static int FL_Init_flash2(void)
{
    pr_err(" FL_Init_flash2 line=%d\n",__LINE__);
    INIT_WORK(&workTimeOut, work_timeOutFunc);
  //  register_low_battery_callback(&lowPowerCB, LOW_BATTERY_PRIO_FLASHLIGHT);
    //register_low_battery_notify(&lowPowerCB, LOW_BATTERY_PRIO_FLASHLIGHT);



    return 0;
}
static int FL_Uninit_flash2(void)
{
	FlashIc_Disable();
    return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{

	//g_lowPowerLevel=LOW_BATTERY_LEVEL_0;
	return 0;

}


static int detLowPowerEnd(void)
{
	//if(g_lowPowerLevel!=LOW_BATTERY_LEVEL_0)
    //		return 1;
	//else
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;

static int g_b1stInit=1;

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable_flash2();
    pr_err("ledTimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static void timerInit(void)
{



	//mt6333_set_rg_chrwdt_en(0);

    //mt6333_set_rg_chrwdt_td(0); //4 sec
    //mt6333_set_rg_chrwdt_en(1);

    //mt6333_set_rg_chrwdt_en(0);

	if(g_b1stInit==1)
	{
		g_b1stInit=0;


	  	INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000; //1s
		hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		g_timeOutTimer.function=ledTimeOutCallback;
	}



}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	//pr_err("LM3643_LED2_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=0x%x\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			pr_err("FLASH_IOC_SET_TIME_OUT_TIME_MS: 0x%lx\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		pr_err("FLASHLIGHT_DUTY: 0x%lx\n",arg);
			m_duty2_flash2 = arg;
    		break;


    	case FLASH_IOC_SET_STEP:
    		pr_err("FLASH_IOC_SET_STEP: 0x%lx\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		pr_err("kdebug FLASHLIGHT_ONOFF: 0x%lx\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
				LED2Closeflag_flash2 = 0;
				FL_dim_duty_flash2(m_duty2_flash2);
    			FL_Enable_flash2();
    		}
    		else
    		{
    			//m_duty2_flash2 = -1;
    			LED2Closeflag_flash2 = 1;
				FL_dim_duty_flash2(m_duty2_flash2);
    			FL_Disable_flash2();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
/*			
    	case FLASH_IOC_PRE_ON:
    		pr_err("FLASH_IOC_PRE_ON\n");
			FL_preOn();
    		break;
    	case FLASH_IOC_GET_PRE_ON_TIME_MS:
    		pr_err("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
    		temp=13;
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                pr_err(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
*/
        case FLASH_IOC_SET_REG_ADR:
            pr_err("FLASH_IOC_SET_REG_ADR: 0x%lx\n",arg);
            //g_reg = arg;
            break;
        case FLASH_IOC_SET_REG_VAL:
            pr_err("FLASH_IOC_SET_REG_VAL: 0x%lx\n",arg);
            //g_val = arg;
            break;
        case FLASH_IOC_SET_REG:
          //  pr_err("FLASH_IOC_SET_REG: %d %d\n",g_reg, g_val);

            break;

        case FLASH_IOC_GET_REG:
            pr_err("FLASH_IOC_GET_REG: 0x%lx\n",arg);

            //i4RetValue = valTemp;

            break;

        case FLASH_IOC_HAS_LOW_POWER_DETECT:
    		pr_err("FLASH_IOC_HAS_LOW_POWER_DETECT");
    		temp=FL_hasLowPowerDetect();
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                pr_err(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_START:
    		detLowPowerStart();
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_END:
    		i4RetValue = detLowPowerEnd();
    		break;

        default :
    		pr_err(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    pr_err("kdebug flash2 led2 constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init_flash2();
		timerInit();
	}
	pr_err("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    pr_err("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    pr_err(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;


        spin_unlock_irq(&g_strobeSMPLock);
    	FL_Uninit_flash2();
    }
	pr_err("sub dummy release");
    return 0;

}


static FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 strobeInit_main_sid2_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
	return 0;
}
#endif
