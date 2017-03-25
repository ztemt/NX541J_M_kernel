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


#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
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
#include <linux/version.h>

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
#define TAG_NAME "[strobe_main_sid1_part2.c]"
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


static int strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
	return 0;
}

static int strobe_open(void *pArg)
{
	PK_DBG("sub dummy open");
	return 0;
}

static int strobe_release(void *pArg)
{
	PK_DBG("sub dummy release");
	return 0;
}

static FLASHLIGHT_FUNCTION_STRUCT strobeFunc = {
	strobe_open,
	strobe_release,
	strobe_ioctl
};

MUINT32 strobeInit_main_sid1_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &strobeFunc;
	return 0;
}

#else
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
#define TAG_NAME "[strobe_main_sid1_part2.c]"
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

#ifndef e_DutyNum 
#define e_DutyNum 16
#endif
static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x63


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13
//#define GPIO_LED_EN  GPIO114

//#define GPIO_LED_EN                    GPIO_CAMERA_FLASH_EN_PIN 



#define LM3643_REG_ENABLE      0x01
#define LM3643_REG_LED1_FLASH  0x03
#define LM3643_REG_LED2_FLASH  0x04
#define LM3643_REG_LED1_TORCH  0x05
#define LM3643_REG_LED2_TORCH  0x06
#define LM3643_REG_TIMING	   0x08		


/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

extern int KD2684_write_reg( u8 reg, u8 val);
extern int readReg(int reg);
extern int dual_flash_id;

int readReg_flash2(int reg)
{
	#if 0
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    pr_err("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
	#endif
	return readReg(reg);
}

int writeReg(int reg, int data)
{
	#if 0
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
    iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);
    #endif
	KD2684_write_reg(reg,data);
   return 0;
}

#define e_DutyNum1 26
#define e_DutyNum2 21
#define TORCHDUTYNUM 4
static int isMovieMode1[e_DutyNum1] = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int isMovieMode2[e_DutyNum2] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static int torchLED1Reg[e_DutyNum1] = {35,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchLED2Reg[e_DutyNum2] = {35,70,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//50,100,150,179ma
static int flashLED1Reg[e_DutyNum1] = {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110,118,127};
static int flashLED2Reg[e_DutyNum2] = {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84};



int m_duty1_flash2=0;
int m_duty2_flash2=0;
int LED1Closeflag_flash2 = 0;
int LED2Closeflag_flash2 = 0;


int flashEnable_LM3643_1(void)
{
//	int temp;
	return 0;
}
int flashDisable_LM3643_1(void)
{
	//int temp;
    return 0;
}

int setDuty_LM3643_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1_flash2=duty;
	
	return 0;
}



int flashEnable_LM3643_2(void)
{
	int temp;

	pr_err("flashEnable_LM3643_2\n");
	pr_err("LED1Closeflag_flash2 = %d, LED2Closeflag_flash2 = %d\n", LED1Closeflag_flash2, LED2Closeflag_flash2);
	
	temp = readReg_flash2(LM3643_REG_ENABLE);

	if((LED1Closeflag_flash2 == 1) && (LED2Closeflag_flash2 == 1))
	{
		writeReg(LM3643_REG_ENABLE, temp & 0xF0);//close		
	}
	else if(LED1Closeflag_flash2 == 1)
	{
		if(isMovieMode2[m_duty2_flash2] == 1)
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFA);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFE);//flash mode
	}
	else if(LED2Closeflag_flash2 == 1)
	{
		if(isMovieMode1[m_duty1_flash2] == 1)
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xF9);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFD);//flash mode		
	}
	else
	{
		if((isMovieMode1[m_duty1_flash2] == 1) & (isMovieMode2[m_duty2_flash2] == 1))
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFB);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFF);//flash mode
	}
	return 0;

}
int flashDisable_LM3643_2(void)
{
	flashEnable_LM3643_2();
	return 0;
}


int setDuty_LM3643_2(int duty)
{
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum2)
		duty=e_DutyNum2-1;
	m_duty2_flash2=duty;

	pr_err("setDuty_LM3643_2:m_duty = %d, m_duty2_flash2 = %d!\n", m_duty1_flash2, m_duty2_flash2);
	pr_err("LED1Closeflag_flash2 = %d, LED2Closeflag_flash2 = %d\n", LED1Closeflag_flash2, LED2Closeflag_flash2);

	if((LED1Closeflag_flash2 == 1) && (LED2Closeflag_flash2 == 1))
	{
		
	}
	else if(LED1Closeflag_flash2 == 1)
	{
		if(isMovieMode2[m_duty2_flash2] == 1)
		{
			writeReg(LM3643_REG_LED2_TORCH, torchLED2Reg[m_duty2_flash2]);
		}
		else
		{
	 	    writeReg(LM3643_REG_LED2_FLASH, flashLED2Reg[m_duty2_flash2]);
		}
	}
	else if(LED2Closeflag_flash2 == 1)
	{
		if(isMovieMode1[m_duty1_flash2] == 1)
		{
			writeReg(LM3643_REG_LED1_TORCH, torchLED1Reg[m_duty1_flash2]&0x7F);
		}
		else
		{
			writeReg(LM3643_REG_LED1_FLASH, flashLED1Reg[m_duty1_flash2]&0x7F);	
		}		
	}
	else
	{
		if((isMovieMode1[m_duty1_flash2] == 1) && ((isMovieMode2[m_duty2_flash2] == 1)))
		{
			writeReg(LM3643_REG_LED1_TORCH, torchLED1Reg[m_duty1_flash2]&0x7F);
			writeReg(LM3643_REG_LED2_TORCH, torchLED2Reg[m_duty2_flash2]);
		}
		else
		{
	 	    writeReg(LM3643_REG_LED1_FLASH, flashLED1Reg[m_duty1_flash2]&0x7F);
			writeReg(LM3643_REG_LED2_FLASH, flashLED2Reg[m_duty2_flash2]);
		}
	}

	return 0;
}


int FL_Enable_flash2(void)
{

	pr_err(" FL_Enable_flash2 line=%d\n",__LINE__);


    return 0;
}



int FL_Disable_flash2(void)
{
	pr_err(" FL_Disable_flash2 line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty_flash2(kal_uint32 duty)
{
    setDuty_LM3643_1(duty);

    pr_err(" FL_dim_duty_flash2 line=%d\n",__LINE__);
    return 0;
}




int FL_Init_flash2(void)
{
	pr_err("LED1_FL_Init_flash2!\n");
/*
    if(mt_set_gpio_mode(GPIO_LED_EN,GPIO_MODE_00)){pr_err(" set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_LED_EN,GPIO_DIR_OUT)){pr_err(" set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ONE)){pr_err(" set gpio failed!! \n");}
*/
     flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_HIGH); //tanyijun add 
	writeReg(LM3643_REG_TIMING, 0x1F);

	printk("kdebug flash2_LM3643 FL_Init_flash2 dual_flash_id is: 0x%x\n",dual_flash_id);

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    pr_err(" FL_Init_flash2 line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit_flash2(void)
{
	pr_err("LED1_FL_Uninit_flash2!\n");
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable_flash2();
    pr_err("LED1TimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;

static void timerInit(void)
{
  	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	pr_err("LM3643_LED1_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%ld\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			pr_err("FLASH_IOC_SET_TIME_OUT_TIME_MS: %ld\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		pr_err("FLASHLIGHT_DUTY: %ld\n",arg);
			m_duty1_flash2 = arg;
    		break;


    	case FLASH_IOC_SET_STEP:
    		pr_err("FLASH_IOC_SET_STEP: %ld\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		pr_err("FLASHLIGHT_ONOFF 3333 : %ld\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
				LED1Closeflag_flash2 = 0;
    			FL_Enable_flash2();
    		}
    		else
    		{
    			LED1Closeflag_flash2 = 1;
    			FL_Disable_flash2();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
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
    pr_err("kdebug flash2 led1 constant_flashlight_open line=%d\n", __LINE__);

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
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

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

MUINT32 strobeInit_main_sid1_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq_flash2(void)
{

    return 0;
}
EXPORT_SYMBOL(strobe_VDIrq_flash2);
#endif
