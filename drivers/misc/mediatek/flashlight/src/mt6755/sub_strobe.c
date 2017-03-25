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
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"

extern int zte_camera_bkl_level;
atomic_t camera_bklight_set;
extern int primary_display_setbacklight(unsigned int level);


#ifdef CONFIG_ZTEMT_SUB_FLASHLIGHT

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
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
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

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */
    #define CAMERA_FLASH_EN_PIN                    GPIO_CAMERA_FLASH_EN_PIN 
    #define CAMERA_FLASH_EN_PIN_M_GPIO     GPIO_CAMERA_FLASH_EN_PIN_M_GPIO

    #define CAMERA_FLASH_STROBE_PIN                    GPIO_CAMERA_FLASH_MODE_PIN 
    #define CAMERA_FLASH_STROBE_PIN_M_GPIO         GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO

    #define CAMERA_FLASH_TX_PIN                    GPIO_CAMERA_FLASH_TX_PIN 
    #define CAMERA_FLASH_TX_PIN_M_GPIO     GPIO_CAMERA_FLASH_TX_PIN_M_GPIO

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
#define GPIO_UNSUPPORTED 0xff
#define GPIO_SUPPORTED 0
#define GPIO_MODE_GPIO 0

#define GPIO_DIR_OUT 1
extern int KD2684_write_reg( u8 reg, u8 val);
extern int readReg(int reg);

int FL_Enable_sub(void)
{
	int buf[2];
	int regValue=0;	
#if 0
       buf[0] = 10;
	if (gIsTorch[gDuty] == 1)
		buf[1] = 0x71;
	else
		buf[1] = 0x77;
	KD2684_write_reg(KD2684_i2c_client, buf[0], buf[1]);
#endif
/*
      	if(mt_set_gpio_mode(CAMERA_FLASH_EN_PIN,CAMERA_FLASH_EN_PIN_M_GPIO))
		       {PK_DBG("[CAMERA FLASH] set gpio mode failed!! (FLASH_EN)\n");}
        if(mt_set_gpio_dir(CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT))
			{PK_DBG("[CAMERA FLASH] set gpio dir failed!! (FLASH_EN)\n");}
        if(mt_set_gpio_out(CAMERA_FLASH_EN_PIN,GPIO_OUT_ONE))
			{PK_DBG("[CAMERA FLASH] set gpio failed!! (FLASH_EN)\n");}
*/
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_HIGH);
          mdelay(2);
      // regValue = readReg(0x1);

	if(gDuty ==1)  //pre-flash
	{
		KD2684_write_reg(0x5, (readReg(0x5)&0x7F));
		//LED2 torch 100ma
		KD2684_write_reg(0x6, (0x10<<1)|(readReg(0x6)&0x81));
		buf[0] = 0x1;
		buf[1] = 0x7A;//0xAB;
	}
	else if(gDuty ==2){  //main-flash
		KD2684_write_reg(0x5, (readReg(0x5)&0x7F));
		//LED2 torch 150ma
		KD2684_write_reg(0x6, (0x18<<1)|(readReg(0x6)&0x81));
		buf[0] = 0x1;
		buf[1] = 0x7A;//0xAB;
		}else{
		
		buf[0] = 0x1;
		buf[1] = 0x7E;//0xAF;
	}

       KD2684_write_reg( buf[0], buf[1]);
	PK_DBG(" FL_Enable_sub line=%d\n", __LINE__);
	return 0;
}



int FL_Disable_sub(void)
{
	int buf[2];
	buf[0] = 0x1;
	buf[1] = 0xA3;
	KD2684_write_reg( buf[0], buf[1]);
	if(mt_set_gpio_out(CAMERA_FLASH_EN_PIN,GPIO_OUT_ZERO))
		{PK_DBG("[CAMERA FLASH] set gpio failed!! (FLASH_EN)\n");}
	/*PK_DBG(" FL_Disable_sub line=%d\n", __LINE__);*/
	return 0;
}

int FL_dim_duty_sub(kal_uint32 duty)
{
	int buf[2];
	if (duty > 17)
		duty = 17;
	if (duty < 0)
		duty = 0;
	gDuty = duty;
//	buf[0] = 9;
//	buf[1] = gLedDuty[duty];
//	KD2684_write_reg(KD2684_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_dim_duty_sub line=%d\n", __LINE__);
	return 0;
}

int FL_Init_sub(void)
{
	int buf[2];
/*
      	if(mt_set_gpio_mode(GPIO13,CAMERA_FLASH_EN_PIN_M_GPIO))
		       {PK_DBG("[CAMERA FLASH] set gpio mode failed!! (FLASH_EN)\n");}
        if(mt_set_gpio_dir(CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT))
			{PK_DBG("[CAMERA FLASH] set gpio dir failed!! (FLASH_EN)\n");}
        if(mt_set_gpio_out(CAMERA_FLASH_EN_PIN,GPIO_OUT_ONE))
			{PK_DBG("[CAMERA FLASH] set gpio failed!! (FLASH_EN)\n");}
*/

	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_HIGH);
        mdelay(2);
	
	buf[0] = 0X1;
	buf[1] = 0xA3;
	KD2684_write_reg( buf[0], buf[1]);
#if 0
     //LED1 flash current 902mA
	buf[0] = 0x3;
	buf[1] = 0x4c;
	KD2684_write_reg( buf[0], buf[1]);
#endif
     //LED2 flash current 750mA
	buf[0] = 0x4;
	buf[1] = 0x2F;
	KD2684_write_reg( buf[0], buf[1]);
#if 0
     //LED1 torch current 199mA
	buf[0] = 0x5;
	buf[1] = 0x43;
	KD2684_write_reg( buf[0], buf[1]);
#endif
     //LED2 torch current 205mA
	buf[0] = 0x6;
	buf[1] = 0x44;
	KD2684_write_reg( buf[0], buf[1]);



	PK_DBG(" FL_Init_sub line=%d\n", __LINE__);
	return 0;
}


int FL_Uninit_sub(void)
{
	FL_Disable_sub();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable_sub();
	PK_DBG("ledTimeOut_callback\n");
	/* printk(KERN_ALERT "work handler function./n"); */
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
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}
static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	/* PK_DBG
	    ("KD2684 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg); */
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty_sub(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;
				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable_sub();
		} else {
			FL_Disable_sub();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init_sub();
		timerInit();
	}
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/

	return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
	PK_DBG("sub dummy open");
	int i4RetValue = 0;
	//PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init_sub();
		timerInit();
	}
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/

	return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
	PK_DBG("sub dummy release");
	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit_sub();
	}

	PK_DBG(" Done\n");

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
/* //delete by kangxiong in case of no sub flash or LCD flash
static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
	return 0;
}

static int sub_strobe_open(void *pArg)
{
	PK_DBG("sub dummy open");
	return 0;

}

static int sub_strobe_release(void *pArg)
{
	PK_DBG("sub dummy release");
	return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}*/
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
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	PK_DBG("ledTimeOut_callback\n");
	atomic_set(&camera_bklight_set, 0);
	/* printk(KERN_ALERT "work handler function./n"); */
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
	g_timeOutTimeMs = 4000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	static int camera_lcm_bklight_on = 1;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	
	PK_DBG("sub dummy ioctl");
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	/* PK_DBG
	    ("KD2684 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg); */
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		gDuty = arg;
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d gDuty=%d\n", (int)arg, gDuty);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;
				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			atomic_set(&camera_bklight_set, 1);
			if (gDuty == 1) {
				camera_lcm_bklight_on = 1;
				primary_display_setbacklight(255);
			} else {				
				primary_display_setbacklight(255);
				camera_lcm_bklight_on = 0;
			}
		} else {
			hrtimer_cancel(&g_timeOutTimer);
			if (camera_lcm_bklight_on == 0) {
				camera_lcm_bklight_on = 1;
				primary_display_setbacklight(zte_camera_bkl_level);
				atomic_set(&camera_bklight_set, 0);
			}
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}



static int sub_strobe_open(void *pArg)
{
	int i4RetValue = 0;
	//PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	PK_DBG("sub dummy open");

	if (0 == strobe_Res) {
		timerInit();
	}
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/

	return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
	PK_DBG("sub dummy release");
	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);
		atomic_set(&camera_bklight_set, 0);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

	}

	PK_DBG(" Done\n");
	return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}

#endif

//ZTEMT: added by congshan start
static ssize_t brightness_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)

{

	return sprintf(buf, "%u\n", zte_camera_bkl_level);
}
static int bkl_recov = 1;

static ssize_t brightness_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)

{
	uint32_t val = 0;
	sscanf(buf, "%d", &val);
	if((0 != bkl_recov) && (0 != val)) {
		atomic_set(&camera_bklight_set, 1);
		primary_display_setbacklight(val);
	}
	return size;
}
static ssize_t bkl_recov_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)

{

	return sprintf(buf, "%u\n", zte_camera_bkl_level);
}

static ssize_t bkl_recov_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)

{
	uint32_t val = 0;
	sscanf(buf, "%d", &val);
	pr_err("%s\n", __func__);
	atomic_set(&camera_bklight_set, 0);
	bkl_recov = val;
	//zte_camera_lcd_bkl(zte_camera_lcm_handle, NULL, 25);
	return size;
}

static struct kobj_attribute kobj_attr_flash_bkl[] = {
	__ATTR(brightness,      0664, brightness_show,     brightness_store),
	__ATTR(bkl_recov,       0664, bkl_recov_show,      bkl_recov_store),
};

static struct kobject *flash_bkl_kobj = NULL;

static int __init flash_bkl_init(void)
{
	int retval = 0;
	int attr_count = 0;
    flash_bkl_kobj = kobject_create_and_add("flash_bkl", kernel_kobj);
    
    if (!flash_bkl_kobj) {
        pr_err("failed to create and add flash_bkl\n");
        return -ENOMEM;
    }
	for (attr_count = 0; attr_count < ARRAY_SIZE(kobj_attr_flash_bkl); attr_count++) {
	    retval = sysfs_create_file(flash_bkl_kobj, &kobj_attr_flash_bkl[attr_count].attr);
	    if (retval < 0) {
		    pr_err("failed to create flash_bkl sysfs attributes\n");
		    goto err_sys_creat;
	    }
	}
	return retval;
	err_sys_creat:
	for (; attr_count >= 0; attr_count--)
	    sysfs_remove_file(flash_bkl_kobj, &kobj_attr_flash_bkl[attr_count].attr);
	kobject_put(flash_bkl_kobj);
	return retval;

}

module_init(flash_bkl_init);

//ZTEMT: added by congshan end

