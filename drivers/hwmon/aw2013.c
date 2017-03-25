/***********************************************************************************/
/* File Name: aw2013.c */
/* File Description: this file is used to make aw2013 driver to be added in kernel or module. */

/*  Copyright (c) 2002-2015, ZTEMT, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: ZTEMT, Inc.,            */
/***********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
//#include <linux/delay.h>// xiaofeng add for debug


#include  <../../include/linux/printk.h>
#include <linux/ctype.h>

#include <linux/regulator/consumer.h>
#include "aw2013.h"
//#include <cust_gpio_usage.h>//xiaofengadd 
//#include "mt_pm_ldo.h"   //xiaofeng add for hwPowerOn and its parameter

#include "upmu_common.h"//alexander add
//#include "pmic.h"
#include "upmu_hw.h"
//#include <../misc/mediatek/include/mt-plat/mt6755/include/mach/upmu_hw.h>


static bool aw2013_SUSPEND_FLAG=false; 
//static int blink_mode = 0;

extern unsigned short pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);

#define DRV_NAME "class/leds/nubia_led/outn"

#define AW2013_I2C_BUSNUM 1 //xiaofeng

#define xf_ext_flag //xiaofeng add

//#define PMIC_CHRIND_EN  1712

//#define xf_breath_all_test

#ifdef xf_breath_all_test
//LED breath
#define Imax          0x02   //LED Imax,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x02   //LED rise time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x01   //LED max light time light 0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x02   //LED fall time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x01   //LED off time ,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time   0x00   //LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15

/*
static ssize_t aw2013_store_led(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t aw2013_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw2013_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DEVICE_ATTR(led, S_IRUGO | S_IWUSR, NULL, aw2013_store_led);
static DEVICE_ATTR(reg, S_IRUGO | S_IWUGO,aw2013_get_reg,  aw2013_set_reg);
*/



//*******************************AW2013 LED Program***********************************///
void aw2013_breath_all(int led0,int led1,int led2)  //led on=0x01   ledoff=0x00
{  

	//write_reg(0x00, 0x55);				// Reset 
	AW2013_i2c_write_reg(0x01, 0x01);		// enable LED 		

	AW2013_i2c_write_reg(0x31, Imax|0x70);	//config mode, IMAX = 5mA	
	AW2013_i2c_write_reg(0x32, Imax|0x70);	//config mode, IMAX = 5mA	
	AW2013_i2c_write_reg(0x33, Imax|0x70);	//config mode, IMAX = 5mA	

	AW2013_i2c_write_reg(0x34, 0xff);	// LED0 level,
	AW2013_i2c_write_reg(0x35, 0xff);	// LED1 level,
	AW2013_i2c_write_reg(0x36, 0xff);	// LED2 level,
											
	AW2013_i2c_write_reg(0x37, Rise_time<<4 | Hold_time);	  //led0  				
	AW2013_i2c_write_reg(0x38, Fall_time<<4 | Off_time);	  //led0 
	AW2013_i2c_write_reg(0x39, Delay_time<<4| Period_Num);  //led0 

	AW2013_i2c_write_reg(0x3a, Rise_time<<4 | Hold_time);	  //led1						
	AW2013_i2c_write_reg(0x3b, Fall_time<<4 | Off_time);	  //led1 
	AW2013_i2c_write_reg(0x3c, Delay_time<<4| Period_Num);  //led1  

	AW2013_i2c_write_reg(0x3d, Rise_time<<4 | Hold_time);	  //led2 			
	AW2013_i2c_write_reg(0x3e, Fall_time<<4 | Off_time);    //led2 
	AW2013_i2c_write_reg(0x3f, Delay_time<<4| Period_Num);  //

	AW2013_i2c_write_reg(0x30, led2<<2|led1<<1|led0);	      //led on=0x01 ledoff=0x00	
	AW2013_delay_1us(8);//Delay >5us
}

#endif


enum aw_outn_mode{
	AW_SW_RESET,	    // 0  soft_reset , all regs revert to default value.
	AW_CONST_ON,	    // 1 work on a constant lightness.
	AW_CONST_OFF,	    // 2 darkness is comming
	AW_AUTO_BREATH, 	// 3 self breathing, used in sences such as missing message.
	AW_STEP_FADE_IN,	// 4  fade in means that the lightness is getting stronger.
	AW_STEP_FADE_OUT,	// 5  fade out means that the lightness is getting weaker
	AW_BREATH_ONCE,     // 6 only breath once, touch the home menu for instance.
	AW_RESERVED,		// 7 reserverd.
};

#define GRADE_PARAM_LEN 20
#define CONST_MIN_GRADE  10
#define CONST_MAX_GRADE  200
#define FADE_PARAM_LEN 20

#define AW2013_I2C_MAX_LOOP 		50

#define AW2013_DEVICE_NAME "aw2013"

static struct i2c_client *drv_client;//1231

extern int led_classdev_register(struct device *parent, struct led_classdev *led_cdev);
//extern int led_classdev_unregister(struct device *parent, struct led_classdev *led_cdev);//at last ,comment this due to conflict with new added leds.h about statement for this interface


static struct i2c_board_info __initdata aw2013_i2c_info = { I2C_BOARD_INFO(AW2013_DEVICE_NAME, 0x45) };//xiaofeng//accord to zhengcaiyin, addr turn to 0x45

/*
#define Imax          0x02   //LED Imax,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x05   	 //t1, LED rise time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x05   	 //t2, LED max light time light 0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x05   //t3, LED fall time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x05   //t4, LED off time ,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
*/

#define Delay_time   0x00    //t0, LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00     //rpt,LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15

//static int aw2013_irq_gpio = -1; //in 535 ,this pin is connect VIO18_PMU

static struct aw2013_regs_data aw2013_regs =
{
	.soft_reset = 0x00,
	.enable_led = 0x01,
	.irq_state = 0x20,
	.enable_ledx = 0x30,

	.led0_mode_conf = 0x31,
	.led1_mode_conf = 0x32,
	.led2_mode_conf = 0x33,

	.pwm0_max_lightness = 0x34,
	.pwm1_max_lightness = 0x35,
	.pwm2_max_lightness = 0x36,

	.led0_rise_and_hold_time = 0x37,
	.led0_fall_and_off_time = 0x38,
	.led0_delay_and_repeat_time = 0x39,

	.led1_rise_and_hold_time = 0x3A,
	.led1_fall_and_off_time = 0x3B,
	.led1_delay_and_repeat_time = 0x3C,

	.led2_rise_and_hold_time = 0x3D,
	.led2_fall_and_off_time = 0x3E,
	.led2_delay_and_repeat_time = 0x3F,
};

// for debug issue.
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define AW_DBG(x...) do {if (debug_mask) pr_info("aw2013  " x); } while (0)

#if 1 // for debug
#define LED_DEBUG(fmt,args...)  printk(KERN_ERR"ZTEMT:%s,%d" fmt,__func__,__LINE__,##args)
#else
#define LED_DEBUG(fmt,args...)  do {} while (0)
#endif






// basic funtion---i2c function
static int aw2013_i2c_rx_byte_data(
		struct i2c_client *i2c,
		unsigned char  reg,
		unsigned char* buf)
{

	struct i2c_msg msgs[2];
	//int i=0;//xiaofeng add for debug
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;
#ifdef xf_ext_flag
	msgs[0].ext_flag =0x0000;
	msgs[0].timing = 100;
	msgs[1].ext_flag =0x0000;
	msgs[1].timing = 100;
#endif
	
	//while(1)//when debug aw2013 driver, i find shoud use do while instead of while loop
	//mdelay(3000);
	//printk(KERN_ERR"xiaofeng:aw2013,i2c_client.addr:%d,i2c_client.name:%s,in %s\n",i2c->addr,i2c->name,__func__);
#if 0
	do
	{
		if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
			dev_err(&i2c->dev, "%s: transfer failed->\n", __func__);
			//return -4;//xiaofeng comment
			mdelay(10);//xiaofeng add
		}
		else
		{
			break;
		}

		if(i++ > 10)
			break;
	}while(1);
#endif
	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed->\n", __func__);
		return -4;
	}
	pr_debug("return  buf[0]=0x%x!\n",buf[0]);

	return 0;
}

static int aw2013_i2c_tx_byte_data(
		struct i2c_client *i2c,
		unsigned char reg, 
		unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];

	bufwr[0] = reg;
	bufwr[1] = buf;

	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;
#ifdef xf_ext_flag
	msgs.ext_flag =0x0000;
	msgs.timing = 100;
#endif
	//pr_err("aw2013 : write reg[0x%x] , data[%d]!\n",reg,buf);
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed->\n", __func__);
		return -4;
	}
	
	return 0;
}


unsigned char aw2013_read_reg(struct aw2013_control_data *led,unsigned char regaddr) 
{
	unsigned char rdbuf[1], wrbuf[1], ret;

	wrbuf[0] = regaddr;

	ret = aw2013_i2c_rx_byte_data(led->i2c_client,regaddr,rdbuf); // then read the content at the regaddr
			
	//printk("at addr %x value = %x \n",regaddr,rdbuf);
	
	if (ret < 0)
		printk("**********************driver aw2013: 5555   ning aw2013_read_reg failed  %s \r\n", __func__);
    return rdbuf[0];
}


static int write_reg(struct aw2013_control_data *led,unsigned char reg,unsigned char data)
{

	int ret;
	unsigned char i;

	for (i=0; i<AW2013_I2C_MAX_LOOP; i++)
	{
		ret = aw2013_i2c_tx_byte_data(led->i2c_client,reg,data);
		//pr_err("aw2013 : write reg[0x%x] , data[%d]!\n",reg,data);
		if (ret >= 0) // ack success
			break;
		}
	return ret;
}

#if 0
void aw2013_debug(struct aw2013_control_data * led)
{	
	unsigned int addr, buf;

	for(addr = aw2013_regs.enable_ledx; addr < 0x40; addr ++)
		{
		buf = aw2013_read_reg(led,addr);
		printk("at addr %x value is %x\n",addr, buf);
	}
}
#endif

static int aw2013_soft_reset( struct aw2013_control_data *led)
{
	char buf;
	int ret;
	
	buf = 0x55;
	ret = aw2013_i2c_tx_byte_data(led->i2c_client,aw2013_regs.soft_reset,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw2013_regs.soft_reset);
	write_reg(led,aw2013_regs.enable_led,0x01);
	return ret;
}


void led_const_on(struct aw2013_control_data *led)
{
	unsigned char buf;

	write_reg(led,aw2013_regs.led0_mode_conf + led->outn,0x02);

	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);

	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);

	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));

	msleep(6);

}



void led_const_off(struct aw2013_control_data *led)
{
	int buf;

	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx,buf & (~(0x01 << led->outn))); // outn 0 --0xFE , 1--0xFD, 2--0xFB

	msleep(1);
}

#if 0
void led_off_right_after_put_out_screen(truct aw2013_control_data *led)
{
	write_reg(led,aw2013_regs.enable_ledx,0x00);
}
#endif

void led_auto_breath(struct aw2013_control_data *led) // self breath 0 - 255 version.
{

	   //unsigned char buf;

		// 3. set ledx as auto mode and set it's max current.as the Imax is a unchanged value, no need to set again.
		//buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + outn);
		write_reg(led,aw2013_regs.led0_mode_conf + led->outn, 0x72); //FO=FI=MD=1, Imax=10

		write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->max_grade);

		// 4. set auto mode's time
		/*
		write_reg(aw2013_regs.led0_rise_and_hold_time + outn*3, (Rise_time<<4) + Hold_time);
		write_reg(aw2013_regs.led0_fall_and_off_time + outn*3, (Fall_time<<4) + Off_time);
		write_reg(aw2013_regs.led0_delay_and_repeat_time+ outn*3, (Delay_time<<4) + Period_Num);		
		*/
		//  rise is equal to fall  rise , there are 3 time. rise_fall, hold, off
		//printk("alexander: in funtion %s at line %d, led0_rise_and_hold_time=%x,led0_fall_and_off_time=%x,led0_delay_and_repeat_time=%x\n", __func__,__LINE__,(led->Rise_Fall_time << 4) + led->Hold_time,(led->Rise_Fall_time << 4) + led->Off_time,(Delay_time << 4) + Period_Num);
		//printk("alexander:then reset them\n");
		//write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Hold_time);//xiaofeng mask

		//write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time);//xiaofeng mask

		write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, 0x41);//xiaofeng mod to 4 1 1

		write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, 0x44);//xiaofeng mod to 4 1 1

		write_reg(led,aw2013_regs.led0_delay_and_repeat_time + led->outn*3, (Delay_time << 4) + Period_Num);

		
		// 5. enable ledx
       // aw2013_read_reg(led,aw2013_regs.enable_ledx);
       
		write_reg(led,aw2013_regs.enable_ledx, 0x01 << led->outn);
		
		//printk("the enable_ledx's value after auto set is %x\n",aw2013_read_reg(led,aw2013_regs.enable_ledx));
		//printk("the outn is %x\n",outn);

		//printk("zqf debug : in funtion %s at line %d\t", __func__,__LINE__);
		msleep(1);		
		//aw2013_debug(led);
}

#if 0
void led_auto_breath_once(struct aw2013_control_data *led) // self breath 0 - 255 version.
{

	   //unsigned char buf;

	    int nu;
	   
		// 3. set ledx as auto mode and set it's max current.as the Imax is a unchanged value, no need to set again.
		//buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + outn);

		// 4. set auto mode's time
		/*
		write_reg(aw2013_regs.led0_rise_and_hold_time + outn*3, (Rise_time<<4) + Hold_time);
		write_reg(aw2013_regs.led0_fall_and_off_time + outn*3, (Fall_time<<4) + Off_time);
		write_reg(aw2013_regs.led0_delay_and_repeat_time+ outn*3, (Delay_time<<4) + Period_Num);		
		*/
		//  rise is equal to fall  rise , there are 3 time. rise_fall, hold, off
	
		for (nu = 0; nu <3; nu++)
		{
			write_reg(led,aw2013_regs.led0_mode_conf + nu, 0x72); //FO=FI=MD=1, Imax=10
			write_reg(led,aw2013_regs.pwm0_max_lightness + nu, led->max_grade);
			write_reg(led,aw2013_regs.led0_rise_and_hold_time + nu*3, (led->Rise_Fall_time << 4) + led->Hold_time);
			write_reg(led,aw2013_regs.led0_fall_and_off_time + nu*3, (led->Rise_Fall_time << 4) + led->Off_time);
			write_reg(led,aw2013_regs.led0_delay_and_repeat_time + nu*3, (Delay_time << 4) + Period_Num);
		}
		
		// 5. enable ledx
       // aw2013_read_reg(led,aw2013_regs.enable_ledx);
       
		write_reg(led,aw2013_regs.enable_ledx, 0x07);
		
		//printk("the enable_ledx's value after auto set is %x\n",aw2013_read_reg(led,aw2013_regs.enable_ledx));
		//printk("the outn is %x\n",outn);

	//	printk("zqf debug : in funtion %s at line %d\t", __func__,__LINE__);
		msleep(1);

		
		//aw2013_debug(led);

}
#endif


void led_step_fade_in(struct aw2013_control_data *led)
{
		int buf;

		led_const_on(led);

	    //write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->min_grade);

		
		buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
		write_reg(led,aw2013_regs.led0_mode_conf + led->outn, (buf & 0xEF) | 0x20); // set pwm mode and fade in mode
		
	    write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time <<4) + led->Hold_time);
		
		buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
		write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));

	    write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->max_grade);

		//printk("zqf debug : in funtion %s at line %d\t", __func__,__LINE__);

		//aw2013_debug(led);

		msleep(1);
		led_const_on(led);

		write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->min_grade);
}



void led_step_fade_out(struct aw2013_control_data *led)
{
	int buf;

    //	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->max_grade);

	buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
	write_reg(led,aw2013_regs.led0_mode_conf+led->outn, buf | 0x42); // set pwm mode and fade out mode

	//write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn, led->Rise_Fall_time << 4 );
	write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time);

	
	//enable ledx
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));

	// set lightness
	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);

	//led_const_on(led);
	//printk("zqf debug : in funtion %s at line %d\t", __func__,__LINE__);
	//aw2013_debug(led);
	msleep(1);				
}


#if 0
void led_breath_once(struct aw2013_control_data *led)
{

    unsigned char buf;

	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));

	write_reg(led,aw2013_regs.led0_mode_conf + led->outn,0x62); // fi fo md imax : 0110 0010
	write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) | led->Off_time);
	write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time <<4) | led->Hold_time);
	write_reg(led,aw2013_regs.led0_delay_and_repeat_time + led->outn*3, 0x00);

	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);

	msleep(10);

	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->max_grade);

	//msleep((0x0d << led->Rise_Fall_time)*10 + 100);

	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);

	//printk("zqf debug --kernel  funtion %s  at line %d", __func__,__line__);
   // aw2013_debug(led);

	//msleep((0x0d << led->Rise_Fall_time)*10 + 100);
}
#endif

void led_breath_once(struct aw2013_control_data *led)
{

    write_reg(led,aw2013_regs.enable_ledx,0x07);  //enable all leds
	
	write_reg(led,aw2013_regs.led0_mode_conf,0x62); // fi fo md imax : 0110 0010
	//write_reg(led,aw2013_regs.led1_mode_conf,0x62); // fi fo md imax : 0110 0010
	//write_reg(led,aw2013_regs.led2_mode_conf,0x62); // fi fo md imax : 0110 0010


	 write_reg(led,aw2013_regs.led0_fall_and_off_time , (led->Rise_Fall_time << 4) | led->Off_time);
	 write_reg(led,aw2013_regs.led0_rise_and_hold_time, (led->Rise_Fall_time <<4) | led->Hold_time);
	 write_reg(led,aw2013_regs.led0_delay_and_repeat_time, 0x00);

	//write_reg(led,aw2013_regs.led1_fall_and_off_time , (led->Rise_Fall_time << 4) | led->Off_time);
	//write_reg(led,aw2013_regs.led1_rise_and_hold_time, (led->Rise_Fall_time <<4) | led->Hold_time);
	//write_reg(led,aw2013_regs.led1_delay_and_repeat_time, 0x00);

	//write_reg(led,aw2013_regs.led2_fall_and_off_time , (led->Rise_Fall_time << 4) | led->Off_time);
	//write_reg(led,aw2013_regs.led2_rise_and_hold_time, (led->Rise_Fall_time <<4) | led->Hold_time);
	//write_reg(led,aw2013_regs.led2_delay_and_repeat_time, 0x00);

	write_reg(led,aw2013_regs.pwm0_max_lightness, led->max_grade);
	//write_reg(led,aw2013_regs.pwm1_max_lightness, led->max_grade);
	//write_reg(led,aw2013_regs.pwm2_max_lightness, led->max_grade);
	msleep((0x0d << led->Rise_Fall_time)*10 + 100); 

       write_reg(led,aw2013_regs.pwm0_max_lightness, led->min_grade);
	//write_reg(led,aw2013_regs.pwm1_max_lightness, led->min_grade);
	//write_reg(led,aw2013_regs.pwm2_max_lightness, led->min_grade);
	msleep((0x0d << led->Rise_Fall_time)*10 + 100);

#if 0
	for (channel = Start_outn; channel <= End_outn; channel ++)
	{
		write_reg(led,aw2013_regs.led0_mode_conf + channel,0x62); // fi fo md imax : 0110 0010
	    write_reg(led,aw2013_regs.led0_fall_and_off_time + channel*3, (led->Rise_Fall_time << 4) | led->Off_time);
     	write_reg(led,aw2013_regs.led0_rise_and_hold_time + channel*3, (led->Rise_Fall_time <<4) | led->Hold_time);
        write_reg(led,aw2013_regs.led0_delay_and_repeat_time + channel*3, 0x00);
		write_reg(led,aw2013_regs.pwm0_max_lightness + channel, led->min_grade);
		msleep(10);
    	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->max_grade);
		msleep((0x0d << led->Rise_Fall_time)*10 + 100);
		write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);
		msleep((0x0d << led->Rise_Fall_time)*10 + 100);	
	}
#endif

#if 0
	//msleep((0x0d << led->Rise_Fall_time)*10 + 100);

	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);

	//printk("zqf debug --kernel  funtion %s  at line %d", __func__,__line__);
   // aw2013_debug(led);

	//msleep((0x0d << led->Rise_Fall_time)*10 + 100);
#endif
}



#if 0
void all_leds_breath_once_(struct aw2013_control_data *led)
{

    unsigned char buf;
	int channel;
	
	write_reg(led,aw2013_regs.enable_ledx,0x07);

	for(channel=Start_outn; channel <=End_outn; channel ++)
	{
		write_reg(led,aw2013_regs.led0_mode_conf + channel,0x62); // fi fo md imax : 0110 0010
		write_reg(led,aw2013_regs.led0_fall_and_off_time + channel*3, (led->Rise_Fall_time << 4) | led->Off_time);
		write_reg(led,aw2013_regs.led0_rise_and_hold_time + channel*3, (led->Rise_Fall_time <<4) | led->Hold_time);
		write_reg(led,aw2013_regs.led0_delay_and_repeat_time + channel*3, 0x00);
		write_reg(led,aw2013_regs.pwm0_max_lightness + channel, led->min_grade);
		msleep(10);
		write_reg(led,aw2013_regs.pwm0_max_lightness + channel, led->max_grade);
		msleep((0x0d << led->Rise_Fall_time)*10 + 100);
		write_reg(led,aw2013_regs.pwm0_max_lightness + channel, led->min_grade);
		#if 0
		printk("zqf debug --kernel  funtion %s  at line %d", __func__,__line__);
	    aw2013_debug(led);
        #endif
		msleep((0x0d << led->Rise_Fall_time)*10 + 100);
   }
}
#endif

//static enum led_brightness aw2013_breath_mode_get(struct led_classdev *led_cdev)
static ssize_t aw2013_blink_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return sprintf(buf, "%d\n", led->blink_mode);
}


//static void aw2013_breath_mode_set(struct led_classdev *led_cdev,enum aw_outn_mode brightness)
//static void aw2013_breath_mode_set(struct led_classdev *led_cdev,enum led_brightness brightness)
static ssize_t  aw2013_blink_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{

	//int val = brightness;
	//int rc = 0;
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);
	sscanf(buf, "%d", &led->blink_mode);
	
    LED_DEBUG("led->blink_mode = %d\n", led->blink_mode);
    
	switch (led->blink_mode) {
		case AW_SW_RESET:
			aw2013_soft_reset(led);
			break;

		case AW_CONST_ON: 
			led_const_on(led);
			break;

		case AW_CONST_OFF:
			led_const_off(led);
			break;

		case AW_AUTO_BREATH:
			led_auto_breath(led);
			break;

		case AW_STEP_FADE_IN:
		    led_step_fade_in(led);
			break;

		case AW_STEP_FADE_OUT:
			led_step_fade_out(led);
			break;

		case AW_BREATH_ONCE:
		   led_breath_once(led);		 
			break;

		case AW_RESERVED:
			break;

		default:
			break;
	}
	return count;
}

static ssize_t fade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct aw2013_control_data *led;
	char *after, *parm2,*parm3;

	
	unsigned long delay_off,delay_off_1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long delay_on = simple_strtoul(buf, &after, 10);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	delay_off = simple_strtoul(parm2, &after, 10);

	while(isspace(*after))
		after++;
	parm3 = after;
	delay_off_1 = simple_strtoul(parm3, &after, 10);
	led->Rise_Fall_time = (int)delay_on;
	led->Hold_time = (int)delay_off;
	led->Off_time = (int)delay_off_1; 
	LED_DEBUG("fade_time=%d ,on_time=%d , off_time=%d\n",
		led->Rise_Fall_time,led->Hold_time,led->Off_time);
	return count;
}

static ssize_t fade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return snprintf(buf, FADE_PARAM_LEN, "%4d %4d %4d\n",
			led->Rise_Fall_time, led->Hold_time, led->Off_time);
}
	
static ssize_t grade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{

	struct aw2013_control_data *led;
	char *after, *parm2;
	unsigned long parameter_two;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long parameter_one = simple_strtoul(buf, &after, 10);

	
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	parameter_two = simple_strtoul(parm2, &after, 10);

	led->min_grade=(int)parameter_one;

	
	led->max_grade=(int)parameter_two;
	//printk("alexander:grade_parameter is:min_grade=%d,max_grade=%d\n",led->min_grade,led->max_grade);	
	LED_DEBUG("min_grade=%d , max_grade=%d\n",
		led->min_grade,led->max_grade);
	return count;
}
	
static ssize_t grade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return snprintf(buf, GRADE_PARAM_LEN,	"%4d %4d\n",
			led->min_grade,led->max_grade);
}

static ssize_t outn_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct aw2013_control_data *led;
	char *after;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	
	unsigned long parameter_one = simple_strtoul(buf, &after, 10);
	
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

        LED_DEBUG("parameter_one=%d \n",parameter_one);
#ifdef CONFIG_ZTEMT_LIQUID_LED_OUTN_SWITCH
#else
	parameter_one = (parameter_one >> 4) & 0x0f;
	if(parameter_one == 0x01)
		parameter_one = 0x00;
	else if(parameter_one == 0x00)
		parameter_one = 0x01;
#endif

	led->outn =(int) parameter_one;
	LED_DEBUG("ztemt_channel=%d \n",led->outn);
	return count;
}

static ssize_t outn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return sprintf(buf, "%d\n",led->outn);	
}

static DEVICE_ATTR(fade_parameter, 0664, fade_parameter_show, fade_parameter_store);
static DEVICE_ATTR(grade_parameter, 0664, grade_parameter_show, grade_parameter_store);
static DEVICE_ATTR(outn, 0664, outn_show, outn_store);
static DEVICE_ATTR(blink_mode, 0664, aw2013_blink_mode_get, aw2013_blink_mode_set);


static struct attribute *aw2013_attrs[] = {
	&dev_attr_fade_parameter.attr,
	&dev_attr_grade_parameter.attr,
	&dev_attr_outn.attr,
	&dev_attr_blink_mode.attr,
	NULL
};

static const struct attribute_group aw2013_attr_group = {
	.attrs = aw2013_attrs,
};

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int aw2013_power_on(struct device *dev)
{
#if 0
	int rc;
    	static struct regulator *vcc_ana;
    	static struct regulator *vcc_i2c;
		
	pr_info("\n --- aw2013_power_on ---\n");
	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 2850000, 3300000);
		if (rc)
        {
			dev_err(dev, "Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
  
    rc = reg_set_optimum_mode_check(vcc_ana, 15000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_ana set_opt failed rc=%d\n", rc);
        return rc;
    }
    
    rc = regulator_enable(vcc_ana);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_ana enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_ana;
    }

	vcc_i2c = regulator_get(dev, "vcc_i2c");


	if (IS_ERR(vcc_i2c))
    {
		rc = PTR_ERR(vcc_i2c);
		dev_err(dev, "Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}
   
	if (regulator_count_voltages(vcc_i2c) > 0)
    {
 		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000);
		if (rc)
        {
			dev_err(dev, "Regulator set i2c vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}
  
    rc = reg_set_optimum_mode_check(vcc_i2c, 10000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_i2c set_opt failed rc=%d\n", rc);
        goto error_set_vtg_i2c;
    }

    rc = regulator_enable(vcc_i2c);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_i2c;
    }

    msleep(100);
    
    return 0;


error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
error_set_vtg_i2c:
    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;
#endif
	//printk(KERN_ERR"xiaofeng:into %s\n",__func__);

	hwPowerOn(MT6351_POWER_LDO_VIO28, VOL_2800 * 1000, "VIO28_PMU");
	//printk(KERN_ERR"xiaofeng:after VIO28_PMU! %s\n",__func__);

	hwPowerOn(MT6351_POWER_LDO_VIO18, VOL_1800 * 1000, "VIO18_PMU");
	//printk(KERN_ERR"xiaofeng:after VIO18_PMU! %s\n",__func__);

}

static int  aw2013_probe(struct i2c_client *client,

		const struct i2c_device_id *dev_id)

{
	int ret = 0;
	char buf = 0x0;
	
	struct aw2013_control_data *aw2013_data;

	pmic_set_register_value(MT6351_PMIC_CHRIND_EN_SEL,1);
	msleep(10);
	printk(KERN_ERR"alexander:set PMIC_CHRIND_EN to 0\n");
	pmic_set_register_value(PMIC_CHRIND_EN,0);//alexander add
	
	printk(KERN_ERR"%s: xiaofeng: start probe:\n",__func__);
	
	aw2013_data = devm_kzalloc(&client->dev,
	sizeof(struct aw2013_control_data) , GFP_KERNEL);
	if (!aw2013_data) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		   return -ENOMEM;
	}

	ret = aw2013_power_on(&client->dev);
	if (ret < 0) {
	dev_err(&client->dev, "aw2013_power_on failed");
	return -EINVAL;
	}
	
   	aw2013_data->i2c_client = client;
       i2c_set_clientdata(client,aw2013_data);

	//aw2013_data->cdev.brightness_set =  aw2013_breath_mode_set;
	//aw2013_data->cdev.brightness_get = aw2013_breath_mode_get;

	aw2013_data->cdev.name = "nubia_led";

	
	ret = aw2013_i2c_rx_byte_data(aw2013_data->i2c_client,aw2013_regs.soft_reset,&buf);
	if(ret < 0)	{
		printk(KERN_ERR"alexander:aw2013_i2c_rx_byte_data  read chip id fail!!!\n");	
		return ret;	
		}	
	if(0x33 != buf){			
		printk(KERN_ERR"alexander the chip id is not 0x33!\n");		
		return -1;					
		}
	else			
		printk(KERN_ERR"alexander the chip id  is 0x33\n");	
	/*
	aw2013_irq_gpio = of_get_named_gpio(client->dev.of_node,
		"aw2013-irq-gpio", 0);
	pr_err("nubia_debug:%s aw2013-irq-gpio=%d\n",__func__, aw2013_irq_gpio);
	if(aw2013_irq_gpio >=0)	{
		gpio_request(aw2013_irq_gpio,"aw2013-irq-gpio");
		gpio_direction_output(aw2013_irq_gpio,1); // output, default is high.
    }
    */

    aw2013_soft_reset(aw2013_data);
	msleep(20);

	buf = 0x02; // this is the reg address for aw2013's isr reg.
	ret = aw2013_read_reg(aw2013_data,buf);
	if(ret < 0) {
	printk("aw2013_debug: can not get the isr states\n");
	}

    msleep(20);
    write_reg(aw2013_data, 0x01, 0x01);
    printk(KERN_ERR"%s: xiaofeng led_classdev_register\n",__func__);
	ret = led_classdev_register(&client->dev, &aw2013_data->cdev);
	if (ret) {
		pr_err("unable to register breath_led ret=%d\n",ret);
		goto init_fail;
	}

	ret = sysfs_create_group(&aw2013_data->cdev.dev->kobj,
			&aw2013_attr_group);
	if (ret)
		 goto init_fail;

	printk(KERN_ERR"%s: xiaofeng.finish probe:\n",__func__);
	return 0;

init_fail:
	return ret;

}

static int aw2013_remove(struct i2c_client *client)
{	

	struct aw2013_control_data *aw2013_data = i2c_get_clientdata(client);

	led_classdev_unregister(&aw2013_data->cdev);//add para parent->del para parent due to conflict with leds.h

	sysfs_remove_group(&aw2013_data->cdev.dev->kobj, &aw2013_attr_group);
	
	return 0;
}


static int aw2013_suspend(struct i2c_client *cl, pm_message_t mesg)
{
/*
    struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);

    aw2013_SUSPEND_FLAG=true;
    printk("gsc debug:aw2013 enter sleep!\n");
    write_reg(aw2013_data, 0x01, 0x00);//add by gsc  for aw2013 suspend setting GCR Register 0
*/
    struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);
    if((aw2013_data->outn == 0) &&
           ((aw2013_data->blink_mode == AW_SW_RESET)|
               (aw2013_data->blink_mode == AW_CONST_OFF)))
    write_reg(aw2013_data, 0x01, 0x00);// for aw2013 suspend setting GCR Register 0
    return 0;
};

static int aw2013_resume(struct i2c_client *cl)
{
/*
    struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);

    aw2013_SUSPEND_FLAG=false;
    printk("gsc debug:aw2013_resume!\n");
    write_reg(aw2013_data, 0x01, 0x01);//add by gsc  for aw2013 resume setting GCR Register 1
*/

    struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);
    if((aw2013_data->outn == 0) &&
           ((aw2013_data->blink_mode == AW_SW_RESET)|
               (aw2013_data->blink_mode == AW_CONST_OFF)))
    write_reg(aw2013_data, 0x01, 0x01);// for aw2013 resume setting GCR Register 1

    return 0;
};


static const struct i2c_device_id aw2013_id[] = {
	{ "aw2013", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, aw2013_id);

static struct i2c_driver aw2013_driver = {
	.driver = {
		.name = "aw2013",
//xiaofeng comment
#if 0
#ifdef CONFIG_OF
		.of_match_table = aw2013_match_table,
#endif
#endif
	},
	.id_table 	= aw2013_id,
	.probe 		= aw2013_probe,
	.remove 	= aw2013_remove,
	.suspend	= aw2013_suspend,
	.resume 	= aw2013_resume,
};


/*----------------------------------------------------------------------------*/
static int aw2013_platform_probe(struct platform_device *pdev) 
{
	//printk(KERN_ERR"xiaofeng: into aw2013_platform_probe\n");
	if(i2c_add_driver(&aw2013_driver))
	{
		printk(KERN_ERR"alexander: aw2013_platform_probe add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int aw2013_platform_remove(struct platform_device *pdev)
{
	//printk(KERN_ERR"xiaofeng: into aw2013_platform_remove\n");
    i2c_del_driver(&aw2013_driver);
    return 0;
}


#ifdef CONFIG_OF
static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "aw,aw2013", },
	{}
};
#endif


static struct platform_driver aw2013_platform_driver = {
	.probe      = aw2013_platform_probe,
	.remove     = aw2013_platform_remove,
	.driver     =
	{
		.name  = "aw2013",
	//	.owner  = THIS_MODULE,
        #ifdef CONFIG_OF
		.of_match_table = aw2013_match_table,
	#endif
	}
};

static int __init aw2013_init(void)
{
	//printk(KERN_ERR"xiaofeng:before i2c_register_board_info in %s\n",__func__);
	//i2c_register_board_info(AW2013_I2C_BUSNUM, &aw2013_i2c_info,1);//xiaofeng
	//printk(KERN_ERR"xiaofeng:after i2c_register_board_info in %s,before  i2c_add_driver\n",__func__);
	int err = 0;
	struct i2c_adapter *i2c1_host;
	//printk(KERN_ERR"xiaofeng:before i2c_get_adapter in %s\n",__func__);
	msleep(50);
	i2c1_host = i2c_get_adapter(1);
	if(NULL == i2c1_host)
	{
		printk(KERN_ERR"alexander:after  i2c_get_adapter failed in %s\n",__func__);
	}
	//msleep(1000);
	drv_client = i2c_new_device(i2c1_host,&aw2013_i2c_info);
	if(NULL == drv_client)
	{
		printk(KERN_ERR"alexander:after i2c_new_device failed in %s\n",__func__);
	}
	printk(KERN_ERR"alexander:after i2c_new_device in %s\n",__func__);
	//return i2c_add_driver(&aw2013_driver);
	//i2c_put_adapter;
	msleep(10);
	if(err = platform_driver_register(&aw2013_platform_driver))
	{
		printk(KERN_ERR"alexander:failed to register aw2013_platform_driver,err value=%d\n",err);
		return -ENODEV;
	}
	return err;
}

static void __exit aw2013_exit(void)
{
	i2c_del_driver(&aw2013_driver);
}


//late_initcall(aw2013_init);

module_init(aw2013_init);
module_exit(aw2013_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("nubia");
MODULE_DESCRIPTION("aw2013 Linux driver");
MODULE_ALIAS("platform:aw2013");

