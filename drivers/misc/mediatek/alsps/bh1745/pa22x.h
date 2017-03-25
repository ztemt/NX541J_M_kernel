/* 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Version: 1.92.1.01
 *	-Add numerator and denominator value to transfer adc to lux
 *	-Add flag to identify whether update continous lux value or discrete lux value defined in cust_alsps.c
 * Version: 1.92.1.02
 *    -pa22x00001.h -> pa22x.h
 * Version: 1.92.1.03
 *    -add ALS_USE_AVG_DATA
 * Version: 1.92.1.04
 *    -add ALS Interrupt
 * Version: 1.0.1.01
 *	  -Combine 1.92.1.04 & 1.89.1.04 
 */
/*
 * Definitions for TXC PA22X series als/ps sensor chip.
 */
#ifndef __PA22X_H__
#define __PA22X_H__

#include <linux/ioctl.h>

#define PA22_DRIVER_VERSION_H	"1.1.1"


/* DEVICE can change */ 
//#define DEVICE_PA22A
#define	DEVICE_PA224	
/*pa22x als/ps Default*/  
#define PA22_I2C_ADDRESS		0x1E  	/* 7 bit Address */
/*-----------------------------------------------------------------------------*/
#define PA22_ALS_TH_HIGH		65535
#define PA22_ALS_TH_LOW		0
/*-----------------------------------------------------------------------------*/
#define PA22_PS_TH_HIGH		40
#define PA22_PS_TH_LOW			25
#define PA22_PS_TH_MIN		  	0		/* Minimun value */
#define PA22_PS_TH_MAX 			255		/* 8 bit MAX */
/*-----------------------------------------------------------------------------*/
#define PA22_PS_TH_BASE_HIGH 		20
#define PA22_PS_TH_BASE_LOW		18
#define PA22_PS_TH_HIGH_MINIMUM	40
#define PA22_PS_TH_INTERVAL			15
/*-----------------------------------------------------------------------------*/
#define PA22_PS_OFFSET_DEFAULT	0	 	/* for X-talk cannceling */
#define PA22_PS_OFFSET_EXTRA		1
#define PA22_PS_OFFSET_MAX			150
#define PA22_PS_OFFSET_MIN			0
#define PA22_FAST_CAL					1
#define PA22_FAST_CAL_ONCE			0

/* pa22x als/ps parameter setting */
#define PA22_ALS_GAIN		3 	/* 0:125lux | 1:1000lux | 2:2000lux | 3:10000lux */
#define PA22_LED_CURR		5 	/* 0:150mA | 1:100mA | 2:50mA | 3:25mA | 4:15mA | 5:12mA | 6:10mA | 7:7mA*/

#define PA22_PS_PRST		2 /* 0:1point | 1:2points | 2:4points | 3:8points (for INT) */
#define PA22_ALS_PRST		0	/* 0:1point | 1:2points | 2:4points | 3:8points (for INT) */

#define PA22_PS_SET			1	/* 0:ALS only | 1:PS only | 3:BOTH */
#define PA22_PS_MODE			0	/* 0:OFFSET |1:NORMAL */

#define PA22_INT_TYPE			0 	/* 0:Window type | 1:Hysteresis type for Auto Clear flag */
#define PA22_PS_PERIOD		0	/* 0:6.25 ms | 1:12.5 ms | 2:25 ms | 3:50 ms | 4:100 ms | 5:200 ms | 6:400 ms | 7:800 ms */
#define PA22_ALS_PERIOD	0	/* 0:0 ms | 1:100 ms | 2:300 ms | 3:700 ms | 4:1500 ms  */
#define PA22_PS_FLTFC			0	/* 0~4 */

/*pa22x als/ps sensor register map*/
#define REG_CFG0 				0X00		/* ALS_GAIN(D5-4) | PS_ON(D1) | ALS_ON(D0) */
#define REG_CFG1 				0X01 	/* LED_CURR(D6-4) | PS_PRST(D3-2) | ALS_PRST(D1-0) */
#define REG_CFG2 				0X02 	/* PS_MODE(D6) | CLEAR(D4) | INT_SET(D3-2) | PS_INT(D1) | ALS_INT(D0) */
#define REG_CFG3				0X03		/* INT_TYPE(D6) | PS_PERIOD(D5-3) | ALS_PERIOD(D2-0) */
#define REG_ALS_TL_LSB		0X04		/* ALS Threshold Low LSB */
#define REG_ALS_TL_MSB		0X05		/* ALS Threshold Low MSB */
#define REG_ALS_TH_LSB		0X06		/* ALS Threshold high LSB */
#define REG_ALS_TH_MSB		0X07		/* ALS Threshold high MSB */
#define REG_PS_TL				0X08		/* PS Threshold Low */
#define REG_PS_TH				0X0A	/* PS Threshold High */
#define REG_ALS_DATA_LSB	0X0B	/* ALS DATA LSB */
#define REG_ALS_DATA_MSB	0X0C	/* ALS DATA MSB */
#define REG_PS_DATA			0X0E		/* PS DATA */
#define REG_PS_OFFSET		0X10		/* TBD */
#define REG_PS_SET				0X11		/* 0x82 */
#define REG_CFG4				0x12		/* Typical = 0x0C */
       

/* ALS Using average data */
#define ALS_USE_AVG_DATA 0

/* ALS: Update continuous lux or use discrete value defined in cust_alsps.c */
#define PA22_ALS_ADC_TO_LUX_USE_LEVEL	0

#define PS_CAL_FILE_PATH	"/persist/sensors/xtalk_cal"  
/* PS Calibration setting */
//#define PA22_MIN_NEAR_CNT	15  // min 3cm count 
//#define PA22_NEAR_FAR_CNT	10	// 12mA 3cm count ~ 4 cm count
#define PA22_MIN_NEAR_CNT	8   // min 3cm count
/* Interrupt step */
#define forward_step 	25
#define backward_step 	5

/*add by yangguibin start*/
#define PS_CAL_FILE_PATH_DIR	"/persist/sensors/" 
#define PA22X_CHIP_NAME 	"pa224"
#define PA22X_UNCOVER_MAX 	80
#define PA22X_THRES_MIN 	15
#define PA22X_THRES_MAX 	45
/*add by yangguibin end*/


#define PA22X_ONLY_PS


#ifdef PA22X_ONLY_PS
#include "rohm_bh1745_i2c.h"
#endif
#endif

