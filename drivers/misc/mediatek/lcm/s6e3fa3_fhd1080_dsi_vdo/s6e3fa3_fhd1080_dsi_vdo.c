/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION LCM_WIDTHOR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)
 
//modify backlight
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)\
        lcm_util.dsi_set_cmdq_V22(cmdq,cmd,count,ppara,force_update)
#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif


#ifdef BUILD_LK
#define	LCM_DEBUG(format, ...)   printf("lk " format "\n", ## __VA_ARGS__)
#elif defined(BUILD_UBOOT)
#define	LCM_DEBUG(format, ...)   printf("uboot " format "\n", ## __VA_ARGS__)
#else
#define	LCM_DEBUG(format, ...)   printk("kernel " format "\n", ## __VA_ARGS__)
#endif
#define set_gpio_lcd_enp(cmd) lcm_util.set_gpio_lcd_enp_bias(cmd);
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
//static unsigned int is_lcm_first_init = 1;
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID_S6E3FA3_CMD_W1    0x0102
#define LCM_ID_S6E3FA3_CMD_B1     0x0103
#define LCM_ID_S6E3FA3_CMD_W2    0x0304

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        		lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	       	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)									lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define REGFLAG_DELAY             								0xFE
#define REGFLAG_END_OF_TABLE      							0xFF


struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};
unsigned char value1[256]={
        0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
        0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
        0x03,0x03,0x04,0x04,0x04,0x04,0x04,0x04,
        0x04,0x04,0x04,0x04,0x05,0x05,0x05,0x05,
        0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
        0x06,0x06,0x07,0x07,0x08,0x08,0x08,0x08,
        0x08,0x08,0x09,0x09,0x0A,0x0A,0x0B,0x0B,
        0x0B,0x0B,0x0C,0x0C,0x0D,0x0D,0x0E,0x0E,
        0x0E,0x0E,0x0F,0x0F,0x0F,0x0F,0x10,0x10,
        0x10,0x10,0x11,0x11,0x12,0x12,0x13,0x13,
        0x13,0x14,0x14,0x15,0x15,0x16,0x16,0x17,
        0x18,0x18,0x19,0x1A,0x1A,0x1B,0x1B,0x1C,
        0x1D,0x1D,0x1E,0x1F,0x20,0x20,0x21,0x21,
        0x21,0x22,0x23,0x25,0x25,0x27,0x28,0x29,
        0x29,0x2A,0x2B,0x2C,0x2D,0x2D,0x2E,0x2F,
        0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36,
        0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,
        0x3E,0x3F,0x40,0x41,0x42,0x43,0x45,0x47,
        0x49,0x49,0x4B,0x4D,0x4F,0x50,0x51,0x52,
        0x53,0x54,0x56,0x58,0x5A,0x5B,0x5C,0x5D,
        0x5F,0x61,0x62,0x64,0x65,0x66,0x68,0x69,
        0x6B,0x6C,0x6E,0x6F,0x70,0x72,0x73,0x73,
        0x75,0x75,0x77,0x79,0x7B,0x7C,0x7D,0x7E,
        0x7F,0x7F,0x81,0x83,0x85,0x86,0x87,0x88,
        0x89,0x90,0x91,0x92,0x93,0x95,0x96,0x96,
        0x97,0x98,0x9A,0x9C,0x9E,0xA0,0xA2,0xA3,
        0xA4,0xA6,0xA8,0xAA,0xAC,0xAE,0xB0,0xB1,
        0xB1,0xB3,0xB5,0xB7,0xB9,0xBB,0xBD,0xBE,
        0xBE,0xC1,0xC2,0xC4,0xC6,0xC8,0xCA,0xCC,
        0xCE,0xD0,0xD2,0xD4,0xD6,0xD8,0xDA,0xDC,
        0xDE,0xE0,0xE2,0xE4,0xE6,0xE8,0xEA,0xEC,
        0xEE,0xF0,0xF3, 0xF5,0xF8,0xFA,0xFC,0xFF
};

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   =  CMD_MODE;
	
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format     		 = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.packet_size=256;
	params->dsi.word_count=FRAME_WIDTH*3;

	params->dsi.vertical_sync_active					= 2;
	params->dsi.vertical_backporch					= 7;
	params->dsi.vertical_frontporch					= 7;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch					= 14;
	params->dsi.horizontal_frontporch					= 30;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	//Data rate= width*height*1.2* total_bit_per_pixel*frame_per_second/total_lane_num
	//clk=data_rate/2
	params->dsi.PLL_CLOCK =420; 

	params->dsi.cont_clock=1;
	params->dsi.clk_lp_per_line_enable = 0;
	//params->dsi.esd_check_enable 					= 0;
	//params->dsi.customization_esd_check_enable 	= 0;
	//params->dsi.lcm_esd_check_table[0].cmd			= 0x53;
	//params->dsi.lcm_esd_check_table[0].count		= 1;
	//params->dsi.lcm_esd_check_table[0].para_list[0] 	= 0x24;
}

static void lcm_init(void)
{
	unsigned int data_array[16];

      set_gpio_lcd_enp(1);
	
	SET_RESET_PIN(0);
	MDELAY(5);
        SET_RESET_PIN(0);

    MDELAY(15);
        SET_RESET_PIN(1);
//    MDELAY(15);
//    mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ZERO);
//    MDELAY(15);
//    mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(5);	


	//sleep out
	data_array[0] = 0x00110500;		   
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(25);
	//common setting -TE
	data_array[0] = 0x00351500;			   
	dsi_set_cmdq(data_array, 1, 1); 

	//AVC setting
	data_array[0]= 0x00033902;
	data_array[1]= 0x005A5AFC;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x1EB01500;			   
	dsi_set_cmdq(data_array, 1, 1); 

	data_array[0] = 0xA8FD1500;
	dsi_set_cmdq(data_array, 1, 1); 
	
	data_array[0]= 0x00033902;
	data_array[1]= 0x00A5A5FC;
	dsi_set_cmdq(data_array, 2, 1);
	
	//brightness control
	data_array[0] = 0x20531500; 			   
        dsi_set_cmdq(data_array, 1, 1);
 
        data_array[0] = 0x01551500;
        dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x00511500;
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(120);
        data_array[0] = 0x002C1500;
        dsi_set_cmdq(data_array, 1, 1);

    	data_array[0] = 0x00290500;			   
    	dsi_set_cmdq(data_array, 1, 1); 
    	MDELAY(20);
	
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];


	// Display Off
	data_array[0]=0x00280500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 

	// Sleep In
	data_array[0] = 0x00100500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);   
        set_gpio_lcd_enp(1);
        MDELAY(2);

	SET_RESET_PIN(0);
}

static void lcm_resume(void)
{
	lcm_init(); 
}
         
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[3] = {0};
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 

	array[0] = 0x00033700;
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0x04, buffer, 3);
	id = (buffer[1]<<8) | buffer[2]; //driver IC  
    #ifdef BUILD_LK
	printf("%s, LK s6e3fa3 debug:  buffer[0] = 0x%04x, id = 0x%04x, buffer[2] = 0x%04x\n", 
		__func__, buffer[0], id, buffer[2]);
    #endif

    if((id == LCM_ID_S6E3FA3_CMD_W1)||
	(id == LCM_ID_S6E3FA3_CMD_W2)||
	(id == LCM_ID_S6E3FA3_CMD_B1))
    	return 1;
    else
        return 0;
}
//modify backlight
static void lcm_setbacklight_cmdq(void* handle,unsigned int level)
{	unsigned int cmd = 0x51;
	unsigned char count = 1;
	unsigned char value = level;
        unsigned char valuecode = 0;
        valuecode=value1[value];
	dsi_set_cmdq_V22(handle,cmd,count,&valuecode,1);
}

LCM_DRIVER s6e3fa3_fhd1080_dsi_vdo_lcm_drv = 
{
	.name			= "s6e3fa3_fhd1080_dsi_vdo",
	.set_util_funcs 		= lcm_set_util_funcs,
	.get_params     		= lcm_get_params,
	.init           			= lcm_init,
	.suspend        		= lcm_suspend,
	.resume         		= lcm_resume,
	.set_backlight_cmdq		= lcm_setbacklight_cmdq,
	.compare_id     		= lcm_compare_id,
	.update         		= lcm_update,
};


