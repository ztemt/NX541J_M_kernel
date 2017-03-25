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
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
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
#include <linux/time.h>
#include <linux/rtc.h>

#endif
#include "../inc/lcm_drv.h"
//#include <cust_gpio_usage.h>

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_gpio.h>

#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#endif

#define FRAME_WIDTH                                         (1080)
#define FRAME_HEIGHT                                        (1920)
#ifndef MACH_FPGA
#define GPIO_LCD_BIAS_ENP_PIN 	(GPIO57 | 0x80000000)    //GPIO57   ->LCD_AVDD_P_EN
#define GPIO_LCD_BIAS_ENN_PIN 	(GPIO58 | 0x80000000)    //GPIO58   ->LCD_AVDD_N_EN
#define GPIO_LCD_IOVCC_EN   	(GPIO104 | 0x80000000)   //GPIO104  ->LCD_IOVCC_EN
#define GPIO_LCD_ID1            (GPIO3 | 0x80000000)     //GPIO3    ->LCD_ID1
#define GPIO_LCD_ID0            (GPIO82 | 0x80000000)    //GPIO82   ->LCD_ID0
#endif
static unsigned int lcm_isok=0;
#define LCM_ID                       (0x99)

#define REGFLAG_DELAY               (0XFE)
#define REGFLAG_END_OF_TABLE        (0x100) // END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE                                    0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

//#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define GPIO_112_LCD_RST_PIN (GPIO158 |0x80000000)
#define SET_RESET_PIN(v) (lcm_util.set_gpio_out((GPIO_112_LCD_RST_PIN), (v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xB9, 3, {0xFF,0x83,0x99}},
    {0x11, 1 ,{0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0xE0, 54 ,{0x02,0x22,0x33,0x31,0x6F,0x79,0x84,0x7E,0x85,0x8D,
                0x94,0x98,0x9B,0xA0,0xA8,0xAC,0xAE,0xB5,0xB7,0xBE,
                0xAF,0xBC,0xBF,0x61,0x5C,0x65,0x7C,0x02,0x22,0x33,
                0x31,0x6F,0x79,0x84,0x7E,0x85,0x8D,0x93,0x98,0x9B,
                0xA5,0xAB,0xAB,0xAE,0xB6,0xB6,0xBE,0xB3,0xBD,0xBB,
                0x60,0x5C,0x67,0x7C}},
    {REGFLAG_DELAY, 5, {}},
    {0xC1, 43 ,{0x01,0x00,0x08,0x10,0x18,0x21,0x29,0x31,0x39,0x41,
                0x4A,0x51,0x58,0x61,0x69,0x71,0x7A,0x81,0x88,0x8F,
                0x98,0xA0,0xA8,0xB1,0xB8,0xC0,0xC8,0xD0,0xD8,0xE0,
                0xE8,0xF0,0xF8,0xFF,0x1B,0x01,0x07,0x00,0xED,0xAB,
                0x60,0x55,0xC0}}, 
    {0xBD, 1 ,{0x01}},
    {0xC1, 42 ,{0x00,0x08,0x10,0x18,0x20,0x28,0x30,0x38,0x3F,0x48,
                0x50,0x57,0x5F,0x67,0x6F,0x78,0x80,0x87,0x8E,0x97,
                0x9F,0xA7,0xB0,0xB7,0xBF,0xC7,0xCF,0xD7,0xDF,0xE7,
                0xEF,0xF7,0xFF,0x06,0x94,0xE1,0xE9,0x9C,0x53,0xA9,
                0x67,0xC0}},
    {0xBD, 1 ,{0x02}},
    {0xC1, 42 ,{0x00,0x08,0x0F,0x17,0x1F,0x27,0x30,0x38,0x40,0x48,
                0x50,0x57,0x60,0x67,0x6F,0x78,0x81,0x88,0x8F,0x98,
                0xA0,0xA8,0xB2,0xB9,0xC1,0xC9,0xD1,0xD9,0xE1,0xE9,
                0xF1,0xF9,0xFF,0x0F,0xF0,0x36,0x3F,0x5D,0xB1,0xB9,
                0xFD,0xC0}},
    {0xBD, 1 ,{0x00}},
    {0xE4, 2 ,{0x01,0x03}},
    {REGFLAG_DELAY, 10, {}},
    {0xB1,15, {0x02,0x04,0x6D,0x8D,0x01,0x32,0x33,0x11,0x11,0x5A,0x5F,0x56,0x73,0x02,0x02}},
    {0xB2,11, {0x00,0x80,0x80,0xAE,0x05,0x07,0x5A,0x11,0x10,0x10,0x00}},
    {0xc9, 3 ,{0x03,0x00,0x0A}},
    {0xB4,44, {0x00,0xFF,0x10,0x18,0x04,0x9A,0x00,0x00,0x06,0x00,0x02,0x04,0x00,0x24,0x02,0x04,0x0A,0x21,0x03,0x00,0x00,0x02,0x9F,0x88,0x10,0x18,0x04,0x9A,0x00,0x00,0x08,0x00,0x02,0x04,0x00,0x24,0x02,0x04,0x0A,0x00,0x00,0x02,0x9F,0x12}},
    {0xD3,33, {0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x05,0x00,0x05,0x00,0x07,0x88,0x07,0x88,0x00,0x00,0x00,0x00,0x00,0x15,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x05,0x40}},
    {0xD5,32, {0x20,0x20,0x19,0x19,0x18,0x18,0x01,0x01,0x00,0x00,0x25,0x25,0x18,0x18,0x18,0x18,0x24,0x24,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x2F,0x2F,0x30,0x30,0x31,0x31}},
    {0xD6,32, {0x24,0x24,0x18,0x18,0x19,0x19,0x00,0x00,0x01,0x01,0x25,0x25,0x18,0x18,0x18,0x18,0x20,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x2F,0x2F,0x30,0x30,0x31,0x31}},
    {0xD8,16, {0xAA,0x8A,0xAA,0xAA,0xAA,0x8A,0xAA,0xAA,0xAA,0x8A,0xAA,0xAA,0xAA,0x8A,0xAA,0xAA}},
    {0xBD, 1, {0x01}},
    {0xD8,16, {0x38,0x0A,0x2A,0x80,0x38,0x0A,0x2A,0x80,0xBA,0xCA,0xAA,0xAA,0xBA,0xCA,0xAA,0xAA}},
    {0xBD, 1, {0x02}},
    {0xD8, 8, {0xFF,0xFC,0xC0,0x3F,0xFF,0xFC,0xC0,0x3F}},
    {0xBD, 1, {0x00}},
    {0xB6, 2, {0x86,0x86}},
    {0xCC, 1, {0x08}},
    //{0xE0,54, {0x02,0x24,0x31,0x2C,0x60,0x6A,0x77,0x73,0x7B,0x83,0x8C,0x90,0x96,0x9C,0x9C,0xA5,0xAA,0xAE,0xB0,0xB8,0xA7,0xB4,0xB9,0x5D,0x57,0x5E,0x7C,0x02,0x24,0x31,0x2C,0x60,0x6A,0x77,0x73,0x7B,0x83,0x8B,0x90,0x96,0x9C,0xA7,0xA4,0xA8,0xAF,0xB0,0xB5,0xAC,0xB6,0xB6,0x5B,0x56,0x60,0x7C}}, //55
    {0x51, 2 ,{0x00,0x00}},
    {0x55, 1 ,{0x01}},
    {0x53, 1 ,{0x24}},
    {REGFLAG_DELAY, 5, {}},
    {0x29, 1 ,{0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}

};

#if 0
static struct LCM_setting_table lcm_esd_setting[] = {
	{0xFF,1,{0xEE}},
	{0x26,1,{0x08}},
	{0x26,1,{0x00}},
	{0xFF,1,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

unsigned char value1[256]={
/*0x03 lowest backlight,less than 0x03,pwm duty less than backlight ic spec*/
0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
0x03, 0x03, 0x04, 0x04, 0x05, 0x05, 0x05, 0x06,
0x06, 0x06, 0x07, 0x07, 0x07, 0x07, 0x08, 0x08,
0x08, 0x08, 0x09, 0x09, 0x0a, 0x0a, 0x0a, 0x0b,
0x0b, 0x0b, 0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x0e,
0x0e, 0x0e, 0x0f, 0x0f, 0x10, 0x10, 0x10, 0x11,
0x11, 0x12, 0x12, 0x13, 0x13, 0x14, 0x14, 0x15,
0x15, 0x16, 0x16, 0x17, 0x17, 0x18, 0x18, 0x19,
0x19, 0x1a, 0x1a, 0x1b, 0x1c, 0x1c, 0x1d, 0x1d,
0x1e, 0x1f, 0x20, 0x20, 0x21, 0x22, 0x23, 0x23,
0x24, 0x25, 0x26, 0x26, 0x27, 0x28, 0x29, 0x29,
0x2a, 0x2b, 0x2c, 0x2c, 0x2d, 0x2e, 0x2f, 0x2f,
0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
0x38, 0x39, 0x3a, 0x3a, 0x3b, 0x3c, 0x3d, 0x3d,
0x3e, 0x3f, 0x40, 0x41, 0x43, 0x44, 0x45, 0x46,
0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e,
0x4f, 0x50, 0x52, 0x53, 0x54, 0x55, 0x57, 0x58,
0x59, 0x5a, 0x5c, 0x5d, 0x5e, 0x5f, 0x61, 0x62,
0x63, 0x64, 0x66, 0x67, 0x68, 0x69, 0x6b, 0x6c,
0x6d, 0x6f, 0x70, 0x72, 0x73, 0x75, 0x76, 0x78,
0x79, 0x7b, 0x7c, 0x7e, 0x7f, 0x81, 0x82, 0x84,
0x85, 0x87, 0x88, 0x8a, 0x8c, 0x8d, 0x8f, 0x90,
0x92, 0x94, 0x95, 0x97, 0x98, 0x9a, 0x9b, 0x9d,
0x9e, 0xa1, 0xa3, 0xa6, 0xa8, 0xab, 0xad, 0xb0,
0xb2, 0xb4, 0xb6, 0xb8, 0xba, 0xbb, 0xbd, 0xbf,
0xc1, 0xc3, 0xc4, 0xc6, 0xc8, 0xc9, 0xcb, 0xcc,
0xce, 0xd1, 0xd3, 0xd6, 0xd8, 0xdb, 0xdd, 0xe0,
0xe2, 0xe4, 0xe7, 0xe9, 0xeb, 0xed, 0xf0, 0xf2,
0xf4, 0xf6, 0xf7, 0xf9, 0xfa, 0xfc, 0xfd, 0xff
};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    //{0x2C, 1, {0x00}},
    //{0x13, 1, {0x00}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 60, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    //{0xB1, 1, {0x03}},
    //{REGFLAG_DELAY, 10, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#if 0
static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
    {0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 2, {0x0F,0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_level_setting[] = {
    {0x55, 1, {0x01}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned int cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
            
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
                
            case REGFLAG_END_OF_TABLE :
                break;
                
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
    
}
#if 0
static struct LCM_setting_table lcm_saturation_level_setting[] = {
    {0xE4, 2, {0x01,0x03}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static unsigned int saturation_level=3;

static void lcm_set_saturation(unsigned int level){
    lcm_initialization_setting[11].para_list[1] = level;
    lcm_saturation_level_setting[0].para_list[1] = level;
    saturation_level = level;
    push_table(lcm_saturation_level_setting,sizeof(lcm_saturation_level_setting) / sizeof(struct LCM_setting_table),1);
}

static void lcm_get_saturation(unsigned int *level){
      *level = saturation_level;
}
#endif
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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

	// enable tearing-free
//#if (LCM_DSI_CMD_MODE)
//	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
//	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
//
//#endif

//#if (LCM_DSI_CMD_MODE)
//	params->dsi.mode   = CMD_MODE;
//#else
    params->dsi.mode = SYNC_PULSE_VDO_MODE;
//#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;// 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;	
		
	params->dsi.vertical_sync_active				= 4;//2
	params->dsi.vertical_backporch					= 3;//4;//16;///8
	params->dsi.vertical_frontporch					= 20;//16;///8
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 20;//8;
	params->dsi.horizontal_backporch				= 40;//40//60;
	params->dsi.horizontal_frontporch				= 40;//60;//60;//140;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 446;

#if 1
  params->dsi.cont_clock=0;   
  params->dsi.esd_check_enable = 0;
  params->dsi.customization_esd_check_enable      = 0;

  params->dsi.lcm_esd_check_table[0].cmd          = 0x09;
  params->dsi.lcm_esd_check_table[0].count        = 3;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
  params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
  params->dsi.lcm_esd_check_table[0].para_list[2] = 0x04;
#endif		
}

static void lcm_init_power(void)
{
#ifndef MACH_FPGA
    mt_set_gpio_mode(GPIO_LCD_ID0, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_ID0, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_LCD_ID0, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_LCD_ID0, GPIO_PULL_UP);

    mt_set_gpio_mode(GPIO_LCD_ID1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_ID1, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_LCD_ID1, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_LCD_ID1, GPIO_PULL_UP);
    MDELAY(2);
    mt_set_gpio_mode(GPIO_LCD_IOVCC_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_IOVCC_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_IOVCC_EN, GPIO_OUT_ONE);
    MDELAY(2);
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
    MDELAY(5);
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    MDELAY(5);

    mt_set_gpio_mode(GPIO_112_LCD_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);

    MDELAY(5);
#endif
}

static void lcm_resume_power(void)
{
    lcm_init_power();
}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
    printf("[erick-lk]%s\n", __func__);
#else
    printk("[erick-k]%s\n", __func__);
#endif

    mt_set_gpio_mode(GPIO_112_LCD_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_DIR_OUT);	
    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
    MDELAY(5);

    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);

    MDELAY(5);

    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
#ifndef MACH_FPGA
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
    MDELAY(2);
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
    MDELAY(2);
    mt_set_gpio_mode(GPIO_LCD_ID0, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_ID0, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_ID0, GPIO_OUT_ZERO);
    mt_set_gpio_mode(GPIO_LCD_ID1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_ID1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_ID1, GPIO_OUT_ZERO);
    mt_set_gpio_mode(GPIO_LCD_IOVCC_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_IOVCC_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_IOVCC_EN, GPIO_OUT_ZERO);
    MDELAY(5);
#endif

}

static void lcm_init(void)
{
    mt_set_gpio_mode(GPIO_112_LCD_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_DIR_OUT);	
    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(5);

    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);

    MDELAY(10);

    mt_set_gpio_out(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(50);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK
    printf("[erick-lk]%s\n", __func__);
#else
    printk("[erick-k]%s\n", __func__);
#endif
}

static void lcm_suspend(void)
{
    lcm_isok=0;
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);   //wqtao. enable
}


static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
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

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_compare_id(void)
{
    unsigned int id0 =1;
    unsigned int id1 =1;

    id0 = mt_get_gpio_in(GPIO_LCD_ID0);
    id1 = mt_get_gpio_in(GPIO_LCD_ID1);
    #ifdef BUILD_LK
            printf("%s,LK hx8399c debug: hx8399c id0 =%x id1 =%x\n",__func__,id0,id1);
	#else
            printk("%s,kernel hx8399c debug: hx8399c id0 =%x id1 =%x\n",__func__,id0,id1);
	#endif

    if((id0==0)&&(id1==1))
        return 1;
    else
        return 0;
}

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)   lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
    unsigned int cmd = 0x51;
    unsigned char count = 2;
    unsigned char value[2]  = {0};

    pr_err("%s level:%d\n", __func__, level);
    level=value1[level];
    value[0] = ((level*0x0fff/0x00ff)>>8)&0xff;
    value[1] = (level*0x0fff/0x00ff)&0xff;

    dsi_set_cmdq_V22(handle, cmd, count, value, 1);

    MDELAY(20);
    lcm_isok=1;
}

#if 0
static unsigned int cabc_level=CABC_LEVER1;
static void lcm_set_cabc(unsigned int level){
    lcm_cabc_level_setting[0].para_list[0] = level-CABC_OFF;
	cabc_level = level;
    push_table(lcm_cabc_level_setting,sizeof(lcm_cabc_level_setting) / sizeof(struct LCM_setting_table),1);
}

static void lcm_get_cabc(unsigned int *level){
      *level = cabc_level;
}

static void is_lcmon(unsigned int *level){
    *level = lcm_isok;
}
#endif
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx8399c_dsi_vdo_fhd_auo_lcm_drv = 
{
    .name           = "hx8399c_dsi_vdo_fhd_auo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight_cmdq  = lcm_setbacklight_cmdq,
    .compare_id    = lcm_compare_id,
    .init_power    = lcm_init_power,
    .resume_power  = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
#if 0
    .set_cabc = lcm_set_cabc,
    .get_cabc = lcm_get_cabc,
    .set_saturation = lcm_set_saturation,
    .get_saturation = lcm_get_saturation,
    .is_lcmon = is_lcmon,
#endif
};

