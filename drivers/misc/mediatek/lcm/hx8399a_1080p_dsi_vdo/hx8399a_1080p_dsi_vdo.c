#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
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

#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)
#ifndef MACH_FPGA
#define GPIO_LCD_BIAS_ENP_PIN 	(GPIO57 | 0x80000000)    //GPIO57   ->LCD_AVDD_P_EN
#define GPIO_LCD_BIAS_ENN_PIN 	(GPIO58 | 0x80000000)    //GPIO58   ->LCD_AVDD_N_EN
#define GPIO_LCD_IOVCC_EN   	(GPIO104 | 0x80000000)   //GPIO104  ->LCD_IOVCC_EN
#define GPIO_LCD_ID1            (GPIO3 | 0x80000000)     //GPIO3    ->LCD_ID1
#define GPIO_LCD_ID0            (GPIO82 | 0x80000000)    //GPIO82   ->LCD_ID0
#endif
static unsigned int lcm_isok=0;
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};
//#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define GPIO_112_LCD_RST_PIN (GPIO158 |0x80000000)
#define SET_RESET_PIN(v) (lcm_util.set_gpio_out((GPIO_112_LCD_RST_PIN), (v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = 
{
	{0XB9, 3, {0XFF,0X83,0X99}},
	{0XD2, 1, {0X00}},

	{0XB1, 12, 
			{0X00,0X64,0X2F,
			 0X2F,0X00,0X09,0X22,
			 0X22,0X71,0XF1,0X54,
			 0X5E}}, 

	{0XB2, 10, 
			{0X00,0X80,0X00,
			 0X7F,0X05,0X07,0X23,
			 0X4D,0X02,0X01}}, 

    {0XBA, 2, {0X63,0X82}},

	{0XB4, 40, 
		  {0X00,0X44,0X02,
		   0X04,0X06,0X00,0X00,
		   0X00,0X08,0X00,0X00,
		   0X01,0X00,0X10,0X01,
		   0X02,0X05,0X41,0X00,
		   0X04,0X44,0X02,0X04,
		   0X06,0X00,0X00,0X00,
		   0X08,0X00,0X00,0X01,
		   0X00,0X10,0X01,0X02,
		   0X05,0X01,0X00,0X04,
		   0X44}}, 

	{0XD3, 31, 
			{0X00,0X01,0X00,0X00,
			0X00,0X04,0X04,0X32,
			0X10,0X04,0X00,0X04,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X21,0X05,
			0X05,0X13,0X00,0X00,
			0X00,0X05,0X08}}, 

	{0XD5, 32, 
			{0X00,0X00,0X21,0X20,
			0X19,0X19,0X18,0X18,
			0X00,0X00,0X01,0X00,
			0X18,0X18,0X03,0X02,
			0X19,0X19,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X32,0X32,
			0X31,0X31,0X30,0X30}}, 

	{0XD6, 32, 
			{0X40,0X40,0X20,0X21,
			0X18,0X18,0X19,0X19,
			0X40,0X40,0X02,0X03,
			0X18,0X18,0X00,0X01,
			0X19,0X19,0X40,0X40,
			0X40,0X40,0X40,0X40,
			0X40,0X40,0X32,0X32,
			0X31,0X31,0X30,0X30}}, 

	{0XD8, 48, 
			{0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0X00,0X00,0X00,0X00,
			0XAA,0XAE,0XEA,0XAA,
			0XAA,0XAE,0XEA,0XAA}}, 

	{0XE0, 42, 
			{0X00,0X0B,0X0F,0X2E,
			0X35,0X3F,0X18,0X37,
			0X05,0X0C,0X0D,0X10,
			0X11,0X10,0X12,0X13,
			0X18,0X09,0X17,0X08,
			0X14,0X00,0X0B,0X0F,
			0X2E,0X35,0X3F,0X18,
			0X37,0X05,0X0C,0X0D,
			0X10,0X11,0X10,0X12,
			0X13,0X18,0X09,0X17,
			0X08,0X14}}, 

	{0XC0, 2, {0X01,0X91}},
	
	{0XCC, 1, {0X08}},//{0XCC, 0X01, {0X00}} image filp

    {0XBC, 1, {0X03}},

    {0XB0, 4, {0X00,0X00,0X7D,0X0D}},
    {0xc9, 3 ,{0x1F,0x00,0x1B}},

    {0X11, 0, {0X00}},
    {REGFLAG_DELAY, 120, {}},
    {0X51, 1, {0Xff}},
    {0x55, 1 ,{0x01}},
    {0X53, 1, {0X24}},
    {REGFLAG_DELAY, 5, {}},

    {0X29, 0, {0X00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 60, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	//{0xB1, 1, {0x01}},
	//{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
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
    for(i = 0; i < count; i++) {		
        unsigned cmd;
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
static unsigned int lcm_compare_id(void)
{
	unsigned int id0 =1;
    unsigned int id1 =1;

    id0 = mt_get_gpio_in(GPIO_LCD_ID0);
    id1 = mt_get_gpio_in(GPIO_LCD_ID1);
    #ifdef BUILD_LK
            printf("%s,LK hx8399a debug: hx8399a id0 =%x id1 =%x\n",__func__,id0,id1);
	#else
            printk("%s,kernel hx8399a debug: hx8399a id0 =%x id1 =%x\n",__func__,id0,id1);
	#endif

    if((id0==0)&&(id1==0))
        return 1;
    else
        return 0;
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
#if defined(BUILD_LK)
    printf("%s\n", __func__);
#elif defined(BUILD_UBOOT)
#else
    printk("lcm_set_util_funcs\n");
#endif
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 3;
		params->dsi.vertical_frontporch					= 20;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active			= 20;
		params->dsi.horizontal_backporch				= 40;
		params->dsi.horizontal_frontporch				= 40;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		params->dsi.PLL_CLOCK=446;		// 240

		params->dsi.cont_clock=0;   
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable      = 0;

		params->dsi.lcm_esd_check_table[0].cmd          = 0x09;
		params->dsi.lcm_esd_check_table[0].count        = 3;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
		params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
		params->dsi.lcm_esd_check_table[0].para_list[2] = 0x04;

#if defined(BUILD_LK)
    printf("%s\n", __func__);
#elif defined(BUILD_UBOOT)
#else
    printk("lcm_get_params\n");
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
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    MDELAY(5);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    MDELAY(5);

    mt_set_gpio_mode(GPIO_112_LCD_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
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
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(10);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
#ifndef MACH_FPGA
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
    MDELAY(2);
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
    MDELAY(5);
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
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(5);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_dir(GPIO_112_LCD_RST_PIN, GPIO_OUT_ONE);
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
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);   //wqtao. enable
}


static void lcm_resume(void)
{
	lcm_init();
}

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
    unsigned int cmd = 0x51;
    unsigned char count = 1;
    unsigned char value ;
    if(level<0x06)level=0x06;
    value = level;

    dsi_set_cmdq_V22(handle, cmd, count, &value, 1);

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

LCM_DRIVER hx8399a_1080p_dsi_vdo_lcm_drv = 
{
  .name			= "hx8399a_1080p_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.set_backlight_cmdq  = lcm_setbacklight_cmdq,
	.compare_id     = lcm_compare_id,
	.init_power    = lcm_init_power,
	.resume_power  = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
//	.set_cabc = lcm_set_cabc,
//	.get_cabc = lcm_get_cabc,
//	.is_lcmon = is_lcmon,
};

