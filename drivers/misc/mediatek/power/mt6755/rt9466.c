/*
 *  Driver for Richtek RT9466 Charger
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  ShuFanLee <shufan_lee@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include "rt9466.h"
#include "mtk_charger_intf.h"

#ifdef CONFIG_RT_REGMAP
#include <mt-plat/rt-regmap.h>
#endif

kal_bool chargin_hw_init_done; /* Used by MTK battery driver */
int(*mtk_charger_intf[CHARGING_CMD_NUMBER])(void *data);

/* ======================= */
/* RT9466 Parameter        */
/* ======================= */

const u32 rt9466_boost_oc_threshold[RT9466_BOOST_OC_THRESHOLD_NUM] = {
	500, 700, 1100, 1300, 1800, 2100, 2400, 3000,
}; /* mA */

enum rt9466_charging_status {
	RT9466_CHG_STATUS_READY = 0,
	RT9466_CHG_STATUS_PROGRESS,
	RT9466_CHG_STATUS_DONE,
	RT9466_CHG_STATUS_FAULT,
	RT9466_CHG_STATUS_MAX,
};

/* Charging status name */
static const char *rt9466_chg_status_name[RT9466_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};


/* ======================= */
/* Address & Default value */
/* ======================= */


static const unsigned char rt9466_reg_addr[RT9466_REG_IDX_MAX] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x10, 0x11, 0x19, 0x1A,
	0x40, 0x42, 0x43, 0x44, 0x45,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65,
};


#define RT9466_REG_EN_HIDDEN_MODE_MAX 8
static const u8 rt9466_reg_en_hidden_mode[RT9466_REG_EN_HIDDEN_MODE_MAX] = {
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
};

static const u8 rt9466_val_en_hidden_mode[RT9466_REG_EN_HIDDEN_MODE_MAX] = {
	0x49, 0x32, 0xB6, 0x27, 0x48, 0x18, 0x03, 0xE2,
};

enum rt9466_iin_limit_sel {
	RT9466_IIMLMTSEL_PSEL_OTG,
	RT9466_IINLMTSEL_AICR = 2,
	RT9466_IINLMTSEL_LOWER_LEVEL, /* lower of above two */
};

enum rt9466_adc_sel {
	RT9466_ADC_VBUS_DIV5 = 1,
	RT9466_ADC_VBUS_DIV2,
	RT9466_ADC_VSYS,
	RT9466_ADC_VBAT,
	RT9466_ADC_TS_BAT = 6,
	RT9466_ADC_IBAT = 9,
	RT9466_ADC_REGN = 11,
	RT9466_ADC_TEMP_JC,
	RT9466_ADC_MAX,
};

/* Unit for each ADC parameter
 * 0 stands for reserved
 * For TS_BAT, the real unit is 0.25.
 * Here we use 25, please remember to divide 100 while showing the value
 */
static const int rt9466_adc_unit[RT9466_ADC_MAX] = {
	0,
	RT9466_ADC_UNIT_VBUS_DIV5,
	RT9466_ADC_UNIT_VBUS_DIV2,
	RT9466_ADC_UNIT_VSYS,
	RT9466_ADC_UNIT_VBAT,
	0,
	RT9466_ADC_UNIT_TS_BAT,
	0,
	0,
	RT9466_ADC_UNIT_IBAT,
	0,
	RT9466_ADC_UNIT_REGN,
	RT9466_ADC_UNIT_TEMP_JC,
};

static const int rt9466_adc_offset[RT9466_ADC_MAX] = {
	0,
	RT9466_ADC_OFFSET_VBUS_DIV5,
	RT9466_ADC_OFFSET_VBUS_DIV2,
	RT9466_ADC_OFFSET_VSYS,
	RT9466_ADC_OFFSET_VBAT,
	0,
	RT9466_ADC_OFFSET_TS_BAT,
	0,
	0,
	RT9466_ADC_OFFSET_IBAT,
	0,
	RT9466_ADC_OFFSET_REGN,
	RT9466_ADC_OFFSET_TEMP_JC,
};

/* ========= */
/* RT Regmap */
/* ========= */

#ifdef CONFIG_RT_REGMAP
RT_REG_DECL(RT9466_REG_CORE_CTRL0, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL4, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL5, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL6, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL7, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL8, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL9, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL10, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL11, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL12, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL13, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL14, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL15, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL16, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_ADC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL17, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_CTRL18, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_DEVICE_ID, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_STAT, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_NTC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_ADC_DATA_H, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_ADC_DATA_L, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_STATC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_FAULT, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_TS_STATC, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ1, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ2, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ3, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_STATC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_FAULT_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_TS_STATC_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ1_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ2_CTRL, 1, RT_VOLATILE, {});
RT_REG_DECL(RT9466_REG_CHG_IRQ3_CTRL, 1, RT_VOLATILE, {});

static rt_register_map_t rt9466_regmap_map[RT9466_REG_IDX_MAX] = {
	RT_REG(RT9466_REG_CORE_CTRL0),
	RT_REG(RT9466_REG_CHG_CTRL1),
	RT_REG(RT9466_REG_CHG_CTRL2),
	RT_REG(RT9466_REG_CHG_CTRL3),
	RT_REG(RT9466_REG_CHG_CTRL4),
	RT_REG(RT9466_REG_CHG_CTRL5),
	RT_REG(RT9466_REG_CHG_CTRL6),
	RT_REG(RT9466_REG_CHG_CTRL7),
	RT_REG(RT9466_REG_CHG_CTRL8),
	RT_REG(RT9466_REG_CHG_CTRL9),
	RT_REG(RT9466_REG_CHG_CTRL10),
	RT_REG(RT9466_REG_CHG_CTRL11),
	RT_REG(RT9466_REG_CHG_CTRL12),
	RT_REG(RT9466_REG_CHG_CTRL13),
	RT_REG(RT9466_REG_CHG_CTRL14),
	RT_REG(RT9466_REG_CHG_CTRL15),
	RT_REG(RT9466_REG_CHG_CTRL16),
	RT_REG(RT9466_REG_CHG_ADC),
	RT_REG(RT9466_REG_CHG_CTRL17),
	RT_REG(RT9466_REG_CHG_CTRL18),
	RT_REG(RT9466_REG_DEVICE_ID),
	RT_REG(RT9466_REG_CHG_STAT),
	RT_REG(RT9466_REG_CHG_NTC),
	RT_REG(RT9466_REG_ADC_DATA_H),
	RT_REG(RT9466_REG_ADC_DATA_L),
	RT_REG(RT9466_REG_CHG_STATC),
	RT_REG(RT9466_REG_CHG_FAULT),
	RT_REG(RT9466_REG_TS_STATC),
	RT_REG(RT9466_REG_CHG_IRQ1),
	RT_REG(RT9466_REG_CHG_IRQ2),
	RT_REG(RT9466_REG_CHG_IRQ3),
	RT_REG(RT9466_REG_CHG_STATC_CTRL),
	RT_REG(RT9466_REG_CHG_FAULT_CTRL),
	RT_REG(RT9466_REG_TS_STATC_CTRL),
	RT_REG(RT9466_REG_CHG_IRQ1_CTRL),
	RT_REG(RT9466_REG_CHG_IRQ2_CTRL),
	RT_REG(RT9466_REG_CHG_IRQ3_CTRL),
};

static struct rt_regmap_properties rt9466_regmap_prop = {
	.name = "rt9466",
	.aliases = "rt9466",
	.register_num = RT9466_REG_IDX_MAX,
	.rm = rt9466_regmap_map,
	.rt_regmap_mode = RT_SINGLE_BYTE | RT_CACHE_DISABLE | RT_IO_PASS_THROUGH,
	.io_log_en = 0,
};
#endif /* CONFIG_RT_REGMAP */


struct rt9466_info {
	struct i2c_client *i2c;
	struct mutex access_lock;
	u8 device_id;
#ifdef CONFIG_RT_REGMAP
	struct rt_regmap_device *regmap_dev;
#endif
};

static struct rt9466_info g_rt9466_info = {
	.i2c = NULL,
	.device_id = RT9466_DEVICE_ID_E2,
#ifdef CONFIG_RT_REGMAP
	.regmap_dev = NULL,
#endif
};

/* ========================= */
/* I2C operations            */
/* ========================= */

static int rt9466_regmap_read(void *client, u32 addr, int leng, void *dst)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_read_i2c_block_data(i2c, addr, leng, dst);

	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: I2CR[0x%02X] failed\n",
			__func__, addr);
	else
		battery_log(BAT_LOG_FULL, "%s: I2CR[0x%02X] = 0x%02X\n",
			__func__, addr, *((u8 *)dst));

	return ret;
}

static int rt9466_regmap_write(void *client, u32 addr, int leng, const void *src)
{
	int ret = 0;
	struct i2c_client *i2c = NULL;

	i2c = (struct i2c_client *)client;
	ret = i2c_smbus_write_i2c_block_data(i2c, addr, leng, src);

	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: I2CW[0x%02X] = 0x%02X failed\n",
			__func__, addr, *((u8 *)src));
	else
		battery_log(BAT_LOG_FULL, "%s: I2CW[0x%02X] = 0x%02X\n",
			__func__, addr, *((u8 *)src));

	return ret;
}

#ifdef CONFIG_RT_REGMAP
static struct rt_regmap_fops rt9466_regmap_fops = {
	.read_device = rt9466_regmap_read,
	.write_device = rt9466_regmap_write,
};

static int rt9466_register_rt_regmap(void)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);
	g_rt9466_info.regmap_dev = rt_regmap_device_register(
		&rt9466_regmap_prop,
		&rt9466_regmap_fops,
		&(g_rt9466_info.i2c->dev),
		g_rt9466_info.i2c,
		&g_rt9466_info
	);

	if (!g_rt9466_info.regmap_dev) {
		dev_err(&g_rt9466_info.i2c->dev, "register regmap device failed\n");
		return -EIO;
	}

	battery_log(BAT_LOG_CRTI, "%s: ends\n", __func__);
	return ret;
}
#endif /* CONFIG_RT_REGMAP */

static int rt9466_i2c_write_byte(u8 cmd, u8 data)
{
	int ret = 0;

	ret = rt9466_regmap_write(g_rt9466_info.i2c, cmd, 1, &data);
	return ret;
}

static int rt9466_i2c_read_byte(u8 cmd)
{
	int ret = 0, ret_val = 0;

	ret = rt9466_regmap_read(g_rt9466_info.i2c, cmd, 1, &ret_val);

	if (ret < 0)
		return ret;

	ret_val = ret_val & 0xFF;

	return ret_val;
}

static int rt9466_i2c_test_bit(u8 cmd, u8 shift)
{
	int ret = 0;

	ret = rt9466_i2c_read_byte(cmd);
	if (ret < 0)
		return ret;

	ret = ret & (1 << shift);

	return ret;
}

static int rt9466_i2c_update_bits(u8 cmd, u8 data, u8 mask)
{
	int ret = 0;
	u8 reg_data = 0;

	mutex_lock(&g_rt9466_info.access_lock);
	ret = rt9466_i2c_read_byte(cmd);
	if (ret < 0) {
		mutex_unlock(&g_rt9466_info.access_lock);
		return ret;
	}

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);

	ret = rt9466_i2c_write_byte(cmd, reg_data);
	mutex_unlock(&g_rt9466_info.access_lock);

	return ret;
}

#define rt9466_set_bit(reg, mask) \
	rt9466_i2c_update_bits(reg, mask, mask)

#define rt9466_clr_bit(reg, mask) \
	rt9466_i2c_update_bits(reg, 0x00, mask)

/* ================== */
/* Internal Functions */
/* ================== */

/* The following APIs will be reference in internal functions */
static int rt_charger_set_ichg(void *data);
static int rt_charger_set_aicr(void *data);
static int rt_charger_set_mivr(void *data);
static int rt_charger_get_ichg(void *data);
static int rt_charger_get_aicr(void *data);
static int rt_charger_set_boost_current_limit(void *data);
static int rt_charger_enable_safety_timer(void *data);


static u8 rt9466_find_closest_reg_value(const u32 min, const u32 max,
	const u32 step, const u32 num, const u32 target)
{
	u32 i = 0, cur_val = 0, next_val = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	for (i = 0; i < num - 1; i++) {
		cur_val = min + i * step;
		next_val = cur_val + step;

		if (cur_val > max)
			cur_val = max;

		if (next_val > max)
			next_val = max;

		if (target >= cur_val && target < next_val)
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return num - 1;
}

static u8 rt9466_find_closest_reg_value_via_table(const u32 *value_table,
	const u32 table_size, const u32 target_value)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target_value < value_table[0])
		return 0;

	for (i = 0; i < table_size - 1; i++) {
		if (target_value >= value_table[i] &&
		    target_value < value_table[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return table_size - 1;
}

static u32 rt9466_find_closest_real_value(const u32 min, const u32 max,
	const u32 step, const u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static bool rt9466_is_hw_exist(void)
{
	int ret = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_DEVICE_ID);
	if (ret < 0)
		return false;

	if ((ret & 0xFF) == RT9466_DEVICE_ID_E2) {
		battery_log(BAT_LOG_CRTI, "%s: E2 revision\n", __func__);
		g_rt9466_info.device_id = RT9466_DEVICE_ID_E2;
		return true;
	}

	if ((ret & 0xFF) >= RT9466_DEVICE_ID_E3) {
		battery_log(BAT_LOG_CRTI, "%s: 0x%02X revision\n",
			__func__, (ret & 0xFF));
		g_rt9466_info.device_id = 0; /* Stands for higher than E3 */
		return true;
	}

	return false;
}

static int rt9466_set_fast_charge_timer(u32 hour)
{
	int ret = 0;
	u8 reg_fct = 0;

	battery_log(BAT_LOG_CRTI, "%s: set fast charge timer to %d\n",
		__func__, hour);

	reg_fct = rt9466_find_closest_reg_value(RT9466_WT_FC_MIN, RT9466_WT_FC_MAX,
		RT9466_WT_FC_STEP, RT9466_WT_FC_NUM, hour);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL12,
		reg_fct << RT9466_SHIFT_WT_FC,
		RT9466_MASK_WT_FC
	);

	return ret;
}


static int rt9466_enable_watchdog_timer(bool enable)
{
	int ret = 0;

	ret = rt9466_set_bit(RT9466_REG_CHG_CTRL13, RT9466_MASK_WDT_EN);
	return ret;
}

static int rt9466_set_ircmp_resistor(u32 resistor)
{
	int ret = 0;
	u8 reg_resistor = 0;

	battery_log(BAT_LOG_CRTI, "%s: set ir comp resistor = %d\n",
		__func__, resistor);

	reg_resistor = rt9466_find_closest_reg_value(RT9466_IRCMP_RES_MIN,
		RT9466_IRCMP_RES_MAX, RT9466_IRCMP_RES_STEP, RT9466_IRCMP_RES_NUM,
		resistor);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL18,
		reg_resistor << RT9466_SHIFT_IRCMP_RES,
		RT9466_MASK_IRCMP_RES
	);

	return ret;
}

static int rt9466_set_ircmp_vclamp(u32 vclamp)
{
	int ret = 0;
	u8 reg_vclamp = 0;

	battery_log(BAT_LOG_CRTI, "%s: set ir comp vclamp = %d\n",
		__func__, vclamp);

	reg_vclamp = rt9466_find_closest_reg_value(RT9466_IRCMP_VCLAMP_MIN,
		RT9466_IRCMP_VCLAMP_MAX, RT9466_IRCMP_VCLAMP_STEP,
		RT9466_IRCMP_VCLAMP_NUM, vclamp);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL18,
		reg_vclamp << RT9466_SHIFT_IRCMP_VCLAMP,
		RT9466_MASK_IRCMP_VCLAMP
	);

	return ret;
}

/* Hardware pin current limit */
static int rt9466_enable_ilim(u8 enable)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: enable ilim = %d\n", __func__, enable);

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL3, RT9466_MASK_ILIM_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL3, RT9466_MASK_ILIM_EN));

	return ret;
}

/* Select IINLMTSEL to use AICR */
static int rt9466_select_input_current_limit(enum rt9466_iin_limit_sel sel)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: select input current limit = %d\n",
		__func__, sel);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL2,
		sel << RT9466_SHIFT_IINLMTSEL,
		RT9466_MASK_IINLMTSEL);

	return ret;
}

/* Enter hidden mode */
static int rt9466_enter_hidden_mode(void)
{
	int ret = 0, i = 0;

	battery_log(BAT_LOG_CRTI, "%s: enter hidden mode\n", __func__);

	for (i = 0; i < RT9466_REG_EN_HIDDEN_MODE_MAX; i++) {
		ret = rt9466_i2c_write_byte(
			rt9466_reg_en_hidden_mode[i],
			rt9466_val_en_hidden_mode[i]
		);
		if (ret < 0) {
			battery_log(BAT_LOG_CRTI,
				"%s: enter hidden mode failed\n", __func__);
			return ret;
		}
	}

	return ret;
}

/* Software workaround */
static int rt9466_sw_workaround(void)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	/* Enter hidden mode */
	ret = rt9466_enter_hidden_mode();
	if (ret < 0)
		return ret;

	/* ICC: modify sensing node, make it more accurate */
	ret = rt9466_i2c_write_byte(0x27, 0x00);
	if (ret < 0)
		return ret;

	/* DIMIN level */
	ret = rt9466_i2c_write_byte(0x28, 0x86);

	return ret;
}

static int rt9466_get_adc(enum rt9466_adc_sel adc_sel, int *adc_val)
{
	int ret = 0, i = 0;
	const int wait_retry = 5;
	u8 adc_h = 0, adc_l = 0;

	/* Select ADC to temperature channel*/
	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_ADC,
		adc_sel << RT9466_SHIFT_ADC_IN_SEL,
		RT9466_MASK_ADC_IN_SEL
	);

	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "%s: select ADC to %d failed\n",
			__func__, adc_sel);
		return ret;
	}

	/* Start ADC conversation */
	ret = rt9466_set_bit(RT9466_REG_CHG_ADC, RT9466_MASK_ADC_START);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "%s: start ADC conversation failed\n",
			__func__);
		return ret;
	}

	/* Wait for ADC conversation */
	for (i = 0; i < wait_retry; i++) {
		mdelay(35);
		if (rt9466_i2c_test_bit(RT9466_REG_CHG_IRQ3, RT9466_SHIFT_ADC_DONEI) > 0) {
			battery_log(BAT_LOG_FULL,
				"%s: sel = %d, wait_times = %d\n",
				__func__, adc_sel, i);
			break;
		}
		if ((i + 1) == wait_retry) {
			battery_log(BAT_LOG_CRTI,
				"%s: Wait ADC conversation failed\n", __func__);
			return -EINVAL;
		}
	}

	/* Read ADC data high byte */
	ret = rt9466_i2c_read_byte(RT9466_REG_ADC_DATA_H);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI,
			"%s: read ADC high byte data failed\n", __func__);
		return ret;
	}
	adc_h = ret & 0xFF;

	/* Read ADC data low byte */
	ret = rt9466_i2c_read_byte(RT9466_REG_ADC_DATA_L);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI,
			"%s: read ADC low byte data failed\n", __func__);
		return ret;
	}
	adc_l = ret & 0xFF;
	battery_log(BAT_LOG_FULL,
		"%s: adc_sel = %d, adc_h = 0x%02X, adc_l = 0x%02X\n",
		__func__, adc_sel, adc_h, adc_l);

	/* Calculate ADC value */
	*adc_val = ((adc_h * 256 + adc_l) * rt9466_adc_unit[adc_sel])
		+ rt9466_adc_offset[adc_sel];

	return ret;
}

static int rt9466_hw_init(void)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	ret = rt9466_set_bit(RT9466_REG_CORE_CTRL0, RT9466_MASK_RST);

	return ret;
}

static int rt9466_enable_te(const u8 enable)
{
	int ret = 0;

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL2, RT9466_MASK_TE_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL2, RT9466_MASK_TE_EN));

	return ret;
}

static int rt9466_set_ieoc(const u32 ieoc)
{
	int ret = 0;

	/* Find corresponding reg value */
	u8 reg_ieoc = rt9466_find_closest_reg_value(RT9466_IEOC_MIN,
		RT9466_IEOC_MAX, RT9466_IEOC_STEP, RT9466_IEOC_NUM, ieoc);

	battery_log(BAT_LOG_CRTI, "%s: ieoc = %d\n", __func__, ieoc);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL9,
		reg_ieoc << RT9466_SHIFT_IEOC,
		RT9466_MASK_IEOC
	);

	return ret;
}

static int rt9466_get_mivr(u32 *mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_CHG_CTRL6);
	if (ret < 0)
		return ret;
	reg_mivr = ((ret & RT9466_MASK_MIVR) >> RT9466_SHIFT_MIVR) & 0xFF;

	*mivr = rt9466_find_closest_real_value(RT9466_MIVR_MIN, RT9466_MIVR_MAX,
		RT9466_MIVR_STEP, reg_mivr);


	return ret;
}


static int rt9466_sw_init(void)
{
	int ret = 0;
	u32 ichg = 200000; /* 10uA */
	u32 aicr = 50000; /* 10uA */
	u32 mivr = 4400; /* mV */
	u8 enable_safety_timer = 1;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	/* Disable hardware ILIM */
	ret = rt9466_enable_ilim(0);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: disable ilim failed\n", __func__);

	/* Select IINLMTSEL to use AICR */
	ret = rt9466_select_input_current_limit(RT9466_IINLMTSEL_AICR);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: select iinlmtsel failed\n",
			__func__);

	ret = rt_charger_set_ichg(&ichg);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set ichg failed\n", __func__);

	ret = rt_charger_set_aicr(&aicr);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set aicr failed\n", __func__);

	ret = rt_charger_set_mivr(&mivr);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set mivr failed\n", __func__);

	ret = rt9466_set_ieoc(250);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set ieoc failed\n", __func__);

	ret = rt9466_enable_te(1);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set te failed\n", __func__);

	/* Set fast charge timer to 12 hours */
	ret = rt9466_set_fast_charge_timer(12);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set fast timer failed\n",
			__func__);

	ret = rt_charger_enable_safety_timer(&enable_safety_timer);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: enable charger timer failed\n",
			__func__);

	ret = rt9466_enable_watchdog_timer(true);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: enable watchdog failed\n",
			__func__);

	/* Set ircomp according to BIF */
#ifdef CONFIG_MTK_BIF_SUPPORT
	ret = rt9466_set_ircmp_resistor(0);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,
			"%s: set IR compensation resistor failed\n", __func__);

	ret = rt9466_set_ircmp_vclamp(0);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,
			"%s: set IR compensation voltage clamp failed\n",
			__func__);
#else
	ret = rt9466_set_ircmp_resistor(80);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,
			"%s: set IR compensation resistor failed\n", __func__);

	ret = rt9466_set_ircmp_vclamp(224);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,
			"%s: set IR compensation voltage clamp failed\n",
			__func__);
#endif
	return ret;
}

static int rt9466_get_charging_status(enum rt9466_charging_status *chg_stat)
{
	int ret = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_CHG_STAT);
	if (ret < 0)
		return ret;

	*chg_stat = (ret & RT9466_MASK_CHG_STAT) >> RT9466_SHIFT_CHG_STAT;

	return ret;
}

static int rt9466_get_ieoc(u32 *ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_CHG_CTRL9);
	if (ret < 0)
		return ret;

	reg_ieoc = (ret & RT9466_MASK_IEOC) >> RT9466_SHIFT_IEOC;
	*ieoc = rt9466_find_closest_real_value(RT9466_IEOC_MIN, RT9466_IEOC_MAX,
		RT9466_IEOC_STEP, reg_ieoc);

	return ret;
}

static int rt9466_is_charging_enable(u8 *enable)
{
	int ret = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_CHG_CTRL2);
	if (ret < 0)
		return ret;

	*enable = ((ret & RT9466_MASK_CHG_EN) >> RT9466_SHIFT_CHG_EN) & 0xFF;

	return ret;
}

static int rt9466_enable_pump_express(u8 enable)
{
	int ret = 0, i = 0, retry_times = 5;

	battery_log(BAT_LOG_CRTI, "%s: enable pumpX = %d\n", __func__, enable);

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_EN));

	if (ret < 0)
		return ret;

	for (i = 0; i < retry_times; i++) {
		if (rt9466_i2c_test_bit(RT9466_REG_CHG_CTRL17, RT9466_SHIFT_PUMPX_EN) == 0)
			break;
		msleep(2500);
		if ((i + 1) == retry_times) {
			battery_log(BAT_LOG_CRTI, "%s: pumpX wait failed\n",
				__func__);
			return -EIO;
		}
	}

	return ret;
}

/* =========================================================== */
/* The following is implementation for interface of rt_charger */
/* =========================================================== */

/* Set register's value to default */
static int rt_charger_hw_init(void *data)
{
	return 0;
}

static int rt_charger_dump_register(void *data)
{
#ifndef CONFIG_RT_REGMAP
	int i = 0;
#endif
	int ret = 0;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0;
	u8 chg_enable = 0;
	int adc_vsys = 0, adc_vbat = 0, adc_ibat = 0;
	enum rt9466_charging_status chg_status = RT9466_CHG_STATUS_READY;

#ifndef CONFIG_RT_REGMAP
	for (i = 0; i < RT9466_REG_IDX_MAX; i++) {
		ret = rt9466_i2c_read_byte(rt9466_reg_addr[i]);
		if (ret < 0)
			return ret;
	}
#endif

	ret = rt_charger_get_ichg(&ichg);
	ret = rt_charger_get_aicr(&aicr);
	ret = rt9466_get_mivr(&mivr);
	ret = rt9466_is_charging_enable(&chg_enable);
	ret = rt9466_get_ieoc(&ieoc);
	ret = rt9466_get_charging_status(&chg_status);
	ret = rt9466_get_adc(RT9466_ADC_VSYS, &adc_vsys);
	ret = rt9466_get_adc(RT9466_ADC_VBAT, &adc_vbat);
	ret = rt9466_get_adc(RT9466_ADC_IBAT, &adc_ibat);

	battery_log(BAT_LOG_CRTI,
		"%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA\n",
		__func__, ichg / 100, aicr / 100, mivr, ieoc);

	battery_log(BAT_LOG_CRTI,
		"%s: VSYS = %dmV, VBAT = %dmV, IBAT = %dmA\n",
		__func__, adc_vsys, adc_vbat, adc_ibat);

	battery_log(BAT_LOG_CRTI,
		"%s: CHG_EN = %d, CHG_STATUS = %s\n",
		__func__, chg_enable, rt9466_chg_status_name[chg_status]);

	return ret;
}

static int rt_charger_enable_charging(void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL2, RT9466_MASK_CHG_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL2, RT9466_MASK_CHG_EN));

	return ret;
}

static int rt_charger_enable_hz(void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL1, RT9466_MASK_HZ_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL1, RT9466_MASK_CHG_EN));

	return ret;
}

static int rt_charger_enable_safety_timer(void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL12, RT9466_MASK_TMR_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL12, RT9466_MASK_TMR_EN));

	return ret;
}

static int rt_charger_enable_otg(void *data)
{
	int ret = 0, i = 0;
	const int ss_wait_times = 5; /* soft start wait times */
	u8 enable = *((u8 *)data);
	u32 current_limit = 500;

	/* Set OTG_OC to 500mA */
	ret = rt_charger_set_boost_current_limit(&current_limit);
	if (ret < 0)
		return ret;

	/* Switch OPA mode to boost mode */
	battery_log(BAT_LOG_CRTI, "%s: otg enable = %d\n", __func__, enable);
	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL1, RT9466_MASK_OPA_MODE)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL1, RT9466_MASK_OPA_MODE));

	if (!enable) {
		/* Workaround: reg[0x25] = 0x0F after leaving OTG mode */
		ret = rt9466_i2c_write_byte(0x25, 0x0F);
		return ret;
	}

	/* Wait for soft start finish interrupt */
	for (i = 0; i < ss_wait_times; i++) {
		ret = rt9466_i2c_test_bit(RT9466_REG_CHG_IRQ2,
			RT9466_SHIFT_SSFINISHI);
		if (ret > 0) {
			/* Woraround reg[0x25] = 0x00 after entering OTG mode */
			ret = rt9466_i2c_write_byte(0x25, 0x00);
			battery_log(BAT_LOG_FULL,
				"%s: otg soft start successfully\n", __func__);
			break;
		}
		mdelay(100);
		if ((i + 1) == ss_wait_times)
			battery_log(BAT_LOG_CRTI,
				"%s: otg soft start failed\n", __func__);
	}

	return ret;
}

static int rt_charger_enable_power_path(void *data)
{
	int ret = 0;
	u8 enable = *((u8 *)data);

	ret = (enable ? rt9466_set_bit(RT9466_REG_CHG_CTRL2, RT9466_MASK_CFO_EN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL2, RT9466_MASK_CFO_EN));

	return ret;
}

static int rt_charger_set_ichg(void *data)
{
	int ret = 0;
	u8 reg_ichg = 0;
	u32 ichg = *((u32 *)data);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	ichg /= 100;

	/* Find corresponding reg value */
	reg_ichg = rt9466_find_closest_reg_value(RT9466_ICHG_MIN,
		RT9466_ICHG_MAX, RT9466_ICHG_STEP, RT9466_ICHG_NUM, ichg);

	battery_log(BAT_LOG_CRTI, "%s: ichg = %d\n", __func__, ichg);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL7,
		reg_ichg << RT9466_SHIFT_ICHG,
		RT9466_MASK_ICHG
	);

	return ret;
}

static int rt_charger_set_aicr(void *data)
{
	int ret = 0;
	u8 reg_aicr = 0;
	u32 aicr = *((u32 *)data);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	aicr /= 100;

	/* Find corresponding reg value */
	reg_aicr = rt9466_find_closest_reg_value(RT9466_AICR_MIN,
		RT9466_AICR_MAX, RT9466_AICR_STEP, RT9466_AICR_NUM, aicr);

	battery_log(BAT_LOG_CRTI, "%s: aicr = %d\n", __func__, aicr);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL3,
		reg_aicr << RT9466_SHIFT_AICR,
		RT9466_MASK_AICR
	);

	return ret;
}

static int rt_charger_set_mivr(void *data)
{
	int ret = 0;
	u8 reg_mivr = 0;
	u32 mivr = *((u32 *)data);

	/* Find corresponding reg value */
	reg_mivr = rt9466_find_closest_reg_value(RT9466_MIVR_MIN,
		RT9466_MIVR_MAX, RT9466_MIVR_STEP, RT9466_MIVR_NUM, mivr);

	battery_log(BAT_LOG_CRTI, "%s: mivr = %d\n", __func__, mivr);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL6,
		reg_mivr << RT9466_SHIFT_MIVR,
		RT9466_MASK_MIVR
	);

	return ret;
}

static int rt_charger_set_battery_voreg(void *data)
{
	int ret = 0;
	u8 reg_voreg = 0;
	u32 voreg = *((u32 *)data);

	/* MTK's voltage unit : uV */
	/* Our voltage unit : mV */
	voreg /= 1000;

	reg_voreg = rt9466_find_closest_reg_value(RT9466_BAT_VOREG_MIN,
		RT9466_BAT_VOREG_MAX, RT9466_BAT_VOREG_STEP,
		RT9466_BAT_VOREG_NUM, voreg);

	battery_log(BAT_LOG_CRTI, "%s: bat voreg = %d\n", __func__, voreg);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL4,
		reg_voreg << RT9466_SHIFT_BAT_VOREG,
		RT9466_MASK_BAT_VOREG
	);

	return ret;
}

static int rt_charger_set_boost_current_limit(void *data)
{
	int ret = 0;
	u8 reg_ilimit = 0;
	u32 current_limit = *((u32 *)data);

	reg_ilimit = rt9466_find_closest_reg_value_via_table(
		rt9466_boost_oc_threshold,
		RT9466_BOOST_OC_THRESHOLD_NUM,
		current_limit
	);

	battery_log(BAT_LOG_CRTI, "%s: boost ilimit = %d\n",
		__func__, current_limit);

	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL10,
		reg_ilimit << RT9466_SHIFT_BOOST_OC,
		RT9466_MASK_BOOST_OC
	);

	return ret;
}

static int rt_charger_set_pep_current_pattern(void *data)
{
	int ret = 0;
	u8 is_pump_up = *((u8 *)data);
	u32 ichg = 200000; /* 10uA */
	u32 aicr = 80000; /* 10uA */

	battery_log(BAT_LOG_CRTI, "%s: pe1.0 pump_up = %d\n",
		__func__, is_pump_up);

	/* Set to PE1.0 */
	ret = rt9466_clr_bit(RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_20_10);

	/* Set Pump Up/Down */
	ret = (is_pump_up ?
		rt9466_set_bit(RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_UP_DN)
		: rt9466_clr_bit(RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_UP_DN));

	ret = rt_charger_set_aicr(&aicr);
	if (ret < 0)
		return ret;

	ret = rt_charger_set_ichg(&ichg);
	if (ret < 0)
		return ret;

	/* Enable PumpX */
	ret = rt9466_enable_pump_express(1);

	return ret;
}

static int rt_charger_set_pep20_reset(void *data)
{
	int ret = 0;
	u32 mivr = 4500; /* mA */
	u32 ichg = 51200; /* 10uA */
	u32 aicr = 10000; /* 10uA */

	ret = rt_charger_set_mivr(&mivr);
	if (ret < 0)
		return ret;

	ret = rt_charger_set_ichg(&ichg);
	if (ret < 0)
		return ret;

	ret = rt_charger_set_aicr(&aicr);
	if (ret < 0)
		return ret;

	msleep(200);

	aicr = 70000;
	ret = rt_charger_set_aicr(&aicr);
	return ret;
}

int rt_charger_set_pep20_current_pattern(void *data)
{
	int ret = 0;
	u8 reg_volt = 0;
	u32 voltage = *((u32 *)data);

	battery_log(BAT_LOG_CRTI, "%s: pep2.0  = %d\n", __func__, voltage);

	/* Find register value of target voltage */
	reg_volt = rt9466_find_closest_reg_value(RT9466_PEP20_VOLT_MIN,
		RT9466_PEP20_VOLT_MAX, RT9466_PEP20_VOLT_STEP,
		RT9466_PEP20_VOLT_NUM, voltage);

	/* Set to PEP2.0 */
	ret = rt9466_set_bit(RT9466_REG_CHG_CTRL17, RT9466_MASK_PUMPX_20_10);
	if (ret < 0)
		return ret;

	/* Set Voltage */
	ret = rt9466_i2c_update_bits(
		RT9466_REG_CHG_CTRL17,
		reg_volt << RT9466_SHIFT_PUMPX_DEC,
		RT9466_MASK_PUMPX_DEC);
	if (ret < 0)
		return ret;

	/* Enable PumpX */
	ret = rt9466_enable_pump_express(1);

	return ret;
}

static int rt_charger_get_ichg(void *data)
{
	int ret = 0;
	u8 reg_ichg = 0;
	u32 ichg = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_CHG_CTRL7);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & RT9466_MASK_ICHG) >> RT9466_SHIFT_ICHG;
	ichg = rt9466_find_closest_real_value(RT9466_ICHG_MIN, RT9466_ICHG_MAX,
		RT9466_ICHG_STEP, reg_ichg);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	ichg *= 100;
	*((u32 *)data) = ichg;

	return ret;
}

static int rt_charger_get_aicr(void *data)
{
	int ret = 0;
	u8 reg_aicr = 0;
	u32 aicr = 0;

	ret = rt9466_i2c_read_byte(RT9466_REG_CHG_CTRL3);
	if (ret < 0)
		return ret;

	reg_aicr = (ret & RT9466_MASK_AICR) >> RT9466_SHIFT_AICR;
	aicr = rt9466_find_closest_real_value(RT9466_AICR_MIN, RT9466_AICR_MAX,
		RT9466_AICR_STEP, reg_aicr);

	/* MTK's current unit : 10uA */
	/* Our current unit : mA */
	aicr *= 100;
	*((u32 *)data) = aicr;

	return ret;
}

static int rt_charger_get_temperature(void *data)
{
	int ret = 0, adc_temp = 0;

	/* Get value from ADC */
	ret = rt9466_get_adc(RT9466_ADC_TEMP_JC, &adc_temp);
	if (ret < 0)
		return ret;

	((int *)data)[0] = adc_temp;
	((int *)data)[1] = adc_temp;

	battery_log(BAT_LOG_CRTI, "%s: temperature = %d\n", __func__, adc_temp);
	return ret;
}

static int rt_charger_is_charging_done(void *data)
{
	int ret = 0;
	enum rt9466_charging_status chg_stat = RT9466_CHG_STATUS_READY;

	ret = rt9466_get_charging_status(&chg_stat);

	/* Return is charging done or not */
	switch (chg_stat) {
	case RT9466_CHG_STATUS_READY:
	case RT9466_CHG_STATUS_PROGRESS:
	case RT9466_CHG_STATUS_FAULT:
		*((u8 *)data) = 0;
		break;
	case RT9466_CHG_STATUS_DONE:
		*((u8 *)data) = 1;
		break;
	default:
		*((u8 *)data) = 0;
		break;
	}

	return ret;
}


static int rt_charger_reset_watchdog_timer(void *data)
{
	int ret = 0;
	enum rt9466_charging_status chg_status = RT9466_CHG_STATUS_READY;

	/* Any I2C communication can reset watchdog timer */
	ret = rt9466_get_charging_status(&chg_status);

	return ret;
}

static void rt9466_charger_intf_init(void)
{
	mtk_charger_intf[CHARGING_CMD_INIT] = rt_charger_hw_init;
	mtk_charger_intf[CHARGING_CMD_DUMP_REGISTER] = rt_charger_dump_register;
	mtk_charger_intf[CHARGING_CMD_ENABLE] = rt_charger_enable_charging;
	mtk_charger_intf[CHARGING_CMD_SET_HIZ_SWCHR] = rt_charger_enable_hz;
	mtk_charger_intf[CHARGING_CMD_ENABLE_SAFETY_TIMER] = rt_charger_enable_safety_timer;
	mtk_charger_intf[CHARGING_CMD_ENABLE_OTG] = rt_charger_enable_otg;
	mtk_charger_intf[CHARGING_CMD_ENABLE_POWER_PATH] = rt_charger_enable_power_path;
	mtk_charger_intf[CHARGING_CMD_SET_CURRENT] = rt_charger_set_ichg;
	mtk_charger_intf[CHARGING_CMD_SET_INPUT_CURRENT] = rt_charger_set_aicr;
	mtk_charger_intf[CHARGING_CMD_SET_VINDPM] = rt_charger_set_mivr;
	mtk_charger_intf[CHARGING_CMD_SET_CV_VOLTAGE] = rt_charger_set_battery_voreg;
	mtk_charger_intf[CHARGING_CMD_SET_BOOST_CURRENT_LIMIT] = rt_charger_set_boost_current_limit;
	mtk_charger_intf[CHARGING_CMD_SET_TA_CURRENT_PATTERN] = rt_charger_set_pep_current_pattern;
	mtk_charger_intf[CHARGING_CMD_SET_TA20_RESET] = rt_charger_set_pep20_reset;
	mtk_charger_intf[CHARGING_CMD_SET_TA20_CURRENT_PATTERN] = rt_charger_set_pep20_current_pattern;
	mtk_charger_intf[CHARGING_CMD_GET_CURRENT] = rt_charger_get_ichg;
	mtk_charger_intf[CHARGING_CMD_GET_CHARGER_TEMPERATURE] = rt_charger_get_temperature;
	mtk_charger_intf[CHARGING_CMD_GET_CHARGING_STATUS] = rt_charger_is_charging_done;
	mtk_charger_intf[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = rt_charger_reset_watchdog_timer;

	/*
	 * The following interfaces are not related to charger
	 * Define in mtk_charger_intf.c
	 */
	mtk_charger_intf[CHARGING_CMD_SW_INIT] = mtk_charger_sw_init;
	mtk_charger_intf[CHARGING_CMD_SET_HV_THRESHOLD] = mtk_charger_set_hv_threshold;
	mtk_charger_intf[CHARGING_CMD_GET_HV_STATUS] = mtk_charger_get_hv_status;
	mtk_charger_intf[CHARGING_CMD_GET_BATTERY_STATUS] = mtk_charger_get_battery_status;
	mtk_charger_intf[CHARGING_CMD_GET_CHARGER_DET_STATUS] = mtk_charger_get_charger_det_status;
	mtk_charger_intf[CHARGING_CMD_GET_CHARGER_TYPE] = mtk_charger_get_charger_type;
	mtk_charger_intf[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] = mtk_charger_get_is_pcm_timer_trigger;
	mtk_charger_intf[CHARGING_CMD_SET_PLATFORM_RESET] = mtk_charger_set_platform_reset;
	mtk_charger_intf[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] = mtk_charger_get_platform_boot_mode;
	mtk_charger_intf[CHARGING_CMD_SET_POWER_OFF] = mtk_charger_set_power_off;
	mtk_charger_intf[CHARGING_CMD_GET_POWER_SOURCE] = mtk_charger_get_power_source;
	mtk_charger_intf[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = mtk_charger_get_csdac_full_flag;
	mtk_charger_intf[CHARGING_CMD_SET_ERROR_STATE] = mtk_charger_set_error_state;
	mtk_charger_intf[CHARGING_CMD_DISO_INIT] = mtk_charger_diso_init;
	mtk_charger_intf[CHARGING_CMD_GET_DISO_STATE] = mtk_charger_get_diso_state;
	mtk_charger_intf[CHARGING_CMD_SET_VBUS_OVP_EN] = mtk_charger_set_vbus_ovp_en;
	mtk_charger_intf[CHARGING_CMD_GET_BIF_VBAT] = mtk_charger_get_bif_vbat;
	mtk_charger_intf[CHARGING_CMD_SET_CHRIND_CK_PDN] = mtk_charger_set_chrind_ck_pdn;
	mtk_charger_intf[CHARGING_CMD_GET_BIF_TBAT] = mtk_charger_get_bif_tbat;
	mtk_charger_intf[CHARGING_CMD_SET_DP] = mtk_charger_set_dp;
}

/* ========================= */
/* I2C driver function       */
/* ========================= */

static int rt9466_probe(struct i2c_client *i2c,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

	g_rt9466_info.i2c = i2c;
	mutex_init(&g_rt9466_info.access_lock);

	/* Is HW exist */
	if (!rt9466_is_hw_exist()) {
		battery_log(BAT_LOG_CRTI, "%s: no rt9466 exists\n", __func__);
		BUG_ON(1);
		return -ENODEV;
	}

#ifdef CONFIG_RT_REGMAP
	ret = rt9466_register_rt_regmap();
	if (ret < 0)
		return ret;
#endif

	/* Reset chip */
	ret = rt9466_hw_init();
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,
			"%s: set register to default value failed\n", __func__);

	ret = rt9466_sw_init();
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: set failed\n", __func__);

	/* Only need this workaround for E2 revision */
	if (g_rt9466_info.device_id == RT9466_DEVICE_ID_E2) {
		ret = rt9466_sw_workaround();
		if (ret < 0)
			battery_log(BAT_LOG_CRTI,
				"%s: software workaround failed\n", __func__);
	}

	rt_charger_dump_register(NULL);

	/* Hook chr_control_interface with battery's interface */
	rt9466_charger_intf_init();
	battery_charging_control = chr_control_interface;
	chargin_hw_init_done = KAL_TRUE;
	battery_log(BAT_LOG_CRTI, "%s: ends\n", __func__);

	return ret;
}


static int rt9466_remove(struct i2c_client *client)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);

#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(g_rt9466_info.regmap_dev);
#endif
	return ret;
}

static void rt9466_shutdown(struct i2c_client *client)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "%s: starts\n", __func__);
	ret = rt9466_hw_init();
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: reset chip failed\n", __func__);
}

static const struct i2c_device_id rt9466_i2c_id[] = {
	{"rt9466", 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id rt9466_of_match[] = {
	{ .compatible = "mediatek,swithing_charger", },
	{},
};
#else /* Not define CONFIG_OF */

#define RT9466_BUSNUM 1

static struct i2c_board_info rt9466_i2c_board_info __initdata = {
	I2C_BOARD_INFO("rt9466", RT9466_SALVE_ADDR)
};
#endif /* CONFIG_OF */


static struct i2c_driver rt9466_i2c_driver = {
	.driver = {
		.name = "rt9466",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt9466_of_match,
#endif
	},
	.probe = rt9466_probe,
	.remove = rt9466_remove,
	.shutdown = rt9466_shutdown,
	.id_table = rt9466_i2c_id,
};

static int __init rt9466_init(void)
{
	int ret = 0;

#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "%s: with dts\n", __func__);
#else
	battery_log(BAT_LOG_CRTI, "%s: without dts\n", __func__);
	i2c_register_board_info(RT9466_BUSNUM, &rt9466_i2c_board_info, 1);
#endif

	ret = i2c_add_driver(&rt9466_i2c_driver);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "%s: register i2c driver failed\n",
			__func__);

	return ret;
}
module_init(rt9466_init);


static void __exit rt9466_exit(void)
{
	i2c_del_driver(&rt9466_i2c_driver);
}
module_exit(rt9466_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("RT9466 Charger Driver");
MODULE_VERSION("0.0.6_MTK");
