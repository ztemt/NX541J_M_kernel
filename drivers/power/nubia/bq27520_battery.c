/*
 * bq27520 fg chip driver
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/rtc.h>
//#include <mach/mt_gpio.h>
//#include <mach/eint.h>
//#include <mach/memory.h>
//#include <cust_gpio_usage.h>
//#include <cust_eint.h>
#ifdef CONFIG_NUBIA_GENERAL_CHARGE_COMMON
#include <linux/power/general_charger.h>
#include <linux/power/general_battery.h>
#endif

#define BQ27520_FG_BQ27520_UPDATER

#ifdef BQ27520_FG_BQ27520_UPDATER
#include "bqfs_images_4000mah.h"
#endif


/* Bq27520 standard data commands */
#define BQ27520_REG_CNTL		0x00
#define BQ27520_REG_AR			0x02
#define BQ27520_REG_ARTTE		0x04
#define BQ27520_REG_TEMP		0x06
#define BQ27520_REG_VOLT		0x08
#define BQ27520_REG_FLAGS		0x0A
#define BQ27520_REG_NAC			0x0C
#define BQ27520_REG_FAC			0x0e
#define BQ27520_REG_RM			0x10
#define BQ27520_REG_FCC			0x12
#define BQ27520_REG_AI			0x14
#define BQ27520_REG_TTE			0x16
#define BQ27520_REG_SI			0x18
#define BQ27520_REG_STTE	    0x1a
#define BQ27520_REG_SOH		    0x1c
#define BQ27520_REG_CC			0x1e
#define BQ27520_REG_SOC		    0x20
#define BQ27520_REG_INSTC	    0x22
#define BQ27520_REG_INTTEMP		0x28
#define BQ27520_REG_RSCL	    0x2a
#define BQ27520_REG_OPCFIG		0x2c
#define BQ27520_REG_DESCAP		0x2e
#define BQ27520_REG_UFRM		0x6c
#define BQ27520_REG_FRM 		0x6e
#define BQ27520_REG_DFFCC		0x70
#define BQ27520_FLAG_FFCC		0x72
#define BQ27520_FLAG_UFSOC		0x74

#define BQ27520_REG_DFCLASS         0x3E
#define BQ27520_REG_DFBLOCK         0x3F
#define BQ27520_REG_BLOCKDATA       0x40

#define BQ27520_REG_BLKCHKSUM       0x60
#define BQ27520_REG_BLKDATCTL       0x61

/* Control subcommands */
#define BQ27520_SUBCMD_CTNL_STATUS  0x0000
#define BQ27520_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27520_SUBCMD_FW_VER       0x0002
#define BQ27520_SUBCMD_HW_VER       0x0003
#define BQ27520_SUBCMD_DF_CSUM      0x0004
#define BQ27520_SUBCMD_PREV_MACW    0x0007
#define BQ27520_SUBCMD_CHEM_ID      0x0008
#define BQ27520_SUBCMD_BD_OFFSET    0x0009
#define BQ27520_SUBCMD_INT_OFFSET   0x000a
#define BQ27520_SUBCMD_CC_VER       0x000b
#define BQ27520_SUBCMD_OCV          0x000c
#define BQ27520_SUBCMD_BAT_INS      0x000d
#define BQ27520_SUBCMD_BAT_REM      0x000e
#define BQ27520_SUBCMD_SET_HIB      0x0011
#define BQ27520_SUBCMD_CLR_HIB      0x0012
#define BQ27520_SUBCMD_SET_SLP      0x0013
#define BQ27520_SUBCMD_CLR_SLP      0x0014
#define BQ27520_SUBCMD_FCT_RES      0x0015
#define BQ27520_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27520_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27520_SUBCMD_SEALED       0x0020
#define BQ27520_SUBCMD_ENABLE_IT    0x0021
#define BQ27520_SUBCMD_DISABLE_IT   0x0023
#define BQ27520_SUBCMD_CAL_MODE     0x0040
#define BQ27520_SUBCMD_RESET        0x0041
#define BQ27520_SUBCMD_ENTER_ROM    0x0F00

#define BQ27520_SECURITY_SEALED     0x03
#define BQ27520_SECURITY_UNSEALED   0x02
#define BQ27520_SECURITY_FA         0x01
#define BQ27520_SECURITY_MASK       0x03

#define BQ27520_UNSEAL_KEY          0x36720414
#define BQ27520_FA_KEY              0xFFFFFFFF

#define I2C_RETRY_CNT    5
#define BQGAUGE_I2C_ROM_ADDR    (0x16 >> 1)
#define BQGAUGE_I2C_DEV_ADDR    (0xAA >> 1)

enum battery_type_id
{
    BATT_ID_SDI = 470,  //Samsung
    BATT_ID_ATL = 15,   //ATL
    BATT_ID_UNKOWN = 0,
};

struct bq27520_device{
    struct device		    *dev;
    struct mutex            read_mutex;
    struct i2c_client       *client;

    char                    *dma_buff;
    char                    *batt_type;
	char                    *fw_version;

	const bqfs_cmd_t        *bqfs_image;
	int                     bqfs_image_size;
	int                     batt_id;

    struct power_supply     fg_psy;
};

static struct bq27520_device *the_chip = NULL;

static int bq27520_update_bqfs(struct bq27520_device *chip);
static int bq27520_unseal(struct bq27520_device *chip);
static bool bq27520_check_rom_mode(struct bq27520_device *chip);

static DEFINE_MUTEX(bq27520_fg_mutex);

/********************************************************
 *					 I2C I/O function 				              *
 *********************************************************/
static int bq27520_fg_i2c_read_byte(struct bq27520_device *chip, unsigned char reg, unsigned char *val)
{
	int ret;
	int i;

    mutex_lock(&chip->read_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_read_byte_data(chip->client, reg);
        if(ret >= 0)
			break;
        msleep(5);
    }
    mutex_unlock(&chip->read_mutex);

	if(ret < 0){
        pr_err("fail to read reg[0x%x],ret=%d\n",reg,ret);
        return ret;
	}

	*val = ret;

	pr_debug("reg[%x]=0x%x,ret=%d\n",reg,*val,ret);

	return ret;
}

static int bq27520_fg_i2c_write_byte(struct bq27520_device *chip, u8 reg, u8 val)
{
	int ret;
	int i;

    mutex_lock(&chip->read_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_write_byte_data(chip->client, reg, val);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&chip->read_mutex);

	if(ret < 0){
        pr_err("fail to write reg[0x%x],ret=%d\n",reg,ret);
        return ret;
	}

	return ret;
}

static inline int bq27520_fg_i2c_write_word(struct bq27520_device *chip, u8 reg, u16 val)
{
    int ret;
	int i;

	val = __cpu_to_le16(val);

    mutex_lock(&chip->read_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_write_i2c_block_data(chip->client, reg, sizeof(u16), (u8 *)&val);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&chip->read_mutex);
	
	if(ret < 0)
		pr_err("fail to write reg[0x%x],ret=%d\n",reg,ret);

	return ret;
}

static inline int bq27520_fg_i2c_read_word(struct bq27520_device *chip, u8 reg, u16 *val)
{
	int ret;
	u16 tmp;
	int i;

    mutex_lock(&chip->read_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_read_i2c_block_data(chip->client, reg, sizeof(u16), (u8 *)&tmp);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&chip->read_mutex);

	if(ret < 0)
		pr_err("fail to read reg[0x%x],ret=%d\n",reg,ret);
	
	*val = __le16_to_cpu(tmp);

	pr_debug("reg[%x]=0x%x,ret=%d\n",reg,*val,ret);

	return ret;
}

static inline int bq27520_fg_read_i2c_blk(struct bq27520_device *chip, u8 reg, u8 *data, u8 len)
{

    struct i2c_client *client = chip->client;
    int ret;
    int i = 0;
    u32 ext_flag = client->ext_flag;
    dma_addr_t dma_addr = 0;
    u8 *vir_addr = NULL;
    u8 buf_len = 200;
    
    if (!client->adapter){
        ret = -ENODEV;
        goto err;
    }    

    if(len <= sizeof(u16)){
        struct i2c_msg msg[2];
        
        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].buf = &reg;
        msg[0].len = 1;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].buf = data;
        msg[1].len = len;
        mutex_lock(&chip->read_mutex);
        for(i = 0; i < I2C_RETRY_CNT; i++){
            ret =  i2c_smbus_read_i2c_block_data(client, reg, len, data);
            if(ret >= 0) break;
            msleep(200);
        }
        mutex_unlock(&chip->read_mutex); 
        if (ret < 0)
            goto err;
    }else{
        vir_addr = (u8 *)dma_alloc_coherent(chip->dev, buf_len, &dma_addr, GFP_KERNEL);
        if (vir_addr == NULL) {
            pr_err("Cannot allocate memory\n");
            ret = -ENOMEM;;
            goto err;
        }
        memset(vir_addr,0,buf_len);
        *vir_addr = reg;
        client->ext_flag = client->ext_flag|I2C_DMA_FLAG;
        mutex_lock(&chip->read_mutex);
        for(i = 0; i < I2C_RETRY_CNT; i++){
    	    ret = i2c_master_send(client, (void *)dma_addr, sizeof(reg));
	        if (ret < 0)continue;
            memset(vir_addr,0,buf_len);
            ret = i2c_master_recv(client, (void *)dma_addr, len);
            if(ret >= 0) break;
            msleep(200);
        }
        mutex_unlock(&chip->read_mutex);    
        if (ret < 0)
            goto err;
	    if((ret <= len) && (ret > 0)){
            memcpy(data,vir_addr,ret);
	    }
    }
    
err:
    if(vir_addr){
        dma_free_coherent(chip->dev, buf_len, vir_addr, dma_addr);
        vir_addr = NULL;
        dma_addr = 0;
    }
    client->ext_flag = ext_flag;
    return ret;
}

static inline int bq27520_fg_write_i2c_blk(struct bq27520_device *chip, u8 reg, u8 *data, u8 len)
{
    struct i2c_msg msg;
    struct i2c_client *client = chip->client;
    int ret;
    int i = 0;
    u32 ext_flag = client->ext_flag;
    dma_addr_t dma_addr = 0;
    u8 *vir_addr = NULL;
    u8 buf_len = 200;

    if (!client->adapter){
        ret = -ENODEV;
        goto err;
    } 
    if(len <= sizeof(u16)){
        u8 buf[4];
        buf[0] = reg;
        memcpy(&buf[1], data, len);
        
        msg.buf = buf;
        msg.addr = client->addr;
        msg.flags = 0;
        msg.len = len + 1;
        mutex_lock(&chip->read_mutex);    
        for(i = 0; i < I2C_RETRY_CNT; i++){
            ret = i2c_smbus_write_i2c_block_data(client, reg, len, data);
            if(ret >= 0) break;
            msleep(200);
        }
        mutex_unlock(&chip->read_mutex);
    }else{
        vir_addr = (u8 *)dma_alloc_coherent(chip->dev, buf_len, &dma_addr, GFP_KERNEL);
        if (vir_addr == NULL) {
            pr_err("Cannot allocate DMA memory\n");
            ret = -ENOMEM;
            goto err;
        }
        
        memset(vir_addr,0,buf_len);
        *vir_addr = reg;
        memcpy(vir_addr+1,data,len);
        client->ext_flag = client->ext_flag|(I2C_DMA_FLAG);
        mutex_lock(&chip->read_mutex);    
        for(i = 0; i < I2C_RETRY_CNT; i++){
            ret = i2c_master_send(client, (void *)(dma_addr), len+1);
            if(ret >= 0) break;
            msleep(200);
        }
        mutex_unlock(&chip->read_mutex);
    }
    
    if (ret < 0)
        goto err;
err:
    if(vir_addr){
        dma_free_coherent(chip->dev, buf_len, vir_addr, dma_addr);
        vir_addr = NULL;
        dma_addr = 0;
    }
    client->ext_flag = ext_flag;
    return ret;
}

static inline int bq27520_fg_write_verify_reg(struct bq27520_device *fg_chip, u8 reg, u16 val)
{
	int i;
	int ret;
	u16 tmp;

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		ret = bq27520_fg_i2c_write_word(fg_chip, reg, val);
		if (unlikely(ret < 0))
			return ret;
			
		ret = bq27520_fg_i2c_read_word(fg_chip, reg, &tmp);
		if (unlikely(ret < 0))
			return ret;

		if (likely(val == tmp))
			return 0;
	}
	
	return -EIO;
}

/******************************************************** 
 *				bq27520 special functions 				                     *
 *********************************************************/
static u8 bq27520_checksum(u8 *data, u8 len)
{
    u16 sum = 0;
    int i;

    for (i = 0; i < len; i++)
        sum += data[i];

    sum &= 0xFF;

    return 0xFF - sum;
}

static int bq27520_read_fw_version(struct bq27520_device *chip)
{
    int ret;
    u16 buf;

    ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_FW_VER);
    if(ret < 0)
    {
        pr_err("Failed to send read fw version command\n");
        return ret;
    }

    mdelay(2);
    ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_CNTL, &buf);
    if(ret < 0){
        dev_err(chip->dev,"Failed to read read fw version \n");
        return ret;
    }

    return buf;

}

#if 0
static int bq27520_get_batt_flags(struct bq27520_device *chip)
{
    int ret;
    u16 buf;

    ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_FLAGS, &buf);
    if(ret < 0)
    {
        pr_err("Failed to read read batt flags \n");
        return ret;
    }

    return buf;
}
#endif

static int bq27520_reset_fuel_gauge(struct bq27520_device *chip, void *data)
{
    int ret;

    //1. unseal
    ret = bq27520_unseal(chip);
    if(ret < 0)
    {
        pr_err("fail to unseal,ret=%d\n",ret);
        return ret;
    }

    //2. reset the fuel gauge
    mdelay(2);
    ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_RESET);
    if(ret < 0)
        pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);

    //3. sealed
    mdelay(10);
    ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_SEALED);
    if(ret < 0)
        pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);

    return ret;
}

static int bq27520_enter_rom_subclass(struct bq27520_device *chip, int classid)
{
    int ret;

    //1. unseal
    ret = bq27520_unseal(chip);
    if(ret == 0)
    {
        pr_err("fail to unseal,ret=%d\n",ret);
        return -1;
    }
    //2. reset
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_BLKDATCTL, 0);
    if(ret < 0)
        return ret;

    //3. access the registers subclass
    mdelay(2);
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_DFCLASS, classid);
    if(ret < 0)
        return ret;

    return 0;
}

static int bq27520_modify_opconfigb_wrtemp(struct bq27520_device *chip)
{
    int ret = 0;
    u8 op_confib,new_confib;
    u8 checksum;
    u8 temp;

    //1,2,3 enter rom subclass
	ret = bq27520_enter_rom_subclass(chip, 0x40);
    if(ret < 0)
    {
        pr_err("fail to enter rom subclass,ret=%d\n",ret);
        return ret;
    }

    //4. write the block offset location
    mdelay(2);
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_DFBLOCK, 0x00);
    if(ret < 0)
    {
        pr_err("fail to write reg[0x%x],ret=%d\n",0x3f, ret);
        return ret;
    }

    //5. read the data of a specific offset
    mdelay(2);
    ret = bq27520_fg_i2c_read_byte(chip, 0x4b, &op_confib);
    if(ret < 0)
    {
        pr_err("fail to write reg[0x%x],ret=%d\n",0x4b,ret);
        return ret;
    }
    pr_debug("read reg[0x%x]=0x%x\n",0x4b,op_confib);

    //6. read 1-byte checksum
    ret = bq27520_fg_i2c_read_byte(chip, BQ27520_REG_BLKCHKSUM, &checksum);
    if(ret < 0)
    {
        pr_err("fail to write reg %d,ret=%d\n",0x60,ret);
        return ret;
    }
    pr_debug("read reg[0x%x]=0x%x\n",0x60,checksum);

    //7. set new WRTEMP
    new_confib = op_confib | 0x80;

    temp = (255-checksum-op_confib) % 256;

    checksum = 255 - (temp + new_confib) % 256;

    //8. write new opconfigB
    ret = bq27520_fg_i2c_write_byte(chip, 0x4b, new_confib);
    if(ret < 0)
    {
        pr_err("fail to write reg[0x%x],ret=%d\n",0x4b,ret);
        return ret;
    }

    //9. write new checksum
    mdelay(2);
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_BLKCHKSUM, checksum);
    if(ret < 0)
    {
        pr_err("fail to write reg[0x%x],ret=%d\n",0x60,ret);
        return ret;
    }

    //10. reset
    mdelay(2);
    ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_RESET);
    if(ret < 0)
    {
        pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);
        return ret;
    }

    //11. sealed
    ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_SEALED);
    if(ret < 0)
    {
        pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);
        return ret;
    }

    return ret;
}

static int bq27520_read_df(struct bq27520_device *chip, u8 classid, u8 offset, u8* buf, u8 len)
{
    int ret;
    u8 tmp_buf[40];
    int i;
    u8 crc;
    u8 crc_calc = 0;

    if (offset % 32 + len > 32)
        return -EINVAL; // less than one block boundary one time

    ret = bq27520_enter_rom_subclass(chip, classid);
    if(ret < 0)
    {
        pr_err("fail to enter rom subclass,ret=%d\n",ret);
        return ret;
    }

    mdelay(2);
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_DFBLOCK, offset / 32);
    if(ret < 0)
        return ret;

    mdelay(2);
    ret = bq27520_fg_read_i2c_blk(chip, BQ27520_REG_BLOCKDATA, tmp_buf, 32);
    if(ret < 0)
        return ret;

    bq27520_fg_i2c_read_byte(chip, BQ27520_REG_BLKCHKSUM, &crc);
    crc_calc = bq27520_checksum(tmp_buf,32);
    if(crc != crc_calc)
        return -2;

    for(i = 0; i < len; i++)
    {
        buf[i] =  tmp_buf[offset % 32 + i];
    }

    return len;
}

static int bq27520_write_df(struct bq27520_device *chip, u8 classid, u8 offset, u8* buf, u8 len)
{
    int ret;
    u8 tmp_buf[40];
    int i;
    u8 crc;
    u8 crc_calc = 0;

    if (offset % 32 + len > 32)
        return -EINVAL; // less than one block one time

    ret = bq27520_enter_rom_subclass(chip, classid);
    if(ret < 0)
    {
        pr_err("fail to enter rom subclass,ret=%d\n",ret);
        return ret;
    }
    mdelay(2);
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_DFBLOCK, offset / 32);
    if(ret < 0)
        return ret;

    mdelay(2);
    ret = bq27520_fg_read_i2c_blk(chip, BQ27520_REG_BLOCKDATA, tmp_buf, 32);
    if(ret < 0)
        return ret;

    mdelay(2);
    bq27520_fg_i2c_read_byte(chip, BQ27520_REG_BLKCHKSUM, &crc);
    crc_calc = bq27520_checksum(tmp_buf,32);
    if(crc != crc_calc)
        return -2;
    //update with new value
    for(i = 0; i < len; i++)
        tmp_buf[offset % 32 + i] = buf[i];
    // calculate new crc
    crc_calc = bq27520_checksum(tmp_buf,32);

    mdelay(2);
    ret = bq27520_fg_write_i2c_blk(chip, BQ27520_REG_BLOCKDATA, tmp_buf, 32);
    if(ret < 0)
        return ret;

    mdelay(2);
    ret = bq27520_fg_i2c_write_byte(chip, BQ27520_REG_BLKCHKSUM, crc_calc);
    return ret;
}

int bq27520_fg_get_batt_voltage(struct bq27520_device *chip, void *data)
{
    int ret;
	u16 value;

	if (!chip) {
		pr_err("called before init\n");
		return 0;
	}

	ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_VOLT, &value);
	if (ret<0) {
		pr_err("fail to read batt vol\n");
		return 0;
	}

	return value;
}

static int bq27520_fg_hw_init(struct bq27520_device *chip, void *data)
{
    return 0;
}

static int bq27520_fg_set_batt_temp(struct bq27520_device *chip, void *data)
{
    int ret;
    int batt_temp = *(int*)data;

	batt_temp = batt_temp + 2730;
    
	ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_TEMP,(u16)batt_temp);
	if (ret<0) {
		pr_err("fail to write batt temp\n");
		return -EINVAL;
	}
	
	return 0;
}

static int bound_soc(int soc)
{
    soc = max(0, soc);
    soc = min(100, soc);
    return soc;
}

int bq27520_fg_get_cycle_count(struct bq27520_device *chip, void *data)
{
    int ret;
    u16 value;
    ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_CC, &value);
    if (ret<0)
    {
        pr_err("fail to get cycle count.\n");
        return 0;
    }
    return value;
}

int bq27520_fg_get_batt_soc(struct bq27520_device *chip, void *data)
{
    int ret;
    u16 value;

    ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_SOC,&value);
    if (ret < 0)
    {
        value = 50;
    }

	pr_err("soc:%d\n", value);

    value = bound_soc(value);

    return value;
}

int bq27520_fg_get_ibatt_now(struct bq27520_device *chip, void *data)
{
    int ret;
	u16 value = 0;

	ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_AI, &value);
	if (ret<0) {
		pr_err("fail to get batt current\n");
		return 0;
	}
	
	return (s16)(value*(-1));
}

static int bq27520_fg_get_rm_mah(struct bq27520_device *chip, void *data)
{
    int ret;
	u16 value = 0;

	ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_RM, &value);
	if (ret<0) {
		pr_err("fail to rm mah\n");
		return 0;
	}
	
	return (s16)value;
}

static int bq27520_fg_get_full_mah(struct bq27520_device *chip, void *data)
{
    int ret;
	u16 value = 0;

	ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_FCC, &value);
	if (ret<0) {
		pr_err("fail to full mah\n");
		return 0;
	}
	
	return (s16)value;
}

#define BQ27520_QMAX_MAH_CLASSID     82
#define BQ27520_QMAX_MAH_OFFSET      1
#define BQ27520_QMAX_MAH_LENGTH      2
static int bq27520_fg_get_qmax_mah(struct bq27520_device *chip, void *data)
{
    u8 buf[2] = {0};
    int ret;
	int val;

    ret = bq27520_read_df(chip,
		                  BQ27520_QMAX_MAH_CLASSID,
		                  BQ27520_QMAX_MAH_OFFSET, 
		                  buf, 
		                  BQ27520_QMAX_MAH_LENGTH);
	
    if( ret != BQ27520_QMAX_MAH_LENGTH){
		pr_info("qmax_mah read df err,ret=%d\n",ret);
        return -1;
    }
	
	val = (buf[0]<<8) | buf[1];
	pr_debug("get qmax_mah=%d \n",val);
	
	return val;
}

static int bq27520_fg_dump_regs(struct bq27520_device *chip, void *data)
{
    int i;
    u16 val;
    static int first_flag = 1;

    if(first_flag)
    {
        first_flag = 0;
        printk("MDREG ");
    }

    for(i=0; i<=0x2e; i+=2)
    {
        bq27520_fg_i2c_read_word(chip, i, &val);
        printk("reg[0x%02x]0x%04x \n",i,val);
    }
    printk("fw version:%s\n", chip->fw_version);
    
    return 0;
}

/* the following routines are for bqfs/dffs update purpose, can be removed if not used*/
static int bq27520_check_seal_state(struct bq27520_device *chip)
{
    int status;
    int ret;
    u16 buf;

    bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_CTNL_STATUS);
    mdelay(2);

    ret = bq27520_fg_i2c_read_word(chip, BQ27520_REG_CNTL, &buf);
    if(ret < 0)
        return ret;

    if((buf & 0x6000) == 0) //FA and SS neither set
        status = BQ27520_SECURITY_FA;
    else if((buf & 0x2000) == 0) // SS not set
        status = BQ27520_SECURITY_UNSEALED;
    else
        status = BQ27520_SECURITY_SEALED;

    return status;
}

static int bq27520_unseal(struct bq27520_device *chip)
{
    int ret;

    bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_UNSEAL_KEY & 0xFFFF);
    mdelay(2);
    bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, (BQ27520_UNSEAL_KEY >> 16)& 0xFFFF);
    mdelay(5);

    ret = bq27520_check_seal_state(chip);
    if(ret == BQ27520_SECURITY_UNSEALED || ret == BQ27520_SECURITY_FA)
        return 1;
    else
        return 0;
}

static int bq27520_unseal_full_access(struct bq27520_device *chip)
{
    int ret;

    bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_FA_KEY & 0xFFFF);
    mdelay(2);
    bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, (BQ27520_FA_KEY >> 16)& 0xFFFF);
    mdelay(5);

    ret = bq27520_check_seal_state(chip);
    if(ret == BQ27520_SECURITY_FA)
        return 1;
    else
        return 0;
}

static bool bq27520_check_rom_mode(struct bq27520_device *chip)
{
    struct i2c_client *client = chip->client;
    int ret;
    u8 val;

    client->addr = BQGAUGE_I2C_ROM_ADDR;
    ret = bq27520_fg_i2c_read_byte(chip, 0x66, &val);
    mdelay(2);
    client->addr = BQGAUGE_I2C_DEV_ADDR;//restore address
    if(ret < 0 )
    {
        pr_info("it is not in rom mode\n");
        return false;
    }
    pr_info("it is in rom mode\n");
    return true;
}

static bool bq27520_enter_rom_mode(struct bq27520_device *chip)
{
    int ret;

    ret = bq27520_fg_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_ENTER_ROM);
    mdelay(2);
    if(ret < 0)
        return false;

    return bq27520_check_rom_mode(chip);
}

#define BQ27520_DEVICE_NAME_CLASSID     48
#define BQ27520_DEVICE_NAME_OFFSET      17
#define BQ27520_DEVICE_NAME_LENGTH      7

static bool bq27520_check_update_necessary(struct bq27520_device *chip)
{
    // this is application specific, return true if need update firmware or data flash
    u8 buf[40] = {0};
    int ret;

    ret = bq27520_read_df(chip,
                          BQ27520_DEVICE_NAME_CLASSID,
                          BQ27520_DEVICE_NAME_OFFSET,
                          buf,
                          BQ27520_DEVICE_NAME_LENGTH);

    if( ret > BQ27520_DEVICE_NAME_LENGTH){
		pr_info("%s:read df err,ret=%d\n",__func__,ret);
        return false;
    }
	
	pr_info("get current bqfs_version = %s\n",buf);
	
    if(strncmp(buf, chip->fw_version, BQ27520_DEVICE_NAME_LENGTH) == 0) {
        return false;
    } else{    
  		pr_info("need update to new bqfs_version = %s\n",chip->fw_version);
        return true;
    }

    return true;
}

static bool bq27520_mark_as_updated(struct bq27520_device *chip)
{
    // this is application specific
    int ret;
    ret = bq27520_write_df(chip,
                           BQ27520_DEVICE_NAME_CLASSID,
                           BQ27520_DEVICE_NAME_OFFSET,
		                   chip->fw_version, 
                           BQ27520_DEVICE_NAME_LENGTH);
    if(ret < 0)
        return false;
    else
        return true;
}

static bool bq27520_update_execute_cmd(struct bq27520_device *chip, const bqfs_cmd_t *cmd)
{
    int ret;
    uint8_t tmp_buf[CMD_MAX_DATA_SIZE];

    switch (cmd->cmd_type)
    {
    case CMD_R:
        ret = bq27520_fg_read_i2c_blk(chip, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
        if( ret < 0)
        {
            pr_err("CMD_R:fail to read block reg=0x%x,data[0] = 0x%x, len=%d, ret=%d",cmd->reg, cmd->data.bytes[0], cmd->data_len, ret);
            return false;
        }
        return true;

    case CMD_W:
        ret = bq27520_fg_write_i2c_blk(chip, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
        if(ret < 0)
        {
            pr_err("CMD_W:fail to write block reg=0x%x,data[0] = 0x%x, len=%d, ret=%d",cmd->reg, cmd->data.bytes[0], cmd->data_len, ret);
            return false;
        }
        return true;

    case CMD_C:
        ret = bq27520_fg_read_i2c_blk(chip, cmd->reg, tmp_buf, cmd->data_len);
        if (ret < 0)
        {
            pr_err("CMD_C:fail to read block reg=0x%x,data[0] = 0x%x, len=%d, ret=%d",cmd->reg, cmd->data.bytes[0], cmd->data_len, ret);
            return false;
        }
        if (memcmp(tmp_buf, cmd->data.bytes, cmd->data_len))
        {
            pr_err("\nCommand C failed at line %d: reg[0x%x] data[0x%x], temp[0x%x]\n",
                   cmd->line_num,cmd->reg,*(cmd->data.bytes),*tmp_buf);
            return false;
        }
        return true;

    case CMD_X:
        mdelay(cmd->data.delay);
        return true;

    default:
        pr_err("Unsupported command at line %d\n",
               cmd->line_num);
        return false;
    }
}

static int bq27520_update_bqfs(struct bq27520_device *chip)
{
    struct i2c_client *client = chip->client;
    u16 i;
    int ret;

	chip->fw_version = BQFS_VERSION_NX535_ATL;
	chip->bqfs_image = bqfs_image_nx535_atl;
	chip->bqfs_image_size = ARRAY_SIZE(bqfs_image_nx535_atl);
    if(bq27520_check_rom_mode(chip))
        goto update;// already in rom mode
    // check if needed update
    if(!bq27520_check_update_necessary(chip)){
		pr_info("not need update\n");
		return -EINVAL;
    }

    if (bq27520_check_seal_state(chip) != BQ27520_SECURITY_FA){
        if(!bq27520_unseal(chip)) 
            return -EINVAL;
        mdelay(10);
        if(!bq27520_unseal_full_access(chip)) 
            return -EINVAL;
    }

    if(!bq27520_enter_rom_mode(chip)){ 
		pr_err("fail to enter rom mode\n");
		return -EINVAL;
    }

update:
    client->timing = 100;
    client->addr = BQGAUGE_I2C_ROM_ADDR;
    pr_err("BQFS Updating");
    for(i = 0; i < chip->bqfs_image_size; i++){
        dev_err(chip->dev,". %d/%d",i+1, chip->bqfs_image_size);
        if(!bq27520_update_execute_cmd(chip,&chip->bqfs_image[i])){
            pr_err("%s: Failed at command=%d addr=0x%x reg=0x%x\n",__func__,
                   i,chip->bqfs_image[i].addr, chip->bqfs_image[i].reg);
            return -EINVAL;
        }
    }
    pr_err("BQFS Done!\n");

    client->addr = BQGAUGE_I2C_DEV_ADDR;
    // mark as updated
    ret = bq27520_mark_as_updated(chip);
    if(!ret)
        pr_err("fail to mark update\n");

    return 0;
}

static int bq27520_fg_update_batt_profile(struct bq27520_device *chip, void *data)
{
    //int battery_id;
    u16 i;
    int ret;
    struct i2c_client *client = chip->client;
 
    chip->fw_version = BQFS_VERSION_NX535_ATL;
	chip->bqfs_image = bqfs_image_nx535_atl;
	chip->bqfs_image_size = ARRAY_SIZE(bqfs_image_nx535_atl);

    if(bq27520_check_rom_mode(chip))
        goto update;// already in rom mode

    if (bq27520_check_seal_state(chip) != BQ27520_SECURITY_FA) {
        if(!bq27520_unseal(chip))
            return -EINVAL;
        mdelay(10);
        if(!bq27520_unseal_full_access(chip))
            return -EINVAL;
    }

    if(!bq27520_enter_rom_mode(chip)){
        pr_err("fail to enter rom mode\n");
        return -EINVAL;
    }

update:
    client->addr = BQGAUGE_I2C_ROM_ADDR;
    pr_err("BQFS Updating");
    for(i = 0; i < ARRAY_SIZE(bqfs_image_nx535_atl); i++){
        dev_err(chip->dev,". %d/%d",i+1, chip->bqfs_image_size);
        if(!bq27520_update_execute_cmd(chip, &chip->bqfs_image[i])){
            pr_err("%s: Failed at command=%d addr=0x%x reg=0x%x\n",__func__,
                   i,chip->bqfs_image[i].addr, chip->bqfs_image[i].reg);
            return -EINVAL;
        }
    }
    pr_err("BQFS Done!\n");

    client->addr = BQGAUGE_I2C_DEV_ADDR;
    // mark as updated
    ret = bq27520_mark_as_updated(chip);
    if(!ret)
        pr_err("fail to mark update\n");

    return ret;
}

static int (*fg_func[BATTERY_COMMOM_CMD_NUMBER]) (struct bq27520_device *chip, void *data);


/*
* FUNCTION
*        Internal_chr_control_handler
*
* DESCRIPTION
*         This function is called to set the charger hw
*
* CALLS
*
* PARAMETERS
*        None
*
* RETURNS
*
*
* GLOBALS AFFECTED
*       None
*/
int bq27520_fg_control_interface(BATTERY_COMMON_CTRL_CMD cmd, void *data)
{
    int status;
    static signed int init = -1;
    
	if (!the_chip) {
		pr_err("called before init\n");
		return -1;
	}

    if (init == -1)
    {
        init = 0;
        fg_func[BATTERY_COMMON_CMD_HW_INIT] = bq27520_fg_hw_init;
        fg_func[BATTERY_COMMON_CMD_HW_RESET] = bq27520_reset_fuel_gauge;
        fg_func[BATTERY_COMMON_CMD_DUMP_REGISTER] = bq27520_fg_dump_regs;
        fg_func[BATTERY_COMMON_CMD_GET_BATT_VOLT] = bq27520_fg_get_batt_voltage;
        fg_func[BATTERY_COMMON_CMD_GET_BATT_RMC] = bq27520_fg_get_rm_mah;
        fg_func[BATTERY_COMMON_CMD_GET_BATT_FCC] = bq27520_fg_get_full_mah;
        fg_func[BATTERY_COMMON_CMD_GET_BATT_SOC] = bq27520_fg_get_batt_soc;
		fg_func[BATTERY_COMMON_CMD_GET_BATT_CURRENT] = bq27520_fg_get_ibatt_now;
		fg_func[BATTERY_COMMON_CMD_GET_BATT_DCC] = bq27520_fg_get_qmax_mah;
        fg_func[BATTERY_COMMON_CMD_GET_CYCLE_COUNT] = bq27520_fg_get_cycle_count;
		fg_func[BATTERY_COMMON_CMD_SET_BATT_TEMP] = bq27520_fg_set_batt_temp;
        fg_func[BATTERY_COMMON_CMD_UPDATE_BATT_PROFILE] = bq27520_fg_update_batt_profile;
    }

    if (cmd < BATTERY_COMMOM_CMD_NUMBER)
    {
        if (fg_func[cmd] != NULL)
            status = fg_func[cmd](the_chip, data);
        else
        {
            return -1;//STATUS_UNSUPPORTED;
        }
    }
    else
    {
        return -1;//STATUS_UNSUPPORTED;
    }
    return status;

}

enum power_supply_property bq27520_batt_props[] =
{
    POWER_SUPPLY_PROP_CONTROL_FUNCTION,
};

static int bq27520_fg_get_property(struct power_supply *psy,
                                   enum power_supply_property psp,
                                   union power_supply_propval *val)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_CONTROL_FUNCTION:
        val->int64val = (int64_t)bq27520_fg_control_interface;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int  bq27520_fg_probe(struct i2c_client *client,
                             const struct i2c_device_id *dev_id)
{
    int ret;
    struct bq27520_device *chip;
    int fw_version;

    chip = kzalloc(sizeof(struct bq27520_device), GFP_KERNEL);
    if(!chip){
        pr_err("Cannt allocate memory\n");
    }
    chip->client = client;
    chip->dev = &client->dev;

    mutex_init(&chip->read_mutex);
    i2c_set_clientdata(client, chip);
    ret = bq27520_update_bqfs(chip);
    if(ret < 0)
        pr_err("do not update bqfs\n");

    bq27520_modify_opconfigb_wrtemp(chip);
    fw_version = bq27520_read_fw_version(chip);

    chip->fg_psy.name = "fg";
    chip->fg_psy.type = POWER_SUPPLY_TYPE_BATTERY;
    chip->fg_psy.properties = bq27520_batt_props;
    chip->fg_psy.num_properties = ARRAY_SIZE(bq27520_batt_props);
    chip->fg_psy.get_property = bq27520_fg_get_property;
    power_supply_register(chip->dev, &chip->fg_psy);

    the_chip = chip;
    pr_err("fw_version=0x%x\n",fw_version);

    return ret;
}

static struct of_device_id  bq27520_fg_match_table[] =
{
    { .compatible = "mediatek,EXT_FUELGAUGE",},
    {}
};

static const struct i2c_device_id bq27520_fg_id[] =
{
    {"bq27520", 1},
    {}
};


static struct i2c_driver bq27520_fg_driver =
{
    .probe 		= bq27520_fg_probe,
    .id_table 	= bq27520_fg_id,
    .driver = {
        .name = "bq27520",
        .of_match_table = bq27520_fg_match_table,
    },
};

#if 0
static struct i2c_board_info i2c_bq27520 __initdata =
{
    I2C_BOARD_INFO("bq27520", (0xAA >> 1))
};
#endif

static int __init bq27520_fg_init(void)
{
    printk( "%s:enter...\n", __func__);
    return i2c_add_driver(&bq27520_fg_driver);
}

subsys_initcall(bq27520_fg_init);

MODULE_AUTHOR("bq27520 Leo.guo");
MODULE_DESCRIPTION("bq27520 fuel gauge battery driver");
MODULE_LICENSE("GPL");
