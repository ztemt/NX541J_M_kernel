/*
 * ALSA SoC Texas Instruments TAS2555 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2015 Texas Instruments, Inc.
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
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

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

//#include <dt-bindings/sound/tas2555.h>

#include "tas2555.h"

#include <linux/dma-mapping.h>
//set default PLL CLKIN to GPI2 (MCLK) = 0x0D
#define TAS2555_DEFAULT_PLL_CLKIN 0x0D

#define P_GPIO_TAS2555_RST        157
#define P_GPIO_TAS2555_MCLK        0
#define P_GPIO_TAS2555_I2SDI       7

#define GPIO_TAS2555_RST_PIN         (P_GPIO_TAS2555_RST | 0x80000000)
#define GPIO_TAS2555_MCLK_PIN         (P_GPIO_TAS2555_MCLK | 0x80000000)
#define GPIO_TAS2555_I2SDI_PIN            (P_GPIO_TAS2555_I2SDI | 0x80000000)

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0
#define GPIO_MODE_06 6
#define GPIO_MODE_02 2
#define GPIO_MODE_03 3
#define GPIO_MODE_00 0
#define GPIO_DIR_OUT 1
#define GPIO_DIR_IN  0
extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_set_gpio_out(unsigned long pin, unsigned long output);
extern int mt_set_gpio_pull_enable(unsigned long pin,  unsigned long enable);
extern int mt_set_gpio_pull_select(unsigned long pin, unsigned long select);


//set default PLL CLKIN to GPI2 (MCLK) = 0x00
//#define TAS2555_DEFAULT_PLL_CLKIN 0x00

#define ENABLE_TILOAD			//only enable this for in-system tuning or debug, not for production systems
#ifdef ENABLE_TILOAD
#include "tiload.h"
#endif

#define TAS2555_FW_FULL_NAME     "/etc/firmware/tas2555_uCDSP.bin"
#define TAS2555_CAL_NAME    "/persist/audio/tas2555_cal.bin"

#define TAS2555_FW_FORMAT   0x01
const char *tas2555_fw_header = "TAS2555-uCDSP";

#define MAX_TUNINGS 16

#define TAS2555_UDELAY 0xFFFFFFFE

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2555_BLOCK_PLL			0x00
#define TAS2555_BLOCK_BASE_MAIN		0x01
#define TAS2555_BLOCK_CONF_COEFF	0x03
#define TAS2555_BLOCK_CONF_PRE		0x04
#define TAS2555_BLOCK_CONF_POST		0x05
#define TAS2555_BLOCK_CONF_POST_POWER	0x06
#define TAS2555_BLOCK_CONF_CAL		0x0A

//static struct tas2555_register register_addr = { 0 };

#define TAS2555_REG_IS_VALID(book, page, reg) \
        ((book >= 0) && (book <= 255) &&\
        (page >= 0) && (page <= 255) &&\
        (reg >= 0) && (reg <= 127))

static struct i2c_client *g_client = NULL;
static struct tas2555_priv *g_tas2555 = NULL;
static int g_logEnable = 1;

static int fw_parse_header(struct tas2555_priv *pTAS2555, 
	TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize);
static void tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock);
static void tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType);
static void tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame);
static void tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName);
static void tas2555_fw_ready(const struct firmware *pFW, void *pContext);

static DEFINE_MUTEX(smartpa_lock); 
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa;
#define GTP_DMA_MAX_TRANSACTION_LENGTH  128   // for DMA mode
#define I2C_MASTER_CLOCK                400

static ssize_t tas2555_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tas2555_priv *pTAS2555 = dev_get_drvdata(dev);
	
	int nResult = 0;
	dev_err(pTAS2555->dev,"%s enter\n", __FUNCTION__);
	mutex_lock(&pTAS2555->file_lock);
	dev_err(pTAS2555->dev,"%s  222222\n", __FUNCTION__);
	nResult = request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
		pTAS2555->dev, GFP_KERNEL, pTAS2555, tas2555_fw_ready);
	dev_err(pTAS2555->dev,"%s, %d \n", __FUNCTION__, nResult);
	mutex_unlock(&pTAS2555->file_lock);

	return count;
}

static DEVICE_ATTR(tas2555_data, 0220, NULL, tas2555_data_store);

static int tas2555_set_bit_rate(struct tas2555_priv *pTAS2555, unsigned int nBitRate)
{
	int ret = 0, n = -1;
	
	dev_dbg(pTAS2555->dev, "tas2555_set_bit_rate: nBitRate = %d \n",
		nBitRate);
	switch(nBitRate){
		case 16:
			n = 0;
		break;
		case 20:
			n = 1;
		break;
		case 24:
			n = 2;
		break;
		case 32:
			n = 3;
		break;
	}
	
	if(n >= 0)
		ret = pTAS2555->update_bits(pTAS2555, 
			TAS2555_ASI1_DAC_FORMAT_REG, 0x18, n<<3);	
		
	return ret;	
}

static int tas2555_i2c_read_byte(struct i2c_client *client, unsigned char addr, unsigned int *returnData)
{
    char     cmd_buf[1] = {0x00};
    char     readData = 0;
    int     ret = 0;
    cmd_buf[0] = addr;

    mutex_lock(&smartpa_lock);

    ret = i2c_master_send(client, &cmd_buf[0], 1);
    if (ret < 0)
    {
        mutex_unlock(&smartpa_lock);
		dev_err(&client->dev, 
			"%s, i2c send fail %d \n", __FUNCTION__, ret);
        return -1;
    }
    ret = i2c_master_recv(client, &readData, 1);
    mutex_unlock(&smartpa_lock);
    if (ret < 0)
    {
		dev_err(&client->dev, 
			"%s, i2c recv fail %d \n", __FUNCTION__, ret);
        return -1;
    }
	//if(g_logEnable) printk("%s, R[0x%x]=0x%x\n", __FUNCTION__, addr, readData);
    *returnData = readData;
    //printk("addr 0x%x data 0x%x \n", addr, readData);
    return 1;
}

//write register
static int tas2555_i2c_write_byte(struct i2c_client *client, unsigned char addr, unsigned char writeData)
{
    char    write_data[2] = {0};
    int    ret = 0;
    write_data[0] = addr;         // ex. 0x01
    write_data[1] = writeData;

    mutex_lock(&smartpa_lock);
    ret = i2c_master_send(client, write_data, 2);
	//if(g_logEnable) printk("%s, R[0x%x]=0x%x\n", __FUNCTION__, addr, writeData);
    mutex_unlock(&smartpa_lock);
    if (ret < 0)
    {
		dev_err(&client->dev, 
			"%s, send fail %d \n", __FUNCTION__, ret);
        return -1;
    }
    //printk("addr 0x%x data 0x%x \n", addr, writeData);
    return 1;
}

static int tas2555_i2c_dma_read(struct i2c_client *client, unsigned char addr, unsigned char *rxbuf, int len)
{
    int ret;
    int retry = 0;
    unsigned char buffer[2];

    struct i2c_msg msg[2] =
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buffer,
            .len = 1,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = (unsigned char *)gpDMABuf_pa,     
            .len = len,
            .timing = I2C_MASTER_CLOCK
        },
    };
    
    buffer[0] = addr;

	 mutex_lock(&smartpa_lock);
	//if(g_logEnable) printk("%s, R[0x%x] len=%d\n", __FUNCTION__, addr, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(rxbuf, gpDMABuf_va, len);
        break;
    }

	mutex_unlock(&smartpa_lock);
	
    return ret;
}


static int tas2555_i2c_dma_write(struct i2c_client *client, unsigned char addr, unsigned char *txbuf, unsigned int len)
{
    int ret;
    int retry = 0;
    unsigned char *wr_buf = gpDMABuf_va;
    
    struct i2c_msg msg =
    {
        .addr = (client->addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = 0,
        .buf = (unsigned char *)gpDMABuf_pa,
        .len = 1 + len,
        .timing = I2C_MASTER_CLOCK
    };
    
    wr_buf[0] = addr;


    memcpy(&wr_buf[1], txbuf, len);

	//if(g_logEnable) printk("%s, R[0x%x] len=%d\n", __FUNCTION__, addr, len);

	mutex_lock(&smartpa_lock);	
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
        break;
    }
	
	mutex_unlock(&smartpa_lock);

    return ret;
}


static void tas2555_change_book_page(struct tas2555_priv *pTAS2555, int nBook,
	int nPage)
{
	if ((pTAS2555->mnCurrentBook == nBook) && pTAS2555->mnCurrentPage == nPage)
		return;

	if (pTAS2555->mnCurrentBook != nBook) {	
		tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_PAGE, 0);
		pTAS2555->mnCurrentPage = 0;		
		tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_REG, nBook);		
		pTAS2555->mnCurrentBook = nBook;
		if (nPage != 0) {		
			tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_PAGE, nPage);
			pTAS2555->mnCurrentPage = nPage;
		}
	} else if (pTAS2555->mnCurrentPage != nPage) {		
		tas2555_i2c_write_byte(pTAS2555->client, TAS2555_BOOKCTL_PAGE, nPage);
		pTAS2555->mnCurrentPage = nPage;
	}
}

static int tas2555_dev_read(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int *pValue)
{
	int ret = 0;

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only reads from TILoad pass.
		nRegister &= ~0x80000000;
	}

	dev_dbg(pTAS2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
		TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister),
		TAS2555_PAGE_REG(nRegister));
	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));

    /*mtk_i2c_byte_read*/
	ret = tas2555_i2c_read_byte(pTAS2555->client, TAS2555_PAGE_REG(nRegister), pValue);

	return ret;
}

static int tas2555_dev_write(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int nValue)
{
	int ret = 0;
	if ((nRegister == 0xAFFEAFFE) && (nValue == 0xBABEBABE)) {
		pTAS2555->mbTILoadActive = true;
		return 0;
	}

	if ((nRegister == 0xBABEBABE) && (nValue == 0xAFFEAFFE)) {
		pTAS2555->mbTILoadActive = false;
		return 0;
	}

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}

	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
//  dev_err(codec->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
//      __func__, TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister),
//      TAS2555_PAGE_REG(nRegister), value);

	      /*mtk_i2c_byte_write*/
		ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_PAGE_REG(nRegister), nValue);
    return ret;
}

static int tas2555_dev_update_bits(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int nMask, unsigned int nValue)
{
	unsigned int temp = 0;
	int ret;
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}
	
	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
		
	ret = tas2555_i2c_read_byte(pTAS2555->client, TAS2555_PAGE_REG(nRegister), &temp);
	
	if((temp & nMask) != (nValue&nMask)){
		temp &= ~nMask;
		temp |= (nValue&nMask);
		ret = tas2555_i2c_write_byte(pTAS2555->client, TAS2555_PAGE_REG(nRegister), temp);
	}
	
	return ret;
}

static int tas2555_dev_bulk_read(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
    int ret;
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}

	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));

          /*mtk_i2c_dma_read*/
	ret = tas2555_i2c_dma_read(pTAS2555->client, TAS2555_PAGE_REG(nRegister), pData, nLength);

    return ret;
}

static int tas2555_dev_bulk_write(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
    int ret;
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			return 0;			// let only writes from TILoad pass.
		nRegister &= ~0x80000000;
	}

	tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));

   /*mtk_i2c_dma_write*/
		ret = tas2555_i2c_dma_write(pTAS2555->client, TAS2555_PAGE_REG(nRegister),pData, nLength);

  return ret;

}


#define TAS2555_PCTRL1_MASK (TAS2555_MADC_POWER_UP | \
                 TAS2555_MDAC_POWER_UP | \
                 TAS2555_DSP_POWER_UP)
#define TAS2555_PCTRL2_MASK (TAS2555_VSENSE_ENABLE | \
                 TAS2555_ISENSE_ENABLE | \
                 TAS2555_BOOST_ENABLE)
#define TAS2555_MUTE_MASK   (TAS2555_ISENSE_MUTE | TAS2555_CLASSD_MUTE)
#define TAS2555_SOFT_MUTE_MASK  (TAS2555_PDM_SOFT_MUTE | \
                 TAS2555_VSENSE_SOFT_MUTE | \
                 TAS2555_ISENSE_SOFT_MUTE | \
                 TAS2555_CLASSD_SOFT_MUTE)

static unsigned int p_tas2555_default_data[] = {
	TAS2555_ASI1_DAC_FORMAT_REG, 0x10,	//ASI1 DAC word length = 24 bits

	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,	
	TAS2555_RAMP_GEN_FREQ_REG, 0x11,
	TAS2555_MAIN_CLKIN_REG, 0x0F,	//NDIV_MUX_CLKIN = PLL_CLK
	TAS2555_PLL_P_VAL_REG, 0x01,	//PLL P = 1
//  TAS2555_PLL_J_VAL_REG,      0x10, //PLL J = 16
	TAS2555_PLL_J_VAL_REG, 0x0e,	//PLL J = 48 -> PLL_CLK = 1.536MHz * 48 = 73.728MHz
	TAS2555_PLL_D_VAL_MSB_REG, 0x00,	//PLL D = 0
	TAS2555_PLL_D_VAL_LSB_REG, 0x00,
	TAS2555_PLL_N_VAL_REG, 0x07,	//PLL N = 3 -> NDIV_CLK = 24.576MHz
	TAS2555_DAC_MADC_VAL_REG, 0x04,	//MDAC = 8
	TAS2555_CLK_MISC_REG, 0x20,	//DSP CLK = PLL out
//  TAS2555_ISENSE_DIV_REG,     0x40, //Isense div and MADC final divider configure auto
	TAS2555_ISENSE_DIV_REG, 0x40,	//Isense div and MADC final divider configure auto
//  TAS2555_RAMP_CLK_DIV_LSB_REG,   0x20, //ramp_clk divider = 32 so that 12.288MHz/32 = 384KHz
	TAS2555_RAMP_CLK_DIV_LSB_REG, 0x20,	//ramp_clk divider = 64 so that 24.576MHz/64 = 384KHz
	TAS2555_DSP_MODE_SELECT_REG, 0x22,	//DSP ROM mode 2, default coeffs

//  TAS2555_SPK_CTRL_REG,       0x74, //DAC channel gain
	TAS2555_SPK_CTRL_REG, 0x7C,	//DAC channel gain
//  TAS2555_POWER_CTRL2_REG,    0xA3, //power up
//  TAS2555_POWER_CTRL1_REG,    0xF8, //power up
//  TAS2555_MUTE_REG,       0x00, //unmute
//  TAS2555_SOFT_MUTE_REG,      0x00, //soft unmute
//  TAS2555_CLK_ERR_CTRL,       0x09, //enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};

#define TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX 3
static unsigned int p_tas2555_startup_data[] = {
	TAS2555_CLK_ERR_CTRL, 0x00,	//disable clock error detection on PLL
	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,
	TAS2555_POWER_CTRL2_REG, 0xA0,	//Class-D, Boost power up
	TAS2555_POWER_CTRL2_REG, 0xA3,	//Class-D, Boost, IV sense power up
	TAS2555_POWER_CTRL1_REG, 0xF8,	//PLL, DSP, clock dividers power up
	TAS2555_UDELAY, 2000,		//delay	
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_unmute_data[] = {
	TAS2555_MUTE_REG, 0x00,		//unmute
	TAS2555_SOFT_MUTE_REG, 0x00,	//soft unmute
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_shutdown_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	TAS2555_POWER_CTRL2_REG, 0x00,	//Class-D, Boost power down
	TAS2555_POWER_CTRL1_REG, 0x00,	//all power down
	0xFFFFFFFF, 0xFFFFFFFF
};

#if 0
static unsigned int p_tas2555_shutdown_clk_err[] = {
	TAS2555_CLK_ERR_CTRL, 0x09,	//enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};
#endif

static unsigned int p_tas2555_mute_DSP_down_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static int tas2555_dev_load_data(struct tas2555_priv *pTAS2555,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
		if (nRegister == TAS2555_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF){
			ret = pTAS2555->write(pTAS2555, nRegister, nData);
			if(ret < 0) {
				dev_err(pTAS2555->dev, "Reg Write err %d\n", ret);
				break;
			}
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);
	
	return ret;
}

int tas2555_load_default(struct tas2555_priv *pTAS2555)
{
	return tas2555_dev_load_data(pTAS2555, p_tas2555_default_data);
}
void tas2555_enable(struct tas2555_priv *pTAS2555, bool bEnable)
{
	dev_dbg(pTAS2555->dev, "Enable: %d\n", bEnable);
	if (bEnable) {
		if (!pTAS2555->mbPowerUp) {
			TConfiguration *pConfiguration;

			if (!pTAS2555->mbCalibrationLoaded) {
				tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
				pTAS2555->mbCalibrationLoaded = true;
			}
			dev_dbg(pTAS2555->dev, "Enable: load startup sequence\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			if (pTAS2555->mpFirmware->mpConfigurations) {
				pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
				tas2555_load_data(pTAS2555, &(pConfiguration->mData),
					TAS2555_BLOCK_CONF_POST_POWER);
				if (pTAS2555->mbLoadConfigurationPostPowerUp) {
					dev_dbg(pTAS2555->dev,	"Enable: load configuration: %s, %s\n",
						pConfiguration->mpName, pConfiguration->mpDescription);
					tas2555_load_data(pTAS2555, &(pConfiguration->mData),
						TAS2555_BLOCK_CONF_COEFF);
					pTAS2555->mbLoadConfigurationPostPowerUp = false;
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
						tas2555_load_block(pTAS2555, &(pTAS2555->mpCalFirmware->mpCalibrations[0].mBlock));
					}
				}
			}
			dev_dbg(pTAS2555->dev, "Enable: load unmute sequence\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
			pTAS2555->mbPowerUp = true;
		}
	} else {
		if (pTAS2555->mbPowerUp) {
			dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
			pTAS2555->mbPowerUp = false;
		}
	}
}

int tas2555_set_sampling_rate(struct tas2555_priv *pTAS2555, unsigned int nSamplingRate)
{
	TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2555->dev, "tas2555_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_dbg(pTAS2555->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2555->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		if (pConfiguration->mnSamplingRate == nSamplingRate) {
			dev_dbg(pTAS2555->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			tas2555_load_configuration(pTAS2555, nConfiguration, false);
			return 0;
		}
	}

	dev_dbg(pTAS2555->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

	return -EINVAL;
}

static void fw_print_header(struct tas2555_priv *pTAS2555, TFirmware * pFirmware)
{
	dev_info(pTAS2555->dev, "  FW Size       = %d", pFirmware->mnFWSize);
	dev_info(pTAS2555->dev, "  Checksum      = 0x%04X", pFirmware->mnChecksum);
	dev_info(pTAS2555->dev, "  PPC Version   = 0x%04X", pFirmware->mnPPCVersion);
	dev_info(pTAS2555->dev, "  FW  Version   = 0x%04X", pFirmware->mnFWVersion);
	dev_info(pTAS2555->dev, "  Timestamp     = %d", pFirmware->mnTimeStamp);
	dev_info(pTAS2555->dev, "  DDC Name      = %s", pFirmware->mpDDCName);
	dev_info(pTAS2555->dev, "  Description   = %s", pFirmware->mpDescription);
	dev_info(pTAS2555->dev, "  Device Family = %d", pFirmware->mnDeviceFamily);
	dev_info(pTAS2555->dev, "  Device        = %d", pFirmware->mnDevice);
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(struct tas2555_priv *pTAS2555, 
	TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };
	if (nSize < 102) {
		dev_err(pTAS2555->dev, "Firmware: Header too short");
		return -1;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		dev_err(pTAS2555->dev, "Firmware: Magic number doesn't match");
		return -1;
	}

	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;

	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	if ((pData - pDataStart) >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Header too short after DDC description");
		return -1;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	fw_print_header(pTAS2555, pFirmware);

	return pData - pDataStart;
}

static int fw_parse_block_data(TBlock * pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);
	pData += n;

	return pData - pDataStart;
}

static int fw_parse_data(TData * pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;

	pImageData->mpBlocks =
		kmalloc(sizeof(TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(&(pImageData->mpBlocks[nBlock]), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_pll_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nPLL;
	TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPLLs = kmalloc(sizeof(TPLL) * pFirmware->mnPLLs, GFP_KERNEL);
	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_block_data(&(pPLL->mBlock), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_program_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nProgram;
	TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPrograms =
		kmalloc(sizeof(TProgram) * pFirmware->mnPrograms, GFP_KERNEL);
	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_data(&(pProgram->mData), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_configuration_data(TFirmware * pFirmware,
	unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nConfiguration;
	TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;
//  pFirmware->mnConfigurations = fw_convert_number(pData);
//  pData += 4;

	pFirmware->mpConfigurations =
		kmalloc(sizeof(TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);
	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;

		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(&(pConfiguration->mData), pData);
		pData += n;
	}

	return pData - pDataStart;
}

int fw_parse_calibration_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nCalibration;
	TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpCalibrations =
		kmalloc(sizeof(TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);
	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_block_data(&(pCalibration->mBlock), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware,
	unsigned char *pData,
	unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2555, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2555->dev, "Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition = fw_parse_calibration_data(pFirmware, pData);

	return 0;
}

static void tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock)
{
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2555->dev, "TAS2555 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);
	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;

		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F)
			pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset),
				nData);
		if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;
			dev_dbg(pTAS2555->dev,
				"TAS2555 load block: nOffset = 0x81 -> sleep %d [ms]\n",
				nSleep);
			msleep(nSleep);
			dev_dbg(pTAS2555->dev,
				"TAS2555 load block: just woke up from sleep %d [ms]\n",
				nSleep);
		}
		if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1)
				pTAS2555->bulk_write(pTAS2555, TAS2555_REG(nBook, nPage,
						nOffset), pData + 3, nLength);
			else
				pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset),
					pData[3]);

			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}
}

static void tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType)
{
	unsigned int nBlock;
	TBlock *pBlock;

	dev_dbg(pTAS2555->dev,
		"TAS2555 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName,
		pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType)
			tas2555_load_block(pTAS2555, pBlock);
	}

	dev_dbg(pTAS2555->dev, "%s(), exit\n", __func__);
}

static void tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame)
{
	TConfiguration *pCurrentConfiguration;
	TConfiguration *pNewConfiguration;
	TPLL *pNewPLL;

	dev_dbg(pTAS2555->dev, "tas2555_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return;
	}

	if ((nConfiguration == pTAS2555->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_dbg(pTAS2555->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return;
	}

	pCurrentConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);

	if (pNewConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		return;
	}

	pNewPLL = &(pTAS2555->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);

	if (pTAS2555->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up -> mute and power down DSP before loading new configuration\n");
			//tas2555_i2c_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
			tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);

			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			dev_dbg(pTAS2555->dev, "TAS2555: power up TAS2555\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, post power up block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			dev_dbg(pTAS2555->dev, "TAS2555: unmute TAS2555\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
		} else {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
		}
		pTAS2555->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2555->dev,
			"TAS2555 was powered down -> set flag to load configuration data when OS powers up the TAS2555 the next time\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
		}
		pTAS2555->mbLoadConfigurationPostPowerUp = true;
	}

	pTAS2555->mnCurrentConfiguration = nConfiguration;
}

int tas2555_set_config(struct tas2555_priv *pTAS2555, int config)
{
	TConfiguration *pConfiguration;
	TProgram *pProgram;
	unsigned int nProgram = pTAS2555->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return -1;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2555->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		return -1;
	}

	tas2555_load_configuration(pTAS2555, nConfiguration, false);

	dev_dbg(pTAS2555->dev, "tas2555_configuration_put = %d\n",
		pTAS2555->mnCurrentConfiguration);

	return 0;
}

void tas2555_clear_firmware(TFirmware *pFirmware)
{
	unsigned int n, nn;
	if (!pFirmware) return;
	if (pFirmware->mpDescription) kfree(pFirmware->mpDescription);	

	for (n = 0; n < pFirmware->mnPLLs; n++)
	{
		kfree(pFirmware->mpPLLs[n].mpDescription);
		kfree(pFirmware->mpPLLs[n].mBlock.mpData);
	}
	kfree(pFirmware->mpPLLs);

	for (n = 0; n < pFirmware->mnPrograms; n++)
	{
		kfree(pFirmware->mpPrograms[n].mpDescription);
		kfree(pFirmware->mpPrograms[n].mData.mpDescription);
		for (nn = 0; nn < pFirmware->mpPrograms[n].mData.mnBlocks; nn++)
			kfree(pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData);
		kfree(pFirmware->mpPrograms[n].mData.mpBlocks);
	}
	kfree(pFirmware->mpPrograms);

	for (n = 0; n < pFirmware->mnConfigurations; n++)
	{
		kfree(pFirmware->mpConfigurations[n].mpDescription);
		kfree(pFirmware->mpConfigurations[n].mData.mpDescription);
		for (nn = 0; nn < pFirmware->mpConfigurations[n].mData.mnBlocks; nn++)
			kfree(pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData);
		kfree(pFirmware->mpConfigurations[n].mData.mpBlocks);
	}
	kfree(pFirmware->mpConfigurations);

	for (n = 0; n < pFirmware->mnCalibrations; n++)
	{
		kfree(pFirmware->mpCalibrations[n].mpDescription);
		kfree(pFirmware->mpCalibrations[n].mBlock.mpData);
	}
	kfree(pFirmware->mpCalibrations);

	memset(pFirmware, 0x00, sizeof(TFirmware));
}

static void tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName)
{
	int nResult;
	int nFile;
	mm_segment_t fs;
	unsigned char pBuffer[512];
	int nSize = 0;

	dev_dbg(pTAS2555->dev, "%s:\n", __func__);

	fs = get_fs();
	set_fs(KERNEL_DS);
	nFile = sys_open(pFileName, O_RDONLY, 0);

	dev_info(pTAS2555->dev, "TAS2555 calibration file = %s, handle = %d\n",
		pFileName, nFile);

	if (nFile >= 0) {
		nSize = sys_read(nFile, pBuffer, 512);
		sys_close(nFile);
	} else {
		dev_err(pTAS2555->dev, "TAS2555 cannot open calibration file: %s\n",
			pFileName);
	}

	set_fs(fs);

	if (!nSize)
		return;

	tas2555_clear_firmware(pTAS2555->mpCalFirmware);
		
	dev_info(pTAS2555->dev, "TAS2555 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2555, pTAS2555->mpCalFirmware, pBuffer, nSize);

	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 calibration file is corrupt\n");
		return;
	}

	dev_info(pTAS2555->dev, "TAS2555 calibration: %d calibrations\n",
		pTAS2555->mpCalFirmware->mnCalibrations);
}

void tas2555_fw_load(const struct firmware *pFW, struct tas2555_priv *pTAS2555)
{
	TConfiguration *pConfiguration;
	TPLL *pPLL;
	int nResult;
	unsigned int Value;

	dev_info(pTAS2555->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(pTAS2555->dev, "%s firmware is not loaded.\n",
			TAS2555_FW_FULL_NAME);
		return;
	}

	tas2555_clear_firmware(pTAS2555->mpFirmware);
	
	nResult = fw_parse(pTAS2555, pTAS2555->mpFirmware, 
		(unsigned char *) (pFW->data),	pFW->size);
	
	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 firmware is corrupt\n");
		return;
	}

	dev_info(pTAS2555->dev, "TAS2555 firmware: %d programs\n",
		pTAS2555->mpFirmware->mnPrograms);
	dev_info(pTAS2555->dev, "TAS2555 firmware: %d configurations\n",
		pTAS2555->mpFirmware->mnConfigurations);

	if (!pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555 firmware contains no programs\n");
		return;
	}

	if (!pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "TAS2555 firmware contains no configurations\n");
		return;
	}

	tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
	pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	udelay(1000);

	pTAS2555->mnCurrentBook = 0;
	pTAS2555->mnCurrentPage = 0;

	pTAS2555->write(pTAS2555, TAS2555_CRC_RESET_REG, 0x01);
	dev_info(pTAS2555->dev, "TAS2555 load base image: %s main block\n",
		pTAS2555->mpFirmware->mpPrograms[0].mpName);
	tas2555_load_data(pTAS2555, &(pTAS2555->mpFirmware->mpPrograms[0].mData),
		TAS2555_BLOCK_BASE_MAIN);
	pTAS2555->mbLoadConfigurationPostPowerUp = true;
	pTAS2555->mnCurrentConfiguration = 0;
	pTAS2555->mnCurrentProgram = 0;

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[0]);
	if (pConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"TAS2555 Configuration #0 doesn't have a valid PLL index #%d, max = %d\n",
			pConfiguration->mnPLL, pTAS2555->mpFirmware->mnPLLs);
		return;
	} else {
		pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
		dev_info(pTAS2555->dev,
			"TAS2555 load PLL: %s block for Configuration %s\n",
			pPLL->mpName, pConfiguration->mpName);
		tas2555_load_block(pTAS2555, &(pPLL->mBlock));
	}

	nResult = pTAS2555->read(pTAS2555, TAS2555_CRC_CHECKSUM_REG, &Value);
	if (nResult < 0)
		dev_err(pTAS2555->dev, "%d, ERROR!\n", __LINE__);
	else
		dev_info(pTAS2555->dev, "uCDSP Checksum: 0x%02x\n", Value);

	nResult = pTAS2555->read(pTAS2555, TAS2555_PLL_CLKIN_REG, &Value);
	dev_info(pTAS2555->dev, "TAS2555 PLL_CLKIN = 0x%02X\n", Value);
	p_tas2555_startup_data[TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX] = Value;

	tas2555_load_data(pTAS2555, &(pConfiguration->mData),
		TAS2555_BLOCK_CONF_PRE);
}

void tas2555_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *) pContext;

	tas2555_fw_load(pFW, pTAS2555);
	

	release_firmware(pFW);
}

int tas2555_set_program(struct tas2555_priv *pTAS2555,
	unsigned int nProgram)
{
	TPLL *pPLL;
	TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	bool bFound = false;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -1;
	}
	if (nProgram >= pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555: Program %d doesn't exist\n",
			nConfiguration);
		return -1;
	}
	
	if(pTAS2555->mnCurrentProgram == nProgram){
		dev_info(pTAS2555->dev, 
			"Program %d, no need to set again\n",
			nProgram);
		return 0;
	}

	nSampleRate = 
		pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration].mnSamplingRate;
	pTAS2555->mnCurrentProgram = nProgram;

	pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);

	udelay(1000);
	pTAS2555->mnCurrentBook = 0;
	pTAS2555->mnCurrentPage = 0;

	tas2555_load_data(pTAS2555,
		&(pTAS2555->mpFirmware->mpPrograms[nProgram].mData),
		TAS2555_BLOCK_BASE_MAIN);

	nConfiguration = 0;
	while (!bFound && (nConfiguration < pTAS2555->mpFirmware->mnConfigurations)) {
		if ((pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram ==
			nProgram) 
				&&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate ==
					nSampleRate))
		{
			bFound = true;
			dev_err(pTAS2555->dev,
				"%s, find configuration %d\n",
				__FUNCTION__, nConfiguration);
		}
		else
			nConfiguration++;
	}

	if (bFound) {
		pTAS2555->mnCurrentConfiguration = nConfiguration;

		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
		dev_dbg(pTAS2555->dev,
			"TAS2555 load PLL: %s block for Configuration %s\n",
			pPLL->mpName, pConfiguration->mpName);
		tas2555_load_block(pTAS2555, &(pPLL->mBlock));
		tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_PRE);

		if (pTAS2555->mbPowerUp){
			tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			tas2555_load_data(pTAS2555, &(pConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
		}
		tas2555_load_configuration(pTAS2555, nConfiguration, true);
		if (pTAS2555->mbPowerUp)
			tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);

		dev_dbg(pTAS2555->dev,
			"tas2555_program_put = %d, found configuration = %d, %d\n",
			pTAS2555->mnCurrentProgram, bFound, nConfiguration);
	}else{
		dev_err(pTAS2555->dev, 
			"Program %d, no valid configuration found\n",
			nProgram);
	}

	return 0;
}

int tas2555_set_calibration(struct tas2555_priv *pTAS2555,
	unsigned int nCalibration)
{
	if ((!pTAS2555->mpFirmware->mpPrograms) || (!pTAS2555->mpFirmware->mpConfigurations)) 
	{
		printk(KERN_ERR "TAS2555: Firmware not loaded\n\r");
		return -1;
	}

	if (nCalibration == 0x00FF)
	{
		printk(KERN_ERR "TAS2555: load new calibration file %s\n\r", TAS2555_CAL_NAME); 	
		tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
		nCalibration = 0;
	}

	if (nCalibration >= pTAS2555->mpFirmware->mnCalibrations) {
		printk(KERN_ERR "TAS2555: Calibration %d doesn't exist\n\r", nCalibration);
		return -1;
	}

	pTAS2555->mnCurrentCalibration = nCalibration;
	tas2555_load_block(pTAS2555, 
		&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));

	return 0;
}

void tas2555_load_fs_firmware(struct tas2555_priv *pTAS2555,
	char *pFileName)
{
	int nFile;
	mm_segment_t fs;
	struct firmware fw;
	unsigned char *p_kBuf;
	int nSize = 0;
	unsigned int count = 30000;

	dev_dbg(pTAS2555->dev, "%s:\n", __func__);
	p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
	if(p_kBuf == NULL){
		dev_err(pTAS2555->dev, "not enough memory for %d bytes\n", count);
		return;
	}
	
	fs = get_fs();
	set_fs(KERNEL_DS);
	nFile = sys_open(pFileName, O_RDONLY, 0);

	dev_info(pTAS2555->dev, "TAS2555 firmware file = %s, handle = %d\n",
		pFileName, nFile);
		
	if (nFile >= 0) {
		nSize = sys_read(nFile, p_kBuf, count);
		sys_close(nFile);
	} else {
		dev_err(pTAS2555->dev, "TAS2555 cannot open firmware file: %s\n",
			pFileName);
	}

	set_fs(fs);

	if(nSize == count){
		dev_err(pTAS2555->dev, 
			"buffer length (%d) may not contain all the firmware\n",
			count);
	}
	
	if (!nSize){
		dev_err(pTAS2555->dev, 
			"firmware size error\n");
	}else{
		dev_info(pTAS2555->dev, "TAS2555 firmware size= %d\n",
				nSize);		
	
		fw.size = nSize;
		fw.data = p_kBuf;
		tas2555_fw_load(&fw, pTAS2555);
		
		dev_info(pTAS2555->dev, "TAS2555 firmware: %d configurations\n",
			pTAS2555->mpFirmware->mnConfigurations);		
	}
	
	kfree(p_kBuf);		
}


static int tas2555_file_open(struct inode *inode, struct file *file)
{
	struct tas2555_priv *pTAS2555 = g_tas2555;
	if (!try_module_get(THIS_MODULE)) return -ENODEV;

	printk("tas2555_file_open .........");
	mt_set_gpio_mode(GPIO_TAS2555_MCLK_PIN,GPIO_MODE_06); 
	mt_set_gpio_pull_enable(GPIO_TAS2555_MCLK_PIN,1);
	//mt_set_gpio_out(GPIO_TAS2555_MCLK_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_TAS2555_I2SDI_PIN,GPIO_MODE_02); 
        mt_set_gpio_dir(GPIO_TAS2555_I2SDI_PIN,GPIO_DIR_IN);

	file->private_data = (void*)pTAS2555;
	return 0;
}

static int tas2555_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void*)NULL;
	module_put(THIS_MODULE);

	return 0;
}

static ssize_t tas2555_file_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)file->private_data;
	int ret = 0;
	unsigned int nValue = 0;
	unsigned char value = 0;
	unsigned char *p_kBuf = NULL;

	mutex_lock(&pTAS2555->file_lock);
	switch(pTAS2555->mnDBGCmd)
	{
		case TIAUDIO_CMD_REG_READ:
		{
			if(g_logEnable) dev_info(pTAS2555->dev,
				"TIAUDIO_CMD_REG_READ: current_reg = 0x%x, count=%d\n", pTAS2555->mnCurrentReg, (int)count);
			if(count == 1){
				ret = pTAS2555->read(pTAS2555, pTAS2555->mnCurrentReg, &nValue);
				if( 0 > ret) {
					dev_err(pTAS2555->dev, "dev read fail %d\n", ret);
					break;
				}			
				
				value = (u8)nValue;
				if(g_logEnable) dev_info(pTAS2555->dev,
					"TIAUDIO_CMD_REG_READ: nValue=0x%x, value=0x%x\n", 
					nValue, value);
				ret = copy_to_user(buf, &value, 1);
				if (0 != ret) {
					/* Failed to copy all the data, exit */
					dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
				}	
			}else if(count > 1){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					ret = pTAS2555->bulk_read(pTAS2555, pTAS2555->mnCurrentReg, p_kBuf, count);
					if( 0 > ret) {
						dev_err(pTAS2555->dev, "dev bulk read fail %d\n", ret);
					}else{						
						ret = copy_to_user(buf, p_kBuf, count);
						if (0 != ret) {
							/* Failed to copy all the data, exit */
							dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
						}
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
				}
			}
		}
		break;
		
		case TIAUDIO_CMD_PROGRAM:
		{
			if(count == PROGRAM_BUF_SIZE){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					TProgram * pProgram = 
						&(pTAS2555->mpFirmware->mpPrograms[pTAS2555->mnCurrentProgram]);
					
					p_kBuf[0] = pTAS2555->mpFirmware->mnPrograms;							
					p_kBuf[1] = pTAS2555->mnCurrentProgram;					
					memcpy(&p_kBuf[2], pProgram->mpName, FW_NAME_SIZE);
					strcpy(&p_kBuf[2+FW_NAME_SIZE], pProgram->mpDescription);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
				}				
			}else{
				dev_err(pTAS2555->dev, "read buffer not sufficient\n");
			}
		}
		break;
		
		case TIAUDIO_CMD_CONFIGURATION:
		{
			if(count == CONFIGURATION_BUF_SIZE){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					TConfiguration * pConfiguration = 
						&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);

					p_kBuf[0] = pTAS2555->mpFirmware->mnConfigurations;											
					p_kBuf[1] = pTAS2555->mnCurrentConfiguration;					
					memcpy(&p_kBuf[2], pConfiguration->mpName, FW_NAME_SIZE);
					p_kBuf[2+FW_NAME_SIZE] = pConfiguration->mnProgram;
					p_kBuf[3+FW_NAME_SIZE] = pConfiguration->mnPLL;
					p_kBuf[4+FW_NAME_SIZE] = (pConfiguration->mnSamplingRate&0x000000ff);
					p_kBuf[5+FW_NAME_SIZE] = ((pConfiguration->mnSamplingRate&0x0000ff00)>>8);
					p_kBuf[6+FW_NAME_SIZE] = ((pConfiguration->mnSamplingRate&0x00ff0000)>>16);
					p_kBuf[7+FW_NAME_SIZE] = ((pConfiguration->mnSamplingRate&0xff000000)>>24);
					strcpy(&p_kBuf[8+FW_NAME_SIZE], pConfiguration->mpDescription);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
				}				
			}else{
				dev_err(pTAS2555->dev, "read buffer not sufficient\n");
			}
		}
		break;
		
		case TIAUDIO_CMD_FW_TIMESTAMP:
		{
			if(count == 4){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){
					p_kBuf[0] = (pTAS2555->mpFirmware->mnTimeStamp&0x000000ff);
					p_kBuf[1] = ((pTAS2555->mpFirmware->mnTimeStamp&0x0000ff00)>>8);
					p_kBuf[2] = ((pTAS2555->mpFirmware->mnTimeStamp&0x00ff0000)>>16);
					p_kBuf[3] = ((pTAS2555->mpFirmware->mnTimeStamp&0xff000000)>>24);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
				}	
			}
		}
		break;
		
		case TIAUDIO_CMD_CALIBRATION:
		{
			if(g_logEnable) dev_info(pTAS2555->dev,
						"TIAUDIO_CMD_CALIBRATION: count = %d\n", 
						(int)count);

			if(count == 1){
				unsigned char curCal = pTAS2555->mnCurrentCalibration;
				ret = copy_to_user(buf, &curCal, 1);
				if (0 != ret) {
					/* Failed to copy all the data, exit */
					dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
				}	
			}
		}
		break;
		
		case TIAUDIO_CMD_SAMPLERATE:
		{
			if(g_logEnable) dev_info(pTAS2555->dev,
						"TIAUDIO_CMD_SAMPLERATE: count = %d\n", 
						(int)count);
			if(count == 4){
				p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
				if(p_kBuf != NULL){				
					TConfiguration *pConfiguration = 
						&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);

					p_kBuf[0] = (pConfiguration->mnSamplingRate&0x000000ff);
					p_kBuf[1] = ((pConfiguration->mnSamplingRate&0x0000ff00)>>8);
					p_kBuf[2] = ((pConfiguration->mnSamplingRate&0x00ff0000)>>16);
					p_kBuf[3] = ((pConfiguration->mnSamplingRate&0xff000000)>>24);
					
					ret = copy_to_user(buf, p_kBuf, count);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					}
					
					kfree(p_kBuf);
				}else{
					dev_err(pTAS2555->dev, "read no mem\n");
				}	
			}
		}
		break;
		
		case TIAUDIO_CMD_BITRATE:
		{
			if(g_logEnable) dev_info(pTAS2555->dev,
						"TIAUDIO_CMD_BITRATE: count = %d\n", 
						(int)count);

			if(count == 1){
				unsigned int dac_format = 0;
				unsigned char bitRate = 0;
				ret = pTAS2555->read(pTAS2555, 
					TAS2555_ASI1_DAC_FORMAT_REG, &dac_format);
				if(ret >=0){	
					bitRate = (dac_format&0x18)>>3;
					if(bitRate == 0) bitRate = 16;
					else if(bitRate == 1) bitRate = 20;
					else if(bitRate == 2) bitRate = 24;
					else if(bitRate == 3) bitRate = 32;
					ret = copy_to_user(buf, &bitRate, 1);
					if (0 != ret) {
					/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					}
				}					
			}
		}
		break;
		
		case TIAUDIO_CMD_DACVOLUME:
		{
			if(g_logEnable) dev_info(pTAS2555->dev,
						"TIAUDIO_CMD_DACVOLUME: count = %d\n", 
						(int)count);

			if(count == 1){
				unsigned int value = 0;
				unsigned char volume = 0;
				ret = pTAS2555->read(pTAS2555, 
					TAS2555_SPK_CTRL_REG, &value);
				if(ret >=0){	
					volume = (value&0x78)>>3;					
					ret = copy_to_user(buf, &volume, 1);
					if (0 != ret) {
					/* Failed to copy all the data, exit */
						dev_err(pTAS2555->dev, "copy to user fail %d\n", ret);
					}
				}					
			}
		}
		break;

	}
	 pTAS2555->mnDBGCmd = 0;

	mutex_unlock(&pTAS2555->file_lock); 
	return count;
}

static ssize_t tas2555_file_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)file->private_data;
	int ret = 0;
//	unsigned int value = 0;
	unsigned char *p_kBuf = NULL;
//	unsigned char cmd = 0;
	unsigned int reg = 0;
	unsigned int len = 0;

	mutex_lock(&pTAS2555->file_lock);
		
	p_kBuf = (unsigned char *)kzalloc(count, GFP_KERNEL);
	if(p_kBuf == NULL) {
		dev_err(pTAS2555->dev, "write no mem\n");
		goto err;
	}
	
	ret = copy_from_user(p_kBuf, buf, count);
	if (0 != ret) {
		dev_err(pTAS2555->dev,"copy_from_user failed.\n");
		goto err;
	}

	pTAS2555->mnDBGCmd = p_kBuf[0];
	switch(pTAS2555->mnDBGCmd)
	{
		case TIAUDIO_CMD_REG_WITE:
		if(count > 5){
			reg = ((unsigned int)p_kBuf[1] << 24) + 
				((unsigned int)p_kBuf[2] << 16) + 
				((unsigned int)p_kBuf[3] << 8) + 
				(unsigned int)p_kBuf[4];
			len = count - 5;
			if(len == 1){
				ret = pTAS2555->write(pTAS2555, reg, p_kBuf[5]);
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_REG_WITE, Reg=0x%x, Val=0x%x\n", 
					reg, p_kBuf[5]);
			}else{
				ret = pTAS2555->bulk_write(pTAS2555, reg, &p_kBuf[5], len);
			}
		}else{
			dev_err(pTAS2555->dev,"%s, write len fail, count=%d.\n", 
				__FUNCTION__, (int)count);
		}
		pTAS2555->mnDBGCmd = 0;
		break;
		
		case TIAUDIO_CMD_REG_READ:
		if(count == 5){
			pTAS2555->mnCurrentReg = ((unsigned int)p_kBuf[1] << 24) + 
				((unsigned int)p_kBuf[2] << 16) + 
				((unsigned int)p_kBuf[3] << 8) 	+ 
				(unsigned int)p_kBuf[4];		
			if(g_logEnable){
				dev_info(pTAS2555->dev,
					"TIAUDIO_CMD_REG_READ, whole=0x%x\n", 
					pTAS2555->mnCurrentReg);
			}	
		}else{
			dev_err(pTAS2555->dev,"read len fail.\n");
		}			
		break;
		
		case TIAUDIO_CMD_DEBUG_ON:
		{
			if(count == 2){
				g_logEnable = p_kBuf[1];
			}
			pTAS2555->mnDBGCmd = 0;
		}
		break;
		
		case TIAUDIO_CMD_PROGRAM:
		{
			if(count == 2){
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_PROGRAM, set to %d\n", 
					p_kBuf[1]);
				tas2555_set_program(pTAS2555, p_kBuf[1]);
				pTAS2555->mnDBGCmd = 0;
			}
		}
		break;
		
		case TIAUDIO_CMD_CONFIGURATION:
		{
			if(count == 2){
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_CONFIGURATION, set to %d\n", 
					p_kBuf[1]);
				tas2555_set_config(pTAS2555, p_kBuf[1]);
				pTAS2555->mnDBGCmd = 0;
			}
		}
		break;	
		
		case TIAUDIO_CMD_FW_TIMESTAMP:
		/*let go*/
		break;
		
		case TIAUDIO_CMD_CALIBRATION:
		{
			if(count == 2){
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_CALIBRATION, set to %d\n", 
					p_kBuf[1]);
				tas2555_set_calibration(pTAS2555, p_kBuf[1]);
				pTAS2555->mnDBGCmd = 0;
			}
		}
		break;
		
		case TIAUDIO_CMD_SAMPLERATE:
		{
			if(count == 5){
				unsigned int nSampleRate = ((unsigned int)p_kBuf[1] << 24) + 
					((unsigned int)p_kBuf[2] << 16) + 
					((unsigned int)p_kBuf[3] << 8) 	+ 
					(unsigned int)p_kBuf[4];	
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_SAMPLERATE, set to %d\n", 
					nSampleRate);

				tas2555_set_sampling_rate(pTAS2555, nSampleRate);
			}
		}
		break;


		case TIAUDIO_CMD_BITRATE:
		{
			if(count == 2){
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_BITRATE, set to %d\n", 
					p_kBuf[1]);

				tas2555_set_bit_rate(pTAS2555, p_kBuf[1]);
			}
		}
		break;

		case TIAUDIO_CMD_DACVOLUME:
		{
			if(count == 2){
				unsigned char volume = (p_kBuf[1] & 0x0f);
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_DACVOLUME, set to %d\n", 
					volume);

				pTAS2555->update_bits(pTAS2555, 
					TAS2555_SPK_CTRL_REG, 0x78, volume<<3);
			}
		}
		break;
		
		case TIAUDIO_CMD_SPEAKER:
		{
			if(count == 2){
				if(g_logEnable)
					dev_info(pTAS2555->dev, 
					"TIAUDIO_CMD_SPEAKER, set to %d\n", 
					p_kBuf[1]);
				tas2555_enable(pTAS2555, (p_kBuf[1]>0));
			}
		}
		break;	
		
		case TIAUDIO_CMD_FW_RELOAD:
		{
			if(count == 1){
				dev_err(pTAS2555->dev, 
					"TIAUDIO_CMD_FW_RELOAD");
				//tas2555_load_fs_firmware(pTAS2555, TAS2555_FW_FULL_NAME);
			}
				
		}
		break;		
		default:
			pTAS2555->mnDBGCmd = 0;
		break;
	}

err:
	if(p_kBuf != NULL)
		kfree(p_kBuf);
	
	mutex_unlock(&pTAS2555->file_lock);
	return count;
}

static long tas2555_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tas2555_priv *pTAS2555 = file->private_data;
	//void __user *user_arg = (void __user *)arg;
	int ret = 0;
	
	mutex_lock(&pTAS2555->file_lock);
	switch (cmd) {
		case SMARTPA_SPK_DAC_VOLUME:
		{
			u8 volume = (arg & 0x0f);
			pTAS2555->update_bits(pTAS2555, 
				TAS2555_SPK_CTRL_REG, 0x78, volume<<3);
		}		
		break;
	
		case SMARTPA_SPK_POWER_ON:
		{
			//msm8x16_quin_mi2s_clk_ctl(true);
			//msleep(3);		
			tas2555_enable(pTAS2555, true);
		}
		break;
		
		case SMARTPA_SPK_POWER_OFF:
		{
			tas2555_enable(pTAS2555, false);
			//msleep(3);
			//msm8x16_quin_mi2s_clk_ctl(false);	
		}		
		break;
		
		case SMARTPA_SPK_SWITCH_PROGRAM:
		{
			tas2555_set_program(pTAS2555, arg);
		}
		break;
		
		case SMARTPA_SPK_SWITCH_CONFIGURATION:
		{
			tas2555_set_config(pTAS2555, arg);
		}
		break;
		
		case SMARTPA_SPK_SWITCH_CALIBRATION:
		{
			tas2555_set_calibration(pTAS2555, arg);
		}
		break;
		
		case SMARTPA_SPK_SET_SAMPLERATE:
		{
			tas2555_set_sampling_rate(pTAS2555, arg);
		}
		break;		

		case SMARTPA_SPK_SET_BITRATE:
		{
			tas2555_set_bit_rate(pTAS2555, arg);
		}
		break;	
	}
	
	mutex_unlock(&pTAS2555->file_lock);
	return ret;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = tas2555_file_read,
	.write = tas2555_file_write,
	.unlocked_ioctl = tas2555_file_unlocked_ioctl,
	.open = tas2555_file_open,
	.release = tas2555_file_release,
};

#define MODULE_NAME	"tas2555"
static struct miscdevice tas2555_misc =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODULE_NAME,
	.fops = &fops,
};

static int tas2555_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2555_priv *pTAS2555;
	unsigned int n;
	int nResult;

	int ret;
    
    dev_err(&pClient->dev, "%s, enter\n", __FUNCTION__);
    g_client = pClient;
//  regmap_config = &tas2555_i2c_regmap;

	pClient->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&pClient->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	
	if(!gpDMABuf_va){
		dev_err(&pClient->dev, "%s, line %d, no memory for tas2555\n", __FUNCTION__, __LINE__);
	}
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);

	pTAS2555 = devm_kzalloc(&pClient->dev, sizeof(struct tas2555_priv), GFP_KERNEL);
	if (!pTAS2555)
		return -ENOMEM;

	pTAS2555->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2555);
	dev_set_drvdata(&pClient->dev, pTAS2555);
    pTAS2555->client = pClient;                        

    g_tas2555 = pTAS2555;

    dev_err(pTAS2555->dev, "tas2555_i2c_probe111: HW reset TAS2555\n");
	/* Hardware Reset the chip */    
     mt_set_gpio_mode(GPIO_TAS2555_RST_PIN,GPIO_MODE_00);      
     mt_set_gpio_dir(GPIO_TAS2555_RST_PIN,GPIO_DIR_OUT);    
     mt_set_gpio_out(GPIO_TAS2555_RST_PIN,GPIO_OUT_ONE);    
     msleep(1);	
     mt_set_gpio_out(GPIO_TAS2555_RST_PIN, GPIO_OUT_ZERO);	
     msleep(10);	
     mt_set_gpio_out(GPIO_TAS2555_RST_PIN,GPIO_OUT_ONE);
     msleep(1);	

	pTAS2555->read = tas2555_dev_read;
	pTAS2555->write = tas2555_dev_write;
	pTAS2555->bulk_read = tas2555_dev_bulk_read;
	pTAS2555->bulk_write = tas2555_dev_bulk_write;
	pTAS2555->update_bits = tas2555_dev_update_bits;
	pTAS2555->set_config = tas2555_set_config;
	
	mutex_init(&pTAS2555->dev_lock);
	
	/* Reset the chip */ /*mtk_i2c*/
	nResult = tas2555_dev_write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	if(nResult < 0){
		dev_err(&pClient->dev, "I2C communication ERROR: %d\n",
			nResult);
		return nResult;
	}
	udelay(1000);

	pTAS2555->mpFirmware =
		devm_kzalloc(&pClient->dev, sizeof(TFirmware),
		GFP_KERNEL);
	if (!pTAS2555->mpFirmware)
		return -ENOMEM;

	pTAS2555->mpCalFirmware =
		devm_kzalloc(&pClient->dev, sizeof(TFirmware),
		GFP_KERNEL);
	if (!pTAS2555->mpCalFirmware)
		return -ENOMEM;

	pTAS2555->mnCurrentPage = 0;
	pTAS2555->mnCurrentBook = 0;

	nResult = tas2555_dev_read(pTAS2555, TAS2555_REV_PGID_REG, &n);
	dev_err(&pClient->dev, "TAS2555 PGID: 0x%02x\n", n);

	tas2555_load_default(pTAS2555);
	mutex_init(&pTAS2555->file_lock);

	ret = device_create_file(pTAS2555->dev, &dev_attr_tas2555_data);
	if (ret) {
		pr_err("%s: Error to create reg_data\n", __FUNCTION__);
	}
	//nResult = request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
	//	&pClient->dev, GFP_KERNEL, pTAS2555, tas2555_fw_ready);

	pTAS2555->mbTILoadActive = false;

	
	misc_register(&tas2555_misc);
#ifdef ENABLE_TILOAD
	tiload_driver_init(pTAS2555);
#endif

	nResult = 0;
    dev_info(pTAS2555->dev, "%s, leave\n", __FUNCTION__);

	return nResult;
}


static int tas2555_i2c_remove(struct i2c_client *pClient)
{
	misc_deregister(&tas2555_misc);
	return 0;
}

static const struct i2c_device_id tas2555_i2c_id[] = {
	{"tas2555", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tas2555_i2c_id);


static const struct of_device_id tas2555_of_match[] = {
	{.compatible = "mediatek,tas2555"},
	{},
};

MODULE_DEVICE_TABLE(of, tas2555_of_match);


static struct i2c_driver tas2555_i2c_driver = {
	.driver = {
			.name = "tas2555",
			.owner = THIS_MODULE,

			.of_match_table = of_match_ptr(tas2555_of_match),

		},
	.probe = tas2555_i2c_probe,
	.remove = tas2555_i2c_remove,
	.id_table = tas2555_i2c_id,
};
static int __init tas2555_modinit(void)
{
	printk("tas2555_modinit.............\n");
	i2c_add_driver(&tas2555_i2c_driver);
	return 0;
}
module_init(tas2555_modinit);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TAS2555 Smart Amplifier driver");
MODULE_LICENSE("GPLv2");
