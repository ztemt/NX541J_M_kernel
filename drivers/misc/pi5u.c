/*
 * pi5u.c (v1.1) -- PI5U USB TYPE-C Controller device driver 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/reboot.h>

#include "pi5u.h"

#define LOG_TAG "PERICOM-PI5U"
#define DEBUG_ON //DEBUG SWITCH


#define USB_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define USB_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define USB_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define USB_LOG_DEBUG(fmt, args...)
#endif

//#define NUBIA_TYPEC_DEBUG
#ifdef NUBIA_TYPEC_DEBUG
#define PR_DEBUG_NUBIA_TYPEC(fmt, args...) printk(KERN_ALERT "nubia_OTG: FILE:%s LINE:%d FUNC:%s " fmt, __FILE__ , __LINE__ , __func__ , ##args)
#else
#define PR_DEBUG_NUBIA_TYPEC(fmt, args...) do{}while(0)
#endif

#define PERICOM_TYPEC_ERROR_STATE       0x4
/*PERICOM Type-C chip ID*/
#define PI5U_30216A_ID          0x0
#define PI5U_30216D_ID          0x20

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_DEV_ID              0x01
#define REG_CON                 0x02
#define REG_INT                 0x03
#define REG_CC_STAT             0x04

/******************************************************************************
* Register bits
*******************************************************************************/
/*	  REG_DEV_ID (0x01)    */
/*    REG_CON (0x02)    */
#define TYPE_INT_MASK           0x01
#define TYPE_MODE_SEL_SHIFT     1
#define TYPE_MODE_SEL           (0x03 << TYPE_MODE_SEL_SHIFT)

/******************************************************************************/

struct pi5u_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct pi5u_platform_data	 *platform_data;
	struct mutex		mutex;
	struct class *pusb_class;
	int irq;
	enum pi5u_type pusb_type;
};

static int pi5u_poweroff(struct notifier_block *nb, unsigned long event, void *unused);
static struct notifier_block pi5u_poweroff_notifier = {
        .notifier_call = pi5u_poweroff,
};

static int pi5u_poweroff(struct notifier_block *nb, unsigned long event, void *unused)
{
    switch (event) {
        case SYS_RESTART:
            USB_LOG_INFO("SYS_RESTART\n");
            break;
        case SYS_HALT:
            USB_LOG_INFO("SYS_HALT\n");
            break;
        case SYS_POWER_OFF:
            USB_LOG_INFO("SYS_POWER_OFF\n");
            break;
        default:
            break;
        }
        return NOTIFY_DONE;
}

static int pi5u_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		USB_LOG_ERROR("failed to read reg(0x%x), ret(%d)\n",reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
static int pi5u_30216D_setDRP(struct i2c_client *i2c )
{
    u8 regbuf[4] = { 0 };
    u8 writebuf[2] = { 0 };
	int ret;
    ret = i2c_smbus_read_i2c_block_data(i2c,REG_DEV_ID,4, regbuf);
    if(ret <= 0)
    {
        USB_LOG_ERROR("Read i2c block data error !");
        goto error;
    }

    writebuf[0] = regbuf[1] | 0x4;
    writebuf[1] = PI5U_30216D_ID;
    ret = i2c_smbus_write_i2c_block_data(i2c, REG_DEV_ID, 2, writebuf);
    if(ret != 0)
    {
        USB_LOG_ERROR("Write i2c block data error !");
        goto error;
    }
    ret = i2c_smbus_read_i2c_block_data(i2c,REG_DEV_ID,4, regbuf);
    USB_LOG_INFO("ret = %d,0X%x,0X%x,0X%x,0X%x\n",ret,regbuf[0],regbuf[1],regbuf[2],regbuf[3]);
    return 0;
error:
        USB_LOG_ERROR("PI5U 30216D set DRP mode failed!");
        return -1;
}

/*
static int pi5u_write_reg(struct i2c_client *i2c, BYTE reg, BYTE value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		USB_LOG_ERROR("failed to write reg(0x%x), ret(%d)\n",reg, ret);

	return ret;
}
*/
static void pi5u_source_cb(bool attach, int bc_lvl)
{
	USB_LOG_INFO("attached ->%d, BC_LVL ->%d\n",attach, bc_lvl);
}

static void pi5u_sink_cb(bool attach)
{
	USB_LOG_INFO("attached -> %d\n",attach);
    // VBUS switch control
}

static void pi5u_check_type(struct pi5u_info *info, BYTE type)
{
    const char *string;

    if((type & TYPE_MODE_SEL) == PI5U_TYPE_UFP)
    {
        info->pusb_type = PI5U_TYPE_UFP;
        string = "UFP";
    }
	else if((type & TYPE_MODE_SEL) == PI5U_TYPE_DFP)
	{
	    info->pusb_type = PI5U_TYPE_DFP;
            string = "DFP";
	}
	else if((type & TYPE_MODE_SEL) == PI5U_TYPE_DRP)
	{
	    info->pusb_type = PI5U_TYPE_DRP;
            string = "DRP";
	}
	else
	{
	    printk("%s: No device type!\n", __func__);
            return;
	}
	
       printk("%s: Attached TYPE is %s\n", __func__, string);
}

static ssize_t show_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	BYTE rdata = 0;
 	struct pi5u_info *info = dev_get_drvdata(dev);

	pi5u_read_reg(info->i2c, REG_CON, &rdata);
	pi5u_check_type(info, rdata);

	switch(info->pusb_type){
		case PI5U_TYPE_UFP:
			return sprintf(buf, "PI5U_TYPE_UFP\n");
		case PI5U_TYPE_DFP:
			return sprintf(buf, "PI5U_TYPE_DFP\n");
		case PI5U_TYPE_DRP:
			return sprintf(buf, "PI5U_TYPE_DRP\n");		
		default:
			return sprintf(buf, "TYPE ERROR\n");
	}
	
}

static DEVICE_ATTR(type, 0444, show_type, NULL);

#define MTK_ANDROID_M_GPIO_DTS_CONFIG
#ifdef MTK_ANDROID_M_GPIO_DTS_CONFIG
static struct platform_device *pi5u_pinctl_dev = NULL;
static struct pinctrl *pi5u_pinctrl = NULL;
static struct pinctrl_state *pi5u_pinctl_en = NULL;

static int mt_pi5u_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	pi5u_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pi5u_pinctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(pi5u_pinctrl);
		goto failed;
	}

	pi5u_pinctl_en = pinctrl_lookup_state(pi5u_pinctrl, "pi5u_gpio_en");
	if (IS_ERR(pi5u_pinctl_en)) {
		ret = PTR_ERR(pi5u_pinctl_en);
		dev_err(&pdev->dev, "%s : cannot find pinctl en\n", __func__);
		goto failed;
	}

	ret = pinctrl_select_state(pi5u_pinctrl, pi5u_pinctl_en);
	usleep_range(900, 1000);

	dev_err(&pdev->dev, "%s : suss\n", __func__);

failed:

	return ret;
}


static int mt_pi5u_pinctrl_probe(struct platform_device *pdev)
{
	int ret = 0;

	pi5u_pinctl_dev = pdev;

	dev_err(&pdev->dev, "%s : pi5u_pinctl_dev=%p\n", __func__, pi5u_pinctl_dev);

	/* pinctrl init */
	ret = mt_pi5u_pinctrl_init(pdev);

	return 0;
}

static int mt_pi5u_pinctl_remove(struct platform_device *pdev)
{
	dev_err(&pdev->dev, "%s : pdev=%p\n", __func__, pdev);
	return 0;
}

/*  platform driver */
static const struct of_device_id pi5u_pinctl_dev_of_match[] = {
	{.compatible = "mediatek,pi5u_pinctl",},
	{},
};

static struct platform_driver pi5u_pinctl_platform_driver = {
	.probe = mt_pi5u_pinctrl_probe,
	.remove = mt_pi5u_pinctl_remove,
	.driver = {
		   .name = "pi5u_pinctl",
		   .owner = THIS_MODULE,
		   .of_match_table = pi5u_pinctl_dev_of_match,
		   },
};

#endif

static int pi5u_read_device_id(struct pi5u_info *info,u8 *device_id)
{
	int retry_times = 3;
	int res = -1;
	USB_LOG_INFO("start to read device id\n");
	while(retry_times--) {
		res = pi5u_read_reg(info->i2c, REG_DEV_ID, device_id);
		if (res >= 0) {
			USB_LOG_INFO("device_id = %x\n", *device_id);
			if (*device_id == PI5U_30216A_ID || *device_id == PI5U_30216D_ID) {
				USB_LOG_INFO("typeC IC is PERICOM PI5U !\n");
				return 0;
			}
		}
	}
	USB_LOG_ERROR("read device id failed\n");
	return res;

}

static int pi5u_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pi5u_info *info=NULL;
	struct pi5u_platform_data *pdata=client->dev.platform_data;
//	u8 regbuf[4] = { 0 };
	u8 writebuf[2] = { 0 };
	int ret = 0;
        u8 chip_id = 0;
	PR_DEBUG_NUBIA_TYPEC("probe start!\n");
	USB_LOG_DEBUG("probe start!\n");
	info = kzalloc(sizeof(struct pi5u_info), GFP_KERNEL);
	if (!info) {
		USB_LOG_ERROR("kzalloc pi5u_info failed\n");
		ret= -ENOMEM;
		goto exit;	
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
	USB_LOG_ERROR("i2c_check_functionality error");
	ret = -ENODEV;
	goto exit;
	}
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct pi5u_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			USB_LOG_ERROR("Failed to allocate memory\n");
			ret = -ENOMEM;
			goto exit_platform_failed;
		}

		client->dev.platform_data = pdata;
	} else{
		pdata = client->dev.platform_data;
		if (!pdata) {
			USB_LOG_ERROR("No platform data\n"); 
			ret = -ENODEV;
			goto exit_platform_failed;
		}
	}

	USB_LOG_INFO("parse device tree success!\n");
	pdata->source_cb = pi5u_source_cb;
	pdata->sink_cb = pi5u_sink_cb;

    register_reboot_notifier(&pi5u_poweroff_notifier);
       
    info->i2c = client;
	i2c_set_clientdata(client, info);
	info->platform_data = pdata;

	
	mutex_init(&info->mutex);
    ret = pi5u_read_device_id(info,&chip_id);
    if(ret < 0){
        USB_LOG_ERROR("pi5u_read_device_id fail!\n");
        goto exit_read_id_failed;
    }

    if(chip_id == PI5U_30216A_ID)
    {
	writebuf[0] = 0x1;
	ret = i2c_smbus_write_i2c_block_data(info->i2c, REG_DEV_ID, 2, writebuf);
	if(ret != 0)
	{
		USB_LOG_ERROR("Write i2c block data error !");
		goto exit_read_id_failed;
	}
	msleep(30);
	writebuf[0] = 0x4;
	ret = i2c_smbus_write_i2c_block_data(info->i2c, REG_DEV_ID, 2, writebuf);
	if(ret != 0)
	{
		USB_LOG_ERROR("Write i2c block data error !");
		goto exit_read_id_failed;
	}
    }

    if(chip_id == PI5U_30216D_ID)
    {
        pi5u_30216D_setDRP(info->i2c);
    }

	info->pusb_class = class_create(THIS_MODULE, "pi5u");
	info->dev_t = device_create(info->pusb_class, NULL, 0, NULL, "pi5u");
	if (IS_ERR(info->dev_t)) {
		ret = PTR_ERR(info->dev_t);
		USB_LOG_ERROR("device_create  failed\n");
		goto create_dev_failed;
	}
	ret = device_create_file(info->dev_t, &dev_attr_type);
	if (ret < 0) {
		USB_LOG_ERROR("device_create_file  failed\n");
		goto create_dev_failed;
	}
	dev_set_drvdata(info->dev_t, info);

	return 0;

create_dev_failed:
	device_destroy(info->pusb_class, 0);
	class_destroy(info->pusb_class);
exit_read_id_failed:
exit_platform_failed:
	kfree(info);
exit:
	return ret;
}


static int pi5u_remove(struct i2c_client *client)
{
    struct pi5u_info *info = i2c_get_clientdata(client);

    device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->pusb_class, 0);
	class_destroy(info->pusb_class);	
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
    unregister_reboot_notifier(&pi5u_poweroff_notifier);
	
	kfree(info);
	
	return 0;
}

#if 0
static int  pi5u_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  pi5u_resume(struct i2c_client *client)
{
	return 0;
}
#endif
static const struct i2c_device_id pi5u_i2c_id[] = {
	{ "pi5u", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pi5u_i2c_id);


#ifdef CONFIG_OF
static struct of_device_id pi5u_usbtypec_match_table[] = {
        { .compatible = "mediatek,TYPEC_PI5U",},
        {},
};
#else
#define  pi5u_usbtypec_match_table NULL
#endif


static struct i2c_driver pi5u_i2c_driver = {
	.driver = {
		.name = "pi5u",
		.owner = THIS_MODULE,
		.of_match_table = pi5u_usbtypec_match_table,
	},
	.probe    = pi5u_probe,
	.remove   = pi5u_remove,
//	.suspend  = pi5u_suspend,
//	.resume	  = pi5u_resume,
	.id_table = pi5u_i2c_id,
};

static __init int pi5u_i2c_init(void)
{

	int ret;

#ifdef MTK_ANDROID_M_GPIO_DTS_CONFIG
	platform_driver_register(&pi5u_pinctl_platform_driver);
#endif
	ret =  i2c_add_driver(&pi5u_i2c_driver);
	PR_DEBUG_NUBIA_TYPEC("register tpyec i2c reg dirver ret = %d\n", ret);
	return ret;

}

static __exit void pi5u_i2c_exit(void)
{
	i2c_del_driver(&pi5u_i2c_driver);
}

module_init(pi5u_i2c_init);
module_exit(pi5u_i2c_exit);

MODULE_AUTHOR("shuchao gao<gao.shuchao123@zte.com.cn>");
MODULE_DESCRIPTION("I2C bus driver for pi5u USB Type-C");
MODULE_LICENSE("GPL v2");
