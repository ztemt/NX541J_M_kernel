/* MicroArray Fprint Driver Code
 * madev.h
 * Date: 2016-09-10
 * Version: v4.1.2
 * Author: czl&cpy&guq
 * Contact: [czl|cpy|guq]@microarray.com.cn
 */

#ifndef __MADEV_H_
#define __MADEV_H_


//settings macro
#define MTK   			//[MTK|QUALCOMM|SPRD]

#define MALOGD_LEVEL	KERN_EMERG     //[KERN_DEBUG|KERN_EMERG] usually, the debug level is used for the release version

#define MA_CHR_FILE_NAME 	"madev0"  //do not neeed modify usually

#define MA_EINT_NAME            "mediatek,finger_print-eint"
#define MA_CHR_DEV_NAME 		"madev"
//key define   just modify the KEY_FN_* for different platform
#define FINGERPRINT_SWIPE_UP 			KEY_FN_F1//827
#define FINGERPRINT_SWIPE_DOWN 			KEY_FN_F2//828
#define FINGERPRINT_SWIPE_LEFT 			KEY_FN_F3//829
#define FINGERPRINT_SWIPE_RIGHT 		KEY_FN_F4//830
#define FINGERPRINT_TAP 				KEY_FN_F5//	831
#define FINGERPRINT_DTAP				KEY_FN_F6// 	832
#define FINGERPRINT_LONGPRESS 			KEY_F3//833

//key define end


//old macro
#define SPI_SPEED 	(6*1000000) 	//120/121:10M, 80/81:6M

//表面类型
#define	COVER_T		1
#define COVER_N		2
#define COVER_M		3
#define COVER_NUM	COVER_N

//指纹类型
#define AFS120	0x78
//#define AFS80 	0x50

#if defined(AFS120)
#define W   	120   //宽
#define H   	120   //高
#define WBUF	121
#define FBUF  	(1024*15)	//读写长度
#elif defined(AFS80)
#define W   	80    //宽
#define H   	192   //高
#define WBUF	81
#define FIMG	(W*H)
#define FBUF  	(1024*16)	//读写长度
#endif

//settings macro end
#include <linux/notifier.h>
#include <linux/fb.h>
#include <asm/ioctl.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/ioctl.h>
#include <linux/wakelock.h>

#ifdef MTK
#include "mtk-settings.h"
#elif defined QUALCOMM
#include "qualcomm-settings.h"
#elif defined SPRD
#include "sprd-settings.h"
#endif


//value define
//fprint_spi struct use to save the value
struct fprint_spi {
    u8 do_what;             //工作内容
    u8 f_wake;              //唤醒标志
    int value;
    volatile u8 f_irq;      //中断标志
    volatile u8 f_repo;     //上报开关
    spinlock_t spi_lock;
    struct spi_device *spi;
    struct list_head dev_entry;
    struct spi_message msg;
    struct spi_transfer xfer;
    struct input_dev *input;
    struct work_struct work;
    struct workqueue_struct *workq;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend suspend;
#endif
    struct wake_lock wl;
};
//end

struct fprint_dev {
    dev_t idd;
    int major;
    int minor;
    struct cdev *chd;
    struct class *cls;
    struct device *dev;
};


//function define

//extern the settings.h function
extern void mas_select_transfer(struct spi_device *spi, int len);
extern int mas_finger_get_gpio_info(struct platform_device *pdev);
extern int mas_finger_set_gpio_info(int cmd);
extern void mas_enable_spi_clock(struct spi_device *spi);
extern void mas_disable_spi_clock(struct spi_device *spi);
extern unsigned int mas_get_irq(void);
extern int mas_get_platform(void);

//end

//use for the log print
#define MALOG_TAG "MAFP_"
#define MALOGE(x) printk(KERN_ERR "%s%s: error log! the function %s is failed, ret = %d\n", MALOG_TAG, __func__, x, ret);  	//error log
#define MALOGF(x) printk(MALOGD_LEVEL "%s%s: debug log! %s!\n", MALOG_TAG, __func__, x);										//flag log
#define MALOGD(x) MALOGF(x)																									//debug log
#define MALOGW(x) printk(KERN_WARNING "%s%s: warning log! the function %s's ret = %d\n", MALOG_TAG, __func__,x, ret);			//warning log
//use for the log print





//user for the ioctl,new version
//#define MA_IOC_MAGIC    (('M'<<24)|('A'<<16)|('F'<<8)|'P')
#define MA_IOC_MAGIC 	'M'
//#define MA_IOC_INIT   _IOR(MA_IOC_MAGIC, 0, unsigned char)
#define MA_IOC_DELK     _IO(MA_IOC_MAGIC,  1)					//dealy lock
#define MA_IOC_SLEP     _IO(MA_IOC_MAGIC,  2)					//sleep, remove the process out of the runqueue
#define MA_IOC_WKUP     _IO(MA_IOC_MAGIC,  3)					//wake up, sechdule the process into the runqueeue
#define MA_IOC_ENCK     _IO(MA_IOC_MAGIC,  4)					//only use in tee while the spi clock is not enable
#define MA_IOC_DICK		_IO(MA_IOC_MAGIC,  5)					//disable spi clock
#define MA_IOC_EINT		_IO(MA_IOC_MAGIC,  6)					//enable irq
#define MA_IOC_DINT		_IO(MA_IOC_MAGIC,  7)					//disable irq
#define MA_IOC_TPDW		_IO(MA_IOC_MAGIC,  8)					//tap DOWN
#define MA_IOC_TPUP		_IO(MA_IOC_MAGIC,  9)					//tap UP
#define MA_IOC_SGTP		_IO(MA_IOC_MAGIC,  11)					//single tap
#define MA_IOC_DBTP		_IO(MA_IOC_MAGIC,  12)					//double tap
#define MA_IOC_LGTP		_IO(MA_IOC_MAGIC,  13)					//log tap
#define MA_IOC_VTIM		_IOR(MA_IOC_MAGIC,  14, unsigned char)					//version time
#define MA_IOC_CNUM		_IOR(MA_IOC_MAGIC,  15, unsigned char)					//cover num
#define MA_IOC_SNUM		_IOR(MA_IOC_MAGIC,  16, unsigned char)					//sensor type
#define MA_IOC_UKRP		_IOW(MA_IOC_MAGIC,  17, unsigned char)					//user define the report key
#define MA_IOC_EIRQ		_IO(MA_IOC_MAGIC,  31)					//request_irq
#define MA_IOC_DIRQ		_IO(MA_IOC_MAGIC,  32)					//free_irq
#define MA_IOC_SPAR		_IOW(MA_IOC_MAGIC,  33, unsigned char)
#define MA_IOC_GPAR 	_IOR(MA_IOC_MAGIC,  34, unsigned char)
//ioctl end
#define MA_IOC_GET_SCREEN	_IOR(MA_IOC_MAGIC, 44, unsigned char)
#define MA_IOC_REGISTER_FB _IO(MA_IOC_MAGIC, 43)




/**
 *	the old ioctl command, compatible for the old version
 */
//ioctl cmd
#if 0
#define IOCTL_DEBUG			0x100	//调试信息 			//debug message
#define IOCTL_IRQ_ENABLE	0x101	//中断使能 			//enable interrupt
#define IOCTL_SPI_SPEED   	0x102	//SPI速度 			//spi speed
#define IOCTL_READ_FLEN		0x103	//读帧长度(保留)		//the length of one frame
#define IOCTL_LINK_DEV		0x104	//连接设备(保留)		//connect the device
#define IOCTL_COVER_NUM		0x105	//材料编号			//the index of the material
#define IOCTL_GET_VDATE		0x106	//版本日期			//the date fo the version

#define IOCTL_CLR_INTF		0x110	//清除中断标志
#define IOCTL_GET_INTF		0x111	//获取中断标志
#define IOCTL_REPORT_FLAG	0x112 	//上报标志
#define IOCTL_REPORT_KEY	0x113	//上报键值
#define IOCTL_SET_WORK		0x114	//设置工作
#define IOCTL_GET_WORK		0x115	//获取工作
#define IOCTL_SET_VALUE		0x116	//设值
#define IOCTL_GET_VALUE		0x117	//取值
#define IOCTL_TRIGGER		0x118	//自触发
#define IOCTL_WAKE_LOCK		0x119	//唤醒上锁
#define IOCTL_WAKE_UNLOCK	0x120	//唤醒解锁

#define IOCTL_SCREEN_ON		0x121

#define IOCTL_KEY_DOWN		0x121	//按下
#define IOCTL_KEY_UP		0x122	//抬起
#define IOCTL_SET_X			0x123	//偏移X
#define IOCTL_SET_Y			0x124	//偏移Y
#define IOCTL_KEY_TAP		0x125	//单击
#define IOCTL_KEY_DTAP		0x126	//双击
#define IOCTL_KEY_LTAP		0x127	//长按

#define IOCTL_ENABLE_CLK    0x128
#define TRUE 	1
#define FALSE 	0
#endif

#endif /* __MADEV_H_ */

