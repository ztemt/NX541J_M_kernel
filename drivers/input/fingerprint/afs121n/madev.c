/* MicroArray Fprint Driver Code for REE enviroment
 * madev.c
 * Date: 2016-09-10
 * Version: v4.1.2
 * Author: czl&cpy&guq
 * Contact: [czl|cpy|guq]@microarray.com.cn
 */
#include "madev.h"


//spdev use for recording the data for other use
static unsigned int irq, ret;
static DECLARE_WAIT_QUEUE_HEAD(gWaitq);
struct wake_lock gIntWakeLock;
struct wake_lock gProcessWakeLock;
struct work_struct gWork;
struct workqueue_struct *gWorkq;

//
static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_lock);
static DEFINE_MUTEX(drv_lock);
static DECLARE_WAIT_QUEUE_HEAD(drv_waitq);
//

static struct fprint_dev *sdev = NULL;
static struct fprint_spi *smas = NULL;

static u8 *stxb = NULL;
static u8 *srxb = NULL;


static void mas_work(struct work_struct *pws)
{
    smas->f_irq = 1;
    wake_up_interruptible(&gWaitq);
}

static irqreturn_t mas_interrupt(int irq, void *dev_id)
{
    wake_lock_timeout(&gIntWakeLock, 5 * HZ);
    queue_work(gWorkq, &gWork);
    return IRQ_HANDLED;
}


/*---------------------------------- fops ------------------------------------*/

/* 读写数据
 * @buf 数据
 * @len 长度
 * @返回值：0成功，否则失败
 */
int mas_sync(u8 *txb, u8 *rxb, int len)
{
    int ret;
    mas_select_transfer(smas->spi, len);
    smas->xfer.tx_nbits = SPI_NBITS_SINGLE;
    smas->xfer.tx_buf = txb;
    smas->xfer.rx_nbits = SPI_NBITS_SINGLE;
    smas->xfer.rx_buf = rxb;
    smas->xfer.delay_usecs = 1;
    smas->xfer.len = len;
    smas->xfer.bits_per_word = 8;
    smas->xfer.speed_hz = smas->spi->max_speed_hz;
    spi_message_init(&smas->msg);
    spi_message_add_tail(&smas->xfer, &smas->msg);
    ret = spi_sync(smas->spi, &smas->msg);
    return ret;
}



/* 读数据
 * @return 成功:count, -1count太大，-2通讯失败, -3拷贝失败
 */
static ssize_t mas_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int val, ret = 0;
    //MALOGD("start");
    if(count > FBUF) {
        MALOGW("data too long");
        return -1;
    }
    ret = mas_sync(stxb, srxb, count);
    if(ret) {
        MALOGW("mas_sync failed.");
        return -2;
    }
    ret = copy_to_user(buf, srxb, count);
    if(!ret) val = count;
    else {
        val = -3;
        MALOGW("copy_to_user failed.");
    }
    //MALOGD("end.");
    return val;
}



//this function mas_poll using on old struct
#if 0
static unsigned int mas_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int mask = 0;

    printd("%s: start. f_irq=%d f_repo=%d f_wake=%d\n",
           __func__, smas->f_irq, smas->f_repo, smas->f_wake);

    poll_wait(filp, &drv_waitq, wait);
    if(smas->f_irq && smas->f_repo) {
        smas->f_repo = FALSE;
        mask |= POLLIN | POLLRDNORM;
    } else if( smas->f_wake ) {
        smas->f_wake = FALSE;
        mask |= POLLPRI;
    }

    printd("%s: end. mask=%d\n", __func__, mask);

    return mask;
}
#endif


static void mas_set_input(void)
{
    struct input_dev *input = NULL;
    input = input_allocate_device();
    if (!input) {
        MALOGW("input_allocate_device failed.");
        return ;
    }
    set_bit(EV_KEY, input->evbit);
    set_bit(EV_ABS, input->evbit);
    set_bit(EV_SYN, input->evbit);
    set_bit(FINGERPRINT_SWIPE_UP, input->keybit); //单触
    set_bit(FINGERPRINT_SWIPE_DOWN, input->keybit);
    set_bit(FINGERPRINT_SWIPE_LEFT, input->keybit);
    set_bit(FINGERPRINT_SWIPE_RIGHT, input->keybit);
    set_bit(FINGERPRINT_TAP, input->keybit);
    set_bit(FINGERPRINT_DTAP, input->keybit);
    set_bit(FINGERPRINT_LONGPRESS, input->keybit);

    set_bit(KEY_POWER, input->keybit);

    input->name = MA_CHR_DEV_NAME;
    input->id.bustype = BUS_SPI;
    ret = input_register_device(input);
    if (ret) {
        input_free_device(input);
        MALOGW("failed to register input device.");
        return;
    }
    smas->input  = input;
}



//static int mas_ioctl (struct inode *node, struct file *filp, unsigned int cmd, uns igned long arg)
//this function only supported while the linux kernel version under v2.6.36,while the kernel version under v2.6.36, use this line
static long mas_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    //MALOGF("start");
    switch(cmd) {
    case MA_IOC_DELK:                                                       //延时锁    timeout lock
        wake_lock_timeout(&gProcessWakeLock, 5 * HZ);
        break;
    case MA_IOC_SLEP:                                                       //remove the process out of the runqueue
        smas->f_irq = 0;
        wait_event_interruptible(gWaitq, smas->f_irq != 0);
        break;
    case MA_IOC_WKUP:                                                       //wake up, schedule the process into the runqueue
        smas->f_irq = 1;
        wake_up_interruptible(&gWaitq);
        break;
    case MA_IOC_ENCK:
        mas_enable_spi_clock(smas->spi);                                    //if the spi clock is not opening always, do this methods
        break;
    case MA_IOC_DICK:
        mas_disable_spi_clock(smas->spi);                                   //disable the spi clock
        break;
    case MA_IOC_EINT:
        enable_irq(irq);                                                    //enable the irq,in fact, you can make irq enable always
        break;
    case MA_IOC_DINT:
        disable_irq(irq);                                                    //disable the irq
        break;
    case MA_IOC_TPDW:
        input_report_key(smas->input, FINGERPRINT_TAP, 1);
        input_sync(smas->input);                                                 //tap down
        break;
    case MA_IOC_TPUP:
        input_report_key(smas->input, FINGERPRINT_TAP, 0);
        input_sync(smas->input);                                                     //tap up
        break;
    case MA_IOC_SGTP:
        input_report_key(smas->input, FINGERPRINT_TAP, 1);
        input_sync(smas->input);
        input_report_key(smas->input, FINGERPRINT_TAP, 0);
        input_sync(smas->input);                                                       //single tap
        break;
    case MA_IOC_DBTP:
        input_report_key(smas->input, FINGERPRINT_DTAP, 1);
        input_sync(smas->input);
        input_report_key(smas->input, FINGERPRINT_DTAP, 0);
        input_sync(smas->input);                                              //double tap
        break;
    case MA_IOC_LGTP:
        input_report_key(smas->input, FINGERPRINT_LONGPRESS, 1);
        input_sync(smas->input);
        input_report_key(smas->input, FINGERPRINT_LONGPRESS, 0);
        input_sync(smas->input);                                               //long tap
        break;
    default:
        ret = -EINVAL;
        MALOGW("mas_ioctl no such cmd");
    }
    //MALOGF("end");
    return ret;
}

#ifdef CONFIG_COMPAT
static long mas_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int retval = 0;
    retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);
    return retval;
}
#endif


/* 写数据
 * @return 成功:count, -1count太大，-2拷贝失败
 */
static ssize_t mas_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    int val;
    //MALOGD("start");
    if(count == 6) {                                                                            //cmd ioctl, old version used the write interface to do ioctl, this is only for the old version
        int cmd, arg;
        u8 tmp[6];
        ret = copy_from_user(tmp, buf, count);
        cmd = tmp[0];
        cmd <<= 8;
        cmd += tmp[1];
        arg = tmp[2];
        arg <<= 8;
        arg += tmp[3];
        arg <<= 8;
        arg += tmp[4];
        arg <<= 8;
        arg += tmp[5];
        val = (int)mas_ioctl(NULL, (unsigned int)cmd, (unsigned long)arg);
    } else {
        if(count > FBUF) {
            MALOGW("write data to long");
            return -1;
        }
        memset(stxb, 0, FBUF);
        ret = copy_from_user(stxb, buf, count);
        if(ret) {
            MALOGW("copy form user failed");
            val = -2;
        } else {
            val = count;
        }
    }
    //MALOGD("end");
    return val;
}



/*---------------------------------- fops ------------------------------------*/
static const struct file_operations sfops = {
    .owner = THIS_MODULE,
    .write = mas_write,
    .read = mas_read,
    .unlocked_ioctl = mas_ioctl,
    //.ioctl = mas_ioctl,
    //using the previous line replacing the unlock_ioctl while the linux kernel under version2.6.36
#ifdef CONFIG_COMPAT
    .compat_ioctl = mas_compat_ioctl,
#endif

};
/*---------------------------------- fops end ---------------------------------*/

static int init_file_node(void)
{
    int ret;
    //MALOGF("start");
    ret = alloc_chrdev_region(&sdev->idd, 0, 1, MA_CHR_DEV_NAME);
    if(ret < 0) {
        MALOGW("alloc_chrdev_region error!");
        return -1;
    }
    sdev->chd = cdev_alloc();
    if (!sdev->chd) {
        MALOGW("cdev_alloc error!");
        return -1;
    }
    sdev->chd->owner = THIS_MODULE;
    sdev->chd->ops = &sfops;
    cdev_add(sdev->chd, sdev->idd, 1);
    sdev->cls = class_create(THIS_MODULE, MA_CHR_DEV_NAME);
    if (IS_ERR(sdev->cls)) {
        MALOGE("class_create");
        return -1;
    }
    sdev->dev = device_create(sdev->cls, NULL, sdev->idd, NULL, MA_CHR_FILE_NAME);
    ret = IS_ERR(sdev->dev) ? PTR_ERR(sdev->dev) : 0;
    if(ret) {
        MALOGE("device_create");
    }
    //MALOGF("end");
    return 0;
}

static int deinit_file_node(void)
{
    unregister_chrdev_region(sdev->idd, 1);
    cdev_del(sdev->chd);
    return 0;
}

static int init_interrupt(void)
{
    const char *tname = MA_EINT_NAME;
    irq = mas_get_irq();
    if(irq <= 0) {
        ret = irq;
        MALOGE("mas_get_irq");
    }
    ret = request_irq(irq, mas_interrupt, IRQF_TRIGGER_RISING, tname, NULL);
    if(ret <= 0) {
        MALOGE("request_irq");
    }
    enable_irq(irq);
    return ret;
}
static int deinit_interrupt(void)
{
    disable_irq(irq);
    free_irq(irq, NULL);
    return 0;
}


static int init_vars(void)
{
    sdev = kmalloc(sizeof(struct fprint_dev), GFP_KERNEL);
    smas = kmalloc(sizeof(struct fprint_spi), GFP_KERNEL);
    srxb = kmalloc(FBUF * sizeof(u8), GFP_KERNEL);
    stxb = kmalloc(FBUF * sizeof(u8), GFP_KERNEL);
    if (sdev == NULL || smas == NULL || srxb == NULL || stxb == NULL) {
        MALOGW("smas kmalloc failed.");
        if(sdev != NULL) kfree(sdev);
        if(smas != NULL) kfree(smas);
        if(stxb != NULL) kfree(stxb);
        if(srxb != NULL) kfree(srxb);
        return -ENOMEM;
    }
    wake_lock_init(&gIntWakeLock, WAKE_LOCK_SUSPEND, "microarray_int_wakelock");
    wake_lock_init(&gProcessWakeLock, WAKE_LOCK_SUSPEND, "microarray_process_wakelock");
    INIT_WORK(&gWork, mas_work);
    gWorkq = create_singlethread_workqueue("mas_workqueue");
    if (!gWorkq) {
        MALOGW("create_single_workqueue error!");
        return -ENOMEM;
    }
    return 0;
}
static int deinit_vars(void)
{
    destroy_workqueue(gWorkq);
    wake_lock_destroy(&gIntWakeLock);
    wake_lock_destroy(&gProcessWakeLock);
    return 0;
}

static int init_spi(struct spi_device *spi)
{
    mas_enable_spi_clock(spi);                 //if needed
    smas->spi = spi;
    smas->spi->max_speed_hz = SPI_SPEED;
    spi_setup(spi);
    INIT_LIST_HEAD(&smas->dev_entry);
    return 0;
}

static int deinit_spi(struct spi_device *spi)
{
    smas->spi = NULL;
    mas_disable_spi_clock(spi);
    return 0;
}
//this function used in chip check
#if 0
static int init_connect(void);
static int deinit_connect(void);
/* 连接设备
 * @return 1成功，0失败
 */
static int init_connect(void)
{
    int i, len = 4, ret = 0;
    const unsigned char cdat[] = { 0x41, 0x78, 0x4d, 0x41 };

    //printd("%s: start. cmd=0x%.2x par=0x%.2x\n", __func__, cmd, par);

    stxb[0] = 0x8c;
    stxb[1] = 0xff;
    stxb[2] = 0xff;
    stxb[3] = 0xff;
    mas_sync(stxb, srxb, len); //reset
    msleep(5);
    for(i = 2; i < 4; i++) {
        stxb[0] = i * 4;
        stxb[1] = cdat[i];
        ret = mas_sync(stxb, srxb, len);
        if(ret != 0) return 0;
        if(srxb[2] != cdat[i]) return 0;
    }

    return 1;
}

static int deinit_connect(void)
{

    return 0;
}
#endif

int mas_plat_probe(struct platform_device *pdev)
{
    //MALOGD("start");
    ret = mas_finger_get_gpio_info(pdev);
    if(ret) {
        MALOGE("mas_plat_probe do mas_finger_get_gpio_info");
    }
    ret = mas_finger_set_gpio_info(1);
    if(ret) {
        MALOGE("mas_plat_probe do mas_finger_set_gpio_info");
    }
    //MALOGD("end");
    return ret;
}

int mas_plat_remove(struct platform_device *pdev)
{
    mas_finger_set_gpio_info(0);
    return 0;
}


int mas_probe(struct spi_device *spi)
{

    //MALOGD("start");
    ret = init_vars();
    if(ret) {
        goto err1;
    }
    ret = init_interrupt();
    if(ret) {
        goto err2;
    }
    ret = init_file_node();
    if(ret) {
        goto err3;
    }
    ret = init_spi(spi);
    if(ret) {
        goto err4;
    }
    mas_set_input();
    MALOGF("end");

    return ret;

#if 0
    //if needed
    ret = init_connect();
    goto err0;
    //end
#endif
err4:
    deinit_spi(spi);
err3:
    deinit_file_node();
err2:
    deinit_interrupt();
err1:
    deinit_vars();


    return 0;
}

int mas_remove(struct spi_device *spi)
{
    deinit_file_node();
    deinit_interrupt();
    deinit_vars();
    return 0;
}


static int __init mas_init(void)
{
    int ret = 0;
    MALOGF("start");
    ret = mas_get_platform();
    if(ret) {
        MALOGE("mas_get_platform");
    }
    return ret;
}

static void __exit mas_exit(void)
{
}

module_init(mas_init);
module_exit(mas_exit);

MODULE_AUTHOR("Microarray");
MODULE_DESCRIPTION("Driver for microarray fingerprint sensor");
MODULE_LICENSE("GPL");
