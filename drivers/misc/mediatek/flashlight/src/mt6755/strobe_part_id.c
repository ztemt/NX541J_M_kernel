/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
#ifdef CONFIG_ZTEMT_SINGLE_FLASHLIGHT
int strobe_getPartId(int sensorDev, int strobeId)
{
	/* return 1 or 2 (backup flash part). Other numbers are invalid. */
	if (sensorDev == e_CAMERA_MAIN_SENSOR && strobeId == 1) {
		return 1;
	} else if (sensorDev == e_CAMERA_MAIN_SENSOR && strobeId == 2) {
		return 1;
	} else if (sensorDev == e_CAMERA_SUB_SENSOR && strobeId == 1) {
		return 1;
	} else if (sensorDev == e_CAMERA_SUB_SENSOR && strobeId == 2) {
		return 1;
	} else {		/* e_CAMERA_MAIN_2_SENSOR */

		return 200;
	}
	return 100;
}

#else
//ZTEMT:wangkai add for dual flash comp---Start
extern int dual_flash_id;
//ZTEMT:wangkai add for dual flash comp---End
int strobe_getPartId(int sensorDev, int strobeId)
{
	/* return 1 or 2 (backup flash part). Other numbers are invalid. */
	
    //ZTEMT:wangkai add for dual flash comp---Start
    //printk(" kdebug strobe_getPartId sensorDev=%d, strobeId=%d,dual_flash_id = 0x%x\n", sensorDev, strobeId,dual_flash_id);
	if (sensorDev == e_CAMERA_MAIN_SENSOR && strobeId == 1) {
		if (dual_flash_id == 0x8)
		return 1;
		else if(dual_flash_id == 0x2)
		return 2;
	} else if (sensorDev == e_CAMERA_MAIN_SENSOR && strobeId == 2) {
		if (dual_flash_id == 0x8)
		return 1;
		else if(dual_flash_id == 0x2)
		return 2;
	//ZTEMT:wangkai add for dual flash comp---End
	} else if (sensorDev == e_CAMERA_SUB_SENSOR && strobeId == 1) {
		return 1;
	} else if (sensorDev == e_CAMERA_SUB_SENSOR && strobeId == 2) {
		return 1;
	} else {		/* e_CAMERA_MAIN_2_SENSOR */

		return 200;
	}

	return 100;
}
#endif
