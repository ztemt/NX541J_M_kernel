/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

#include "ts_key.h"

struct obj_container{
	u8 id;		//container pair: ID value for search
	int value;	//container value
	struct list_head node;
};

struct obj_link{
	u16 object;	//object
	struct list_head sublist;	//container head
	struct list_head node;	//next link
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	atomic_t depth;		/* irq nested counter*/
	int irq;			/* irq issued by device 	*/

	unsigned gpio_reset;
	unsigned gpio_irq;

	bool use_retrigen_workaround;
	bool use_regulator;
	bool common_vdd_supply;

	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	
	const char *cfg_name;

	struct list_head keylist;

	bool mem_allocated;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
