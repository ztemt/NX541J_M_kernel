/*
 * Atmel maXTouch Touchscreen driver Plug in
 *
 * Copyright (C) 2013 Atmel Co.Ltd
 *
 * Author: Pitter Liao <pitter.liao@atmel.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/****************************************************************  
	Pitter Liao add for macro for the global platform
		email:  pitter.liao@atmel.com 
		mobile: 13244776877
-----------------------------------------------------------------*/
#define PLUG_TICK_VERSION 0x0001
/*----------------------------------------------------------------

*/

//first version for TICK Plug: just watch T72 status

#include "plug.h"

#define TICK_FLAG_RESETING				P_FLAG_EV_RESETING
#define TICK_FLAG_CALING					P_FLAG_EV_CALING
#define TICK_FLAG_WORKAROUND_HALT		P_FLAG_EV_HALT

#define TICK_FLAG_RESET					P_FLAG_EV_RESET
#define TICK_FLAG_CAL					P_FLAG_EV_CAL
#define TICK_FLAG_RESUME					P_FLAG_EV_RESUME

#define TICK_FLAG_MASK_LOW				P_FLAG_EV_DONE_MASK
#define TICK_FLAG_MASK_NORMAL		(0x00f00)
#define TICK_NOISE_MASK				(0xf0000)
#define TICK_FLAG_MASK				(-1)

struct tick_observer{
	unsigned long flag;
};

struct tick_config{
	unsigned long rsv;
};

static void plugin_tick_hook_t6(struct plugin_tick *p, u8 status)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct tick_observer *obs = p->obs;

	plugin_p_hook_t6(status, &obs->flag,TICK_FLAG_MASK_NORMAL);

	dev_info2(dev, "mxt tick flag=0x%lx %x\n",
		 obs->flag, status);
}

static int plugin_tick_hook_t100(struct plugin_tick *p, int id, int x, int y, struct ext_info *in)
{
	return 0;
}

static void plugin_tick_start(struct plugin_tick *p, bool resume)
{
	struct tick_observer *obs = p->obs;

	clear_flag(TICK_FLAG_WORKAROUND_HALT, &obs->flag);

	if (resume)
		set_flag(TICK_FLAG_RESUME, &obs->flag);
}

static void plugin_tick_stop(struct plugin_tick *p)
{
	struct tick_observer *obs = p->obs;

	set_and_clr_flag(TICK_FLAG_WORKAROUND_HALT,TICK_FLAG_RESUME, &obs->flag);
}

static long plugin_tick_post_process_messages(struct plugin_tick *p, unsigned long pl_flag)
{
	struct tick_observer *obs = p->obs;
	long interval = MAX_SCHEDULE_TIMEOUT;
	
	if (test_flag(TICK_FLAG_WORKAROUND_HALT,&obs->flag))
		return interval;

	if (test_flag(TICK_FLAG_RESETING|TICK_FLAG_CALING,&obs->flag))
		return interval;

	clear_flag(TICK_FLAG_MASK_LOW,&obs->flag);

	return interval;
}

static int plugin_tick_show(struct plugin_tick *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct tick_observer * obs = p->obs;

	dev_info(dev, "[mxt]PLUG_TICK_VERSION: 0x%x\n",PLUG_TICK_VERSION);
	
	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]TICK cfg :\n");
	dev_info(dev, "[mxt]\n");

	dev_info(dev, "[mxt]TICK obs :\n");
	dev_info(dev, "[mxt]status: Flag=0x%08lx\n",
		obs->flag);
	dev_info(dev, "[mxt]\n");

	return 0;
}


static int plugin_tick_store(struct plugin_tick *p, const char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;	
	struct tick_observer * obs = p->obs;

	dev_info(dev, "[mxt]tick store:%s\n",buf);

	if (!p->init)
		return 0;

	if (sscanf(buf, "status: Flag=0x%lx\n",
		&obs->flag) > 0) {
		dev_info(dev, "[mxt] OK\n");
	}else{
		dev_info(dev, "[mxt] BAD\n");
	}
	
	return 0;
}

static int init_tick_object(struct plugin_tick *p)
{ 
	return 0;
}

static int deinit_tick_object(struct plugin_tick *p)
{
	return 0;
}


static int plugin_tick_init(struct plugin_tick *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;

	dev_info(dev, "%s: plugin tick version 0x%x\n", 
			__func__,PLUG_TICK_VERSION);

	p->obs = kzalloc(sizeof(struct tick_observer), GFP_KERNEL);
	if (!p->obs) {
		dev_err(dev, "Failed to allocate memory for tick observer\n");
		return -ENOMEM;
	}

	p->cfg = kzalloc(sizeof(struct tick_config), GFP_KERNEL);
	if (!p->cfg) {
		dev_err(dev, "Failed to allocate memory for tick cfg\n");
		kfree(p->obs);
		p->obs =NULL;
		return -ENOMEM;
	}

	if (init_tick_object(p) != 0) {
		dev_err(dev, "Failed to allocate memory for tick cfg\n");
		kfree(p->obs);
		p->obs = NULL;
		kfree(p->cfg);
		p->cfg = NULL;
	}
	
	return  0;
}

static void plugin_tick_deinit(struct plugin_tick *p)
{
	if (p->obs) {
		deinit_tick_object(p);
		kfree(p->obs);
	}
	if (p->cfg)
		kfree(p->cfg);
}

static struct plugin_tick mxt_plugin_tick_if = 
{
	.init = plugin_tick_init,
	.deinit = plugin_tick_deinit,
	.start = plugin_tick_start,
	.stop = plugin_tick_stop,
	.hook_t6 = plugin_tick_hook_t6,
	.hook_t100 = plugin_tick_hook_t100,
	.post_process = plugin_tick_post_process_messages,
	.show = plugin_tick_show,
	.store = plugin_tick_store,
};

int plugin_interface_tick_init(struct plugin_tick *p)
{
	memcpy(p, &mxt_plugin_tick_if, sizeof(struct plugin_tick));

	return 0;
}

