/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * drivers/misc/mediatek/pd/tcpc_usb_cb.c
 * TCPC USB Call Back Driver
 *
 * Author: Sakya <jeff_chang@richtek.com>
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
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/usb/class-dual-role.h>
#include <mt-plat/pd/tcpm.h>
#include <typec.h>

#define TCPC_USB_INFO(format, args...)	\
	pr_info("TCPC-USB " format, ##args)

static int usb_register_num = 2;
static struct tcpc_device *tcpc_dev;
static struct notifier_block usb_nb;
static struct usbtypc *exttypec;
static bool otg_attached;

static int trigger_driver(struct usbtypc *typec, int type, int stat, int dir)
{
#ifdef CONFIG_MTK_SIB_USB_SWITCH
	if (typec->sib_enable) {
		TCPC_USB_INFO("SIB enable!\n");
		goto end;
	}
#endif /* CONFIG_MTK_SIB_USB_SWITCH */

	TCPC_USB_INFO("trigger_driver: type:%d, stat:%d, dir%d\n",
							type, stat, dir);

	if (type == DEVICE_TYPE && typec->device_driver) {
		if ((stat == DISABLE) && (typec->device_driver->disable)
				&& (typec->device_driver->on == ENABLE)) {
			typec->device_driver->disable(
					typec->device_driver->priv_data);
			typec->device_driver->on = DISABLE;

			TCPC_USB_INFO("trigger_driver: disable\n");
		} else if ((stat == ENABLE) && (typec->device_driver->enable)
				&& (typec->device_driver->on == DISABLE)) {
			typec->device_driver->enable(
					typec->device_driver->priv_data);
			typec->device_driver->on = ENABLE;

			TCPC_USB_INFO("trigger_driver: enable dev drv\n");
		} else
			TCPC_USB_INFO("No device driver to enable\n");
	} else if (type == HOST_TYPE && typec->host_driver) {
		if ((stat == DISABLE) && (typec->host_driver->disable)
				&& (typec->host_driver->on == ENABLE)) {
			typec->host_driver->disable(
					typec->host_driver->priv_data);
			typec->host_driver->on = DISABLE;

			TCPC_USB_INFO("trigger_driver: disable host drv\n");
		} else if ((stat == ENABLE) &&
				(typec->host_driver->enable) &&
				(typec->host_driver->on == DISABLE)) {
			typec->host_driver->enable(
					typec->host_driver->priv_data);
			typec->host_driver->on = ENABLE;

			TCPC_USB_INFO("trigger_driver: enable host drv\n");
		} else

			TCPC_USB_INFO("No device driver to enable\n");
	} else
		TCPC_USB_INFO("trigger_driver: no callback func\n");
#ifdef CONFIG_MTK_SIB_USB_SWITCH
end:
#endif /* CONFIG_MTK_SIB_USB_SWITCH */
	return 0;
}

static int usb_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;

	switch (event) {
	case TCP_NOTIFY_TYPEC_STATE:
		TCPC_USB_INFO("%s polarity = %d, rp_level = %d\n", __func__,
			noti->typec_state.polarity, noti->typec_state.rp_level);
		switch (noti->typec_state.new_state) {
		case TYPEC_UNATTACHED: /* USB Plug out */
			if (noti->typec_state.old_state == TYPEC_ATTACHED_SNK)
				trigger_driver(exttypec,
						DEVICE_TYPE, DISABLE, UP_SIDE);
			break;
		case TYPEC_ATTACHED_SNK: /* USB Plug in */
			trigger_driver(exttypec, DEVICE_TYPE, ENABLE, UP_SIDE);
			break;
		default:
			break;
		}
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		if (noti->vbus_state.mv) { /* OTG Plug in */
			trigger_driver(exttypec, HOST_TYPE, ENABLE, UP_SIDE);
			otg_attached = true;
		} else {
			if (otg_attached) { /* OTG Plug out */
				trigger_driver(exttypec,
						HOST_TYPE, DISABLE, UP_SIDE);
				otg_attached = false;
			}
		}
		break;
	case TCP_NOTIFY_SINK_VBUS:
		if (noti->vbus_state.mv) {
			TCPC_USB_INFO("%s sink_vbus %dmv, %dma\n", __func__,
				noti->vbus_state.mv, noti->vbus_state.ma);
		}
		break;
	}
	return NOTIFY_OK;
}

int register_typec_switch_callback(struct typec_switch_data *new_driver)
{
	struct dual_role_phy_instance *usb_dr;
	unsigned int val;
	int ret;

	TCPC_USB_INFO("Register driver %s %d\n",
			new_driver->name, new_driver->type);
	usb_register_num--;

	if (!usb_register_num) {
		tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
		if (!tcpc_dev) {
			pr_err("%s get tcpc device type_c_port0 fail\n",
								__func__);
			return -ENODEV;
		}

		usb_nb.notifier_call = usb_tcp_notifier_call;
		ret = register_tcp_dev_notifier(tcpc_dev, &usb_nb);
		if (ret < 0) {
			pr_err("%s: register usb tcpc notifer fail\n",
								__func__);
			return -EINVAL;
		}
	}

	if (new_driver->type == DEVICE_TYPE) {
		exttypec->device_driver = new_driver;
		exttypec->device_driver->on = 0;
		return 0;
	}

	if (new_driver->type == HOST_TYPE) {
		exttypec->host_driver = new_driver;
		exttypec->host_driver->on = 0;
		usb_dr = dual_role_phy_instance_get_byname(
					"dual-role-type_c_port0");
		if (!usb_dr) {
			pr_err("%s RT1711 Dual Role Class not ready\n",
								__func__);
			return -ENODEV;
		}
		ret = dual_role_get_property(usb_dr, DUAL_ROLE_PROP_DR, &val);
		if (ret < 0) {
			pr_err("%s dual-role-rt1711 get prop fail\n", __func__);
			return -EINVAL;
		}
		if (val == DUAL_ROLE_PROP_DR_HOST)
			trigger_driver(exttypec,
				HOST_TYPE, ENABLE, DONT_CARE);
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(register_typec_switch_callback);

int unregister_typec_switch_callback(struct typec_switch_data *new_driver)
{
	TCPC_USB_INFO("UnRegister driver %s %d\n",
			new_driver->name, new_driver->type);

	if ((new_driver->type == DEVICE_TYPE) &&
				(exttypec->device_driver == new_driver))
		exttypec->device_driver = NULL;

	if ((new_driver->type == HOST_TYPE) &&
			(exttypec->host_driver == new_driver))
		exttypec->host_driver = NULL;

	return 0;
}

static int tcpc_usb_cb_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	exttypec = kzalloc(sizeof(struct usbtypc), GFP_KERNEL);

	pr_info("%s OK!!\n", __func__);
	return ret;
}

static int tcpc_usb_cb_remove(struct platform_device *pdev)
{
	unregister_tcp_dev_notifier(tcpc_dev, &usb_nb);
	return 0;
}

static const struct of_device_id tcpc_usb_cb_of_match[] = {
	{ .compatible = "mediatek,tcpc_usb_cb" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcpc_usb_cb_of_match);

static struct platform_driver tcpc_usb_cb_driver = {
	.driver = {
		.name = "tcpc-usb-cb",
		.of_match_table = of_match_ptr(tcpc_usb_cb_of_match),
	},
	.probe = tcpc_usb_cb_probe,
	.remove = tcpc_usb_cb_remove,
};

static int __init tcpc_usb_cb_init(void)
{
	return platform_driver_register(&tcpc_usb_cb_driver);
}

static void __exit tcpc_usb_cb_exit(void)
{
	platform_driver_unregister(&tcpc_usb_cb_driver);
}

subsys_initcall(tcpc_usb_cb_init);
module_exit(tcpc_usb_cb_exit);

MODULE_AUTHOR("Jeff Chang");
MODULE_DESCRIPTION("Richtek TCPC USB Call Back Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0_MTK");
