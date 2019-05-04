/*
 * f_autobot.h -- USB Projector (Autobot) function driver
 *
 * Copyright (C) 2017 HTC Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _F_AUTOBOT_H
#define _F_AUTOBOT_H

#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/random.h>
#include <linux/module.h>
#include <linux/htc_mode_server.h>
#include <linux/usb/composite.h>
#include <linux/usb/htc_info.h>

#if MINIFB_READY
#include <linux/minifb.h>
#endif

#ifdef VDBG
#undef VDBG
#endif

#if 1
#define VDBG(x...) do {} while (0)
#else
#define VDBG(x...) printk(KERN_INFO x)
#endif

/*16KB*/
#define TXN_MAX 16384
#define RXN_MAX 4096

/* number of rx requests to allocate */
#define PROJ_RX_REQ_MAX 4

#ifdef DUMMY_DISPLAY_MODE_320_480
#define DEFAULT_PROJ_WIDTH			320
#define DEFAULT_PROJ_HEIGHT			480
#define TOUCH_WIDTH					320
#define TOUCH_HEIGHT				480
#else
#define DEFAULT_PROJ_WIDTH			480
#define DEFAULT_PROJ_HEIGHT			800
#define TOUCH_WIDTH					480
#define TOUCH_HEIGHT				800
#endif

#define BITSPIXEL 16
#define FRAME_INTERVAL_TIME 200

/* for configfs support */
#define MAX_INST_NAME_LEN	40

struct f_autobot_opts {
	struct usb_function_instance func_inst;
	struct htcmode_protocol config;
	const char *name;
};

struct autobot_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;

	int online;
	int error;

	struct list_head tx_idle;
	struct list_head rx_idle;

	int rx_done;

	u32 bitsPixel;
	u32 framesize;
	u32 width;
	u32 height;
	u8	init_done;
	u8 enabled;
	u16 frame_count;
	u32 rx_req_count;
	u32 tx_req_count;
	struct input_dev *keypad_input;
	struct input_dev *touch_input;
	char *fbaddr;

	atomic_t cand_online;
	struct switch_dev cand_sdev;
	struct switch_dev htcmode_sdev;
	struct work_struct notifier_work;
	struct work_struct htcmode_notifier_work;

	struct workqueue_struct *wq_display;
	struct work_struct send_fb_work;
	struct work_struct send_fb_work_legacy;
	int start_send_fb;

	/* HTC Mode Protocol Info */
	struct htcmode_protocol *htcmode_proto;
	u8 is_htcmode;
	struct hsml_header header;
	u8 notify_authenticator;
};

static struct usb_interface_descriptor autobot_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor autobot_superspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor autobot_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor autobot_superspeed_in_comp_desc = {
	.bLength =		sizeof(autobot_superspeed_in_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	.bMaxBurst =		2,
	/* .bmAttributes =	0, */
};

static struct usb_ss_ep_comp_descriptor autobot_superspeed_out_comp_desc = {
	.bLength =		sizeof(autobot_superspeed_out_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	 .bMaxBurst =		2,
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor autobot_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor autobot_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor autobot_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor autobot_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_autobot_descs[] = {
	(struct usb_descriptor_header *) &autobot_interface_desc,
	(struct usb_descriptor_header *) &autobot_fullspeed_in_desc,
	(struct usb_descriptor_header *) &autobot_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_autobot_descs[] = {
	(struct usb_descriptor_header *) &autobot_interface_desc,
	(struct usb_descriptor_header *) &autobot_highspeed_in_desc,
	(struct usb_descriptor_header *) &autobot_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_autobot_descs[] = {
	(struct usb_descriptor_header *) &autobot_interface_desc,
	(struct usb_descriptor_header *) &autobot_superspeed_in_desc,
	(struct usb_descriptor_header *) &autobot_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &autobot_superspeed_out_desc,
	(struct usb_descriptor_header *) &autobot_superspeed_out_comp_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string autobot_string_defs[] = {
	[0].s = "HTC AUTOBOT",
	{  } /* end of list */
};

static struct usb_gadget_strings autobot_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		autobot_string_defs,
};

static struct usb_gadget_strings *autobot_strings[] = {
	&autobot_string_table,
	NULL,
};

struct size {
	int w;
	int h;
};

enum {
    NOT_ON_AUTOBOT,
    DOCK_ON_AUTOBOT,
    HTC_MODE_RUNNING
};

void htc_mode_enable(int enable);
int check_htc_mode_status(void);
int autobot_ctrlrequest(struct usb_composite_dev *cdev,
		const struct usb_ctrlrequest *ctrl);

static inline struct f_autobot_opts *ci_to_f_autobot_opts(
		struct config_item *item)
{
	return container_of(to_config_group(item), struct f_autobot_opts,
			func_inst.group);
}

static inline struct f_autobot_opts *fi_to_f_autobot_opts(
		struct usb_function_instance *fi)
{
	return container_of(fi, struct f_autobot_opts, func_inst);
}

static inline struct autobot_dev *func_to_autobot_dev(
		struct usb_function *f)
{
	return container_of(f, struct autobot_dev, function);
}

#endif //#ifndef _F_AUTOBOT_H
