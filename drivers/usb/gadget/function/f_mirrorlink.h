/*
 * f_mirrorlink.h -- USB Projector (Mirrorlink) function driver
 *
 * Copyright (C) 2010 HTC Corporation
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

#ifndef _F_MIRRORLINK_H
#define _F_MIRRORLINK_H

#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/random.h>
#include <linux/switch.h>
#include <linux/module.h>
#include <linux/htc_mode_server.h>
#include <linux/usb/composite.h>
#include <linux/usb/htc_info.h>

#if MINIFB_READY
#include <linux/minifb.h>
#endif

/*16KB*/
#define TXN_MAX 16384
#define RXN_MAX 4096

/* number of rx requests to allocate */
#define ML_RX_REQ_MAX 4

#define DEFAULT_ML_WIDTH			480
#define DEFAULT_ML_HEIGHT		800
#define ML_TOUCH_WIDTH					2048
#define ML_TOUCH_HEIGHT				2048

#define MIRRORLINK_FUNCTION_NAME "mirrorlink"
#define FRAME_INTERVAL_TIME 200
#define CONTEXT_INFO_SIZE			28
#define MAX_NUM_CONTEXT_INFO		15
#define cHSML_UUID_SIZE				18

#define cHSML_12_CAP_ENDIAN         (1 << HSML_12_CAP_ENDIAN)

/* for configfs support */
#define MAX_INST_NAME_LEN	40

static u32 display_setting[32][2] = {
	{640,360},{640,480},{720,480},{720,576},
	{800,480},{800,600},{848,480},{854,480},
	{864,480},{960,540},{1024,600},{1024,768},
	{1152,864},{1280,720},{1280,768},{1280,800},
	{1280,1024},{1360,768},{1400,900},{1400,900},
	{1400,1050},{1600,900},{1600,1200},{1680,1024},
	{1680,1050},{1920,1080},{1920,1200},{0,0},
	{0,0},{0,0},{0,0},{0,0},
};

static DEFINE_MUTEX(hsml_header_lock);

enum {
	ML_OFFLINE,
	ML_ONLINE,
	ML_PROJECTING,
};

enum {
	cHSML_STREAM_OFF = 0,
	cHSML_STREAM_ON,
	cHSML_ON_DEMAND,
};

struct hsml_header07 {
	u8 signature[8];
	u16 seq;
	u32 timestamp;
	u32 frameBufferDataSize;
	u16 num_context_info;
	u8 context_info[492];
} __attribute__ ((__packed__));

#if HSML_VERSION_12
struct hsml_header12 {
	u8 signature[8];
	u32 seq;
	u32 timestamp;
	u32 frameBufferDataSize;
	u16 wWidth;
	u16 wHeight;
	u8 bPixelFormat;
	u8 bEncoding;
	u8 aucReserved[486];
} __attribute__ ((__packed__));
#endif

static struct switch_dev ml_switch = {
	.name = "mirror_link",
};

struct f_mirrorlink_opts {
	struct usb_function_instance func_inst;
	struct hsml_protocol config;
	const char *name;
};

struct mirrorlink_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	struct usb_endpoint_descriptor	*in_desc;
	struct usb_endpoint_descriptor	*out_desc;

	int online;
	int error;

	struct list_head tx_idle;
	struct list_head rx_idle;

	int rx_done;

	u32 bitsPixel;
	u32 framesize;
	u32 width;
	u32 height;
	u8 init_done;
	u8 enabled;
	u16 frame_count;
	u32 rx_req_count;
	u32 tx_req_count;
#if HSML_VERSION_12
    u16 hsml_ver;
    u8 aucUUID[cHSML_UUID_SIZE];
#endif
	struct input_dev *keypad_input;
	struct input_dev *touch_input;
	char *fbaddr;

	atomic_t ml_status;
	atomic_t ml_enable_HSML;
	struct switch_dev ml_status_sdev;
	struct work_struct notifier_display_work;
	struct work_struct notifier_setting_work;

	struct workqueue_struct *wq_display;
	struct work_struct send_fb_work;
	int start_send_fb;

	/* HSML Protocol Info */
	struct hsml_protocol *hsml_proto;

	u8 is_htcmode;
	struct hsml_header07 header;
#if HSML_VERSION_12
	struct hsml_header12 header12;
#endif
	u8 notify_authenticator;
};

static struct usb_interface_descriptor mirrorlink_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor mirrorlink_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mirrorlink_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mirrorlink_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mirrorlink_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_mirrorlink_descs[] = {
	(struct usb_descriptor_header *) &mirrorlink_interface_desc,
	(struct usb_descriptor_header *) &mirrorlink_fullspeed_in_desc,
	(struct usb_descriptor_header *) &mirrorlink_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_mirrorlink_descs[] = {
	(struct usb_descriptor_header *) &mirrorlink_interface_desc,
	(struct usb_descriptor_header *) &mirrorlink_highspeed_in_desc,
	(struct usb_descriptor_header *) &mirrorlink_highspeed_out_desc,
	NULL,
};

/* string descriptors: */
static struct usb_string mirrorlink_string_defs[] = {
	[0].s = "HSML Server",
	{  } /* end of list */
};

static struct usb_gadget_strings mirrorlink_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		mirrorlink_string_defs,
};

static struct usb_gadget_strings *mirrorlink_strings[] = {
	&mirrorlink_string_table,
	NULL,
};

int mirrorlink_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl);
int mirrorlink_check_state(void);
void mirrorlink_reset_state(void);

static inline struct f_mirrorlink_opts *ci_to_f_mirrorlink_opts(
		struct config_item *item)
{
	return container_of(to_config_group(item), struct f_mirrorlink_opts,
			func_inst.group);
}

static inline struct f_mirrorlink_opts *fi_to_f_mirrorlink_opts(
		struct usb_function_instance *fi)
{
	return container_of(fi, struct f_mirrorlink_opts, func_inst);
}
static inline struct mirrorlink_dev *func_to_mirrorlink_dev(
		struct usb_function *f)
{
	return container_of(f, struct mirrorlink_dev, function);
}

#endif //#ifndef _F_MIRRORLINK_H
