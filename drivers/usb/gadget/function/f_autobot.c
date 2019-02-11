/*
 * f_autobot.c -- USB Projector (Autobot) function driver
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

#include "f_autobot.h"

static unsigned short *test_frame;
static int keypad_code[] = {KEY_WAKEUP, 0, 0, 0, KEY_HOME, 0, KEY_BACK};
static const char cand_shortname[] = "htc_cand";
static const char htcmode_shortname[] = "htcmode";
static ktime_t start;
static int touch_init = 0;
static int keypad_init = 0;
static int max_input_current;

static struct autobot_dev *_autobot_dev;

#if BATTERY_READY
extern int htc_battery_set_max_input_current(int target_ma);
#endif

/* the value of htc_mode_status should be one of above status */
static atomic_t htc_mode_status = ATOMIC_INIT(0);

/*
 * 1: enable; 0: disable
 */
void htc_mode_enable(int enable)
{
	struct autobot_dev *dev = _autobot_dev;

	pr_info("enable = %d, current htc_mode_status = %d\n",
			enable, atomic_read(&htc_mode_status));

	if (enable)
		atomic_set(&htc_mode_status, DOCK_ON_AUTOBOT);
	else
		atomic_set(&htc_mode_status, NOT_ON_AUTOBOT);

	schedule_work(&dev->htcmode_notifier_work);
}

int htc_usb_enable_function(char *name, int ebl);
static void usb_setup_android_projector(struct work_struct *work)
{
#if defined(CONFIG_USB_CONFIGFS_UEVENT)
	if (_autobot_dev) {
#if 1
		htc_usb_enable_function("adb,mass_storage,serial,projector", 1);
#else
		htc_usb_enable_function("serial,autobot", 1);
#endif
		atomic_set(&_autobot_dev->cand_online, 1);
		pr_info("startcand %d\n",
				atomic_read(&_autobot_dev->cand_online));
		schedule_work(&_autobot_dev->notifier_work);
	}
#endif //defined(CONFIG_USB_CONFIGFS_UEVENT)
}
static DECLARE_WORK(conf_usb_work, usb_setup_android_projector);

static void battery_set_max_input_current(struct work_struct *work)
{
#if BATTERY_READY
	htc_battery_set_max_input_current(max_input_current);
#endif
}
static DECLARE_WORK(set_current_work, battery_set_max_input_current);

static struct usb_request *autobot_request_new(
		struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void autobot_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

/* add a request to the tail of a list */
static void autobot_req_put(struct autobot_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *autobot_req_get(
		struct autobot_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void autobot_queue_out(struct autobot_dev *dev)
{
	int ret;
	struct usb_request *req;

	/* if we have idle read requests, get them queued */
	while ((req = autobot_req_get(dev, &dev->rx_idle))) {
		req->length = RXN_MAX;
		VDBG("%s: queue %p\n", __func__, req);
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			VDBG("autobot: failed to queue out req (%d)\n", ret);
			dev->error = 1;
			autobot_req_put(dev, &dev->rx_idle, req);
			break;
		}
	}
}

static void touch_event_func(struct autobot_dev *dev,
		struct touch_content *data, int num_touch)
{
	if (num_touch > 0) {
		int i = 0;
		for (i = 0; i < num_touch; i++) {
			input_report_abs(dev->touch_input, ABS_MT_PRESSURE,
							data->pressure);
			input_report_abs(dev->touch_input, ABS_MT_POSITION_X,
							data->x);
			input_report_abs(dev->touch_input, ABS_MT_POSITION_Y,
							data->y);
			input_mt_sync(dev->touch_input);
			data++;
		}
	} else
		input_mt_sync(dev->touch_input);

	input_sync(dev->touch_input);
}

static void autobot_send_multitouch_event(struct autobot_dev *dev,
		char *data)
{
	struct touch_event *event;
	struct touch_content *content;

	event = (struct touch_event *)data;
	if (event->num_touch == 0)
		content = NULL;
	else {
		/* Move to point to touch data */
		content = (struct touch_content *)(data + sizeof(struct touch_event));
	}
	touch_event_func(dev, content, event->num_touch);
}


/* for mouse event type, 1 :move, 2:down, 3:up */
static void autobot_send_touch_event(struct autobot_dev *dev,
	int iPenType, int iX, int iY)
{
	struct input_dev *tdev = dev->touch_input;
	static int b_prePenDown = false;
	static int b_firstPenDown = true;
	static int iCal_LastX;
	static int iCal_LastY;
	static int iReportCount;

	if (iPenType != 3) {
		if (b_firstPenDown) {
			input_report_abs(tdev, ABS_X, iX);
			input_report_abs(tdev, ABS_Y, iY);
			input_report_abs(tdev, ABS_PRESSURE, 100);
			input_report_abs(tdev, ABS_TOOL_WIDTH, 1);
			input_report_key(tdev, BTN_TOUCH, 1);
			input_report_key(tdev, BTN_2, 0);
			input_sync(tdev);
			b_firstPenDown = false;
			b_prePenDown = true; /* For one pen-up only */
			pr_debug("autobot: Pen down %d, %d\n", iX, iY);
		} else {
			/* don't report the same point */
			if (iX != iCal_LastX || iY != iCal_LastY) {
				input_report_abs(tdev, ABS_X, iX);
				input_report_abs(tdev, ABS_Y, iY);
				input_report_abs(tdev, ABS_PRESSURE, 100);
				input_report_abs(tdev, ABS_TOOL_WIDTH, 1);
				input_report_key(tdev, BTN_TOUCH, 1);
				input_report_key(tdev, BTN_2, 0);
				input_sync(tdev);
				iReportCount++;
				if (iReportCount < 10)
					pr_debug("autobot: Pen move %d, %d\n", iX, iY);
			}
		}
	} else if (b_prePenDown) {
		input_report_abs(tdev, ABS_X, iX);
		input_report_abs(tdev, ABS_Y, iY);
		input_report_abs(tdev, ABS_PRESSURE, 0);
		input_report_abs(tdev, ABS_TOOL_WIDTH, 0);
		input_report_key(tdev, BTN_TOUCH, 0);
		input_report_key(tdev, BTN_2, 0);
		input_sync(tdev);
		pr_debug("autobot: Pen up %d, %d\n", iX, iY);
		b_prePenDown = false;
		b_firstPenDown = true;
		iReportCount = 0;
	}
	iCal_LastX = iX;
	iCal_LastY = iY;
}

/* key code: 4 -> home, 5-> menu, 6 -> back, 0 -> system wake */
static void autobot_send_Key_event(struct autobot_dev *dev,
	int iKeycode)
{
	struct input_dev *kdev = dev->keypad_input;
	pr_debug("keycode %d\n", iKeycode);

	/* ics will use default Generic.kl to translate linux keycode WAKEUP
	   to android keycode POWER. by this, device will suspend/resume as
	   we press power key. Even in GB, default qwerty.kl will not do
	   anything for linux keycode WAKEUP, i think we can just drop here.
	*/
	if (iKeycode <= 0 || iKeycode >= sizeof(keypad_code)/sizeof(keypad_code[0]))
		return;

	input_report_key(kdev, keypad_code[iKeycode], 1);
	input_sync(kdev);
	input_report_key(kdev, keypad_code[iKeycode], 0);
	input_sync(kdev);
}


static void autobot_report_key_event(struct autobot_dev *dev,
	struct key_event *event)
{
	struct input_dev *kdev = dev->keypad_input;
	pr_debug("keycode %d, down=%d\n", event->code, event->down);

	input_report_key(kdev, event->code, event->down);
	input_sync(kdev);
}

static void send_fb(struct autobot_dev *dev)
{

	struct usb_request *req;
	int xfer;
	int count = dev->framesize;
	char *frame = NULL;
#if MINIFB_READY
	unsigned long frameSize = 0;
#endif
	int last_pkt = 0;

	if (dev->htcmode_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
#if MINIFB_READY
		if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0) {
			pr_warn("no frame\n");
			return;
		}
#else
		pr_warn("no frame\n");
		return;
#endif
	}

	if (frame == NULL) {
		pr_warn("send_fb: frame == NULL\n");
		return;
	}
	while (count > 0 || last_pkt == 1) {
		req = autobot_req_get(dev, &dev->tx_idle);
		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					autobot_req_put(dev, &dev->tx_idle, req);
					pr_warn("failed to queue req %p\n", req);
					break;
				}
				continue;
			}

			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			memcpy(req->buf, frame, xfer);

			count -= xfer;
			frame += xfer;

			if (count <= 0)
				req->zero = 1;

			if (count  <= 0 && (xfer % 512) == 0)
				last_pkt = 1;

			if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
				autobot_req_put(dev, &dev->tx_idle, req);
				pr_warn("failed to queue req %p\n", req);
				break;
			}
		} else {
			pr_err("send_fb: no req to send\n");
			break;
		}
	}
#if MINIFB_READY
	if (!dev->htcmode_proto->debug_mode)
		minifb_unlockbuf();
#endif
}

static void send_server_info(struct autobot_dev *dev);

static int send_hsml_header(struct autobot_dev *dev)
{
	struct usb_request *req;
	static u16 prev_x;
	static u16 prev_y;
	static u16 prev_w;
	static u16 prev_h;
	int err;

	dev->header.x = 0;
	dev->header.y = 0;
	dev->header.w = dev->htcmode_proto->server_info.width;
	dev->header.h = dev->htcmode_proto->server_info.height;
	if ((prev_x != dev->header.x) || (prev_y != dev->header.y) ||
		(prev_w != dev->header.w) || (prev_h != dev->header.h)) {
		int i = 0;
		u8 *ptr = (u8 *)&dev->header;

		dev->header.msg_id = FB_HEADER_MSGID;
		dev->header.checksum = 0;
		for (i = 0; i < sizeof(struct hsml_header) - 1; i++)
			dev->header.checksum ^= *ptr++;

		prev_x = dev->header.x;
		prev_y = dev->header.y;
		prev_w = dev->header.w;
		prev_h = dev->header.h;
	}

	while (!(req = autobot_req_get(dev, &dev->tx_idle))) {
		msleep(1);
		if (!dev->online)
			break;
	}

	if (req) {
		req->length = sizeof(struct hsml_header);
		memcpy(req->buf, &dev->header, req->length);
		err = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (err < 0) {
			autobot_req_put(dev, &dev->tx_idle, req);
			pr_warn("failed to queue req %p\n", req);
		}
	} else {
		err = -ENODEV;
	}

	return err;
}


static void send_fb2(struct autobot_dev *dev)
{
	struct usb_request *req;
	int xfer;
	static char *frame,*pre_frame;
#if MINIFB_READY
	unsigned long frameSize;
#endif
	int last_pkt = 0;
	int count = dev->htcmode_proto->server_info.width *
				dev->htcmode_proto->server_info.height * (BITSPIXEL / 8);
	ktime_t diff;

	if (dev->htcmode_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
#if MINIFB_READY
		if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0){
			return;
		}
#else
		pr_warn("no frame\n");
		return;
#endif
	}

	if (frame == NULL)
		return;

	/* HTC: same frame check might be removed
	   since minifb_lockbuf() will inform this (?) */

	if (frame == pre_frame && frame != (char *)test_frame) {
		diff = ktime_sub(ktime_get(), start);
		if (ktime_to_ms(diff) < FRAME_INTERVAL_TIME)
			goto unlock;
		else
			start = ktime_get();
	}

	if (dev->htcmode_proto->version >= 0x0006 &&
		send_hsml_header(dev) < 0) {
			pr_warn("failed to send hsml header\n");
			goto unlock;
	}

	pre_frame = frame;

	while ((count > 0 || last_pkt == 1) && dev->online) {

		while (!(req = autobot_req_get(dev, &dev->tx_idle))) {
			msleep(1);

			if (!dev->online)
				break;
		}

		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					autobot_req_put(dev, &dev->tx_idle, req);
					pr_warn("failed to queue req %p\n", req);
					break;
				}
				continue;
			}

			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			if (dev->htcmode_proto->version >= 0x0006 &&
				!dev->htcmode_proto->auth_result) {
				memset(req->buf, 0xFF, xfer);
				pr_err("failed to authenticate\n");
			} else
				memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
				autobot_req_put(dev, &dev->tx_idle, req);
				pr_warn("failed to queue req %p\n", req);
				break;
			}
			count -= xfer;
			frame += xfer;
		} else {
			pr_err("send_fb: no req to send\n");
			break;
		}
	}

unlock:
	if (!dev->htcmode_proto->debug_mode){
#if MINIFB_READY
		minifb_unlockbuf();
#endif
	}
}

static void send_fb_do_work(struct work_struct *work)
{
	struct autobot_dev *dev = _autobot_dev;
	start = ktime_get();
	while (dev->start_send_fb) {
		send_fb2(dev);
		msleep(1);
	}
}

static void send_fb_do_work_legacy(struct work_struct *work)
{
	struct autobot_dev *dev = _autobot_dev;

	if (!dev->online)
		return;

	send_fb(dev);
	dev->frame_count++;
	/* 30s send system wake code */
	if (dev->frame_count == 30 * 30) {
		autobot_send_Key_event(dev, 0);
		dev->frame_count = 0;
	}
}

static void send_info(struct autobot_dev *dev)
{
	struct usb_request *req;

	req = autobot_req_get(dev, &dev->tx_idle);
	if (req) {
		req->length = 20;
		memcpy(req->buf, "okay", 4);
		memcpy(req->buf + 4, &dev->bitsPixel, 4);
		memcpy(req->buf + 8, &dev->framesize, 4);
		memcpy(req->buf + 12, &dev->width, 4);
		memcpy(req->buf + 16, &dev->height, 4);
		if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
			autobot_req_put(dev, &dev->tx_idle, req);
			pr_warn("failed to queue req %p\n",	req);
		}
	} else
		pr_info("no req to send\n");
}

static void send_server_info(struct autobot_dev *dev)
{
	struct usb_request *req;

	req = autobot_req_get(dev, &dev->tx_idle);
	if (req) {
		req->length = sizeof(struct msm_server_info);
		memcpy(req->buf, &dev->htcmode_proto->server_info, req->length);
		if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
			autobot_req_put(dev, &dev->tx_idle, req);
			pr_warn("failed to queue req %p\n",	req);
		}
	} else {
		pr_info("no req to send\n");
	}
}

static struct size rotate(struct size v)
{
	struct size r;
	r.w = v.h;
	r.h = v.w;
	return r;
}

static struct size get_projection_size(struct autobot_dev *dev,
		struct msm_client_info *client_info)
{
	int server_width = 0;
	int server_height = 0;
	struct size client;
	struct size server;
	struct size ret;
	int perserve_aspect_ratio = client_info->display_conf & (1 << 0);
	int server_orientation = 0;
	int client_orientation = (client_info->width > client_info->height);
	int align_w = 0;

	server_width = dev->width;
	server_height = dev->height;

	server_orientation = (server_width > server_height);

	pr_debug("perserve_aspect_ratio= %d\n", perserve_aspect_ratio);

	client.w = client_info->width;
	client.h = client_info->height;
	server.w = server_width;
	server.h = server_height;

	if (server_orientation != client_orientation)
		client = rotate(client);

	align_w = client.h * server.w > server.h * client.w;

	if (perserve_aspect_ratio) {
		if (align_w) {
			ret.w = client.w;
			ret.h = (client.w * server.h) / server.w;
		} else {
			ret.w = (client.h * server.w) / server.h;
			ret.h = client.h;
		}

		ret.w = round_down(ret.w, 32);
	} else {
		ret = client;
	}

	pr_debug("autobot projector size(w=%d, h=%d)\n", ret.w, ret.h);

	return ret;
}

#if 0
static void projector_get_msmfb(struct projector_dev *dev)
{
    struct msm_fb_info fb_info;

	msmfb_get_var(&fb_info);

	dev->bitsPixel = BITSPIXEL;
	dev->width = fb_info.xres;
	dev->height = fb_info.yres;
	dev->fbaddr = get_fb_addr();
	dev->framesize = dev->width * dev->height * (dev->bitsPixel / 8);
	printk(KERN_INFO "projector: width %d, height %d framesize %d, %p\n",
		   fb_info.xres, fb_info.yres, dev->framesize, dev->fbaddr);
}
#endif

static void autobot_enable_fb_work(struct autobot_dev *dev, int enabled)
{
	dev->start_send_fb = enabled;
	if (enabled) {
		queue_work(dev->wq_display, &dev->send_fb_work);

		if (atomic_inc_return(&htc_mode_status) != HTC_MODE_RUNNING)
			atomic_dec(&htc_mode_status);
		pr_info("startfb current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
	} else {
		if (atomic_dec_return(&htc_mode_status) != DOCK_ON_AUTOBOT)
			atomic_inc(&htc_mode_status);
		pr_info("endfb current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
	}
	schedule_work(&dev->htcmode_notifier_work);
}

/*
 * Handle common messages and return 1 if message has been handled
 */
static int autobot_handle_common_msg(struct autobot_dev *dev,
		struct usb_request *req)
{
	unsigned char *data = req->buf;
	int handled = 1;

	if (!strncmp("startfb", data, 7)) {
		autobot_enable_fb_work(dev, 1);
	} else if (!strncmp("endfb", data, 5)) {
		autobot_enable_fb_work(dev, 0);
	} else {
		handled = 0;
	}

	return handled;
}
/*
 * Handle HTC Mode specific messages and return 1 if message has been handled
 */
static int autobot_handle_htcmode_msg(struct autobot_dev *dev,
		struct usb_request *req)
{
	unsigned char *data = req->buf;
	int handled = 1;
	struct size projector_size;

	if ((data[0] == CLIENT_INFO_MESGID) && (req->actual == sizeof(struct msm_client_info))) {
		memcpy(&dev->htcmode_proto->client_info, req->buf, sizeof(struct msm_client_info));

		projector_size = get_projection_size(dev, &dev->htcmode_proto->client_info);

		dev->htcmode_proto->server_info.mesg_id = SERVER_INFO_MESGID;
		dev->htcmode_proto->server_info.width = projector_size.w;
		dev->htcmode_proto->server_info.height = projector_size.h;
		dev->htcmode_proto->server_info.pixel_format = PIXEL_FORMAT_RGB565;
		dev->htcmode_proto->server_info.ctrl_conf = CTRL_CONF_TOUCH_EVENT_SUPPORTED |
									  CTRL_CONF_NUM_SIMULTANEOUS_TOUCH;
		send_server_info(dev);
	} else if (dev->htcmode_proto->version >= 0x0006 &&
			data[0] == HSML_TOUCH_EVENT_ID) {
		autobot_send_multitouch_event(dev, data);
	} else if (dev->htcmode_proto->version >= 0x0006 &&
			data[0] == HSML_KEY_EVENT_ID) {
		autobot_report_key_event(dev, (struct key_event *)data);
	} else if (!strncmp("startcand", data, 9)) {
		/*
		 * Ignore this message because we already started the CAN daemon at
		 * very beginning.
		 */
	} else if (!strncmp("endcand", data, 7)) {
		atomic_set(&dev->cand_online, 0);
		pr_info("endcand %d\n", atomic_read(&dev->cand_online));

		schedule_work(&dev->notifier_work);
	} else {
		handled = autobot_handle_common_msg(dev, req);
	}

	return handled;
}

static void autobot_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct autobot_dev *dev = _autobot_dev;
	autobot_req_put(dev, &dev->tx_idle, req);
}

static void autobot_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct autobot_dev *dev = _autobot_dev;
	unsigned char *data = req->buf;
	int mouse_data[3];
	int i;
	int handled = 0;
	VDBG("%s: status %d, %d bytes\n", __func__,
		req->status, req->actual);

	if (req->status != 0) {
		dev->error = 1;
		autobot_req_put(dev, &dev->rx_idle, req);
		return ;
	}

	if (dev->is_htcmode)
		handled = autobot_handle_htcmode_msg(dev, req);

	if (!handled)
		handled = autobot_handle_common_msg(dev, req);

	if (!handled) {
		/* for mouse event type, 1 :move, 2:down, 3:up */
		mouse_data[0] = *((int *)(req->buf));

		if (!strncmp("init", data, 4)) {
			dev->init_done = 1;
			dev->bitsPixel = BITSPIXEL;
			dev->width = DEFAULT_PROJ_WIDTH;
			dev->height = DEFAULT_PROJ_HEIGHT;
			dev->framesize = dev->width * dev->height * (BITSPIXEL / 8);

			send_info(dev);
			/* system wake code */
			autobot_send_Key_event(dev, 0);

			atomic_set(&htc_mode_status, HTC_MODE_RUNNING);
			pr_info("init current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
			schedule_work(&dev->htcmode_notifier_work);
		} else if (*data == ' ') {
			queue_work(dev->wq_display, &dev->send_fb_work_legacy);
		} else if (mouse_data[0] > 0) {
			 if (mouse_data[0] < 4) {
				for (i = 0; i < 3; i++)
					mouse_data[i] = *(((int *)(req->buf))+i);
				autobot_send_touch_event(dev,
					mouse_data[0], mouse_data[1], mouse_data[2]);
			} else {
				autobot_send_Key_event(dev, mouse_data[0]);
				pr_info("autobot: Key command data %02x, keycode %d\n",
					*((char *)(req->buf)), mouse_data[0]);
			}
		} else if (mouse_data[0] != 0)
			pr_err("autobot: Unknown command data %02x, mouse %d,%d,%d\n",
				*((char *)(req->buf)), mouse_data[0], mouse_data[1], mouse_data[2]);
	}
	autobot_req_put(dev, &dev->rx_idle, req);
	autobot_queue_out(dev);
}

static int autobot_create_bulk_endpoints(struct autobot_dev *dev,
		struct usb_endpoint_descriptor *in_desc,
		struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	pr_debug("autobot_create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		pr_debug("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	pr_debug("usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		pr_debug("usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	pr_debug("usb_ep_autoconfig for autobot ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	for (i = 0; i < dev->rx_req_count; i++) {
		req = autobot_request_new(dev->ep_out, RXN_MAX);
		if (!req)
			goto fail;
		req->complete = autobot_complete_out;
		autobot_req_put(dev, &dev->rx_idle, req);
	}
	for (i = 0; i < dev->tx_req_count; i++) {
		req = autobot_request_new(dev->ep_in, TXN_MAX);
		if (!req)
			goto fail;
		req->complete = autobot_complete_in;
		autobot_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	while ((req = autobot_req_get(dev, &dev->tx_idle)))
		autobot_request_free(req, dev->ep_in);
	while ((req = autobot_req_get(dev, &dev->rx_idle)))
		autobot_request_free(req, dev->ep_out);
	pr_err("autobot: could not allocate requests\n");
	return -1;
}

static int autobot_touch_init(struct autobot_dev *dev)
{
	int x = TOUCH_WIDTH;
	int y = TOUCH_HEIGHT;
	int ret = 0;
	struct input_dev *tdev = dev->touch_input;
	if (touch_init) {
		pr_info("autobot_touch already initial\n");
		return 0;
	}

	pr_info("x=%d y=%d\n", x, y);
	dev->touch_input  = input_allocate_device();
	if (dev->touch_input == NULL) {
		pr_err("Failed to allocate input device\n");
		return -1;
	}
	tdev = dev->touch_input;
	tdev->name = "autobot_input";
	set_bit(EV_SYN,    tdev->evbit);
	set_bit(EV_KEY,    tdev->evbit);
	set_bit(EV_ABS,    tdev->evbit);


	/* Set input parameters boundary. */
	if (dev->htcmode_proto->version < 0x0006) {
		pr_info("single-touch support\n");

		set_bit(BTN_TOUCH, tdev->keybit);
		set_bit(BTN_2,     tdev->keybit);

		input_set_abs_params(tdev, ABS_X, 0, x, 0, 0);
		input_set_abs_params(tdev, ABS_Y, 0, y, 0, 0);
		input_set_abs_params(tdev, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(tdev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
		input_set_abs_params(tdev, ABS_HAT0X, 0, x, 0, 0);
		input_set_abs_params(tdev, ABS_HAT0Y, 0, y, 0, 0);
	} else {
		pr_info("multi-touch support\n");
		input_set_abs_params(tdev, ABS_MT_POSITION_X, 0, x, 0, 0);
		input_set_abs_params(tdev, ABS_MT_POSITION_Y, 0, y, 0, 0);
		input_set_abs_params(tdev, ABS_MT_PRESSURE, 0, 1, 0, 0);
	}

	ret = input_register_device(tdev);
	if (ret) {
		pr_err("Unable to register %s input device\n", tdev->name);
		input_free_device(tdev);
		return -1;
	}
	touch_init = 1;
	pr_info("autobot_touch_init OK\n");
	return 0;
}

static int autobot_keypad_init(struct autobot_dev *dev)
{
	struct input_dev *kdev;
	if (keypad_init) {
		pr_info("autobot_keypad already initial\n");
		return 0;
	}
	/* Initialize input device info */
	dev->keypad_input = input_allocate_device();
	if (dev->keypad_input == NULL) {
		pr_err("Failed to allocate input device\n");
		return -1;
	}
	kdev = dev->keypad_input;
	set_bit(EV_KEY, kdev->evbit);
	set_bit(KEY_SEND, kdev->keybit);
	set_bit(KEY_END, kdev->keybit);
	set_bit(KEY_MUTE, kdev->keybit);
	set_bit(KEY_PLAY, kdev->keybit);
	set_bit(KEY_PAUSE, kdev->keybit);
	set_bit(KEY_STOP, kdev->keybit);
	set_bit(KEY_NEXTSONG, kdev->keybit);
	set_bit(KEY_PLAYPAUSE, kdev->keybit);
	set_bit(KEY_PREVIOUSSONG, kdev->keybit);
	set_bit(KEY_FORWARD, kdev->keybit);
	set_bit(KEY_REWIND, kdev->keybit);
	set_bit(KEY_VOLUMEDOWN, kdev->keybit);
	set_bit(KEY_VOLUMEUP, kdev->keybit);
	set_bit(KEY_HOME, kdev->keybit);
	set_bit(KEY_BACK, kdev->keybit);
	set_bit(KEY_SEARCH, kdev->keybit);
	set_bit(KEY_ENTER, kdev->keybit);
	set_bit(KEY_DELETE, kdev->keybit);
	set_bit(KEY_ZOOMIN, kdev->keybit);
	set_bit(KEY_ZOOMOUT, kdev->keybit);

	set_bit(KEY_WAKEUP, kdev->keybit);

	kdev->name = "autobot-Keypad";
	kdev->phys = "input2";
	kdev->id.bustype = BUS_HOST;
	kdev->id.vendor = 0x0123;
	kdev->id.product = 0x5220 /*dummy value*/;
	kdev->id.version = 0x0100;
	kdev->keycodesize = sizeof(unsigned int);

	/* Register linux input device */
	if (input_register_device(kdev) < 0) {
		pr_err("Unable to register %s input device\n", kdev->name);
		input_free_device(kdev);
		return -1;
	}
	pr_info("autobot_keypad_init OK\n");
	keypad_init = 1;
	return 0;
}

static int autobot_function_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct autobot_dev *dev = func_to_autobot_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	pr_debug("%s begin\n", __func__);

	dev->bitsPixel = BITSPIXEL;
	dev->width = DEFAULT_PROJ_WIDTH;
	dev->height = DEFAULT_PROJ_HEIGHT;
	dev->rx_req_count = PROJ_RX_REQ_MAX;
	dev->tx_req_count = (dev->width * dev->height * 2 / TXN_MAX) + 5;
	pr_debug("resolution: %u*%u"
		", rx_cnt: %u, tx_cnt:%u\n", dev->width, dev->height,
		dev->rx_req_count, dev->tx_req_count);
	if (autobot_touch_init(dev) < 0)
		return -1;
	if (autobot_keypad_init(dev) < 0)
		return -1;

	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->tx_idle);

	dev->wq_display = create_singlethread_workqueue("autobot_mode");
	if (!dev->wq_display)
		return -1;

#if 0 // there is warning
	workqueue_set_max_active(dev->wq_display, 1);
#endif

	INIT_WORK(&dev->send_fb_work, send_fb_do_work);
	INIT_WORK(&dev->send_fb_work_legacy, send_fb_do_work_legacy);

	/* allocate string ID(s) */
	if (autobot_string_defs[0].id == 0) {
		id = usb_string_id(c->cdev);
		if (id < 0)
			return id;
		autobot_string_defs[0].id = id;
		autobot_interface_desc.iInterface = id;
	}

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	autobot_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = autobot_create_bulk_endpoints(dev, &autobot_fullspeed_in_desc,
			&autobot_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		autobot_highspeed_in_desc.bEndpointAddress =
			autobot_fullspeed_in_desc.bEndpointAddress;
		autobot_highspeed_out_desc.bEndpointAddress =
			autobot_fullspeed_out_desc.bEndpointAddress;
	}

	pr_debug("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}


static int autobot_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct autobot_dev *dev = func_to_autobot_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	pr_debug("intf: %d alt: %d\n", intf, alt);

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		pr_err("config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		pr_err("failed to enable ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		pr_err("config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		pr_err("failed to enable ep %s, result %d\n",
				dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->online = 1;

	autobot_queue_out(dev);

	return 0;
}

static void cand_online_notify(struct work_struct *w)
{
	struct autobot_dev *dev = container_of(w,
					struct autobot_dev, notifier_work);
	pr_debug("%s begin\n", __func__);
	switch_set_state(&dev->cand_sdev, atomic_read(&dev->cand_online));
}

static void htcmode_status_notify(struct work_struct *w)
{
	struct autobot_dev *dev = container_of(w,
					struct autobot_dev, htcmode_notifier_work);
	pr_debug("%s begin\n", __func__);

	if (dev->htcmode_proto->notify_authenticator) {
		char *envp[] = {
				"SWITCH_NAME=htcmode",
				"SWITCH_STATE=check_client_sig", 0
		};
		kobject_uevent_env(&dev->htcmode_sdev.dev->kobj, KOBJ_CHANGE, envp);
		dev->htcmode_proto->notify_authenticator = 0;
	} else
		switch_set_state(&dev->htcmode_sdev, atomic_read(&htc_mode_status));
}

int check_htc_mode_status(void)
{
	return atomic_read(&htc_mode_status);
}

static ssize_t print_cand_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", cand_shortname);
}

static ssize_t print_cand_switch_state(struct switch_dev *cand_sdev, char *buf)
{
	struct autobot_dev *dev = container_of(cand_sdev,
					struct autobot_dev, cand_sdev);
	return sprintf(buf, "%s\n", (atomic_read(&dev->cand_online) ?
		"online" : "offline"));
}

static ssize_t print_htcmode_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", htcmode_shortname);
}

static ssize_t print_htcmode_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", (atomic_read(&htc_mode_status) == HTC_MODE_RUNNING ?
		"projecting" : (atomic_read(&htc_mode_status) == DOCK_ON_AUTOBOT ?
		"online" : "offline")));
}

static void autobot_function_disable(struct usb_function *f)
{
	struct autobot_dev *dev = func_to_autobot_dev(f);

	pr_debug("%s begin\n", __func__);

	dev->start_send_fb = false;
	dev->online = 0;
	dev->error = 1;
/*    This is changed from unbind function.
 *    Because it need once android function bind when switching USB functions by framework,
 *    the total function bind is twice.
 *    Before second USB functions bind, the enabled functions will unbind.
 *    However, if this flag is in unbind function, it will be clear to 0 so that projector
 *    can't enable the HTC mode. Therefore, we change this flag to disable function. When
 *    USB disconnect to autobot, the disable function will be executed.
 */
	dev->is_htcmode = 0;
	htc_mode_enable(0);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	atomic_set(&dev->cand_online, 0);
	schedule_work(&dev->notifier_work);

	pr_debug("%s disabled\n", dev->function.name);
}


static void autobot_function_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct autobot_dev *dev = func_to_autobot_dev(f);
	struct usb_request *req;

	pr_debug("%s begin\n", __func__);

	destroy_workqueue(dev->wq_display);

	while ((req = autobot_req_get(dev, &dev->tx_idle)))
		autobot_request_free(req, dev->ep_in);
	while ((req = autobot_req_get(dev, &dev->rx_idle)))
		autobot_request_free(req, dev->ep_out);

	dev->online = 0;
	dev->error = 1;
	if (dev->htcmode_proto)
		dev->htcmode_proto->auth_in_progress = 0;

/*
	touch_init = 0;
	keypad_init = 0;

	if (dev->touch_input) {
		input_unregister_device(dev->touch_input);
		input_free_device(dev->touch_input);
	}
	if (dev->keypad_input) {
		input_unregister_device(dev->keypad_input);
		input_free_device(dev->keypad_input);
	}
*/
}

static int autobot_setup(struct htcmode_protocol *config)
{
	struct autobot_dev *dev;
	int ret = 0;

	const char sig[] = {
		0xFF, 0xFF, 0x48, 0x53,
		0x4D, 0x4C, 0xFF, 0xFF
	};

	pr_debug("%s begin\n", __func__);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	_autobot_dev = dev;

	dev->htcmode_proto = config;
	dev->htcmode_proto->server_info.height = DEFAULT_PROJ_HEIGHT;
	dev->htcmode_proto->server_info.width = DEFAULT_PROJ_WIDTH;
	dev->htcmode_proto->client_info.display_conf = 0;
	memcpy(&dev->header.signature, sig, sizeof(dev->header.signature));
	dev->htcmode_proto->notify_authenticator = 0;

	INIT_WORK(&dev->notifier_work, cand_online_notify);
	INIT_WORK(&dev->htcmode_notifier_work, htcmode_status_notify);

	dev->cand_sdev.name = cand_shortname;
	dev->cand_sdev.print_name = print_cand_switch_name;
	dev->cand_sdev.print_state = print_cand_switch_state;
	ret = switch_dev_register(&dev->cand_sdev);
	if (ret < 0) {
		pr_err("usb cand_sdev switch_dev_register register fail\n");
		goto err_free;
	}

	dev->htcmode_sdev.name = htcmode_shortname;
	dev->htcmode_sdev.print_name = print_htcmode_switch_name;
	dev->htcmode_sdev.print_state = print_htcmode_switch_state;
	ret = switch_dev_register(&dev->htcmode_sdev);
	if (ret < 0) {
		pr_err("usb htcmode_sdev switch_dev_register register fail\n");
		goto err_unregister_cand;
	}

	return 0;

err_unregister_cand:
	switch_dev_unregister(&dev->cand_sdev);
err_free:
	kfree(dev);
	_autobot_dev = NULL;
	pr_err("projector (autobot) gadget driver failed to initialize, err=%d\n", ret);
	return ret;

}

static void autobot_free_func(struct usb_function *f)
{
	//struct autobot_dev *dev = func_to_autobot_dev(f);

	pr_info("%s begin\n", __func__);

	//switch_dev_unregister(&dev->cand_sdev);
	//switch_dev_unregister(&dev->htcmode_sdev);

	//kfree(dev);
}

static void autobot_complete_req(struct usb_ep *ep, struct usb_request *req)
{
	struct autobot_dev *dev = ep->driver_data;
	int length = req->actual;
	char *dst = NULL;

	pr_debug("status=%d, request=0x%02X\n",
			req->status, dev->htcmode_proto->request);

	if (req->status != 0) {
		pr_warn("autobot_complete_req, err %d\n", req->status);
		return;
	}

	switch (dev->htcmode_proto->request) {
	case HSML_06_REQ_SEND_CLIENT_INFO:
		dst = (char *)&dev->htcmode_proto->client_info;
		memcpy(dst + 1, req->buf, length);
		break;

	case HSML_06_REQ_SEND_EXTENDED_CLIENT_INFO:
		break;

	case HSML_06_REQ_SET_CLIENT_AUTH:
		dst = (char *)&dev->htcmode_proto->client_sig;
		memcpy(dst, req->buf, length);
		dev->htcmode_proto->notify_authenticator = 1;
		schedule_work(&dev->htcmode_notifier_work);
		break;

	default:
		break;
	}
}

int autobot_ctrlrequest(struct usb_composite_dev *cdev,
		const struct usb_ctrlrequest *ctrl)
{
	struct autobot_dev *dev = _autobot_dev;
	int value = -EOPNOTSUPP;
	u16	w_length = le16_to_cpu(ctrl->wLength);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_index = le16_to_cpu(ctrl->wIndex);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		char *ptr = (char *)cdev->req->buf;
		struct size projector_size;
		int i = 0;

		pr_debug("request(req=0x%02x, wValue=%d, "
						 "wIndex=%d, wLength=%d)\n",
						 ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
		if (ctrl->bRequest == HTC_MODE_CONTROL_REQ) {
			if (check_htc_mode_status() == NOT_ON_AUTOBOT) {
				if (dev) {
					dev->htcmode_proto->vendor = w_index;
					dev->htcmode_proto->version = w_value;
					/*
					 * 0x0034 is for Autobot. It is not a correct HTC mode version.
					 */
					if (dev->htcmode_proto->version == 0x0034)
						dev->htcmode_proto->version = 0x0003;
					dev->is_htcmode = 1;
					dev->htcmode_proto->notify_authenticator = 0;
					pr_debug("HTC Mode version = 0x%04X\n", dev->htcmode_proto->version);
					pr_debug("HSML Client vendor = 0x%04X\n", dev->htcmode_proto->vendor);
					dev->htcmode_proto->auth_in_progress = 0;
					dev->htcmode_proto->auth_result = 1;
					htc_mode_enable(1);

					schedule_work(&conf_usb_work);
				} else {
					pr_err("autobot_dev is NULL!!");
				}
				value = 0;
			}
		}
		else if (check_htc_mode_status() != NOT_ON_AUTOBOT) {
			switch (ctrl->bRequest) {
			case HSML_06_REQ_GET_SERVER_VERSION:
				ptr[0] = (char)(HSML_PROTOCOL_VERSION >> 8);
				ptr[1] = (char)(HSML_PROTOCOL_VERSION & 0xFF);
				value = sizeof(u16);
				break;

			case HSML_06_REQ_GET_SERVER_INFO:
				projector_size = get_projection_size(dev, &dev->htcmode_proto->client_info);

				dev->htcmode_proto->server_info.mesg_id = SERVER_INFO_MESGID;
				dev->htcmode_proto->server_info.width = projector_size.w;
				dev->htcmode_proto->server_info.height = projector_size.h;
				dev->htcmode_proto->server_info.pixel_format = PIXEL_FORMAT_RGB565;
				dev->htcmode_proto->server_info.ctrl_conf = CTRL_CONF_TOUCH_EVENT_SUPPORTED |
											  CTRL_CONF_NUM_SIMULTANEOUS_TOUCH;
				value = w_length;
				ptr = (char *)&dev->htcmode_proto->server_info;
				memcpy(cdev->req->buf, ptr + 1, w_length);
				break;

			case HSML_06_REQ_GET_FB:
				if (!w_value)
					autobot_enable_fb_work(dev, 1);
				else
					queue_work(dev->wq_display, &dev->send_fb_work_legacy);
				value = 0;
				break;

			case HSML_06_REQ_STOP:
				autobot_enable_fb_work(dev, 0);
				value = 0;
				break;

			case HSML_06_REQ_GET_SERVER_NONCE:
				if (w_length == HSML_SERVER_NONCE_SIZE) {
					for (i = 0; i < w_length / sizeof(int); i++)
						*((int *)(dev->htcmode_proto->nonce + sizeof(int) * i)) = get_random_int();
					memcpy(cdev->req->buf, dev->htcmode_proto->nonce, w_length);
				} else
					pr_err("The request size for server nonce is incorrect. (w_length=%d)",
							w_length);
				value = w_length;
				dev->htcmode_proto->auth_result = 0;
				dev->htcmode_proto->auth_in_progress = 1;
				break;

			case HSML_06_REQ_SET_CLIENT_AUTH:
			case HSML_06_REQ_SEND_CLIENT_INFO:
			case HSML_06_REQ_SEND_EXTENDED_CLIENT_INFO:
				cdev->gadget->ep0->driver_data = dev;
				cdev->req->complete = autobot_complete_req;
				dev->htcmode_proto->request = ctrl->bRequest;
				value = w_length;
				break;

			case HSML_06_REQ_GET_SERVER_AUTH:
				/*
				 * Respond with STALL to tell the host that the authentication is
				 * in progress.
				 */
				if (!dev->htcmode_proto->auth_in_progress) {
					memcpy(cdev->req->buf,
							dev->htcmode_proto->server_sig, w_length);
					value = w_length;
				}
				break;
			case HSML_06_REQ_SET_MAX_CHARGING_CURRENT:
				max_input_current = (int) w_value;
				schedule_work(&set_current_work);
				value = 0;
				break;

			default:
				pr_debug("unrecognized request(req=0x%02x, wValue=%d, "
								 "wIndex=%d, wLength=%d)\n",
						ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
				break;
			}
		}
	}

	if (value >= 0) {
		cdev->req->zero = 0;
		cdev->req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (value < 0)
			pr_err("setup response queue error\n");
	}

	return value;
}

static int autobot_ctrlreq_configfs(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	if (f->config != NULL && f->config->cdev != NULL)
		return autobot_ctrlrequest(f->config->cdev, ctrl);
	else
		return -1;
}

static int autobot_set_inst_name(struct usb_function_instance *fi,
		const char *name)
{
	struct f_autobot_opts *opts = fi_to_f_autobot_opts(fi);
	char *ptr;
	int name_len;

	name_len = strlen(name) + 1;
	if (name_len > MAX_INST_NAME_LEN)
		return -ENAMETOOLONG;

	ptr = kstrndup(name, name_len, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	opts->name = ptr;

	return 0;
}

static void autobot_free_instance(struct usb_function_instance *fi)
{
	struct f_autobot_opts *opts = fi_to_f_autobot_opts(fi);

	kfree(opts);
}

static void autobot_opts_release(struct config_item *item)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations autobot_item_ops = {
	.release	= autobot_opts_release,
};

static ssize_t autobot_width_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->server_info.width);
}
CONFIGFS_ATTR_RO(autobot_, width);

static ssize_t autobot_height_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->server_info.height);
}
CONFIGFS_ATTR_RO(autobot_, height);

static ssize_t autobot_rotation_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", (config->client_info.display_conf & CLIENT_INFO_SERVER_ROTATE_USED));
}
CONFIGFS_ATTR_RO(autobot_, rotation);

static ssize_t autobot_version_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->version);
}
CONFIGFS_ATTR_RO(autobot_, version);

static ssize_t autobot_vendor_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->vendor);
}
CONFIGFS_ATTR_RO(autobot_, vendor);

static ssize_t autobot_server_nonce_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	memcpy(page, config->nonce, HSML_SERVER_NONCE_SIZE);
	return HSML_SERVER_NONCE_SIZE;
}
CONFIGFS_ATTR_RO(autobot_, server_nonce);

static ssize_t autobot_client_sig_show(struct config_item *item, char *page)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	memcpy(page, config->client_sig, HSML_CLIENT_SIG_SIZE);
	return HSML_CLIENT_SIG_SIZE;
}
CONFIGFS_ATTR_RO(autobot_, client_sig);

static ssize_t autobot_server_sig_store(struct config_item *item,
		const char *page, size_t len)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	memcpy(config->server_sig, page, HSML_SERVER_SIG_SIZE);
	return HSML_SERVER_SIG_SIZE;
}
CONFIGFS_ATTR_WO(autobot_, server_sig);

static ssize_t autobot_auth_store(struct config_item *item,
		const char *page, size_t len)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	memcpy(&config->auth_result, page, sizeof(config->auth_result));
	config->auth_in_progress = 0;
	return sizeof(config->auth_result);
}
CONFIGFS_ATTR_WO(autobot_, auth);

static ssize_t autobot_debug_mode_store(struct config_item *item,
		const char *page, size_t len)
{
	struct f_autobot_opts *opts = ci_to_f_autobot_opts(item);
	struct htcmode_protocol *config = &opts->config;
	int value, i;
	int framesize = DEFAULT_PROJ_HEIGHT * DEFAULT_PROJ_WIDTH;

	if (sscanf(page, "%d", &value) == 1) {
		if (!test_frame)
			test_frame = kzalloc(framesize * 2, GFP_KERNEL);

		if (test_frame)
			for (i = 0 ; i < framesize ; i++)
				if (i < framesize/4)
					test_frame[i] = 0xF800;
				else if (i < framesize*2/4)
					test_frame[i] = 0x7E0;
				else if (i < framesize*3/4)
					test_frame[i] = 0x1F;
				else
					test_frame[i] = 0xFFFF;

		config->debug_mode = value;
		return len;
	}
	return -EINVAL;
}
CONFIGFS_ATTR_WO(autobot_, debug_mode);

static struct configfs_attribute *autobot_attrs[] = {
	&autobot_attr_width,
	&autobot_attr_height,
	&autobot_attr_rotation,
	&autobot_attr_version,
	&autobot_attr_vendor,
	&autobot_attr_server_nonce,
	&autobot_attr_client_sig,
	&autobot_attr_server_sig,
	&autobot_attr_auth,
	&autobot_attr_debug_mode,
	NULL,
};

static struct config_item_type autobot_func_type = {
	.ct_item_ops	= &autobot_item_ops,
	.ct_attrs	= autobot_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct usb_function_instance *autobot_alloc_inst(void)
{
	struct f_autobot_opts *opts;
	int ret;

	pr_debug("%s enter\n", __func__);

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.set_inst_name = autobot_set_inst_name;
	opts->func_inst.free_func_inst = autobot_free_instance;

	ret = autobot_setup(&opts->config);
	if (ret) {
		kfree(opts);
		pr_err("Error setting autobot\n");
		return ERR_PTR(ret);
	}

	config_group_init_type_name(&opts->func_inst.group, "",
			&autobot_func_type);
	return &opts->func_inst;
}

static struct usb_function *autobot_alloc_func(struct usb_function_instance *fi)
{
	//struct f_autobot_opts *opts = fi_to_f_autobot_opts(fi);
	struct autobot_dev *dev = _autobot_dev;

	pr_debug("%s enter\n", __func__);

	dev->function.name = "Autobot Projector Function";
	dev->function.strings = autobot_strings;

	dev->function.fs_descriptors = fs_autobot_descs;
	dev->function.hs_descriptors = hs_autobot_descs;
	dev->function.ss_descriptors = ss_autobot_descs;

	dev->function.bind = autobot_function_bind;
	dev->function.unbind = autobot_function_unbind;
	dev->function.free_func = autobot_free_func;
	dev->function.set_alt = autobot_function_set_alt;
	//dev->function.get_alt =
	dev->function.disable = autobot_function_disable;
	dev->function.setup = autobot_ctrlreq_configfs;
	//dev->function.req_match =
	//dev->function.suspend =
	//dev->function.resume =
	//dev->function.get_status =
	//dev->function.func_suspend =

	return &dev->function;
}

DECLARE_USB_FUNCTION_INIT(autobot, autobot_alloc_inst, autobot_alloc_func);
MODULE_DESCRIPTION("USB Projector (Autobot) Driver");
MODULE_AUTHOR("Kyle Tso");
MODULE_LICENSE("GPL v2");
