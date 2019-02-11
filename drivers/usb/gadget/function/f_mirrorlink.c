/*
 * f_mirrorlink.c -- USB Projector (Mirrorlink) function driver
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

#include "f_mirrorlink.h"

static const char ml_shortname[] = "ml_status";
static unsigned short *test_frame;
static int touch_init_ml = 0;
static struct mirrorlink_dev *_ml_dev = NULL;

int mirrorlink_check_state(void) {
	struct mirrorlink_dev *dev = _ml_dev;

	if (!dev)
		return -EINVAL;

	return atomic_read(&dev->ml_enable_HSML);
}

void mirrorlink_reset_state(void) {
	switch_set_state(&ml_switch, 0);
	return;
}

static struct usb_request *mirrorlink_request_new(struct usb_ep *ep,
		int buffer_size)
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

static void mirrorlink_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

/* add a request to the tail of a list */
static void mirrorlink_req_put(struct mirrorlink_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *mirrorlink_req_get(struct mirrorlink_dev *dev, struct list_head *head)
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

static int send_hsml_header07(struct mirrorlink_dev *dev)
{
	struct usb_request *req;
	int err;
	static u32 seq;

	/* prevent seq overflow */
	seq &= 0x0000ffff;
	dev->header.seq = htons((u16)seq++);
	dev->header.timestamp = htonl((u32)ktime_to_ms(ktime_get()));
	dev->header.frameBufferDataSize = htonl(dev->hsml_proto->set_display_info.wHeight *
			dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8));

	while (!(req = mirrorlink_req_get(dev, &dev->tx_idle))) {
		msleep(1);
		if (!dev->online)
			break;
	}

	if (req) {
		req->length = sizeof(struct hsml_header);
		mutex_lock(&hsml_header_lock);
		memcpy(req->buf, &dev->header, req->length);
		err = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (err < 0) {
			mirrorlink_req_put(dev, &dev->tx_idle, req);
			pr_warn("%s: failed to queue req"
				    " %p\n", __func__, req);
		}
		if (dev->header.num_context_info != 0) {
			dev->header.num_context_info = 0;
			memset(dev->header.context_info, 0, sizeof(dev->header.context_info));
		}
		mutex_unlock(&hsml_header_lock);
	} else {
		err = -ENODEV;
	}
	return err;
}

#if HSML_VERSION_12
static int send_hsml_header12(struct mirrorlink_dev *dev)
{
	struct usb_request *req;
	int err;
	static u32 seq;

	dev->header12.seq = htonl(seq++);
	dev->header12.timestamp = htonl((u32)ktime_to_ms(ktime_get()));
	dev->header12.frameBufferDataSize = htonl(dev->hsml_proto->set_display_info.wHeight *
			dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8));
	dev->header12.wWidth = htons(dev->hsml_proto->set_display_info.wWidth);
	dev->header12.wHeight = htons(dev->hsml_proto->set_display_info.wHeight);
	dev->header12.bPixelFormat = dev->hsml_proto->set_display_info.bPixelFormat;
	dev->header12.bEncoding = 0;

	while (!(req = mirrorlink_req_get(dev, &dev->tx_idle))) {
		msleep(1);
		if (!dev->online)
			break;
	}

	if (req) {
		req->length = sizeof(struct hsml_header12);
		mutex_lock(&hsml_header_lock);
		memcpy(req->buf, &dev->header12, req->length);
		err = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (err < 0) {
			mirrorlink_req_put(dev, &dev->tx_idle, req);
			pr_warn("%s: failed to queue req"
				    " %p\n", __func__, req);
		}
		mutex_unlock(&hsml_header_lock);
	} else {
		err = -ENODEV;
	}
	return err;
}
#endif

static void mirrorlink_queue_out(struct mirrorlink_dev *dev)
{
	int ret;
	struct usb_request *req;

	/* if we have idle read requests, get them queued */
	while ((req = mirrorlink_req_get(dev, &dev->rx_idle))) {
		req->length = RXN_MAX;
		pr_debug("%s: queue %p\n", __func__, req);
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			pr_debug("mirrorlink: failed to queue out req (%d)\n", ret);
			dev->error = 1;
			mirrorlink_req_put(dev, &dev->rx_idle, req);
			break;
		}
	}
}

static void touch_event_rotate(struct mirrorlink_dev *dev,
		unsigned short *x, unsigned short *y)
{
	int tmp = ntohs(*x) * ML_TOUCH_WIDTH
		/ dev->hsml_proto->set_display_info.wWidth;
	*x = (dev->hsml_proto->set_display_info.wHeight - ntohs(*y))
		* ML_TOUCH_HEIGHT / dev->hsml_proto->set_display_info.wHeight;
	*y = tmp;
}

static void touch2_event_func(struct mirrorlink_dev *dev,
		struct touch_content *data, int num_touch)
{
	if (num_touch > 0) {
		int i = 0;
		for (i = 0; i < num_touch; i++) {
			touch_event_rotate(dev, &data->x, &data->y);
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


static void mirrorlink_send_multitouch_event(struct mirrorlink_dev *dev,
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
	touch2_event_func(dev, content, event->num_touch);
}

static void mirrorlink_send_fb(struct mirrorlink_dev *dev)
{

	struct usb_request *req;
	int xfer;
	int count = dev->hsml_proto->set_display_info.wHeight *
		dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8);

	char *frame = NULL;
#if MINIFB_READY
	unsigned long frameSize = 0;
#endif
	int last_pkt = 0;

	if (dev->hsml_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
#if MINIFB_READY
		if (minifb_lockbuf((void **)&frame, &frameSize, MINIFB_REPEAT) < 0) {
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
		req = mirrorlink_req_get(dev, &dev->tx_idle);
		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					mirrorlink_req_put(dev, &dev->tx_idle, req);
					pr_warn("%s: failed to queue req %p\n",
						__func__, req);
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
				mirrorlink_req_put(dev, &dev->tx_idle, req);
				pr_warn("%s: failed to queue req %p\n",
					__func__, req);
				break;
			}
		} else {
			pr_err("send_fb: no req to send\n");
			break;
		}
	}
#if MINIFB_READY
	if (!dev->hsml_proto->debug_mode)
		minifb_unlockbuf();
#endif
}

static uint mirrorlink_send_fb2(struct mirrorlink_dev *dev)
{
	struct usb_request *req;
	int xfer;
	char *frame;
	unsigned long frameSize = 0;
	int last_pkt = 0;
	int count = dev->hsml_proto->set_display_info.wHeight *
		dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8);
	uint uXferCnt = 0;

	if (dev->hsml_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
#if HSML_VERSION_12
		if (dev->hsml_ver != cHSML_VER_12) {
#if MINIFB_READY
			if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0)
#endif
				return 0;
		} else {
#if MINIFB_READY
			if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_NOREPEAT) < 0)
#endif
				return 0;
		}
#else //HSML_VERSION_12
#if MINIFB_READY
		if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0)
#endif
			return 0;
#endif //HSML_VERSION_12
	}

	if (frame == NULL)
		return 0;

	if (count > frameSize) {
		pr_warn("[HSML] frameSize mismatch: %d/%lu\n", count, frameSize);
		count = frameSize;
	}

	if (dev->online
	    && (
#if HSML_VERSION_12
	        (_ml_dev->hsml_ver == cHSML_VER_12) ?
	            send_hsml_header12(dev)	:
#endif
	            send_hsml_header07(dev)) < 0) {
		pr_warn("%s: failed to send hsml header\n", __func__);
		goto unlock;
	}

	uXferCnt ++;
	while ((count > 0 || last_pkt == 1) && dev->online) {
		while (!(req = mirrorlink_req_get(dev, &dev->tx_idle))) {
			msleep(1);

			if (!dev->online)
				break;
		}

		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					mirrorlink_req_put(dev, &dev->tx_idle, req);
					pr_warn("%s: failed to queue req %p\n",
						__func__, req);
					break;
				}
				continue;
			}

			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
				mirrorlink_req_put(dev, &dev->tx_idle, req);
				pr_warn("%s: failed to queue req %p\n",
					__func__, req);
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
	if (!dev->hsml_proto->debug_mode) {
#if MINIFB_READY
		minifb_unlockbuf();
#endif
	}
	return uXferCnt;
}

void mirrorlink_send_fb_do_work(struct work_struct *work)
{
	struct mirrorlink_dev *dev = _ml_dev;
	unsigned int uXferCnt;
	while (dev->start_send_fb) {
		uXferCnt = mirrorlink_send_fb2(dev);
		if (uXferCnt && (dev->start_send_fb == cHSML_ON_DEMAND))
			dev->start_send_fb = cHSML_STREAM_OFF;
		msleep(1);
	}
}

static void mirrorlink_enable_fb_work(struct mirrorlink_dev *dev, int enabled)
{
	if ((dev->start_send_fb == enabled) && (enabled == cHSML_ON_DEMAND)) {
		queue_work(dev->wq_display, &dev->send_fb_work);
		return;
	}
	dev->start_send_fb = enabled;
	if (enabled) {
		if (atomic_read(&dev->ml_status) != ML_PROJECTING)
			atomic_set(&dev->ml_status, ML_PROJECTING);

		queue_work(dev->wq_display, &dev->notifier_display_work);
		queue_work(dev->wq_display, &dev->send_fb_work);
	} else {
		if (atomic_read(&dev->ml_status) != ML_OFFLINE)
			atomic_set(&dev->ml_status, ML_OFFLINE);

		queue_work(dev->wq_display, &dev->notifier_display_work);
	}
}

static void mirrorlink_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct mirrorlink_dev *dev = _ml_dev;
	mirrorlink_req_put(dev, &dev->tx_idle, req);
}

static void mirrorlink_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct mirrorlink_dev *dev = _ml_dev;
	unsigned char *data = req->buf;

	if (req->status != 0) {
		pr_err("%s: error, status=%d\n", __func__, req->status);
		dev->error = 1;
		mirrorlink_req_put(dev, &dev->rx_idle, req);
		return ;
	}

	switch (data[0]) {
	case HSML_MSG_TOUCH:
		mirrorlink_send_multitouch_event(dev, data);
		break;

	default:
		pr_err("%s: Unknown message identifier %d\n", __func__, data[0]);
		break;
	}

	mirrorlink_req_put(dev, &dev->rx_idle, req);
	mirrorlink_queue_out(dev);
}


static int mirrorlink_create_bulk_endpoints(struct mirrorlink_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	pr_debug("mirrorlink_create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		pr_debug("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	pr_debug("usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

#if HSML_VERSION_12
	if (out_desc)
#endif
	{
		ep = usb_ep_autoconfig(cdev->gadget, out_desc);
		if (!ep) {
			pr_debug("usb_ep_autoconfig for ep_out failed\n");
			return -ENODEV;
		}
		pr_debug("usb_ep_autoconfig for mirrorlink ep_out got %s\n", ep->name);
		ep->driver_data = dev;		/* claim the endpoint */
		dev->ep_out = ep;
	}
#if HSML_VERSION_12
	else
		dev->ep_out = NULL;
#endif

	/* now allocate requests for our endpoints */
#if HSML_VERSION_12
	if (out_desc)
#endif
	{
		for (i = 0; i < dev->rx_req_count; i++) {
			req = mirrorlink_request_new(dev->ep_out, RXN_MAX);
			if (!req)
				goto fail;
			req->complete = mirrorlink_complete_out;
			mirrorlink_req_put(dev, &dev->rx_idle, req);
		}
	}

	for (i = 0; i < dev->tx_req_count; i++) {
		req = mirrorlink_request_new(dev->ep_in, TXN_MAX);
		if (!req)
			goto fail;
		req->complete = mirrorlink_complete_in;
		mirrorlink_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	while ((req = mirrorlink_req_get(dev, &dev->tx_idle)))
		mirrorlink_request_free(req, dev->ep_in);
#if HSML_VERSION_12
	if (out_desc)
#endif
	{
		while ((req = mirrorlink_req_get(dev, &dev->rx_idle)))
			mirrorlink_request_free(req, dev->ep_out);
	}

	pr_err("mirrorlink: could not allocate requests\n");
	return -1;
}

static int mirrorlink_touch_init(struct mirrorlink_dev *dev)
{
	int x = ML_TOUCH_WIDTH;
	int y = ML_TOUCH_HEIGHT;
	int ret = 0;
	struct input_dev *tdev = dev->touch_input;
	if (touch_init_ml){
		pr_info("mirrorlink_touch already initialized\n");
		return 0;
	}
	dev->touch_input  = input_allocate_device();
	if (dev->touch_input == NULL) {
		pr_err("%s: Failed to allocate input device\n",
			__func__);
		return -1;
	}
	tdev = dev->touch_input;
	tdev->name = "hsml_touchscreen";
	set_bit(EV_SYN,    tdev->evbit);
	set_bit(EV_KEY,    tdev->evbit);
	set_bit(EV_ABS,    tdev->evbit);

	/* Set input parameters boundary. */
	input_set_abs_params(tdev, ABS_MT_POSITION_X, 0, x, 0, 0);
	input_set_abs_params(tdev, ABS_MT_POSITION_Y, 0, y, 0, 0);
	input_set_abs_params(tdev, ABS_MT_PRESSURE, 0, 1, 0, 0);
	ret = input_register_device(tdev);
	if (ret) {
		pr_err("%s: Unable to register %s input device\n",
			__func__, tdev->name);
		input_free_device(tdev);
		return -1;
	}
	touch_init_ml = 1;
	pr_info("mirrorlink_touch_init OK \n");
	return 0;
}


static void mirrorlink_notify_display(struct work_struct *w)
{
	struct mirrorlink_dev *dev = container_of(w,
					struct mirrorlink_dev, notifier_display_work);
	pr_debug("%s\n", __func__);
	switch_set_state(&dev->ml_status_sdev, atomic_read(&dev->ml_status));
}

static void mirrorlink_notify_setting(struct work_struct *w)
{
	struct mirrorlink_dev *dev = container_of(w,
					struct mirrorlink_dev, notifier_setting_work);
	pr_debug("%s\n", __func__);
	switch_set_state(&ml_switch, atomic_read(&dev->ml_enable_HSML));
}

static void mirrorlink_function_disable(struct usb_function *f)
{
	struct mirrorlink_dev *dev = func_to_mirrorlink_dev(f);

	pr_debug("%s\n", __func__);

//	dev->start_send_fb = false;
	dev->start_send_fb = cHSML_STREAM_OFF;
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);

#if HSML_VERSION_12
	if (dev->ep_out)
#endif
		usb_ep_disable(dev->ep_out);

	if (atomic_read(&dev->ml_status) != ML_OFFLINE) {
		atomic_set(&dev->ml_status, ML_OFFLINE);
		schedule_work(&dev->notifier_display_work);
	}

	if (atomic_read(&dev->ml_enable_HSML) != 0) {
		atomic_set(&dev->ml_enable_HSML, 0);
		pr_info("[MIRROR_LINK]%s, set state: 0\n",__func__);
		schedule_work(&dev->notifier_setting_work);
	}

	pr_debug("%s disabled\n", dev->function.name);
}

static void mirrorlink_function_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct mirrorlink_dev *dev = func_to_mirrorlink_dev(f);
	struct usb_request *req;

	pr_debug("%s\n", __func__);

	destroy_workqueue(dev->wq_display);

	while ((req = mirrorlink_req_get(dev, &dev->tx_idle)))
		mirrorlink_request_free(req, dev->ep_in);

#if HSML_VERSION_12
	if (dev->ep_out)
#endif
	{
		while ((req = mirrorlink_req_get(dev, &dev->rx_idle)))
			mirrorlink_request_free(req, dev->ep_out);
	}

	dev->online = 0;
	dev->error = 1;
	dev->is_htcmode = 0;
#if HSML_VERSION_12
	dev->in_desc = NULL;
	dev->out_desc = NULL;
#endif

	/*
	 * Due to some timing issue, mirrorlink may be unbinded and re-bindeded.
	 * Hence setting may receive uevnet to disable Mirror Link.
	 * Only send uevnet in mirrorlink_function_disable.
	 */
	/*
	if (atomic_read(&dev->prj2_status) != PRJ2_OFFLINE) {
		atomic_set(&dev->prj2_status, PRJ2_OFFLINE);
		schedule_work(&dev->notifier_display_work);
	}

	if (atomic_read(&dev->prj2_enable_HSML) != 0) {
		atomic_set(&dev->prj2_enable_HSML, 0);
		schedule_work(&dev->notifier_setting_work);
	}
	*/
}

static int mirrorlink_function_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct mirrorlink_dev *dev = func_to_mirrorlink_dev(f);
	int ret = -ENODEV;

	pr_debug("%s\n", __func__);

	dev->cdev = cdev;
	dev->bitsPixel = 0;
	dev->width = DEFAULT_ML_WIDTH;
	dev->height = DEFAULT_ML_HEIGHT;
	dev->rx_req_count = ML_RX_REQ_MAX;
	dev->tx_req_count = (dev->width * dev->height * 2 / TXN_MAX) + 3;
	pr_debug("[USB] resolution: %u*%u"
		", rx_cnt: %u, tx_cnt:%u\n", dev->width, dev->height,
		dev->rx_req_count, dev->tx_req_count);

	if (mirrorlink_touch_init(dev) < 0)
		goto err_free;

	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->tx_idle);

	dev->wq_display = create_singlethread_workqueue("mirrorlink_mode");
	if (!dev->wq_display)
		goto err_free;

	workqueue_set_max_active(dev->wq_display,1);

	INIT_WORK(&dev->send_fb_work, mirrorlink_send_fb_do_work);

	/*atomic_set(&_ml_dev->ml_enable_HSML, 1);*/

	/* allocate string ID(s) */
	if (mirrorlink_string_defs[0].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			goto err_free;
		mirrorlink_string_defs[0].id = ret;
		mirrorlink_interface_desc.iInterface = ret;
	}

	/* allocate interface ID(s) */
	ret = usb_interface_id(c, f);
	if (ret < 0)
		goto err_free;
	mirrorlink_interface_desc.bInterfaceNumber = ret;

	/* allocate endpoints */
#if HSML_VERSION_12
	if (dev->hsml_ver != cHSML_VER_12)
#endif
		ret = mirrorlink_create_bulk_endpoints(dev, &mirrorlink_fullspeed_in_desc,
				&mirrorlink_fullspeed_out_desc);
#if HSML_VERSION_12
	else
		ret = mirrorlink_create_bulk_endpoints(dev, &mirrorlink_fullspeed_in_desc,
				NULL);
#endif
	if (ret)
		goto err_free;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		mirrorlink_highspeed_in_desc.bEndpointAddress =
			mirrorlink_fullspeed_in_desc.bEndpointAddress;
#if HSML_VERSION_12
		if (dev->hsml_ver != cHSML_VER_12)
#endif
			mirrorlink_highspeed_out_desc.bEndpointAddress =
				mirrorlink_fullspeed_out_desc.bEndpointAddress;
	}

	pr_debug("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name,
			dev->ep_out ? dev->ep_out->name : "NULL"
			);
	return 0;

err_free:
	pr_err("mirrorlink gadget driver failed to initialize, err=%d\n", ret);
	return ret;
}


static int mirrorlink_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct mirrorlink_dev *dev = func_to_mirrorlink_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	pr_debug("%s intf: %d alt: %d\n", __func__, intf, alt);

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

#if HSML_VERSION_12
	if (dev->ep_out)
#endif
	{
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
	}

	dev->online = 1;

#if HSML_VERSION_12
	if (dev->ep_out)
#endif
		mirrorlink_queue_out(dev);

	return 0;
}

static void mirrorlink_complete_req(struct usb_ep *ep, struct usb_request *req)
{
	struct mirrorlink_dev *dev = ep->driver_data;
	int length = req->actual;
	char *dst = NULL;

	pr_debug("status=%d, request=0x%02X\n",
			req->status, dev->hsml_proto->request);

	if (req->status != 0) {
		pr_warn("mirrorlink_complete_req, err %d\n", req->status);
		return;
	}

#if HSML_VERSION_12
	if (dev->hsml_ver == cHSML_VER_12) {
		switch (dev->hsml_proto->request) {
			case HSML_12_REQ_SET_PARAMETERS:
				dst = (char *) &dev->hsml_proto->set_parameters_info;
				memcpy(dst, req->buf, length);
				dev->hsml_proto->set_display_info.bPixelFormat = dev->hsml_proto->set_parameters_info.pixelFormat;

				switch (dev->hsml_proto->set_display_info.bPixelFormat) {
					case HSML_12_PIXEL_FORMAT_ARGB888:
						dev->bitsPixel = 32;
						break;
					default:
						dev->bitsPixel = 16;
						break;
				}
				dev->framesize = dev->hsml_proto->set_display_info.wHeight *
								dev->hsml_proto->set_display_info.wWidth *
								(dev->bitsPixel / 8);
				break;

			default:
				break;
		}
	}
	else
#endif
	{
		switch (dev->hsml_proto->request) {
			case HSML_08_REQ_SET_SERVER_CONFIGURATION:
				dst = (char *)&dev->hsml_proto->set_server_configuation_info;
				memcpy(dst, req->buf, length);
				break;

			case HSML_08_REQ_SET_SERVER_DISPLAY:
				dst = (char *)&dev->hsml_proto->set_display_info;
				memcpy(dst, req->buf, length);
				switch (dev->hsml_proto->set_display_info.bPixelFormat) {
				case PIXEL_FORMAT_ARGB888:
					dev->bitsPixel = 32;
					break;
				case PIXEL_FORMAT_RGB565:
				case PIXEL_FORMAT_RGB555:
					dev->bitsPixel = 16;
					break;
				default:
					dev->bitsPixel = 16;
					break;
				}
				dev->framesize = dev->hsml_proto->set_display_info.wHeight *
					dev->hsml_proto->set_display_info.wWidth *
					(dev->bitsPixel / 8);
				break;

			default:
				break;
		}
	}
}

#if HSML_VERSION_12
void vAdjustDesc(u16 hsml_ver)
{
	pr_debug("[HSML] %s\n", __func__);
	if (hsml_ver != cHSML_VER_12) {
		mirrorlink_interface_desc.bNumEndpoints = 2;
		mirrorlink_interface_desc.bInterfaceSubClass = 0xFF,
		mirrorlink_interface_desc.bInterfaceProtocol = 0xFF,

		fs_mirrorlink_descs[2] =
			(struct usb_descriptor_header *) &mirrorlink_fullspeed_out_desc;
		hs_mirrorlink_descs[2] =
			(struct usb_descriptor_header *) &mirrorlink_highspeed_out_desc;

		mirrorlink_string_defs[0].s = "HSML Server";
	}
	else {
		mirrorlink_interface_desc.bNumEndpoints = 1;
		mirrorlink_interface_desc.bInterfaceSubClass = 0xCC,
		mirrorlink_interface_desc.bInterfaceProtocol = 0x01,

		fs_mirrorlink_descs[2] = NULL;
		hs_mirrorlink_descs[2] = NULL;

		mirrorlink_string_defs[0].s = "HSML Source";
	}
}
#endif

int mirrorlink_ctrlrequest(struct usb_composite_dev *cdev,
											const struct usb_ctrlrequest *ctrl)
{
	int value = -EOPNOTSUPP;
	u16 w_length = le16_to_cpu(ctrl->wLength);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		pr_debug("request(req=0x%02x, wValue=%d, "
						"wIndex=%d, wLength=%d)\n",
						ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);

		switch (ctrl->bRequest) {
			case HSML_08_REQ_MIRROR_LINK:
				if (_ml_dev) {
#if HSML_VERSION_12
					if (le16_to_cpu(ctrl->wValue) >= 0x0201)
						_ml_dev->hsml_ver = cHSML_VER_12;
					else
						_ml_dev->hsml_ver = cHSML_VER_08;
					vAdjustDesc(_ml_dev->hsml_ver);

					pr_info("set state: 1 (ver=%04X,%d)\n",
						le16_to_cpu(ctrl->wValue), _ml_dev->hsml_ver);
#else
					pr_info("[MIRROR_LINK]%s, set state: 1\n", __func__);
#endif
					atomic_set(&_ml_dev->ml_enable_HSML, 1);
					schedule_work(&_ml_dev->notifier_setting_work);
				}
				value = w_length;
				break;
			default:
				pr_debug("unrecognized request(req=0x%02x, wValue=%d, "
						"wIndex=%d, wLength=%d)\n",
						ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
				break;
		}
	}
	if (value >= 0) {
		cdev->req->zero = 0;
		cdev->req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (value < 0)
			pr_err("%s setup response queue error\n",__func__);
	}
	return value;
}

static int mirrorlink_function_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct mirrorlink_dev *dev = func_to_mirrorlink_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int value = -EOPNOTSUPP;
	u16	w_length = le16_to_cpu(ctrl->wLength);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u32 ret = 0;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		char *ptr = (char *)cdev->req->buf;
		int i = 0;

		pr_debug("request(req=0x%02x, wValue=%d, "
						 "wIndex=%d, wLength=%d)\n",
						 ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);

#if HSML_VERSION_12
		if (_ml_dev->hsml_ver == cHSML_VER_12) {
			switch (ctrl->bRequest) {
				case HSML_12_REQ_GET_VERSION:
					ptr[0] = (char)(HSML_12_PROTOCOL_VERSION >> 8);
					ptr[1] = (char)(HSML_12_PROTOCOL_VERSION & 0xFF);
					value = sizeof(u16);
					break;

				case HSML_12_REQ_GET_PARAMETERS:
					value = sizeof(struct get_parameters);
					memset(cdev->req->buf, 0, value);
					dev->hsml_proto->get_parameters_info.height =
						dev->hsml_proto->set_display_info.wHeight;
					dev->hsml_proto->get_parameters_info.width =
						dev->hsml_proto->set_display_info.wWidth;

					ptr = (char *) &_ml_dev->hsml_proto->get_parameters_info;
					memcpy(cdev->req->buf, ptr, value);
					break;

				case HSML_12_REQ_SET_PARAMETERS:
					cdev->gadget->ep0->driver_data = dev;
					cdev->req->complete = mirrorlink_complete_req;
					dev->hsml_proto->request = ctrl->bRequest;
					value = w_length;
					break;

				case HSML_12_REQ_START_FB_TRANS:
					if (!w_value)
						mirrorlink_enable_fb_work(_ml_dev, cHSML_STREAM_ON);
					else
						//mirrorlink_send_fb(ml_dev);
						mirrorlink_enable_fb_work(_ml_dev, cHSML_ON_DEMAND);
					value = 0;
					break;

				case HSML_12_REQ_PAUSE_FB_TRANS:
				case HSML_12_REQ_STOP_FB_TRANS:
					mirrorlink_enable_fb_work(_ml_dev, cHSML_STREAM_OFF);
					value = 0;
					break;

				case HSML_12_REQ_SET_MAX_FRAME_RATE:
					_ml_dev->hsml_proto->MaxFPS = w_value;
					queue_work(dev->wq_display, &dev->notifier_display_work);
					value = 0;
					break;

				case HSML_12_REQ_GET_ID:
					memset(cdev->req->buf, 0, w_length);
					ptr = (char *) dev->aucUUID;
					value = w_length;
					if (value > sizeof(dev->aucUUID))
						value = sizeof(dev->aucUUID);
					memcpy(cdev->req->buf, ptr, value);
					break;

				default:
					pr_debug("unrecognized request (HSML_12) (req=0x%02x, wValue=%d, "
							"wIndex=%d, wLength=%d)\n",
							ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
					break;
			}
		}
		else
#endif
		{
			switch (ctrl->bRequest) {
				case HSML_08_REQ_GET_SERVER_VERSION:
					ptr[0] = (char)(HSML_07_PROTOCOL_VERSION >> 8);
					ptr[1] = (char)(HSML_07_PROTOCOL_VERSION & 0xFF);
					value = sizeof(u16);
					break;

				case HSML_08_REQ_NUM_COMPRESSION_SETTINGS:
					memset(cdev->req->buf,0, w_length);
					value = sizeof(u16);
					break;

				case HSML_08_REQ_GET_SERVER_CONFIGURATION:
					value = sizeof(struct get_server_configuation);
					memset(cdev->req->buf, 0, value);
					ptr = (char *)&_ml_dev->hsml_proto->get_server_configuation_info;
					memcpy(cdev->req->buf, ptr, value);
					break;

				case HSML_08_REQ_GET_FB:
					if (!w_value)
						mirrorlink_enable_fb_work(_ml_dev, 1);
					else
						mirrorlink_send_fb(_ml_dev);
					value = 0;
					break;

				case HSML_08_REQ_STOP:
					mirrorlink_enable_fb_work(_ml_dev, 0);
					value = 0;
					break;

				case HSML_08_REQ_GET_SERVER_DISPLAY: {
					int maxSize = 0;
					struct fb_info *info;

					info = registered_fb[0];
					if (!info) {
						pr_warn("%s: Can not access framebuffer\n", __func__);
					} else {
						pr_info("device(%d, %d)\n", info->var.xres, info->var.yres);
						maxSize = info->var.xres * info->var.yres;
					}

					/* implement me later */
					for (i = 0; i <= 26; i++) {
						if ((display_setting[i][0] * display_setting[i][1]) <= maxSize)
						ret |= (1 << i);
					}

					_ml_dev->hsml_proto->get_display_capabilities_info.dwResolutionSupported = ret;
					_ml_dev->hsml_proto->get_display_capabilities_info.dwPixelFormatSupported =
						(1 << PIXEL_FORMAT_RGB565) | (1 << PIXEL_FORMAT_ARGB888) |
						(1 << PIXEL_FORMAT_RGB555);
					ptr = (char *)&_ml_dev->hsml_proto->get_display_capabilities_info;
					memcpy(cdev->req->buf, ptr, w_length);

					value = w_length;
					break;
				}
				case HSML_08_REQ_SET_SERVER_CONFIGURATION:
				case HSML_08_REQ_SET_SERVER_DISPLAY:
					cdev->gadget->ep0->driver_data = dev;
					cdev->req->complete = mirrorlink_complete_req;
					dev->hsml_proto->request = ctrl->bRequest;
					value = w_length;
					break;

				case HSML_08_REQ_SET_MAX_FRAME_RATE:
					_ml_dev->hsml_proto->MaxFPS = w_value;
					queue_work(dev->wq_display, &dev->notifier_display_work);
					value = 0;
					break;

				default:
					pr_debug("unrecognized request (HSML_08) (req=0x%02x, wValue=%d, "
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
			pr_err("%s setup response queue error\n", __func__);
	}

	return value;
}

static ssize_t print_ml_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ml_shortname);
}

static ssize_t print_ml_switch_state(struct switch_dev *ml_status_sdev,
		char *buf)
{
	struct mirrorlink_dev *dev = container_of(ml_status_sdev,
					struct mirrorlink_dev, ml_status_sdev);
	return sprintf(buf, "%s\n", (atomic_read(&dev->ml_status) == ML_PROJECTING?
		"projecting" : (atomic_read(&dev->ml_status) == ML_ONLINE ? "online" : "offline")));
}

static int mirrorlink_setup(struct hsml_protocol *config)
{
	struct mirrorlink_dev *dev;
	int ret = 0;
	const char sig[] = {
		0xFF, 0xFF, 0x48, 0x53,
		0x4D, 0x4C, 0xFF, 0xFF
	};

#if HSML_VERSION_12
	u8 ucTmpUUID[] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	};
#endif

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	_ml_dev = dev;
	dev->hsml_proto = config;

#if HSML_VERSION_12
	dev->hsml_ver = cHSML_VER_08;
	dev->in_desc = NULL;
	dev->out_desc = NULL;

	dev->hsml_proto->get_parameters_info.capabilities = 0;
	dev->hsml_proto->get_parameters_info.pixelFormatSupported =
		(1 << HSML_12_PIXEL_FORMAT_RGB565) | (1 << HSML_12_PIXEL_FORMAT_ARGB888) |
		(1 << HSML_12_PIXEL_FORMAT_RGB555);
	dev->hsml_proto->get_parameters_info.height = DEFAULT_ML_WIDTH;
	dev->hsml_proto->get_parameters_info.width = DEFAULT_ML_HEIGHT;
	dev->hsml_proto->get_parameters_info.encodingSupported = 1;

	memset(dev->aucUUID, 0, sizeof(dev->aucUUID));
	memcpy(dev->aucUUID, ucTmpUUID, cHSML_UUID_SIZE);
#endif

	dev->hsml_proto->get_server_configuation_info.dwCapabilities =
		(1 << HSML_SERVER_CAP_TOUCH);
	dev->hsml_proto->get_server_configuation_info.dwTouchConfiguration =
		(1 << 8) | (3 << 0);

	dev->hsml_proto->set_display_info.wHeight = DEFAULT_ML_HEIGHT;
	dev->hsml_proto->set_display_info.wWidth = DEFAULT_ML_WIDTH;

	memcpy(&dev->header.signature, sig, sizeof(dev->header.signature));
	dev->header.seq = 0;
	dev->header.timestamp = 0;

#if HSML_VERSION_12
	memcpy(&dev->header12.signature, sig, sizeof(dev->header12.signature));
	dev->header12.seq = 0;
	dev->header12.timestamp = 0;
	memset(dev->header12.aucReserved, 0, sizeof(dev->header12.aucReserved));
#endif

	INIT_WORK(&dev->notifier_display_work, mirrorlink_notify_display);
	INIT_WORK(&dev->notifier_setting_work, mirrorlink_notify_setting);

	ret = switch_dev_register(&ml_switch);
	if (ret < 0) {
		pr_err("[MIRROR_LINK]fail to register mirror_link switch!\n");
		goto err_free;
	}

	dev->ml_status_sdev.name = ml_shortname;
	dev->ml_status_sdev.print_name = print_ml_switch_name;
	dev->ml_status_sdev.print_state = print_ml_switch_state;
	ret = switch_dev_register(&dev->ml_status_sdev);
	if (ret < 0) {
		pr_err("usb ml_status_sdev switch_dev_register register fail\n");
		goto err_free;
	}

	return 0;

err_free:
	kfree(dev);
	pr_err("mirrorlink gadget driver failed to initialize, err=%d\n", ret);
	return ret;
}

static void __maybe_unused mirrorlink_cleanup(void)
{
	struct mirrorlink_dev *dev = _ml_dev;

	if (dev->touch_input) {
		input_unregister_device(dev->touch_input);
		input_free_device(dev->touch_input);
	}
	touch_init_ml = 0;
	kfree(_ml_dev);
}

static void mirrorlink_free_func(struct usb_function *f)
{
	/* no operation here */
}

static ssize_t mirrorlink_client_width_show(struct config_item *item,
		char *page)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->set_display_info.wWidth);
}

#if HSML_VERSION_12
#define cHSML_WIDTH_SIZE        2

static ssize_t mirrorlink_client_width_store(struct config_item *item,
		const char *page, size_t len)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;
	u16 uValue;
	u8 aucWidth[cHSML_WIDTH_SIZE];

    if (len <= cHSML_WIDTH_SIZE) {
        memset(aucWidth, 0, sizeof(aucWidth));
        memcpy(aucWidth, page, len);
        uValue = be16_to_cpu(*((__le16 *) aucWidth));
        config->set_display_info.wWidth = uValue;
        return len;
    } else {
        pr_err("%s: size is invalid %zu/%d\n", __func__, len, cHSML_WIDTH_SIZE);
    }
    return -EINVAL;
}
CONFIGFS_ATTR(mirrorlink_, client_width);
#else
CONFIGFS_ATTR_RO(mirrorlink_, client_width);
#endif

static ssize_t mirrorlink_client_height_show(struct config_item *item,
		char *page)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->set_display_info.wHeight);
}

#if HSML_VERSION_12
static ssize_t mirrorlink_client_height_store(struct config_item *item,
		const char *page, size_t len)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;
	u16 uValue;
	u8 aucWidth[cHSML_WIDTH_SIZE];

    if (len <= cHSML_WIDTH_SIZE) {
        memset(aucWidth, 0, sizeof(aucWidth));
        memcpy(aucWidth, page, len);
        uValue = be16_to_cpu(*((__le16 *) aucWidth));
        config->set_display_info.wHeight = uValue;
        return len;
    } else {
        printk(KERN_ERR "%s: size is invalid %zu/%d\n", __func__, len, cHSML_WIDTH_SIZE);
    }

    return -EINVAL;
}
CONFIGFS_ATTR(mirrorlink_, client_height);
#else
CONFIGFS_ATTR_RO(mirrorlink_, client_height);
#endif

static ssize_t mirrorlink_client_maxfps_show(struct config_item *item,
		char *page)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->MaxFPS);
}
CONFIGFS_ATTR_RO(mirrorlink_, client_maxfps);

static ssize_t mirrorlink_client_pixel_format_show(struct config_item *item,
		char *page)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;
	return snprintf(page, PAGE_SIZE, "%d\n", config->set_display_info.bPixelFormat);
}
CONFIGFS_ATTR_RO(mirrorlink_, client_pixel_format);

static ssize_t mirrorlink_client_context_info_store(struct config_item *item,
		const char *page, size_t len)
{
	struct mirrorlink_dev *mirrorlink_dev = _ml_dev;
	if (len % CONTEXT_INFO_SIZE) {
		pr_err("%s: Array size invalid, array size should be N*28, size=%zu\n",
			__func__, len);
		return -EINVAL;
	} else {
			if ((len / CONTEXT_INFO_SIZE) <= MAX_NUM_CONTEXT_INFO) {
			mutex_lock(&hsml_header_lock);
			memset(mirrorlink_dev->header.context_info, 0,
				sizeof(mirrorlink_dev->header.context_info));
			memcpy(mirrorlink_dev->header.context_info, page, len);
			mirrorlink_dev->header.num_context_info = htons(len / CONTEXT_INFO_SIZE);
			mutex_unlock(&hsml_header_lock);
		} else {
			pr_err("%s: N is invalid value, N=%zu\n",__func__,
					len / CONTEXT_INFO_SIZE);
			return -EINVAL;
		}
	}
	return len;
}
CONFIGFS_ATTR_WO(mirrorlink_, client_context_info);

#if HSML_VERSION_12
static ssize_t mirrorlink_client_ver_show(struct config_item *item, char *page)
{
	struct mirrorlink_dev *mirrorlink_dev = _ml_dev;

	return snprintf(page, PAGE_SIZE, "%d\n", mirrorlink_dev->hsml_ver);
}
CONFIGFS_ATTR_RO(mirrorlink_, client_ver);

static ssize_t mirrorlink_client_cap_show(struct config_item *item, char *page)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);
	struct hsml_protocol *config = &opts->config;

	return snprintf(page, PAGE_SIZE, "%d\n",
		config->set_parameters_info.capabilities & cHSML_12_CAP_ENDIAN);
}
CONFIGFS_ATTR_RO(mirrorlink_, client_cap);

static ssize_t mirrorlink_client_uuid_store(struct config_item *item,
		const char *page, size_t len)
{
	struct mirrorlink_dev *mirrorlink_dev = _ml_dev;

	if (len <= cHSML_UUID_SIZE) {
		memset(mirrorlink_dev->aucUUID, 0, sizeof(mirrorlink_dev->aucUUID));
		memcpy(mirrorlink_dev->aucUUID, page, len);
	} else {
		pr_err("%s: size is invalid %zu/%d\n", __func__, len, cHSML_UUID_SIZE);
		return -EINVAL;
	}
	return len;
}
CONFIGFS_ATTR_WO(mirrorlink_, client_uuid);
#endif

static int __maybe_unused mirrorlink_ctrlreq_configfs(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	if (f->config != NULL && f->config->cdev != NULL)
		return mirrorlink_ctrlrequest(f->config->cdev, ctrl);
	else
		return -1;
}

static int mirrorlink_set_inst_name(struct usb_function_instance *fi,
		const char *name)
{
	struct f_mirrorlink_opts *opts = fi_to_f_mirrorlink_opts(fi);
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

static void mirrorlink_free_instance(struct usb_function_instance *fi)
{
	struct f_mirrorlink_opts *opts = fi_to_f_mirrorlink_opts(fi);

	kfree(opts);
}

static void mirrorlink_opts_release(struct config_item *item)
{
	struct f_mirrorlink_opts *opts = ci_to_f_mirrorlink_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations mirrorlink_item_ops = {
	.release	= mirrorlink_opts_release,
};

static struct configfs_attribute *mirrorlink_attrs[] = {
	&mirrorlink_attr_client_width,
	&mirrorlink_attr_client_height,
	&mirrorlink_attr_client_maxfps,
	&mirrorlink_attr_client_pixel_format,
	&mirrorlink_attr_client_context_info,
#if HSML_VERSION_12
	&mirrorlink_attr_client_ver,
	&mirrorlink_attr_client_cap,
	&mirrorlink_attr_client_uuid,
#endif
	NULL,
};

static struct config_item_type mirrorlink_func_type = {
	.ct_item_ops	= &mirrorlink_item_ops,
	.ct_attrs	= mirrorlink_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct usb_function_instance *mirrorlink_alloc_inst(void)
{
	struct f_mirrorlink_opts *opts;
	int ret;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.set_inst_name = mirrorlink_set_inst_name;
	opts->func_inst.free_func_inst = mirrorlink_free_instance;

	ret = mirrorlink_setup(&opts->config);
	if (ret) {
		kfree(opts);
		pr_err("Error setting mirrorlink\n");
		return ERR_PTR(ret);
	}

	config_group_init_type_name(&opts->func_inst.group, "",
			&mirrorlink_func_type);
	return &opts->func_inst;
}

static struct usb_function *mirrorlink_alloc_func(struct usb_function_instance *fi)
{
	struct mirrorlink_dev *dev = _ml_dev;

	dev->function.name = "Mirrorlink Projector Function";
	dev->function.strings = mirrorlink_strings;

	dev->function.fs_descriptors = fs_mirrorlink_descs;
	dev->function.hs_descriptors = hs_mirrorlink_descs;
	//dev->function.ss_descriptors = ss_mirrorlink_descs;

	dev->function.bind = mirrorlink_function_bind;
	dev->function.unbind = mirrorlink_function_unbind;
	dev->function.free_func = mirrorlink_free_func;
	dev->function.set_alt = mirrorlink_function_set_alt;
	//dev->function.get_alt =
	dev->function.disable = mirrorlink_function_disable;
	dev->function.setup = mirrorlink_function_setup;
	//dev->function.req_match =
	//dev->function.suspend =
	//dev->function.resume =
	//dev->function.get_status =
	//dev->function.func_suspend =

	return &dev->function;
}

DECLARE_USB_FUNCTION_INIT(mirrorlink, mirrorlink_alloc_inst, mirrorlink_alloc_func);
MODULE_DESCRIPTION("USB Projector (Mirrorlink) Driver");
MODULE_AUTHOR("Kyle Tso");
MODULE_LICENSE("GPL v2");
