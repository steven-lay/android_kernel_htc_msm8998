/*
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 8628 $
 * $Date: 2017-01-09 15:12:35 +0800 (週一, 09 一月 2017) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#ifdef HTC_FEATURE
#include <linux/pinctrl/consumer.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_PROC
#ifdef HTC_FEATURE
extern int32_t nvt_extra_proc_init(struct nvt_ts_data* ts);
#else
extern int32_t nvt_extra_proc_init(void);
#endif
#endif
#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

#if !defined(HTC_FEATURE)
static struct workqueue_struct *nvt_wq;
#endif

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

static const struct nvt_ts_mem_map NT36772_memory_map = {
	.EVENT_BUF_ADDR      = 0x11E00,
	.RAW_PIPE0_ADDR      = 0x10000,
	.RAW_PIPE1_ADDR      = 0x12000,
	.BASELINE_ADDR       = 0x10E70,
	.BASELINE_BTN_ADDR   = 0x12E70,
	.DIFF_PIPE0_ADDR     = 0x10830,
	.DIFF_PIPE1_ADDR     = 0x12830,
	.RAW_BTN_PIPE0_ADDR  = 0x10E60,
	.RAW_BTN_PIPE1_ADDR  = 0x12E60,
	.DIFF_BTN_PIPE0_ADDR = 0x10E68,
	.DIFF_BTN_PIPE1_ADDR = 0x12E68,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR = 0x14002,
};

static const struct nvt_ts_mem_map NT36525_memory_map = {
	.EVENT_BUF_ADDR      = 0x11A00,
	.RAW_PIPE0_ADDR      = 0x10000,
	.RAW_PIPE1_ADDR      = 0x12000,
	.BASELINE_ADDR       = 0x10B08,
	.BASELINE_BTN_ADDR   = 0x12B08,
	.DIFF_PIPE0_ADDR     = 0x10640,
	.DIFF_PIPE1_ADDR     = 0x12640,
	.RAW_BTN_PIPE0_ADDR  = 0x10AF0,
	.RAW_BTN_PIPE1_ADDR  = 0x12AF0,
	.DIFF_BTN_PIPE0_ADDR = 0x10AFC,
	.DIFF_BTN_PIPE1_ADDR = 0x12AFC,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR = 0x14002,
};

#define NVT_ID_BYTE_MAX 6
struct nvt_ts_trim_id_table  {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
	{.id = {0x55, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0x55, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1}, .mmap = &NT36772_memory_map},
	{.id = {0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03}, .mask = {0, 0, 0, 1, 1, 1}, .mmap = &NT36525_memory_map}
};

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

static uint8_t bTouchSkipResetInt = 1;
#ifdef HTC_FEATURE
uint32_t debug_mask = 0;
#endif

/*******************************************************
Description:
	Novatek touchscreen set skip reset interrupt function.

return:
	n.a.
*******************************************************/
void nvt_ts_set_skip_reset_int(uint8_t skip)
{
	bTouchSkipResetInt = skip;
}

/*******************************************************
Description:
	Novatek touchscreen get skip reset interrupt function.

return:
	Executive outcomes. 0---not skip. 1---skip reset int.
*******************************************************/
uint8_t nvt_ts_get_skip_reset_int(void)
{
	return bTouchSkipResetInt;
}

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c dummy read function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	uint8_t buf[8] = {0};
	int32_t ret = -1;

	ret = CTP_I2C_READ(client, address, buf, 2);
	if (ret < 0)
		NVT_ERR("CTP_I2C_READ_DUMMY failed.(%d)\n", ret);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}


/*******************************************************
Description:
	Novatek touchscreen IC hardware reset function.

return:
	n.a.
*******************************************************/
void nvt_hw_reset(void)
{
	//---trigger rst-pin to reset (pull low for 50ms)---
	gpio_set_value(ts->reset_gpio, 1);
	msleep(20);
	gpio_set_value(ts->reset_gpio, 0);
	msleep(50);
	gpio_set_value(ts->reset_gpio, 1);
	msleep(20);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] >= check_reset_state) && (buf[1] < 0xFF)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 200)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X\n", retry, buf[1]);
			ret = -1;
			break;
		}
	}

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str))
		return -EFAULT;

	if (copy_from_user(str, buff, count))
		return -EFAULT;

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) {	//I2C write
		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 1)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) {	//I2C read
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/NVTflash\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

#if WAKEUP_GESTURE
static struct wake_lock gestrue_wakelock;
static uint8_t bTouchIsAwake = 1;
#endif
/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id)
{
	uint32_t keycode = 0;

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_LOG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			NVT_LOG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_LOG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}

	msleep(250);
}
#endif

#ifdef HTC_FEATURE
void nvt_set_irq_enable(bool enable)
{
	NVT_LOG("%s: %s\n", __func__, enable ? "enable" : "disable");

	if (ts->irq_enabled == enable) {
		NVT_ERR("irq already %s\n", ts->irq_enabled ? "enabled" : "disabled");
		return;
	}

	if (enable)
		enable_irq(ts->client->irq);
	else
		disable_irq(ts->client->irq);

	ts->irq_enabled = enable;
}

void nvt_set_tp_mode(uint8_t tp_mode, bool enable)
{
	uint8_t buf[2] = {0};

	if (atomic_read(&ts->firmware_updating)) {
		NVT_LOG("%s: firmware updating, skip mode setting (b%d: %s)\n", __func__, tp_mode, enable ? "enable" : "disable");
		return;
	}

	switch (tp_mode) {
	case GLOVE_MODE_BIT:
		NVT_LOG("%s: set glove mode: %d\n", __func__, enable);
		buf[0] = EVENT_MAP_HOST_CMD;
		if (enable)
			buf[1] = 0x71;
		else
			buf[1] = 0x72;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		break;
	case COVER_MODE_BIT:
		//TBD
		break;
	default:
		NVT_LOG("%s: incorrect mode\n", __func__);
		return;
		break;
	}

	ts->tp_mode_state &= ~tp_mode;
	ts->tp_mode_state |= enable * tp_mode;

	return;
}

void nvt_set_hopping_debug_mode(bool enable)
{
	uint8_t buf[2] = {0};
	NVT_LOG("Set hopping debug log %s\n", enable ? "on" : "off");
	buf[0] = EVENT_MAP_HOST_CMD;
	if (enable)
		buf[1] = 0x79;
	else
		buf[1] = 0x7A;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
}

static int novatek_pinctrl_configure(struct nvt_ts_data *ts, bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state = pinctrl_lookup_state(ts->tp_pinctrl, "pmx_ts_active");
		if (IS_ERR(set_state)) {
			NVT_ERR("cannot get ts pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(ts->tp_pinctrl, "pmx_ts_suspend");
		if (IS_ERR(set_state)) {
			NVT_ERR("cannot get ts pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(ts->tp_pinctrl, set_state);
	if (retval) {
		NVT_ERR("cannot set ts pinctrl state\n");
		return retval;
	}

	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static void novatek_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 value;

	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	if(ts->irq_gpio < 0)
		ts->irq_gpio = -1;
#ifdef HTC_FEATURE
	if (of_property_read_u32(np, "novatek,support-glove", &value)) {
		NVT_LOG("%s: fail to request glove mode setting\n", __func__);
		ts->support_glove = 0;
	} else {
		ts->support_glove = value;
		NVT_LOG("%s: set support glove mode %d\n", __func__, value);
	}

	ts->switch_gpio = of_get_named_gpio_flags(np, "novatek,switch-gpio", 0, &ts->switch_flags);
	if(ts->switch_gpio < 0)
		ts->switch_gpio = -1;
	NVT_LOG("switch-gpio=%d\n", ts->switch_gpio);

	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	if(ts->reset_gpio < 0)
		ts->reset_gpio = -1;

	ts->power_gpio = of_get_named_gpio_flags(np, "novatek,power-gpio", 0, &ts->power_flags);
	if(ts->power_gpio < 0)
		ts->power_gpio = -1;
	NVT_LOG("irq-gpio=%d, reset-gpio=%d, power-gpio=%d\n",
			ts->irq_gpio, ts->reset_gpio, ts->power_gpio);
#else
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	ts->disp_rst_gpio = of_get_named_gpio_flags(np, "novatek,disp-rst-gpio", 0, &ts->disp_rst_flags);
	NVT_LOG("novatek,reset-gpio=%d, novatek,irq-gpio=%d\n", ts->reset_gpio, ts->irq_gpio);
	NVT_LOG("novatek,disp-rst-gpio=%d\n", ts->disp_rst_gpio);
#endif
}
#else
static void novatek_parse_dt(struct device *dev)
{
	ts->reset_gpio = NVTTOUCH_RST_PIN;
	ts->irq_gpio = NVTTOUCH_INT_PIN;
	ts->disp_rst_gpio = NVTTOUCH_DISP_RST_PIN;
}
#endif

#if POINT_DATA_CHECKSUM
/*******************************************************
Description:
	Novatek touchscreen check i2c packet checksum function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_ts_point_data_checksum(struct i2c_client *client, uint8_t *buf, uint8_t length)
{
	uint8_t checksum = 0;
	int32_t i = 0;

	// Generate checksum
	for (i = 0; i < length; i++) {
		checksum += buf[i+1];
	}
	checksum = (~checksum + 1);

	// Compare ckecksum and dump fail data
	if (checksum != buf[length + 1]) {
		NVT_ERR("i2c packet checksum not match. (point_data[%d]=0x%02X, checksum=0x%02X)\n", (length+1), buf[length+1], checksum);

		for (i = 0; i < 10; i++) {
			NVT_ERR("%02X %02X %02X %02X %02X %02X\n", buf[1+i*6], buf[2+i*6], buf[3+i*6], buf[4+i*6], buf[5+i*6], buf[6+i*6]);
		}

		for (i = 0; i < (length - 60); i++) {
			NVT_ERR("%02X ", buf[1+60+i]);
		}

		return -1;
	}

	return 0;
}
#endif /* POINT_DATA_CHECKSUM */

#define POINT_DATA_LEN 64
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
#ifdef HTC_FEATURE
static void nvt_ts_input_report(void)
#else
static void nvt_ts_work_func(struct work_struct *work)
#endif
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 2] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};

	int32_t i = 0;
	int32_t finger_cnt = 0;

	mutex_lock(&ts->lock);

	if (unlikely(nvt_ts_get_skip_reset_int() == 1)) {
		nvt_ts_set_skip_reset_int(0);
		goto XFER_ERROR;
	}

#ifdef HTC_FEATURE
	if (debug_mask & TOUCH_BREAKDOWN_TIME)
		getnstimeofday(&ts->tp_read_start_time);
	if (debug_mask & TOUCH_BREAKDOWN_LOG)
		pr_info("[TP][KPI] %s: CTP_I2C_READ START\n", __func__);
#endif
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 2);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}

#ifdef HTC_FEATURE
	if (debug_mask & TOUCH_BREAKDOWN_TIME)
		getnstimeofday(&ts->tp_read_done_time);
	if (debug_mask & TOUCH_BREAKDOWN_LOG)
		pr_info("[TP][KPI] %s: CTP_I2C_READ DONE\n", __func__);

	//--- dump I2C buf ---
	if (debug_mask & SHOW_INT_I2C_BUF) {
		for (i = 0; i < 10; i++) {
			NVT_LOG("Data %2d: %02X %02X %02X %02X %02X %02X\n",
					i,
					point_data[1+i*6], point_data[2+i*6],
					point_data[3+i*6], point_data[4+i*6],
					point_data[5+i*6], point_data[6+i*6]);
		}
	}
#else
	//--- dump I2C buf ---
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	printk("\n");
#endif

#if POINT_DATA_CHECKSUM
	ret = nvt_ts_point_data_checksum(ts->client, point_data, POINT_DATA_LEN);
	if (ret < 0) {
		goto XFER_ERROR;
	}
#endif /* POINT_DATA_CHECKSUM */

	finger_cnt = 0;
	input_id = (uint8_t)(point_data[1] >> 3);


#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		nvt_ts_wakeup_gesture_report(input_id);
#if !defined(HTC_FEATURE)
		enable_irq(ts->client->irq);
#endif
		mutex_unlock(&ts->lock);
		return;
	}
#endif

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if (input_id > ts->max_touch_num)
			continue;

#ifdef HTC_FEATURE
		if (((point_data[position] & 0x03) == 0x01) || ((point_data[position] & 0x03) == 0x02)) {	//finger down (enter & moving)
#else
		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#endif
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			input_w = (uint32_t)(point_data[position + 4]) + 10;
			if (input_w > 255)
				input_w = 255;
#ifdef HTC_FEATURE
			input_p = 1;
#else
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;

				// Fix : Multi-touch fingers are filterred when input_p is 0
				if (input_p == 0)
					input_p = 1;
			} else {
				input_p = 1;
			}
#endif
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
				continue;

			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
#ifdef HTC_FEATURE
			ts->report_points[input_id - 1].x = input_x;
			ts->report_points[input_id - 1].y = input_y;
			ts->report_points[input_id - 1].z = input_p;
			ts->report_points[input_id - 1].type = !!(point_data[position] & 0x04);
#endif

			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

#if !defined(HTC_FEATURE)
	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#endif

#else /* #if MT_PROTOCOL_B */

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);

		if ((point_data[position] & 0x07) == 0x03) {	// finger up (break)
			continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
		} else if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			input_w = (uint32_t)(point_data[position + 4]) + 10;
			if (input_w > 255)
				input_w = 255;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;

				// Fix : Multi-touch fingers are filterred when input_p is 0
				if (input_p == 0)
					input_p = 1;
			} else {
				input_p = 1;
			}

			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
				continue;

			press_id[input_id - 1] = 1;
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);

			input_mt_sync(ts->input_dev);

			finger_cnt++;
		}
	}
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);

		input_mt_sync(ts->input_dev);
	}
#endif /* #if MT_PROTOCOL_B */


#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);
#ifdef HTC_FEATURE
	if (debug_mask & (TOUCH_KPI_LOG | TOUCH_BREAKDOWN_TIME)) {
		getnstimeofday(&ts->tp_sync_time);
#if 0 //systrace, trace_clock_set_rate is not supported now
		if (ts->report_points[i].y > 0)
			trace_clock_set_rate("tp_report", ts->report_points[i].y, -4);
#endif
	}
	if ((point_data[63] <= HOPPINT_STATUS_MAX) && (point_data[64] <= HOPPINT_SEED_MAX)) {
		if ((point_data[63] != ts->report_noise.hopping_status) || (point_data[64] != ts->report_noise.hopping_seed)) {
			ts->report_noise.hopping_status = point_data[63];
			ts->report_noise.hopping_seed = point_data[64];
			NVT_LOG("HS=%d, Freq=%d\n", ts->report_noise.hopping_status, ts->report_noise.hopping_seed);
		}
	} else {
		NVT_LOG("Invalid HS, Freq value\n");
		ts->report_noise.hopping_status = -1;
		ts->report_noise.hopping_seed = -1;
	}
	if (debug_mask & TOUCH_BREAKDOWN_LOG)
		pr_info("[TP][KPI] %s: input_sync done\n", __func__);

	if (debug_mask & TOUCH_DOWN_UP_LOG) {
		for (i = 0; i < ts->max_touch_num; i++) {
			if (ts->report_points[i].press_status != press_id[i]) {
				if (press_id[i] == 1) { //press
					NVT_LOG("Screen:%c[%02d]:%s, X=%d, Y=%d, Z=%d, HS=%d, Freq=%d\n",
							ts->report_points[i].type ? 'G':'F',
							i,
							"Down",
							ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
							ts->report_noise.hopping_status,
							ts->report_noise.hopping_seed);
				} else { //release
					NVT_LOG("Screen:%c[%02d]:%s, X=%d, Y=%d, Z=%d, HS=%d, Freq=%d\n",
							ts->report_points[i].type ? 'G':'F',
							i,
							"Up",
							ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
							ts->report_noise.hopping_status,
							ts->report_noise.hopping_seed);
				}
				ts->report_points[i].press_status = press_id[i];
			}
		}
	} else if (debug_mask & TOUCH_KPI_LOG) {
		for (i = 0; i < ts->max_touch_num; i++) {
			if(ts->report_points[i].press_status != press_id[i]) {
				if (press_id[i] == 1) { //press
					pr_info("[TP][KPI] Screen:%c[%02d]:%s, X=%4d, Y=%4d, Z=%3d, time:\t%ld.%06ld\tto\t%ld.%06ld\n",
							ts->report_points[i].type ? 'G':'F',
							i,
							"Down",
							ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
							ts->tp_handler_time.tv_sec % 1000,
							ts->tp_handler_time.tv_nsec / 1000,
							ts->tp_sync_time.tv_sec % 1000,
							ts->tp_sync_time.tv_nsec / 1000);
				} else { //release
					pr_info("[TP][KPI] Screen:%c[%02d]:%s, X=%4d, Y=%4d, Z=%3d, time:\t%ld.%06ld\tto\t%ld.%06ld\n",
							ts->report_points[i].type ? 'G':'F',
							i,
							"Up",
							ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
							ts->tp_handler_time.tv_sec % 1000,
							ts->tp_handler_time.tv_nsec / 1000,
							ts->tp_sync_time.tv_sec % 1000,
							ts->tp_sync_time.tv_nsec / 1000);
				}
				ts->report_points[i].press_status = press_id[i];
			} else if (press_id[i] == 1)
				pr_info("[TP][KPI] Screen:%c[%02d]:%s, X=%4d, Y=%4d, Z=%3d, time:\t%ld.%06ld\tto\t%ld.%06ld\n",
						ts->report_points[i].type ? 'G':'F',
						i,
						"Move",
						ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
						ts->tp_handler_time.tv_sec % 1000,
						ts->tp_handler_time.tv_nsec / 1000,
						ts->tp_sync_time.tv_sec % 1000,
						ts->tp_sync_time.tv_nsec / 1000);
		}
	} else if (debug_mask & TOUCH_BREAKDOWN_TIME) {
		for (i = 0; i < ts->max_touch_num; i++) {
			if (ts->report_points[i].press_status != press_id[i]) {
				if (press_id[i] == 1) { //press
					pr_info("[TP][KPI] Screen:%c[%02d]:%s, X=%4d, Y=%4d, Z=%3d, time=\t%4ld\tBUS:\t%4ld\tD\n",
							ts->report_points[i].type ? 'G':'F',
							i,
							"Down",
							ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
							((ts->tp_sync_time.tv_sec * 1000000000 + ts->tp_sync_time.tv_nsec)
									- (ts->tp_handler_time.tv_sec * 1000000000 + ts->tp_handler_time.tv_nsec)) / 1000,
							((ts->tp_read_done_time.tv_sec * 1000000000 + ts->tp_read_done_time.tv_nsec)
									- (ts->tp_read_start_time.tv_sec * 1000000000 + ts->tp_read_start_time.tv_nsec)) / 1000);
				} else { //release
					pr_info("[TP][KPI] Screen:%c[%02d]:%s, X=%4d, Y=%4d, Z=%3d, time=\t%4ld\tBUS:\t%4ld\tU\n",
							ts->report_points[i].type ? 'G':'F',
							i,
							"Up",
							ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
							((ts->tp_sync_time.tv_sec * 1000000000 + ts->tp_sync_time.tv_nsec)
									- (ts->tp_handler_time.tv_sec * 1000000000 + ts->tp_handler_time.tv_nsec)) / 1000,
							((ts->tp_read_done_time.tv_sec * 1000000000 + ts->tp_read_done_time.tv_nsec)
									- (ts->tp_read_start_time.tv_sec * 1000000000 + ts->tp_read_start_time.tv_nsec)) / 1000);
				}
				ts->report_points[i].press_status = press_id[i];
			} else if (press_id[i] == 1)
				pr_info("[TP][KPI] Screen:%c[%02d]:%s, X=%4d, Y=%4d, Z=%3d, time=\t%4ld\tBUS:\t%4ld\tM\n",
						ts->report_points[i].type ? 'G':'F',
						i,
						"Move",
						ts->report_points[i].x, ts->report_points[i].y, ts->report_points[i].z,
						((ts->tp_sync_time.tv_sec * 1000000000 + ts->tp_sync_time.tv_nsec)
								- (ts->tp_handler_time.tv_sec * 1000000000 + ts->tp_handler_time.tv_nsec)) / 1000,
						((ts->tp_read_done_time.tv_sec * 1000000000 + ts->tp_read_done_time.tv_nsec)
								- (ts->tp_read_start_time.tv_sec * 1000000000 + ts->tp_read_start_time.tv_nsec)) / 1000);
		}
	}
#endif

XFER_ERROR:
#if !defined(HTC_FEATURE)
	enable_irq(ts->client->irq);
#endif

	mutex_unlock(&ts->lock);
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
#ifdef HTC_FEATURE
	if (debug_mask & (TOUCH_KPI_LOG | TOUCH_BREAKDOWN_TIME))
		getnstimeofday(&ts->tp_handler_time);
	if (debug_mask & TOUCH_BREAKDOWN_LOG)
		pr_info("[TP][KPI] %s: ++\n", __func__);
#else
	disable_irq_nosync(ts->client->irq);
#endif

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
	}
#endif

#ifdef HTC_FEATURE
	nvt_ts_input_report();
#else
	queue_work(nvt_wq, &ts->nvt_work);
#endif

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {
		nvt_bootloader_reset();
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}


/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
#ifdef HTC_FEATURE
static void dsi_status_detect(int status);
static struct dsi_status_notifier dsi_event_notifier = {
	.name = "nvt_dsi_event_handler",
	.func = dsi_status_detect,
};
#endif

static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif
#ifdef HTC_FEATURE
	static char *htc_bootmode = NULL;

	htc_bootmode = htc_get_bootmode();
	NVT_LOG("%s: htc_bootmode = %s", __func__, htc_bootmode);
	if((strcmp(htc_bootmode, "offmode_charging") == 0) ||
			(strcmp(htc_bootmode, "charger") == 0) ||
			(strcmp(htc_bootmode, "recovery") == 0) ||
			(strcmp(htc_bootmode, "MFG_MODE_OFFMODE_CHARGING") == 0) ||
			(strcmp(htc_bootmode, "MFG_MODE_POWER_TEST") == 0) ||
			(strcmp(htc_bootmode, "MFG_MODE_RECOVERY") == 0)) {
		NVT_LOG("%s: --skip--", __func__);
		return 0;
	}

	NVT_LOG("%s: +++\n", __func__);

	if (get_tamper_sf() == 0) {
		debug_mask = TOUCH_DOWN_UP_LOG;
		NVT_LOG("%s: set debug_mask to %d\n", __func__, debug_mask);
	}
#else
	NVT_LOG("start\n");
#endif

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);


	//---parse dts---
	novatek_parse_dt(&client->dev);
	ts->tp_mode_state = 0;

	//---request RST-pin & INT-pin---
#ifdef HTC_FEATURE
	/* Get pinctrl if target uses pinctrl */
	ts->tp_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(ts->tp_pinctrl)) {
		if (PTR_ERR(ts->tp_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		NVT_LOG("Target does not use pinctrl\n");
		ts->tp_pinctrl = NULL;
	}

	if (ts->tp_pinctrl) {
		ret = novatek_pinctrl_configure(ts, true);
		if (ret) {
			NVT_ERR("cannot set ts pinctrl active state\n");
			goto err_check_functionality_failed;
		}
	}

	ts->report_points = kzalloc(sizeof(struct nvt_finger_info) * TOUCH_MAX_FINGER_NUM, GFP_KERNEL);
	if (gpio_is_valid(ts->power_gpio)) {
		ret = gpio_request_one(ts->power_gpio, GPIOF_OUT_INIT_HIGH, "NVT-pwr");
		if (ret)
			NVT_ERR("Failed to get NVT-pwr GPIO\n");
		gpio_set_value(ts->power_gpio, 1);
		NVT_LOG("set power-gpio(%d) to %d\n", ts->power_gpio,
				gpio_get_value(ts->power_gpio));
	} else
		NVT_ERR("power_gpio is invalid\n");

	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-rst");
		if (ret)
			NVT_ERR("Failed to get NVT-rst GPIO\n");
		gpio_set_value(ts->reset_gpio, 1);
		NVT_LOG("set reset-gpio(%d) to %d\n", ts->reset_gpio,
				gpio_get_value(ts->reset_gpio));
	} else
		NVT_ERR("reset_gpio is invalid\n");

	if (gpio_is_valid(ts->switch_gpio)) {
		ret = gpio_request_one(ts->switch_gpio, GPIOF_OUT_INIT_LOW, "NVT-swh");
		if (ret)
			NVT_ERR("Failed to get NVT-swh GPIO\n");
		gpio_set_value(ts->switch_gpio, 0);
		ts->tp_bus_sel_en = 1;
		NVT_LOG("set switch-gpio(%d) to %d\n", ts->switch_gpio,
				gpio_get_value(ts->switch_gpio));
	} else {
		ts->tp_bus_sel_en = 0;
		NVT_ERR("switch_gpio is invalid\n");
	}
#else
	ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-rst");
	if (ret)
		NVT_ERR("Failed to get NVT-rst GPIO\n");
	gpio_set_value(ts->reset_gpio, 1);
#endif

	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret)
			NVT_ERR("Failed to get NVT-int GPIO\n");
	} else
		NVT_ERR("irq_gpio is invalid\n");

#ifdef HTC_FEATURE //Display driver control
#else
	// set output high due to display module reset pin RESX is global reset
	ret = gpio_request_one(ts->disp_rst_gpio, GPIOF_OUT_INIT_HIGH, "NVT-disp-rst");
	if (ret)
		NVT_ERR("Failed to get NVT-disp-rst GPIO\n");
	gpio_set_value(ts->disp_rst_gpio, 1);
	gpio_free(ts->disp_rst_gpio);
#endif

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

#ifdef HTC_FEATURE
	atomic_set(&ts->firmware_updating, 0);
	ts->report_noise.hopping_status = -1;
	ts->report_noise.hopping_seed = -1;
	nvt_set_hopping_debug_mode(true);
#endif

	mutex_init(&ts->lock);

#if !defined(HTC_FEATURE)
	//---create workqueue---
	nvt_wq = create_workqueue("nvt_wq");
	if (!nvt_wq) {
		NVT_ERR("nvt_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts->nvt_work, nvt_ts_work_func);
#endif

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

#ifdef HTC_FEATURE
	ts->int_trigger_type = INT_TRIGGER_TYPE | IRQF_ONESHOT;
#else
	ts->int_trigger_type = INT_TRIGGER_TYPE;
#endif

	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if !defined(HTC_FEATURE)
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}


	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);

#ifdef HTC_FEATURE
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_irq_handler, ts->int_trigger_type,
				client->name, ts);
#elif WAKEUP_GESTURE
		ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type | IRQF_NO_SUSPEND, client->name, ts);
#else
		ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type, client->name, ts);
#endif
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			disable_irq(client->irq);
			ts->irq_enabled = false;
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

#if BOOT_UPDATE_FIRMWARE
	if ((strcmp(htc_bootmode, "download")  == 0) ||
	    (strcmp(htc_bootmode, "RUU") == 0) ||
	    (strcmp(htc_bootmode, "offmode_charging") == 0) ||
	    (strcmp(htc_bootmode, "recovery") == 0)) {
	    NVT_LOG("skip firmware update since under %s mode.\n", htc_bootmode);
	} else {
		nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
		if (!nvt_fwu_wq) {
			NVT_ERR("nvt_fwu_wq create workqueue failed\n");
			ret = -ENOMEM;
			goto err_create_nvt_fwu_wq_failed;
		}
		INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
		// please make sure boot update start after display reset(RESX) sequence
		queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
	}
#endif

	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	mutex_unlock(&ts->lock);

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
#ifdef HTC_FEATURE
	ret = nvt_extra_proc_init(ts);
#else
	ret = nvt_extra_proc_init();
#endif
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if defined(CONFIG_FB)
#ifdef HTC_FEATURE
	dsi_register_notifier(&dsi_event_notifier);
#endif

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret) {
		NVT_ERR("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	ret = register_early_suspend(&ts->early_suspend);
	if(ret) {
		NVT_ERR("register early suspend failed. ret=%d\n", ret);
		goto err_register_early_suspend_failed;
	}
#endif

#ifdef HTC_FEATURE
	nvt_set_irq_enable(true);
	NVT_LOG("%s: ---\n", __func__);
#else
	NVT_LOG("end\n");
	enable_irq(client->irq);
#endif
	return 0;

#if defined(CONFIG_FB)
err_register_fb_notif_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
err_register_early_suspend_failed:
#endif
err_init_NVT_ts:
	free_irq(client->irq,ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
#if !defined(HTC_FEATURE)
err_create_nvt_wq_failed:
#endif
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
err_check_functionality_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	//struct nvt_ts_data *ts = i2c_get_clientdata(client);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

	mutex_destroy(&ts->lock);

	NVT_LOG("Removing driver...\n");

	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

#ifdef HTC_FEATURE
static void release_all_fingers(void)
{
	uint32_t i = 0;

	/* release all touches */
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_sync(ts->input_dev);

	memset(ts->report_points, 0, sizeof(struct nvt_finger_info) * TOUCH_MAX_FINGER_NUM);

	NVT_LOG("Release touch\n");
}

#ifdef CONFIG_NANOHUB_TP_SWITCH
void switch_sensor_hub(int mode)
{
	int mask = 0;

	if(ts->tp_bus_sel_en == 0) {
		NVT_LOG("[SensorHub] Switch disabled\n");
		return;
	}

	if (ts->support_glove)
		mask = !!(ts->tp_mode_state & GLOVE_MODE_BIT) << 1;

	if (gpio_is_valid(ts->switch_gpio)) {
		switch (mode) {
		case 0:
			nanohub_tp_status(0 | mask);
			gpio_direction_output(ts->switch_gpio, 0);
			ts->i2c_to_mcu = 0;
			NVT_LOG("[SensorHub] Switch touch i2c to CPU\n");
			break;
		case 1:
			gpio_direction_output(ts->switch_gpio, 1);
			ts->i2c_to_mcu = 1;
			NVT_LOG("[SensorHub] Switch touch i2c to MCU\n");
			nanohub_tp_status(1 | mask);
			break;
		default:
			NVT_ERR("incorrect mode %d\n", mode);
			break;
		}
	}
	else
		NVT_ERR("invalid switch gpio %d\n", ts->switch_gpio);
}
#endif
#endif

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
#if !defined(HTC_FEATURE)
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	mutex_lock(&ts->lock);

	NVT_LOG("%s: +++\n", __func__);

#if WAKEUP_GESTURE
	bTouchIsAwake = 0;

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
#if 0 // Do not set 0xFF first, ToDo
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
#else
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif

	enable_irq_wake(ts->client->irq);

	NVT_LOG("Enabled touch wakeup gesture\n");

#else // WAKEUP_GESTURE
	disable_irq(ts->client->irq);

	//---write i2c command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x12;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif // WAKEUP_GESTURE

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	mutex_unlock(&ts->lock);

	NVT_LOG("%s: ---\n", __func__);

	return 0;
}
#endif // !defined(HTC_FEATURE)

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	mutex_lock(&ts->lock);

	NVT_LOG("%s: +++\n", __func__);

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_REK);

#ifdef HTC_FEATURE
	release_all_fingers();

	if (ts->tp_mode_state & GLOVE_MODE_BIT)
		nvt_set_tp_mode(GLOVE_MODE_BIT, true);
	nvt_set_hopping_debug_mode(true);
#endif

#if WAKEUP_GESTURE
	bTouchIsAwake = 1;
#ifdef HTC_FEATURE
	nvt_set_irq_enable(true);
#endif
#else
	enable_irq(ts->client->irq);
#endif

	mutex_unlock(&ts->lock);

	NVT_LOG("%s: ---\n", __func__);

	return 0;
}

#ifdef HTC_FEATURE
static void dsi_status_detect(int status)
{
	uint8_t buf[4] = {0};

//	NVT_LOG("suspend: received dsi event=%d\n", status);

	switch (status) {
	case LCM_EARLY_MIPI_OFF_CMD:
		NVT_LOG("%s: +++\n", __func__);
//		bTouchIsAwake = 0;

		nvt_set_irq_enable(false);

		mutex_lock(&ts->lock);
		//---write i2c command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		mutex_unlock(&ts->lock);

		msleep(20);
		NVT_LOG("Enabled touch wakeup gesture\n");
		break;
	case LCM_MIPI_OFF_CMD:
//		mutex_lock(&ts->lock);
//		release_all_fingers();
//		mutex_unlock(&ts->lock);
		break;
	case LCM_EARLY_POWERDOWN:
		msleep(120);

		mutex_lock(&ts->lock);
		//---write i2c command to enter "TP stop scanning"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x1C;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		mutex_unlock(&ts->lock);

		msleep(20);
		NVT_LOG("Stop TP scanning\n");
		break;
	case LCM_POWERDOWN:
#ifdef CONFIG_NANOHUB_TP_SWITCH
		switch_sensor_hub(1);
#endif
		NVT_LOG("%s: ---\n", __func__);
		break;
	default:
		NVT_ERR("%s: Upsupported status\n", __func__);
		break;
	};
}
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN) {
#if !defined(HTC_FEATURE)
			nvt_ts_suspend(&ts->client->dev);
#endif
		}
#ifdef HTC_FEATURE
#ifdef CONFIG_NANOHUB_TP_SWITCH
		else if (*blank == FB_BLANK_UNBLANK) {
			switch_sensor_hub(0);
		}
#endif
#endif
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			nvt_ts_resume(&ts->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver early suspend function.

return:
	n.a.
*******************************************************/
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

/*******************************************************
Description:
	Novatek touchscreen driver late resume function.

return:
	n.a.
*******************************************************/
static void nvt_ts_late_resume(struct early_suspend *h)
{
	nvt_ts_resume(ts->client);
}
#endif

#if 0
static const struct dev_pm_ops nvt_ts_dev_pm_ops = {
	.suspend = nvt_ts_suspend,
	.resume  = nvt_ts_resume,
};
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts",},
	{ },
};
#endif
/*
static struct i2c_board_info __initdata nvt_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(NVT_I2C_NAME, I2C_FW_Address),
	},
};
*/

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
//	.suspend	= nvt_ts_suspend,
//	.resume		= nvt_ts_resume,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
#if 0
#ifdef CONFIG_PM
		.pm = &nvt_ts_dev_pm_ops,
#endif
#endif
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
#ifdef HTC_FEATURE
static void __init nvt_driver_init_async(void *unused, async_cookie_t cookie)
{
	NVT_LOG("%s: ++\n", __func__);
	//---add i2c driver---
	if (i2c_add_driver(&nvt_i2c_driver))
		pr_err("%s: failed to add i2c driver", __func__);
	else
		NVT_LOG("%s: --\n", __func__);
	return;
}

static int32_t __init nvt_driver_init(void)
{
	NVT_LOG("%s: ++\n", __func__);
	async_schedule(nvt_driver_init_async, NULL);
	NVT_LOG("%s: --\n", __func__);

	return 0;
}
#else
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("%s: +++\n", __func__);
	//---add i2c driver---
	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		pr_err("%s: failed to add i2c driver", __func__);
		goto err_driver;
	}

	NVT_LOG("%s: ---\n", __func__);

err_driver:
	return ret;
}
#endif

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);

#if !defined(HTC_FEATURE)
	if (nvt_wq)
		destroy_workqueue(nvt_wq);
#endif

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif
}

late_initcall(nvt_driver_init);
//module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
