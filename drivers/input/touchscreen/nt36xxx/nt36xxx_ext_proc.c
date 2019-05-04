/*
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 8361 $
 * $Date: 2016-12-29 20:01:24 +0800 (週四, 29 十二月 2016) $
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


#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"

#define I2C_TANSFER_LENGTH  64

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static uint8_t fw_ver = 0;
static uint8_t x_num = 0;
static uint8_t y_num = 0;
static uint8_t button_num = 0;
#ifdef HTC_FEATURE
#define READ_DIFF_DATA 1
#define READ_RAW_DATA 2
static unsigned char diag_command = READ_RAW_DATA;
#endif

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;

extern struct nvt_ts_data *ts;
extern int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t nvt_clear_fw_status(void);
extern int32_t nvt_check_fw_status(void);
#ifdef HTC_FEATURE
extern void nvt_set_irq_enable(bool enable);
extern void nvt_set_tp_mode(uint8_t tp_mode, bool enable);
#endif

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int8_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int8_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	fw_ver = buf[1];
	x_num = buf[3];
	y_num = buf[4];
	button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		x_num = 0;
		y_num = 0;
		button_num = 0;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			ret = -1;
		}
	} else {
		ret = 0;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[I2C_TANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = x_num * y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---change xdata index---
		buf[0] = 0xFF;
		buf[1] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 16) & 0xFF;
		buf[2] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read xdata by I2C_TANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / I2C_TANSFER_LENGTH); j++) {
			//---read data---
			buf[0] = I2C_TANSFER_LENGTH * j;
			CTP_I2C_READ(ts->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + I2C_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---change xdata index---
		buf[0] = 0xFF;
		buf[1] = ((xdata_addr + data_len - residual_len) >> 16) & 0xFF;
		buf[2] = ((xdata_addr + data_len - residual_len) >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read xdata by I2C_TANSFER_LENGTH
		for (j = 0; j < (residual_len / I2C_TANSFER_LENGTH + 1); j++) {
			//---read data---
			buf[0] = I2C_TANSFER_LENGTH * j;
			CTP_I2C_READ(ts->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + I2C_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	buf[0] = 0xFF;
	buf[1] = (xdata_btn_addr >> 16) & 0xFF;
	buf[2] = ((xdata_btn_addr >> 8) & 0xFF);
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[x_num * y_num + i] = (buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = x_num;
    *m_y_num = y_num;
#if TOUCH_KEY_NUM > 0
    memcpy(buf, xdata, ((x_num * y_num + TOUCH_KEY_NUM) * 4));
#else
    memcpy(buf, xdata, (x_num * y_num * 4));
#endif /* #if TOUCH_KEY_NUM > 0 */
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", fw_ver, x_num, y_num, button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < y_num; i++) {
		for (j = 0; j < x_num; j++) {
			seq_printf(m, "%5d, ", (int16_t)xdata[i * x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", (int16_t)xdata[x_num * y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	NVT_LOG("--\n");

	mutex_unlock(&ts->lock);

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_get_fw_info();

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
#ifdef HTC_FEATURE
static ssize_t nvt_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += snprintf(buf + count, PAGE_SIZE, "novatek, fw_ver:%d\n", ts->fw_ver);

	return count;
}

static ssize_t nvt_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t i = 0;
	int32_t j = 0;
	size_t count = 0;
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (diag_command == READ_DIFF_DATA) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	//show data
	for (i = 0; i < y_num; i++) {
		for (j = 0; j < x_num; j++) {
			count += snprintf(buf + count, PAGE_SIZE, "%5d ",
					(int16_t)xdata[i * x_num + j]);
		}
		count += snprintf(buf + count, PAGE_SIZE, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%5d ",
				(int16_t)xdata[x_num * y_num + i]);
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
#endif

	//seq_printf(m, "\n\n");

	NVT_LOG("--\n");
	return count;
}

static ssize_t nvt_diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] == '1')
		diag_command = READ_DIFF_DATA;
	else if (buf[0] == '2')
		diag_command = READ_RAW_DATA;

	return count;
}


static ssize_t nvt_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int reset;

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	NVT_LOG("tp reset(%d)\n", reset);
	if (reset != 1)
		return -EINVAL;

	gpio_set_value(ts->reset_gpio, 1);
	msleep(20);
	gpio_set_value(ts->reset_gpio, 0);
	msleep(50);
	gpio_set_value(ts->reset_gpio, 1);
	msleep(20);

	return count;
}

static ssize_t nvt_attn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	if (gpio_is_valid(ts->irq_gpio))
		size += snprintf(buf + size, PAGE_SIZE,
				"gpio%d: %d\n",
				ts->irq_gpio,
				gpio_get_value(ts->irq_gpio));
	return size;
}

extern uint32_t debug_mask;
static ssize_t nvt_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0)
		return -EINVAL;

	debug_mask = value;

	return count;
}

static ssize_t nvt_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t size = 0;

	size += snprintf(buf + size, PAGE_SIZE,
			"debug_mask = %d (0x%08X)\n", debug_mask, debug_mask);
	return size;
}

static ssize_t nvt_glove_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += snprintf(buf + count, PAGE_SIZE, "%u\n", (uint8_t)(ts->tp_mode_state & GLOVE_MODE_BIT));

	return count;
}

static ssize_t nvt_glove_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (!ts->support_glove)
		return count;

	sscanf(buf, "%d", &value);

	if (value > 1 || value < 0) {
		NVT_LOG("%s: wrong parameter\n", __func__);
		return -EINVAL;
	}

	nvt_set_tp_mode(GLOVE_MODE_BIT, !!value);
	NVT_LOG("%s: tp_mode_state change to %d, input: %d\n", __func__, ts->tp_mode_state, value);

	return count;
}

static ssize_t nvt_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int enabled = -1;

	if (sscanf(buf, "%d", &enabled) != 1)
		return -EINVAL;

	NVT_LOG("%s: set irq %s\n", __func__, enabled ? "enable" : "disable");

	switch (enabled) {
	case 0:
		nvt_set_irq_enable(false);
		break;
	case 1:
		nvt_set_irq_enable(true);
		break;
	default:
		NVT_ERR("%s: unsupport enabled value\n", __func__);
		break;
	}

	return count;
}

static ssize_t nvt_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size = 0;
	size += snprintf(buf + size, PAGE_SIZE,
			"enabled : %s, %d\n",
			(ts->irq_enabled % 2) ? "Enabled" : "Disabled",
			ts->irq_enabled);
	return size;
}

#ifdef CONFIG_NANOHUB_TP_SWITCH
extern void switch_sensor_hub(int mode);
static ssize_t nvt_bus_switch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0)
		return -EINVAL;

	NVT_LOG("bus_switch(%d)\n", value);

	switch (value) {
	case 0:
		ts->tp_bus_sel_en = 0;
		break;
	case 1:
		ts->tp_bus_sel_en = 1;
		break;
	case 2:
		switch_sensor_hub(0);/* Switch bus to CPU */
		nvt_set_irq_enable(true);
		break;
	case 3:
		nvt_set_irq_enable(false);
		switch_sensor_hub(1);/* Switch bus to MCU */
		break;
	default:
		NVT_ERR("%s: unsupport value\n", __func__);
		break;
	}

	return count;
}

static ssize_t nvt_bus_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t size = 0;

	if (ts->tp_bus_sel_en == 0)
		size += snprintf(buf + size, PAGE_SIZE,
				"Switch disabled\n");
	else {
		if (gpio_is_valid(ts->switch_gpio))
			size += snprintf(buf + size, PAGE_SIZE,
					"Switch bus to %s\n",
					gpio_get_value(ts->switch_gpio) ? "MCU" : "CPU");
	}

	return size;
}
#endif

#ifdef CONFIG_NANOHUB_TP_SWITCH
static DEVICE_ATTR(bus_switch, (S_IWUSR | S_IRUGO),
		nvt_bus_switch_show, nvt_bus_switch_store);
#endif
static DEVICE_ATTR(enabled, (S_IWUSR | S_IRUGO), nvt_enabled_show, nvt_enabled_store);
static DEVICE_ATTR(glove_setting, (S_IWUSR | S_IRUGO), nvt_glove_show, nvt_glove_store);
static DEVICE_ATTR(debug_level, (S_IWUSR | S_IRUGO), nvt_debug_show, nvt_debug_store);
static DEVICE_ATTR(attn, S_IRUGO, nvt_attn_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, nvt_reset_store);
static DEVICE_ATTR(diag, (S_IWUSR | S_IRUGO), nvt_diag_show, nvt_diag_store);
static DEVICE_ATTR(vendor, S_IRUGO, nvt_vendor_show, NULL);

static struct attribute *htc_attrs[] = {
#ifdef CONFIG_NANOHUB_TP_SWITCH
	&dev_attr_bus_switch.attr,
#endif
	&dev_attr_enabled.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_attn.attr,
	&dev_attr_reset.attr,
	&dev_attr_diag.attr,
	&dev_attr_vendor.attr,
	NULL
};

static struct attribute *glove_attr = &dev_attr_glove_setting.attr;

static const struct attribute_group attr_group = {
	.attrs = htc_attrs,
};
static struct kobject *android_touch_kobj;
int32_t nvt_extra_proc_init(struct nvt_ts_data* ts)
#else
int32_t nvt_extra_proc_init(void)
#endif
{
#ifdef HTC_FEATURE
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		NVT_ERR("subsystem_register failed!\n");
		return -ENOMEM;
	}

	if (sysfs_create_link(android_touch_kobj, &ts->input_dev->dev.kobj, "nt36xxx") < 0) {
		NVT_ERR("failed to create link\n");
		return -ENOMEM;
	}

	if (sysfs_create_group(android_touch_kobj, &attr_group) < 0) {
		NVT_ERR("Failed to create sysfs attributes\n");
	}

	if (ts->support_glove) {
		if (sysfs_create_file(android_touch_kobj, glove_attr) < 0)
			NVT_ERR("Failed to create glove attribute\n");
	}
#endif

	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_fw_version Succeeded!\n");
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_baseline Succeeded!\n");
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_raw Succeeded!\n");
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_diff Succeeded!\n");
	}

	return 0;
}
#endif
