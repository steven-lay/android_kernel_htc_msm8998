/*
 * XR15510 edge sensor driver - Copyright (C) 2015 Exar Corporation.
 * Author: Martin xu <martin.xu@exar.com>
 *
 *  Based on sc16is7xx.c, by Jon Ringle <jringle@gridpoint.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include "linux/version.h"
#include <linux/input.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#ifdef CONFIG_NANOHUB_EDGE
#include <linux/nanohub_htc.h>
#endif

#define CONFIG_EDGE_SENSOR_FW_UPDATE
#if defined(CONFIG_EDGE_SENSOR_FW_UPDATE)
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#define TYPE_B_PROTOCOL
#endif

#define EDGE_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[EDGE][DBG] " fmt, ##__VA_ARGS__)
#define EDGE_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[EDGE] " fmt, ##__VA_ARGS__)
#define EDGE_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[EDGE][ERR] " fmt, ##__VA_ARGS__)

#define GPIO_HIGH	            1 /* Level High*/
#define GPIO_LOW	            0 /* Level Low */

struct input_dev *input_dev;
#define XR15510_NAME	        "xr15510"
#define INPUT_PHYS_NAME         "exar/edge_input"
#define NUM_OF_SENSORS          16
#define SENSOR_X1_RIGHT         200
#define SENSOR_X2_LEFT          1200

#define MCU_INIT_TIME           100
#define REPORT_DATA_THRESHOLD   40
#define MASK_8BIT	            0xFF
#define CMD_LEN		            4  /* message cotent length */
#define ACK_LEN		            4  /* message cotent length */
#define PERIOD		            40 /* ms */
#define MAX_FW_UPDATE_RETRY     3

#define XR15510_BL_VERSION      0x00
#define XR15510_FW_VERSION      0x03
#define XR15510_UNKNOWN         0x10
#define XR15510_DATA_COUNT      0x7C
#define XR15510_DATA_BUFFER     0x7f

// project id
#define Project_OCN     0

// hw version
#define OCE_P1P2_SENSOR 0
#define OCE_P3_SENSOR   1

static long xr15510_ioctl(struct file *port, unsigned int cmd, unsigned long arg);
static int  xr15510_i2c_write(struct i2c_client *client, u8 reg, u8 *data, unsigned short length);
static int  xr15510_i2c_read(struct i2c_client *client, u8 reg, u8 *val, unsigned short length);
static int  xr15510_hw_reset(void);
static int  check_fw_version(void);
static void edge_fw_check_work(struct work_struct *work);
static void edge_fw_update_work(struct work_struct *work);

struct xr15510_data {
    struct i2c_client *client;

    /* misc device */
    struct miscdevice miscdev;
    struct device *sensor_dev;

    /* gpios */
    uint32_t gpio_reset;
    uint32_t gpio_int;
    uint32_t gpio_override;
    uint32_t gpio_sel;
    uint32_t gpio_power;
};

static uint32_t gpio_reset = -1;
static uint32_t gpio_int = -1;
static uint32_t gpio_override = -1;
static uint32_t gpio_sel = -1;
static uint32_t gpio_power = -1;

static int32_t fw_version = -1;
static int32_t fw_crc     = 0;
static int32_t fw_retry_cnt = 0;

bool polling_flag = 1;

struct workqueue_struct *edge_polling_workqueue;
struct workqueue_struct *fw_workqueue;
struct work_struct fw_upd_work;
struct work_struct fw_chk_work;
struct delayed_work edge_polling_work;

/* Default setting is profile#3 and it's gadc is 60 */
uint8_t p2_sensor_module = 0;
uint32_t edge_gadc = 60;
static uint32_t project_id = Project_OCN;
static uint8_t hw_id = OCE_P3_SENSOR;
static uint32_t switch_reset = 0;
struct mutex polling_lock;

struct pinctrl *edge_pinctrl;
struct pinctrl_state *gpio_state_active;
struct pinctrl_state *gpio_state_suspend;

static struct kobject *android_edge_kobj = NULL;

static struct i2c_client *xr15510_i2c_client = NULL;
static struct i2c_client *xr15510_i2c_flash_client = NULL;

/************************************************
 ******** Start: For developement stage *********
 ************************************************/

static int report_threshold = 40;
static int gadc = 40;
static int gadc_calibrated = 0;
module_param(report_threshold, int, 0600);
module_param(gadc, int, 0600);
module_param(gadc_calibrated, int, 0600);

//#define SENSOR_REVERSE
#define DEBUG_LOG_ENABLE
int gain_1[8] = {13, 31, 33, 22,
                 13, 28, 28, 18};
int gain_2[8] = {15, 26, 30, 19,
                 17, 28, 25, 14};

/*************************************************
 ******** End: For developement stage  ***********
 *************************************************/

static ssize_t xr15510_vendor_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	uint16_t ret = 0;
	uint8_t bl_ver[ACK_LEN * 2] = {0x00};
	uint8_t fw_ver[ACK_LEN * 2] = {0x00};

    ret = xr15510_i2c_read(xr15510_i2c_client,
                XR15510_BL_VERSION, bl_ver, sizeof(bl_ver));
	if (ret) {
		EDGE_ERR_LOG("read bl_ver fail, ret = %d\n", ret);
		return  -EFAULT;
	}

    ret = xr15510_i2c_read(xr15510_i2c_client,
                XR15510_FW_VERSION, fw_ver, sizeof(fw_ver));
	if (ret) {
		EDGE_ERR_LOG("read fw_ver fail, ret = %d\n", ret);
		return  -EFAULT;
	}

	EDGE_INFO_LOG("%s: Bootloader verion: 0x%02X%02X%02X%02X; "
                      "Firmware version: 0x%02X\n",
		      __func__, bl_ver[7], bl_ver[6], bl_ver[5], bl_ver[4], fw_ver[0]);

	ret = snprintf(buf, PAGE_SIZE,
                      "Bootloader verion:\t0x%02X%02X%02X%02X\n"
                      "Firmware version:\t0x%02X\n",
		                bl_ver[7], bl_ver[6], bl_ver[5], bl_ver[4], fw_ver[0]);
	return ret;
}

static ssize_t xr15510_gpio_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	uint16_t ret = 0, value = 0;

	if (gpio_is_valid(gpio_reset)) {
		value = gpio_get_value(gpio_reset);
		EDGE_INFO_LOG("%s: gpio_reset: %d(%d)\n", __func__, gpio_reset, value);
		ret = snprintf(buf, PAGE_SIZE, "gpio_reset: %d(%d)\n", gpio_reset, value);
	} else
		ret = snprintf(buf, PAGE_SIZE, "gpio_reset: invalid\n");

	if (gpio_is_valid(gpio_int)) {
		value = gpio_get_value(gpio_int);
		EDGE_INFO_LOG("%s: gpio_intr:  %d(%d)\n", __func__, gpio_int, value);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_intr:  %d(%d)\n",
                                                            gpio_int, value);
	} else
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_intr:  invalid\n");

	if (gpio_is_valid(gpio_override)) {
		value = gpio_get_value(gpio_override);
		EDGE_INFO_LOG("%s: gpio_ovrde: %d(%d)\n", __func__, gpio_override, value);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_ovrde: %d(%d)\n",
                                                            gpio_override, value);
	} else
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_ovrde: invalid\n");

	if (gpio_is_valid(gpio_sel)) {
		value = gpio_get_value(gpio_sel);
		EDGE_INFO_LOG("%s: gpio_selct: %d(%d)\n", __func__, gpio_sel, value);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_selct: %d(%d)\n",
                                                            gpio_sel, value);
	} else
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_selct: invalid\n");

	if (gpio_is_valid(gpio_power)) {
		value = gpio_get_value(gpio_power);
		EDGE_INFO_LOG("%s: gpio_power: %d(%d)\n", __func__, gpio_power, value);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_power: %d(%d)\n",
                                                            gpio_power, value);
	} else
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "gpio_power: invalid\n");

	return ret;
}

static ssize_t xr15510_hw_version_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    uint16_t ret = 0;

    EDGE_INFO_LOG("%s: %d.%d\n", __func__, project_id, hw_id);
    ret = snprintf(buf, PAGE_SIZE, "%d.%d\n", project_id, hw_id);

    return ret;
}
static ssize_t xr15510_reset_store(struct device *dev,
				            struct device_attribute *attr,
                            const char *buf, size_t count)
{
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		EDGE_ERR_LOG("%s: Failed to get the inout value\n", __func__);
		return -EINVAL;
	}
	if (input > 2) {
		EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

	if (gpio_is_valid(gpio_reset)) {
		switch (input) {
		case 0:
		case 1:
			EDGE_INFO_LOG("%s: gpio_reset(%d) value was %d\n", __func__,
                                    gpio_reset, gpio_get_value(gpio_reset));
			gpio_set_value(gpio_reset, input);
			EDGE_INFO_LOG("%s: gpio_reset(%d) value is %d\n", __func__,
                                    gpio_reset, gpio_get_value(gpio_reset));
			break;
		case 2:
			EDGE_INFO_LOG("%s: gpio_reset(%d) value was %d\n", __func__,
                                    gpio_reset, gpio_get_value(gpio_reset));
			gpio_set_value(gpio_reset, 1);
			EDGE_INFO_LOG("%s: gpio_reset(%d) value is %d\n", __func__,
                                    gpio_reset, gpio_get_value(gpio_reset));
			msleep(100);
			gpio_set_value(gpio_reset, 0);
			EDGE_INFO_LOG("%s: gpio_reset(%d) value is %d\n", __func__,
                                    gpio_reset, gpio_get_value(gpio_reset));
			break;
		default:
			EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
			break;
		}
	}

	return count;
}

static ssize_t xr15510_data_store(struct device *dev,
                            	struct device_attribute *attr,
                                const char *buf, size_t count)
{
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		EDGE_ERR_LOG("%s: Failed to get the inout value\n", __func__);
		return -EINVAL;
	}
	if (input > 1) {
		EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

	polling_flag = input;

    if(input)
        queue_delayed_work(edge_polling_workqueue,
                        &edge_polling_work, MCU_INIT_TIME);
    else
        cancel_delayed_work(&edge_polling_work);

	EDGE_INFO_LOG("%s: %s\n", __func__,
                        input ? "Start polling":"Stop polling");

	return count;
}

static ssize_t xr15510_override_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		EDGE_ERR_LOG("%s: Failed to get the inout value\n", __func__);
		return -EINVAL;
	}
	if (input > 2) {
		EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

	if (gpio_is_valid(gpio_override)) {
		switch (input) {
		case 0:
		case 1:
			EDGE_INFO_LOG("%s: gpio_override(%d) value was %d\n",
                    __func__, gpio_override, gpio_get_value(gpio_override));
			gpio_set_value(gpio_override, input);
			EDGE_INFO_LOG("%s: gpio_override(%d) value is %d\n",
                    __func__, gpio_override, gpio_get_value(gpio_override));
			break;
		case 2:
			EDGE_INFO_LOG("%s: gpio_override(%d) value was %d\n",
                    __func__, gpio_override, gpio_get_value(gpio_override));
			gpio_set_value(gpio_override, 1);
			EDGE_INFO_LOG("%s: gpio_override(%d) value is %d\n",
                    __func__, gpio_override, gpio_get_value(gpio_override));
			msleep(100);
			gpio_set_value(gpio_override, 0);
			EDGE_INFO_LOG("%s: gpio_override(%d) value is %d\n",
                    __func__, gpio_override, gpio_get_value(gpio_override));
			break;
		default:
			EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
			break;
		}
	}

	return count;
}

static ssize_t xr15510_select_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		EDGE_ERR_LOG("%s: Failed to get the inout value\n", __func__);
		return -EINVAL;
	}
	if (input > 2) {
		EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

    if (gpio_is_valid(gpio_sel)) {
        EDGE_INFO_LOG("%s: gpio_select(%d) value was %d\n",
                __func__, gpio_sel, gpio_get_value(gpio_sel));
        gpio_set_value(gpio_sel, input);
        EDGE_INFO_LOG("%s: gpio_select(%d) value is %d\n",
                __func__, gpio_sel, gpio_get_value(gpio_sel));
    }

	return count;
}

static ssize_t xr15510_pwren_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		EDGE_ERR_LOG("%s: Failed to get the inout value\n", __func__);
		return -EINVAL;
	}
	if (input > 2) {
		EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

    if (gpio_is_valid(gpio_power)) {
        EDGE_INFO_LOG("%s: gpio_power(%d) value was %d\n",
                __func__, gpio_power, gpio_get_value(gpio_power));
        gpio_set_value(gpio_power, input);
        EDGE_INFO_LOG("%s: gpio_power(%d) value is %d\n",
                __func__, gpio_power, gpio_get_value(gpio_power));
    }

	return count;
}

static ssize_t xr15510_gadc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", edge_gadc);
}

static ssize_t xr15510_gadc_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long input;

    if (kstrtoul(buf, 10, &input) != 0) {
        EDGE_ERR_LOG("%s: Failed to get the inout value\n", __func__);
        return -EINVAL;
    }
    if (input <= 0) {
        EDGE_ERR_LOG("%s: invalid parameter\n", __func__);
        return -EINVAL;
    }

    edge_gadc = input;

    return count;
}

static struct device_attribute attrs[] = {
	__ATTR(vendor, S_IRUGO,
			xr15510_vendor_show,
			NULL),
	__ATTR(gpio, S_IRUGO,
			xr15510_gpio_show,
			NULL),
	__ATTR(reset, S_IWUSR,
			NULL,
			xr15510_reset_store),
	__ATTR(data, S_IWUSR,
			NULL,
			xr15510_data_store),
	__ATTR(override, S_IWUSR,
			NULL,
			xr15510_override_store),
	__ATTR(select, S_IWUSR,
			NULL,
			xr15510_select_store),
	__ATTR(pwren, S_IWUSR,
			NULL,
			xr15510_pwren_store),
	__ATTR(gadc, S_IWUSR | S_IRUGO,
			xr15510_gadc_show,
			xr15510_gadc_store),
	__ATTR(hw_version, S_IRUGO,
			xr15510_hw_version_show,
			NULL),
};

static void xr15510_data_polling(struct work_struct *work)
{
	EDGE_INFO_LOG("%s: ++\n", __func__);

	while (1) {
		uint8_t report_count[3] = {0, 0, 0};
#ifdef DEBUG_LOG_ENABLE
		EDGE_INFO_LOG("%s: polling_flag = %d\n", __func__, polling_flag);
#endif
		if (polling_flag) {
			unsigned char read_count;
			unsigned char rx_buf[512], val, ret;
			short sensor_data[16];
			int i, ii;
			int re_try, time_out;
			int successful_flag;
#ifdef SENSOR_REVERSE
            int sensor_id;
#endif

#ifdef DEBUG_LOG_ENABLE
			EDGE_INFO_LOG("[AUZ] %s: polling...\n", __func__);
#else
			EDGE_INFO_LOG("%s++\n", __func__);
#endif
			mutex_lock(&polling_lock);

			re_try = 0;
			time_out = 0;
			ret = 1;
			val = 1;
			while((ret != 0)||(val != 0))
			{
//				ret = xrm117x_port_read(data_port, 0x10, &val, sizeof(val));
                ret = xr15510_i2c_read(xr15510_i2c_client, 0x10, &val, sizeof(val));
				//EDGE_INFO_LOG("xrm117x_port_read(0x10) ret = %d val=0x%02x\n", ret, val);
				if((re_try > 2)||(ret))
				{
					time_out =1;
					break;
				}
				else if ((ret != 0)||(val != 0))
				{
					msleep(100);
					re_try++;
				}
			}
			if(time_out)
			{
				if((val == 0x01)&&(ret == 0))
				{
					EDGE_ERR_LOG("Calibration is not completed\n");
					mutex_unlock(&polling_lock);
					continue;
				}
				else
				{
					EDGE_ERR_LOG("Get Channel Data Timeout\n");
					mutex_unlock(&polling_lock);
					continue;
				}
			}

			if((ret == 0)&&(val == 0))
			{
//				ret = xrm117x_port_read(data_port, 0x7c, &read_count, sizeof(read_count));
                ret = xr15510_i2c_read(xr15510_i2c_client, 0x7c, &read_count, sizeof(read_count));
				if((ret)||(read_count > 128))
				{
					EDGE_ERR_LOG("Failed to read fifo count ret =%d read_count =0x%02x\n", ret, read_count);
					mutex_unlock(&polling_lock);
					continue;
				}
				//printk("fifo count =%d \n",read_count);
				if(read_count == 0)
				{  //nothing to do if we get the 0 length data
					mutex_unlock(&polling_lock);
					continue;
				}

				//show_vid_pid(data_port);
				successful_flag = 1;

//				ret = xrm117x_port_read(data_port, 0x7f, rx_buf, sizeof(uint8_t) * read_count);
                ret = xr15510_i2c_read(xr15510_i2c_client, 0x7f, rx_buf, sizeof(uint8_t) * read_count);
				if(ret)
				{
					EDGE_ERR_LOG("Read I/O error[%d]\n",ret);
					successful_flag = 0;
				}

				 if(successful_flag)
				 {
					for (i = 0; i < read_count / 37; i++)
					{
						i = read_count/37 - 1;
//						EDGE_INFO_LOG("%s: data[%d-0]: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", __func__, i, rx_buf[i*37+0], rx_buf[i*37+1], rx_buf[i*37+2], rx_buf[i*37+3], rx_buf[i*37+4]);
						for(ii = 5 ; ii < 37; ii += 2) {
#ifdef SENSOR_REVERSE
                            /* Swap */
                            if(ii==7) ii=9;
                            else if(ii==9) ii=7;
#endif

                            /* gain correction on delta and keep the rest the same*/
                            if(!((ii-3)%4)) {
                                sensor_data[(ii-3)/2] = ((rx_buf[i*37+ii] << 8) & 0xFF00) | (rx_buf[i*37+ii+1] & 0xFF);
                                //EDGE_INFO_LOG("[AUZ] id[%d] orig: %i++\n", (ii-7)/4, sensor_data[(ii-3)/2]);
                                if (gadc_calibrated)
                                    sensor_data[(ii-3)/2] = sensor_data[(ii-3)/2] * gadc / gain_1[(ii-7)/4]; //gain_den[(ii-7)/4];
                                //EDGE_INFO_LOG("[AUZ] id[%d] corc: %i--\n", (ii-7)/4, sensor_data[(ii-3)/2]);
                            } else {
                               sensor_data[(ii-3)/2] = ((rx_buf[i*37+ii] << 8) & 0xFF00) | (rx_buf[i*37+ii+1] & 0xFF);
                            }

//							EDGE_INFO_LOG("%s: data[%d-%d]: 0x%02x, 0x%02x -> %d\n", __func__, i, (ii-3)/2, rx_buf[i*37+ii], rx_buf[i*37+ii+1], sensor_data[(ii-3)/2]);
//
							if ((ii <= 19) && !((ii-3)%4) && sensor_data[(ii-3)/2] >= report_threshold) {//REPORT_DATA_THRESHOLD) {
#ifdef DEBUG_LOG_ENABLE
                                EDGE_INFO_LOG("%s: report delta[%d] = %d on X-%d, Y-%d\n", __func__, (ii-7)/4, sensor_data[(ii-3)/2], SENSOR_X2_LEFT, 2000 + (ii-7)/4 * 100);
#endif

                                /* gain correction on delta */
#ifdef SENSOR_REVERSE
                                sensor_id = (ii-7)/4;

                                if (sensor_id == 1) {
                                    input_mt_slot(input_dev, 2);
                                    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

                                    input_report_abs(input_dev, ABS_MT_POSITION_X, SENSOR_X1_RIGHT);
                                    input_report_abs(input_dev, ABS_MT_POSITION_Y, 2000);
                                } else if (sensor_id == 2) {
                                    input_mt_slot(input_dev, 1);
                                    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

                                    input_report_abs(input_dev, ABS_MT_POSITION_X, SENSOR_X1_RIGHT);
                                    input_report_abs(input_dev, ABS_MT_POSITION_Y, 1900);
                                } else {
#endif
                                    input_mt_slot(input_dev, (ii-7)/4);
                                    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

                                    input_report_abs(input_dev, ABS_MT_POSITION_X, SENSOR_X1_RIGHT);
                                    input_report_abs(input_dev, ABS_MT_POSITION_Y, 1800 + (ii-7)/4 * 100);
#ifdef SENSOR_REVERSE
                                }
#endif
                                input_report_abs(input_dev, ABS_MT_PRESSURE, sensor_data[(ii-3)/2]);
                                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 1);
                                input_report_abs(input_dev, ABS_MT_TOUCH_MINOR, 1);
                                report_count[i]++;
							} else if ((ii > 19) && !((ii-3)%4) && (sensor_data[(ii-3)/2] <= -report_threshold)) {//-REPORT_DATA_THRESHOLD)) {
#ifdef DEBUG_LOG_ENABLE
								EDGE_INFO_LOG("%s: report delta[%d] = %d on X-%d, Y-%d\n", __func__, (ii-7)/4, -sensor_data[(ii-3)/2], SENSOR_X1_RIGHT, 2000 + (ii-7)/4 * 100);
#endif
								input_mt_slot(input_dev, (ii-7)/4);
								input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

								input_report_abs(input_dev, ABS_MT_POSITION_X, SENSOR_X2_LEFT);
								input_report_abs(input_dev, ABS_MT_POSITION_Y, 1400 + (ii-7)/4 * 100);
								input_report_abs(input_dev,	ABS_MT_PRESSURE, -sensor_data[(ii-3)/2]);
								input_report_abs(input_dev,	ABS_MT_TOUCH_MAJOR, 1);
								input_report_abs(input_dev, ABS_MT_TOUCH_MINOR, 1);

								report_count[i]++;
							}
							else if (!((ii-3)%4)) {
#ifdef DEBUG_LOG_ENABLE
								EDGE_INFO_LOG("%s: XXXXXX delta[%d] = %d\n", __func__, (ii-7)/4, sensor_data[(ii-3)/2]);
#endif
								input_mt_slot(input_dev, (ii-7)/4);
								input_mt_report_slot_state(input_dev,
										MT_TOOL_FINGER, false);
							}
#ifdef SENSOR_REVERSE
                            /* Reset the Swap */
                            if(ii==7) ii=9;
                            else if(ii==9) ii=7;
#endif
						}
						for (ii = 8; ii < NUM_OF_SENSORS; ii++)
						{
//							EDGE_INFO_LOG("%s: XXXXXX no-data[%d]\n", __func__, ii);
							input_mt_slot(input_dev, ii);
							input_mt_report_slot_state(input_dev,
									MT_TOOL_FINGER, false);
						}
						input_sync(input_dev);
					}
				}
				else
				{
					EDGE_ERR_LOG("Failed to run xrm117x_port_read\n");
					mutex_unlock(&polling_lock);
					continue;
				}
			}
			else
			{
				EDGE_ERR_LOG("I/O Error OR Devices is Busy ret = %02x val=%02x\n", ret, val);
				mutex_unlock(&polling_lock);
				continue;
			}
			mutex_unlock(&polling_lock);
		}
#ifdef DEBUG_LOG_ENABLE
		EDGE_INFO_LOG("[AUZ] %s: DONE, report_count = %d, %d, %d\n", __func__, report_count[0], report_count[1], report_count[2]);
#else
		EDGE_INFO_LOG("%s--\n", __func__);
#endif
		msleep(50);
	}

//	queue_delayed_work(edge_polling_workqueue,
//					&edge_polling_work, 1000);
}

#define XR_USB_SERIAL_IOC_MAGIC       	'v'
#define XRIOC_GET_REG           	_IOWR(XR_USB_SERIAL_IOC_MAGIC, 1, int)
#define XRIOC_SET_REG           	_IOWR(XR_USB_SERIAL_IOC_MAGIC, 2, int)
#define XRIOC_READ_CHANNEL_DATA     _IOWR(XR_USB_SERIAL_IOC_MAGIC, 3, int)

static long xr15510_ioctl(struct file *port, unsigned int cmd,
			 unsigned long arg)
{
	unsigned int reg_addr, reg_value;
	unsigned char read_count;
	unsigned char tx_buf, rx_buf[512], val = 0, ret;
	int result;
	int re_try, time_out;
	int successful_flag;

	EDGE_INFO_LOG("%s: ++, cmd = %u\n", __func__, cmd);

	//struct xr10950_read_info Read_info;
	switch (cmd) {
	case XRIOC_GET_REG:
		EDGE_INFO_LOG("%s: XRIOC_GET_REG\n", __func__);
		if (get_user(reg_addr, (int __user *)arg))
			return -EFAULT;
//		result = xrm117x_port_read(port, reg_addr, &val, sizeof(val));
        result = xr15510_i2c_read(xr15510_i2c_client, reg_addr, &val, sizeof(val));
		if (result == 0) {
			reg_value = val;
			if (put_user(reg_value, (int __user *)(arg + sizeof(int)))) {
				return -EFAULT;
			}
		} else {
			return -EFAULT;
		}
		EDGE_INFO_LOG("%s: --\n", __func__);
		return 1;
		break;

	case XRIOC_SET_REG:
		EDGE_INFO_LOG("%s: XRIOC_SET_REG\n", __func__);
		if (get_user(reg_addr, (int __user *)arg)) {
			return -EFAULT;
		}
		if (get_user(reg_value, (int __user *)(arg + sizeof(int)))) {
			return -EFAULT;
		}
		tx_buf = reg_value & MASK_8BIT;
//		xrm117x_port_write(port, (u8)reg_addr, &tx_buf, sizeof(tx_buf));
        xr15510_i2c_write(xr15510_i2c_client, (u8)reg_addr, &tx_buf, sizeof(tx_buf));
		EDGE_INFO_LOG("%s: -- addr: 0x%02X, cmd:0x%02X\n", __func__, reg_addr, tx_buf);
		return 1;
		break;

	case XRIOC_READ_CHANNEL_DATA:
		EDGE_INFO_LOG("%s: XRIOC_READ_CHANNEL_DATA\n", __func__);
		mutex_lock(&polling_lock);
		re_try = 0;
		time_out = 0;
		ret = 1;
		val = 1;
		while ((ret != 0) || (val != 0)) {
//			ret = xrm117x_port_read(port, 0x10, &val, sizeof(val));
            ret = xr15510_i2c_read(xr15510_i2c_client, 0x10, &val, sizeof(val));
			EDGE_INFO_LOG("xrm117x_port_read(0x10) ret = %d val=0x%02x\n", ret, val);
			if ((re_try > 2) || (ret)) {
				time_out = 1;
				break;
			} else if ((ret != 0) || (val != 0)) {
				msleep(100);
				re_try++;
			}
		}
		if (time_out) {
			if ((val == 0x01) && (ret == 0)) {
				EDGE_ERR_LOG("Calibration is not completed!\n");
				mutex_unlock(&polling_lock);
				return  -135;
			} else {
				EDGE_ERR_LOG("Ioctl Get Channel Data Timeout\n");
				mutex_unlock(&polling_lock);
				return -EFAULT;
			}
		}
		if ((ret == 0) && (val == 0)) {
//			ret = xrm117x_port_read(port, 0x7c, &read_count, sizeof(read_count));
            ret = xr15510_i2c_read(xr15510_i2c_client, 0x7c, &read_count, sizeof(read_count));
			if ((ret) || (read_count > 128)) {
				EDGE_ERR_LOG("Failed to read fifo count ret =%d, read_count =0x%02x\n", ret, read_count);
				mutex_unlock(&polling_lock);
				return  -EFAULT;
			}
			EDGE_INFO_LOG("fifo count = %d\n", read_count);
			if (read_count == 0) {
				//nothing to do if we get the 0 length data
				EDGE_ERR_LOG("%s: --read_count = %d\n", __func__, read_count);
				mutex_unlock(&polling_lock);
				return -EAGAIN;
			}

			successful_flag = 1;
//			ret = xrm117x_raw_read(port, rx_buf, read_count);
            ret = xr15510_i2c_read(xr15510_i2c_client, 0x7f, rx_buf, read_count);
			if (ret) {
				EDGE_ERR_LOG("Read I/O error[%d]\n", ret);
				successful_flag = 0;
			}

			if (successful_flag) {
				/*
				printk("DriverReceiveData:");
				for (i = 0; i < read_count; i++) {
					if ((i % 16) == 0) printk("\n");
					printk("%d ", rx_buf[i]);
				}
				*/

                if(gadc_calibrated) {
                    int fifo_cnt = 0;
                    int sensor_cnt = 0;
                    short temp = 0;

                    for (fifo_cnt = 0; fifo_cnt < read_count/37; fifo_cnt++) {
                        for (sensor_cnt = 0; sensor_cnt < 8; sensor_cnt++) {
                            temp = ((rx_buf[fifo_cnt*37+5+sensor_cnt*4+2] << 8) & 0xFF00) | (rx_buf[fifo_cnt*37+5+sensor_cnt*4+3] & 0xFF);
                            temp = temp * gadc / gain_1[sensor_cnt];
                            rx_buf[fifo_cnt*37+5+sensor_cnt*4+2] = (uint8_t)((temp & 0xFF00) >> 8);
                            rx_buf[fifo_cnt*37+5+sensor_cnt*4+3] = (uint8_t)(temp & 0xFF);
                        }
                    }
                }

				if (copy_to_user((int __user *)(arg + sizeof(int)), rx_buf, sizeof(unsigned char) * read_count)) {
					EDGE_ERR_LOG("Failed to copy_to_user\n");
					mutex_unlock(&polling_lock);
					return -EFAULT;
				}
				EDGE_INFO_LOG("%s: -- read_count = %d\n", __func__, read_count);
				mutex_unlock(&polling_lock);
				return read_count;
			} else {
				EDGE_ERR_LOG("Failed to run xrm117x_port_read\n");
				mutex_unlock(&polling_lock);
				return -EFAULT;
			}
		} else {
			EDGE_ERR_LOG("I/O Error OR Devices is Busy ret = %02x val=%02x\n", ret, val);
			mutex_unlock(&polling_lock);
			return -EFAULT;
		}
		mutex_unlock(&polling_lock);
		break;
	default:
		EDGE_ERR_LOG("switch - default\n");
		break;
	}

	EDGE_ERR_LOG("%s: --ENOIOCTLCMD (%d)\n", __func__, ENOIOCTLCMD);

	return -ENOIOCTLCMD;
}

static void xr15510_parse_dt(struct device *dev)
{
	uint32_t check_sensor_profile = 0;
	uint8_t engineer_id = 0;
	struct device_node *node = dev->of_node;
	struct device_node *mfgnode;

    /* reset pin */
	if (of_find_property(node, "reset-gpio", NULL)) {
		gpio_reset = of_get_named_gpio(node, "reset-gpio", 0);
		EDGE_INFO_LOG("%s: gpio_reset: %d\n", __func__, gpio_reset);
	} else
		EDGE_INFO_LOG("%s: Unable to parse reset-gpio\n", __func__);

    /* interrupt pin */
	if (of_find_property(node, "irq-gpio", NULL)) {
		gpio_int = of_get_named_gpio(node, "irq-gpio", 0);
		EDGE_INFO_LOG("%s: gpio_intr: %d\n", __func__, gpio_int);
	} else
		EDGE_INFO_LOG("%s: Unable to parse irq-gpio\n", __func__);

    /* boot-override pin */
	if (of_find_property(node, "override-gpio", NULL)) {
		gpio_override = of_get_named_gpio(node, "override-gpio", 0);
		EDGE_INFO_LOG("%s: gpio_override: %d\n", __func__, gpio_override);
	} else
		EDGE_INFO_LOG("%s: Unable to parse override-gpio\n", __func__);

    /* switch-select pin */
	if (of_find_property(node, "sel-gpio", NULL)) {
		gpio_sel = of_get_named_gpio(node, "sel-gpio", 0);
		EDGE_INFO_LOG("%s: gpio_select: %d\n", __func__, gpio_sel);
	} else
		EDGE_INFO_LOG("%s: Unable to parse sel-gpio\n", __func__);

    /* power 3v3 pin */
	if (of_find_property(node, "pwren-gpio", NULL)) {
		gpio_power = of_get_named_gpio(node, "pwren-gpio", 0);
		EDGE_INFO_LOG("%s: gpio_power: %d\n", __func__, gpio_power);
	} else
		EDGE_INFO_LOG("%s: Unable to parse pwren-gpio\n", __func__);

    /* check whether to switch reset pin  */
    if (of_find_property(node, "switch_reset", NULL)) {
        of_property_read_u32(node, "switch_reset", &switch_reset);
        EDGE_INFO_LOG("%s: switch_reset = %u\n", __func__, switch_reset);
    } else
        EDGE_INFO_LOG("%s: default switch_reset = %u\n", __func__, switch_reset);

    /* project id */
    if (of_find_property(node, "project_id", NULL)) {
        of_property_read_u32(node, "project_id", &project_id);
        EDGE_INFO_LOG("%s: project_id: %d\n", __func__, project_id);
    } else
        EDGE_INFO_LOG("%s: Unable to parse project_id\n", __func__);

    /* edge golden adc */
    if (of_find_property(node, "edge_gadc", NULL)) {
        of_property_read_u32(node, "edge_gadc", &edge_gadc);
        EDGE_INFO_LOG("%s: edge_gadc: %d\n", __func__, edge_gadc);
    } else
        EDGE_INFO_LOG("%s: Unable to parse edge_gadc\n", __func__);

    /* check sensor profile */
    if (of_find_property(node, "check_sensor_profile", NULL)) {
        of_property_read_u32(node, "check_sensor_profile", &check_sensor_profile);
        EDGE_INFO_LOG("%s: check_sensor_profile: %d\n", __func__, check_sensor_profile);
    } else
        EDGE_INFO_LOG("%s: Unable to parse check_sensor_profile\n", __func__);

    /* check sensor profile to dynamic select the gadc */
    if(check_sensor_profile) {
        mfgnode = of_find_node_by_path("/chosen/mfg");
        if (mfgnode) {
            if (of_property_read_u8(mfgnode, "skuid.engineer_id", &engineer_id))
                EDGE_INFO_LOG("%s: Failed to get property: engineer_id\n", __func__);
        } else {
            EDGE_INFO_LOG("%s: Failed to find mfg device node\n", __func__);
        }
        EDGE_INFO_LOG("%s: engineer_id = %u\n", __func__, engineer_id);

        if(engineer_id == 0) {
            p2_sensor_module = 1;
            edge_gadc = 50;
            hw_id = OCE_P1P2_SENSOR;
        }
    }
}

static int xr15510_pinctrl_init(struct device *dev)
{
	int retval = 0;

	/* Get pinctrl if target uses pinctrl */
	edge_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(edge_pinctrl)) {
		EDGE_INFO_LOG(
			"Target does not use pinctrl\n");
		retval = PTR_ERR(edge_pinctrl);
		edge_pinctrl = NULL;
		return retval;
	}

	gpio_state_active
		= pinctrl_lookup_state(edge_pinctrl, "edge_mux_active");
	if (IS_ERR_OR_NULL(gpio_state_active)) {
		EDGE_DBG_LOG(
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(gpio_state_active);
		edge_pinctrl = NULL;
		return retval;
	}

	gpio_state_suspend
		= pinctrl_lookup_state(edge_pinctrl, "edge_mux_suspend");
	if (IS_ERR_OR_NULL(gpio_state_suspend)) {
		EDGE_DBG_LOG(
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(gpio_state_suspend);
		edge_pinctrl = NULL;
		return retval;
	}

	return retval;
}

static void xr15510_pinctrl_deinit(void)
{
	/* Put pinctrl if target uses pinctrl */
	if (edge_pinctrl != NULL) {
		gpio_state_active = NULL;
		gpio_state_suspend = NULL;
		devm_pinctrl_put(edge_pinctrl);
	}
}

static int xr15510_pinctrl_select(bool on)
{
	struct pinctrl_state *pins_state;
	int ret = 0;

	pins_state = on ? gpio_state_active
		     : gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(edge_pinctrl, pins_state);
		if (ret) {
			EDGE_INFO_LOG(
				"can not set %s pins\n",
				on ? "edge_mux_active" : "edge_mux_suspend");
			return ret;
		}
	} else
		EDGE_ERR_LOG(
			"not a valid '%s' pinstate\n",
			on ? "edge_mux_active" : "edge_mux_suspend");

	return ret;
}

static int xr15510_set_gpio(struct device *dev)
{
	int rc = 0;

	if (gpio_is_valid(gpio_power)) {
		rc = gpio_request(gpio_power, "xrm117x_gpio_power");
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_request failed for "
                                "gpio_power\n", __func__);
			return rc;
		}

		rc = gpio_direction_output(gpio_power, 1);
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_direction_output failed "
                                "for gpio_power\n", __func__);
			gpio_free(gpio_power);
			return rc;
		}

		EDGE_INFO_LOG("%s: gpio_power: %d(%d)\n", __func__,
                        gpio_power, gpio_get_value(gpio_power));
	}

	if (gpio_is_valid(gpio_reset)) {
		rc = gpio_request(gpio_reset, "xrm117x_gpio_reset");
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_request failed for "
                                "gpio_reset\n", __func__);
			return rc;
		}

		rc = gpio_direction_output(gpio_reset, 0);
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_direction_output failed "
                                "for gpio_reset\n", __func__);
			gpio_free(gpio_reset);
			return rc;
		}

		EDGE_INFO_LOG("%s: gpio_reset: %d(%d)\n", __func__,
                        gpio_reset, gpio_get_value(gpio_reset));
	}

	if (gpio_is_valid(gpio_int)) {
		rc = gpio_request(gpio_int, "xrm117x_gpio_int");
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_request failed for "
                                "gpio_int\n", __func__);
			return rc;
		}

		rc = gpio_direction_input(gpio_int);
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_direction_output failed "
                                "for gpio_int\n", __func__);
			gpio_free(gpio_int);
			return rc;
		}

		EDGE_INFO_LOG("%s: gpio_intr:  %d(%d)\n", __func__,
                        gpio_int, gpio_get_value(gpio_int));
	}

	if (gpio_is_valid(gpio_override)) {
		rc = gpio_request(gpio_override, "xrm117x_gpio_override");
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_request failed for "
                                "gpio_override\n", __func__);
			return rc;
		}

        rc = gpio_direction_output(gpio_override, 0);
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_direction_output failed for "
                                "gpio_override\n", __func__);
			gpio_free(gpio_override);
			return rc;
		}

		EDGE_INFO_LOG("%s: gpio_ovrde: %d(%d)\n", __func__,
                    gpio_override, gpio_get_value(gpio_override));
	}

	if (gpio_is_valid(gpio_sel)) {
		rc = gpio_request(gpio_sel, "xrm117x_gpio_sel");
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_request failed for "
                                "gpio_sel\n", __func__);
			return rc;
		}

		rc = gpio_direction_output(gpio_sel, 0);
		if (rc) {
			EDGE_ERR_LOG("%s: gpio_direction_output failed "
                                "for gpio_sel\n", __func__);
			gpio_free(gpio_sel);
			return rc;
		}

		EDGE_INFO_LOG("%s: gpio_sel: %d(%d)\n", __func__,
                        gpio_sel, gpio_get_value(gpio_sel));
	}

	return rc;
}

static int xr15510_hw_reset(void)
{
	int ret;

	EDGE_INFO_LOG("%s++\n", __func__);
//	Send a hardware reset. Set nReset pin low of at least 2.5 us
	ret = gpio_direction_output(gpio_reset, GPIO_HIGH);
	mdelay(20);
	ret |= gpio_direction_output(gpio_reset, GPIO_LOW);
	if (ret) {
		EDGE_ERR_LOG("%s: gpio_direction_output failed for gpio_reset(%d).\n", __func__, ret);
		return -EFAULT;
	}
	mdelay(1000);

	EDGE_INFO_LOG("%s--\n", __func__);

	return 0;
}

static int xr15510_create_file_sys(struct i2c_client *client)
{
	int ret = 0, attr_count;

	if (android_edge_kobj == NULL)
		android_edge_kobj = kobject_create_and_add("android_edge", NULL);
	if (android_edge_kobj == NULL) {
		EDGE_ERR_LOG("%s: kobject_create_and_add failed\n", __func__);
		return -ENOMEM;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		ret = sysfs_create_file(android_edge_kobj,
					&attrs[attr_count].attr);
		if (ret < 0) {
			EDGE_ERR_LOG("%s: Failed to create sysfs attributes\n", __func__);
			return ret;
		}
	}

//	ret = sysfs_create_link(NULL, &client->dev.kobj, "android_edge");
//	if (ret < 0) {
//		EDGE_ERR_LOG("%s: subsystem_register failed\n", __func__);
//		return ret;
//	}

	return ret;
}

static void xr15510_set_input_params(struct device *dev)
{
	EDGE_INFO_LOG("%s: ++\n", __func__);

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X,
			-2, 1440, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y,
			-2, 2560, 0, 0);

	input_set_abs_params(input_dev,
			ABS_MT_PRESSURE,
			0, 255, 0, 0);

	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR,
			0, 255, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MINOR,
			0, 255, 0, 0);

#ifdef TYPE_B_PROTOCOL
	input_mt_destroy_slots(input_dev);
	input_mt_init_slots(input_dev, NUM_OF_SENSORS, 0);
#endif

	return;
}

static int xr15510_set_input_dev(struct device *dev)
{
	int retval;

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		EDGE_ERR_LOG("%s: Failed to allocate input device\n", __func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

	input_dev->name = XR15510_NAME;
	input_dev->phys = INPUT_PHYS_NAME;
	input_dev->id.product = 0x1;
	input_dev->id.version = 0x1;
	input_dev->dev.parent = dev->parent;
	input_set_drvdata(input_dev, dev->platform_data);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	xr15510_set_input_params(dev);

	retval = input_register_device(input_dev);
	if (retval) {
		EDGE_ERR_LOG("%s: Failed to register input device\n", __func__);
		goto err_register_input;
	}

	return 0;

err_register_input:
//	synaptics_rmi4_empty_fn_list(rmi4_data);
//	input_free_device(input_dev);

err_input_device:
	return retval;
}

static int check_fw_version(void)
{
	int ret = 0;
	uint8_t fw_ver[ACK_LEN * 2] = {0x00};

	ret = xr15510_i2c_read(xr15510_i2c_client,
			XR15510_FW_VERSION, fw_ver, sizeof(fw_ver));
	if (ret) {
		EDGE_ERR_LOG("read fw_ver fail, ret = %d\n", ret);
		return ret;
	}

	EDGE_INFO_LOG("%s: Firmware version: 0x%02X\n", __func__, fw_ver[0]);
	return fw_ver[0];
}

static const struct file_operations xr15510_fops = {
    .owner          =   THIS_MODULE,
    .unlocked_ioctl =   xr15510_ioctl,
    //.ioctl        =   xr15510_ioctl,
    //.open         =   xrm117x_open,
    //.flush        =   xrm117x_flush,
};

static int xr15510_create_file_ioctl(struct xr15510_data *edge_data)
{
    int ret = 0;

    edge_data->miscdev.minor = MISC_DYNAMIC_MINOR;
    edge_data->miscdev.name = "ttyXRM0";
    edge_data->miscdev.fops = &xr15510_fops;
    if (misc_register(&edge_data->miscdev) != 0)
        EDGE_ERR_LOG("%s: Failed to register misc dev for edge sensor\n", __func__);

	return ret;
}

static int xr15510_remove(struct device *dev)
{
	return 0;
}

/***************************************************************
 ************************ Probe ********************************
 ***************************************************************/
static int xr15510_i2c_probe(struct i2c_client *i2c,
	                 const struct i2c_device_id *id)
{
	int ret = 0;
    struct xr15510_data *edge_data = NULL;

    EDGE_INFO_LOG("%s: Enter++\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		EDGE_ERR_LOG("%s: i2c check functionality error\n", __func__);
		return -EIO;
	}

    /* Allocate memory for device data*/
    edge_data = kzalloc(sizeof(struct xr15510_data), GFP_KERNEL);
    if(!edge_data) {
		EDGE_ERR_LOG("%s: alloc memory for device data error\n", __func__);
        return -ENOMEM;
    }

    /* Initialize i2c(0x4a) at global*/
    edge_data->client = i2c;
	xr15510_i2c_client = i2c;

    /* Initialize ioctrl */
    ret = xr15510_create_file_ioctl(edge_data);

    /* Parse device tree*/
	/*ret=*/xr15510_parse_dt(&i2c->dev);

    /* Initialize pinctrl state*/
	ret = xr15510_pinctrl_init(&i2c->dev);
	if (!ret && (edge_pinctrl != NULL)) {
		ret = xr15510_pinctrl_select(true);
		if (ret < 0)
			goto err_config_gpio;
	}

    /* Initialize gpio status */
	/*ret=*/xr15510_set_gpio(&i2c->dev);

    /* Initialize input sub-system */
	/*ret=*/xr15510_set_input_dev(&i2c->dev);

    /* Initialize attribute files */
	/*ret=*/xr15510_create_file_sys(i2c);

    /* Initialize metex */
	mutex_init(&polling_lock);

    /* Initialize work/workqueue */
	edge_polling_workqueue = create_singlethread_workqueue("edge_polling_wq");
    fw_workqueue = create_singlethread_workqueue("fw_workqueue");

	INIT_DELAYED_WORK(&edge_polling_work, xr15510_data_polling);
    INIT_WORK(&fw_chk_work, edge_fw_check_work);
    INIT_WORK(&fw_upd_work, edge_fw_update_work);

    /* For temporary test only, since now the data reporting
     * is doing at nanohub side, so comment out the polling
	queue_delayed_work(edge_polling_workqueue,
			&edge_polling_work, MCU_INIT_TIME);
    */

    /* Queue out to read firmware version */
    queue_work(fw_workqueue, &fw_chk_work);

    EDGE_INFO_LOG("%s: End--\n", __func__);

    return ret;
err_config_gpio:
    kfree(edge_data);
	return ret;
}

static int xr15510_i2c_remove(struct i2c_client *client)
{
	xr15510_pinctrl_deinit();
	return xr15510_remove(&client->dev);
}

static const struct i2c_device_id xr15510_i2c_id_table[] = {
	{ XR15510_NAME,	0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, xr15510_i2c_id_table);

#ifdef CONFIG_OF
static struct of_device_id xr15510_i2c_of_match_table[] = {
	{.compatible = "exar,xr15510",},
	{},
};
MODULE_DEVICE_TABLE(of, xr15510_i2c_of_match_table);
#else
#define xr15510_i2c_of_match_table NULL
#endif

static struct i2c_driver xr15510_i2c_edge_driver = {
	.driver = {
		.name		= XR15510_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = xr15510_i2c_of_match_table,
	},
	.probe		= xr15510_i2c_probe,
	.remove		= xr15510_i2c_remove,
	.id_table	= xr15510_i2c_id_table,
};


/**********************************************************
 ******************* Firmware update **********************
 **********************************************************/

#ifdef CONFIG_EDGE_SENSOR_FW_UPDATE

#define XR15510_FLASH_NAME "xr15510_flash"
#define XR15510_FW_NAME    "edge_xr15510.img"
#define HEADER_LEN	28
#define DATA_LEN	95
#define SIZE_LEN	4
#define TX_LEN		DATA_LEN + SIZE_LEN
#define DEF_RETRY	10	/* times */
#define FW_HEADER_LENGTH	16

typedef enum {
	ACK_READY,
	ACK_COMPLETE
} ACK_TYPE;

static uint32_t debug_mask = 0x1;
uint8_t *fw_data_start;
uint32_t fw_size;
atomic_t in_flash = ATOMIC_INIT(0);
const struct firmware *fw;
static DEFINE_MUTEX(xr15510_i2c_access_lock);
struct wake_lock fw_wake_lock;

static int xr15510_i2c_read(struct i2c_client *client, u8 reg, u8 *val, unsigned short length)
{
	struct i2c_msg xfer[2];
//	u8 reg_value = 0;
	u8 cmd = (reg << 0);
	int ret;
//	*val = 0;
	mutex_lock(&xr15510_i2c_access_lock);
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &cmd;

	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = length;
	xfer[1].buf = val;

	ret = i2c_transfer(client->adapter, xfer, 2);
	mutex_unlock(&xr15510_i2c_access_lock);
	if (ret == 2) {
		//*val = reg_value;
		return 0;
	} else {
		EDGE_ERR_LOG("%s: Failed <%d>\n", __func__, ret);
		return -1;
	}
}

static int xr15510_i2c_write(struct i2c_client *client, u8 reg, u8 *data, unsigned short length)
{
	unsigned char i2c_buf[length + 1];
	int		ret;
	i2c_buf[0] = reg;
//	i2c_buf[1] = data;
	memcpy(&i2c_buf[1], &data[0], length);

	mutex_lock(&xr15510_i2c_access_lock);
	ret = i2c_master_send(client, i2c_buf, length + 1);
	mutex_unlock(&xr15510_i2c_access_lock);
	if (ret == (length + 1))
		return 0;
	else if (ret < 0) {
		EDGE_ERR_LOG("%s: Failed <%d>\n", __func__, ret);
		return ret;
	} else {
		return -EIO;
	}
}

uint32_t calCRC32(unsigned char *data, uint16_t length)
{
	struct timespec time_start, time_end, time_delta;
	const uint32_t poly32 = 0x1EDC6F41;
	uint32_t rem = 0;
	uint16_t i;
	uint8_t  j;

	if (debug_mask & BIT(1)) {
		getnstimeofday(&time_start);
	}

	for (i = 0; i < length; i++) {
		rem = rem ^ (*(data + i) << 24);

		for (j = 0; j < 8; j++) {
			if (rem & 0x80000000)
				rem = (rem << 1) ^ poly32;
			else
				rem = rem << 1;

			rem = rem & 0xFFFFFFFF;
		}
	}

	if (debug_mask & BIT(1)) {
		getnstimeofday(&time_end);
		time_delta.tv_nsec = (time_end.tv_sec * 1000000000 + time_end.tv_nsec)
				     - (time_start.tv_sec * 1000000000 + time_start.tv_nsec);
		EDGE_INFO_LOG("CRC calculation time = %ld us, ret = %08X\n", time_delta.tv_nsec / 1000, rem);
	}

	EDGE_INFO_LOG("%s: 0x%08X\n", __func__, rem);
	return rem;
}

static uint8_t bootloader_procedure_control(uint8_t enable)
{
	int ret;

	EDGE_INFO_LOG("%s: %d\n", __func__, enable);
//	Set Boot-Override pin
	ret = gpio_direction_output(gpio_override, enable);
	if (ret) {
		EDGE_ERR_LOG("%s: gpio_direction_output failed for gpio_override,(%d).\n", __func__, ret);
		return false;
	}

	mdelay(100);

//	Send a hardware reset. Set nReset pin low of at least 2.5 us
	ret |= gpio_direction_output(gpio_reset, GPIO_HIGH);
	mdelay(20);
	ret |= gpio_direction_output(gpio_reset, GPIO_LOW);
	if (ret) {
		EDGE_ERR_LOG("%s: gpio_direction_output failed for gpio_reset(%d).\n", __func__, ret);
		return false;
	}

	return true;
}

static uint8_t checkIntStatus(uint8_t state, uint32_t delay, uint8_t retry)
{
	uint8_t i;

	if (debug_mask & BIT(1)) {
		EDGE_INFO_LOG("%s\n", __func__);
	}
	for (i = 0; i < retry; i++) {
		mdelay(delay);
		if (gpio_get_value(gpio_int) == state) {
			if (debug_mask & BIT(1)) {
				EDGE_INFO_LOG("%s: gpio_int state MATCH %s.\n", __func__, state?"HIGH":"LOW");
			}
			return true;
		} else
			EDGE_INFO_LOG("%s: gpio_int state unmatch, waiting for %s (try: %d)\n", __func__, state?"HIGH":"LOW", i+1);
	}

	EDGE_ERR_LOG("%s: --failed--\n", __func__);
	return false;
}

static uint8_t checkACK(ACK_TYPE type_ACK)
{
	const unsigned char ack_1[ACK_LEN] 		= {0x00, 0x00, 0x00, 0x00};
	const unsigned char ack_2[ACK_LEN] 		= {0x02, 0x00, 0x00, 0x00};
	const unsigned char ack_complete[ACK_LEN]	= {0x03, 0x00, 0x00, 0x00};
	unsigned char data[ACK_LEN] = {0xFF};
	int ret;

	ret = xr15510_i2c_read(xr15510_i2c_flash_client, 0x00, data, sizeof(data));
	if (ret) {
		EDGE_ERR_LOG("read ACK reg fail, ret = %d\n", ret);
		return  -EFAULT;
	}

	ret = false;
	if (type_ACK == ACK_READY) {
		if (strncmp(ack_1, data, ACK_LEN) != 0
		    && strncmp(ack_2, data, ACK_LEN) != 0) {
			EDGE_ERR_LOG("%s: status = 0x%02x%02x%02x%02x\n",
				     __func__, data[0], data[1], data[2], data[3]);
		} else {
			if (debug_mask & BIT(1)) {
				EDGE_INFO_LOG("%s: ACK_READY\n", __func__);
			}
			ret = true;
		}
	} else if (type_ACK == ACK_COMPLETE) {
		if (strncmp(ack_complete, data, ACK_LEN) != 0) {
			EDGE_ERR_LOG("%s: status = 0x%02x%02x%02x%02x\n",
					__func__, data[0], data[1], data[2], data[3]);
		} else {
			EDGE_INFO_LOG("%s: ACK_COMPLETE\n", __func__);
			ret = true;
		}
	} else {
		EDGE_ERR_LOG("%s: Unsupport checking type!\n", __func__);
	}

	return ret;
}

uint8_t handshaking(ACK_TYPE type_ACK)
{
	u8 cmd_ack[CMD_LEN] = {0x00, 0x00, 0x00, 0x00};
	int ret = 0;

	if (debug_mask & BIT(1)) {
		EDGE_INFO_LOG("%s: +++\n", __func__);
	}

//	On entering bootloader/writing command or packet, the interrupt pin will be toggled LOW.
//	Read ACK REG (address 0x00, 4 bytes), it should return values either value 0x00000000 or 0x00000002 .
	ret = checkIntStatus(GPIO_LOW, PERIOD, DEF_RETRY);
	if (ret != true) {
		EDGE_ERR_LOG("%s: Wrong GPIO status/Timeout\n", __func__);
		return false;
	}

	ret = checkACK(type_ACK);
	if (ret != true) {
		EDGE_ERR_LOG("%s: Wrong ACK, %d\n", __func__, __LINE__);
		return false;
	}

	if (type_ACK == ACK_READY) {
//	Write address 0x00 value 0x00000000; this should set interrupt pin HIGH.
//	And reading ACK REG should return values either value 0x00000000 or 0x00000002.
		if (debug_mask & BIT(1)) {
			EDGE_INFO_LOG("%s: Write ACK command\n", __func__);
		}
		xr15510_i2c_write(xr15510_i2c_flash_client, 0x00, cmd_ack, CMD_LEN);

		ret = checkIntStatus(GPIO_HIGH, PERIOD, DEF_RETRY);
		if (ret != true) {
			EDGE_ERR_LOG("%s: Wrong GPIO status/Timeout\n", __func__);
			return false;
		}

		ret = checkACK(ACK_READY);
		if (ret != true) {
			EDGE_ERR_LOG("%s: Wrong ACK, %d\n", __func__, __LINE__);
			return false;
		}
	}

	if (debug_mask & BIT(1)) {
		EDGE_INFO_LOG("%s: ---\n", __func__);
	}
	return true;
}

static int flash_firmware(struct device *dev, uint8_t *buf, int len)
{
	int ret = 0, bytes_remaining = 0, transfer_size = 0, offset = 0, i = 0;
	uint32_t crc;
	uint8_t header[HEADER_LEN]		= {0x00};
	uint8_t TxBuffer[TX_LEN]		= {0x00};
	uint8_t cmd_new_image[CMD_LEN]		= {0x02, 0x00, 0x00, 0x00};
	uint8_t cmd_new_packet[CMD_LEN]		= {0x03, 0x00, 0x00, 0x00};
	uint8_t cmd_reset[CMD_LEN]		= {0x04, 0x00, 0x00, 0x00};

//	1)	Set Boot-Override pin Low
//	2)	Send a hardware reset. Set nReset pin low of at least 2.5 us

	bootloader_procedure_control(1);

//	3)	Once the bootloader loads, the interrupt pin will be LOW.
//		Read ACK REG (address 0x00, 4 bytes), it should return values either value 0x00000000 or 0x00000002.
//	4)	Write address 0x00 value 0x00000000; this should set interrupt pin HIGH.
//		And reading ACK REG should return values either value 0x00000000 or 0x00000002.

//	ret = checkIntStatus(GPIO_HIGH, PERIOD, DEF_RETRY);
//	if (ret != true) {
//		EDGE_ERR_LOG("%s: Wrong GPIO status/Timeout\n", __func__);
//		ret = -EFAULT;
//		goto err_update_process;
//	}

	if (handshaking(ACK_READY) != true) {
		ret = -EFAULT;
		goto err_update_process;
	}

//	5)	The user will need to calculate Checksum of the binary image needed to be programmed.
//	    The CRC32 polynomial to calculate checksum is 0x1EDC6F41.

	fw_crc = crc = calCRC32(buf, len);

//	6)	Enter the flash memory Address at I2C address 0x04:0x07 value 0x00008000, Last byte first
//	7)	Convert the bin file to array and calculate the length of the binary files in terms of bytes.
//		Write length of the binary array to I2C reg location 0x08:0x0B, Last byte first.
//	8)	Enter the calculated CRC values at I2C reg location 0x0C:0x0F, last byte first.
//	9)	The first 32-bit value of array of Step 7 is the Stack pointer value and
//		the second 32-bit is the Reset Vector values.
//		Both of these values needs to be written to I2C reg location 0x18:0x1B for Stack pointer
//		and 0x1C:0x1F for Reset Vector.
//	10)	Write NEW_IMAGE command to location 0x00:0x03

	// address for bootloader
	header[0] = 0x00;	header[1] = 0x80;	header[2] = 0x00;	header[3] = 0x00;
	// byte length of binary file
	header[4] = len & MASK_8BIT;		header[5] = (len >>  8) & MASK_8BIT;
	header[6] = (len >> 16) & MASK_8BIT;	header[7] = (len >> 24) & MASK_8BIT;
	// crc of binary file
	header[8]  = crc & MASK_8BIT;		header[9]  = (crc >>  8) & MASK_8BIT;
	header[10] = (crc >> 16) & MASK_8BIT;	header[11] = (crc >> 24) & MASK_8BIT;
	// gpio
	header[12] = 0x3;	header[13] = 0x0;	header[14] = 0x0;	header[15] = 0x0;

	header[16] = 0x0;	header[17] = 0x0;	header[18] = 0x0;	header[19] = 0x0;
	// stack pointer
	header[20] = *buf;		header[21] = *(buf + 1);
	header[22] = *(buf + 2);	header[23] = *(buf + 3);
	// reset vector
	header[24] = *(buf + 4);	header[25] = *(buf + 5);
	header[26] = *(buf + 6);	header[27] = *(buf + 7);

	printk("[EDGE] Header:\n[EDGE] ");
	for (; i < HEADER_LEN; i++) {
		printk("%02X ", header[i]);
		if ((i + 1) % 8 == 0)
			printk("\n[EDGE] ");
	}
	printk("\n");

	xr15510_i2c_write(xr15510_i2c_flash_client, 0x04, header, HEADER_LEN);
	mdelay(100);
	if (debug_mask & BIT(1)) {
		EDGE_INFO_LOG("%s: delay 100ms after writing header to 0x04\n", __func__);
	}
	xr15510_i2c_write(xr15510_i2c_flash_client, 0x00, cmd_new_image, CMD_LEN);
	mdelay(100);
	if (debug_mask & BIT(1)) {
		EDGE_INFO_LOG("%s: delay 100ms after writing NEW_IMAGE to 0x00\n", __func__);
	}

//	11)	On writing NEW_IMAGE the interrupt pin will be toggled LOW.
//		Read ACK REG (address 0x00, 4 bytes), it should return values either value 0x00000000 or 0x00000002 .
//	12)	Write address 0x00 value 0x00000000; this should set interrupt pin HIGH.
//		And reading ACK REG should return values either value 0x00000000 or 0x00000002.

	if (handshaking(ACK_READY) != true) {
		ret = -EFAULT;
		goto err_update_process;
	}

//	13)	Next start sending binary files to flash memory, by using NEW_PACKET command.
//		The format of NEW_PACKET command is: first 4 bytes is the NEW_PACKET command,
//		next 4 bytes is the length of the payload, followed by the payload.
//	14)	After every NEW_PACKET commands the interrupt pin will be LOW and
//		user will read address 4 bytes from address 0x00 to confirm if the packet was transferred correctly,
//		if yes, write 0x00 ACK to set interrupt pin HIGH.

	EDGE_INFO_LOG("%s: +++++ Send packets to chip! +++++\n", __func__);

	// send data payload to MCU by transferring packets (TxBuffer)
	bytes_remaining = len;

    if (debug_mask & BIT(1)) {
        EDGE_INFO_LOG("%s: bytes_remaining = ", __func__);
    }
	//write packets to MCU until none remaining
	while (bytes_remaining > 0) {
        if (debug_mask & BIT(1)) {
            printk("%d->", bytes_remaining);
        }

		//in this example, data size is 112 bytes
		if (bytes_remaining > DATA_LEN)
			transfer_size = DATA_LEN;
		else
			transfer_size = bytes_remaining;

		//total size of transfer buffer
		//in this examples, total size is 116 bytes
		memset(TxBuffer, 0x00, (SIZE_LEN + transfer_size) * sizeof(u8));

		//transfer size in written into first 4 bytes of buffer
		TxBuffer[0] = transfer_size;
		TxBuffer[1] = 0x00;
		TxBuffer[2] = 0x00;
		TxBuffer[3] = 0x00;

		//move data from binary file to buffer, place after first 4 bytes (TxBuffer[4]...])
		memcpy(TxBuffer + SIZE_LEN, buf + offset, transfer_size);

		//write buffer to MCU (transfer_size + data) starting at address 0x04
		xr15510_i2c_write(xr15510_i2c_flash_client, 0x4, TxBuffer, SIZE_LEN + transfer_size);
		//mdelay(100);

		bytes_remaining = bytes_remaining - transfer_size;
		offset = offset + transfer_size;

		//write 4-byte new packet command (0x00000003) to address 0x00
		xr15510_i2c_write(xr15510_i2c_flash_client, 0x00, cmd_new_packet, CMD_LEN);
		mdelay(100);
		if (debug_mask & BIT(1)) {
			EDGE_INFO_LOG("%s: delay 100ms after writing NEW_PACKET to 0x00\n", __func__);
		}

		//wait until interrupt pin is low
		if (bytes_remaining > 0) {
			if (handshaking(ACK_READY) != true) {
				EDGE_ERR_LOG("%s: bytes_remaining=%d \n",
					     __func__, bytes_remaining);
				ret = -EFAULT;
				goto err_update_process;
			}
		}
	}
	if (debug_mask & BIT(1))
		printk("\n");
	EDGE_INFO_LOG("%s: ----- End of packets sending! -----\n", __func__);

//	15)	Once all the bytes are transferred the user should get response IMAGE_COMPLETE for successful transfer.

	if (handshaking(ACK_COMPLETE) != true) {
		EDGE_ERR_LOG("%s: Wrong ACK; Line:%d; bytes_remaining=%d \n",
			     __func__, __LINE__, bytes_remaining);
		ret = -EFAULT;
		goto err_update_process;
	}
	/*
		ret = checkACK(ACK_COMPLETE);
		if (ret != true) {
			EDGE_ERR_LOG("%s: Wrong ACK, %d\n", __func__, __LINE__);
			ret = -EFAULT;
			goto err_update_process;
		}
	*/
//	16)	Write RESET command to register 0x00:0x03

err_update_process:
	EDGE_INFO_LOG("%s: SW RESET\n", __func__);
	xr15510_i2c_write(xr15510_i2c_flash_client, 0x00, cmd_reset, CMD_LEN);


//	17)	Set boot-override GPIO HIGH and send Hardware reset.
//		If the firmware programming was successful then the boot loader should load in to user mode.

	bootloader_procedure_control(0);

	return ret;
}

#define CHECK_FW_VERSION 0
#define FORCE_UPDATE_FW  1
struct Header_data {
	union {
		struct {
			uint32_t reserved;   //RESERVED
			uint32_t fw_version; //FIRMWARE version
			uint32_t crc_value;  //CRC
			uint32_t fw_size;    //NUMBER OF BYTES needed to be programmed in the MCU
		} __packed;
		uint32_t data[4];
	};
};
static int firmware_update(bool force_update)
{
	int ret, i;
	struct Header_data header_data;

	EDGE_INFO_LOG("%s: debug_mask=0x%08X\n", __func__, debug_mask);

	if (atomic_read(&in_flash) != 0) {
		EDGE_INFO_LOG("%s: firmware update process is performing.\n", __func__);
        return 0;
	} else
		atomic_set(&in_flash, 1);

    wake_lock(&fw_wake_lock);

	EDGE_INFO_LOG("%s: Firmware request\n", __func__);
	for (i = 1; i < 4; i++) {
		ret = request_firmware(&fw, XR15510_FW_NAME, &xr15510_i2c_flash_client->dev);
		if (ret) {
			EDGE_ERR_LOG("%s: trial #%d, Request_firmware failed, ret = %d\n",
					__func__, i, ret);
			msleep(1000);
			continue;
		} else
			break;
	}

	if (!ret) {
		memcpy(&header_data, fw->data, sizeof(header_data));

        /* Big-endian to little endian */
		header_data.reserved    = be32_to_cpu(header_data.reserved);
		header_data.fw_version  = be32_to_cpu(header_data.fw_version);
		header_data.crc_value	= be32_to_cpu(header_data.crc_value);
		header_data.fw_size		= be32_to_cpu(header_data.fw_size);

		EDGE_INFO_LOG("%s: Firmware blob size = %d \n", __func__, (uint32_t)fw->size);

        /* Judgement whether headered and non-headered */
        if(header_data.fw_size == (fw->size - FW_HEADER_LENGTH)) {
			fw_data_start = (uint8_t *)(fw->data + FW_HEADER_LENGTH);
			fw_size = header_data.fw_size;

            EDGE_INFO_LOG("%s: Firmware w/ header: 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
                    __func__, header_data.reserved, header_data.fw_version,
                    header_data.crc_value, header_data.fw_size);
        } else {
            EDGE_INFO_LOG("%s: Firmware w/o header\n", __func__);
			fw_data_start = (uint8_t *)(fw->data);
            fw_size = fw->size;
        }

        /* Judgement for firmware auto update */
		if (force_update || fw_version != header_data.fw_version) {
			EDGE_INFO_LOG("%s: Firmware up to %d%s, fw_size = %d\n",
					__func__, header_data.fw_version, force_update ? "(force update)" : "",
                              fw_size);

			EDGE_INFO_LOG("%s: Firmware start with 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
					__func__, fw_data_start[0], fw_data_start[1],
                    fw_data_start[2], fw_data_start[3], fw_data_start[4]);

            if (flash_firmware(&xr15510_i2c_flash_client->dev, fw_data_start, fw_size) < 0) {
                /* Retry scenario #1:
                 *  1.a. monitor interrupt pin timout
                 *  1.b. ACK flag is not matched */
                if(fw_retry_cnt <= MAX_FW_UPDATE_RETRY) {
                    EDGE_ERR_LOG("%s: Firmwar update retry (%d)[Intr status error/i2c Nack]\n",
                                            __func__, fw_retry_cnt);
                    goto fw_update_retry;
                } else
                    EDGE_ERR_LOG("%s: Firmware upgrade error\n", __func__);
            } else {
                /* Chip ready time */
                mdelay(1000);
                fw_version = check_fw_version();

                /* Retry scenario #2:
                 *  2.a. firmware version is not matched as
                 *  the one defined in verbose header  */
                if((fw_version != header_data.fw_version) && (fw_retry_cnt <= MAX_FW_UPDATE_RETRY)) {
                    EDGE_ERR_LOG("%s: Firmware update retry(%d)[version mis-matched]\n",
                                            __func__, fw_retry_cnt);
                    goto fw_update_retry;

                } else if (fw_version != header_data.fw_version) {
                    EDGE_ERR_LOG("%s: Firmware upgrade warning[version mis-matched(%d != %d)] \n",
                                            __func__, fw_version, header_data.fw_version);
                } else
                    EDGE_INFO_LOG("%s: Firmware upgrade OK\n", __func__);
            }
		} else
			EDGE_INFO_LOG("%s: Firmware version(%d) matched, Bypass\n",
                                __func__, header_data.fw_version);

		release_firmware(fw);
	}

    wake_unlock(&fw_wake_lock);
	atomic_set(&in_flash, 0);

    return 1;

fw_update_retry:
    wake_unlock(&fw_wake_lock);
	atomic_set(&in_flash, 0);
    fw_retry_cnt++;
    queue_work(fw_workqueue, &fw_upd_work);
    return 0;
}

#define SENS_PROFILE_ID_P2  2
#define SENS_PROFILE_ID_P3  3
#define SENS_PROFILE_ID_CMD 11
static void edge_fw_check_work(struct work_struct *work)
{

    EDGE_INFO_LOG("%s: ++\n", __func__);

    /* Chip needs 350ms to ready itself */
    mdelay(350);

    /* Reset sensor */
	xr15510_hw_reset();

    fw_version = check_fw_version();

    EDGE_INFO_LOG("%s: --\n", __func__);
}

static void edge_fw_update_work(struct work_struct *work)
{
    uint8_t data, ret = 0;

    EDGE_INFO_LOG("%s: ++\n", __func__);
    ret = firmware_update(CHECK_FW_VERSION);

    if(ret) {
        /* Write sensor profile ID */
        data = p2_sensor_module ? SENS_PROFILE_ID_P2 : SENS_PROFILE_ID_P3;
        xr15510_i2c_write(xr15510_i2c_client, 0x40, &data, 1);
        data = SENS_PROFILE_ID_CMD;
        xr15510_i2c_write(xr15510_i2c_client, 0x04, &data, 1);

        /* Swtich i2c to MCU */
        EDGE_INFO_LOG("%s: gpio_select(%d) value was %d\n",
                __func__, gpio_sel, gpio_get_value(gpio_sel));
        gpio_set_value(gpio_sel, 1);
        EDGE_INFO_LOG("%s: gpio_select(%d) value is %d\n",
                __func__, gpio_sel, gpio_get_value(gpio_sel));

#ifdef CONFIG_NANOHUB_EDGE
        /* Inform i2c bus has switched over to hub*/
        nanohub_edge_i2c_switch(I2C_TO_SHUB);

        /* Set reset pin as input so MCU can manipulate this gpio */
        if (switch_reset) {
            msleep(40);
            gpio_direction_input(gpio_reset);
        }
#endif
    }

    EDGE_INFO_LOG("%s: --\n", __func__);
}

static ssize_t xr15510_fw_update_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t input;
	if (kstrtou32(buf, 16, &input) != 0) {
		EDGE_ERR_LOG("%s: bad parameter\n", __func__);
		return count;
	}

	if (input <= 2) {
		debug_mask = input;
		firmware_update(FORCE_UPDATE_FW);
	} else if (input == 3) {
		fw_version = 0x12;
		firmware_update(CHECK_FW_VERSION);
	} else if (input == 4) {
		fw_version = check_fw_version();
		firmware_update(CHECK_FW_VERSION);
	}

    return count;
}

static struct device_attribute flash_attrs[] = {
	__ATTR(flash, S_IWUSR,
			NULL,
			xr15510_fw_update_store),
};

static int xr15510_fw_create_file_sys(struct i2c_client *client)
{
	int ret, attr_count;

	if (android_edge_kobj == NULL)
		android_edge_kobj = kobject_create_and_add("android_edge", NULL);
	if (android_edge_kobj == NULL) {
		EDGE_ERR_LOG("%s: kobject_create_and_add failed\n", __func__);
		return -ENOMEM;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(flash_attrs); attr_count++) {
		ret = sysfs_create_file(android_edge_kobj,
					&flash_attrs[attr_count].attr);
		if (ret < 0) {
			EDGE_ERR_LOG("%s: Failed to create sysfs attributes\n", __func__);
			return ret;
		}
	}

//	ret = sysfs_create_link(NULL, &client->dev.kobj, "android_edge");
//	if (ret < 0) {
//		EDGE_ERR_LOG("%s: subsystem_register failed\n", __func__);
//		return ret;
//	}

	return 0;
}

static int xr15510_fw_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	int ret;
	xr15510_i2c_flash_client = i2c;

    EDGE_INFO_LOG("%s: Enter++\n", __func__);

    /* Create attribute file*/
	ret = xr15510_fw_create_file_sys(i2c);
	if (ret < 0) {
		EDGE_ERR_LOG("%s: xr15510_fw_create_file_sys failed\n", __func__);
		return ret;
	}

    /* Initialize wakelock */
    wake_lock_init(&fw_wake_lock, WAKE_LOCK_SUSPEND, "fw_wake_lock");

    /* Queue out the fw-update work to workqueue
     * and initialize retry counter */
    fw_retry_cnt = 0;
    queue_work(fw_workqueue, &fw_upd_work);

    EDGE_INFO_LOG("%s: End--\n", __func__);

	return ret;
}

static int xr15510_fw_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id xr15510_fw_i2c_id[] = {
	{ XR15510_FLASH_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id xr15510_fw_i2c_of_match_table[] = {
	{.compatible = "exar,xr15510_fw_flash",},
	{},
};
#else
#define xr15510_fw_i2c_of_match_table NULL
#endif

static struct i2c_driver xr15510_i2c_fw_driver = {
	.driver = {
		.name = XR15510_FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xr15510_fw_i2c_of_match_table,
	},
	.probe    = xr15510_fw_i2c_probe,
	.remove   = xr15510_fw_i2c_remove,
	.id_table = xr15510_fw_i2c_id,
};
#endif //CONFIG_EDGE_SENSOR_FW_UPDATE

static int __init xr15510_init(void)
{
    int ret, ignore;
	EDGE_INFO_LOG("%s++\n", __func__);

    ret = i2c_add_driver(&xr15510_i2c_edge_driver);
    if (ret) {
        EDGE_ERR_LOG("%s: add edge driver failed: %d\n", __func__, ret);
        return ret;
    }
#ifdef CONFIG_EDGE_SENSOR_FW_UPDATE
    ignore = i2c_add_driver(&xr15510_i2c_fw_driver);
    if (ignore) {
        EDGE_ERR_LOG("%s: add edge fw driver failed: %d\n", __func__, ret);
        // don't return, even if 0x10 probe failed, but the 0x4a probe
        // successfully. It should be taken as a success.
        // return ignore;
    }
#endif //CONFIG_EDGE_SENSOR_FW_UPDATE
	EDGE_INFO_LOG("%s--\n", __func__);
    return ret;
}
module_init(xr15510_init);

static void __exit xr15510_exit(void)
{
	EDGE_INFO_LOG("%s: ++\n", __func__);

	i2c_del_driver(&xr15510_i2c_edge_driver);
#ifdef CONFIG_EDGE_SENSOR_FW_UPDATE
	i2c_del_driver(&xr15510_i2c_fw_driver);
#endif //CONFIG_EDGE_SENSOR_FW_UPDATE
}
module_exit(xr15510_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("xr15510 edge sensor driver");
