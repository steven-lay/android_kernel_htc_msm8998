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
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/i2c.h>
#include <linux/input.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define NVT_DEBUG 0
#define HTC_FEATURE

#ifdef HTC_FEATURE
#if 0
#include <trace/events/power.h>		//systrace: trace_clock_set_rate()
#endif
#include <linux/async.h>
#include <linux/htc_flags.h>
#include <linux/atomic.h>
#ifdef CONFIG_NANOHUB_TP_SWITCH
#include <linux/nanohub_htc.h>
#endif
#endif

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943
#define NVTTOUCH_DISP_RST_PIN 956


//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
//#define IRQ_TYPE_LEVEL_HIGH 4
//#define IRQ_TYPE_LEVEL_LOW 8
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING


//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

#ifdef HTC_FEATURE
#define NVT_LOG(fmt, args...)    pr_info("[TP] " fmt, ##args)
#define NVT_ERR(fmt, args...)    pr_err("[TP][ERR] %s: " fmt, __func__, ##args)
#else	/* #ifdef HTC_FEATURE */
#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif	/* #ifdef HTC_FEATURE */

//---Input device info.---
#ifdef HTC_FEATURE
#define NVT_TS_NAME "nvt_touchscreen"
#else
#define NVT_TS_NAME "nvt_tp_input"
#endif

//---Touch info.---
#define TOUCH_MAX_WIDTH 1080
#define TOUCH_MAX_HEIGHT 1920
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#define POINT_DATA_CHECKSUM 0

#ifdef HTC_FEATURE
#define GLOVE_MODE_BIT BIT(0)
#define COVER_MODE_BIT BIT(1)
#endif

#ifdef HTC_FEATURE
#define HOPPINT_STATUS_MAX	3
#define HOPPINT_SEED_MAX	5
#endif

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
	uint32_t READ_FLASH_CHECKSUM_ADDR;
	uint32_t RW_FLASH_DATA_ADDR;
};

#ifdef HTC_FEATURE
struct nvt_finger_info {
	uint8_t press_status;
	int x;
	int y;
	int z;
	int type;
};

struct nvt_noise_info {
	int8_t hopping_status;
	int8_t hopping_seed;
};
#endif

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
#ifdef HTC_FEATURE
	int32_t switch_gpio;
	uint32_t switch_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	int32_t power_gpio;
	uint32_t power_flags;
	uint8_t fw_ver;
	struct nvt_finger_info *report_points;
	struct nvt_noise_info report_noise;
	struct pinctrl *tp_pinctrl;
	uint8_t i2c_to_mcu;
	bool tp_bus_sel_en;
	uint8_t support_glove;
	uint8_t tp_mode_state;
	uint8_t irq_enabled;
	atomic_t firmware_updating;
#define SHOW_INT_I2C_BUF		BIT(2)
//Priority: Bit(3) > Bit(4) > Bit(5)
#define TOUCH_DOWN_UP_LOG		BIT(3)	//Print Down/Up logs
#define TOUCH_KPI_LOG			BIT(4)	//Print Down/Move/Up with interrupt received time and input_sync time
#define TOUCH_BREAKDOWN_TIME		BIT(5)	//Print Down/Move/Up with total handling time and bus time
#define TOUCH_BREAKDOWN_LOG		BIT(6)	//Print logs while enter or leave critical functions.
	struct timespec tp_handler_time;
	struct timespec tp_read_start_time;
	struct timespec tp_read_done_time;
	struct timespec tp_sync_time;
#else
	int32_t reset_gpio;
	uint32_t reset_flags;
	int32_t disp_rst_gpio;
	uint32_t disp_rst_flags;
#endif
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN	// normal run
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
} I2C_EVENT_MAP;

#endif /* _LINUX_NVT_TOUCH_H */
