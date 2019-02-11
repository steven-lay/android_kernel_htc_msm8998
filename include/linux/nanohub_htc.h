/*
 * Copyright (C) 2013 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _NANOHUB_HTC_H
#define _NANOHUB_HTC_H


#define EVT_APP_FROM_HOST                 0x000000F8
#define EVT_NO_FIRST_SENSOR_EVENT         0x00000200
#define EVT_NO_SENSOR_CONFIG_EVENT        0x00000300
#define EVT_RESET_REASON                  0x00000403
#define SENS_TYPE_PROX                    13
#define SENS_TYPE_GESTURE                 24
#define SENS_TYPE_HALL                    29

#ifdef CONFIG_NANOHUB_HTC_SENSTYPE_OFFSET
#define SENS_TYPE_HTC_BASE        96
#define SENS_TYPE_HTC_EASY_ACCESS (SENS_TYPE_HTC_BASE + 2)
#define SENS_TYPE_HTC_TOUCH       (SENS_TYPE_HTC_BASE + 3)
#define SENS_TYPE_HTC_SECOND_DISP (SENS_TYPE_HTC_BASE + 4)
#define SENS_TYPE_HTC_TOUCH_POINT (SENS_TYPE_HTC_BASE + 7)
#define SENS_TYPE_HTC_EDGE        (SENS_TYPE_HTC_BASE + 8)
#define SENS_TYPE_HTC_EDWK        (SENS_TYPE_HTC_BASE + 9)
#else
#define SENS_TYPE_HTC_EASY_ACCESS         54
#define SENS_TYPE_HTC_TOUCH               55
#define SENS_TYPE_HTC_SECOND_DISP         56
#define SENS_TYPE_HTC_TOUCH_POINT         59
#define SENS_TYPE_HTC_EDGE                60
#define SENS_TYPE_HTC_EDWK                61
#endif
#define CONFIG_CMD_CFG_DATA               3
#define sensorGetMyEventType(_sensorType) (EVT_NO_FIRST_SENSOR_EVENT + (_sensorType))

#define APP_ID_MAKE(vendor, app)       ((((uint64_t)(vendor)) << 24) | ((app) & 0x00FFFFFF))
#define APP_ID_VENDOR_HTC       0x4854437470ULL /* "HTCtp" */
#define BMI160_APP_ID APP_ID_MAKE(APP_ID_VENDOR_HTC, 2)

struct __attribute__ ((packed)) hal_cfg_data {
	uint8_t s_pole_near:1;
	uint8_t n_pole_near:1;
};


struct __attribute__ ((packed)) eza_cfg_data {
	uint8_t setting[4];
#if defined(CONFIG_NANOHUB_SECOND_DISP) || defined(CONFIG_NANOHUB_AOD)
	uint8_t lcd_mode;
#endif
};

struct __attribute__ ((packed)) tou_cfg_data {
	uint8_t status;
	uint8_t solution;
#if defined(CONFIG_NANOHUB_SECOND_DISP) || defined(CONFIG_NANOHUB_AOD)
	uint8_t mode;
#endif
};

struct __attribute__ ((packed)) snd_cfg_data {
	uint8_t switch_mcu;
	uint8_t cpu_suspend;
	uint16_t bl_ctrl;
};

struct __attribute__ ((packed)) pnt_cfg_data {
	uint8_t cpu_suspend;
	uint8_t switch_mcu;
	uint8_t lcd_mode;
};

struct __attribute__ ((packed)) edge_cfg_data {
	uint32_t threshold;
	uint8_t  usb_status;
	uint8_t  switch_mcu;
	uint8_t  key_status;
	uint8_t  reserved[1];
    uint32_t header;
    uint32_t i2c_switch;
};

struct __attribute__ ((packed)) ConfigCmd {
	uint32_t evtType;
	uint64_t latency;
	uint32_t rate;
	uint8_t sensorType;
	uint8_t cmd;
	uint16_t flags;
	uint8_t data[];
};

struct __attribute__ ((packed)) HostHubRawPacket {
	uint64_t appId;
	uint8_t dataLen;
};

struct __attribute__ ((packed)) MsgCmd {
	uint32_t evtType;
	struct HostHubRawPacket msg;
};


#define NANOHUB_TP_SWITCH_AP            0x00
#define NANOHUB_TP_SWITCH_MCU_NORMAL    0x01
#define NANOHUB_TP_SWITCH_MCU_GLOVE     0x02
#define NANOHUB_TP_SWITCH_DISABLED      0x08

#ifdef CONFIG_NANOHUB_TP_SWITCH
int nanohub_tp_status(uint8_t status);
int nanohub_tp_solution(uint8_t solution);
#else
static inline int nanohub_tp_status(uint8_t status) {
	return 0;
}
static inline int nanohub_tp_solution(uint8_t solution) {
	return 0;
}
#endif

#ifdef CONFIG_NANOHUB_SECOND_DISP
int nanohub_tp_mode(uint8_t mode);
int nanohub_is_switch_operating(void);
#elif defined(CONFIG_NANOHUB_AOD)
int nanohub_tp_mode(uint8_t mode);
#else
static inline int nanohub_tp_mode(uint8_t mode) {
	return 0;
}
static inline int nanohub_is_switch_operating(void) {
	return 0;
}
#endif

#ifdef CONFIG_NANOHUB_EDGE
#define I2C_TO_SHUB 1
#define I2C_TO_ACPU 0
int nanohub_edge_i2c_switch(uint8_t mode);
#else
static inline int nanohub_edge_i2c_switch(uint8_t mode) {
    return 0;
}
#endif

int nanohub_vbus_status(uint8_t status);

#ifdef CONFIG_NANOHUB
int nanohub_notifier(uint8_t event_id, void *val);
#else
static inline int nanohub_notifier(uint8_t event_id, void *val) {
	return 0;
}
#endif

enum {
	SECOND_DISP_BL_CTRL,
};

#ifdef CONFIG_NANOHUB_AOD
enum {
	LCD_MODE_U0 = 0,	//Display off
	LCD_MODE_U1,		//Reserve
	LCD_MODE_U2,		//AOD mode
	LCD_MODE_U3,		//Display on (normal mode)
};
#endif

#define NANOHUB_CPU_STATUS_RESUME       0x00
#define NANOHUB_CPU_STATUS_SUSPEND      0x01

#define HTC_PROX_DATA_BUFFER_INDEX_START    24

#endif
