/* Himax Android Driver Sample Code for HMX852xES chipset
*
* Copyright (C) 2014 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#ifndef HIMAX852xES_H
#define HIMAX852xES_H

#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/wakelock.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
//#include <linux/rtpm_prio.h>
#include <linux/htc_flags.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <uapi/linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
//#include <linux/of_gpio.h>
#include <linux/of_irq.h>

//#include <linux/htc_devices_dtb.h>


#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
#include <linux/touch_psensor.h>
#endif

#define QCT

#if defined(CONFIG_HMX_DB)
#include <linux/regulator/consumer.h>
#endif

#ifdef MTK
#ifdef CONFIG_OF
#define CONFIG_OF_TOUCH
#include <linux/of.h>
#include <linux/of_irq.h>

#endif
#include <linux/kthread.h>

//#include <mach/mt_reg_base.h>
//#include <mach/mt_typedefs.h>

//#include <mach/board.h>
//#include <mach/eint.h>

//#include <cust_eint.h>
//#include "cust_gpio_usage.h"
//#include <linux/dma-mapping.h>
//#include "tpd.h"
#define TPD_I2C_NUMBER				2
#endif

#include <linux/time.h>
#include <linux/jiffies.h>

#if defined(CONFIG_HMX_DB)
/* Analog voltage @2.7 V */
#define HX_VTG_MIN_UV			2700000
#define HX_VTG_MAX_UV			3300000
#define HX_ACTIVE_LOAD_UA		15000
#define HX_LPM_LOAD_UA 			10
/* Digital voltage @1.8 V */
#define HX_VTG_DIG_MIN_UV		1800000
#define HX_VTG_DIG_MAX_UV		1800000
#define HX_ACTIVE_LOAD_DIG_UA	10000
#define HX_LPM_LOAD_DIG_UA 		10

#define HX_I2C_VTG_MIN_UV		1800000
#define HX_I2C_VTG_MAX_UV		1800000
#define HX_I2C_LOAD_UA 			10000
#define HX_I2C_LPM_LOAD_UA 		10
#endif

struct himax_i2c_platform_data {	
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	uint8_t fw_version;
	uint8_t tw_id;
	uint8_t powerOff3V3;
	uint8_t cable_config[2];
	uint8_t protocol_type;
	uint8_t wake_up_reset;
	uint8_t glove_mode_en;
	uint8_t always_K_WA;
	uint8_t irq_on_state;
	uint32_t gpio_irq;
	uint32_t gpio_reset;
	uint32_t gpio_en;
	int (*power)(int on);
	void (*reset)(void);
	struct himax_virtual_key *virtual_key;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;

	struct himax_config *hx_config;
	int hx_config_size;
#if defined(CONFIG_HMX_DB)
	bool	i2c_pull_up;
	bool	digital_pwr_regulator;
	int reset_gpio;
	u32 reset_gpio_flags;
	int irq_gpio;
	u32 irq_gpio_flags;

	struct regulator *vcc_ana; //For Dragon Board
	struct regulator *vcc_dig; //For Dragon Board
	struct regulator *vcc_i2c; //For Dragon Board
#endif
	uint32_t SW_timer_debounce;
};



#define HIMAX_DRIVER_VER "0.1.6.0"

#define HIMAX852xes_NAME_CAP "Himax852xes_cap"
#define HIMAX852xes_FINGER_SUPPORT_NUM 10
#define HIMAX_I2C_ADDR				0x4b
#define INPUT_DEV_NAME	"himax-cap"
#define FLASH_DUMP_FILE "/data/user/Flash_Dump.bin"
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

#define KOBJ_NAME	"android_cap"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
#define D(x...) printk("[CAP] " x)
#define I(x...) printk("[CAP] " x)
#define W(x...) printk("[CAP][WARNING] " x)
#define E(x...) printk("[CAP][ERROR] " x)
#define DIF(x...) \
	if (debug_flag) \
	printk("[CAP][DEBUG] " x) \
} while(0)

#define I_TIME(fmt, arg...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_INFO "[CAP] " fmt \
		" (%02d-%02d %02d:%02d:%02d.%03lu)\n", \
		## arg, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, \
		tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000); \
	} while (0)

//#define HX_TP_PROC_DIAG
//#define HX_TP_PROC_RESET
//#define HX_TP_PROC_REGISTER
//#define HX_TP_PROC_DEBUG
//#define HX_TP_PROC_FLASH_DUMP
//#define HX_TP_PROC_SELF_TEST
//#define HX_TP_PROC_HITOUCH
//#define HX_TP_PROC_2T2R //if enable, Need to check "HX_2T2R_Addr"
						//and "HX_2T2R_en_setting" with project FW eng.

#define HX_TP_SYS_DIAG
#define HX_TP_SYS_REGISTER
#define HX_TP_SYS_DEBUG
#define HX_TP_SYS_FLASH_DUMP
#define HX_TP_SYS_SELF_TEST
#define HX_TP_SYS_RESET
#define HX_TP_SYS_HITOUCH
//#define HX_TP_SYS_2T2R

#else
#define D(x...) 
#define I(x...) 
#define W(x...) 
#define E(x...) 
#define DIF(x...)
#endif
//===========Himax Option function=============
#define HX_RST_PIN_FUNC
//#define HX_LOADIN_CONFIG
//#define HX_AUTO_UPDATE_FW
//#define HX_AUTO_UPDATE_CONFIG		//if enable HX_AUTO_UPDATE_CONFIG, need to disable HX_LOADIN_CONFIG
//#define HX_SMART_WAKEUP		//HTC does not use this feature in previous projects!
//#define HX_DOT_VIEW
//#define HX_PALM_REPORT
#if defined(CONFIG_TOUCHSCREEN_HIMAX_ESD_EN_CAP)
#define HX_ESD_WORKAROUND
//#define HX_CHIP_STATUS_MONITOR		//for ESD 2nd solution,default off
#endif
//#define HX_USB_DETECT

//#define HX_EN_SEL_BUTTON		       // Support Self Virtual key		,default is close
//#define HX_EN_MUT_BUTTON		       // Support Mutual Virtual Key	,default is close

#define HX_85XX_A_SERIES_PWON		1
#define HX_85XX_B_SERIES_PWON		2
#define HX_85XX_C_SERIES_PWON		3
#define HX_85XX_D_SERIES_PWON		4
#define HX_85XX_E_SERIES_PWON		5
#define HX_85XX_ES_SERIES_PWON		6

#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3
	
#define HX_KEY_MAX_COUNT             4			
#define DEFAULT_RETRY_CNT            10
#define MAX_RETRY_CNT            	30

#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOME
#define HX_VKEY_2   KEY_RESERVED
#define HX_VKEY_3   KEY_RESERVED
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

#define SHIFTBITS 5
#define FLASH_SIZE 32768

#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE   "pmx_ts_release"

struct himax_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
};

struct himax_config {
	uint8_t  default_cfg;
	uint8_t  sensor_id;
	uint8_t  fw_ver_main;
	uint8_t  fw_ver_minor;
	uint16_t length;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	uint8_t c1[11];
	uint8_t c2[11];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[17];
	uint8_t c19[15];
	uint8_t c20[5];
	uint8_t c21[11];
	uint8_t c22[4];
	uint8_t c23[3];
	uint8_t c24[3];
	uint8_t c25[4];
	uint8_t c26[2];
	uint8_t c27[2];
	uint8_t c28[2];
	uint8_t c29[2];
	uint8_t c30[2];
	uint8_t c31[2];
	uint8_t c32[2];
	uint8_t c33[2];
	uint8_t c34[2];
	uint8_t c35[3];
	uint8_t c36[5];
	uint8_t c37[5];
	uint8_t c38[9];
	uint8_t c39[14];
	uint8_t c40[159];
	uint8_t c41[99];
};

struct himax_ts_data {
	bool suspended;
	atomic_t suspend_mode;
	atomic_t sensing_status;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_command;
	uint8_t vendor_fw_ver_H;
	uint8_t vendor_fw_ver_L;
	uint8_t vendor_config_ver;
	uint8_t vendor_sensor_id;
	
	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t raw_data_frame_size;
	uint8_t raw_data_nframes;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint8_t diag_self[50];
	uint8_t update_feature;
	
	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;

	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	
	int use_irq;
	int (*power)(int on);
	int pre_finger_data[10][2];
	
	struct device *dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct i2c_client *client;
	struct himax_i2c_platform_data *pdata;	
	struct himax_virtual_key *button;
	struct wake_lock ts_flash_wake_lock;
	struct input_dev *sr_input_dev;

	const struct firmware *fw;
	uint8_t *fw_data_start;
	uint32_t fw_size;
	atomic_t in_flash;
	uint8_t cap_glove_mode_status;
#ifdef CONFIG_AK8789_HALLSENSOR
	struct notifier_block hallsensor_handler;
	uint8_t cs_cover_mode;
	uint8_t cap_hall_s_pole_status;
#endif

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *himax_att_wq;
	struct delayed_work work_att;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct workqueue_struct *himax_FW_wq;
	struct delayed_work work_FW;
#ifdef HX_CHIP_STATUS_MONITOR
	struct workqueue_struct *himax_chip_monitor_wq;
	struct delayed_work himax_chip_monitor;
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
	struct wake_lock ts_wake_lock;
#endif
#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)
	struct workqueue_struct 			*flash_wq;
	struct work_struct 					flash_work;
#endif
#ifdef HX_RST_PIN_FUNC
	uint32_t rst_gpio;
#endif
	uint32_t irq_gpio;

#ifdef HX_SMART_WAKEUP
	int SMWP_enable;
	int gesture_id;
	struct wake_lock ts_SMWP_wake_lock;
	uint8_t gesture_cust_en[16];
#endif
#ifdef HX_DOT_VIEW
	uint8_t cover_enable;
#endif
#ifdef HX_USB_DETECT
	uint8_t usb_connected;
	uint8_t *cable_config;
#endif
	uint32_t SW_timer_debounce;
	struct workqueue_struct *monitor_wq[2];
	struct workqueue_struct *recal_work_wq;
	struct delayed_work monitorDB[2];
	struct work_struct work_debounce_irq[2];
	struct work_struct recal_work;
	struct timer_list timer_debounce[2];
	unsigned char bouncing_flag[2];
	spinlock_t lock;
	bool debounced[2];
	uint8_t K_counter;
	uint8_t ESD_counter;
	struct timespec time_start;
	struct timespec ESD_time_start;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
};

static struct himax_ts_data *private_ts;

#define HX_CMD_NOP					 0x00	
#define HX_CMD_SETMICROOFF			 0x35	
#define HX_CMD_SETROMRDY			 0x36	
#define HX_CMD_TSSLPIN				 0x80	
#define HX_CMD_TSSLPOUT 			 0x81	
#define HX_CMD_TSSOFF				 0x82	
#define HX_CMD_TSSON				 0x83	
#define HX_CMD_ROE					 0x85	
#define HX_CMD_RAE					 0x86	
#define HX_CMD_RLE					 0x87	
#define HX_CMD_CLRES				 0x88	
#define HX_CMD_TSSWRESET			 0x9E	
#define HX_CMD_SETDEEPSTB			 0xD7	
#define HX_CMD_SET_CACHE_FUN		 0xDD	
#define HX_CMD_SETIDLE				 0xF2	
#define HX_CMD_SETIDLEDELAY 		 0xF3	
#define HX_CMD_SELFTEST_BUFFER		 0x8D	
#define HX_CMD_MANUALMODE			 0x42
#define HX_CMD_FLASH_ENABLE 		 0x43
#define HX_CMD_FLASH_SET_ADDRESS	 0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND	 0x47
#define HX_CMD_FLASH_WRITE_BUFFER	 0x48
#define HX_CMD_FLASH_PAGE_ERASE 	 0x4D
#define HX_CMD_FLASH_SECTOR_ERASE	 0x4E
#define HX_CMD_CB					 0xCB
#define HX_CMD_EA					 0xEA
#define HX_CMD_4A					 0x4A
#define HX_CMD_4F					 0x4F
#define HX_CMD_B9					 0xB9
#define HX_CMD_76					 0x76

#define HX_VER_FW_MAJ				 0x33
#define HX_VER_FW_MIN				 0x32
#define HX_VER_FW_CFG				 0x39

#define CS_SENSITIVITY_START_REG	0x8F

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

enum hx_cap_sensitivity {
	CS_SENS_HIGH	= 0x80,
	CS_SENS_DEFAULT	= 0x00
};

//1uA
static unsigned char E_IrefTable_1[16][2] = { {0x20,0x0F},{0x20,0x1F},{0x20,0x2F},{0x20,0x3F},
											{0x20,0x4F},{0x20,0x5F},{0x20,0x6F},{0x20,0x7F},
											{0x20,0x8F},{0x20,0x9F},{0x20,0xAF},{0x20,0xBF},
											{0x20,0xCF},{0x20,0xDF},{0x20,0xEF},{0x20,0xFF}};

//2uA
static unsigned char E_IrefTable_2[16][2] = { {0xA0,0x0E},{0xA0,0x1E},{0xA0,0x2E},{0xA0,0x3E},
											{0xA0,0x4E},{0xA0,0x5E},{0xA0,0x6E},{0xA0,0x7E},
											{0xA0,0x8E},{0xA0,0x9E},{0xA0,0xAE},{0xA0,0xBE},
											{0xA0,0xCE},{0xA0,0xDE},{0xA0,0xEE},{0xA0,0xFE}};													

//3uA
static unsigned char E_IrefTable_3[16][2] = { {0x20,0x0E},{0x20,0x1E},{0x20,0x2E},{0x20,0x3E},
											{0x20,0x4E},{0x20,0x5E},{0x20,0x6E},{0x20,0x7E},
											{0x20,0x8E},{0x20,0x9E},{0x20,0xAE},{0x20,0xBE},
											{0x20,0xCE},{0x20,0xDE},{0x20,0xEE},{0x20,0xFE}};			
		
//4uA
static unsigned char E_IrefTable_4[16][2] = { {0xA0,0x0D},{0xA0,0x1D},{0xA0,0x2D},{0xA0,0x3D},
											{0xA0,0x4D},{0xA0,0x5D},{0xA0,0x6D},{0xA0,0x7D},
											{0xA0,0x8D},{0xA0,0x9D},{0xA0,0xAD},{0xA0,0xBD},
											{0xA0,0xCD},{0xA0,0xDD},{0xA0,0xED},{0xA0,0xFD}};	 
																
//5uA
static unsigned char E_IrefTable_5[16][2] = { {0x20,0x0D},{0x20,0x1D},{0x20,0x2D},{0x20,0x3D},
											{0x20,0x4D},{0x20,0x5D},{0x20,0x6D},{0x20,0x7D},
											{0x20,0x8D},{0x20,0x9D},{0x20,0xAD},{0x20,0xBD},
											{0x20,0xCD},{0x20,0xDD},{0x20,0xED},{0x20,0xFD}};	 
																
//6uA
static unsigned char E_IrefTable_6[16][2] = { {0xA0,0x0C},{0xA0,0x1C},{0xA0,0x2C},{0xA0,0x3C},
											{0xA0,0x4C},{0xA0,0x5C},{0xA0,0x6C},{0xA0,0x7C},
											{0xA0,0x8C},{0xA0,0x9C},{0xA0,0xAC},{0xA0,0xBC},
											{0xA0,0xCC},{0xA0,0xDC},{0xA0,0xEC},{0xA0,0xFC}};	 
																
//7uA
static unsigned char E_IrefTable_7[16][2] = { {0x20,0x0C},{0x20,0x1C},{0x20,0x2C},{0x20,0x3C},
											{0x20,0x4C},{0x20,0x5C},{0x20,0x6C},{0x20,0x7C},
											{0x20,0x8C},{0x20,0x9C},{0x20,0xAC},{0x20,0xBC},
											{0x20,0xCC},{0x20,0xDC},{0x20,0xEC},{0x20,0xFC}}; 
static u8 		HW_RESET_ACTIVATE 	= 1;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
	#define HTC_FEATURE

	#define HIMAX_PROC_TOUCH_FOLDER 	"android_touch"
	#define HIMAX_PROC_DEBUG_LEVEL_FILE	"debug_level"
	#define HIMAX_PROC_VENDOR_FILE		"vendor"
	#define HIMAX_PROC_ATTN_FILE		"attn"
	#define HIMAX_PROC_INT_EN_FILE		"int_en"
	#define HIMAX_PROC_LAYOUT_FILE		"layout"

	static struct proc_dir_entry *himax_touch_proc_dir 			= NULL;
	static struct proc_dir_entry *himax_proc_debug_level_file 	= NULL;
	static struct proc_dir_entry *himax_proc_vendor_file 		= NULL;
	static struct proc_dir_entry *himax_proc_attn_file 			= NULL;
	static struct proc_dir_entry *himax_proc_int_en_file 		= NULL;
	static struct proc_dir_entry *himax_proc_layout_file 		= NULL;

	static uint8_t HX_PROC_SEND_FLAG;

#ifdef HX_TP_PROC_RESET
#define HIMAX_PROC_RESET_FILE		"reset"
static struct proc_dir_entry *himax_proc_reset_file 		= NULL;
#endif

#if defined(HX_TP_PROC_DIAG) || defined(HX_TP_SYS_DIAG)
	#define HIMAX_PROC_DIAG_FILE	"diag"
#if defined(HX_TP_PROC_DIAG)
	static struct proc_dir_entry *himax_proc_diag_file = NULL;
#endif	
	static int  touch_monitor_stop_flag 		= 0;
	static int 	touch_monitor_stop_limit 		= 5;
	static uint8_t x_channel 		= 0;
	static uint8_t y_channel 		= 0;
	static uint8_t *diag_mutual = NULL;

	static int diag_command = 0;
	static uint8_t diag_coor[128];// = {0xFF};
	static uint8_t diag_self[100] = {0};

	static uint8_t *getMutualBuffer(void);
	static uint8_t *getSelfBuffer(void);
	static uint8_t 	getDiagCommand(void);
	static uint8_t 	getXChannel(void);
	static uint8_t 	getYChannel(void);
	
	static void 	setMutualBuffer(void);
	static void 	setXChannel(uint8_t x);
	static void 	setYChannel(uint8_t y);
	
	static uint8_t	coordinate_dump_enable = 0;
	struct file	*coordinate_fn;
#if defined(HX_TP_PROC_2T2R) || defined(HX_TP_SYS_2T2R)
	static bool Is_2T2R = false;
	static int HX_2T2R_Addr = 0x96;//Need to check with project FW eng.
	static int HX_2T2R_en_setting = 0x02;//Need to check with project FW eng.
	static uint8_t x_channel_2 = 0;
	static uint8_t y_channel_2 = 0;
	static uint8_t *diag_mutual_2 = NULL;
	static int		HX_RX_NUM_2	= 0;
	static int 		HX_TX_NUM_2	= 0;

	static uint8_t *getMutualBuffer_2(void);
	static uint8_t 	getXChannel_2(void);
	static uint8_t 	getYChannel_2(void);

	static void 	setMutualBuffer_2(void);
	static void 	setXChannel_2(uint8_t x);
	static void 	setYChannel_2(uint8_t y);
#endif
#endif

#if defined(HX_TP_PROC_REGISTER) || defined(HX_TP_SYS_REGISTER)
	#define HIMAX_PROC_REGISTER_FILE	"register"
#if defined(HX_TP_PROC_REGISTER)
	static struct proc_dir_entry *himax_proc_register_file = NULL;
#endif
	static uint8_t register_command 			= 0;
	static uint8_t multi_register_command = 0;
	static uint8_t multi_register[8] 			= {0x00};
	static uint8_t multi_cfg_bank[8] 			= {0x00};
	static uint8_t multi_value[1024] 			= {0x00};
	static bool 	config_bank_reg 				= false;
#endif

#if defined(HX_TP_PROC_DEBUG) || defined(HX_TP_SYS_DEBUG)
	#define HIMAX_PROC_DEBUG_FILE	"debug"
#if defined(HX_TP_PROC_DEBUG)
	static struct proc_dir_entry *himax_proc_debug_file = NULL;
#endif
	static bool	fw_update_complete = false;

	static int handshaking_result = 0;
	static unsigned char debug_level_cmd = 0;
	static unsigned char upgrade_fw[32*1024];
#endif

#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)
	#define HIMAX_PROC_FLASH_DUMP_FILE	"flash_dump"
#if defined(HX_TP_PROC_FLASH_DUMP)
	static struct proc_dir_entry *himax_proc_flash_dump_file = NULL;
#endif
	static uint8_t *flash_buffer 				= NULL;
	static uint8_t flash_command 				= 0;
	static uint8_t flash_read_step 			= 0;
	static uint8_t flash_progress 			= 0;
	static uint8_t flash_dump_complete	= 0;
	static uint8_t flash_dump_fail 			= 0;
	static uint8_t sys_operation				= 0;
	static uint8_t flash_dump_sector	 	= 0;
	static uint8_t flash_dump_page 			= 0;
	static bool    flash_dump_going			= false;

	static uint8_t getFlashCommand(void);
	static uint8_t getFlashDumpComplete(void);
	static uint8_t getFlashDumpFail(void);
	static uint8_t getFlashDumpProgress(void);
	static uint8_t getFlashReadStep(void);
	static uint8_t getSysOperation(void);
	static uint8_t getFlashDumpSector(void);
	static uint8_t getFlashDumpPage(void);
#ifdef MTK	
	static bool	   getFlashDumpGoing(void);
#endif
	static void setFlashBuffer(void);
	static void setFlashCommand(uint8_t command);
	static void setFlashReadStep(uint8_t step);
	static void setFlashDumpComplete(uint8_t complete);
	static void setFlashDumpFail(uint8_t fail);
	static void setFlashDumpProgress(uint8_t progress);
	static void setSysOperation(uint8_t operation);
	static void setFlashDumpSector(uint8_t sector);
	static void setFlashDumpPage(uint8_t page);
	static void setFlashDumpGoing(bool going);
#endif

#ifdef HX_TP_SYS_SELF_TEST
	static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf);
	static int himax_chip_self_test(void);
#endif

#ifdef HX_TP_PROC_SELF_TEST
	#define HIMAX_PROC_SELF_TEST_FILE	"self_test"
	static struct proc_dir_entry *himax_proc_self_test_file = NULL;
#endif

#if defined(HX_TP_PROC_HITOUCH) || defined(HX_TP_SYS_HITOUCH)
	#define HIMAX_PROC_HITOUCH_FILE	"hitouch"
#if defined(HX_TP_PROC_HITOUCH)
	static struct proc_dir_entry *himax_proc_hitouch_file = NULL;
#endif
	static int	hitouch_command			= 0;
	static bool hitouch_is_connect	= false;
#endif
#endif

#ifdef HX_ESD_WORKAROUND
	static u8 		ESD_RESET_ACTIVATE 	= 1;
	static u8 		ESD_COUNTER 		= 0;
	//static int 		ESD_COUNTER_SETTING = 3;
	static u8		ESD_R36_FAIL		= 0;
#endif

#ifdef HX_RST_PIN_FUNC
	void himax_HW_reset_cap(uint8_t loadconfig,uint8_t int_off);
#endif

#ifdef HX_SMART_WAKEUP
#define GEST_PTLG_ID_LEN    (4)
#define GEST_PTLG_HDR_LEN   (4)
#define GEST_PTLG_HDR_ID1   (0xCC)
#define GEST_PTLG_HDR_ID2   (0x44)
#define GEST_PT_MAX_NUM     (128)

static int gest_pt_cnt;
static int gest_pt_x[GEST_PT_MAX_NUM];
static int gest_pt_y[GEST_PT_MAX_NUM];
static int gest_start_x=0,gest_start_y=0,gest_end_x=0,gest_end_y=0;
static int gest_left_top_x=0,gest_left_top_y=0,gest_right_down_x=0,gest_right_down_y=0;
static int gest_key_pt_x[3];
static int gest_key_pt_y[3];

	enum gesture_event_type {
		EV_GESTURE_01 = 0x01,
		EV_GESTURE_02,
		EV_GESTURE_03,
		EV_GESTURE_04,
		EV_GESTURE_05,
		EV_GESTURE_06,
		EV_GESTURE_07,
		EV_GESTURE_08,
		EV_GESTURE_09,
		EV_GESTURE_10,
		EV_GESTURE_11,
		EV_GESTURE_12,
		EV_GESTURE_13,
		EV_GESTURE_14,
		EV_GESTURE_15,
		EV_GESTURE_PWR = 0x80,
	};

	#define KEY_CUST_01 0x21d //Up
	#define KEY_CUST_02 0x21e //Down
	#define KEY_CUST_03 0x21c //Left
	#define KEY_CUST_04 0x21b //Right
	#define KEY_CUST_05 0x225 //C
	#define KEY_CUST_06 0x226 //Z
	#define KEY_CUST_07 0x222 //m
	#define KEY_CUST_08 0x220 //O
	#define KEY_CUST_09 0x226 //S
	#define KEY_CUST_10 0x228 //V
	#define KEY_CUST_11 0x223 //W
	#define KEY_CUST_12 0x221 //e
	#define KEY_CUST_13 0xFFF //reserve
	#define KEY_CUST_14 0xFFF //reserve
	#define KEY_CUST_15 0xFFF //reserve
	
	//#define HIMAX_PROC_SMWP_FILE "SMWP"
	//static struct proc_dir_entry *himax_proc_SMWP_file = NULL;
	//#define HIMAX_PROC_GESTURE_FILE "GESTURE"
	//static struct proc_dir_entry *himax_proc_GESTURE_file = NULL;
	static bool FAKE_POWER_KEY_SEND = false;
#endif

#ifdef HX_DOT_VIEW
	#include <linux/hall_sensor.h>
	#define HIMAX_PROC_COVER_FILE "cover"
	static struct proc_dir_entry *himax_proc_cover_file = NULL;
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
#include <linux/hall_sensor.h>
static int hallsensor_handler_callback(struct notifier_block *self,
		unsigned long event, void *data);
#endif

#ifdef HX_AUTO_UPDATE_CONFIG
static int		CFB_START_ADDR					= 0;
static int		CFB_LENGTH						= 0;
static int		CFB_INFO_LENGTH 				= 0;
#endif

#ifdef HX_CHIP_STATUS_MONITOR
static int HX_CHIP_POLLING_COUNT;
static int HX_POLLING_TIMER		=5;//unit:sec
static int HX_POLLING_TIMES		=2;//ex:5(timer)x2(times)=10sec(polling time)
static int HX_ON_HAND_SHAKING   =0;//
#endif

#ifdef MTK
#ifdef CONFIG_OF
#define CONFIG_OF_TOUCH
#endif

#define CAP_DEVICE            "mtk-tpd-cap"

struct i2c_client *hx_i2c_client_point = NULL;
static int tpd_flag = 0;
static struct task_struct *touch_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static unsigned short force[] = {0, 0x94, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };

//Custom set some config
//static int hx_panel_coords[4] = {0,720,0,1280};//[1]=X resolution, [3]=Y resolution
//static int hx_display_coords[4] = {0,720,0,1280};
static int report_type = PROTOCOL_TYPE_A;

static int himax852xes_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int himax852xes_detect(struct i2c_client *client, struct i2c_board_info *info);
static int himax852xes_remove(struct i2c_client *client);
#endif

#endif
