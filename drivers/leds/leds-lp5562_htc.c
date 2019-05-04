/* driver/leds/leds-lp5562_htc.c
 *
 * Copyright (C) 2010 HTC Corporation.
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
/*#include <linux/android_alarm.h>*/
#include <linux/leds.h>
#include <linux/leds-lp5562_htc.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
/*include <mach/ADP5585_ioextender.h>*/
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#define LP5562_MAX_LEDS			9	/* Maximum number of LEDs */
#define LED_DEBUG				1
#if LED_DEBUG
#define D(x...) printk(KERN_DEBUG "[LED]" x)
#define I(x...) printk(KERN_INFO "[LED]" x)
#else
#define D(x...)
#define I(x...)
#endif

#define MAIN_TOUCH_SOLUTION 2
#define SEC_TOUCH_SOLUTION 1

static int led_rw_delay, chip_enable, rgb_enable, vk_enable;
static int current_time;
static struct i2c_client *private_lp5562_client = NULL;
static struct mutex	led_mutex;
static struct mutex	vk_led_mutex;
static struct workqueue_struct *g_led_work_queue;
static uint32_t ModeRGB;
static int VK_brightness;
static int last_pwm;
static uint8_t virtual_key_led_ignore_flag = 0;
static uint16_t g_led_touch_solution = MAIN_TOUCH_SOLUTION;
static u8 color_table[20] = {0};
static int use_color_table = 0;
static u8 current_table[20] = {0};
static int use_current_table = 0;
static int table_level_num = 0;
static uint32_t charging_flag = 0; /* cei mfg test only */
#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
static int display_flag = 0;
module_param(display_flag, int, 0660); /* mfg test only */
static struct lp5562_led *g_led_led_data;
#endif
static struct led_i2c_platform_data *plat_data;

module_param(virtual_key_led_ignore_flag, byte, S_IRUSR | S_IWUSR);

#define Mode_Mask (0xff << 24)
#define Red_Mask (0xff << 16)
#define Green_Mask (0xff << 8)
#define Blue_Mask 0xff

#define RG_Mask 0x3C
#define VK_Mask 0x03

#define VK_LED_FADE_LEVEL	16
#define VL_LED_FADE_TIME	125
#define VK_LED_SLEEP_TIME	115

static int gCurrent_param = 95;
static int gVK_Current_param = 95;

module_param(gCurrent_param, int, 0600);
module_param(gVK_Current_param, int, 0600);

#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
extern bool htc_check_panel_connection(void);
#endif

struct lp5562_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev	cdev;
	struct mutex 		led_data_mutex;
/*	struct alarm 		led_alarm;*/
	struct work_struct 	led_work;
	struct work_struct 	led_work_multicolor;
	uint8_t 		Mode;
	uint8_t			Red;
	uint8_t 		Green;
	uint8_t 		Blue;
	int			VK_brightness;
	struct delayed_work	blink_delayed_work;
	struct wake_lock        led_wake_lock;
};

struct lp5562_chip {
	struct led_i2c_platform_data *pdata;
	struct mutex		led_i2c_rw_mutex; /* Serialize control */
	struct i2c_client	*client;
	struct lp5562_led	leds[LP5562_MAX_LEDS];
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_init;
};

typedef enum {
	INDICATOR_LED_ID = 0,
	VIRTUAL_KEY_LED_ID = 1,
} LED_ID;

static int lp5562_parse_dt(struct device *, struct led_i2c_platform_data *);

static char *hex2string(uint8_t *data, int len)
{
	static char buf[LED_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = LED_I2C_WRITE_BLOCK_SIZE -1;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		snprintf(buf + i * 4, sizeof(buf),"[%02X]", data[i]);

	return buf;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
		uint8_t *data, int length)
{
	int retry;
	uint8_t buf[LED_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct lp5562_chip *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > LED_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[LED] i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->led_i2c_rw_mutex);
	msleep(1);
	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(led_rw_delay);
	}
	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[LED] i2c_write_block retry over %d times\n",
				I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->led_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->led_i2c_rw_mutex);

	return 0;
}


static int I2C_RxData_2(char *rxData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msgs[] = {
		{
			.addr = private_lp5562_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = private_lp5562_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_WRITE_RETRY_TIMES; loop_i++) {
		if (i2c_transfer(private_lp5562_client->adapter, msgs, 2) > 0)
			break;
		msleep(10);
	}

	if (loop_i >= I2C_WRITE_RETRY_TIMES) {
		printk(KERN_ERR "[LED] %s retry over %d times\n",
				__func__, I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_read_block(struct i2c_client *client,
		uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err("[LED]%s: length %d> 2: \n", __func__, length);
		return ret;
	}
	buffer[0] = cmd;
	ret = I2C_RxData_2(buffer, length);
	if (ret < 0) {
		pr_err("[LED]%s: I2C_RxData fail \n", __func__);
		return ret;
	}

	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
	}
	return ret;
}

static int write_enable_register(struct i2c_client *client, uint8_t data, uint8_t vk_led)
{
	int ret = 0;
	uint8_t temp = 0, current_data = 0;

	if(!rgb_enable && !vk_enable && data == 0) {
		I("write_enable_register disable\n");
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
		return ret;
	}
	else
		temp = 0x40;

	ret = i2c_read_block(client, ENABLE_REGISTER, &current_data, 1);
	if(vk_led)
		temp |= (current_data & RG_Mask) | (data & VK_Mask);
	else
		temp |= (data & RG_Mask) | (current_data & VK_Mask);

	ret = i2c_write_block(client, ENABLE_REGISTER, &temp, 1);
	return ret;
}
static int write_operation_register(struct i2c_client *client, uint8_t data, uint8_t vk_led)
{
	int ret = 0;
	uint8_t temp = 0, current_data = 0;

	ret = i2c_read_block(client, OPRATION_REGISTER, &current_data, 1);

	if(vk_led)
		temp |= (current_data & RG_Mask) | (data & VK_Mask);
	else
		temp |= (data & RG_Mask) | (current_data & VK_Mask);

	ret = i2c_write_block(client, OPRATION_REGISTER, &temp, 1);
	return ret;
}

static int write_vk_led_program(struct i2c_client *client)
{
	int ret = 0, reg_index = 0;
	uint8_t data, step_time, target_pwm;
	uint8_t command_data[32] = {0};

	data = 0x01;
	ret = write_operation_register(client, data, 1);


	/*=== Increase 16 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 1 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 32 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 2 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 48 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 3 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 64 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 4 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 80 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 5 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 96 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 6 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 112 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 7 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 128 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 8 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Decrease 16 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 1 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 32 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 2 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 48 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 3 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 64 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 4 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 80 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 5 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 96 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 6 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 112 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 7 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 128 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 8 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, 32);

	data = 0x02;
	ret = write_operation_register(client, data, 1);

	return ret;
}

static void lp5562_led_enable(struct i2c_client *client, int blink_enable)
{
	int ret = 0;
	uint8_t data;

	char data1[1] = {0};

	I(" %s +++\n" , __func__);

	if (chip_enable) {
		I(" %s return, chip already enable\n" , __func__);
		return;
	}
#if 0 // LED enable pin keep high to avoid SRAM abnormal
	/* === led pin enable ===*/
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	} /*else if (pdata->ena_gpio_io_ext) {
	    ret = ioext_gpio_set_value(pdata->ena_gpio_io_ext, 1);
	    if (ret < 0) {
	    pr_err("[LED] %s: io_extender high failed %d\n", __func__, ret);
	    gpio_free(pdata->ena_gpio);
	    }
	    }*/

	msleep(1);
#endif

#if 1
	if (blink_enable) {
		/* === reset ===*/
		data = 0xFF;
		ret = i2c_write_block(client, RESET_CONTROL, &data, 1);
		msleep(20);
		ret = i2c_read_block(client, R_CURRENT_CONTROL, data1, 1);
		if (data1[0] != 0xaf) {
			I(" %s reset not ready %x\n" , __func__, data1[0]);
		}
	}
#endif
	chip_enable = 1;
	mutex_lock(&led_mutex);
	/* === enable CHIP_EN === */
	data = 0x40;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	/* === configuration control in power save mode=== */
	data = 0x29;
	ret = i2c_write_block(client, CONFIG_REGISTER, &data, 1);
	/* ===  Init LED mapping === */
	data = 0xDB;
	ret = i2c_write_block(client, LED_MAP_CONTROL, &data, 1);
	ret = i2c_read_block(client, LED_MAP_CONTROL, &data, 1);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_led_disable(struct i2c_client *client)
{
	int ret = 0;
	uint8_t data;
	I(" %s +++\n" , __func__);

	if (!chip_enable) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}

	data = 0x00;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
#if 0 // LED enable pin keep high to avoid SRAM abnormal
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	}
#endif

	chip_enable = 0;

	I(" %s ---\n" , __func__);
}

static void lp5562_red_long_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;

	I(" %s +++\n" , __func__);
	mutex_lock(&led_mutex);
	data = 0x10;
	ret = write_operation_register(client, data, 0);
	udelay(200);

	reg_index = 0;
	/* === set pwm to 200 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.999s === */
	data = 0x7f;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === set pwm to 0 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.999s === */
	data = 0x7f;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === clear register === */
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);

	/* === run program === */

	data = 0x20;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = 0x60;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_color_blink(struct i2c_client *client, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;
	uint8_t mode = 0x00;
	I(" %s +++ red:%d, green:%d, blue:%d\n" , __func__, red, green, blue);
	mutex_lock(&led_mutex);

	if (red)
		mode |= (3 << 4);
	if (green)
		mode |= (3 << 2);
	if (blue)
		mode |= 3;
	data = mode & 0x15;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
	udelay(200);
	ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);
#endif

	if (red) {
		reg_index = 0;
		/* === set red pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);

		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &red, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	}
	if (green) {
		reg_index = 0;
		/* === set green pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &green, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	}
#ifdef LP5562_BLUE_LED
	if (blue) {
		reg_index = 0;
		/* === set blue pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);

		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &blue, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
	}
#endif
	/* === run program === */
	data = mode & 0x2a;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (mode & 0x2a)|0x40;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_dual_color_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;

	I(" %s +++\n" , __func__);
	mutex_lock(&led_mutex);
	data = 0x14;
	ret = write_operation_register(client, data, 0);
	udelay(200);

	reg_index = 0;
	/* === set pwm to 200 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.064s === */
	data = 0x44;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === set pwm to 0 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.25s === */
	data = 0x50;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === trigger sg, wg === */
	data = 0xe1;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x04;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === clear register === */
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	udelay(550);

	/* === trigger wr === */
	reg_index = 0;
	data = 0xe0;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x80;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	udelay(550);
	/* set pwm to 200 */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait 0.064s === */
	data = 0x44;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === set pwm to 0 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait 0.999s === */
	data = 0x7f;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait 0.622s === */
	data = 0x68;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === trigger sr === */
	data = 0xe0;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x02;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === clear register === */
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	udelay(550);

	/* === run program === */

	data = 0x28;
	ret = write_operation_register(client, data, 0);
	udelay(200);

	data = 0x68;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_green_blink (struct i2c_client *client,  uint8_t green)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;

	I(" %s +++ \n" , __func__);
	mutex_lock(&led_mutex);

	data = 0x04;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
	udelay(200);
	ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);
#endif
	if (green) {
		reg_index = 0;
		/* === set green pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &green, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	}

	/* === run program === */
	data = 0x08;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = 0x08|0x40;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_led_off(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;
	char data1[1] = {0};

	I(" %s +++\n" , __func__);
	if (!chip_enable) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}
	ret = i2c_read_block(client, ENABLE_REGISTER, data1, 1);
	if (!data1[0]) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}

	mutex_lock(&led_mutex);
	/* === reset red green blue === */
	data = 0x00;
	ret = i2c_write_block(client, R_PWM_CONTROL, &data, 1);
	ret = i2c_write_block(client, G_PWM_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
	ret = i2c_write_block(client, B_PWM_CONTROL, &data, 1);
#endif
	ret = write_operation_register(client, data, 0);
	ret = write_enable_register(client, data, 0);
	mutex_unlock(&led_mutex);

	if(!rgb_enable && !vk_enable)
		lp5562_led_disable(client);

	I(" %s ---\n" , __func__);
}

#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
static void green_blink_mfg(int onoff){
	I(" %s , set display_flag = %d\n" , __func__, onoff);
	display_flag = onoff;
	if(display_flag){
		g_led_led_data->Mode = 2;
		g_led_led_data->Red = 0;
		g_led_led_data->Green = 0xc8;
		g_led_led_data->Blue = 0;
		queue_work(g_led_work_queue, &g_led_led_data->led_work_multicolor);
	}else {
		g_led_led_data->Mode = 0;
		g_led_led_data->Red = 0;
		g_led_led_data->Green = 0;
		g_led_led_data->Blue = 0;
		queue_work(g_led_work_queue, &g_led_led_data->led_work_multicolor);
	}
}
#endif

static void led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5562_led, led_work);
	lp5562_led_off(client);
	I(" %s ---\n" , __func__);
}

static void multicolor_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;
	int ret;
	uint8_t data = 0x00;
        char data1[1] = {0};
        int i;

	if(!client)
		return;

	ldata = container_of(work, struct lp5562_led, led_work_multicolor);
	I(" %s , Mode = %x\n" , __func__, ldata->Mode);

	if (ldata->Mode > 1 && ldata->Mode <= 6)
		lp5562_led_enable(client, 1);
	else if (ldata->Mode == 1)
		lp5562_led_enable(client, 0);
	if (ldata->Red) {
		rgb_enable = 1;
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, R_CURRENT_CONTROL, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, R_CURRENT_CONTROL, &data, 1);
	}
	if (ldata->Green) {
		rgb_enable = 1;
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, G_CURRENT_CONTROL, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, G_CURRENT_CONTROL, &data, 1);
	}
#ifdef LP5562_BLUE_LED
	if (ldata->Blue) {
		rgb_enable = 1;
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
	}
#endif
	if (ldata->Mode == 0 || (!ldata->Red && !ldata->Green && !ldata->Blue)) {
		rgb_enable = 0;
		lp5562_led_off(client);
	} else if (ldata->Mode == 1) {  /* === set red, green, blue direct control === */
		mutex_lock(&led_mutex);
		ret = i2c_write_block(client, R_PWM_CONTROL, &ldata->Red, 1);
		ret = i2c_write_block(client, G_PWM_CONTROL, &ldata->Green, 1);
#ifdef LP5562_BLUE_LED
		ret = i2c_write_block(client, B_PWM_CONTROL, &ldata->Blue, 1);
#endif
		data = 0x3f;
		ret = write_operation_register(client, data, 0);
		udelay(200);
		data = 0x40;
		ret = write_enable_register(client, data, 0);
		udelay(500);
		mutex_unlock(&led_mutex);
	} else if (ldata->Mode == 2) { /* === set short blink === */
		lp5562_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	} else if (ldata->Mode == 3) { /* === set delayed short blink === */
		msleep(1000);
		lp5562_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	} else if (ldata->Mode == 4 && ldata->Red && !ldata->Green && !ldata->Blue) { /* === set red long blink === */
		lp5562_red_long_blink(client);
	} else if (ldata->Mode ==5 && ldata->Red && ldata->Green && !ldata->Blue) { /* === set red green blink === */
		lp5562_dual_color_blink(client);
	} else if (ldata->Mode == 6) {
		lp5562_green_blink(client, ldata->Green);
	} else {
		for (i = 0; i <= 0x6f; i++) {
			ret = i2c_read_block(client, i, data1, 1);
			I(" %s i2c(%x) = 0x%x\n", __func__, i, data1[0]);
		}
	}
}
/*
static void led_alarm_handler(struct alarm *alarm)
{
	struct lp5562_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(alarm, struct lp5562_led, led_alarm);
	queue_work(g_led_work_queue, &ldata->led_work);
	I(" %s ---\n" , __func__);
}
*/
static void led_blink_do_work(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5562_led, blink_delayed_work.work);
	lp5562_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	I(" %s ---\n" , __func__);
}

static int virtual_key_led_change_pwm(struct i2c_client *client, int pwm_diff)
{
	int target_pwm_diff, ret = 0;
	uint8_t pc_addr1, pc_addr2, pwm_level, data;

	if(pwm_diff > 0){
		target_pwm_diff = pwm_diff;
		pc_addr1 = 0;
		pc_addr2 = 0;
	}
	else if (pwm_diff < 0){
		target_pwm_diff = -pwm_diff;
		pc_addr1 = 8;
		pc_addr2 = 8;
	}
	else {
		I("%s: pwm no changed, return.\n", __func__);
		return ret;
	}
	pwm_level = target_pwm_diff / VK_LED_FADE_LEVEL;
	if(pwm_level / 2) {
		pc_addr2 += (pwm_level / 2) - 1;
		pc_addr1 = pc_addr2 + (pwm_level % 2);
	}

	ret = i2c_write_block(client, ENG_3_PC_CONTROL, &pc_addr1, 1);
	data = 0x43;
	ret = write_enable_register(client, data, 1);
	if(pwm_level / 2) {
		msleep(VK_LED_SLEEP_TIME);
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &pc_addr2, 1);
		data = 0x43;
		ret = write_enable_register(client, data, 1);
	}
	msleep(VK_LED_SLEEP_TIME);

	return ret;
}

void virtual_key_led_reset_blink(int onoff)
{
	struct i2c_client *client = private_lp5562_client;
	int ret = 0, reg_index = 0;
	int target_pwm;
	uint8_t data;
	uint8_t command_data[10] = {0};

	if(!client)
		return;

	if (!plat_data->vk_use) {
		I("No virtual key led used\n");
		return;
	}

	I("virtual_key_led_reset_blink +++, onoff = %d\n", onoff);

	if(onoff) {
		virtual_key_led_ignore_flag = 1;
		lp5562_led_enable(client, 0);

		data = 0;
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);

		data = 0x01;
		ret = write_operation_register(client, data, 1);

		/*=== Set PWM to 0 ===*/
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = 0x00;

		/*=== wait for 300 ms ===*/
		command_data[reg_index++] = 0x54;
		command_data[reg_index++]  = 0x00;

		/*=== Set PWM to last_pwm ===*/
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = 0xFF;

		/*=== wait for 300 ms ===*/
		command_data[reg_index++] = 0x54;
		command_data[reg_index++]  = 0x00;

		/* === clear register === */
		command_data[reg_index++] = 0x00;
		command_data[reg_index++]  = 0x00;

		ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, 10);

		data = 0x02;
		ret = write_operation_register(client, data, 1);
		data = 0x42;
		ret = write_enable_register(client, data, 1);
	} else {
		virtual_key_led_ignore_flag = 0;
		data = 0x40;
		ret = write_enable_register(client, data, 1);
		write_vk_led_program(client);

		mutex_lock(&vk_led_mutex);
		target_pwm = VK_brightness / VK_LED_FADE_LEVEL * VK_LED_FADE_LEVEL;
		ret = virtual_key_led_change_pwm(client, (int)target_pwm);
		last_pwm = target_pwm;
		if(last_pwm)
			vk_enable = 1;
		else
			vk_enable = 0;
		mutex_unlock(&vk_led_mutex);

		if(!rgb_enable && !vk_enable)
			lp5562_led_disable(client);
	}
}

EXPORT_SYMBOL(virtual_key_led_reset_blink);

static void virtual_key_led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;
	int ret, reg_index = 0;
	int target_pwm;
	uint8_t data, change_current = 0;

	if(!client)
		return;

	ldata = container_of(work, struct lp5562_led, led_work);

	if(ldata->VK_brightness) {
		if(use_color_table && ldata->VK_brightness < table_level_num){
			if(use_current_table) {
				if(gVK_Current_param != current_table[ldata->VK_brightness] && ldata->VK_brightness != 0) {
					gVK_Current_param = current_table[ldata->VK_brightness];
					change_current = 1;
				}
			}
			ldata->VK_brightness = VK_brightness;
		}
		target_pwm = ldata->VK_brightness / VK_LED_FADE_LEVEL * VK_LED_FADE_LEVEL;
		if (vk_enable) {
		I(" %s virtual key already enable, change brightness\n" , __func__);

		if(change_current) {
			data = (u8)gVK_Current_param;
			ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
			ret = i2c_write_block(client, W_CURRENT_CONTROL, &data, 1);
		}
		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, (int)(target_pwm - last_pwm));
		last_pwm = target_pwm;
		mutex_unlock(&vk_led_mutex);
		return;
		}

		lp5562_led_enable(client, 0);
		data = (u8)gVK_Current_param;
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
		ret = i2c_write_block(client, W_CURRENT_CONTROL, &data, 1);
		write_vk_led_program(client);
		vk_enable = 1;
		reg_index = 0;

		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, (int)target_pwm);
		last_pwm = target_pwm;
		mutex_unlock(&vk_led_mutex);
	}else {
		if (!vk_enable) {
			I(" %s return, virtual key already disable\n" , __func__);
			return;
		}
		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, -last_pwm);
		last_pwm = 0;
		mutex_unlock(&vk_led_mutex);
		queue_delayed_work(g_led_work_queue, &ldata->blink_delayed_work, msecs_to_jiffies(VK_LED_SLEEP_TIME));
	}
}

static void led_fade_do_work(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;

	ldata = container_of((struct delayed_work *)work, struct lp5562_led, blink_delayed_work);

	if(!ldata->VK_brightness) {
		vk_enable = 0;
		if(!rgb_enable && !vk_enable)
			lp5562_led_disable(client);
	}
}



static ssize_t lp5562_charging_led_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", charging_flag);
}

static ssize_t lp5562_charging_led_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5562_client;
	uint8_t data = 0x00;
	int flag,ret;
	sscanf(buf, "%d", &flag);
	if(flag == 1 && flag != charging_flag) {
		charging_flag=flag;
		mutex_lock(&led_mutex);
		data = 0x00;
		ret = i2c_write_block(client, R_PWM_CONTROL, &data, 1);
		ret = i2c_write_block(client, G_PWM_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
		ret = i2c_write_block(client, B_PWM_CONTROL, &data, 1);
#endif
		ret = write_operation_register(client, data, 0);
		ret = write_enable_register(client, data, 0);
		ret = gpio_direction_output(plat_data->charging_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(plat_data->charging_gpio);
			return ret;
		}
	}
	else if(flag == 0 && flag != charging_flag) {
		charging_flag=flag;
		ret = gpio_direction_output(plat_data->charging_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(plat_data->charging_gpio);
			return ret;
		}
		mutex_unlock(&led_mutex);
	}
	return count;
}

static DEVICE_ATTR(charging_led_switch, 0640, lp5562_charging_led_switch_show,
		lp5562_charging_led_switch_store);


static ssize_t lp5562_led_off_timer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", current_time);;
}

static ssize_t lp5562_led_off_timer_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5562_led *ldata;
	int min, sec;
	uint16_t off_timer;
/*	ktime_t interval;
	ktime_t next_alarm;*/

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);
	I(" %s , min = %d, sec = %d\n" , __func__, min, sec);
	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5562_led, cdev);

	off_timer = min * 60 + sec;
	cancel_work_sync(&ldata->led_work);
	/*alarm_cancel(&ldata->led_alarm);
	if (off_timer) {
		interval = ktime_set(off_timer, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&ldata->led_alarm, next_alarm, next_alarm);
	}*/

	return count;
}

static DEVICE_ATTR(off_timer, 0644, lp5562_led_off_timer_show,
		lp5562_led_off_timer_store);

static ssize_t lp5562_led_multi_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", ModeRGB);
}

static ssize_t lp5562_led_multi_color_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5562_led *ldata;
	uint32_t val;
#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
	if(display_flag) {
		I(" %s , display_flag = %d, return\n" , __func__, display_flag);
		return count;
	}
#endif
	sscanf(buf, "%x", &val);

	if (val > 0xFFFFFFFF)
		return -EINVAL;
	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5562_led, cdev);
	wake_lock_timeout(&(ldata->led_wake_lock), 2*HZ);
	ldata->Mode = (val & Mode_Mask) >> 24;
	ldata->Red = (val & Red_Mask) >> 16;
	ldata->Green = (val & Green_Mask) >> 8;
#ifdef LP5562_BLUE_LED
	ldata->Blue = val & Blue_Mask;
#endif
	ModeRGB = val;
	I(" %s , ModeRGB = %x\n" , __func__, val);
	queue_work(g_led_work_queue, &ldata->led_work_multicolor);
	return count;
}

static DEVICE_ATTR(ModeRGB, 0644, lp5562_led_multi_color_show,
		lp5562_led_multi_color_store);

enum led_brightness lp5562_led_get_brightness(struct led_classdev *led_cdev)
{
	return ModeRGB;
}

static void lp5562_led_set_brightness(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	return;
}

enum led_brightness lp5562_vk_led_get_brightness(struct led_classdev *led_cdev)
{
	return VK_brightness;
}

static void lp5562_vk_led_set_brightness(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	struct lp5562_led *ldata;

	ldata = container_of(led_cdev, struct lp5562_led, cdev);

	ldata->VK_brightness = brightness == LED_FULL? 256 : brightness;

	if(use_color_table && brightness < table_level_num){
		I("color_table[%d] = %d, current_table[%d] = %d\n", brightness, color_table[brightness], brightness, current_table[brightness]);
		brightness = color_table[brightness];
	}

	VK_brightness = brightness == LED_FULL? 256 : brightness;
	I(" %s , VK_brightness = %u\n" , __func__, VK_brightness);

	if(!virtual_key_led_ignore_flag)
		queue_work(g_led_work_queue, &ldata->led_work);
	return;
}

/* === read/write i2c and control enable pin for debug === */
static ssize_t lp5562_led_i2c_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	char data[1] = {0};
	int i;
	struct i2c_client *client = private_lp5562_client;

	if(!client)
		return 0;

	for (i = 0; i <= 0x6f; i++) {
		ret = i2c_read_block(client, i, data, 1);
		I(" %s i2c(%x) = 0x%x\n", __func__, i, data[0]);
	}
	return ret;
}

static ssize_t lp5562_led_i2c_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5562_client;
	int i, ret;
	char *token[10];
	unsigned long ul_reg, ul_data = 0;
	uint8_t reg = 0, data;
	char value[1] = {0};
	struct led_i2c_platform_data *pdata;

	if(!client)
		return count;

	pdata = client->dev.platform_data;

	for (i = 0; i < 2; i++) {
		token[i] = strsep((char **)&buf, " ");
		D("%s: token[%d] = %s\n", __func__, i, token[i]);
	}
	ret = kstrtoul(token[0], 16, &ul_reg);
	ret = kstrtoul(token[1], 16, &ul_data);

	reg = ul_reg;
	data = ul_data;

	if (reg < 0x6F) {
		ret = i2c_write_block(client, reg, &data, 1);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Set REG=0x%x, data=0x%x\n" , __func__, ret, reg, data);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Get REG=0x%x, data=0x%x\n" , __func__, ret, reg, value[0]);
	}
	if (reg == 0x99) {
		if (data == 1) {
			I("%s , pull up enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		} else if (data == 0) {
			I("%s , pull down enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		}
	}
	return count;
}

static DEVICE_ATTR(i2c, 0644, lp5562_led_i2c_show, lp5562_led_i2c_store);


static int lp5562_pinctrl_init(struct lp5562_chip *cdata, struct device *dev){

	int retval, ret;

	/* Get pinctrl if target uses pinctrl */
	D("[LED]LP5562_pinctrl_init");

	cdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(cdata->pinctrl)) {
		retval = PTR_ERR(cdata->pinctrl);
		pr_err("[LED][lp5562 error]%s: Target does not use pinctrl\n", __func__);
		cdata->pinctrl = NULL;
		goto err_pinctrl_get;
	}
	cdata->gpio_state_init = pinctrl_lookup_state(cdata->pinctrl, "lp5562_init");
	if (IS_ERR_OR_NULL(cdata->gpio_state_init)) {
		pr_err("[LED][lp5562 error]%s: Cannot get pintctrl state\n", __func__);
		retval = PTR_ERR(cdata->gpio_state_init);
		cdata->pinctrl = NULL;
		return retval;
	}
	ret = pinctrl_select_state(cdata->pinctrl, cdata->gpio_state_init);
	if (ret) {
		pr_err("[LED][LP5562 error]%s: Cannot init INT gpio\n", __func__);
		return ret;
	}

	return 0;

err_pinctrl_get:
	cdata->pinctrl = NULL;
	return retval;
}

#define CG_ID_LEN 5
#define BLACK_ID 1
#define WHITE_ID 2

static void get_brightness_mapping_table(struct device_node *node)
{
	struct property *prop;
	int current_table_level_num;
	int color_ID = BLACK_ID;
	int touch_solution = g_led_touch_solution;
	const char* cmdline;
	char* temp_cmdline;

	prop = of_find_property(node, "vk-pwm-array",
			&table_level_num);
	if(!prop) {
		I("Not use color mapping table\n");
		return;
	}
    I("%s, vk-pwm-array table_level_num: %d\n", __func__, table_level_num);
	use_color_table = 1;
	memcpy(color_table, prop->value, table_level_num);

	cmdline = kstrdup(saved_command_line, GFP_KERNEL);
	if (cmdline) {
		I("Get cmdline success\n");
		temp_cmdline = strstr(cmdline, "color_ID=");
		if(temp_cmdline == NULL) {
			I("No color_ID at devices\n");
			kfree(cmdline);
		} else {
			temp_cmdline += strlen("color_ID=");
			temp_cmdline[CG_ID_LEN] = '\0';
			if(of_property_match_string(node, "vk-black-cg-id-def", temp_cmdline) >= 0) {
				color_ID = BLACK_ID;
				I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", temp_cmdline, color_ID, touch_solution);
			} else if(of_property_match_string(node, "vk-white-cg-id-def", temp_cmdline) >= 0) {
				color_ID = WHITE_ID;
				I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", temp_cmdline, color_ID, touch_solution);
			} else {
				I("No color_ID matched\n");
			}
			kfree(cmdline);
		}
	} else {
		I("Get cmdline failed\n");
	}

	if(color_ID == BLACK_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-black-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-black-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	} else if(color_ID == WHITE_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-white-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-white-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	}

	if(touch_solution == SEC_TOUCH_SOLUTION) {
		prop = of_find_property(node, "vk-current-array-sec",
						&current_table_level_num);
	} else {
		prop = of_find_property(node, "vk-current-array-def",
						&current_table_level_num);
	}
	if(!prop) {
		use_current_table = 0;
	} else {
		use_current_table = 1;
		memcpy(current_table, prop->value, current_table_level_num);
	}
}

#ifdef CONFIG_LEDS_SYNC_TOUCH_SOLUTION
void set_led_touch_solution(uint16_t solution)
{
	g_led_touch_solution = solution;

	if(private_lp5562_client == NULL) {
		I("%s, virtual key led probe not ready\n", __func__);
		return;
	}
	I("%s, led_touch_solution = %d\n", __func__, g_led_touch_solution);

	get_brightness_mapping_table(private_lp5562_client->dev.of_node);
}
EXPORT_SYMBOL(set_led_touch_solution);
#endif

static ssize_t led_color_ID_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct device_node *node;
	struct property *prop;
	char color_ID_name[6];
	int current_table_level_num;
	int color_ID = BLACK_ID;
	int touch_solution = g_led_touch_solution;

	if(private_lp5562_client == NULL) {
		return count;
	}

	node = private_lp5562_client->dev.of_node;

	memcpy(color_ID_name, buf, CG_ID_LEN);
	color_ID_name[CG_ID_LEN] = '\0';

    I("%s, Update color mapping talbe of color_ID: %s\n", __func__, color_ID_name);

	if(of_property_match_string(node, "vk-black-cg-id-def", color_ID_name) >= 0) {
		color_ID = BLACK_ID;
		I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", color_ID_name, color_ID, touch_solution);
	} else if(of_property_match_string(node, "vk-white-cg-id-def", color_ID_name) >= 0) {
		color_ID = WHITE_ID;
		I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", color_ID_name, color_ID, touch_solution);
	} else {
		I("No color_ID matched\n");
		return count;
	}

	if(color_ID == BLACK_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-black-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-black-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	} else if(color_ID == WHITE_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-white-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-white-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	}

	if(touch_solution == SEC_TOUCH_SOLUTION) {
		prop = of_find_property(node, "vk-current-array-sec",
				&current_table_level_num);
	} else {
		prop = of_find_property(node, "vk-current-array-def",
				&current_table_level_num);
	}
	if(!prop) {
		use_current_table = 0;
	} else {
		use_current_table = 1;
		memcpy(current_table, prop->value, current_table_level_num);
	}
	return count;
}

static DEVICE_ATTR(set_color_ID, 0200, NULL, led_color_ID_store);


static int lp5562_parse_dt(struct device *dev, struct led_i2c_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	int rc = 3;
	prop = of_find_property(dt, "lp5562,lp5562_en", NULL);
	if (prop) {
		pdata->ena_gpio = of_get_named_gpio(dt, "lp5562,lp5562_en", 0);
	}
	prop = of_find_property(dt, "lp5562,charging_en", NULL);
	if (prop) {
		pdata->charging_gpio = of_get_named_gpio(dt, "lp5562,charging_en", 0);
	}

	pdata->tp_3v3_en = 0;
	prop = of_find_property(dt, "lp5562,LED_3v3_en", NULL);
	if (prop) {
		pdata->tp_3v3_en = of_get_named_gpio(dt, "lp5562,LED_3v3_en", 0);
		rc = gpio_request(pdata->tp_3v3_en, "led_3v3");
		if(rc < 0) {
			pr_err("[LED] gpio_request failed led_3v3 gpio %d\n", rc);
		}

		rc = gpio_direction_output(pdata->tp_3v3_en, 1);
		if(rc <0) {
			pr_err("[LED] gpio_direction_output led_3v3 failed %d\n", rc);
		}
		rc = gpio_get_value(pdata->tp_3v3_en);
		printk("[LED][PARSE] %d, gpio_dir gpio_get_value= %d \n",pdata->tp_3v3_en, rc);
	}

	prop = of_find_property(dt, "lp5562,num_leds", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5562,num_leds", &pdata->num_leds);
	}
	prop = of_find_property(dt, "lp5562,current_param", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5562,current_param", &gCurrent_param);
	}
	if (prop) {
		of_property_read_u32(dt, "lp5562,vk_current_param", &gVK_Current_param);
	}
	pdata->vk_use = of_property_read_bool(dt, "lp5562,vk_use");
	return 0;
}

static int lp5562_led_probe(struct i2c_client *client
		, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lp5562_chip		*cdata;
	struct led_i2c_platform_data *pdata;
	int ret =0;
	int i;
	u8 check_chip_used;


	printk("[LED][PROBE] led driver probe +++\n");

	/* === init platform and client data === */
	cdata = kzalloc(sizeof(struct lp5562_chip), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		printk("[LED][PROBE_ERR] failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);
	cdata->client = client;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}
	ret = lp5562_parse_dt(&client->dev, pdata);

	/*=== led pinctrl init ===*/
	if (lp5562_pinctrl_init(cdata, &client->dev) < 0) {
		pr_err("[LED] pinctrl setup failed");
	}

	led_rw_delay = 5;
	/* === led enable pin === */
	if (pdata->ena_gpio) {
		ret = gpio_request(pdata->ena_gpio, "led_enable");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed ena gpio %d\n", __func__, ret);
			goto err_request_ena_gpio;
		}
		ret = gpio_direction_output(pdata->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->ena_gpio);
			goto err_request_ena_gpio;
		}
		msleep(1);
	} /*else if (pdata->ena_gpio_io_ext) {
	    ret = ioext_gpio_set_value(pdata->ena_gpio_io_ext, 1);
	    if (ret < 0) {
	    pr_err("[LED] %s: io extender high failed %d\n", __func__, ret);
	    gpio_free(pdata->ena_gpio);
	    }
	    }*/
	/* === charging led switch pin === */
	if (pdata->charging_gpio) {
		ret = gpio_request(pdata->charging_gpio, "charging_led_switch");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed charging switch %d\n", __func__, ret);
		}
	}
	/* === led trigger signal pin === */
	if (pdata->tri_gpio) {
		ret = gpio_request(pdata->tri_gpio, "led_trigger");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed led trigger %d\n", __func__, ret);
		}
		ret = gpio_direction_output(pdata->tri_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->tri_gpio);
		}
	}
	private_lp5562_client = client;

	ret = i2c_read_block(client, ENABLE_REGISTER, &check_chip_used, 1);
	if(ret < 0) {
		I("Not use LP5562 LED.\n");
		goto err_check_chip_not_used;
	}

	g_led_work_queue = create_singlethread_workqueue("led");
	if (!g_led_work_queue) {
		ret = -10;
		pr_err("[LED] %s: create workqueue fail %d\n", __func__, ret);
		goto err_create_work_queue;
	}
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == VIRTUAL_KEY_LED_ID && pdata->vk_use) {
			I("VK probe, i = %d, num_leds = %d\n", i, pdata->num_leds);
			cdata->leds[i].cdev.name = "button-backlight";
			ret = led_classdev_register(dev, &cdata->leds[i].cdev);
			if (ret < 0) {
				dev_err(dev, "couldn't register led[%d]\n", i);
				goto err_register_button_backlight_dev;
			}

			get_brightness_mapping_table(client->dev.of_node);

			cdata->leds[i].cdev.brightness_set = lp5562_vk_led_set_brightness;
			cdata->leds[i].cdev.brightness_get = lp5562_vk_led_get_brightness;
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_set_color_ID);

			INIT_WORK(&cdata->leds[i].led_work, virtual_key_led_work_func);
			//INIT_WORK(&cdata->leds[i].led_work_multicolor, virtual_key_led_blink_work_func);
			INIT_DELAYED_WORK(&cdata->leds[i].blink_delayed_work, led_fade_do_work);
			mutex_init(&vk_led_mutex);
		} else if (i == INDICATOR_LED_ID) {
			cdata->leds[i].cdev.name = "indicator";
			ret = led_classdev_register(dev, &cdata->leds[i].cdev);
			if (ret < 0) {
				dev_err(dev, "couldn't register led[%d]\n", i);
				goto err_create_work_queue;
			}

			cdata->leds[i].cdev.brightness_set = lp5562_led_set_brightness;
			cdata->leds[i].cdev.brightness_get = lp5562_led_get_brightness;

			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_ModeRGB);
			if (ret < 0) {
				pr_err("%s: failed on create attr ModeRGB [%d]\n", __func__, i);
				goto err_register_attr_ModeRGB;
			}

			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_off_timer);
			if (ret < 0) {
				pr_err("%s: failed on create attr off_timer [%d]\n", __func__, i);
				goto err_register_attr_off_timer;
			}
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_i2c);
			if (ret < 0) {
				pr_err("%s: failed on create attr i2c [%d]\n", __func__, i);
				goto err_register_attr_i2c;
			}
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_charging_led_switch);
			if (ret < 0) {
				pr_err("%s: failed on create attr charging_led_switch [%d]\n", __func__, i);
				goto err_register_attr_charging_led_switch;
			}
			wake_lock_init(&cdata->leds[i].led_wake_lock, WAKE_LOCK_SUSPEND, "lp5562");
			INIT_WORK(&cdata->leds[i].led_work, led_work_func);
			INIT_WORK(&cdata->leds[i].led_work_multicolor, multicolor_work_func);
			INIT_DELAYED_WORK(&cdata->leds[i].blink_delayed_work, led_blink_do_work);
			/*alarm_init(&cdata->leds[i].led_alarm,
					ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
					led_alarm_handler);*/
#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
			g_led_led_data = &cdata->leds[i];
#endif
		}
	}
	mutex_init(&cdata->led_i2c_rw_mutex);
	mutex_init(&led_mutex);
	plat_data = pdata;
	/* Avoid led turn off when entering kernel */
#if 0
	/* === disable CHIP_EN === */
	data = 0x00;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	if (pdata->ena_gpio) {
		gpio_direction_output(pdata->ena_gpio, 0);
	} /*else if (pdata->ena_gpio_io_ext) {
	    ioext_gpio_set_value(pdata->ena_gpio_io_ext, 0);
	    }*/
#endif
	printk("[LED][PROBE] led driver probe ---\n");

#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
	if(!htc_check_panel_connection()){
		if(pdata->tp_3v3_en)
			ret = gpio_direction_output(pdata->tp_3v3_en, 1);
		green_blink_mfg(1);
	}
#endif
	return 0;

err_register_button_backlight_dev:
	wake_lock_destroy(&cdata->leds[INDICATOR_LED_ID].led_wake_lock);
err_register_attr_charging_led_switch:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_charging_led_switch);
	}
err_register_attr_i2c:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_i2c);
	}
err_register_attr_off_timer:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
	}
err_register_attr_ModeRGB:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_ModeRGB);
	}

	led_classdev_unregister(&cdata->leds[INDICATOR_LED_ID].cdev);
err_create_work_queue:
err_check_chip_not_used:
	if (pdata->ena_gpio) {
		gpio_direction_output(pdata->ena_gpio, 0);
		gpio_free(pdata->ena_gpio);
	}
	if (pdata->charging_gpio)
		gpio_free(pdata->charging_gpio);
	if (pdata->tri_gpio)
		gpio_free(pdata->tri_gpio);
	private_lp5562_client = NULL;
err_request_ena_gpio:
	kfree(pdata);
err_exit:
	kfree(cdata);
err_cdata:
	return ret;
}

static int lp5562_led_remove(struct i2c_client *client)
{
	struct lp5562_chip *cdata;
	int i,ret;

	cdata = i2c_get_clientdata(client);

	ret = lp5562_parse_dt(&client->dev, plat_data);
	if (plat_data->ena_gpio) {
		gpio_direction_output(plat_data->ena_gpio, 0);
	} /*else if (pdata->ena_gpio_io_ext) {
	    ioext_gpio_set_value(pdata->ena_gpio_io_ext, 0);
	    }*/
	for (i = 0; i < plat_data->num_leds; i++) {
		if(i == INDICATOR_LED_ID) {
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_ModeRGB);
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_i2c);
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_charging_led_switch);
		}
		led_classdev_unregister(&cdata->leds[i].cdev);
	}
	destroy_workqueue(g_led_work_queue);
	kfree(cdata);

	return 0;
}


static const struct i2c_device_id led_i2c_id[] = {
	{ "LP5562-LED", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, led_i2c_id);

static const struct of_device_id lp5562_mttable[] = {
	{ .compatible = "LP5562-LED"},
	{ },
};

static struct i2c_driver led_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "LP5562-LED",
		.of_match_table = lp5562_mttable,
	},
	.id_table = led_i2c_id,
	.probe = lp5562_led_probe,
	.remove = lp5562_led_remove,
};
module_i2c_driver(led_i2c_driver);
/*
   static int __init lp5562_led_init(void)
   {
   int ret;

   ret = i2c_add_driver(&led_i2c_driver);
   if (ret)
   return ret;
   return 0;
   }

   static void __exit lp5562_led_exit(void)
   {
   i2c_del_driver(&led_i2c_driver);
   }

   module_init(lp5562_led_init);
   module_exit(lp5562_led_exit);
 */
MODULE_AUTHOR("<ShihHao_Shiung@htc.com>, <Dirk_Chang@htc.com>");
MODULE_DESCRIPTION("LP5562 LED driver");

