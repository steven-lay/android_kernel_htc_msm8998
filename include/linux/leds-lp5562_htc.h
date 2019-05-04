#ifndef _LINUX_LP5562_HTC_H
#define _LINUX_LP5562_HTC_H

#define LED_I2C_NAME "LP5562-LED"

#define ENABLE_REGISTER 	0x00
#define OPRATION_REGISTER	0x01
#define CONFIG_REGISTER		0x08
#define R_PWM_CONTROL	 	0x02
#define G_PWM_CONTROL 		0x03
#define B_PWM_CONTROL 		0x04
#define W_PWM_CONTROL 		0x0E
#define R_CURRENT_CONTROL	0x07
#define G_CURRENT_CONTROL 	0x06
#define B_CURRENT_CONTROL 	0x05
#define W_CURRENT_CONTROL 	0x0F
#define CMD_ENG_1_BASE		0x10
#define CMD_ENG_2_BASE		0x30
#define CMD_ENG_3_BASE		0x50
#define ENG_1_PC_CONTROL	0x09
#define ENG_2_PC_CONTROL	0x0A
#define ENG_3_PC_CONTROL	0x0B
#define LED_MAP_CONTROL		0x70
#define RESET_CONTROL		0x0D


#define I2C_WRITE_RETRY_TIMES		2
#define LED_I2C_WRITE_BLOCK_SIZE	80

struct led_i2c_config {
	const char *name;
};

struct led_i2c_platform_data {
	struct led_i2c_config *led_config;
	int num_leds;
	int ena_gpio;
	int ena_gpio_io_ext;
	int tri_gpio;
	int charging_gpio;
	int button_lux;
	int red_charge;
	int green_charge;
	int tp_3v3_en;
	bool vk_use;
};



void led_behavior(struct i2c_client *client, int val);
void lp5562_led_current_set_for_key(int brightness_key);

#endif /*_LINUXLP5562-LED_H*/

