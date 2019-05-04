/*
 * TI LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_data/lp855x.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>

/* HTC ADD */
#include <linux/regmap.h>
#include <linux/htc_flashlight.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/fb.h>

/* LP8550/1/2/3/6 Registers */
#define LP855X_BRIGHTNESS_CTRL		0x00
#define LP855X_DEVICE_CTRL		0x01
#define LP855X_EEPROM_START		0xA0
#define LP855X_EEPROM_END		0xA7
#define LP8556_EPROM_START		0xA0
#define LP8556_EPROM_END		0xAF

/* LP8555/7 Registers */
#define LP8557_BL_CMD			0x00
#define LP8557_BL_MASK			0x01
#define LP8557_BL_ON			0x01
#define LP8557_BL_OFF			0x00
#define LP8557_BRIGHTNESS_CTRL		0x04
#define LP8557_CONFIG			0x10
#define LP8555_EPROM_START		0x10
#define LP8555_EPROM_END		0x7A
#define LP8557_EPROM_START		0x10
#define LP8557_EPROM_END		0x1E

#define DEFAULT_BL_NAME		"lcd-backlight"
#define MAX_BRIGHTNESS		255

/* HTC ADD */
#define MAX_BRIGHTNESS_12BIT				4095
#define MAX_CURRENT_20MA					0x30
#define MAX_CURRENT_30MA					0x60
#define LP8556_BACKLIGHT_12BIT_LSB_MASK		0xFF
#define LP8556_BACKLIGHT_12BIT_MSB_MASK		0x0F
#define LP8556_BACKLIGHT_12BIT_MSB_SHIFT	8
#define LP8556_MAX_CURRENT_MASK				(BIT(4) | BIT(5) | BIT(6))
#define LP8556_CFG1_REG						0xA1
#define LP8556_FULLBRIGHT_LSB_REG			0x10
#define LP8556_FULLBRIGHT_MSB_REG			0x11
#define LP8556_MAX_REGISTER					0xAF

#define FLASH_MODE_BRIGHTNESS				4095
#define FLASH_MODE_MAX_DURATION_MS			500
#define TORCH_MODE_MAX_DURATION_MS			10000

static int current_fb_status = FB_BLANK_UNBLANK;

enum lp855x_brightness_ctrl_mode {
	PWM_BASED = 1,
	REGISTER_BASED,
};

enum lp855x_power_control_action{
	NORMAL_POWER_ON,
	NORMAL_POWER_OFF,
	AOD_POWER_ON,
	AOD_ENABLE,
	ADO_DISABLE,
};

struct lp855x;

/* HTC ADD  regcmd function struct*/
struct lp855x_regcmd{
	unsigned int address;
	unsigned int parameter1;
};

struct lp855x_regcmd_sets{
	int reg_cmds_num;
	struct lp855x_regcmd *reg_cmds;
	struct regmap *regmap;
};

/*
 * struct lp855x_device_config
 * @pre_init_device: init device function call before updating the brightness
 * @reg_brightness: register address for brigthenss control
 * @reg_devicectrl: register address for device control
 * @post_init_device: late init device function call
 */
struct lp855x_device_config {
	int (*pre_init_device)(struct lp855x *);
	u8 reg_brightness;
	u8 reg_devicectrl;
	int (*post_init_device)(struct lp855x *);
};

struct lp855x {
	const char *chipname;
	enum lp855x_chip_id chip_id;
	enum lp855x_brightness_ctrl_mode mode;
	struct lp855x_device_config *cfg;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct lp855x_platform_data *pdata;
	struct pwm_device *pwm;
	struct regulator *supply;	/* regulator for VDD input */

	/* HTC ADD */
	struct regmap *regmap;
	struct delayed_work flash_work;
	bool flash_enabled;
	struct htc_flashlight_dev flash_dev;
	int torch_brightness;
	int gpio;
	bool gpio_enabled;
	struct notifier_block lp855x_fb_notif;
	struct lp855x_regcmd_sets power_on_cmd_sets;
	struct lp855x_regcmd_sets aod_power_on_cmd_sets;
	struct lp855x_regcmd_sets aod_enable_cmd_sets;
	struct lp855x_regcmd_sets aod_disable_cmd_sets;
};


/* HTC ADD regcmd function --- start*/
static void lp855x_exec_cmds(struct regmap *regmap, struct lp855x_regcmd *cmds, int num)
{
	int i = 0, ret = 0;
	struct lp855x_regcmd *m_cmd = cmds;


	if (!regmap){
		pr_err("%s, regmap is NULL  \n", __func__);
		return ;
	}

	if (!cmds){
		pr_err("%s, regcmd is NULL  \n", __func__);
		return ;
	}

	for (i=0; i<num; i++){
		ret = regmap_write(regmap, m_cmd->address, m_cmd->parameter1);
		if (ret){
			pr_err("%s: I2C write NG:address: %x, parameter1: %x\n", __func__, m_cmd->address, m_cmd->parameter1);
		}else{
			pr_debug("%s: I2C write OK:address: %x, parameter1: %x\n", __func__, m_cmd->address, m_cmd->parameter1);
		}
		m_cmd++;
	}
}

static struct lp855x_regcmd *lp855x_parse_dt_reg_cmds(struct device *dev, const char *cmd_key, int *cmd_num)
{

	const char *data;
	struct lp855x_regcmd *m_cmds, *p_cmd;
	unsigned char *buf, *ptr_buf;
	int blen = 0, sets, i;

	data = of_get_property(dev->of_node, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return NULL;
	}

	ptr_buf= buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
		if (!buf)
			return NULL;

	memcpy(buf, data, blen);

	sets = blen/2;
	m_cmds = devm_kzalloc(dev,sizeof(*m_cmds) * sets, GFP_KERNEL);
	if (!m_cmds)
		goto error;

	p_cmd = m_cmds;
	for (i=0;i<sets;i++){
		p_cmd->address = *ptr_buf++;
		p_cmd->parameter1 = *ptr_buf++;
		pr_err("%s ,parse cmd-> address:%x , parameter1:%x\n", __func__, p_cmd->address, p_cmd->parameter1);
		p_cmd++;
	}

	*cmd_num = sets;

	kfree(buf);
	return m_cmds;

error:
	kfree(buf);
	return NULL;
}

static void lp855x_parse_dt_reg_cmd_sets(struct device *dev, struct lp855x_regcmd_sets *cmd_sets, struct regmap *regmap, const char *cmd_key)
{

	if (!cmd_sets){
		dev_err(dev, "%s: cmd_sets is null \n", __func__);
		return ;
	}

	if (!regmap){
		dev_err(dev, "%s: regmap is null \n", __func__);
		return ;
	}

	cmd_sets->regmap = regmap;
	cmd_sets->reg_cmds = lp855x_parse_dt_reg_cmds(dev, cmd_key, &cmd_sets->reg_cmds_num);

}

static void lp855x_run_power_on_cmds(struct lp855x_regcmd_sets *cmd_sets)
{
	if (cmd_sets){
		if (cmd_sets->reg_cmds_num && cmd_sets->reg_cmds){
			lp855x_exec_cmds(cmd_sets->regmap, cmd_sets->reg_cmds, cmd_sets->reg_cmds_num);
		}
	}
}
/* HTC ADD regcmd function --- end*/

static int lp855x_write_byte(struct lp855x *lp, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(lp->client, reg, data);
}

static int lp855x_update_bit(struct lp855x *lp, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = i2c_smbus_read_byte_data(lp->client, reg);
	if (ret < 0) {
		dev_err(lp->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}

	tmp = (u8)ret;
	tmp &= ~mask;
	tmp |= data & mask;

	return lp855x_write_byte(lp, reg, tmp);
}

static bool lp855x_is_valid_rom_area(struct lp855x *lp, u8 addr)
{
	u8 start, end;

	switch (lp->chip_id) {
	case LP8550:
	case LP8551:
	case LP8552:
	case LP8553:
		start = LP855X_EEPROM_START;
		end = LP855X_EEPROM_END;
		break;
	case LP8556:
		start = LP8556_EPROM_START;
		end = LP8556_EPROM_END;
		break;
	case LP8555:
		start = LP8555_EPROM_START;
		end = LP8555_EPROM_END;
		break;
	case LP8557:
		start = LP8557_EPROM_START;
		end = LP8557_EPROM_END;
		break;
	default:
		return false;
	}

	return addr >= start && addr <= end;
}

static int lp8557_bl_off(struct lp855x *lp)
{
	/* BL_ON = 0 before updating EPROM settings */
	return lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK,
				LP8557_BL_OFF);
}

static int lp8557_bl_on(struct lp855x *lp)
{
	/* BL_ON = 1 after updating EPROM settings */
	return lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK,
				LP8557_BL_ON);
}

static struct lp855x_device_config lp855x_dev_cfg = {
	.reg_brightness = LP855X_BRIGHTNESS_CTRL,
	.reg_devicectrl = LP855X_DEVICE_CTRL,
};

static struct lp855x_device_config lp8557_dev_cfg = {
	.reg_brightness = LP8557_BRIGHTNESS_CTRL,
	.reg_devicectrl = LP8557_CONFIG,
	.pre_init_device = lp8557_bl_off,
	.post_init_device = lp8557_bl_on,
};

/*
 * Device specific configuration flow
 *
 *    a) pre_init_device(optional)
 *    b) update the brightness register
 *    c) update device control register
 *    d) update ROM area(optional)
 *    e) post_init_device(optional)
 *
 */
static int lp855x_configure(struct lp855x *lp)
{
	u8 val, addr;
	int i, ret;
	struct lp855x_platform_data *pd = lp->pdata;

	switch (lp->chip_id) {
	case LP8550:
	case LP8551:
	case LP8552:
	case LP8553:
	case LP8556:
		lp->cfg = &lp855x_dev_cfg;
		break;
	case LP8555:
	case LP8557:
		lp->cfg = &lp8557_dev_cfg;
		break;
	default:
		return -EINVAL;
	}

	if (lp->cfg->pre_init_device) {
		ret = lp->cfg->pre_init_device(lp);
		if (ret) {
			dev_err(lp->dev, "pre init device err: %d\n", ret);
			goto err;
		}
	}

	val = pd->initial_brightness;
	ret = lp855x_write_byte(lp, lp->cfg->reg_brightness, val);
	if (ret)
		goto err;

	val = pd->device_control;
	ret = lp855x_write_byte(lp, lp->cfg->reg_devicectrl, val);
	if (ret)
		goto err;

	if (pd->size_program > 0) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret = lp855x_write_byte(lp, addr, val);
			if (ret)
				goto err;
		}
	}

	if (lp->cfg->post_init_device) {
		ret = lp->cfg->post_init_device(lp);
		if (ret) {
			dev_err(lp->dev, "post init device err: %d\n", ret);
			goto err;
		}
	}

	return 0;

err:
	return ret;
}

static void lp855x_pwm_ctrl(struct lp855x *lp, int br, int max_br)
{
	unsigned int period = lp->pdata->period_ns;
	unsigned int duty = br * period / max_br;
	struct pwm_device *pwm;

	/* request pwm device with the consumer name */
	if (!lp->pwm) {
		pwm = devm_pwm_get(lp->dev, lp->chipname);
		if (IS_ERR(pwm))
			return;

		lp->pwm = pwm;
	}

	pwm_config(lp->pwm, duty, period);
	if (duty)
		pwm_enable(lp->pwm);
	else
		pwm_disable(lp->pwm);
}


/* HTC Implmenetation */
static int lp8556_backlight_update_brightness_register(struct lp855x *lp, int brightness)
{
	int ret;
	struct regmap *regmap = lp->regmap;
	u8 val, max_current;

	if (lp->bl->props.max_brightness == MAX_BRIGHTNESS_12BIT)
	{
		if (brightness == MAX_BRIGHTNESS_12BIT)
			max_current = MAX_CURRENT_30MA; //Set maximum LED current to 30mA
		else
			max_current = MAX_CURRENT_20MA; //Set maximum LED current to 20mA

		ret = regmap_update_bits(regmap, LP8556_CFG1_REG, LP8556_MAX_CURRENT_MASK, max_current);

		if (ret)
			return ret;

		val = brightness & LP8556_BACKLIGHT_12BIT_LSB_MASK;
		ret = regmap_write(regmap, LP8556_FULLBRIGHT_LSB_REG, val);

		if (ret)
			return ret;

		val = (brightness >> LP8556_BACKLIGHT_12BIT_MSB_SHIFT) & LP8556_BACKLIGHT_12BIT_MSB_MASK;
		ret = regmap_write(regmap, LP8556_FULLBRIGHT_MSB_REG, val);

	}
	else
	{
		val = brightness & 0xFF;
		ret = regmap_write(regmap, LP855X_BRIGHTNESS_CTRL, val);
	}

	return ret;
}

static int lp855x_flash_en_locked(struct lp855x *lp, int en, int duration, int level)
{
	int rc = 0;

	if (en)
	{
		if (lp->flash_enabled)
		{
			pr_info("%s: already enabled\n", __func__);
			rc = -EBUSY;
			goto exit;
		}

		if (level <= 0 || level > lp->bl->props.max_brightness)
			level = lp->bl->props.max_brightness;

		if (duration < 10)
			duration = 10;

		schedule_delayed_work(&lp->flash_work, msecs_to_jiffies(duration));
	}
	else
	{
		level = lp->bl->props.brightness;
	}

	pr_info("%s: (%d, %d) [level=%d]\n", __func__, en, duration, level);

	rc = lp8556_backlight_update_brightness_register(lp, level);

	lp->flash_enabled = en;
exit:
	pr_info("%s: (%d, %d) done, rc=%d\n", __func__, en, duration, rc);

	return rc;
}

static int lp855x_flash_en(struct lp855x *lp, int en, int duration, int level)
{
	int rc = 0;

	if (!en) {
		cancel_delayed_work_sync(&lp->flash_work);
	}

	mutex_lock(&lp->bl->update_lock);
	rc = lp855x_flash_en_locked(lp, en, duration, level);
	mutex_unlock(&lp->bl->update_lock);

	return rc;
}

static void lp855x_flash_off_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct lp855x *lp = container_of(dwork, struct lp855x, flash_work);

	mutex_lock(&lp->bl->update_lock);
	lp855x_flash_en_locked(lp, 0, 0, 0);
	mutex_unlock(&lp->bl->update_lock);
}

static int lp855x_flash_mode(struct htc_flashlight_dev *fl_dev, int mode1, int mode2)
{
	struct lp855x *lp = container_of(fl_dev, struct lp855x, flash_dev);

	return lp855x_flash_en(lp, mode1, FLASH_MODE_MAX_DURATION_MS, FLASH_MODE_BRIGHTNESS);
}

static int lp855x_torch_mode(struct htc_flashlight_dev *fl_dev, int mode1, int mode2)
{
	struct lp855x *lp = container_of(fl_dev, struct lp855x, flash_dev);

	return lp855x_flash_en(lp, mode1, TORCH_MODE_MAX_DURATION_MS, lp->torch_brightness);
}

static ssize_t lp855x_get_flash_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lp->flash_enabled);
}

static ssize_t lp855x_set_flash_en(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	int data = 0, level = 0, rc = 0;

	if (sscanf(buf, "%d %d", &data, &level) < 1)
		return -EINVAL;

	rc = lp855x_flash_en(lp, !!data, data, level);

	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(flash_en, (S_IRUGO | S_IWUSR | S_IWGRP),
	lp855x_get_flash_en, lp855x_set_flash_en);
static struct attribute *lp855x_flash_attributes[] = {
	&dev_attr_flash_en.attr,
	NULL,
};

static const struct attribute_group lp855x_flash_attr_group = {
	.attrs = lp855x_flash_attributes,
};

/* HTC Implmenetation End */
static void lp855x_enable_control(struct lp855x *lp, int enabled){

	if(enabled){
	/* resume flow */
		if (gpio_is_valid(lp->gpio)){
			gpio_set_value(lp->gpio, 1);
			lp->gpio_enabled = 1;
			pr_debug("%s gpio:%d set value:%d\n", __func__, lp->gpio, 1);
		}
	}else{
	/* suspend flow */
		if (gpio_is_valid(lp->gpio)){
			gpio_set_value(lp->gpio, 0);
			lp->gpio_enabled = 0;
			pr_debug("%s gpio:%d set value:%d\n", __func__, lp->gpio, 0);
		}
	}

}



static void lp855x_fb_status_update(struct lp855x *lp, int next_fb_event)
{
	int lp855x_power_action = NORMAL_POWER_ON;

	pr_debug("%s current_fb_status:%d, next_fb_event:%d\n", __func__,current_fb_status, next_fb_event);

	if (next_fb_event == FB_BLANK_VSYNC_SUSPEND){
		/* bl power status should be the same as panel.
		Currently, FB_BLANK_NORMAL / FB_BLANK_VSYNC_SUSPEND
		keep panel power on and go to same call flow*/
		next_fb_event = FB_BLANK_NORMAL;
	}

	if (next_fb_event != FB_BLANK_NORMAL &&
		next_fb_event != FB_BLANK_UNBLANK &&
		next_fb_event != FB_BLANK_POWERDOWN){
		pr_err("%s : no supported fb event:%d \n", __func__, next_fb_event);
		return;
	}


	if (next_fb_event == current_fb_status){
		return;
	}

	switch (current_fb_status){
		case FB_BLANK_POWERDOWN:

			if (next_fb_event == FB_BLANK_NORMAL){
				lp855x_power_action = AOD_POWER_ON;
			}else if (next_fb_event == FB_BLANK_UNBLANK){
				lp855x_power_action = NORMAL_POWER_ON;
			}

			break;

		case FB_BLANK_NORMAL:

			if (next_fb_event == FB_BLANK_POWERDOWN){
				lp855x_power_action = NORMAL_POWER_OFF;
			}else if (next_fb_event == FB_BLANK_UNBLANK){
				lp855x_power_action = ADO_DISABLE;
			}

			break;
		case FB_BLANK_UNBLANK:

			if (next_fb_event == FB_BLANK_POWERDOWN){
				lp855x_power_action = NORMAL_POWER_OFF;
			}else if (next_fb_event == FB_BLANK_NORMAL){
				lp855x_power_action = AOD_ENABLE;
			}

			break;
	}

	current_fb_status = next_fb_event;

	/* base on power action to control power and resend initial code */
	switch (lp855x_power_action){
		case NORMAL_POWER_ON:
			pr_debug("%s power action: NORMAL_POWER_ON\n", __func__);
			lp855x_enable_control(lp, 1);
			usleep_range(1000, 1500);
			lp855x_run_power_on_cmds(&lp->power_on_cmd_sets);
			break;
		case NORMAL_POWER_OFF:
			pr_debug("%s power action: NORMAL_POWER_OFF\n", __func__);
			lp855x_enable_control(lp, 0);
			break;
		case AOD_POWER_ON:
			pr_debug("%s power action: AOD_POWER_ON\n", __func__);
			lp855x_enable_control(lp, 1);
			usleep_range(1000, 1500);
			lp855x_run_power_on_cmds(&lp->aod_power_on_cmd_sets);
			break;
		case AOD_ENABLE:
			pr_debug("%s power action: AOD_ENABLE\n", __func__);
			lp855x_run_power_on_cmds(&lp->aod_enable_cmd_sets);
			break;
		case ADO_DISABLE:
			pr_debug("%s power action: ADO_DISABLE\n", __func__);
			lp855x_run_power_on_cmds(&lp->aod_disable_cmd_sets);
			break;
	}

}


static int lp855x_bl_update_status(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int ret = 0;


	if (!lp->gpio_enabled){
		pr_debug("%s ignore this update due to gpio is not enabled\n", __func__);
		return ret;
	}

	if(lp->flash_enabled)
		return -EBUSY;

	if (lp->mode == PWM_BASED)
		lp855x_pwm_ctrl(lp, brightness, bl->props.max_brightness);
	else if (lp->mode == REGISTER_BASED && lp->chip_id == LP8556)
		ret = lp8556_backlight_update_brightness_register(lp, brightness);
	else
		lp855x_write_byte(lp, lp->cfg->reg_brightness, (u8)brightness);
	return ret;
}


static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct lp855x *lp = NULL;
	int node = evdata->info->node;
	int fb_blank = 0;

	/* Only listen FB_EVENT_BLANK from fb 0 */
	if (event != FB_EVENT_BLANK || node != 0)
		return 0;

	lp = container_of(self, struct lp855x, lp855x_fb_notif);
	fb_blank = *(int *)evdata->data;

	lp855x_fb_status_update(lp, fb_blank);

	lp855x_bl_update_status(lp->bl);

	return 0;
}

static const struct backlight_ops lp855x_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp855x_bl_update_status,
};

static int lp855x_backlight_register(struct lp855x *lp)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct lp855x_platform_data *pdata = lp->pdata;
	const char *name = pdata->name ? : DEFAULT_BL_NAME;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_PLATFORM;

	if (lp->chip_id == LP8556)
		props.max_brightness = MAX_BRIGHTNESS_12BIT;
	else
		props.max_brightness = MAX_BRIGHTNESS;

	if (pdata->initial_brightness > props.max_brightness)
		pdata->initial_brightness = props.max_brightness;

	props.brightness = pdata->initial_brightness;

	bl = devm_backlight_device_register(lp->dev, name, lp->dev, lp,
				       &lp855x_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	lp->bl = bl;

	return 0;
}

static ssize_t lp855x_get_chip_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lp->chipname);
}

static ssize_t lp855x_get_bl_ctl_mode(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	char *strmode = NULL;

	if (lp->mode == PWM_BASED)
		strmode = "pwm based";
	else if (lp->mode == REGISTER_BASED)
		strmode = "register based";

	return scnprintf(buf, PAGE_SIZE, "%s\n", strmode);
}

static DEVICE_ATTR(chip_id, S_IRUGO, lp855x_get_chip_id, NULL);
static DEVICE_ATTR(bl_ctl_mode, S_IRUGO, lp855x_get_bl_ctl_mode, NULL);

static struct attribute *lp855x_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_bl_ctl_mode.attr,
	NULL,
};

static const struct attribute_group lp855x_attr_group = {
	.attrs = lp855x_attributes,
};

#ifdef CONFIG_OF
static int lp855x_parse_dt(struct lp855x *lp)
{
	struct device *dev = lp->dev;
	struct device_node *node = dev->of_node;
	struct lp855x_platform_data *pdata;
	int rom_length;

	if (!node) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_string(node, "bl-name", &pdata->name);
	of_property_read_u8(node, "dev-ctrl", &pdata->device_control);
	of_property_read_u8(node, "init-brt", &pdata->initial_brightness);
	of_property_read_u32(node, "pwm-period", &pdata->period_ns);
	/*HTC ADD*/
	of_property_read_u32(node, "torch-brt", &lp->torch_brightness);
	/* parse enable gpio */
	lp->gpio = of_get_named_gpio(node, "enable-gpios", 0);
	if(gpio_is_valid(lp->gpio)){
		if (!gpio_get_value(lp->gpio)){
			gpio_set_value(lp->gpio, 1);
		}
		lp->gpio_enabled = 1;
	}

	/* Fill ROM platform data if defined */
	rom_length = of_get_child_count(node);
	if (rom_length > 0) {
		struct lp855x_rom_data *rom;
		struct device_node *child;
		int i = 0;

		rom = devm_kzalloc(dev, sizeof(*rom) * rom_length, GFP_KERNEL);
		if (!rom)
			return -ENOMEM;

		for_each_child_of_node(node, child) {
			of_property_read_u8(child, "rom-addr", &rom[i].addr);
			of_property_read_u8(child, "rom-val", &rom[i].val);
			i++;
		}

		pdata->size_program = rom_length;
		pdata->rom_data = &rom[0];
	}

	lp->pdata = pdata;

	return 0;
}
#else
static int lp855x_parse_dt(struct lp855x *lp)
{
	return -EINVAL;
}
#endif

static int lp855x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lp855x *lp;
	struct regmap_config regmap_cfg;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	lp = devm_kzalloc(&cl->dev, sizeof(struct lp855x), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	lp->client = cl;
	lp->dev = &cl->dev;
	lp->chipname = id->name;
	lp->chip_id = id->driver_data;
	lp->pdata = dev_get_platdata(&cl->dev);

	if (!lp->pdata) {
		ret = lp855x_parse_dt(lp);
		if (ret < 0)
			return ret;
	}

	if (lp->pdata->period_ns > 0)
		lp->mode = PWM_BASED;
	else
		lp->mode = REGISTER_BASED;

	lp->supply = devm_regulator_get(lp->dev, "power");
	if (IS_ERR(lp->supply)) {
		if (PTR_ERR(lp->supply) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		lp->supply = NULL;
	}

	if (lp->supply) {
		ret = regulator_enable(lp->supply);
		if (ret < 0) {
			dev_err(&cl->dev, "failed to enable supply: %d\n", ret);
			return ret;
		}
	}

	/* Setup regmap */
	memset(&regmap_cfg, 0, sizeof(struct regmap_config));
	regmap_cfg.reg_bits = 8;
	regmap_cfg.val_bits = 8;
	regmap_cfg.name = id->name;
	regmap_cfg.max_register = LP8556_MAX_REGISTER;

	lp->regmap = devm_regmap_init_i2c(cl, &regmap_cfg);

	if (IS_ERR(lp->regmap))
	{
		return PTR_ERR(lp->regmap);
	}

	lp855x_parse_dt_reg_cmd_sets(lp->dev, &lp->power_on_cmd_sets, lp->regmap, "normal-power-on-cmds");
	lp855x_parse_dt_reg_cmd_sets(lp->dev, &lp->aod_power_on_cmd_sets, lp->regmap, "aod-power-on-cmds");
	lp855x_parse_dt_reg_cmd_sets(lp->dev, &lp->aod_enable_cmd_sets, lp->regmap, "aod-enable-cmds");
	lp855x_parse_dt_reg_cmd_sets(lp->dev, &lp->aod_disable_cmd_sets, lp->regmap, "aod-disable-cmds");

	i2c_set_clientdata(cl, lp);

	ret = lp855x_configure(lp);
	if (ret) {
		dev_err(lp->dev, "device config err: %d", ret);
		return ret;
	}

	ret = lp855x_backlight_register(lp);
	if (ret) {
		dev_err(lp->dev,
			"failed to register backlight. err: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&lp->dev->kobj, &lp855x_attr_group);
	if (ret) {
		dev_err(lp->dev, "failed to register sysfs. err: %d\n", ret);
		return ret;
	}


	backlight_update_status(lp->bl);

	lp->lp855x_fb_notif.notifier_call = fb_notifier_callback;

	fb_register_client(&lp->lp855x_fb_notif);

	if(lp->torch_brightness != 0)
	{
		INIT_DELAYED_WORK(&lp->flash_work, lp855x_flash_off_work);
		lp->flash_dev.id = 1;
		lp->flash_dev.flash_func = lp855x_flash_mode;
		lp->flash_dev.torch_func = lp855x_torch_mode;

		if (register_htc_flashlight(&lp->flash_dev))
		{
			pr_err("%s: register htc_flashlight failed!\n", __func__);
			lp->flash_dev.id = -1;
		}
		else
		{
			ret = sysfs_create_group(&lp->bl->dev.kobj, &lp855x_flash_attr_group);
			dev_info(lp->dev, "create flash attrs, ret=%d\n", ret);
		}
	}


	return 0;
}

static int lp855x_remove(struct i2c_client *cl)
{
	struct lp855x *lp = i2c_get_clientdata(cl);

	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);
	if (lp->supply)
		regulator_disable(lp->supply);
	sysfs_remove_group(&lp->dev->kobj, &lp855x_attr_group);

	fb_unregister_client(&lp->lp855x_fb_notif);

	if(lp->torch_brightness != 0)
	{
		sysfs_remove_group(&lp->bl->dev.kobj, &lp855x_flash_attr_group);
		unregister_htc_flashlight(&lp->flash_dev);
		lp->torch_brightness = 0;
		lp->flash_dev.id = -1;
	}
	return 0;
}

static const struct of_device_id lp855x_dt_ids[] = {
	{ .compatible = "ti,lp8550", },
	{ .compatible = "ti,lp8551", },
	{ .compatible = "ti,lp8552", },
	{ .compatible = "ti,lp8553", },
	{ .compatible = "ti,lp8555", },
	{ .compatible = "ti,lp8556", },
	{ .compatible = "ti,lp8557", },
	{ }
};
MODULE_DEVICE_TABLE(of, lp855x_dt_ids);

static const struct i2c_device_id lp855x_ids[] = {
	{"lp8550", LP8550},
	{"lp8551", LP8551},
	{"lp8552", LP8552},
	{"lp8553", LP8553},
	{"lp8555", LP8555},
	{"lp8556", LP8556},
	{"lp8557", LP8557},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp855x_ids);

static struct i2c_driver lp855x_driver = {
	.driver = {
		   .name = "lp855x",
		   .of_match_table = of_match_ptr(lp855x_dt_ids),
		   },
	.probe = lp855x_probe,
	.remove = lp855x_remove,
	.id_table = lp855x_ids,
};

module_i2c_driver(lp855x_driver);

MODULE_DESCRIPTION("Texas Instruments LP855x Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_LICENSE("GPL");
