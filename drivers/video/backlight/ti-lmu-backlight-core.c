/*
 * TI LMU (Lighting Management Unit) Backlight Driver
 *
 * Copyright 2016 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/backlight.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/ti-lmu.h>
#include <linux/mfd/ti-lmu-backlight.h>
#include <linux/mfd/ti-lmu-register.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define NUM_DUAL_CHANNEL			2
#define LMU_BACKLIGHT_DUAL_CHANNEL_USED		(BIT(0) | BIT(1))
#define LMU_BACKLIGHT_FULL_CHANNEL_USED		(BIT(0) | BIT(1) | BIT(2) | BIT(3))
#define LMU_BACKLIGHT_11BIT_LSB_MASK		(BIT(0) | BIT(1) | BIT(2))
#define LMU_BACKLIGHT_11BIT_MSB_SHIFT		3
#define DEFAULT_PWM_NAME			"lmu-backlight"

#define FLASH_MODE_BRIGHTNESS			2048
#define FLASH_MODE_MAX_DURATION_MS		500
#define TORCH_MODE_MAX_DURATION_MS		10000

static int ti_lmu_backlight_configure(struct ti_lmu_bl *lmu_bl);

static int ti_lmu_backlight_update_ctrl_mode(struct ti_lmu_bl *lmu_bl, int enable)
{
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	u32 *reg = lmu_bl->chip->cfg->reginfo->mode + lmu_bl->bank_id;
	u8 val;

	if (!reg)
		return 0;

	/*
	 * Update PWM configuration register.
	 * If the mode is register based, then clear the bit.
	 */
	if (enable && (lmu_bl->mode == BL_PWM_BASED || lmu_bl->mode == BL_EXTERNAL_PWM_BASED))
		val = LMU_BL_GET_VAL(*reg);
	else
		val = 0;

	return regmap_update_bits(regmap, LMU_BL_GET_ADDR(*reg),
				  LMU_BL_GET_MASK(*reg), val);
}


static int ti_lmu_backlight_enable(struct ti_lmu_bl *lmu_bl, int enable)
{
	struct ti_lmu_bl_chip *chip = lmu_bl->chip;
	struct regmap *regmap = chip->lmu->regmap;
	unsigned long enable_time = chip->cfg->reginfo->enable_usec;
	u8 *reg = chip->cfg->reginfo->enable;
	u8 offset = chip->cfg->reginfo->enable_offset;
	u8 mask = BIT(lmu_bl->bank_id) << offset;
	int ret = 0;

	if (!reg)
		return -EINVAL;

	if (enable) {
		ti_lmu_backlight_update_ctrl_mode(lmu_bl, enable);
		ret = regmap_update_bits(regmap, *reg, mask, mask);
	} else {
		ret = regmap_update_bits(regmap, *reg, mask, 0);
		ti_lmu_backlight_update_ctrl_mode(lmu_bl, enable);
	}

	/* FIXME: maybe extra delay between brightness update */
	if (enable_time > 0)
		usleep_range(enable_time, enable_time + 100);

	return ret;
}

static void ti_lmu_backlight_pwm_ctrl(struct ti_lmu_bl *lmu_bl, int brightness,
				      int max_brightness)
{
	struct pwm_device *pwm;
	unsigned int duty, period;

	if (!lmu_bl->pwm) {
		pwm = devm_pwm_get(lmu_bl->chip->dev, DEFAULT_PWM_NAME);
		if (IS_ERR(pwm)) {
			dev_err(lmu_bl->chip->dev,
				"Can not get PWM device, err: %ld\n",
				PTR_ERR(pwm));
			return;
		}

		lmu_bl->pwm = pwm;
	}

	period = lmu_bl->pwm_period;
	duty = brightness * period / max_brightness;

	pwm_config(lmu_bl->pwm, duty, period);
	if (duty)
		pwm_enable(lmu_bl->pwm);
	else
		pwm_disable(lmu_bl->pwm);
}

static int ti_lmu_backlight_update_brightness_register(struct ti_lmu_bl *lmu_bl,
						       int brightness)
{
	const struct ti_lmu_bl_cfg *cfg = lmu_bl->chip->cfg;
	const struct ti_lmu_bl_reg *reginfo = cfg->reginfo;
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	u8 reg, val;
	int ret;

	if (lmu_bl->mode == BL_PWM_BASED) {
		switch (cfg->pwm_action) {
		case UPDATE_PWM_ONLY:
			/* No register update is required */
			return 0;
		case UPDATE_MAX_BRT:
			/*
			 * PWM can start from any non-zero code and dim down
			 * to zero. So, brightness register should be updated
			 * even in PWM mode.
			 */
			if (brightness > 0)
				brightness = MAX_BRIGHTNESS_11BIT;
			else
				brightness = 0;
			break;
		default:
			break;
		}
	}

	/*
	 * Brightness register update
	 *
	 * 11 bit dimming: update LSB bits and write MSB byte.
	 *		   MSB brightness should be shifted.
	 *  8 bit dimming: write MSB byte.
	 */

	if (!reginfo->brightness_msb)
		return -EINVAL;

	if (cfg->max_brightness == MAX_BRIGHTNESS_11BIT) {
		if (!reginfo->brightness_lsb)
			return -EINVAL;

		reg = reginfo->brightness_lsb[lmu_bl->bank_id];
		ret = regmap_update_bits(regmap, reg,
					 LMU_BACKLIGHT_11BIT_LSB_MASK,
					 brightness);
		if (ret)
			return ret;

		val = (brightness >> LMU_BACKLIGHT_11BIT_MSB_SHIFT) & 0xFF;
	} else {
		val = brightness & 0xFF;
	}

	reg = reginfo->brightness_msb[lmu_bl->bank_id];
	return regmap_write(regmap, reg, val);
}

static int ti_lmu_backlight_update_status(struct backlight_device *bl_dev)
{
	struct ti_lmu_bl *lmu_bl = bl_get_data(bl_dev);
	int brightness = bl_dev->props.brightness;
	int ret;

	if(lmu_bl->flash_enabled)
		return -EBUSY;

	if (bl_dev->props.state & BL_CORE_SUSPENDED) {
		pr_info("%s: allow_always_on = %d\n", __func__, lmu_bl->allow_always_on);
		if (lmu_bl->allow_always_on)
			return 0;
		brightness = 0;
	}

	if (brightness > 0)
		ret = ti_lmu_backlight_enable(lmu_bl, 1);
	else
		ret = ti_lmu_backlight_enable(lmu_bl, 0);

	if (ret)
		return ret;

	if (lmu_bl->mode == BL_PWM_BASED)
		ti_lmu_backlight_pwm_ctrl(lmu_bl, brightness,
					  bl_dev->props.max_brightness);

	return ti_lmu_backlight_update_brightness_register(lmu_bl, brightness);
}

static const struct backlight_ops lmu_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ti_lmu_backlight_update_status,
};

static int ti_lmu_backlight_of_get_ctrl_bank(struct device_node *np,
					     struct ti_lmu_bl *lmu_bl)
{
	const char *name;
	u32 *sources;
	int num_channels = lmu_bl->chip->cfg->num_channels;
	int ret, num_sources;

	sources = devm_kzalloc(lmu_bl->chip->dev, num_channels, GFP_KERNEL);
	if (!sources)
		return -ENOMEM;

	if (!of_property_read_string(np, "label", &name))
		lmu_bl->name = name;
	else
		lmu_bl->name = np->name;

	ret = of_property_count_u32_elems(np, "led-sources");
	if (ret < 0 || ret > num_channels)
		return -EINVAL;

	num_sources = ret;
	ret = of_property_read_u32_array(np, "led-sources", sources,
					 num_sources);
	if (ret)
		return ret;

	lmu_bl->led_sources = 0;
	while (num_sources--)
		set_bit(sources[num_sources], &lmu_bl->led_sources);

	return 0;
}

static void ti_lmu_backlight_of_get_light_properties(struct device_node *np,
						     struct ti_lmu_bl *lmu_bl)
{
	of_property_read_u32(np, "default-brightness-level",
			     &lmu_bl->default_brightness);

	of_property_read_u32(np, "ramp-up-msec",  &lmu_bl->ramp_up_msec);
	of_property_read_u32(np, "ramp-down-msec", &lmu_bl->ramp_down_msec);
	/*HTC ADD*/
	of_property_read_u32(np, "torch-brt", &lmu_bl->torch_brightness);
}

static void ti_lmu_backlight_of_get_brightness_mode(struct device_node *np,
						    struct ti_lmu_bl *lmu_bl)
{
	bool external_pwm = false;
	of_property_read_u32(np, "pwm-period", &lmu_bl->pwm_period);
	external_pwm = of_property_read_bool(np, "use-external-pwm");

	if (lmu_bl->pwm_period > 0)
		lmu_bl->mode = BL_PWM_BASED;
	else if (external_pwm)
		lmu_bl->mode = BL_EXTERNAL_PWM_BASED;
	else
		lmu_bl->mode = BL_REGISTER_BASED;

}
/* HTC Implmenetation */
static int ti_lmu_bl_flash_en_locked(struct ti_lmu_bl *lmu_bl, int en, int duration, int level)
{
	int rc = 0;

	if (en) {
		if (lmu_bl->flash_enabled) {
			pr_info("%s: already enabled\n", __func__);
			rc = -EBUSY;
			goto exit;
		}

		if (level > lmu_bl->chip->cfg->max_brightness){
			lmu_bl->led_sources_backup = lmu_bl->led_sources;
			lmu_bl->led_sources = LMU_BACKLIGHT_FULL_CHANNEL_USED;
		}
		else {
			if(lmu_bl->led_sources_backup != 0){
				lmu_bl->led_sources = lmu_bl->led_sources_backup;
				lmu_bl->led_sources_backup = 0;
			}
		}

		if (level <= 0 || level > lmu_bl->chip->cfg->max_brightness)
			level = lmu_bl->chip->cfg->max_brightness;

		if (duration < 10)
			duration = 10;
		schedule_delayed_work(&lmu_bl->flash_work, msecs_to_jiffies(duration));
	} else {
		if(lmu_bl->led_sources_backup != 0){
			lmu_bl->led_sources = lmu_bl->led_sources_backup;
			lmu_bl->led_sources_backup = 0;
		}
		level = lmu_bl->bl_dev->props.brightness;
	}
	pr_info("%s: (%d, %d) [level=%d] led_sources= %lx\n", __func__, en, duration, level,lmu_bl->led_sources);

	ti_lmu_backlight_update_brightness_register(lmu_bl, level);
	rc = ti_lmu_backlight_configure(lmu_bl);
	if (rc){
		pr_err("%s: ti lmu backligth configure error. rc = %d\n", __func__,rc);
		goto exit;
	}

	lmu_bl->flash_enabled = en;
exit:
	pr_info("%s: (%d, %d) done, rc=%d\n", __func__, en, duration, rc);

	return rc;
}

static int ti_lmu_bl_flash_en(struct ti_lmu_bl *lmu_bl, int en, int duration, int level)
{
	int rc = 0;

	if (!en) {
		cancel_delayed_work_sync(&lmu_bl->flash_work);
	}

	mutex_lock(&lmu_bl->bl_dev->update_lock);
	rc = ti_lmu_bl_flash_en_locked(lmu_bl, en, duration, level);
	mutex_unlock(&lmu_bl->bl_dev->update_lock);

	return rc;
}

static void ti_lmu_bl_flash_off_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti_lmu_bl *lmu_bl = container_of(dwork, struct ti_lmu_bl, flash_work);

	mutex_lock(&lmu_bl->bl_dev->update_lock);
	ti_lmu_bl_flash_en_locked(lmu_bl, 0, 0, 0);
	mutex_unlock(&lmu_bl->bl_dev->update_lock);
}
static int ti_lmu_bl_flash_mode(struct htc_flashlight_dev *fl_dev, int mode1, int mode2)
{
	struct ti_lmu_bl *lmu_bl = container_of(fl_dev, struct ti_lmu_bl, flash_dev);

	return ti_lmu_bl_flash_en(lmu_bl, mode1, FLASH_MODE_MAX_DURATION_MS, FLASH_MODE_BRIGHTNESS);
}

static int ti_lmu_bl_torch_mode(struct htc_flashlight_dev *fl_dev, int mode1, int mode2)
{
	struct ti_lmu_bl *lmu_bl = container_of(fl_dev, struct ti_lmu_bl, flash_dev);

	return ti_lmu_bl_flash_en(lmu_bl, mode1, TORCH_MODE_MAX_DURATION_MS, lmu_bl->torch_brightness);
}
static ssize_t ti_lmu_bl_get_flash_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ti_lmu_bl *lmu_bl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lmu_bl->flash_enabled);
}

static ssize_t ti_lmu_bl_set_flash_en(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ti_lmu_bl *lmu_bl = dev_get_drvdata(dev);
	int data = 0, level = 0, rc = 0;

	if (sscanf(buf, "%d %d", &data, &level) < 1)
		return -EINVAL;

	rc = ti_lmu_bl_flash_en(lmu_bl, !!data, data, level);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(flash_en, (S_IRUGO | S_IWUSR | S_IWGRP),
	ti_lmu_bl_get_flash_en, ti_lmu_bl_set_flash_en);
static struct attribute *ti_lmu_bl_flash_attributes[] = {
	&dev_attr_flash_en.attr,
	NULL,
};

static const struct attribute_group ti_lmu_bl_flash_attr_group = {
	.attrs = ti_lmu_bl_flash_attributes,
};
/* HTC Implmenetation End */

static int ti_lmu_backlight_of_create(struct ti_lmu_bl_chip *chip,
				      struct device_node *np)
{
	struct device_node *child;
	struct ti_lmu_bl *lmu_bl, *each;
	int ret, num_backlights;
	int i = 0;

	num_backlights = of_get_child_count(np);
	if (num_backlights == 0) {
		dev_err(chip->dev, "No backlight strings\n");
		return -ENODEV;
	}

	/* One chip can have mulitple backlight strings */
	lmu_bl = devm_kzalloc(chip->dev, sizeof(*lmu_bl) * num_backlights,
			      GFP_KERNEL);
	if (!lmu_bl)
		return -ENOMEM;

	/* Child is mapped to LMU backlight control bank */
	for_each_child_of_node(np, child) {
		each = lmu_bl + i;
		each->bank_id = i;
		each->chip = chip;

		ret = ti_lmu_backlight_of_get_ctrl_bank(child, each);
		if (ret) {
			of_node_put(np);
			return ret;
		}

		ti_lmu_backlight_of_get_light_properties(child, each);
		ti_lmu_backlight_of_get_brightness_mode(child, each);

		i++;
	}

	chip->lmu_bl = lmu_bl;
	chip->num_backlights = num_backlights;

	return 0;
}

static int ti_lmu_backlight_create_channel(struct ti_lmu_bl *lmu_bl)
{
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	u32 *reg = lmu_bl->chip->cfg->reginfo->channel;
	int num_channels = lmu_bl->chip->cfg->num_channels;
	int i, ret;
	u8 val;

	/*
	 * How to create backlight output channels:
	 *   Check 'led_sources' bit and update registers.
	 *
	 *   1) Dual channel configuration
	 *     The 1st register data is used for single channel.
	 *     The 2nd register data is used for dual channel.
	 *
	 *   2) Multiple channel configuration
	 *     Each register data is mapped to bank ID.
	 *     Bit shift operation is defined in channel registers.
	 *
	 * Channel register data consists of address, mask, value.
	 * Driver can get each data by using LMU_BL_GET_ADDR(),
	 * LMU_BL_GET_MASK(), LMU_BL_GET_VAL().
	 */

	if (num_channels == NUM_DUAL_CHANNEL) {
		if (lmu_bl->led_sources == LMU_BACKLIGHT_DUAL_CHANNEL_USED)
			++reg;

		return regmap_update_bits(regmap, LMU_BL_GET_ADDR(*reg),
					  LMU_BL_GET_MASK(*reg),
					  LMU_BL_GET_VAL(*reg));
	}

	for (i = 0; i < num_channels; i++) {
		if (!reg)
			break;

		/*
		 * The result of LMU_BL_GET_VAL()
		 *
		 * Register set value. One bank controls multiple channels.
		 * LM36274 data should be configured with 'single_bank_used'.
		 * Otherwise, the result is shift bit.
		 * The bank_id should be shifted for the channel configuration.
		 */

		if (test_bit(i, &lmu_bl->led_sources)) {
			if (lmu_bl->chip->cfg->single_bank_used)
				val = LMU_BL_GET_VAL(*reg);
			else
				val = lmu_bl->bank_id << LMU_BL_GET_VAL(*reg);

			ret = regmap_update_bits(regmap, LMU_BL_GET_ADDR(*reg),
						 LMU_BL_GET_MASK(*reg), val);
			if (ret)
				return ret;
		}
		else {
			ret = regmap_update_bits(regmap, LMU_BL_GET_ADDR(*reg),
						 LMU_BL_GET_MASK(*reg), 0);
			if (ret)
				return ret;
		}

		reg++;
	}

	return 0;
}

static int ti_lmu_backlight_convert_ramp_to_index(struct ti_lmu_bl *lmu_bl,
						  enum ti_lmu_bl_ramp_mode mode)
{
	const int *ramp_table = lmu_bl->chip->cfg->ramp_table;
	const int size = lmu_bl->chip->cfg->size_ramp;
	unsigned int msec;
	int i;

	if (!ramp_table)
		return -EINVAL;

	switch (mode) {
	case BL_RAMP_UP:
		msec = lmu_bl->ramp_up_msec;
		break;
	case BL_RAMP_DOWN:
		msec = lmu_bl->ramp_down_msec;
		break;
	default:
		return -EINVAL;
	}

	if (msec <= ramp_table[0])
		return 0;

	if (msec > ramp_table[size - 1])
		return size - 1;

	for (i = 1; i < size; i++) {
		if (msec == ramp_table[i])
			return i;

		/* Find an approximate index by looking up the table */
		if (msec > ramp_table[i - 1] && msec < ramp_table[i]) {
			if (msec - ramp_table[i - 1] < ramp_table[i] - msec)
				return i - 1;
			else
				return i;
		}
	}

	return -EINVAL;
}

static int ti_lmu_backlight_set_ramp(struct ti_lmu_bl *lmu_bl)
{
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	const struct ti_lmu_bl_reg *reginfo = lmu_bl->chip->cfg->reginfo;
	int offset = reginfo->ramp_reg_offset;
	int i, ret, index;
	u32 reg;

	for (i = BL_RAMP_UP; i <= BL_RAMP_DOWN; i++) {
		index = ti_lmu_backlight_convert_ramp_to_index(lmu_bl, i);
		if (index > 0) {
			if (!reginfo->ramp)
				break;

			if (lmu_bl->bank_id == 0)
				reg = reginfo->ramp[i];
			else
				reg = reginfo->ramp[i] + offset;

			/*
			 * Note that the result of LMU_BL_GET_VAL() is
			 * shift bit. So updated bit is shifted index value.
			 */
			ret = regmap_update_bits(regmap, LMU_BL_GET_ADDR(reg),
						 LMU_BL_GET_MASK(reg),
						 index << LMU_BL_GET_VAL(reg));
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int ti_lmu_backlight_configure(struct ti_lmu_bl *lmu_bl)
{
	int ret;

	ret = ti_lmu_backlight_create_channel(lmu_bl);
	if (ret)
		return ret;

	return ti_lmu_backlight_set_ramp(lmu_bl);
}

static int ti_lmu_backlight_init(struct ti_lmu_bl_chip *chip)
{
	struct regmap *regmap = chip->lmu->regmap;
	u32 *reg = chip->cfg->reginfo->init;
	int num_init = chip->cfg->reginfo->num_init;
	int i, ret;

	/*
	 * 'init' register data consists of address, mask, value.
	 * Driver can get each data by using LMU_BL_GET_ADDR(),
	 * LMU_BL_GET_MASK(), LMU_BL_GET_VAL().
	 */

	for (i = 0; i < num_init; i++) {
		if (!reg)
			break;

		ret = regmap_update_bits(regmap, LMU_BL_GET_ADDR(*reg),
					 LMU_BL_GET_MASK(*reg),
					 LMU_BL_GET_VAL(*reg));
		if (ret)
			return ret;

		reg++;
	}

	return 0;
}

static int ti_lmu_backlight_reload(struct ti_lmu_bl_chip *chip)
{
	struct ti_lmu_bl *each;
	int i, ret;

	ret = ti_lmu_backlight_init(chip);
	if (ret)
		return ret;

	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;
		ret = ti_lmu_backlight_configure(each);
		if (ret)
			return ret;

		backlight_update_status(each->bl_dev);
	}

	return 0;
}

static int ti_lmu_backlight_add_device(struct device *dev,
				       struct ti_lmu_bl *lmu_bl)
{
	struct backlight_device *bl_dev;
	struct backlight_properties props;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = lmu_bl->default_brightness;
	props.max_brightness = lmu_bl->chip->cfg->max_brightness;

	bl_dev = backlight_device_register(lmu_bl->name, dev, lmu_bl,
					   &lmu_backlight_ops, &props);
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	lmu_bl->bl_dev = bl_dev;

	return 0;
}

static struct ti_lmu_bl_chip *
ti_lmu_backlight_register(struct device *dev, struct ti_lmu *lmu,
			  const struct ti_lmu_bl_cfg *cfg)
{
	struct ti_lmu_bl_chip *chip;
	struct ti_lmu_bl *each;
	int i, ret;

	if (!cfg) {
		dev_err(dev, "Operation is not configured\n");
		return ERR_PTR(-EINVAL);
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->dev = dev;
	chip->lmu = lmu;
	chip->cfg = cfg;

	ret = ti_lmu_backlight_of_create(chip, dev->of_node);
	if (ret)
		return ERR_PTR(ret);

	ret = ti_lmu_backlight_init(chip);
	if (ret) {
		dev_err(dev, "Backlight init err: %d\n", ret);
		return ERR_PTR(ret);
	}

	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;

		ret = ti_lmu_backlight_configure(each);
		if (ret) {
			dev_err(dev, "Backlight config err: %d\n", ret);
			return ERR_PTR(ret);
		}

		ret = ti_lmu_backlight_add_device(dev, each);
		if (ret) {
			dev_err(dev, "Backlight device err: %d\n", ret);
			return ERR_PTR(ret);
		}

		backlight_update_status(each->bl_dev);

		if(each->torch_brightness != 0){
			INIT_DELAYED_WORK(&each->flash_work, ti_lmu_bl_flash_off_work);
			each->flash_dev.id = 1;
			each->flash_dev.flash_func = ti_lmu_bl_flash_mode;
			each->flash_dev.torch_func = ti_lmu_bl_torch_mode;

			if (register_htc_flashlight(&each->flash_dev)) {
				pr_err("%s: register htc_flashlight failed!\n", __func__);
				each->flash_dev.id = -1;
			} else {
				ret = sysfs_create_group(&each->bl_dev->dev.kobj, &ti_lmu_bl_flash_attr_group);
				dev_info(each->chip->dev, "create flash attrs, ret=%d\n", ret);
			}
                }

	}

	return chip;
}

static void ti_lmu_backlight_unregister(struct ti_lmu_bl_chip *chip)
{
	struct ti_lmu_bl *each;
	int i;

	/* Turn off the brightness */
	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;
		each->bl_dev->props.brightness = 0;
		backlight_update_status(each->bl_dev);
		backlight_device_unregister(each->bl_dev);
		if(each->torch_brightness != 0){
			sysfs_remove_group(&each->bl_dev->dev.kobj, &ti_lmu_bl_flash_attr_group);
			unregister_htc_flashlight(&each->flash_dev);
			each->torch_brightness = 0;
			each->flash_dev.id = -1;
		}
	}
}

static int ti_lmu_backlight_monitor_notifier(struct notifier_block *nb,
					     unsigned long action, void *unused)
{
	struct ti_lmu_bl_chip *chip = container_of(nb, struct ti_lmu_bl_chip,
						   nb);

	if (action == LMU_EVENT_MONITOR_DONE) {
		if (ti_lmu_backlight_reload(chip))
			return NOTIFY_STOP;
	}

	return NOTIFY_OK;
}

static int ti_lmu_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_lmu *lmu = dev_get_drvdata(dev->parent);
	struct ti_lmu_bl_chip *chip;
	int ret;

	chip = ti_lmu_backlight_register(dev, lmu, &lmu_bl_cfg[pdev->id]);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	/*
	 * Notifier callback is required because backlight device needs
	 * reconfiguration after fault detection procedure is done by
	 * ti-lmu-fault-monitor driver.
	 */
	if (chip->cfg->fault_monitor_used) {
		chip->nb.notifier_call = ti_lmu_backlight_monitor_notifier;
		ret = blocking_notifier_chain_register(&chip->lmu->notifier,
						       &chip->nb);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static int ti_lmu_backlight_remove(struct platform_device *pdev)
{
	struct ti_lmu_bl_chip *chip = platform_get_drvdata(pdev);

	if (chip->cfg->fault_monitor_used)
		blocking_notifier_chain_unregister(&chip->lmu->notifier,
						   &chip->nb);

	ti_lmu_backlight_unregister(chip);

	return 0;
}

static struct platform_driver ti_lmu_backlight_driver = {
	.probe  = ti_lmu_backlight_probe,
	.remove = ti_lmu_backlight_remove,
	.driver = {
		.name = "ti-lmu-backlight",
	},
};

static int __init ti_lmu_bl_init(void)
{
	pr_info("ti_lmu_bl_init\n");
	return platform_driver_register(&ti_lmu_backlight_driver);
}
subsys_initcall(ti_lmu_bl_init);

static void __exit ti_lmu_bl_exit(void)
{
	platform_driver_unregister(&ti_lmu_backlight_driver);
}
module_exit(ti_lmu_bl_exit);


MODULE_DESCRIPTION("TI LMU Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ti-lmu-backlight");
