/*
 * Driver for the TUSB1044 USB3.0 re-drive
 *
 * Copyright (C) 2016 HTC Corporation.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/workqueue.h>

#include "tusb1044.h"

extern bool get_speed_lock(void);
static const char * const cc_state_name[] = {
	[CC_STATE_USB3]		= "USB3",
	[CC_STATE_DP]		= "Display Port",
	[CC_STATE_OPEN]		= "Open",
};

static const char * const cc_orientation_name[] = {
	[CC_ORIENTATION_NONE]	= "None",
	[CC_ORIENTATION_CC1]	= "CC1",
	[CC_ORIENTATION_CC2]	= "CC2",
};

struct tusb1044_chip {
	struct device		*device;
	struct i2c_client	*i2c_client;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_active_state;
	struct pinctrl_state	*pinctrl_sleep_state;
	struct pinctrl_state    *pinctrl_vdd_active_state;
	struct pinctrl_state    *pinctrl_vdd_sleep_state;

	struct delayed_work		update_work;
	struct workqueue_struct	*tusb_wq;
	struct mutex		state_lock;

	bool			usb3_disable;
	bool			probe_done;
	bool			vdd_state;
	enum cc_state		cc_state;
	enum cc_orientation	cc_orientation;
	//atomic_t		pm_suspended;

	u32			layout_type;
	bool			chip_exist;
};

struct tusb1044_chip *tusb_chip = NULL;
static void tusb1044_usb3_ui_update_state(void);

void set_redriver_status(void)
{
	struct tusb1044_chip *chip = tusb_chip;
	bool temp = get_speed_lock();

	if (!chip || chip->usb3_disable == temp)
		return;

	chip->usb3_disable = temp;

	if (!chip->layout_type || chip->chip_exist)
		tusb1044_usb3_ui_update_state();
	return;
}

static int tusb1044_1v8_switch(struct tusb1044_chip *chip, bool enabled)
{
	int ret = 0;
	struct pinctrl_state *pinctrl_state = enabled ?
					      chip->pinctrl_active_state :
					      chip->pinctrl_sleep_state;

	ret = pinctrl_select_state(chip->pinctrl, pinctrl_state);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB1044 %s - Error: pinctrl enable %s state failed, ret=%d\n",
			__func__, enabled ? "active" : "sleep", ret);
		return ret;
	}

	dev_err(chip->device, "TUSB1044 %s - pinctrl enable %s state done\n",
		 __func__, enabled ? "active" : "sleep");
	return ret;
}

static int tusb1044_vdd_switch(struct tusb1044_chip *chip, bool enabled)
{
	int ret = 0;
	struct pinctrl_state *pinctrl_state = enabled ?
					      chip->pinctrl_vdd_active_state :
					      chip->pinctrl_vdd_sleep_state;

	ret = pinctrl_select_state(chip->pinctrl, pinctrl_state);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB1044 %s - Error: pinctrl vdd enable %s state failed, ret=%d\n",
			__func__, enabled ? "active" : "sleep", ret);
		return ret;
	}

	chip->vdd_state = enabled ? true : false;

	dev_err(chip->device, "TUSB1044 %s - pinctrl vdd enable %s state done\n",
		 __func__, enabled ? "active" : "sleep");
	return ret;
}

static int tusb1044_aux_snoop(struct tusb1044_chip *chip, bool enabled)
{
	int ret = 0;
	u16 aux_ctrl_value = DISABLE_AUX_SNOOP;

	if (enabled) {
		aux_ctrl_value = ENABLE_AUX_SNOOP;
	} else {
		if (chip->cc_orientation == CC_ORIENTATION_CC1) {
			aux_ctrl_value = DISABLE_AUX_SNOOP | AUX_SBU_CC1;
		} else if (chip->cc_orientation == CC_ORIENTATION_CC2) {
			aux_ctrl_value = DISABLE_AUX_SNOOP | AUX_SBU_CC2;
		}
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_AUX, aux_ctrl_value);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB1044 %s - cannot %s aux snoop, ret=%d\n",
			__func__, enabled ? "enable" : "disable", ret);
		return ret;
	}

	dev_err(chip->device, "TUSB1044 %s - %s aux snoop, aux_ctrl_value = 0x%x\n",
		 __func__, enabled ? "enable" : "disable", aux_ctrl_value);
	return ret;
}

static void tusb1044_dp_update_state(void)
{
	struct tusb1044_chip *chip = tusb_chip;
	int ret = 0;
	bool vdd_enabled = true;
	bool switch_1v8_enabled = true;
	bool aux_snoop_enabled = false;
	u8 config_ctrl_value = 0;

	if (!chip)
		return;

	if (chip->layout_type && !chip->chip_exist) {
		dev_err(chip->device, "%s - not exist\n", __func__);
		return;
	}

	if (chip->cc_orientation == CC_ORIENTATION_CC1)
		config_ctrl_value = DP_ON_CC1;
	else if (chip->cc_orientation == CC_ORIENTATION_CC2)
		config_ctrl_value = DP_ON_CC2;

	// power on the chip
	if (chip->layout_type && chip->chip_exist) {
		ret = tusb1044_vdd_switch(chip, vdd_enabled);
		if (ret < 0)
			return;
	}

	ret = tusb1044_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return;

	ret = tusb1044_aux_snoop(chip, aux_snoop_enabled);
	if (ret < 0)
		return;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value | EQ_OVERRIDE);
	if (ret < 0) {
		dev_err(chip->device, "TUSB1044 %s - i2c write data failed\n", __func__);
		return;
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX1, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x21) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX2, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x20) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX1, 0x77);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX1(0x11) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX2, 0x77);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX2(0x10) failed\n", __func__);
	}

	dev_err(chip->device,
		 "TUSB1044 %s - state = %s, orientation = %s, switch = %s, aux_snoop = %s, config_ctrl = 0x%x\n",
		 __func__, cc_state_name[chip->cc_state],
		 cc_orientation_name[chip->cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return;
}

static void tusb1044_usb3_ui_update_state(void)
{
	struct tusb1044_chip *chip = tusb_chip;
	int ret = 0;
	bool vdd_enabled;
	bool switch_1v8_enabled;
	bool aux_snoop_enabled = true;
	u8 config_ctrl_value = 0;

	if (!chip)
		return;

	if (chip->layout_type && !chip->chip_exist) {
		dev_err(chip->device, "%s - not exist\n", __func__);
		return;
	}

	switch (chip->cc_state) {
	case CC_STATE_USB3:
		if (!chip->usb3_disable) {
			vdd_enabled = true;
			switch_1v8_enabled = true;
			if (chip->cc_orientation == CC_ORIENTATION_CC1)
				config_ctrl_value = USB3_ON_CC1;
			else if (chip->cc_orientation == CC_ORIENTATION_CC2)
				config_ctrl_value = USB3_ON_CC2;
		} else {
			vdd_enabled = false;
			switch_1v8_enabled = false;
			config_ctrl_value = DISABLE_TX_RX;
		}
		break;
	case CC_STATE_OPEN:
	default:
		vdd_enabled = false;
		switch_1v8_enabled = false;
		config_ctrl_value = DISABLE_TX_RX;
		break;
	}

	// power on/off the chip
	if (chip->layout_type && chip->chip_exist) {
		ret = tusb1044_vdd_switch(chip, vdd_enabled);
		// if power off, return here
		if (!vdd_enabled || ret < 0)
			return;
	}

	ret = tusb1044_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return;

	// delay 400ms to ensure that the power on sequence is finished
	if (chip->layout_type)
		msleep(400);

	ret = tusb1044_aux_snoop(chip, aux_snoop_enabled);
	if (ret < 0)
		return;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value | EQ_OVERRIDE);
	if (ret < 0) {
		dev_err(chip->device, "TUSB1044 %s - i2c write data failed\n", __func__);
		return;
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX1, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x21) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX2, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x20) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX1, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX1(0x11) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX2, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX2(0x10) failed\n", __func__);
	}

	dev_err(chip->device,
		 "TUSB1044 %s - state = %s, orientation = %s, switch = %s, aux_snoop = %s, config_ctrl = 0x%x\n",
		 __func__, cc_state_name[chip->cc_state],
		 cc_orientation_name[chip->cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return;
}

static void tusb1044_usb3_update_state(struct work_struct *w)
{
	struct tusb1044_chip *chip = container_of(w, struct tusb1044_chip, update_work.work);
	int ret = 0;
	bool vdd_enabled;
	bool switch_1v8_enabled;
	bool aux_snoop_enabled = true;
	u8 config_ctrl_value = 0;

	switch (chip->cc_state) {
	case CC_STATE_USB3:
		if (!chip->usb3_disable) {
			vdd_enabled = true;
			switch_1v8_enabled = true;
			if (chip->cc_orientation == CC_ORIENTATION_CC1)
				config_ctrl_value = USB3_ON_CC1;
			else if (chip->cc_orientation == CC_ORIENTATION_CC2)
				config_ctrl_value = USB3_ON_CC2;
		} else {
			vdd_enabled = false;
			switch_1v8_enabled = false;
			config_ctrl_value = DISABLE_TX_RX;
		}
		break;
	case CC_STATE_OPEN:
	default:
		vdd_enabled = false;
		switch_1v8_enabled = false;
		config_ctrl_value = DISABLE_TX_RX;
		break;
	}

	// power on/off the chip
	if (chip->layout_type && chip->chip_exist) {
		ret = tusb1044_vdd_switch(chip, vdd_enabled);
		// if power off, return here
		if (!vdd_enabled || ret < 0)
			return;
	}

	ret = tusb1044_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return;

	// delay 400ms to ensure that the power on sequence is finished
	if (chip->layout_type)
		msleep(400);

	ret = tusb1044_aux_snoop(chip, aux_snoop_enabled);
	if (ret < 0)
		return;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value | EQ_OVERRIDE);
	if (ret < 0) {
		dev_err(chip->device, "TUSB1044 %s - i2c write data failed\n", __func__);
		return;
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX1, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x21) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX2, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x20) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX1, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX1(0x11) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX2, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX2(0x10) failed\n", __func__);
	}

	dev_err(chip->device,
		 "TUSB1044 %s - state = %s, orientation = %s, switch = %s, aux_snoop = %s, config_ctrl = 0x%x\n",
		 __func__, cc_state_name[chip->cc_state],
		 cc_orientation_name[chip->cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return;
}

void tusb1044_update_state(enum cc_state cc_state, enum cc_orientation cc_orientation)
{
	struct tusb1044_chip *chip = tusb_chip;

	if (!chip)
		return;

	if (chip->layout_type && !chip->chip_exist) {
		dev_err(chip->device, "%s - not exist\n", __func__);
		return;
	}

	mutex_lock(&chip->state_lock);

	chip->cc_state = cc_state;
	chip->cc_orientation = cc_orientation;

	if (!chip->probe_done) {
		queue_delayed_work(chip->tusb_wq, &chip->update_work, 0);
	} else {
		switch (cc_state) {
		case CC_STATE_OPEN:
		case CC_STATE_USB3:
			tusb1044_usb3_ui_update_state();
			break;
		case CC_STATE_DP:
			tusb1044_dp_update_state();
			break;
		case CC_STATE_RARA:
			if (chip->layout_type)
				tusb1044_vdd_switch(chip, true);
			break;
		default:
			break;
		}
	}

	mutex_unlock(&chip->state_lock);
	return;
}

/*
 * Report whether tusb544 exists or not.
 * Return value:
 *     1: exist
 *     0: not exist
 *     -EPROBE_DEFER: probe function has not finished.
 */
int tusb1044_exist(void)
{
	struct tusb1044_chip *chip = tusb_chip;

	if (!chip)
		return -ENODEV;

	if (!chip->probe_done)
		return -EPROBE_DEFER;

	return chip->chip_exist;
}
EXPORT_SYMBOL(tusb1044_exist);

static int tusb1044_probe_device(struct tusb1044_chip *chip)
{
	int i, ret;
	u8 id[8];

	for (i = 0; i < 4; i++) {
		ret = i2c_smbus_read_byte_data(chip->i2c_client, REG_DEVICE_ID_BASE + i);
		if (i == 0 && ret < 0)
			return ret;
		id[i] = ret;
		dev_err(chip->device, "id[%d] = 0x%02X\n", i, id[i]);
	}

	return 0;
}

static int tusb1044_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int ret = 0;
	u32 layout_type = 0;
	struct tusb1044_chip *chip;
	struct device_node *node;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_sleep_state;
	struct pinctrl_state *pinctrl_active_state;
	struct pinctrl_state *pinctrl_vdd_sleep_state;
	struct pinctrl_state *pinctrl_vdd_active_state;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
			"TUSB1044 %s - i2c_check_functionality error\n",
			__func__);
		return -EIO;
	}

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: pinctrl not ready\n",
			__func__);
		return PTR_ERR(pinctrl);
	}

	pinctrl_active_state = pinctrl_lookup_state(pinctrl, "pin_active");
	if (IS_ERR(pinctrl_active_state)) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: no pin_active pinctrl state, ret=%ld\n",
			__func__, PTR_ERR(pinctrl_active_state));
		return PTR_ERR(pinctrl_active_state);
	}

	pinctrl_sleep_state = pinctrl_lookup_state(pinctrl, "pin_sleep");
	if (IS_ERR(pinctrl_sleep_state)) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: no pin_sleep pinctrl state, ret=%ld\n",
			__func__, PTR_ERR(pinctrl_sleep_state));
		return PTR_ERR(pinctrl_sleep_state);
	}

	pinctrl_vdd_active_state = pinctrl_lookup_state(pinctrl, "vdd_pin_active");
	if (IS_ERR(pinctrl_vdd_active_state)) {
		dev_err(&client->dev,
				"TUSB1044 %s - Error: no pin_active pinctrl state, ret=%ld\n",
				__func__, PTR_ERR(pinctrl_active_state));
		return PTR_ERR(pinctrl_vdd_active_state);
	}

	pinctrl_vdd_sleep_state = pinctrl_lookup_state(pinctrl, "vdd_pin_sleep");
	if (IS_ERR(pinctrl_vdd_sleep_state)) {
		dev_err(&client->dev,
				"TUSB1044 %s - Error: no pin_sleep pinctrl state, ret=%ld\n",
				__func__, PTR_ERR(pinctrl_sleep_state));
		return PTR_ERR(pinctrl_vdd_sleep_state);
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->probe_done = false;
	chip->device = &client->dev;
	chip->i2c_client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->state_lock);
	chip->pinctrl = pinctrl;
	chip->pinctrl_active_state = pinctrl_active_state;
	chip->pinctrl_sleep_state = pinctrl_sleep_state;
	chip->pinctrl_vdd_active_state = pinctrl_vdd_active_state;
	chip->pinctrl_vdd_sleep_state = pinctrl_vdd_sleep_state;
	INIT_DELAYED_WORK(&chip->update_work, tusb1044_usb3_update_state);
	chip->tusb_wq = alloc_ordered_workqueue("tusb_wq", 0);
	chip->usb3_disable = get_speed_lock();
	chip->chip_exist = true;

	tusb_chip = chip;

	if (client->dev.of_node) {
		node = client->dev.of_node;
		of_property_read_u32(node, "htc,layout-type", &layout_type);
		dev_err(chip->device, "%s - type %d\n", __func__, layout_type);
	}
	chip->layout_type = layout_type;

//	if (!chip->usb3_disable) {
	ret = tusb1044_vdd_switch(chip, true);
	if (ret < 0) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: TUSB1044 can't be powered:%d\n", __func__, ret);
		return ret;
	}

	if (layout_type == 1) {
		ret = tusb1044_probe_device(chip);
		if (ret < 0) {
			// retry here ?
			chip->chip_exist = false;
			ret = 0;
		}
	}

	tusb1044_update_state(CC_STATE_OPEN, CC_ORIENTATION_NONE);
/*	} else {
		ret = tusb1044_vdd_switch(chip, false);
		if (ret < 0) {
			dev_err(&client->dev, "TUSB1044 %s - Error: TUSB1044 can't be powered:%d\n", __func__, ret);
			return ret;
		}
		chip->vdd_state = false;
	}
*/

	chip->probe_done = true;

	dev_err(&client->dev,
		 "TUSB1044 %s - probing TUSB1044 i2c driver done\n", __func__);
	return ret;
}

static int tusb1044_i2c_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "TUSB1044 %s - remove TUSB1044 i2c driver\n",
		 __func__);
	return 0;
}

static const struct of_device_id tusb1044_i2c_match_table[] = {
	{.compatible = "ti,tusb1044-i2c" },
	{},
};

static const struct i2c_device_id tusb1044_i2c_id[] = {
	{ "tusb1044-i2c", 0 },
	{ }
};

static struct i2c_driver tusb1044_i2c_driver = {
	.driver = {
		.name		= "tusb1044-i2c",
		.owner		= THIS_MODULE,
		.of_match_table = tusb1044_i2c_match_table,
	},
	.probe		= tusb1044_i2c_probe,
	.remove		= tusb1044_i2c_remove,
	.id_table	= tusb1044_i2c_id,
};
module_i2c_driver(tusb1044_i2c_driver);

