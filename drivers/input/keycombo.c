/* drivers/input/keycombo.c
 *
 * Copyright (C) 2014 Google, Inc.
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

#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/keycombo.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>

static unsigned int vzw_key_flag = 0;

struct keycombo_state {
	struct input_handler input_handler;
	unsigned long keybit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long upbit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long key[BITS_TO_LONGS(KEY_CNT)];
	spinlock_t lock;
	struct  workqueue_struct *wq;
	int key_down_target;
	int key_down;
	int key_up;
	struct delayed_work key_down_work;
	int delay;
	struct work_struct key_up_work;
	void (*key_up_fn)(void *);
	void (*key_down_fn)(void *);
	void *priv;
	int key_is_down;
	struct wakeup_source combo_held_wake_source;
	struct wakeup_source combo_up_wake_source;
};


#if defined(CONFIG_POWER_KEY_CLR_RESET)
static uint32_t clr_gpio;
static struct pinctrl *key_pinctrl;
static struct kobject *android_key_kobj;

static int keycombo_pinctrl_configure(struct pinctrl *key_pinctrl, bool active);
void clear_hw_reset(void)
{
	int error;

	KEY_LOGI("clear_hw_reset++++++++++(%d)\n", clr_gpio);
	if (key_pinctrl) {
		error = keycombo_pinctrl_configure(key_pinctrl, true);
		if (error == 0)
			KEY_LOGI("set pinctrl active\n");
	} else {
		if (gpio_direction_output(clr_gpio, 0) < 0)
			KEY_LOGE("gpio_direction_output GPIO %d failed\n", clr_gpio);
	}

	msleep(100);

	if (key_pinctrl) {
		error = keycombo_pinctrl_configure(key_pinctrl, false);
		if (error == 0)
			KEY_LOGI("set pinctrl normal\n");
	} else {
		if (gpio_direction_input(clr_gpio) < 0)
			KEY_LOGE("gpio_direction_input GPIO %d failed\n", clr_gpio);
	}
	KEY_LOGI("clear_hw_reset----------\n");
}

static void keep_hw_reset_clear(struct work_struct *);
DECLARE_DELAYED_WORK(clear_restart_work, &keep_hw_reset_clear);

static void keep_hw_reset_clear(struct work_struct *dummy)
{
	clear_hw_reset();
	schedule_delayed_work(&clear_restart_work, msecs_to_jiffies(1000));
}

static void start_reset_clear(void *dummy)
{
	schedule_delayed_work(&clear_restart_work, msecs_to_jiffies(1000));
}

static void stop_clearing(void *dummy)
{
	cancel_delayed_work_sync(&clear_restart_work);
}

static ssize_t clr_gpio_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned char value;

	if (kstrtou8(buf, 10, &value) < 0) {
		KEY_LOGI("%s: input out of range\n",__func__);
		return -EINVAL;
	}

	value = value ? true : false;

	if (gpio_direction_output(clr_gpio, value) < 0)
		KEY_LOGE("gpio_direction_output GPIO %d failed\n", clr_gpio);
	else
		KEY_LOGI("%s, set gpio value = %d\n", __func__, value);

	return count;
}

static ssize_t clr_gpio_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(clr_gpio));
}

static DEVICE_ATTR(clr_gpio, S_IRUGO | S_IWUSR , clr_gpio_show, clr_gpio_store);

static int keycombo_sysfs_init(void)
{
	int ret = 0;

	android_key_kobj = kobject_create_and_add("android_key", NULL);

	if (android_key_kobj == NULL) {
		KEY_LOGE("%s: create kobj failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(android_key_kobj, &dev_attr_clr_gpio.attr);
	if (ret) {
		KEY_LOGE("%s: sysfs_create_file clr_gpio failed\n", __func__);
		return ret;
	}

	return 0;
}

static void keycombo_sysfs_remove(void)
{
	sysfs_remove_file(android_key_kobj, &dev_attr_clr_gpio.attr);
	kobject_put(android_key_kobj);
}

static int keycombo_pinctrl_configure(struct pinctrl *key_pinctrl,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(key_pinctrl,
						"keycombo_active");
		if (IS_ERR(set_state)) {
			KEY_LOGE("%s: cannot get pinctrl active state\n", __func__);
			return PTR_ERR(set_state);
		}
	} else {
		set_state =
			pinctrl_lookup_state(key_pinctrl,
						"keycombo_normal");
		if (IS_ERR(set_state)) {
			KEY_LOGE("%s: cannot get pinctrl normal state\n", __func__);
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(key_pinctrl, set_state);
	if (retval) {
		KEY_LOGE("%s: cannot set pinctrl, active = %d\n", __func__, active);
		return retval;
	}

	return 0;
}
#endif /* CONFIG_POWER_KEY_CLR_RESET */

static void do_key_down(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work,
									work);
	struct keycombo_state *state = container_of(dwork,
					struct keycombo_state, key_down_work);
	if (state->key_down_fn)
		state->key_down_fn(state->priv);
}

static void do_key_up(struct work_struct *work)
{
	struct keycombo_state *state = container_of(work, struct keycombo_state,
								key_up_work);
	if (state->key_up_fn)
		state->key_up_fn(state->priv);
	__pm_relax(&state->combo_up_wake_source);
}

static void keycombo_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	unsigned long flags;
	struct keycombo_state *state = handle->private;

	if (type != EV_KEY)
		return;

	if (code >= KEY_MAX)
		return;

	if (!test_bit(code, state->keybit))
		return;

	spin_lock_irqsave(&state->lock, flags);
	if (!test_bit(code, state->key) == !value)
		goto done;
	__change_bit(code, state->key);
	if (test_bit(code, state->upbit)) {
		if (value)
			state->key_up++;
		else
			state->key_up--;
	} else {
		if (value)
			state->key_down++;
		else
			state->key_down--;
	}
	if (state->key_down == state->key_down_target && state->key_up == 0) {
		__pm_stay_awake(&state->combo_held_wake_source);
		state->key_is_down = 1;
		if (queue_delayed_work(state->wq, &state->key_down_work,
								state->delay))
			pr_debug("Key down work already queued!");
	} else if (state->key_is_down) {
		if (!cancel_delayed_work(&state->key_down_work)) {
			__pm_stay_awake(&state->combo_up_wake_source);
			queue_work(state->wq, &state->key_up_work);
		}
		__pm_relax(&state->combo_held_wake_source);
		state->key_is_down = 0;
	}
done:
	spin_unlock_irqrestore(&state->lock, flags);
}

static int keycombo_connect(struct input_handler *handler,
		struct input_dev *dev,
		const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;
	struct keycombo_state *state =
		container_of(handler, struct keycombo_state, input_handler);
	for (i = 0; i < KEY_MAX; i++) {
		if (test_bit(i, state->keybit) && test_bit(i, dev->keybit))
			break;
	}
	if (i == KEY_MAX)
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = KEYCOMBO_NAME;
	handle->private = state;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keycombo_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static int keycombo_parse_dt(struct device_node *dt,
		struct keycombo_platform_data *pdata)
{
	int ret = 0, cnt = 0, num_keys;
	struct property *prop;
	char parser_st[4][15] = {"key_down_delay", "keys_down", "keys_up"
#if defined(CONFIG_POWER_KEY_CLR_RESET)
					, "clr_gpio"
#endif
				};

	if (vzw_key_flag) {
		KEY_LOGI("DT: customize\n");
		snprintf(parser_st[1], 15, "vzw_keys_down");
		snprintf(parser_st[2], 15, "vzw_keys_up");
	}

	/* Parse key_down_delay */
	if (of_property_read_u32(dt, parser_st[0], &pdata->key_down_delay))
		KEY_LOGI("DT:%s parser gets nothing\n", parser_st[0]);

	KEY_LOGI("DT:%s=%d\n", parser_st[0], pdata->key_down_delay);

	/* Parse keys_down keycode */
	prop = of_find_property(dt, parser_st[1], NULL);
	if (!prop) {
		KEY_LOGE("DT:%s property not found\n", parser_st[1]);
		ret = -EINVAL;
		goto err_parse_keys_down_failed;
	} else {
		num_keys = prop->length / sizeof(uint32_t);
		KEY_LOGI("DT:%s num_keys=%d\n", parser_st[1], num_keys);
	}

	pdata->keys_down = kzalloc(num_keys * sizeof(uint32_t), GFP_KERNEL);
	if (!pdata->keys_down) {
		KEY_LOGE("DT:%s fail to allocate memory\n", parser_st[1]);
		ret = -ENOMEM;
		goto err_parse_keys_down_failed;
	}

	if (of_property_read_u32_array(dt, parser_st[1], pdata->keys_down, num_keys)) {
		KEY_LOGE("DT:%s parse err\n", parser_st[1]);
		ret = -EINVAL;
		goto err_keys_down_failed;
	}

	for(cnt = 0; cnt < num_keys; cnt++)
		KEY_LOGI("DT:%s=%d\n", parser_st[1], pdata->keys_down[cnt]);

	/* Parse keys_up keycode */
	prop = of_find_property(dt, parser_st[2], NULL);
	if (!prop) {
		KEY_LOGE("DT:%s property not found\n", parser_st[2]);
		ret = -EINVAL;
		goto err_parse_keys_up_ailed;
	} else {
		num_keys = prop->length / sizeof(uint32_t);
		KEY_LOGI("DT:%s num_keys=%d\n", parser_st[2], num_keys);
	}

	pdata->keys_up = kzalloc(num_keys * sizeof(uint32_t), GFP_KERNEL);
	if (!pdata->keys_up) {
		KEY_LOGE("DT:%s fail to allocate memory\n", parser_st[2]);
		ret = -ENOMEM;
		goto err_parse_keys_up_ailed;
	}

	if (of_property_read_u32_array(dt, parser_st[2], pdata->keys_up, num_keys)) {
		KEY_LOGE("DT:%s parse err\n", parser_st[2]);
		ret = -EINVAL;
		goto err_keys_up_failed;
	}

	for(cnt = 0; cnt < num_keys; cnt++)
		KEY_LOGI("DT:%s=%d\n", parser_st[2], pdata->keys_up[cnt]);

#if defined(CONFIG_POWER_KEY_CLR_RESET)
	/* Parse clr_gpio */
	ret = of_get_named_gpio(dt, parser_st[3], 0);
	if (!gpio_is_valid(ret))
		KEY_LOGI("DT:%s parser fails/gets nothing, ret=%d\n", parser_st[3], ret);
	else
		pdata->clr_gpio = ret;

	KEY_LOGI("DT:%s=(%d)\n", parser_st[3], pdata->clr_gpio);
#endif

	return 0;

err_keys_up_failed:
	kfree(pdata->keys_up);
err_parse_keys_up_ailed:
err_keys_down_failed:
	kfree(pdata->keys_down);
err_parse_keys_down_failed:
	return ret;
}

static const struct input_device_id keycombo_ids[] = {
		{
				.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
				.evbit = { BIT_MASK(EV_KEY) },
		},
		{ },
};
MODULE_DEVICE_TABLE(input, keycombo_ids);

static int keycombo_probe(struct platform_device *pdev)
{
	int ret;
	int key, *keyp;
	struct keycombo_state *state;
	struct keycombo_platform_data *pdata;

	KEY_LOGI("%s: +++\n", __func__);

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(struct keycombo_platform_data), GFP_KERNEL);
		if (!pdata) {
			KEY_LOGE("[KEY] fail to allocate keycombo_platform_data\n");
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}
		ret = keycombo_parse_dt(pdev->dev.of_node, pdata);
		if (ret < 0) {
			KEY_LOGE("[KEY] keycombo_parse_dt fail\n");
			ret = -ENOMEM;
			goto err_parse_fail;
		}
	} else {
		pdata = pdev->dev.platform_data;
		if(!pdata) {
			KEY_LOGE("[KEY] keycombo_platform_data does not exist\n");
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state) {
		KEY_LOGE("[KEY] fail to allocate keycombo_state\n");
		ret = -ENOMEM;
		goto err_parse_fail;
	}

	spin_lock_init(&state->lock);
	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		state->key_down_target++;
		__set_bit(key, state->keybit);
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			__set_bit(key, state->keybit);
			__set_bit(key, state->upbit);
		}
	}

	state->wq = alloc_ordered_workqueue("keycombo", 0);
	if (!state->wq) {
		KEY_LOGE("[KEY] fail to allocate keycombo workqueue\n");
		ret = -ENOMEM;
		goto err_wq_alloc_fail;
	}

	state->priv = pdata->priv;

#if defined(CONFIG_POWER_KEY_CLR_RESET)
	if(state->priv == NULL) {
		/* Get pinctrl if target uses pinctrl */
		key_pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(key_pinctrl)) {
			if (PTR_ERR(key_pinctrl) == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			KEY_LOGI("Target does not use pinctrl\n");
			key_pinctrl = NULL;
		} else {
			if (key_pinctrl) {
				KEY_LOGI("control clr_gpio by pinctrl\n");
				ret = keycombo_pinctrl_configure(key_pinctrl, false);
				if (ret) {
					KEY_LOGE("%s: fail to set pinctrl to normal state, %d\n",
							__func__, ret);
					goto err_pinctrl_configure_fail;
				}
			}
		}

		if (gpio_is_valid(pdata->clr_gpio)) {
			ret = gpio_request(pdata->clr_gpio, "pwr_mistouch");
			if (ret) {
				KEY_LOGE("%s: unable to request gpio %d (%d)\n",
					__func__, pdata->clr_gpio, ret);
				goto err_wq_alloc_fail;
			}

			clr_gpio = pdata->clr_gpio;

			/* Contrl gpio directly if target does not use pinctrl */
			if(key_pinctrl == NULL) {
				KEY_LOGI("control clr_gpio directly\n");
				ret = gpio_direction_input(pdata->clr_gpio);
				if (ret) {
					KEY_LOGE("%s: Unable to set direction, %d\n",
							__func__, ret);
					goto err_wq_alloc_fail;
				}
			}
		} else {
			KEY_LOGE("%s: clr_gpio is not defined\n", __func__);
			ret = -EIO;
			goto err_wq_alloc_fail;
		}

		pdata->key_down_fn	= &start_reset_clear;
		pdata->key_up_fn	= &stop_clearing;
	}
#endif

	if (pdata->key_down_fn)
		state->key_down_fn = pdata->key_down_fn;
	INIT_DELAYED_WORK(&state->key_down_work, do_key_down);

	if (pdata->key_up_fn)
		state->key_up_fn = pdata->key_up_fn;
	INIT_WORK(&state->key_up_work, do_key_up);

	wakeup_source_init(&state->combo_held_wake_source, "key combo");
	wakeup_source_init(&state->combo_up_wake_source, "key combo up");
	state->delay = msecs_to_jiffies(pdata->key_down_delay);

	state->input_handler.event = keycombo_event;
	state->input_handler.connect = keycombo_connect;
	state->input_handler.disconnect = keycombo_disconnect;
	state->input_handler.name = KEYCOMBO_NAME;
	state->input_handler.id_table = keycombo_ids;
	ret = input_register_handler(&state->input_handler);
	if (ret) {
		KEY_LOGE("[KEY] fail to register keycombo input handler\n");
		goto err_input_handler_fail;
	}
	platform_set_drvdata(pdev, state);

#if defined(CONFIG_POWER_KEY_CLR_RESET)
	if(state->priv == NULL){
		keycombo_sysfs_init();
	}
#endif

	KEY_LOGI("%s: ---\n", __func__);
	return 0;

err_input_handler_fail:
	destroy_workqueue(state->wq);
#if defined(CONFIG_POWER_KEY_CLR_RESET)
err_pinctrl_configure_fail:
#endif
err_wq_alloc_fail:
	kfree(state);
err_parse_fail:
	if (pdev->dev.of_node)
		kfree(pdata);
err_get_pdata_fail:
	return ret;
}

int keycombo_remove(struct platform_device *pdev)
{
	struct keycombo_state *state = platform_get_drvdata(pdev);
#if defined(CONFIG_POWER_KEY_CLR_RESET)
	keycombo_sysfs_remove();
#endif
	input_unregister_handler(&state->input_handler);
	destroy_workqueue(state->wq);
	kfree(state);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id keycombo_mttable[] = {
	{ .compatible = KEYCOMBO_NAME},
	{},
};
#else
#define keycombo_mttable NULL
#endif

struct platform_driver keycombo_driver = {
	.probe = keycombo_probe,
	.remove = keycombo_remove,
	.driver = {
		.name = KEYCOMBO_NAME,
		.owner = THIS_MODULE,
		.of_match_table = keycombo_mttable,
	},
};

static int __init keycombo_init(void)
{
	return platform_driver_register(&keycombo_driver);
}

static void __exit keycombo_exit(void)
{
	return platform_driver_unregister(&keycombo_driver);
}

module_init(keycombo_init);
module_exit(keycombo_exit);

static int __init vzw_key_enable_flag(char *str)
{
	int ret = kstrtouint(str, 0, &vzw_key_flag);
	pr_info("vzw_key_enable %d: %d from %s",
			ret, vzw_key_flag, str);
	return ret;
} early_param("vzw_key_enable", vzw_key_enable_flag);
