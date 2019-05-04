/* drivers/input/keyled.c
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
#include <linux/of_gpio.h>
#include <linux/keyled.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/keycombo.h>

struct keyled_state {
	struct platform_device *pdev_child;
};

extern void virtual_key_led_reset_blink(int onoff);
static void keep_led_blink(struct work_struct *);
DECLARE_WORK(led_blink_work, &keep_led_blink);
static void keep_led_blink(struct work_struct *dummy)
{
	KEY_LOGI("%s: start blinking!\n", __func__);
	virtual_key_led_reset_blink(1);
}
static void virtual_key_led_blink(void *dummy)
{
	schedule_work(&led_blink_work);
}
static void stop_led_blink(void *dummy)
{
	cancel_work_sync(&led_blink_work);
	KEY_LOGI("%s: stop blinking!\n", __func__);
	virtual_key_led_reset_blink(0);
}

static int keyled_parse_dt(struct device_node *dt,
		struct keyled_platform_data *pdata)
{
	int ret = 0, cnt = 0, num_keys;
	struct property *prop;
	char parser_st[3][15] = {"key_down_delay", "keys_down", "keys_up"};

	/* Parse key_down_delay */
	if (of_property_read_u32(dt, parser_st[0], &pdata->key_down_delay))
		KEY_LOGI("DT:%s parser gets nothing\n", parser_st[0]);

	KEY_LOGI("DT:%s=%d\n", parser_st[0], pdata->key_down_delay);

	/* Parse keys_down keycode */
	/* Must have keys_down property */
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
	/* keys_up property is optional */
	prop = of_find_property(dt, parser_st[2], NULL);
	if (!prop) {
		KEY_LOGI("DT:%s property not found\n", parser_st[2]);
		return 0;
	} else {
		num_keys = prop->length / sizeof(uint32_t);
		KEY_LOGI("DT:%s num_keys=%d\n", parser_st[2], num_keys);
	}

	pdata->keys_up = kzalloc(num_keys * sizeof(uint32_t), GFP_KERNEL);
	if (!pdata->keys_up) {
		KEY_LOGE("DT:%s fail to allocate memory\n", parser_st[2]);
		ret = -ENOMEM;
		goto err_parse_keys_up_failed;
	}

	if (of_property_read_u32_array(dt, parser_st[2], pdata->keys_up, num_keys)) {
		KEY_LOGE("DT:%s parse err\n", parser_st[2]);
		ret = -EINVAL;
		goto err_keys_up_failed;
	}

	for(cnt = 0; cnt < num_keys; cnt++)
		KEY_LOGI("DT:%s=%d\n", parser_st[2], pdata->keys_up[cnt]);

	return 0;

err_keys_up_failed:
	kfree(pdata->keys_up);
err_parse_keys_up_failed:
err_keys_down_failed:
	kfree(pdata->keys_down);
err_parse_keys_down_failed:
	return ret;
}

static int keyled_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct keycombo_platform_data *pdata_child;
	struct keyled_platform_data *pdata;
	int up_size = 0, down_size = 0, size;
	int key, *keyp;
	struct keyled_state *state;

	KEY_LOGI("%s: +++\n", __func__);

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(struct keyled_platform_data), GFP_KERNEL);
		if (!pdata) {
			KEY_LOGE("[KEY] fail to allocate keyled_platform_data\n");
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}
		ret = keyled_parse_dt(pdev->dev.of_node, pdata);
		if (ret < 0) {
			KEY_LOGE("[KEY] keyled_parse_dt fail\n");
			ret = -ENOMEM;
			goto err_parse_fail;
		}
	} else {
		pdata = pdev->dev.platform_data;
		if(!pdata) {
			KEY_LOGE("[KEY] keyled_platform_data does not exist\n");
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}
	}

	state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		goto err_parse_fail;
	}

	state->pdev_child = platform_device_alloc(KEYCOMBO_NAME,
							PLATFORM_DEVID_AUTO);
	if (!state->pdev_child) {
		ret = -ENOMEM;
		goto err_parse_fail;
	}
	state->pdev_child->dev.parent = &pdev->dev;

	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		down_size++;
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			up_size++;
		}
	}
	size = sizeof(struct keycombo_platform_data);
	pdata_child = devm_kzalloc(&pdev->dev, sizeof(struct keycombo_platform_data),
				GFP_KERNEL);
	if (!pdata_child)
		goto error;

	pdata_child->keys_down = devm_kzalloc(&pdev->dev,
				sizeof(uint32_t) * (down_size + 1), GFP_KERNEL);
	if (!pdata_child->keys_down)
		goto error;

	memcpy(pdata_child->keys_down, pdata->keys_down,
						sizeof(uint32_t) * down_size);
	if (!pdata_child->keys_down)
		goto error;

	if (up_size > 0) {
		pdata_child->keys_up = devm_kzalloc(&pdev->dev,
					sizeof(uint32_t) * (up_size + 1), GFP_KERNEL);
		if (!pdata_child->keys_up)
			goto error;
		memcpy(pdata_child->keys_up, pdata->keys_up,
							sizeof(uint32_t) * up_size);
		if (!pdata_child->keys_up)
			goto error;
	}
	pdata_child->key_down_fn = &virtual_key_led_blink;
	pdata_child->key_up_fn = &stop_led_blink;
	pdata_child->priv = state;
	pdata_child->key_down_delay = pdata->key_down_delay;
	ret = platform_device_add_data(state->pdev_child, pdata_child, size);
	if (ret)
		goto error;
	platform_set_drvdata(pdev, state);

	KEY_LOGI("%s: ---\n", __func__);

	return platform_device_add(state->pdev_child);
error:
	platform_device_put(state->pdev_child);
err_parse_fail:
	if (pdev->dev.of_node) {
		if(pdata->keys_up)
			kfree(pdata->keys_up);
		if(pdata->keys_down)
			kfree(pdata->keys_down);
		kfree(pdata);
	}

err_get_pdata_fail:
	return ret;
}

int keyled_remove(struct platform_device *pdev)
{
	struct keyled_state *state = platform_get_drvdata(pdev);
	platform_device_put(state->pdev_child);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id keyled_mttable[] = {
	{ .compatible = KEYLED_NAME},
	{},
};
#else
#define keyled_mttable NULL
#endif

struct platform_driver keyled_driver = {
	.probe = keyled_probe,
	.remove = keyled_remove,
	.driver = {
		.name = KEYLED_NAME,
		.owner = THIS_MODULE,
		.of_match_table = keyled_mttable,
	},
};

static int __init keyled_init(void)
{
	return platform_driver_register(&keyled_driver);
}

static void __exit keyled_exit(void)
{
	return platform_driver_unregister(&keyled_driver);
}

module_init(keyled_init);
module_exit(keyled_exit);

