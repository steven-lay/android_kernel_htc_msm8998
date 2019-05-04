/*
 * Driver for the TPS61099 external power source
 *
 * Copyright (C) 2017 HTC Corporation.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/usb/htc_info.h>

struct tps61099_power {
	struct device		*dev;

	bool			power_enabled;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_power_enable;
	struct pinctrl_state	*pinctrl_power_disable;

	struct regulator_desc	pswitch_rdesc;
	struct regulator_dev	*pswitch_rdev;
};

static int tps61099_pswitch_regulator_enable(struct regulator_dev *rdev)
{
	struct tps61099_power *tp = rdev_get_drvdata(rdev);
	int ret = 0;

	ret = pinctrl_select_state(tp->pinctrl, tp->pinctrl_power_enable);
	if (ret < 0) {
		pr_err("TPS61099 %s - Error: pinctrl power enable state failed, ret = %d\n", __func__, ret);
		return ret;
	}

	pr_info("TPS61099 external power source enabled\n");

	tp->power_enabled = true;
	return 0;
}

static int tps61099_pswitch_regulator_disable(struct regulator_dev *rdev)
{
	struct tps61099_power *tp = rdev_get_drvdata(rdev);
	int ret = 0;

	ret = pinctrl_select_state(tp->pinctrl, tp->pinctrl_power_disable);
	if (ret < 0) {
		pr_err("TPS61099 %s - Error: pinctrl power enable state failed, ret = %d\n", __func__, ret);
		return ret;
	}

	pr_info("TPS61099 external power source disabled\n");

	tp->power_enabled = false;
	return 0;
}

static int tps61099_pswitch_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct tps61099_power *tp = rdev_get_drvdata(rdev);

	return tp->power_enabled;
}

static struct regulator_ops tps61099_pswitch_regulator_ops = {
	.enable		= tps61099_pswitch_regulator_enable,
	.disable	= tps61099_pswitch_regulator_disable,
	.is_enabled	= tps61099_pswitch_regulator_is_enabled,
};

static int tps61099_regulator_init(struct tps61099_power *tp) {
	struct device *dev = tp->dev;
	struct regulator_config cfg = {};
	struct regulator_init_data *init_data;
	int ret = 0;

	init_data = devm_kzalloc(dev, sizeof(*init_data), GFP_KERNEL);
	if (!init_data)
		return -ENOMEM;

	init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	tp->pswitch_rdesc.owner = THIS_MODULE;
	tp->pswitch_rdesc.type = REGULATOR_VOLTAGE;
	tp->pswitch_rdesc.ops = &tps61099_pswitch_regulator_ops;
	tp->pswitch_rdesc.name = kbasename(dev->of_node->full_name);

	cfg.dev = dev;
	cfg.init_data = init_data;
	cfg.driver_data = tp;
	cfg.of_node = dev->of_node;

	tp->pswitch_rdev = devm_regulator_register(dev, &tp->pswitch_rdesc, &cfg);
	if (IS_ERR(tp->pswitch_rdev)) {
		ret = PTR_ERR(tp->pswitch_rdev);
		pr_err("TPS61099 - %s: regulator register failed:%d\n", __func__, ret);
	}

	return 0;
}

static int htc_tps61099_probe(struct platform_device *pdev)
{
	struct tps61099_power *tp;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_power_enable;
	struct pinctrl_state *pinctrl_power_disable;
	int ret = 0;

	tp = devm_kzalloc(&pdev->dev, sizeof(struct tps61099_power), GFP_KERNEL);
	if (!tp)
		return -ENOMEM;

	tp->dev = &pdev->dev;

	pinctrl = devm_pinctrl_get(tp->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("TPS61099 %s - Error: pinctrl not ready: %d\n", __func__, ret);
		return ret;
	}

	pinctrl_power_enable = pinctrl_lookup_state(pinctrl, "tps61099_active");
	if (IS_ERR(pinctrl_power_enable)) {
		ret = PTR_ERR(pinctrl_power_enable);
		pr_err("TPS61099 %s - Error: no tps61099_active pinctrl state: %d\n", __func__, ret);
		return ret;
	}

	pinctrl_power_disable = pinctrl_lookup_state(pinctrl, "tps61099_sleep");
	if (IS_ERR(pinctrl_power_disable)) {
		ret = PTR_ERR(pinctrl_power_disable);
		pr_err("TPS61099 %s - Error: no tps61099_sleep pinctrl state: %d\n", __func__, ret);
		return ret;
	}

	ret = pinctrl_select_state(pinctrl, pinctrl_power_disable);
	if (ret < 0) {
		pr_err("TPS61099 %s - Error: external power is still enabled\n", __func__);
	}

	tp->pinctrl = pinctrl;
	tp->pinctrl_power_enable = pinctrl_power_enable;
	tp->pinctrl_power_disable = pinctrl_power_disable;
	tp->power_enabled = false;

	tps61099_regulator_init(tp);

	platform_set_drvdata(pdev, tp);
	pr_info("TPS61099 %s - probe TPS61099 driver\n", __func__);
	return 0;
}

static int htc_tps61099_remove(struct platform_device *pdev)
{
	pr_info("TPS61099 %s - remove TPS61099 driver\n", __func__);
	return 0;
}

static void htc_tps61099_shutdown(struct platform_device *pdev)
{
	struct tps61099_power *tp = platform_get_drvdata(pdev);
	int ret = 0;

	if (!tp) {
		pr_err("%s, no driver data\n", __func__);
		return;
	}
	if (tp->power_enabled) {
		ret = pinctrl_select_state(tp->pinctrl, tp->pinctrl_power_disable);
		if (ret < 0) {
			pr_err("TPS61099 %s - set pinctrl power disable state failed\n", __func__);
			return;
		}
		tp->power_enabled = false;
	}
	pr_info("TPS61099 %s - clear setting during device shutdown\n", __func__);
	return;
}

static const struct of_device_id tps61099_dt_match_table[] = {
	{.compatible = "htc,tps61099" },
	{},
};
MODULE_DEVICE_TABLE(of, tps61099_dt_match_table);

static struct platform_driver htc_tps61099_driver = {
	.driver = {
		.name		= "htc,tps61099",
		.owner		= THIS_MODULE,
		.of_match_table = tps61099_dt_match_table,
	},
	.probe		= htc_tps61099_probe,
	.remove		= htc_tps61099_remove,
	.shutdown	= htc_tps61099_shutdown,
};

static int __init htc_tps61099_init(void)
{
	return platform_driver_register(&htc_tps61099_driver);
}

static void __exit htc_tps61099_exit(void)
{
	platform_driver_unregister(&htc_tps61099_driver);
}

MODULE_DESCRIPTION("TPS61099 external power source");
MODULE_LICENSE("GPL");

module_init(htc_tps61099_init);
module_exit(htc_tps61099_exit);
