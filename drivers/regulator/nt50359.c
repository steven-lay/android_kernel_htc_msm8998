/*
 * NOVATEK NT50359 Panel Power IC
 *
 *			Copyright (C) 2017 HTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_data/nt50359.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/fb.h>

#define NT50359_MAX_REGISTER      0xFF

extern int dsi_register_notifier(struct dsi_status_notifier *notifier);

static struct nt50359 *g_nt = NULL;
static int nt50359_power_status = REGULATOR_POWER_UNKNOW;

/* HTC regcmd function --- start*/

static bool nt50359_exec_check_cmds(struct regmap *regmap, struct nt50359_regcmd *cmds, int num)
{
	int i = 0, ret = 0;
	unsigned int reg_read_val;
	struct nt50359_regcmd *m_cmd = cmds;

	if (!regmap){
		pr_err("%s, regmap is NULL by pass check \n", __func__);
		return true;
	}

	if (!cmds){
		pr_err("%s, regcmd is NULL no check items \n", __func__);
		return true;
	}

	for (i=0; i<num ;i++){
		ret = regmap_read(regmap, m_cmd->address, &reg_read_val);
		if (ret){
			pr_err("%s : regmap_read fail:address:%x,parameter1:%x\n", __func__, m_cmd->address, m_cmd->parameter1);
			return false;
		}

		if (reg_read_val != m_cmd->parameter1){
			pr_err("%s : register check NG:address:%x,parameter1:%x,read_result:%x\n", __func__, m_cmd->address, m_cmd->parameter1, reg_read_val);
			return false;
		}else{
			pr_debug("%s : register check OK:address:%x,parameter1:%x,read_result:%x\n", __func__, m_cmd->address, m_cmd->parameter1, reg_read_val);
		}
		m_cmd++;
	}

	return true;
}



static struct nt50359_regcmd *nt50359_parse_dt_reg_check_cmds(struct device *dev, const char *cmd_key, int *cmd_num)
{

	const char *data;
	struct nt50359_regcmd *m_cmds, *p_cmd;
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

static void nt50359_parse_dt_reg_check_cmd_sets(struct device *dev, struct nt50359_regcmd_sets *cmd_sets, struct regmap *regmap)
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

	cmd_sets->power_on_check_cmds = nt50359_parse_dt_reg_check_cmds(dev, "reg-check-cmds", &cmd_sets->power_on_cmds_num);

}

static void nt50359_run_power_on_check_cmds(struct nt50359_regcmd_sets *cmd_sets)
{


	if (!cmd_sets){
		pr_err("%s no register command to check\n", __func__ );
		return;
	}


	if (cmd_sets->power_on_cmds_num && cmd_sets->power_on_check_cmds){
		/* do register check and retry flow */
		int i;
		bool check_result = true;
		/* read register after avee pull high 5ms */
		for (i = 0;i< g_nt->retry_times;i++){
			usleep_range(5000, 5500);
			check_result = nt50359_exec_check_cmds(cmd_sets->regmap,
				cmd_sets->power_on_check_cmds, cmd_sets->power_on_cmds_num);
			if (!check_result){
				/* do reset avee gpio flow */
				if (gpio_is_valid(g_nt->avee_gpio)){
					gpio_set_value(g_nt->avee_gpio, 0);
					pr_err("%s gpio:%d set value:%d\n", __func__, g_nt->avee_gpio, 0);
					usleep_range(1000, 1500);
					gpio_set_value(g_nt->avee_gpio, 1);
					pr_err("%s gpio:%d set value:%d\n", __func__, g_nt->avee_gpio, 1);
				}
			}else{
				break;
			}
		}
	}
}

/* HTC regcmd function --- end*/

static int nt50359_parse_dt(struct nt50359 *nt)
{
	struct device *dev = nt->dev;
	struct device_node *node = dev->of_node;

	if (!node) {
		dev_err(dev, "no nt50359 device node resource\n");
		return -EINVAL;
	}

	of_property_read_u32(node, "retry-times", &nt->retry_times);
	nt->reg_check_enable = of_property_read_bool(node,"reg-check-enable");
	nt->avee_gpio = of_get_named_gpio(node, "avee-gpio", 0);

	return 0;
}

static void dsi_status_detect(int status)
{

	switch (status) {
	case LCM_POWERON:

		if (nt50359_power_status == REGULATOR_POWER_ON){
			return;
		}else{
			nt50359_power_status = REGULATOR_POWER_ON;
		}

		if (g_nt){
			if (g_nt->reg_check_enable){
				nt50359_run_power_on_check_cmds(&g_nt->power_on_check_cmd_sets);
			}
		}
		break;
	case LCM_POWERDOWN:
		nt50359_power_status = REGULATOR_POWER_OFF;
	break;

	};

}

static struct dsi_status_notifier dsi_event_notifier = {
	.name = "nt50359_dsi_event_handler",
	.func = dsi_status_detect,
};


static int nt50359_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct nt50359 *nt;
	struct regmap_config regmap_cfg;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	nt = devm_kzalloc(&cl->dev, sizeof(struct nt50359), GFP_KERNEL);
	if (!nt)
		return -ENOMEM;

	nt->client = cl;
	nt->dev = &cl->dev;
	nt50359_parse_dt(nt);

	/* Setup regmap */
	memset(&regmap_cfg, 0, sizeof(struct regmap_config));
	regmap_cfg.reg_bits = 8;
	regmap_cfg.val_bits = 8;
	regmap_cfg.name = id->name;
	regmap_cfg.max_register = NT50359_MAX_REGISTER;

	nt->regmap = devm_regmap_init_i2c(cl, &regmap_cfg);

	if (IS_ERR(nt->regmap))
	{
		return PTR_ERR(nt->regmap);
	}

	nt50359_parse_dt_reg_check_cmd_sets(nt->dev, &nt->power_on_check_cmd_sets, nt->regmap);

	i2c_set_clientdata(cl, nt);

	g_nt = nt;

	dsi_register_notifier(&dsi_event_notifier);

	return 0;
}


static const struct of_device_id nt50359_dt_ids[] = {
	{ .compatible = "novatek,nt50359", },
	{ }
};
MODULE_DEVICE_TABLE(of, nt50359_dt_ids);

static const struct i2c_device_id nt50359_ids[] = {
	{"nt50359", NT50359},
	{ }
};
MODULE_DEVICE_TABLE(i2c, nt50359_ids);

static struct i2c_driver nt50359_driver = {
	.driver = {
		   .name = "nt50359",
		   .of_match_table = of_match_ptr(nt50359_dt_ids),
		   },
	.probe = nt50359_probe,
	.id_table = nt50359_ids,
};

module_i2c_driver(nt50359_driver);

MODULE_DESCRIPTION("NOVATEK nt50359 driver");
MODULE_LICENSE("GPL");