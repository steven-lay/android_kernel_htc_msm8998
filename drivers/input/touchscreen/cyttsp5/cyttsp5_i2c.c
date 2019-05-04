/*
 * cyttsp5_i2c.c
 * Parade TrueTouch(TM) Standard Product V5 I2C Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2012-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 *
 */

#include "cyttsp5_regs.h"
#include "cyttsp5_core.h"
#include "cyttsp5_platform.h"


#include <linux/i2c.h>
#include <linux/version.h>

#define CY_I2C_DATA_SIZE  (2 * 256)

//htc+
static char *htc_bootmode = NULL;
//htc-

#ifdef CONFIG_TOUCHSCREEN_MTK //htc

/* Button to keycode conversion */
static u16 cyttsp5_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_BACK,		/* 158 */
	KEY_HOMEPAGE,		/* 172 */
	KEY_MENU,		/* 139 */
	KEY_SEARCH,		/* 217 */
//	KEY_VOLUMEDOWN,		/* 114 */
//	KEY_VOLUMEUP,		/* 115 */
//	KEY_CAMERA,		/* 212 */
//	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp5_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp5_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp5_btn_keys),
	.tag = 0,
};

static struct cyttsp5_core_platform_data _cyttsp5_core_platform_data = {
	.irq_gpio = 0,
	.rst_gpio = 0,
	.hid_desc_register = 1,
	.xres = cyttsp5_xres,
	.init = cyttsp5_init,
	.power = cyttsp5_power,
	.detect = cyttsp5_detect,
	.irq_stat = cyttsp5_irq_stat,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Parade Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL,	/* &cyttsp5_sett_param_regs, */
		NULL,	/* &cyttsp5_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp5_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp5_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp5_sett_btn_keys,	/* button-to-keycode table */
	},
	.flags = CY_CORE_FLAG_RESTORE_PARAMETERS,
	.easy_wakeup_gesture = CY_CORE_EWG_NONE,
};

static const int16_t cyttsp5_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -127, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
	ABS_MT_DISTANCE, 0, 255, 0, 0,	/* Used with hover */
};

struct touch_framework cyttsp5_framework = {
	.abs = (uint16_t *)&cyttsp5_abs[0],
	.size = ARRAY_SIZE(cyttsp5_abs),
	.enable_vkeys = 0,
};

static struct cyttsp5_mt_platform_data _cyttsp5_mt_platform_data = {
	.frmwrk = &cyttsp5_framework,
	.flags = 0, //CY_MT_FLAG_INV_X | CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP5_MT_NAME,
	.vkeys_x = CY_VKEYS_X,
	.vkeys_y = CY_VKEYS_Y,
};

static struct cyttsp5_btn_platform_data _cyttsp5_btn_platform_data = {
	.inp_dev_name = CYTTSP5_BTN_NAME,
};

static const int16_t cyttsp5_prox_abs[] = {
	ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp5_prox_framework = {
	.abs = (uint16_t *)&cyttsp5_prox_abs[0],
	.size = ARRAY_SIZE(cyttsp5_prox_abs),
};

static struct cyttsp5_proximity_platform_data
		_cyttsp5_proximity_platform_data = {
	.frmwrk = &cyttsp5_prox_framework,
	.inp_dev_name = CYTTSP5_PROXIMITY_NAME,
};

static struct cyttsp5_platform_data _cyttsp5_platform_data = {
	.core_pdata = &_cyttsp5_core_platform_data,
	.mt_pdata = &_cyttsp5_mt_platform_data,
	.loader_pdata = &_cyttsp5_loader_platform_data,
	.btn_pdata = &_cyttsp5_btn_platform_data,
	.prox_pdata = &_cyttsp5_proximity_platform_data,
};

#endif //#ifdef CONFIG_TOUCHSCREEN_MTK //htc

extern struct tpd_device *tpd;

#ifdef USE_I2C_DMA
//#define MELFAS_I2C_MASTER_CLOCK 100
#define MELFAS_I2C_MASTER_CLOCK 400 //htc
static int cyttsp5_i2c_read_default(struct device *dev, void *buf, int size,
		u8 *I2CDMABuf_va, dma_addr_t I2CDMABuf_pa)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	if (!buf || !size || size > CY_I2C_DATA_SIZE)
		return -EINVAL;

	client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	rc = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, size);
	memcpy(buf, I2CDMABuf_va, size);
	client->ext_flag = client->ext_flag
		& (~I2C_DMA_FLAG) & (~I2C_ENEXT_FLAG);

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

static int cyttsp5_i2c_read_default_nosize(struct device *dev, u8 *buf, u32 max,
		u8 *I2CDMABuf_va, dma_addr_t I2CDMABuf_pa)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;
	u32 size;

	if (!buf)
		return -EINVAL;


	client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	rc = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, 2);
	memcpy(buf, I2CDMABuf_va, 2);
	client->ext_flag = client->ext_flag
			& (~I2C_DMA_FLAG) & (~I2C_ENEXT_FLAG);
	
	if (rc < 0 || rc != 2)
		return (rc < 0) ? rc : -EIO;

	size = get_unaligned_le16(&buf[0]);
	if (!size || size == 2)
		return 0;

	if (size > max)
		return -EINVAL;

	client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	rc = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, size);
	memcpy(buf, I2CDMABuf_va, size);
	client->ext_flag = client->ext_flag
		& (~I2C_DMA_FLAG) & (~I2C_ENEXT_FLAG);

	return (rc < 0) ? rc : rc != (int)size ? -EIO : 0;
}

static int cyttsp5_i2c_write_read_specific(struct device *dev, u8 write_len,
		u8 *write_buf, u8 *read_buf,
		u8 *I2CDMABuf_va, dma_addr_t I2CDMABuf_pa)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len)
		return -EINVAL;
	//msgs[0].addr = client->addr;
	msgs[0].addr = 0x24;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = (u8 *)I2CDMABuf_pa;
	msgs[0].ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	msgs[0].timing = MELFAS_I2C_MASTER_CLOCK;
	memcpy(I2CDMABuf_va, write_buf, write_len);

	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	rc = 0;

	if (read_buf)
		rc = cyttsp5_i2c_read_default_nosize(dev, read_buf,
				CY_I2C_DATA_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
	return rc;
}

#else
static int cyttsp5_i2c_read_default(struct device *dev, void *buf, int size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	if (!buf || !size || size > CY_I2C_DATA_SIZE)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

static int cyttsp5_i2c_read_default_nosize(struct device *dev, u8 *buf, u32 max)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;

	if (!buf)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	size = get_unaligned_le16(&buf[0]);
	if (!size || size == 2 || size >= CY_PIP_1P7_EMPTY_BUF)
		/* Before PIP 1.7, empty buffer is 0x0002;
		From PIP 1.7, empty buffer is 0xFFXX */
		return 0;

	if (size > max)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);

	return (rc < 0) ? rc : rc != (int)size ? -EIO : 0;
}

static int cyttsp5_i2c_write_read_specific(struct device *dev, u8 write_len,
		u8 *write_buf, u8 *read_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = write_buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	rc = 0;

	if (read_buf)
		rc = cyttsp5_i2c_read_default_nosize(dev, read_buf,
				CY_I2C_DATA_SIZE);

	return rc;
}
#endif

static struct cyttsp5_bus_ops cyttsp5_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read_default = cyttsp5_i2c_read_default,
	.read_default_nosize = cyttsp5_i2c_read_default_nosize,
	.write_read_specific = cyttsp5_i2c_write_read_specific,
};

#ifdef CONFIG_TOUCHSCREEN_MTK //htc
static const struct of_device_id mtk_dt_match_table[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
#endif //htc

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
static const struct of_device_id cyttsp5_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp5_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_i2c_of_match);
#endif

static int cyttsp5_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;

//#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	const struct of_device_id *match;
//#endif
	int rc;

//htc+
	htc_bootmode = htc_get_bootmode();
	dev_info(dev, "%s: htc_bootmode = %s", __func__, htc_bootmode);
	if( (strcmp(htc_bootmode, "offmode_charging") == 0) ||
		(strcmp(htc_bootmode, "charger") == 0) ||
		(strcmp(htc_bootmode, "recovery") == 0) ||
		(strcmp(htc_bootmode, "MFG_MODE_OFFMODE_CHARGING") == 0) ||
		(strcmp(htc_bootmode, "MFG_MODE_POWER_TEST") == 0) ||
		(strcmp(htc_bootmode, "MFG_MODE_RECOVERY") == 0)) {
		dev_info(dev, "%s: skip to cyttsp5_i2c_probe()", __func__);
		return 0;
	}

	/*if(htc_get_lcm_id() == NULL_PANEL) { FIXME
		pr_err("%s: null panel detected, skip to cyttsp5_i2c_probe()", __func__);
		return 0;
	}*/
//htc-

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		htc_dev_err(dev, "%s: I2C functionality not Supported\n", __func__);
		return -EIO;
	}

//#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
//htc+
#ifdef CONFIG_TOUCHSCREEN_MTK
	match = of_match_device(of_match_ptr(mtk_dt_match_table), dev);
	if (!match) {
		htc_dev_err(dev, "%s: No device match found\n", __func__);
	}
#else
	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match) {
		rc = cyttsp5_devtree_create_and_get_pdata(dev);
		if (rc < 0)
			return rc;
	}
#endif//#ifdef CONFIG_TOUCHSCREEN_MTK

	if(dev) {
		struct cyttsp5_platform_data *platform_data;
		struct cyttsp5_core_platform_data *core_pdata;

		platform_data = dev->platform_data;
		if(platform_data) {
			core_pdata = platform_data->core_pdata;
			if(core_pdata) {
				core_pdata->pinctrl = devm_pinctrl_get(dev);
				if (IS_ERR(core_pdata->pinctrl)) {
					core_pdata->pinctrl = (struct pinctrl *)NULL;
					rc = -EIO;
					pr_err("%s: no pinctrl", __func__);
				}
				core_pdata->pinsctrl_TS_ACTIVE = pinctrl_lookup_state(core_pdata->pinctrl, "TS_ACTIVE");
				if (IS_ERR(core_pdata->pinsctrl_TS_ACTIVE)) {
					core_pdata->pinsctrl_TS_ACTIVE = (struct pinctrl_state *)NULL;
					rc = -EIO;
					pr_err("%s: no pinsctrl_TS_ACTIVE", __func__);
				}
				core_pdata->htc_tamper_sf = get_tamper_sf();
				pr_info("%s: htc_tamper_sf = %d", __func__, core_pdata->htc_tamper_sf);

				//core_pdata->htc_lcm_id = htc_get_lcm_id(); FIXME
				//pr_info("%s: htc_lcm_id = %d ", __func__, core_pdata->htc_lcm_id);

			}
		}
	}
//htc-

#ifdef CONFIG_TOUCHSCREEN_MTK //htc
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	rc = regulator_set_voltage(tpd->reg, 2800000, 2800000);	//set 2.8v
	if (rc) {
		printk(KERN_ERR"[cyttsp5] regulator_set_voltage(%d) failed!\n", rc);
		return -1;
	}
	rc = regulator_enable(tpd->reg);
	printk(KERN_ERR"[cyttsp5] %s, set voltage 2.8V\n",__func__);
#endif //htc

//#endif
#ifdef USE_I2C_DMA
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
#endif

#ifdef CONFIG_TOUCHSCREEN_MTK //htc
	dev->platform_data = &_cyttsp5_platform_data;
#endif //htc
	
	rc = cyttsp5_probe(&cyttsp5_i2c_bus_ops, &client->dev, client->irq,
			  CY_I2C_DATA_SIZE);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	if (rc && match)
		cyttsp5_devtree_clean_pdata(dev);
#endif

	return rc;
}

static int cyttsp5_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	struct device *dev = &client->dev;
	const struct of_device_id *match;
#endif
	struct cyttsp5_core_data *cd = i2c_get_clientdata(client);

	cyttsp5_release(cd);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match)
		cyttsp5_devtree_clean_pdata(dev);
#endif

	return 0;
}

static const struct i2c_device_id cyttsp5_i2c_id[] = {
	{ CYTTSP5_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp5_i2c_id);


#ifdef CONFIG_TOUCHSCREEN_MTK //htc
//htc: not this way

static int cyttsp5_tpd_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);    
    return 0;
}

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
		.name = CYTTSP5_I2C_NAME,
		//.owner = THIS_MODULE,
		//.pm = &cyttsp5_pm_ops,
		.of_match_table = mtk_dt_match_table,
	},
	.probe = cyttsp5_i2c_probe,
	.remove = cyttsp5_i2c_remove,
	.id_table = cyttsp5_i2c_id,
	.detect = cyttsp5_tpd_detect,
};

#if 0 //(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
module_i2c_driver(cyttsp5_i2c_driver);
#else
//module_init(cyttsp5_i2c_init);

static int tpd_local_init(void)
{
	printk("%s enter\n", __func__);

	if(i2c_add_driver(&cyttsp5_i2c_driver)!=0)
	{
		TPD_DMESG("%s: Error: unable to add i2c driver.\n", __func__);
		return -ENODEV;
	}

	if (tpd_load_status == 0) {
		printk("%s: add error touch panel driver.\n", __func__);
		i2c_del_driver(&cyttsp5_i2c_driver);
		return -1;
	}

	return 0;
}

extern int cyttsp5_core_suspend(struct device *dev);
extern int cyttsp5_core_resume(struct device *dev);

/* Function to manage low power suspend */
static void tpd_suspend(struct device *h)
{
    cyttsp5_core_suspend(h);
}

/* Function to manage power-on resume */
static void tpd_resume(struct device *h)
{
   cyttsp5_core_resume(h);
}



static struct tpd_driver_t ttda_tpd_driver = {
	.tpd_device_name = CYTTSP5_I2C_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,	/* tpd_suspend, */
	.resume = tpd_resume,		/* tpd_resume,  */
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init cyttsp5_tpd_init(void)
{
	tpd_get_dts_info();
	
	if(tpd_driver_add(&ttda_tpd_driver) < 0){
		pr_err("Fail to add tpd driver\n");
		return -ENODEV;
	}

	return 0;
}
module_init(cyttsp5_tpd_init);


/*static void __exit cyttsp5_i2c_exit(void)
{
	i2c_del_driver(&cyttsp5_i2c_driver);
}
module_exit(cyttsp5_i2c_exit);*/

static void __exit cyttsp5_tpd_exit(void)
{
	TPD_DMESG("MediaTek TTDA exit\n");
	/* input_unregister_device(tpd->dev); */
	tpd_driver_remove(&ttda_tpd_driver);
}

module_exit(cyttsp5_tpd_exit);

#endif

#else //#ifdef CONFIG_TOUCHSCREEN_MTK //htc

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
	.name = CYTTSP5_I2C_NAME,
	.owner = THIS_MODULE,
	.pm = &cyttsp5_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	.of_match_table = cyttsp5_i2c_of_match,
#endif
	},
	.probe = cyttsp5_i2c_probe,
	.remove = cyttsp5_i2c_remove,
	.id_table = cyttsp5_i2c_id,
};

static void __init cyttsp5_i2c_init_async(void *unused, async_cookie_t cookie)
{
	pr_info("%s:Enter \n", __func__);
	if (i2c_add_driver(&cyttsp5_i2c_driver) != 0)
		pr_info("unable to add touch i2c driver.\n");

	pr_info("End %s, %d\n", __func__, __LINE__);
}
static int __init cyttsp5_i2c_init(void)
{
	pr_info("%s: Parade TTSP I2C Driver (Built %s) \n", __func__, CY_DRIVER_VERSION);
	async_schedule(cyttsp5_i2c_init_async, NULL);
	return 0;
}
module_init(cyttsp5_i2c_init);

static void __exit cyttsp5_i2c_exit(void)
{
	i2c_del_driver(&cyttsp5_i2c_driver);
}
module_exit(cyttsp5_i2c_exit);
#endif //#ifdef CONFIG_TOUCHSCREEN_MTK //htc

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
