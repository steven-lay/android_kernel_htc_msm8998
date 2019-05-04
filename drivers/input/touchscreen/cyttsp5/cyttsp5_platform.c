/*
 * cyttsp5_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2013-2015 Cypress Semiconductor
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
#include "cyttsp5_platform.h"

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
#if 0 //htc: request by vendor to update firmware
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};
#endif

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#if 0 //htc: request by vendor to update firmware
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
//htc++: request by vendor to enable this
#if 1 //vendor original codes
	.flags = CY_LOADER_FLAG_NONE,
#else
	.flags = CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE,
#endif
//htc--
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{

#ifdef CONFIG_YUDE_MTK
	tpd_gpio_output(0,1);// reset pin high

	msleep(DELAY_BOOT_READY);

	tpd_gpio_output(0,0);// reset pin low
	msleep(DELAY_RESET_LOW);


	tpd_gpio_output(0,1);// reset pin high
	msleep(DELAY_UI_READY);

	printk(KERN_ERR"%s [cyttsp5] Doing Reset..\n",__func__);
	return 0;
#else
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;

	if(rst_gpio > 0) {
		gpio_set_value(rst_gpio, 0);
		msleep(10);
		gpio_set_value(rst_gpio, 1);
		msleep(10);
		dev_info(dev,
			"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
			pdata->rst_gpio, rc);
	}

	return rc;

#endif
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
#ifdef CONFIG_YUDE_MTK
		tpd_gpio_as_int(1);// eint 1
		tpd_gpio_output(0,1);// reset pin high
		printk(KERN_ERR"%s:tpd_gpio_as_int\n",__func__);
		return 0;
#else
//htc+
	int TP_1V8_EN_gpio = pdata->TP_1V8_EN_gpio;
	int TP_3V3_EN_gpio = pdata->TP_3V3_EN_gpio;
	int TP_I2C_SEL_CPU_gpio = pdata->TP_I2C_SEL_CPU_gpio;
//htc-
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {
//htc+
		if(TP_3V3_EN_gpio > 0) {
			rc = gpio_request(TP_3V3_EN_gpio, "TP_3V3_EN");
			if(rc < 0) {
				gpio_free(TP_3V3_EN_gpio);
				rc = gpio_request(TP_3V3_EN_gpio, "TP_3V3_EN");
			}
			if(rc < 0){
				htc_dev_err(dev,
					"%s: Fail request TP_3V3_EN_gpio=%d\n",
					__func__, TP_3V3_EN_gpio);
					gpio_free(TP_3V3_EN_gpio);
			}
			else {
				gpio_direction_output(TP_3V3_EN_gpio, 1);
				printk(KERN_INFO"[TP] %s: gpio_direction_output(TP_3V3_EN_gpio, 1)\n",__func__);
			}
		}
		if(TP_1V8_EN_gpio > 0) {
			rc = gpio_request(TP_1V8_EN_gpio, "TP_1V8_EN");
			if(rc < 0) {
				gpio_free(TP_1V8_EN_gpio);
				rc = gpio_request(TP_1V8_EN_gpio, "TP_1V8_EN");
			}
			if(rc < 0){
				htc_dev_err(dev,
					"%s: Fail request TP_1V8_EN_gpio=%d\n",
					__func__, TP_1V8_EN_gpio);
					gpio_free(TP_1V8_EN_gpio);
			}
			else {
				gpio_direction_output(TP_1V8_EN_gpio, 1);
				pr_info(" %s: gpio_direction_output(TP_1V8_EN_gpio, 1)\n",__func__);
			}
		}

		if(TP_I2C_SEL_CPU_gpio > 0) {
			rc = gpio_request(TP_I2C_SEL_CPU_gpio, "TP_I2C_SEL_CPU");
			if(rc < 0) {
				gpio_free(TP_I2C_SEL_CPU_gpio);
				rc = gpio_request(TP_I2C_SEL_CPU_gpio, "TP_I2C_SEL_CPU");
			}
			if(rc < 0) {
				htc_dev_err(dev,
					"%s: Fail request TP_I2C_SEL_CPU_gpio=%d\n",
					__func__, TP_I2C_SEL_CPU_gpio);
				gpio_free(TP_I2C_SEL_CPU_gpio);
				return rc;
			}
			else {
				gpio_direction_output(TP_I2C_SEL_CPU_gpio, 0);
				pr_info("%s: gpio_direction_output(TP_I2C_SEL_CPU_gpio, 0)\n",__func__);
			}
		}

		if(irq_gpio > 0) {
			rc = gpio_request(irq_gpio, "cyttsp_irq_gpio");
			if (rc < 0) {
				gpio_free(irq_gpio);
				rc = gpio_request(irq_gpio, "cyttsp_irq_gpio");
			}
			if (rc < 0) {
				htc_dev_err(dev,
					"%s: Fail request gpio=%d\n",
					__func__, irq_gpio);
				gpio_free(irq_gpio);
				return rc;
			} else {
				gpio_direction_input(irq_gpio);
				if( pdata->pinctrl != NULL &&
					pdata->pinsctrl_TS_ACTIVE != NULL) {
					pinctrl_select_state(pdata->pinctrl, pdata->pinsctrl_TS_ACTIVE);
				}
			}
		}

		if(rst_gpio > 0) {
			rc = gpio_request(rst_gpio, "cyttsp_rst_gpio");
			if (rc < 0) {
				gpio_free(rst_gpio);
				rc = gpio_request(rst_gpio, "cyttsp_rst_gpio");
			}
			if (rc < 0) {
				htc_dev_err(dev,
					"%s: Fail request gpio=%d\n", __func__,	rst_gpio);
				gpio_free(rst_gpio);
				return rc;
			} else {
				rc = gpio_direction_output(rst_gpio, 1);
				pr_info(" %s: gpio_direction_output(rst_gpio, 1)\n",__func__);
				cyttsp5_xres(pdata, dev);
			}
		}
	} else {
		if(TP_3V3_EN_gpio > 0) {
			gpio_free(TP_3V3_EN_gpio);
			pr_info("%s: gpio free TP_3V3_EN_gpio\n",__func__);
		}
		if(TP_1V8_EN_gpio > 0) {
			gpio_free(TP_1V8_EN_gpio);
			pr_info("%s: gpio free TP_1V8_EN_gpio\n",__func__);
		}
		if(TP_I2C_SEL_CPU_gpio > 0) {
			gpio_free(TP_I2C_SEL_CPU_gpio);
			pr_info("%s: gpio free TP_I2C_SEL_CPU_gpio\n",__func__);
		}
		if(rst_gpio > 0) {
			gpio_free(rst_gpio);
			pr_info("%s: gpio free rst_gpio\n",__func__);
		}
		if(irq_gpio > 0) {
			gpio_free(irq_gpio);
			pr_info("%s: gpio free irq_gpio\n",__func__);
		}
	}

	dev_info(dev, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d rc=%d TP_3V3_EN_gpio=%d TP_1V8_EN_gpio=%d TP_I2C_SEL_CPU_gpio=%d\n",
		__func__, rst_gpio, irq_gpio, rc, TP_3V3_EN_gpio, TP_1V8_EN_gpio, TP_I2C_SEL_CPU_gpio);

//htc-
	return rc;
#endif

}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		dev_vdbg(dev, "%s: Performing a reset\n", __func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		dev_vdbg(dev, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);
	}

	return rc;
}
#endif



