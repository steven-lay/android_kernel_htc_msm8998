/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef HTC_UTIL_H
#define HTC_UTIL_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_dsi.h"

#define UNDEF_VALUE -1U

/* bl_cali will receive parameter from 1 to 20000
 * this mean can support scale rate from 0.0001 to 2
 */
#define CALIBRATION_DATA_PATH "/calibration_data"
#define DISP_FLASH_DATA "disp_flash"
#define DISP_FLASH_DATA_SIZE 64
#define LIGHT_CALI_OFFSET 36
#define LIGHT_CALI_SIZE 8
#define LIGHT_RATIO_INDEX 0
#define LIGHT_R_INDEX 1
#define LIGHT_G_INDEX 2
#define LIGHT_B_INDEX 3
#define RGB_CALI_DEF 255
#define RGB_GAIN_CHECK(x) (x>0 && x<256)
#define RGB_CALIBRATION(ori,comp) ((long)(ori*comp/RGB_CALI_DEF))
#define BL_CALI_DEF  10000
#define BL_CALI_MAX  20000
#define BRI_GAIN_CHECK(x) (x>0 && x<=20000)
#define BACKLIGHT_CALI(ori,comp) ((unsigned int)(ori*comp/BL_CALI_DEF))
#define VALID_CALI_BKLT(val,min,max) ((min) > (val) ? (min) : ((val) > (max) ? (max) : (val)))

enum {
	CABC_INDEX = 0,
	BURST_SWITCH_INDEX = 1,
	COLOR_TEMP_INDEX = 2,
	BL_CALI_ENABLE_INDEX = 3,
	RGB_CALI_ENABLE_INDEX = 4,
	COLOR_PROFILE_INDEX = 5,
};

enum {
	DEFAULT_MODE = 0,	/* COLOR_MODE_DEFAULT */
	SRGB_MODE = 7,		/* COLOR_MODE_SRGB */
};

struct attribute_status {
       char *title;
       u32 req_value;
       u32 cur_value;
       u32 def_value;
};

void htc_register_camera_bkl(int level);
void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd);
void htc_set_cabc(struct msm_fb_data_type *mfd, bool force);
void htc_set_color_temp(struct msm_fb_data_type *mfd, bool force);
void htc_set_color_profile(struct mdss_panel_data *pdata, bool force);
bool htc_is_burst_bl_on(struct msm_fb_data_type *mfd, int value);
void htc_reset_status(void);
void htc_debugfs_init(struct msm_fb_data_type *mfd);
void htc_debugfs_deinit(struct msm_fb_data_type *mfd);
void htc_panel_info(const char *panel);
void htc_update_bl_cali_data(struct msm_fb_data_type *mfd);
void send_dsi_status_notify(int status);
void htc_vreg_vol_switch(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool enable);
void htc_vreg_init(struct platform_device *ctrl_pdev, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void htc_mdss_dsi_parse_esd_params(struct device_node *np);
void htc_hal_color_feature_enabled(bool enable);
void htc_ddic_color_mode_supported(bool enable);
void htc_dimming_on(struct msm_fb_data_type *mfd);
void htc_dimming_off(void);
#endif /* MDSS_FB_H */
