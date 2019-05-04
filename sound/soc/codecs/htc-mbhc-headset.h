/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/types.h>


#define ANCLeft_Headset_Mic_Mask 1
#define ANCRight_Headset_Mic_Mask 2
#define ANC_Headset_Detection_Mask 4

enum htc_id_type {
	ID_TYPE_BEGIN = 0,
	TYPEC_ID1 = ID_TYPE_BEGIN,
	TYPEC_ID2,
	TYPEC_POSITION,
	ID_TYPE_MAX
};

enum htc_switch_type {
	SWITCH_TYPE_BEGIN = 0,
	AUD_POWER_EN = SWITCH_TYPE_BEGIN,
	FSA3030_SEL0,
	FSA3030_SEL1,
	FSA3030_SEL2,
	HEADSET_S3_0,
	HEADSET_S3_1,
	HEADSET_S4,
	HEADSET_S5,
	HSMIC_BIAS_EN,
	SWITCH_TYPE_MAX
};

enum htc_adc_range_type {
	ADC_RANGE_TYPE_BEGIN = 0,
	ADC_35mm_RANGE = ADC_RANGE_TYPE_BEGIN,
	ADC_25mm_RANGE,
	ADC_RANGE_TYPE_MAX
};

enum htc_accessory_type {
	ACCESSORY_UNKNOWN,
	ACCESSORY_HEADPHONE,
	ACCESSORY_ADAPTER
};

enum htc_accessory_position {
	POSITION_NEGATIVE,
	POSITION_POSITIVE
};

enum htc_s3_mode_type {
	S3_MODE_NONE = -1,
	S3_MODE_P,
	S3_MODE_N
};

enum htc_s4_mode_type {
	S4_MODE_NONE = -1,
	S4_MODE_P,
	S4_MODE_N
};


irqreturn_t htc_typec_hs_plug_detect_irq(struct wcd_mbhc *mbhc);
void htc_typec_hs_set_ext_micbias(bool enable, short mask);
int htc_typec_hs_get_headset_type(void);

int htc_typec_hs_dapm_new_controls(struct snd_soc_dapm_context *dapm);

int htc_typec_hs_set_mbhc(struct wcd_mbhc *mbhc);
void htc_typec_hs_remove_mbhc(struct wcd_mbhc *mbhc);
int htc_typec_hs_init(struct platform_device *pdev);
void htc_typec_hs_deinit(void);
