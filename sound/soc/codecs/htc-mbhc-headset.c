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

#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "wcd-mbhc-v2.h"
#include "htc-mbhc-headset.h"


/* check the defination WCD_MBHC_JACK_MASK on wcd-mbhc-v2.c */
#define WCD_MBHC_JACK_MASK (SND_JACK_HEADSET | SND_JACK_OC_HPHL | \
			   SND_JACK_OC_HPHR | SND_JACK_LINEOUT | \
			   SND_JACK_MECHANICAL | SND_JACK_MICROPHONE2 | \
			   SND_JACK_UNSUPPORTED)


#define HTC_AS_HEADSET_DETECT_RETRY   30
#define HTC_AS_HEADSET_DEBOUNCE_TIME  50000


struct htc_hs_gpio {
	const char *name;
	int no;
	unsigned long initial_value;
};

struct htc_adc_range {
	const char *name;
	unsigned int max;
	unsigned int min;
};

struct htc_adc_device {
	const char *name;
	unsigned int channel;
	struct htc_adc_range range[ADC_RANGE_TYPE_MAX];
};

struct htc_headset_dev {
	bool htc_headset_init;
	struct htc_hs_gpio id_gpio[ID_TYPE_MAX];
	struct htc_hs_gpio switch_gpio[SWITCH_TYPE_MAX];
	struct htc_adc_device hs_adc;
	/* report unsupport if it's non HTC analog adapter */
	struct switch_dev unsupported_type;
	/* Add attribute on sysfs for debugging */
	struct class *htc_accessory_class;
	struct device *headset_dev;
	struct device *debug_dev;
	u16 debug_reg[50];
	int debug_reg_count;
};

static struct htc_headset_dev htc_headset = {
	.id_gpio[TYPEC_ID1].name = "htc,aud_typec_id1",
	.id_gpio[TYPEC_ID2].name = "htc,aud_typec_id2",
	.id_gpio[TYPEC_POSITION].name = "htc,aud_usb_position",
	.switch_gpio[AUD_POWER_EN].name = "htc,aud_3v3_en",
	.switch_gpio[AUD_POWER_EN].initial_value = GPIOF_OUT_INIT_HIGH,
	.switch_gpio[FSA3030_SEL0].name = "htc,usb_hph_fsa3030_sel0",
	.switch_gpio[FSA3030_SEL1].name = "htc,usb_hph_fsa3030_sel1",
	.switch_gpio[FSA3030_SEL1].initial_value = GPIOF_OUT_INIT_HIGH,
	.switch_gpio[FSA3030_SEL2].name = "htc,usb_hph_fsa3030_sel2",
	.switch_gpio[HEADSET_S3_0].name = "htc,hpmic_agnd_flip_en_s0",
	.switch_gpio[HEADSET_S3_1].name = "htc,hpmic_agnd_flip_en_s1",
	.switch_gpio[HEADSET_S3_1].initial_value = GPIOF_OUT_INIT_HIGH,
	.switch_gpio[HEADSET_S4].name = "htc,miclr_flip_en",
	.switch_gpio[HEADSET_S5].name = "htc,miclr_dgnd_sw_en",
	.switch_gpio[HSMIC_BIAS_EN].name = "htc,hsmic_2v85_en",
	.hs_adc.name = "htc,headset_adc_channel",
	.hs_adc.range[ADC_35mm_RANGE].name = "htc,adapter_35mm_threshold",
	.hs_adc.range[ADC_25mm_RANGE].name = "htc,adapter_25mm_threshold",
	.htc_headset_init = false,
	.htc_accessory_class = NULL,
	.unsupported_type.name = NULL,
};

/* OCN ONLY + */
static struct htc_hs_gpio aud_remote_sensor_gpio = {
	.name = "htc,aud_remote_sensor"
};
/* OCN ONLY - */

static struct platform_device *sound_device;
static struct wcd_mbhc* __MBHC;
static struct mutex ext_micbias_mutex;

extern void __wcd_mbhc_update_fsm_source(struct wcd_mbhc *mbhc,
				enum wcd_mbhc_plug_type plug_type);
extern void __wcd_enable_mbhc_supply(struct wcd_mbhc *mbhc,
				enum wcd_mbhc_plug_type plug_type);
extern int __wcd_cancel_btn_work(struct wcd_mbhc *mbhc);
extern void __wcd_mbhc_jack_report(struct wcd_mbhc *mbhc, int status, int mask);
extern void __wcd_mbhc_clr_and_turnon_hph_padac(struct wcd_mbhc *mbhc);
extern void __wcd_mbhc_set_and_turnoff_hph_padac(struct wcd_mbhc *mbhc);
extern void __wcd_mbhc_swch_irq_handler(struct wcd_mbhc *mbhc);

/* OCN ONLY + */
static unsigned int pre_condition;
extern int qpnp_pin_pull_config(int gpio, int value);
/* OCN ONLY - */


static int htc_typec_hs_dt_parser(struct platform_device *pdev)
{
	int i = 0;
	int ret = 0;
	uint32_t min_max_array[2]; //[0] : min [1] :max
	const char *name;

	for (i = ID_TYPE_BEGIN; i < ID_TYPE_MAX; i++) {
		name = htc_headset.id_gpio[i].name;
		if (name == NULL)
			continue;
		htc_headset.id_gpio[i].no = of_get_named_gpio(pdev->dev.of_node, name, 0);
		if (gpio_is_valid(htc_headset.id_gpio[i].no)) {
			gpio_free(htc_headset.id_gpio[i].no);
			ret = gpio_request_one(htc_headset.id_gpio[i].no, GPIOF_DIR_IN, name);
			if (ret) {
				pr_err("%s: gpio %s request failed (%d)\n", __func__, name, ret);
				return ret;
			}
		} else {
			pr_err("%s: gpio %s parse fail\n", __func__, name);
			return -EINVAL;
		}
	}

	for (i = SWITCH_TYPE_BEGIN; i < SWITCH_TYPE_MAX; i++) {
		name = htc_headset.switch_gpio[i].name;
		if (name == NULL)
			continue;
		htc_headset.switch_gpio[i].no = of_get_named_gpio(pdev->dev.of_node, name, 0);
		if (gpio_is_valid(htc_headset.switch_gpio[i].no)) {
			gpio_free(htc_headset.switch_gpio[i].no);
			if (htc_headset.switch_gpio[i].initial_value == GPIOF_OUT_INIT_HIGH)
				ret = gpio_request_one(htc_headset.switch_gpio[i].no,
							GPIOF_OUT_INIT_HIGH, name);
			else
				ret = gpio_request_one(htc_headset.switch_gpio[i].no,
							GPIOF_OUT_INIT_LOW, name);
			if (ret) {
				pr_err("%s: gpio %s request failed (%d)\n", __func__, name, ret);
				return ret;
			}
		} else {
			pr_info("%s: gpio %s parse fail\n", __func__, name);
		}

	}

	ret = of_property_read_u32(pdev->dev.of_node,
				htc_headset.hs_adc.name, &htc_headset.hs_adc.channel);
	if (ret < 0) {
		pr_err("%s: adc_channel parser err\n", __func__);
	} else {
		for (i = ADC_RANGE_TYPE_BEGIN; i < ADC_RANGE_TYPE_MAX; i++) {
			name = htc_headset.hs_adc.range[i].name;
			if (name == NULL)
				continue;
			ret = of_property_read_u32_array(pdev->dev.of_node, name, min_max_array, 2);
			if (ret < 0) {
				pr_err("%s: adc_range %s parser err\n", __func__, name);
			} else {
				htc_headset.hs_adc.range[i].min = min_max_array[0];
				htc_headset.hs_adc.range[i].max = min_max_array[1];
			}
		}
	}

/* OCN ONLY + */
	aud_remote_sensor_gpio.no = of_get_named_gpio(pdev->dev.of_node,
									aud_remote_sensor_gpio.name, 0);
	if (gpio_is_valid(aud_remote_sensor_gpio.no)) {
		ret = of_property_read_u32(pdev->dev.of_node,
					"htc,typec-hs-pre-condition", &pre_condition);
		if (ret < 0)
			pre_condition = 0;
	}
/* OCN ONLY - */

	return 0;
}

static int headset_pin_init(struct pinctrl *pinctrl)
{
	struct pinctrl_state *set_state;
	int retval;

	set_state = pinctrl_lookup_state(pinctrl, "headset_pin_default");
	if (IS_ERR(set_state)) {
		pr_err("cannot get headset_gpio_init pinctrl state\n");
		return PTR_ERR(set_state);
	}

	retval = pinctrl_select_state(pinctrl, set_state);
	if (retval) {
		pr_err("cannot set headset_gpio_init pinctrl gpio state\n");
		return retval;
	}

	return 0;
}

static int htc_typec_hs_id_gpio_get(enum htc_id_type index)
{
	return gpio_get_value(htc_headset.id_gpio[index].no);
}

static void htc_typec_hs_switch_gpio_set(enum htc_switch_type index, unsigned int value)
{
	gpio_set_value(htc_headset.switch_gpio[index].no, value);
	return;
}

static int htc_typec_hs_switch_gpio_get(enum htc_id_type index)
{
	return gpio_get_value(htc_headset.switch_gpio[index].no);
}

#if 0 /* disable 3.5mm adapter */
static int htc_hs_qpnp_remote_adc(int *adc)
{
	int rc = 0;
	enum qpnp_vadc_channels chan;
	struct qpnp_vadc_result result;
	static struct qpnp_vadc_chip *vadc_chip = NULL;

	if (vadc_chip == NULL)
		vadc_chip = qpnp_get_vadc(&sound_device->dev, "headset");

	result.physical = -EINVAL;
	chan = htc_headset.hs_adc.channel;

	rc = qpnp_vadc_read(vadc_chip, chan, &result);
	if (rc < 0) {
		pr_err("%s: Read ADC fail, rc = %d\n", __func__, rc);
		return rc;
	}

	*adc = (int)result.physical;
	*adc /= 1000; /* uA to mA */
	pr_info("%s: Remote ADC %d (%#X)\n", __func__, *adc, *adc);

	return rc;
}
#endif

static void htc_typec_hs_set_s3_pos(enum htc_s3_mode_type s3_mode)
{
	switch (s3_mode) {
	case S3_MODE_NONE:
		htc_typec_hs_switch_gpio_set(HEADSET_S3_0, 0);
		htc_typec_hs_switch_gpio_set(HEADSET_S3_1, 1);
		break;
	case S3_MODE_P:
		htc_typec_hs_switch_gpio_set(HEADSET_S3_0, 0);
		htc_typec_hs_switch_gpio_set(HEADSET_S3_1, 0);
		break;
	case S3_MODE_N:
		htc_typec_hs_switch_gpio_set(HEADSET_S3_0, 1);
		htc_typec_hs_switch_gpio_set(HEADSET_S3_1, 1);
		break;
	default:
		break;
	}
}

static void htc_typec_hs_set_s4_pos(enum htc_s4_mode_type s4_mode)
{
	switch (s4_mode) {
	case S4_MODE_P:
		htc_typec_hs_switch_gpio_set(HEADSET_S4, 1);
		break;
	case S4_MODE_NONE:
	case S4_MODE_N:
		htc_typec_hs_switch_gpio_set(HEADSET_S4, 0);
		break;
	default:
		break;
	}
}

static void htc_typec_hs_set_s3s4_position(enum htc_accessory_position position)
{
	if (position == POSITION_POSITIVE) {
		htc_typec_hs_set_s3_pos(S3_MODE_P);
		htc_typec_hs_set_s4_pos(S4_MODE_P);
	} else if (position == POSITION_NEGATIVE) {
		htc_typec_hs_set_s3_pos(S3_MODE_N);
		htc_typec_hs_set_s4_pos(S4_MODE_N);
	}
}

static void htc_typec_hs_set_s5_en(bool enable)
{
	if (enable)
		htc_typec_hs_switch_gpio_set(HEADSET_S5, 1);
	else
		htc_typec_hs_switch_gpio_set(HEADSET_S5, 0);
}

void htc_typec_hs_set_ext_micbias(bool enable, short mask)
{
	static int ext_micbias_count = 0;

	mutex_lock(&ext_micbias_mutex);
	pr_info("%s: enable %d mask %#x\n", __func__, enable, mask);
	if (enable) {
		ext_micbias_count |= mask;
		if (ext_micbias_count != 0) {
			htc_typec_hs_switch_gpio_set(HSMIC_BIAS_EN, 1);
			pr_info("%s: ext_micbias on\n", __func__);
		}
	} else {
		ext_micbias_count &= ~mask;
		if (ext_micbias_count == 0) {
			htc_typec_hs_switch_gpio_set(HSMIC_BIAS_EN, 0);
			pr_info("%s: ext_micbias off\n",__func__);
		}
	}
	mutex_unlock(&ext_micbias_mutex);
}

static void htc_typec_hs_set_ext_micbias_force_en(bool enable)
{
	if (enable)
		htc_typec_hs_switch_gpio_set(HSMIC_BIAS_EN, 1);
	else
		htc_typec_hs_switch_gpio_set(HSMIC_BIAS_EN, 0);
}

static void htc_typec_hs_set_fsa3030_accessory(bool en)
{
	pr_info("%s: rara mode %d\n", __func__, en);

	if (en) {
		/* accessory mode */
		htc_typec_hs_switch_gpio_set(FSA3030_SEL0, 1);
		htc_typec_hs_switch_gpio_set(FSA3030_SEL1, 0);
	} else{
		/* USB mode */
		htc_typec_hs_switch_gpio_set(FSA3030_SEL0, 0);
		htc_typec_hs_switch_gpio_set(FSA3030_SEL1, 1);
	}
}

static bool is_typec_headphone_position(void)
{
	if (htc_typec_hs_id_gpio_get(TYPEC_POSITION) == 1)
		return true;

	return false;
}

static bool is_typec_adapter_positive(void)
{
	if (htc_typec_hs_id_gpio_get(TYPEC_ID1) == 1)
		return true;

	return false;
}

static enum htc_accessory_type htc_typec_hs_get_accessory_type(void)
{
	if (htc_typec_hs_id_gpio_get(TYPEC_ID1) & htc_typec_hs_id_gpio_get(TYPEC_ID2))
		return ACCESSORY_HEADPHONE;
	else if (htc_typec_hs_id_gpio_get(TYPEC_ID1) ^ htc_typec_hs_id_gpio_get(TYPEC_ID2))
		return ACCESSORY_ADAPTER;
	else
		return ACCESSORY_UNKNOWN;
}

static ssize_t htc_typec_hs_headset_print_name(struct switch_dev *sdev, char *buf)
{
	/* report unsupport if it's not HTC analog adapter */
	return snprintf(buf, PAGE_SIZE, "Unsupported_device\n");
}

static void htc_typec_hs_enable_mic_bias(struct wcd_mbhc *mbhc, bool enable)
{
	int rc = -1;

	pr_info("%s: enable %d\n", __func__, enable);

	if (mbhc->mbhc_cb->mbhc_micbias_control) {
		mbhc->mbhc_cb->mbhc_micbias_control(mbhc->codec,
						MIC_BIAS_2,
						(enable)? MICB_ENABLE : MICB_DISABLE);
		if (mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic) {
			rc = mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic(mbhc->codec,
							MIC_BIAS_2, enable);
			if (rc) {
				pr_err("%s: Micbias control for thr mic failed, rc: %d\n",
					__func__, rc);
			}
		} else {
			pr_err("%s: no mbhc_micb_ctrl_thr_mic\n", __func__);
		}
	} else {
		pr_err("%s: no mbhc_micbias_control\n", __func__);
	}
}

static void htc_typec_hs_button_detection(struct wcd_mbhc *mbhc, bool enable)
{
	pr_info("%s: enable %d\n", __func__, enable);
	if(enable) {
		/* Enable HW FSM */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 1);
	} else {
		/* Disable HW FSM */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 0);
	}

	if (mbhc->mbhc_cb->mbhc_micbias_control) {
		__wcd_mbhc_update_fsm_source(mbhc, mbhc->current_plug);
	} else {
		__wcd_enable_mbhc_supply(mbhc, mbhc->current_plug);
	}
}

static int htc_typec_hs_irq_handler(struct wcd_mbhc *mbhc, int insert)
{
	int retry = 0, ret = 0;

	WCD_MBHC_RSC_LOCK(mbhc);

	/* cancel pending button press */
	if (__wcd_cancel_btn_work(mbhc))
		pr_info("%s: button press is canceled\n", __func__);

	pr_info("%s: enter, current_plug=%d\n", __func__, mbhc->current_plug);

	if (mbhc->current_plug == MBHC_PLUG_TYPE_NONE && insert) {
		while (retry < HTC_AS_HEADSET_DETECT_RETRY) {
			if (!is_typec_headphone_position()) {
				htc_typec_hs_set_s3s4_position(POSITION_NEGATIVE);
				usleep_range(HTC_AS_HEADSET_DEBOUNCE_TIME, HTC_AS_HEADSET_DEBOUNCE_TIME);
				pr_info("%s: position flip\n", __func__);
			}

			if (is_typec_headphone_position()) {
				pr_info("%s headset switch status S3_0=%d S3_1=%d S4=%d S5=%d\n", __func__,
					htc_typec_hs_switch_gpio_get(HEADSET_S3_0),
					htc_typec_hs_switch_gpio_get(HEADSET_S3_1),
					htc_typec_hs_switch_gpio_get(HEADSET_S4),
					htc_typec_hs_switch_gpio_get(HEADSET_S5));
				break;
			}

			htc_typec_hs_set_s3s4_position(POSITION_POSITIVE);
			usleep_range(HTC_AS_HEADSET_DEBOUNCE_TIME, HTC_AS_HEADSET_DEBOUNCE_TIME);
			pr_info("%s: retry=%d ID1=%d ID2=%d POSITION=%d\n", __func__, retry,
				htc_typec_hs_id_gpio_get(TYPEC_ID1),
				htc_typec_hs_id_gpio_get(TYPEC_ID2),
				htc_typec_hs_id_gpio_get(TYPEC_POSITION));

			retry++;
		}

		if (is_typec_headphone_position()) {
			mbhc->hph_status |= SND_JACK_HEADSET;
			__wcd_mbhc_jack_report(mbhc, (mbhc->hph_status | SND_JACK_MECHANICAL),
									WCD_MBHC_JACK_MASK);
			pr_info("%s: Reporting insertion (%x)\n", __func__, mbhc->hph_status);
			mbhc->current_plug = MBHC_PLUG_TYPE_AS_HEADSET;
			htc_typec_hs_button_detection(mbhc, true);
		} else {
			ret = -1;
			pr_err("%s retry count = %d",__func__, retry);
		}
	} else if (mbhc->current_plug == MBHC_PLUG_TYPE_AS_HEADSET && !insert) {
		mbhc->hph_status &= ~SND_JACK_HEADSET;
		__wcd_mbhc_jack_report(mbhc, mbhc->hph_status, WCD_MBHC_JACK_MASK);
		pr_info("%s: Reporting removal (%x)\n", __func__, mbhc->hph_status);
		mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
		htc_typec_hs_button_detection(mbhc, false);
	} else {
		pr_err("%s: unknown mbhc->current_plug : %d insert %d hph_status %x\n",
			__func__, mbhc->current_plug, insert, mbhc->hph_status);
	}

	if (!insert) {
		htc_typec_hs_set_s5_en(false);
	}

	WCD_MBHC_RSC_UNLOCK(mbhc);

	return ret;
}

static void htc_typec_adapter_irq_handler(struct wcd_mbhc *mbhc, int insert)
{
	pr_info("%s: enter, current_plug=%d\n", __func__, mbhc->current_plug);

	WCD_MBHC_RSC_LOCK(mbhc);

	/* cancel pending button press */
	if (__wcd_cancel_btn_work(mbhc))
		pr_info("%s: button press is canceled\n", __func__);

	if (mbhc->current_plug == MBHC_PLUG_TYPE_NONE && insert) {
		if (!is_typec_adapter_positive()) {
			htc_typec_hs_set_s3s4_position(POSITION_NEGATIVE);
			usleep_range(HTC_AS_HEADSET_DEBOUNCE_TIME, HTC_AS_HEADSET_DEBOUNCE_TIME);
			pr_info("%s: position flip\n", __func__);
		}
		if (is_typec_adapter_positive()) {
#if 0 /* disable 3.5mm adapter */
			int adc = 0;
			htc_hs_qpnp_remote_adc(&adc);
			if (adc > htc_headset.adc_25mm_min) {
				mbhc->current_plug = MBHC_PLUG_TYPE_25MM_HEADSET;
				mbhc->hph_status |= SND_JACK_HEADSET;
				wcd9xxx_jack_report(mbhc, &mbhc->headset_jack, mbhc->hph_status,
						    WCD9XXX_JACK_MASK);
				pr_info("%s: Reporting insertion (%x)\n", __func__, mbhc->hph_status);
			} else if (adc > htc_headset.adc_35mm_min && adc < htc_headset.adc_35mm_max) {
				mbhc->current_plug = MBHC_PLUG_TYPE_35MM_HEADSET;
			}
			else {
				pr_err("%s: adc is not in range (%d)\n", __func__, adc);
			}
#else
			mbhc->current_plug = MBHC_PLUG_TYPE_25MM_HEADSET;
			mbhc->hph_status |= SND_JACK_HEADSET;
			__wcd_mbhc_jack_report(mbhc, mbhc->hph_status, WCD_MBHC_JACK_MASK);
			pr_info("%s: Reporting insertion (%x)\n", __func__, mbhc->hph_status);
#endif
		} else {
			pr_err("%s: should not be here: ID1=%d ID2=%d\n", __func__,
				htc_typec_hs_id_gpio_get(TYPEC_ID1),
				htc_typec_hs_id_gpio_get(TYPEC_ID2));
		}
#if 0 /* disable 3.5mm adapter */
	} else if ((mbhc->current_plug == MBHC_PLUG_TYPE_25MM_HEADSET || mbhc->current_plug == MBHC_PLUG_TYPE_35MM_HEADSET) && !insert) {
#else
	} else if (mbhc->current_plug == MBHC_PLUG_TYPE_25MM_HEADSET && !insert) {
#endif
		mbhc->hph_status &= ~SND_JACK_HEADSET;
		__wcd_mbhc_jack_report(mbhc, mbhc->hph_status, WCD_MBHC_JACK_MASK);
		pr_info("%s: Reporting removal (%x)\n", __func__, mbhc->hph_status);
		mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
	} else {
		pr_err("%s: unknown mbhc->current_plug : %d insert %d hph_status %x\n",
				__func__, mbhc->current_plug, insert, mbhc->hph_status);
	}

	WCD_MBHC_RSC_UNLOCK(mbhc);
}

irqreturn_t htc_typec_hs_plug_detect_irq(struct wcd_mbhc *mbhc)
{
	int insert = -1, ret = 0;

	if (htc_headset.htc_headset_init) {
		WCD_MBHC_RSC_LOCK(mbhc);
		if (__wcd_cancel_btn_work(mbhc)) {
			pr_err("%s: button press is canceled\n", __func__);
		}
		WCD_MBHC_RSC_UNLOCK(mbhc);

		WCD_MBHC_REG_READ(WCD_MBHC_MECH_DETECTION_TYPE, insert);

		/* Set the detection type appropriately */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MECH_DETECTION_TYPE, !insert);

		if (insert) {
/* OCN ONLY + */
			if (pre_condition)
				qpnp_pin_pull_config(aud_remote_sensor_gpio.no, 1/*QPNP_PIN_PULL_UP_1P5*/);
/* OCN ONLY - */
			htc_typec_hs_set_s5_en(true);
			htc_typec_hs_set_ext_micbias(true, ANC_Headset_Detection_Mask);
			htc_typec_hs_set_s3s4_position(POSITION_POSITIVE);
			htc_typec_hs_enable_mic_bias(mbhc, true);
			usleep_range(HTC_AS_HEADSET_DEBOUNCE_TIME, HTC_AS_HEADSET_DEBOUNCE_TIME);
			pr_info("%s: ID1=%d ID2=%d POSITION=%d\n", __func__,
				htc_typec_hs_id_gpio_get(TYPEC_ID1),
				htc_typec_hs_id_gpio_get(TYPEC_ID2),
				htc_typec_hs_id_gpio_get(TYPEC_POSITION));

			if (htc_typec_hs_get_accessory_type() == ACCESSORY_HEADPHONE
				|| mbhc->current_plug == MBHC_PLUG_TYPE_AS_HEADSET) {
				ret = htc_typec_hs_irq_handler(mbhc, insert);
				htc_typec_hs_enable_mic_bias(mbhc, false);
#if 0 /* disable 3.5mm adapter */
			} else if (htc_typec_hs_get_accessory_type() == ACCESSORY_ADAPTER || mbhc->current_plug == MBHC_PLUG_TYPE_25MM_HEADSET || mbhc->current_plug == MBHC_PLUG_TYPE_35MM_HEADSET){
#else
			} else if (htc_typec_hs_get_accessory_type() == ACCESSORY_ADAPTER
					|| mbhc->current_plug == MBHC_PLUG_TYPE_25MM_HEADSET) {
#endif
				htc_typec_adapter_irq_handler(mbhc, insert);
				htc_typec_hs_set_s5_en(false);
				htc_typec_hs_enable_mic_bias(mbhc, false);
			} else {
				pr_info("%s: mbhc->current_plug : %d insert %d hph_status %x\n",
						__func__, mbhc->current_plug, insert, mbhc->hph_status);
				htc_typec_hs_enable_mic_bias(mbhc, false);
				htc_typec_hs_set_s5_en(false);
			}

/* OCN ONLY + */
			if (pre_condition)
				qpnp_pin_pull_config(aud_remote_sensor_gpio.no, -1/*Recover*/);
/* OCN ONLY - */

			htc_typec_hs_set_ext_micbias(false, ANC_Headset_Detection_Mask); /* HPKB29557 ANC feature*/
			/* set switch to accessory mode */
			htc_typec_hs_set_fsa3030_accessory(true);

			/* after fsa3030 switched */
			__wcd_mbhc_clr_and_turnon_hph_padac(mbhc);

			if (ret < 0) {
				/* Plug in AS headset when pressing button */
				pr_err("AS headset can't be identified during 3 seconds ");
				return IRQ_HANDLED;
			}
		} else {
			/* set switch to usb mode */
			htc_typec_hs_set_fsa3030_accessory(false);

			pr_info("%s: Remove mbhc->current_plug : %d insert %d hph_status %x\n",
					__func__, mbhc->current_plug, insert, mbhc->hph_status);

			htc_typec_hs_set_s5_en(false);
			htc_typec_hs_set_s3_pos(S3_MODE_NONE);
			htc_typec_hs_set_s4_pos(S4_MODE_NONE);

			if (mbhc->current_plug == MBHC_PLUG_TYPE_25MM_HEADSET) {
				WCD_MBHC_RSC_LOCK(mbhc);
				mbhc->hph_status &= ~SND_JACK_HEADSET;
				__wcd_mbhc_jack_report(mbhc, mbhc->hph_status, WCD_MBHC_JACK_MASK);
				pr_info("%s: Reporting removal (%x)\n", __func__, mbhc->hph_status);
				mbhc->current_plug = MBHC_PLUG_TYPE_NONE;

				WCD_MBHC_RSC_UNLOCK(mbhc);
				return IRQ_HANDLED;
			} else if (mbhc->current_plug == MBHC_PLUG_TYPE_AS_HEADSET) {
				WCD_MBHC_RSC_LOCK(mbhc);
				mbhc->hph_status &= ~SND_JACK_HEADSET;
				__wcd_mbhc_jack_report(mbhc, mbhc->hph_status, WCD_MBHC_JACK_MASK);
				pr_info("%s: Reporting removal (%x)\n", __func__, mbhc->hph_status);
				mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
				htc_typec_hs_button_detection(mbhc, false);
				htc_typec_hs_set_s5_en(false);

				WCD_MBHC_RSC_UNLOCK(mbhc);
				__wcd_mbhc_set_and_turnoff_hph_padac(mbhc);
				return IRQ_HANDLED;
			}
		}

		if (0) { /* disable 3.5mm adapter */
			if (mbhc->current_plug != MBHC_PLUG_TYPE_25MM_HEADSET
				&& mbhc->current_plug != MBHC_PLUG_TYPE_AS_HEADSET) {
				pr_info("%s: call mbhc swch\n", __func__);
				__wcd_mbhc_swch_irq_handler(mbhc);
			}
		}

		/* report unsupport if it's non HTC analog adapter */
		if (mbhc->current_plug != MBHC_PLUG_TYPE_25MM_HEADSET
			&& mbhc->current_plug != MBHC_PLUG_TYPE_AS_HEADSET) {
			if (insert) {
				if (htc_headset.unsupported_type.name != NULL)
					switch_set_state(&htc_headset.unsupported_type, 1);
				else
					pr_err("%s: unsupported_type is null\n", __func__);
				pr_info("%s: Reporting insertion unsupported device \n", __func__);
			} else {
				if (htc_headset.unsupported_type.name != NULL)
					switch_set_state(&htc_headset.unsupported_type, 0);
				else
					pr_err("%s: unsupported_type is null\n", __func__);
				pr_info("%s: Reporting removal unsupported device \n", __func__);

				htc_typec_hs_set_s5_en(false);
				htc_typec_hs_set_ext_micbias(false, ANC_Headset_Detection_Mask); /* HPKB29557 ANC feature*/
				htc_typec_hs_set_s3_pos(S3_MODE_NONE);
				htc_typec_hs_set_s4_pos(S4_MODE_NONE);
			}
		}

		if (insert >= 0) {
			if (!insert && mbhc->current_plug != MBHC_PLUG_TYPE_NONE) {
				WCD_MBHC_RSC_LOCK(mbhc);
				pr_err("%s: force remove hph_status=%d, current_plug=%d\n",
						__func__, mbhc->hph_status, mbhc->current_plug);
				mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
				mbhc->hph_status = 0;
				__wcd_mbhc_jack_report(mbhc, mbhc->hph_status, WCD_MBHC_JACK_MASK);
				pr_info("%s %d: Reporting removal (%x)\n", __func__, __LINE__, mbhc->hph_status);
				htc_typec_hs_button_detection(mbhc, false);
				/* Set the detection type appropriately */
				WCD_MBHC_RSC_UNLOCK(mbhc);
			}
		}
	}

	return IRQ_HANDLED;
}

int htc_typec_hs_init(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *headset_gpio_pinctrl = NULL;

	if (htc_headset.htc_headset_init)
		return 0;


	ret = htc_typec_hs_dt_parser(pdev);
	if (ret) {
		pr_err("%s: headset dt parser failed\n", __func__);
		return ret;
	}

	headset_gpio_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(headset_gpio_pinctrl)) {
		pr_err("%s: target does not have pinctrl\n", __func__);
	} else if (headset_gpio_pinctrl) {
		ret = headset_pin_init(headset_gpio_pinctrl);
		if (ret) {
			pr_err("%s: headset gpios init fail, ret=%d\n", __func__, ret);
		}
	}

	mutex_init(&ext_micbias_mutex);

	sound_device = pdev;
	htc_headset.htc_headset_init = true;

	return ret;
}

void htc_typec_hs_deinit()
{
	mutex_destroy(&ext_micbias_mutex);
}

int htc_typec_hs_get_headset_type(void)
{
	return (__MBHC)? __MBHC->current_plug:0;
}

static int htc_typec_hs_lr_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (!strcmp(w->name, "ANCLeft Headset Mic")) {
			htc_typec_hs_set_ext_micbias(true, ANCLeft_Headset_Mic_Mask);
		}
		else if (!strcmp(w->name, "ANCRight Headset Mic")) {
			htc_typec_hs_set_ext_micbias(true, ANCRight_Headset_Mic_Mask);
		}
		else {
			pr_err("%s: wrong widget in ON event", __func__);
			return -EINVAL;
		}
	} else {
		if (!strcmp(w->name, "ANCLeft Headset Mic")) {
			htc_typec_hs_set_ext_micbias(false, ANCLeft_Headset_Mic_Mask);
		}
		else if (!strcmp(w->name, "ANCRight Headset Mic")) {
			htc_typec_hs_set_ext_micbias(false, ANCRight_Headset_Mic_Mask);
		}
		else {
			pr_err("%s: wrong widget in ON event", __func__);
			return -EINVAL;
		}
	}
	return 0;
}

static const struct snd_soc_dapm_widget htc_as_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("ANCRight Headset Mic", htc_typec_hs_lr_event),
	SND_SOC_DAPM_MIC("ANCLeft Headset Mic", htc_typec_hs_lr_event),
};

int htc_typec_hs_dapm_new_controls(struct snd_soc_dapm_context *dapm)
{
	return snd_soc_dapm_new_controls(dapm, htc_as_dapm_widgets,
				ARRAY_SIZE(htc_as_dapm_widgets));
}

static int register_switch_devices(void)
{
	int ret = 0;
	if (htc_headset.unsupported_type.name != NULL)
		return ret;

	pr_info("[AUD][HS] %s\n", __func__);

	htc_headset.debug_reg_count = 0;
	htc_headset.unsupported_type.name = "Unsupported_device";
	htc_headset.unsupported_type.print_name = htc_typec_hs_headset_print_name;

	ret = switch_dev_register(&htc_headset.unsupported_type);
	if (ret) {
		htc_headset.unsupported_type.name = NULL;
		htc_headset.unsupported_type.print_name = NULL;
	}
	return ret;
}

static void unregister_switch_devices(void)
{
	if (htc_headset.unsupported_type.name != NULL)
		switch_dev_unregister(&htc_headset.unsupported_type);
}


/* ---------- ATTR START ---------- */
#define DEVICE_HEADSET_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_headset_##_name = \
	__ATTR(_name, _mode, _show, _store)

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_##_name = \
	__ATTR(flag, _mode, _show, _store)

static int __CTOI(char s)
{
	int ret = 0;
	if (s >= 'a' && s <= 'f') {
		ret = 10 + s - 'a';
	} else if (s >= 'A' && s <= 'F') {
		ret = 10 + s - 'A';
	} else if (s >= '0' && s <= '9') {
		ret = s - '0';
	}
	return ret;
}

static int __ATOI(char *s)
{
	int ret = 0;
	if (strlen(s) == 3) {
		ret += __CTOI(s[0]) * 256;
		ret += __CTOI(s[1]) * 16;
		ret += __CTOI(s[2]);
	}
	pr_info("[AUD][HS] ATOI ret = %d\n", ret);
	return ret;
}

static void dump_register(u16 reg)
{
	int i = 0;

	if (__MBHC == NULL) {
		pr_err("[AUD][HS] __MBHC is NULL\n");
		return;
	}

	if (reg == 0 || reg > 0x6FF) {
		pr_err("[AUD][HS] wrong register\n");
		return;
	}

	if (htc_headset.debug_reg_count >= 50) {
		pr_err("[AUD][HS] debug count more than 50\n");
		return;
	}

	for (i = 0; i < htc_headset.debug_reg_count; i++) {
		if (htc_headset.debug_reg[i] == reg) break;
	}

	if (i == htc_headset.debug_reg_count) {
		htc_headset.debug_reg[htc_headset.debug_reg_count] = reg;
		htc_headset.debug_reg_count++;
	}
	pr_info("[AUD][HS] dump reg %x, current dump count = %d\n",
			reg, htc_headset.debug_reg_count);
/*
	for (i = 0; i < htc_headset.debug_reg_count; i++)
		pr_info("[AUD][HS] reg %x will be dumpped\n", htc_headset.debug_reg[i]);
*/
}

static void undump_register(u16 reg)
{
	int i = 0;

	if (reg == 0 || reg > 0x6FF) {
		pr_err("[AUD][HS] wrong register\n");
		return;
	}

	for (i = 0; i < htc_headset.debug_reg_count; i++)
		if (htc_headset.debug_reg[i] == reg) break;

	if (i < htc_headset.debug_reg_count) {
		for (; i < htc_headset.debug_reg_count; i++)
			if (i + 1 < htc_headset.debug_reg_count)
				htc_headset.debug_reg[i] = htc_headset.debug_reg[i + 1];
		htc_headset.debug_reg_count--;
	}

	pr_info("[AUD][HS] undump reg %x, current dump count = %d\n",
			reg, htc_headset.debug_reg_count);
/*
	for (i = 0; i < htc_headset.debug_reg_count; i++)
		pr_info("[AUD][HS] reg %x will be dumpped\n", htc_headset.debug_reg[i]);
*/
}

static ssize_t debug_flag_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0, i;
	char *s;

	s = buf;
	if (__MBHC == NULL) {
		return (buf - s);
	}

	len =  scnprintf(buf, PAGE_SIZE - 1, "HP_DET = %d\n",
				(__MBHC->current_plug != MBHC_PLUG_TYPE_NONE) ? 0 : 1);
	buf += len;

	for (i = 0; i < htc_headset.debug_reg_count; i++) {
		len =  scnprintf(buf, PAGE_SIZE - 1, "reg 0x%x value 0x%x\n",
					htc_headset.debug_reg[i],
					snd_soc_read(__MBHC->codec, htc_headset.debug_reg[i]));
		buf += len;
	}

	return (buf - s);
}

static ssize_t debug_flag_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u16 reg;
	char s[4] = {'\0'};

	if (strncmp(buf, "enable", count - 1) == 0) {
		pr_info("[AUD][HS] debug enable\n");
	} else if (strncmp(buf, "disable", count - 1) == 0) {
		pr_info("[AUD][HS] debug disable\n");
	} else if (strncmp(buf, "dump 0x", count - 4) == 0) {
		s[0] = buf[7]; s[1] = buf[8]; s[2] = buf[9];
		reg = __ATOI(s);
		dump_register(reg);
	} else if (strncmp(buf, "undump 0x", count - 4) == 0) {
		s[0] = buf[9]; s[1] = buf[10]; s[2] = buf[11];
		reg = __ATOI(s);
		undump_register(reg);
	} else if (strncmp(buf, "no_headset", count - 1) == 0) {
		pr_info("[AUD][HS] set no headset status\n");
	} else {
		pr_err("[AUD][HS] Invalid parameter");
		return count;
	}

	return count;
}

static ssize_t headset_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	char *state = NULL;

	switch (__MBHC->current_plug) {
	case MBHC_PLUG_TYPE_NONE:
		state = "headset_unplug";
		break;
	case MBHC_PLUG_TYPE_HEADSET:
		state = "headset_mic";
		break;
	case MBHC_PLUG_TYPE_HEADPHONE:
		state = "headset_no_mic";
		break;
	case MBHC_PLUG_TYPE_HIGH_HPH:
		state = "headset_tv_out";
		break;
	case MBHC_PLUG_TYPE_GND_MIC_SWAP:
		state = "headset_gnd_mic_swap";
		break;
	case MBHC_PLUG_TYPE_AS_HEADSET:
		state = "AS_headset_mic";
		break;
	case MBHC_PLUG_TYPE_35MM_HEADSET:
		state = "35mm_headset";
		break;
	case MBHC_PLUG_TYPE_25MM_HEADSET:
		state = "25mm_headset";
		break;
	default:
		state = "error_state";
	}

	count = scnprintf(buf, PAGE_SIZE - 1, "%s\n", state);

	return count;
}

static ssize_t headset_state_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("%s: cmd %s\n", __func__, buf);
	return 0;
}

static ssize_t headset_simulate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE - 1, "Command is not supported\n");
}

static ssize_t headset_simulate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("%s: cmd %s, ++\n", __func__, buf);
	if (strncmp(buf, "headset_unplug", count - 1) == 0) {
		pr_info("Headset Simulation: headset_unplug\n");
		__MBHC->current_plug = MBHC_PLUG_TYPE_NONE;
		__MBHC->hph_status &= ~WCD_MBHC_JACK_MASK;
	} else if (strncmp(buf, "headset_no_mic", count - 1) == 0) {
		pr_info("Headset Simulation: headset_no_mic\n");
		__MBHC->current_plug = MBHC_PLUG_TYPE_HEADPHONE;
		__MBHC->hph_status &= ~WCD_MBHC_JACK_MASK;
		__MBHC->hph_status |= SND_JACK_HEADPHONE;
	} else if (strncmp(buf, "headset_mic", count - 1) == 0) {
		pr_info("Headset Simulation: headset_mic\n");
		__MBHC->current_plug = MBHC_PLUG_TYPE_HEADSET;
		__MBHC->hph_status &= ~WCD_MBHC_JACK_MASK;
		__MBHC->hph_status |= SND_JACK_HEADSET;
	} else if (strncmp(buf, "as_headset_mic", count - 1) == 0) {
		pr_info("Headset Simulation: as_headset_mic\n");
		__MBHC->current_plug = MBHC_PLUG_TYPE_AS_HEADSET;
		__MBHC->hph_status &= ~WCD_MBHC_JACK_MASK;
		__MBHC->hph_status |= SND_JACK_HEADSET;
	} else if (strncmp(buf, "25mm_headset", count - 1) == 0) {
		pr_info("Headset Simulation: 25mm_headset\n");
		__MBHC->current_plug = MBHC_PLUG_TYPE_25MM_HEADSET;
		__MBHC->hph_status &= ~WCD_MBHC_JACK_MASK;
		__MBHC->hph_status |= SND_JACK_HEADSET;
	} else {
		pr_info("Invalid parameter\n");
		return count;
	}

	snd_soc_jack_report(&__MBHC->headset_jack, __MBHC->hph_status, WCD_MBHC_JACK_MASK);

	pr_info("%s: --\n", __func__);
	return count;
}

static ssize_t headset_switch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "S3 status [PMI8998 GPIO%d: %d, GPIO%d: %d]\n",
					htc_headset.switch_gpio[HEADSET_S3_0].no,
					htc_typec_hs_switch_gpio_get(HEADSET_S3_0),
					htc_headset.switch_gpio[HEADSET_S3_1].no,
					htc_typec_hs_switch_gpio_get(HEADSET_S3_1));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "S4 status [GPIO%d: %d]\n",
					htc_headset.switch_gpio[HEADSET_S4].no,
					htc_typec_hs_switch_gpio_get(HEADSET_S4));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "S5 status [PM8998 GPIO%d: %d]\n",
					htc_headset.switch_gpio[HEADSET_S5].no,
					htc_typec_hs_switch_gpio_get(HEADSET_S5));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "ID1 status [GPIO%d: %d]\n",
					htc_headset.id_gpio[TYPEC_ID1].no,
					htc_typec_hs_id_gpio_get(TYPEC_ID1));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "ID2 status [GPIO%d: %d]\n",
					htc_headset.id_gpio[TYPEC_ID2].no,
					htc_typec_hs_id_gpio_get(TYPEC_ID2));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "USB Position status [GPIO%d: %d]\n",
					htc_headset.id_gpio[TYPEC_POSITION].no,
					htc_typec_hs_id_gpio_get(TYPEC_POSITION));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "ext micbias status [PM8998 GPIO%d: %d]\n",
					htc_headset.switch_gpio[HSMIC_BIAS_EN].no,
					gpio_get_value(htc_headset.switch_gpio[HSMIC_BIAS_EN].no));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "FSA3030 Sel0 status [GPIO%d: %d]\n",
					htc_headset.switch_gpio[FSA3030_SEL0].no,
					htc_typec_hs_switch_gpio_get(FSA3030_SEL0));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "FSA3030 Sel1 status [GPIO%d: %d]\n",
					htc_headset.switch_gpio[FSA3030_SEL1].no,
					htc_typec_hs_switch_gpio_get(FSA3030_SEL1));

	ret += scnprintf(buf + ret, PAGE_SIZE - 1, "FSA3030 Sel2 status [GPIO%d: %d]\n",
					htc_headset.switch_gpio[FSA3030_SEL2].no,
					htc_typec_hs_switch_gpio_get(FSA3030_SEL2));

	return ret;
}

static ssize_t headset_switch_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	pr_info("%s: cmd %s, ++\n", __func__, buf);
	if (strncmp(buf, "s3_p", count - 1) == 0) {
		htc_typec_hs_set_s3_pos(S3_MODE_P);
	} else if (strncmp(buf, "s3_n", count - 1) == 0) {
		htc_typec_hs_set_s3_pos(S3_MODE_N);
	} else if (strncmp(buf, "s4_p", count - 1) == 0) {
		htc_typec_hs_set_s4_pos(S4_MODE_P);
	} else if (strncmp(buf, "s4_n", count - 1) == 0) {
		htc_typec_hs_set_s4_pos(S4_MODE_N);
	} else if (strncmp(buf, "s5_on", count - 1) == 0) {
		htc_typec_hs_set_s5_en(true);
	} else if (strncmp(buf, "s5_off", count - 1) == 0) {
		htc_typec_hs_set_s5_en(false);
	} else if (strncmp(buf, "ext_micb_on", count - 1) == 0) {
		htc_typec_hs_set_ext_micbias_force_en(true);
	} else if (strncmp(buf, "ext_micb_off", count - 1) == 0) {
		htc_typec_hs_set_ext_micbias_force_en(false);
	} else if (strncmp(buf, "fsa_usb", count - 1) == 0) {
		htc_typec_hs_set_fsa3030_accessory(false);
	} else if (strncmp(buf, "fsa_aud", count - 1) == 0) {
		htc_typec_hs_set_fsa3030_accessory(true);
	} else {
		pr_err("%s: error setting\n", __func__);
	}
	pr_info("%s: --\n", __func__);
	return count;
}

static DEVICE_ACCESSORY_ATTR(debug, 0644, debug_flag_show, debug_flag_store);
static DEVICE_HEADSET_ATTR(state, 0644, headset_state_show,
			   headset_state_store);
static DEVICE_HEADSET_ATTR(simulate, 0644, headset_simulate_show,
				headset_simulate_store);
static DEVICE_HEADSET_ATTR(switch, 0644, headset_switch_show,
				headset_switch_store);

static int register_attributes(void)
{
	int ret = 0;
	if (htc_headset.htc_accessory_class)
		return ret;

	pr_info("[AUD][HS] %s\n", __func__);

	htc_headset.htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(htc_headset.htc_accessory_class)) {
		ret = PTR_ERR(htc_headset.htc_accessory_class);
		htc_headset.htc_accessory_class = NULL;
		goto err_create_class;
	}

	/* Register headset attributes */
	htc_headset.headset_dev = device_create(htc_headset.htc_accessory_class,
					NULL, 0, "%s", "headset");
	if (unlikely(IS_ERR(htc_headset.headset_dev))) {
		ret = PTR_ERR(htc_headset.headset_dev);
		htc_headset.headset_dev = NULL;
		goto err_create_headset_device;
	}

	ret = device_create_file(htc_headset.headset_dev, &dev_attr_headset_state);
	if (ret) {
		goto err_create_headset_state_device_file;
	}

	ret = device_create_file(htc_headset.headset_dev, &dev_attr_headset_simulate);
	if (ret) {
		goto err_create_headset_simulate_device_file;
	}

	ret = device_create_file(htc_headset.headset_dev, &dev_attr_headset_switch);
	if (ret) {
		goto err_create_headset_switch_device_file;
	}

	/* Register debug attributes */
	htc_headset.debug_dev = device_create(htc_headset.htc_accessory_class,
								NULL, 0, "%s", "debug");
	if (unlikely(IS_ERR(htc_headset.debug_dev))) {
		ret = PTR_ERR(htc_headset.debug_dev);
		htc_headset.debug_dev = NULL;
		goto err_create_debug_device;
	}

	ret = device_create_file(htc_headset.debug_dev, &dev_attr_debug);
	if (ret)
		goto err_create_debug_device_file;

	return 0;

err_create_debug_device_file:
	device_unregister(htc_headset.debug_dev);

err_create_debug_device:
	device_remove_file(htc_headset.headset_dev, &dev_attr_headset_switch);

err_create_headset_switch_device_file:
	device_remove_file(htc_headset.headset_dev, &dev_attr_headset_simulate);

err_create_headset_simulate_device_file:
	device_remove_file(htc_headset.headset_dev, &dev_attr_headset_state);

err_create_headset_state_device_file:
	device_unregister(htc_headset.headset_dev);

err_create_headset_device:
	class_destroy(htc_headset.htc_accessory_class);

err_create_class:
	pr_err("[AUD][HS] %s error\n", __func__);
	return ret;
}

static void unregister_attributes(void)
{
	device_remove_file(htc_headset.headset_dev, &dev_attr_headset_switch);
	device_remove_file(htc_headset.headset_dev, &dev_attr_headset_simulate);
	device_remove_file(htc_headset.headset_dev, &dev_attr_headset_state);
	device_remove_file(htc_headset.debug_dev, &dev_attr_debug);
	device_unregister(htc_headset.headset_dev);
	device_unregister(htc_headset.debug_dev);
	class_destroy(htc_headset.htc_accessory_class);
}
/* ---------- ATTR - END ---------- */


int htc_typec_hs_set_mbhc(struct wcd_mbhc *mbhc)
{
	int ret = 0;

	__MBHC = mbhc;

	ret = register_switch_devices();
	if (ret) {
		pr_err("%s: register switch devices failed (%d) \n", __func__, ret);
		return ret;
	}
	ret = register_attributes();
	if (ret) {
		pr_err("%s: register debug attributes fail (%d)\n", __func__, ret);
		goto err_reg_attr;
	}

	return 0;

err_reg_attr:
	unregister_switch_devices();

	return ret;
}

void htc_typec_hs_remove_mbhc(struct wcd_mbhc *mbhc)
{
	unregister_attributes();
	unregister_switch_devices();
}
