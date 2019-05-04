/*
 *
 * Copyright (C) 2012 HTC Corporation.
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
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/qdsp6v2/apr.h>
#include <sound/htc_audio_ioctl.h>
#include <linux/device.h>
#include <sound/apr_audio-v2.h>

#ifndef _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_
#define _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_

#define pr_aud_fmt(fmt) "[AUD] " KBUILD_MODNAME ": " fmt
#define pr_aud_fmt1(fmt) "[AUD]" fmt
#define pr_aud_err(fmt, ...) \
			printk(KERN_ERR pr_aud_fmt(fmt), ##__VA_ARGS__)
#define pr_aud_err1(fmt, ...) \
			printk(KERN_ERR pr_aud_fmt1(fmt), ##__VA_ARGS__)
#define pr_aud_info(fmt, ...) \
			printk(KERN_INFO pr_aud_fmt(fmt), ##__VA_ARGS__)
#define pr_aud_info1(fmt, ...) \
			printk(KERN_INFO pr_aud_fmt1(fmt), ##__VA_ARGS__)

#if defined(CONFIG_DYNAMIC_DEBUG)
#define pr_aud_debug(fmt, ...) \
	dynamic_pr_debug(pr_aud_fmt(fmt), ##__VA_ARGS__)
#elif defined(DEBUG)
#define pr_aud_debug(fmt, ...) \
	printk(KERN_DEBUG pr_aud_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_aud_debug(fmt, ...) \
	no_printk(KERN_DEBUG pr_aud_fmt(fmt), ##__VA_ARGS__)
#endif

#undef pr_debug
#undef pr_info
#undef pr_err
#define pr_debug(fmt, ...) pr_aud_debug(fmt, ##__VA_ARGS__)
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)

#define ADM_MODULE_ONEDOTONE_AUDIO       0x10000135
#define ADM_MODULE_LIMITERCOPP           0x1000012A
#define ADM_MODULE_ID_MISC_EFFECT        0x10030001
#define ADM_COPP_ID_ONEDOTONE_AUDIO           0x10000009
#define ADM_COPP_ID_HTC_HD_AUDIO              0x10000010
#define ADM_COPP_ID_HTC_USB_AUDIO              0x10000011
#define ADM_PARAM_ID_ONEDOTONE_AUDIO_EN       0x10000137
#define ADM_PARAM_ID_LIMITERCOPP_AUDIO_ENABLE 0x1000012C
#define ADM_PARAM_ID_MISC_SET_STEREO_TO_MONO 0x10030081
#define ADM_PARAM_ID_MISC_SET_ACOUSTIC_SHOCK_RAMP 0x10030101
#define ADM_PARAM_ID_MISC_SET_ACOUSTIC_SHOCK_MUTE 0x10030111

#define ADM_COPP_ID_AUDIOZOOM                    0x10000008
#define ADM_MODULE_ID_AUDIOZOOM_EFFECT           0x10000047
#define ADM_PARAM_ID_AUDIOZOOM_PARAM             0x10000056

/* HTC_AUD_START - AS HS {HPKB:7329}*/
#define AFE_PARAM_ID_ADAPTSOUND_CONF_LEFT_RE     0x10000043
#define AFE_PARAM_ID_ADAPTSOUND_CONF_LEFT_IM     0x10000044
#define AFE_PARAM_ID_ADAPTSOUND_CONF_RIGHT_RE    0x10000045
#define AFE_PARAM_ID_ADAPTSOUND_CONF_RIGHT_IM    0x10000046
#define AFE_PARAM_ID_ADAPTSOUND_CONF_LIMITER     0x1000012D
#define AFE_PARAM_ID_ADAPTSOUND_EFFECT_ENABLE    0x10000042
#define AFE_PARAM_ID_ADAPTSOUND_LIMITER_ENABLE   0x1000012C
#define AFE_MODULE_ID_ADAPTSOUND_EFFECT          0x10000040
#define AFE_MODULE_ID_ADAPTSOUND_LIMITER         0x1000012A
#define AFE_COPP_ID_ADAPTIVE_AUDIO_30            0x10000007
/* HTC_AUD_END - AS HS {HPKB:7329}*/


/* HTC Effect START {HPKB:2082} {HPKB:211} {HPKB:7329}*/
#define SPK_MASK		0x00000001
#define HEADSET_MASK		0x00000002
#define HEADSET441MASK		0x00000004
#define A2DP_MASK		0x00000008
#define USB_MASK		0x00000010
#define MAIN_MIC_MASK		0x00000100

#define COPP_EFFECT		0x00000001
#define POPP_EFFECT		0x00000002
#define HD_SUPPORT		0x00000004
#define SWITCH_CTL		0x00000008
#define AaptiveSound		0x00000010
#define PARAM_CTL		0x00000020

struct htc_effects {
	char effect_name[30];
	uint32_t module_id;
	uint32_t param_id;

	int topology_id;
	int port_mask;
	int flag_mask;
};

static const struct htc_effects htc_effects_array[] = {
	{"ASM_HTC_Misc_StereoMono",  ADM_MODULE_ID_MISC_EFFECT, ADM_PARAM_ID_MISC_SET_STEREO_TO_MONO,
		HTC_POPP_TOPOLOGY, (HEADSET_MASK|HEADSET441MASK|USB_MASK), (POPP_EFFECT|HD_SUPPORT|SWITCH_CTL)},

	{"ADM_HTC_Misc_Mute", ADM_MODULE_ID_MISC_EFFECT, ADM_PARAM_ID_MISC_SET_ACOUSTIC_SHOCK_MUTE,
		ADM_COPP_ID_ONEDOTONE_AUDIO, (SPK_MASK), (COPP_EFFECT|HD_SUPPORT|SWITCH_CTL)},

	{"ADM_HTC_Misc_Ramping", ADM_MODULE_ID_MISC_EFFECT, ADM_PARAM_ID_MISC_SET_ACOUSTIC_SHOCK_RAMP,
		ADM_COPP_ID_ONEDOTONE_AUDIO, (SPK_MASK), (COPP_EFFECT|HD_SUPPORT|SWITCH_CTL)},

	/* HTC_AUD_START - AS HS {HPKB:7329}*/
	{"ADM_HTC_AdaptiveSound3_1", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	{"ADM_HTC_AdaptiveSound3_2", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	{"ADM_HTC_AdaptiveSound3_3", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	{"ADM_HTC_AdaptiveSound3_4", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	{"ADM_HTC_AdaptiveSound3_5", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	{"ADM_HTC_AdaptiveSound3_6", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	{"ADM_HTC_AdaptiveSound3_7", 0, 0, AFE_COPP_ID_ADAPTIVE_AUDIO_30, (HEADSET_MASK|HEADSET441MASK), (COPP_EFFECT|AaptiveSound)},
	/* HTC_AUD_END */

	/* HTC_AUD_START - audio zoom {HPKB:29371}*/
	{"ADM_HTC_Audio_Zoom", ADM_MODULE_ID_AUDIOZOOM_EFFECT, ADM_PARAM_ID_AUDIOZOOM_PARAM,
		ADM_COPP_ID_AUDIOZOOM, (MAIN_MIC_MASK), (COPP_EFFECT|PARAM_CTL)},
	/* HTC_AUD_END */

	/* HTC_AUD_START - onedotone {HPKB:7339}*/
	{"ADM_HTC_ONEDOTONE", ADM_MODULE_ONEDOTONE_AUDIO, ADM_PARAM_ID_ONEDOTONE_AUDIO_EN,
		ADM_COPP_ID_ONEDOTONE_AUDIO, (SPK_MASK), (COPP_EFFECT|SWITCH_CTL)},
	{"ADM_HTC_ONEDOTONE_LIMITER", ADM_MODULE_LIMITERCOPP, ADM_PARAM_ID_LIMITERCOPP_AUDIO_ENABLE,
		ADM_COPP_ID_ONEDOTONE_AUDIO, (SPK_MASK), (COPP_EFFECT|SWITCH_CTL)},
	/* HTC_AUD_END */
};

void htc_effect_by_adm_open(int port_id, int topology);
int htc_set_asm_effect(void* payload, int total_size, int topology, bool hd_support);
int htc_set_adm_effect(void* payload, int total_size, int topology, bool hd_support, int port_id);
/* HTC Effect END */

#ifdef CONFIG_HTC_HEADSET_MGR
enum HS_NOTIFY_TYPE {
	HS_AMP_N = 0,
	HS_CODEC_N,
	HS_N_MAX,
};

struct hs_notify_t {
	int used;
	void *private_data;
	int (*callback_f)(void*,int);
};
#endif

enum HTC_FEATURE {
	HTC_Q6_EFFECT = 0,
	HTC_AUD_24BIT,
};

struct avcs_crash_params {
    struct apr_hdr  hdr;
    uint32_t crash_type;
};

struct acoustic_ops {
	void (*set_q6_effect)(int mode);
	int (*get_htc_revision)(void);
	int (*get_hw_component)(void);
	char* (*get_mid)(void);
	int (*enable_digital_mic)(void);
	int (*enable_24b_audio)(void);
	int (*get_q6_effect) (void);
/* HTC_AUD_START - AS HS {HPKB:7329} */
	int (*msm_setparam)(htc_adsp_params_ioctl_t *ctrl);
/* HTC_AUD_END */
};

void htc_acoustic_register_ops(struct acoustic_ops *ops);

#ifdef CONFIG_HTC_HEADSET_MGR
void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify);
#endif

/* To query if feature is enable */
int htc_acoustic_query_feature(enum HTC_FEATURE feature);

#endif
