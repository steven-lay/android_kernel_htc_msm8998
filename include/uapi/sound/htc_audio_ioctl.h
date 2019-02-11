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
#ifndef __HTC_AUDIO_IOCTL_H
#define __HTC_AUDIO_IOCTL_H

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_GET_HW_COMPONENT	_IOR(ACOUSTIC_IOCTL_MAGIC, 45, unsigned)
#define ACOUSTIC_UPDATE_BEATS_STATUS	_IOW(ACOUSTIC_IOCTL_MAGIC, 47, unsigned)
#define ACOUSTIC_UPDATE_DQ_STATUS	_IOW(ACOUSTIC_IOCTL_MAGIC, 52, unsigned)
#define ACOUSTIC_NOTIFIY_FM_SSR		_IOW(ACOUSTIC_IOCTL_MAGIC, 53, unsigned)
#define ACOUSTIC_CONTROL_WAKELOCK	_IOW(ACOUSTIC_IOCTL_MAGIC, 77, unsigned)
#define ACOUSTIC_ADSP_CMD		_IOW(ACOUSTIC_IOCTL_MAGIC, 98, unsigned)
#define ACOUSTIC_GET_HEADSET_TYPE	_IOR(ACOUSTIC_IOCTL_MAGIC, 53, unsigned)
/* For msm set param */
#define ACOUSTIC_MSM_SETPARAM		_IOW(ACOUSTIC_IOCTL_MAGIC, 66, struct htc_adsp_params_ioctl_s)

#define AUD_HW_NUM            10
#define HTC_AUDIO_TPA2051     0x01
#define HTC_AUDIO_TPA2026     0x02
#define HTC_AUDIO_TPA2028     0x04
#define HTC_AUDIO_A1028       0x08
#define HTC_AUDIO_TPA6185     0x10
#define HTC_AUDIO_RT5501      0x20
#define HTC_AUDIO_TFA9887     0x40
#define HTC_AUDIO_TFA9887L    0x80
#define HTC_AUDIO_TFA9888     0x100
#define HTC_AUDIO_RT5503      0x200

/* AS2 effect */
typedef enum {
	eEF_param_type_min    = 0,
	eEF_as2_coef_left_im  = 0,
	eEF_as2_coef_left_re  = 1,
	eEF_as2_coef_right_im = 2,
	eEF_as2_coef_right_re = 3,
	eEF_lmt_coef          = 4,
	eEF_lmt_rtc           = 5,
	eEF_as2_rtc           = 6,
	eEF_param_type_num,
} adsp_effect_params_type;

typedef struct htc_adsp_params_ioctl_s {
	adsp_effect_params_type type;
	uint32_t size;
	uint16_t params[2+2+2+512]; /* module|param|data_size|data */
} htc_adsp_params_ioctl_t;

#endif
