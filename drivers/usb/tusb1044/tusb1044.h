/*
 * Utility definitions for TUSB1044 USB3.0 re-drive
 *
 * Copyright (C) 2016 HTC Corporation.
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

/* General Registers */
#define REG_DEVICE_ID_BASE	0x00
#define REG_REVISION		0x08
#define REG_CONFIG_CTRL		0x0A
#define REG_CHANNEL_SWAP_SEL	0x0B
#define REG_VOD_DCGAIN		0x0C
#define REG_DP_TX2		0x10
#define REG_DP_TX1		0x11
#define REG_DP_AUX		0x13
#define REG_USB3_TX2		0x20
#define REG_USB3_TX1		0x21

/* register CONFIG_CTRL value */
#define DISABLE_TX_RX	0x00
/* data pin: reverse CC setting due to hw layout*/
#define USB3_ON_CC1	0x05   /* original is 0x01 on CC1 */
#define USB3_ON_CC2	0x01   /* original is 0x05 on CC2 */
#define DP_ON_CC1	0x0E   /* original is 0x0A on CC1 */
#define DP_ON_CC2	0x0A   /* original is 0x0E on CC2 */

/* register DP_AUX value */
#define ENABLE_AUX_SNOOP	0x00
#define DISABLE_AUX_SNOOP	0x80
#define AUX_SBU_CC1	0x10	/* AUXP to SBU1 and AUXP to SBU2 */
#define AUX_SBU_CC2	0x20	/* AUXN to SBU2 and AUXP to SBU1 */
#define EQ_OVERRIDE	0x10

enum cc_state {
	CC_STATE_OPEN = 0,
	CC_STATE_USB3,
	CC_STATE_DP,
	CC_STATE_RARA,
};

/* must align with smblib's integer representation of cc orientation. */
enum cc_orientation {
	CC_ORIENTATION_NONE = 0,
	CC_ORIENTATION_CC1 = 1,
	CC_ORIENTATION_CC2 = 2,
};

void tusb1044_update_state(enum cc_state cc_state, enum cc_orientation cc_orientation);
int tusb1044_exist(void);
