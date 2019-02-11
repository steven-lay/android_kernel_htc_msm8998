#ifndef _HTC_RADIO_SMEM_H
#define _HTC_RADIO_SMEM_H

#include <linux/types.h>

#define HTC_RADIO_SMEM_VERSION	0x20170119

struct htc_smem_type {
	uint32_t	version;
	uint32_t	struct_size;
	uint32_t	htc_smem_pid;
	uint32_t	htc_smem_app_run_mode;
	uint8_t		htc_rom_ver[16];
	uint8_t		htc_smem_skuid[48];
	uint32_t	htc_smem_factory_reset;
	uint32_t	htc_smem_ce_radio_dbg_flag;
	/* above variables align all projects */
	/* below variables align in-house projects */
	uint32_t	htc_smem_ce_radio_dbg_flag_ext1;
	uint32_t	htc_smem_ce_radio_dbg_flag_ext2;
	uint32_t	secure_flag;
	uint8_t		htc_smem_cid[8];
	uint8_t		htc_smem_radio_parameter[4];
	uint8_t		reserved[1936];
	/* totally 2048 bytes */
};

#endif /* end of _HTC_RADIO_SMEM_H */
