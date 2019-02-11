#ifndef __NQ_NCI_HTC_MFG_H
#define __NQ_NCI_HTC_MFG_H

#define FTM_NFC_CPLC 1
#define NFC_READ_RFSKUID 1
#define HAS_NFC_CHIP 0x7000000
#define MAX_NFC_DATA_SIZE (512)
#define MAX_FW_BLOCK_SIZE (512)
#define MAX_NFC_RETRY (100)
#define MAX_READ_TIME (5)
#define NFC_READ_DELAY (10)
#define MAX_QUP_DATA_LEN (224)
#define MAX_CMD_LEN 257

#define NFC_GET_BOOTMODE 1
#define SW_ENABLE_OFFMODECHARGING
/* Define boot mode for NFC*/
#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5

void pn544_hw_reset_control(int en_num);
int mfg_nfc_test(int arg);
#if FTM_NFC_CPLC
char* mfg_nfc_cplc_result(void);
char* mfg_nfc_cplc_result_uid(void);
#endif //FTM_NFC_CPLC
char* mfg_nfc_dpc_result(void);

/******************************************************************************
 *
 *	Function pn553_htc_check_rfskuid:
 *	Return With(1)/Without(0) NFC chip if this SKU can get RFSKUID in kernal
 *	Return is_alive(original value) by default.
 *
 ******************************************************************************/
int pn553_htc_check_rfskuid(int in_is_alive);

/******************************************************************************
 *
 *  Function pn548_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return  NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 ******************************************************************************/
int pn553_htc_get_bootmode(void);

#endif
