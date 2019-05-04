/* OIS calibration interface for LC898123 F40
 *
 */
#include	"PhoneUpdate.h"

void RamWrite32A(UINT_16 RamAddr, UINT_32 RamData);
void RamRead32A(UINT_16 RamAddr, UINT_32 * ReadData);
int CntWrt(UINT_8 * PcSetDat, UINT_16 CntWrt);
int CntRd3(UINT_32 addr, void *	PcSetDat, UINT_16	UsDatNum);
void WitTim(UINT_16) ;
void WPBCtrl(UINT_8 UcCtrl);
unsigned char htc_ext_GyroReCalib(struct msm_camera_i2c_client *i2c_client, int cam_id);
unsigned char htc_ext_WrGyroOffsetData( void );
void htc_ext_FlashSectorRead(struct msm_camera_i2c_client *i2c_client, unsigned char *data_ptr, UINT_32 address, UINT_32 num_byte);
void htc_ext_FlashInt32Write(struct msm_camera_i2c_client *i2c_client, UINT_32 address, UINT_32 data);
int htc_checkFWUpdate(struct msm_camera_i2c_client *i2c_client);
//file operation+
struct file* msm_fopen(const char* path, int flags, int rights);
int msm_fwrite(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
void msm_fclose(struct file* file);
//file operation-
