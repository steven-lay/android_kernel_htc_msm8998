/* Himax Android Driver Sample Code for HMX852xES chipset
*
* Copyright (C) 2014 Himax Corporation.
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

#include <linux/himax_852xES_cap.h>

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
//Proximity
static u8 proximity_flag = 0;
static u8 g_proximity_en = 0;
#endif

#if defined(HX_AUTO_UPDATE_FW)||defined(HX_AUTO_UPDATE_CONFIG)
	static unsigned char i_CTPM_FW[]=
	{
		//#include "HQ_AW875_HX8527E44_TXD_A.0E.05_20151225.i"
		#include "HQ_AW875_HX8527E44_TXD_A.0E.06_20160107.i"
	};
#endif

unsigned int himax_touch_irq_cap = 0;
struct himax_i2c_platform_data *private_pdata = NULL;

//static int		tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY; // for Virtual key array
static unsigned char	IC_CHECKSUM               = 0;
static unsigned char	IC_TYPE                   = 0;

static int		HX_TOUCH_INFO_POINT_CNT   = 0;
static int		HX_RX_NUM                 = 0;
static int		HX_TX_NUM                 = 0;
static int		HX_BT_NUM                 = 0;
static int		HX_X_RES                  = 0;
static int		HX_Y_RES                  = 0;
static int		HX_MAX_PT                 = 0;
static bool		HX_XY_REVERSE             = false;
static bool		HX_INT_IS_EDGE            = false;
static bool		HX_NEED_UPDATE_FW            = false;

static unsigned int	FW_VER_MAJ_FLASH_ADDR;
static unsigned int 	FW_VER_MAJ_FLASH_LENG;
static unsigned int 	FW_VER_MIN_FLASH_ADDR;
static unsigned int 	FW_VER_MIN_FLASH_LENG;
static unsigned int 	FW_CFG_VER_FLASH_ADDR;
static unsigned int 	CFG_VER_MAJ_FLASH_ADDR;
static unsigned int 	CFG_VER_MAJ_FLASH_LENG;
static unsigned int 	CFG_VER_MIN_FLASH_ADDR;
static unsigned int 	CFG_VER_MIN_FLASH_LENG;

static uint8_t 	vk_press = 0x00;
static uint8_t 	AA_press = 0x00;
static uint8_t	IC_STATUS_CHECK	= 0xAA;
static uint8_t 	EN_NoiseFilter = 0x00;
static uint8_t	Last_EN_NoiseFilter = 0x00;
static uint8_t  HX_DRIVER_PROBE_Fial = 0;
static int	hx_point_num	= 0;// for himax_ts_work_cap_func use
static int	p_point_num	= 0xFFFF;
static int	tpd_key	   	= 0x00;
static int	tpd_key_old	= 0x00;
static uint8_t mfg_flag = 1;	//Default ON!!!
static int iref_number = 11;
static bool iref_found = false;
static int self_test_inter_flag = 0;
static unsigned int tamper_flag = 0;
static uint8_t g_cap_fw_file_cfg_ver = 0x00;
static int retry_probe = 0;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
#if defined(HTC_FEATURE)
static struct kobject *android_touch_kobj = NULL;// Sys kobject variable
#endif
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

static void himax_int_enable_cap(int irqnum, int enable, int logprint);
static void himax_rst_gpio_set_cap(int pinnum, uint8_t value);
static int8_t himax_int_gpio_read_cap(int pinnum);
static int hx_sensitivity_setup(struct himax_ts_data *cs, uint8_t level);
static void hx_sensor_on(bool enable);

#ifdef CONFIG_OF
MODULE_DEVICE_TABLE(of, himax_match_table);
static struct of_device_id himax_match_table[] = {
	{.compatible = "himax,852xes_cap" },
	{},
};

#else
#define himax_match_table NULL
#endif

static char *HMX_FW_NAME = "cs_HMX.img";
static int irq_enable_count = 0;
static DEFINE_MUTEX(hx_wr_access);
#ifdef MTK
static u8 *gpDMABuf_va = NULL;
static u8 *gpDMABuf_pa = NULL;
#endif
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
		  __func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	uint8_t buf[256];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	memcpy(buf + 1, data, length);

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
		  __func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
		  __func__, toRetry);
		return -EIO;
	}
	return 0;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	uint8_t buf[256];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	memcpy(buf, data, length);

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
		  __func__, toRetry);
		return -EIO;
	}
	return 0;
}

static int himax_hand_shaking(void)    //0:Running, 1:Stop, 2:I2C Fail
{
	int ret, result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];

	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
	memset(hw_reset_check_2, 0x00, sizeof(hw_reset_check_2));

	buf0[0] = 0xF2;
	if (IC_STATUS_CHECK == 0xAA) {
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	} else {
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}

	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:write 0xF2 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(50);

	buf0[0] = 0xF2;
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:write 0x92 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(2);

	ret = i2c_himax_read(private_ts->client, 0x90, hw_reset_check, 1, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:i2c_himax_read 0x90 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}

	if ((IC_STATUS_CHECK != hw_reset_check[0])) {
		msleep(2);
		ret = i2c_himax_read(private_ts->client, 0x90, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("[Himax]:i2c_himax_read 0x90 failed line: %d \n",__LINE__);
			goto work_func_send_i2c_msg_fail;
		}

		if (hw_reset_check[0] == hw_reset_check_2[0]) {
			result = 1;
		} else {
			result = 0;
		}
	} else {
		result = 0;
	}

	return result;

work_func_send_i2c_msg_fail:
	return 2;
}

static int himax_ManualMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if ( i2c_himax_write(private_ts->client, 0x42 ,&cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_FlashMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(int enable)
{
	uint8_t cmd[5];

	if (i2c_himax_write(private_ts->client, 0xAA ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
			return 0;
	}

	/* lock sequence start */
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (enable!=0) {
		cmd[0] = 0x63;cmd[1] = 0x02;cmd[2] = 0x70;cmd[3] = 0x03;
	} else {
		cmd[0] = 0x63;cmd[1] = 0x02;cmd[2] = 0x30;cmd[3] = 0x00;
	}

	if (i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(private_ts->client, 0x4A, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(50);

	if (i2c_himax_write(private_ts->client, 0xA9 ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	return 0;
	/* lock sequence stop */
}


static void himax_changeIref(int selected_iref){

	unsigned char temp_iref[16][2] = {{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
					{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
					{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00},
					{0x00,0x00},{0x00,0x00},{0x00,0x00},{0x00,0x00}};
	uint8_t cmd[10];
	int i = 0;
	int j = 0;

	I("%s: start to check iref,iref number = %d\n",__func__,selected_iref);

	if (i2c_himax_write(private_ts->client, 0xAA ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}

	for(i=0; i<16; i++){
		for(j=0; j<2; j++){
			if(selected_iref == 1){
				temp_iref[i][j] = E_IrefTable_1[i][j];
			}
					else if(selected_iref == 2){
				temp_iref[i][j] = E_IrefTable_2[i][j];
			}
			else if(selected_iref == 3){
				temp_iref[i][j] = E_IrefTable_3[i][j];
			}
			else if(selected_iref == 4){
				temp_iref[i][j] = E_IrefTable_4[i][j];
			}
			else if(selected_iref == 5){
				temp_iref[i][j] = E_IrefTable_5[i][j];
			}
			else if(selected_iref == 6){
				temp_iref[i][j] = E_IrefTable_6[i][j];
			}
			else if(selected_iref == 7){
				temp_iref[i][j] = E_IrefTable_7[i][j];
			}
		}
	}

	if(!iref_found){
		//Read Iref
		//Register 0x43
		cmd[0] = 0x01;
		cmd[1] = 0x00;
		cmd[2] = 0x0A;
		if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return;
		}

		//Register 0x44
		cmd[0] = 0x00;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return ;
		}

		//Register 0x46
		if( i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return ;
		}

		//Register 0x59
		if( i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0){
			E("%s: i2c access fail!\n", __func__);
			return ;
		}

		//find iref group , default is iref 3
		for (i = 0; i < 16; i++){
			if ((cmd[0] == temp_iref[i][0]) &&
					(cmd[1] == temp_iref[i][1])){
				iref_number = i;
				iref_found = true;
				break;
			}
		}

		if(!iref_found ){
			E("%s: Can't find iref number!\n", __func__);
			return ;
		}
		else{
			I("%s: iref_number=%d, cmd[0]=0x%x, cmd[1]=0x%x\n", __func__, iref_number, cmd[0], cmd[1]);
		}
	}

	msleep(5);

	//iref write
	//Register 0x43
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Register 0x44
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Register 0x45
	cmd[0] = temp_iref[iref_number][0];
	cmd[1] = temp_iref[iref_number][1];
	cmd[2] = 0x17;
	cmd[3] = 0x28;

	if( i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Register 0x4A
	if( i2c_himax_write(private_ts->client, 0x4A ,&cmd[0], 0, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Read SFR to check the result
	//Register 0x43
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x0A;
	if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Register 0x44
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Register 0x46
	if( i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	//Register 0x59
	if( i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0){
		E("%s: i2c access fail!\n", __func__);
		return ;
	}

	I( "%s:cmd[0]=%d,cmd[1]=%d,temp_iref_1=%d,temp_iref_2=%d\n",__func__, cmd[0], cmd[1], temp_iref[iref_number][0], temp_iref[iref_number][1]);

	if(cmd[0] != temp_iref[iref_number][0] || cmd[1] != temp_iref[iref_number][1]){
		E("%s: IREF Read Back is not match.\n", __func__);
		E("%s: Iref [0]=%d,[1]=%d\n", __func__,cmd[0],cmd[1]);
	}
	else{
		I("%s: IREF Pass",__func__);
	}

	if (i2c_himax_write(private_ts->client, 0xA9 ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}

}

static uint8_t himax_calculateChecksum(bool change_iref)
{

	int iref_flag = 0;
	uint8_t cmd[10];

	memset(cmd, 0x00, sizeof(cmd));

	//Sleep out
	if( i2c_himax_write(private_ts->client, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(120);

	while(true) {
		if(change_iref) {
			if(iref_flag == 0) {
				himax_changeIref(2); //iref 2
			} else if(iref_flag == 1) {
				himax_changeIref(5); //iref 5
			} else if(iref_flag == 2) {
				himax_changeIref(1); //iref 1
			} else {
				goto CHECK_FAIL;
			}
			iref_flag ++;
		}

		cmd[0] = 0x00;
		cmd[1] = 0x04;
		cmd[2] = 0x0A;
		cmd[3] = 0x02;

		if (i2c_himax_write(private_ts->client, 0xED ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		//Enable Flash
		cmd[0] = 0x01;
		cmd[1] = 0x00;
		cmd[2] = 0x02;

		if (i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		cmd[0] = 0x05;
		if (i2c_himax_write(private_ts->client, 0xD2 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;
		if (i2c_himax_write(private_ts->client, 0x53 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		msleep(200);

		if (i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return -1;
		}

		I("%s 0xAD[0,1,2,3] = %d,%d,%d,%d \n",__func__,cmd[0],cmd[1],cmd[2],cmd[3]);

		if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 ) {
			himax_FlashMode(0);
			goto CHECK_PASS;
		} else {
			himax_FlashMode(0);
			goto CHECK_FAIL;
		}

		CHECK_PASS:
			if(change_iref)
			{
				if(iref_flag < 3){
					continue;
				}
				else {
					return 1;
				}
			}
			else
			{
				return 1;
			}

		CHECK_FAIL:
			return 0;
	}
	return 0;
}

int fts_ctpm_fw_upgrade_with_fs_cap(unsigned char *fw, int len, bool change_iref)
{
	unsigned char* ImageBuffer = fw;
	int fullFileLength = len;
	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	//Try 3 Times
	for (j = 0; j < 3; j++)
	{
		FileLength = fullFileLength;

		if ( i2c_himax_write(private_ts->client, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		msleep(120);

		himax_lock_flash(0);

		cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
		if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if ( i2c_himax_write(private_ts->client, 0x4F ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		msleep(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++)
		{
			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
			{
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if ( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0)
			{
				prePage = cmd[1];
				cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);
			if ( i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (last_byte == 1)
			{
				cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x05;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x00;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				msleep(10);
				if (i == (FileLength - 1))
				{
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(change_iref);//
					//himax_ManualMode(0);
					himax_lock_flash(1);

					if (checksumResult) //Success
					{
						return 1;
					}
					else //Fail
					{
						E("%s: checksumResult fail!\n", __func__);
						//return 0;
					}
				}
			}
		}
	}
	return 0;
}

int fts_ctpm_fw_upgrade_with_sys_fs_cap(unsigned char *fw, int len, bool change_iref)
{
	unsigned char* ImageBuffer = fw;
	int fullFileLength = len;
	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	I("%s: ++\n", __func__);

	//Try 3 Times
	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength;

		if ( i2c_himax_write(private_ts->client, 0x81 , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		msleep(120);

		himax_lock_flash(0);

		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if ( i2c_himax_write(private_ts->client, 0x4F , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		msleep(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) {
			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1) {
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if ( i2c_himax_write(private_ts->client, 0x44 , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];
				cmd[0] = 0x01;
				cmd[1] = 0x09;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x0D;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x09;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &ImageBuffer[4 * i], 4);
			if ( i2c_himax_write(private_ts->client, 0x45 , &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x0D;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x09;//cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (last_byte == 1) {
				cmd[0] = 0x01;
				cmd[1] = 0x01;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x05;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x01;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x00;//cmd[2] = 0x02;
				if ( i2c_himax_write(private_ts->client, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				msleep(10);
				if (i == (FileLength - 1)) {
#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
					touch_fw_update_progress(90);
#endif
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(change_iref);//
					//himax_ManualMode(0);
					himax_lock_flash(1);

					if (checksumResult) { //Success
						return 1;
					} else { //Fail
						E("%s: checksumResult fail!\n", __func__);
						return 0;
					}
				}
			}
#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
			if(i == (3 * FileLength / 4))
				touch_fw_update_progress(75);
			else if(i == (2 * FileLength / 4))
				touch_fw_update_progress(45);
			else if(i == (1 * FileLength / 4))
				touch_fw_update_progress(20);
#endif
		}
	}
	return 0;
}

#include <linux/async.h>
#include <linux/wakelock.h>
#include <linux/string.h>
#include <linux/kthread.h>

static void himax_read_TP_info(struct i2c_client *client);

int himax_touch_fw_update_cap(struct firmware *vfw)
{
	struct himax_ts_data *ts = private_ts;
	int ret = 0, i = 0, tagLen = 0;
	char tag[40];
	const char name[11] = "HIMAX8527E\0";
	char *index = NULL;
	char file_ver_char_maj[3] = {0};
	char file_ver_char_min[3] = {0};
	char file_cfg_char_ver[3] = {0};
	uint8_t file_ver_maj = 0x00;
	uint8_t file_ver_min = 0x00;
	uint8_t file_cfg_ver = 0x00;
	int FileLength;
	int FirmwareUpdate_Count = 0;
	struct wake_lock flash_wake_lock;

	ts->fw = vfw;
	if(ts->fw == NULL) {
		I("[FW] No firmware file, no firmware update\n");
		goto HMX_FW_REQUEST_FAILURE;
	}
#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
	touch_fw_update_progress(0);
#endif

	I("[FW] chip fw version = 0x%02X,%02X,CFG:%02X\n", ts->vendor_fw_ver_H, ts->vendor_fw_ver_L, ts->vendor_config_ver);
	//Check fw header
	if (vfw->data[0] == 'C' && vfw->data[1] == 'S') {
		while ((tag[i] = vfw->data[i]) != '\n')
			i++;
		tag[i] = '\0';
		tagLen = i + 1;
		index = strstr(tag, name);
		I("[FW] image fw tag = %s\n", tag);

		if (index != NULL) {
			/*check fw file contain '_' for version checking */
			if (*(index + strlen(name)) == '_') {
				/* Convert two char after '_' to fw file major version */
				memcpy(file_ver_char_maj, index + strlen(name) + 1, 2);

				if (kstrtou8(file_ver_char_maj, 16, &file_ver_maj) != 0) {
					I("[FW] cannot get major version. Update Bypass\n");
					goto HMX_FW_TAG_FORMAT_WRONG;
				}

				if (*(index + strlen(name) + 1 + 2) == ',') {
					/* Convert two char after ',' to fw file minor version */
					memcpy(file_ver_char_min, index + strlen(name)
					       + 1/* length of '_' */
					       + 2/* length of major version */
					       + 1/* length of ',' */,
					       2  /* length of minor version */);
					if (kstrtou8(file_ver_char_min, 16, &file_ver_min) != 0) {
						I("[FW] cannot get minor version. Update Bypass\n");
						goto HMX_FW_TAG_FORMAT_WRONG;
					}
				}

				if (file_ver_maj == ts->vendor_fw_ver_H &&
				    file_ver_min == ts->vendor_fw_ver_L) {
					if (*(index + strlen(name) + 1 + 2 + 1 + 2) == ',') {
						/* Convert two char after ',' to fw file config version */
						memcpy(file_cfg_char_ver, index + strlen(name)
							+ 1/* length of '_' */
							+ 2/* length of major version */
							+ 1/* length of ',' */
							+ 2/* length of minor version */
							+ 1/* length of ',' */,
							2  /* length of config version */);
						if (kstrtou8(file_cfg_char_ver, 16, &file_cfg_ver) != 0) {
							I("[FW] cannot get config version. Update Bypass\n");
							goto HMX_FW_TAG_FORMAT_WRONG;
						}
					} else {
						// old firmware tag in firmware file. set cfg ver = ts->vendor_config_ver
						file_cfg_ver = ts->vendor_config_ver;
					}
				}
				g_cap_fw_file_cfg_ver = file_cfg_ver;

				I("[FW] image file tag version: 0x%02X,%02X,CFG:%02X\n", file_ver_maj, file_ver_min, file_cfg_ver);
				if (file_ver_maj != ts->vendor_fw_ver_H ||
				    file_ver_min != ts->vendor_fw_ver_L ||
				    file_cfg_ver != ts->vendor_config_ver) {
					I("[FW] FW version/config difference. Need Update\n");
				} else if (HX_NEED_UPDATE_FW) {
					I("[FW] FW wrong checksum. Need Update\n");
				} else {
					I("[FW] Update Bypass\n");
					goto HMX_FW_SAME_VER_IGNORE;
				}
			} else {
				I("[FW] incorrect tag format. Update Bypass\n");
				goto HMX_FW_TAG_FORMAT_WRONG;
			}
		} else {
			I("[FW] pattern %s not found. Update Bypass\n", name);
			goto HMX_FW_TAG_FORMAT_WRONG;
		}
	} else
		I("[FW] No tag, Force Update\n");
	ts->fw_data_start = (u8*)(vfw->data + tagLen);
	ts->fw_size = vfw->size - tagLen;

//#ifdef HX_RST_PIN_FUNC
	//Do HW reset first
//	himax_HW_reset_cap(true, ts->use_irq);
//#endif

	atomic_set(&ts->in_flash, 1);
	if(ts->use_irq)
		himax_int_enable_cap(private_ts->client->irq, 0, true);
	else {
		hrtimer_cancel(&ts->timer);
		cancel_work_sync(&ts->work);
	}
	wake_lock_init(&flash_wake_lock, WAKE_LOCK_SUSPEND, ts->client->name);
	wake_lock(&flash_wake_lock);
	if(wake_lock_active(&flash_wake_lock))
		I("[FW] wake lock successfully acquired!\n");
	else {
		E("[FW] failed to hold wake lock, give up....\n");
		goto WAKE_LOCK_ACQUIRE_FAILED;
	}

	FileLength = ts->fw_size;
#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
	touch_fw_update_progress(10);
#endif
doFirmwareUpdate_Retry:
#ifdef HX_RST_PIN_FUNC
	//Do HW reset first
	himax_HW_reset_cap(false, true);
#endif
	if(fts_ctpm_fw_upgrade_with_sys_fs_cap(ts->fw_data_start, FileLength, true) == 0) {
		if(FirmwareUpdate_Count < 3) {
			FirmwareUpdate_Count++;
			I("%s: TP upgrade Error, Count: %d\n", __func__, FirmwareUpdate_Count);
			goto doFirmwareUpdate_Retry;
		}
		E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
		fw_update_complete = false;
#endif
		ret = -1;	/* Fail */
	} else {
		I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
		fw_update_complete = true;
#endif
	}

#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
	touch_fw_update_progress(95);
#endif
#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true, true);
#endif

	wake_unlock(&flash_wake_lock);
	wake_lock_destroy(&flash_wake_lock);
	atomic_set(&ts->in_flash, 0);

	hx_sensor_on(false);
	I("%s: himax read CAP info\n", __func__);
	himax_read_TP_info(private_ts->client);
	hx_sensor_on(true);

	if(ts->use_irq)
		himax_int_enable_cap(private_ts->client->irq, 1, true);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	I("[FW] firmware flash process complete!\n");
#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
	touch_fw_update_progress(100);
#endif

	return ret;
WAKE_LOCK_ACQUIRE_FAILED:
	wake_lock_destroy(&flash_wake_lock);
	if(ts->use_irq)
		himax_int_enable_cap(private_ts->client->irq, 1, true);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	atomic_set(&ts->in_flash, 0);
HMX_FW_SAME_VER_IGNORE:
HMX_FW_TAG_FORMAT_WRONG:
	I("%s: HMX_FW_SAME_VER_IGNORE/HMX_FW_TAG_FORMAT_WRONG, do himax sesnor on\n", __func__);
	hx_sensor_on(true);
	return 1;	/* Bypass */
HMX_FW_REQUEST_FAILURE:
	I("%s: HMX_FW_REQUEST_FAILURE, do himax sesnor on\n", __func__);
	hx_sensor_on(true);
	return -1;	/* Fail */
}

static void updateFirmware(struct work_struct *work)
{
	int ret = 0, i;
	struct himax_ts_data *ts = private_ts;

	if (himax_calculateChecksum(false) == 0) {
		HX_NEED_UPDATE_FW = true;
		E("%s: wrong checksum\n", __func__);
	}
	// Request firmware for update
	I("[FW] Requesting firmware.....\n");
	for (i = 1; i < 4; i++) {
		I("[FW] enter for loop, i = %d\n", i);
		ret = request_firmware(&ts->fw, HMX_FW_NAME, &ts->client->dev);
		if (ret == 0) {
			D("[FW] success, i= %d, ret = %d \n", i, ret);
			break;
		} else {
			D("[FW] trial #%d, Request_firmware failed, ret = %d\n", i, ret);
			continue;
		}
	}

	if (ret == 0) {
		himax_touch_fw_update_cap((struct firmware *)ts->fw);
	} else {
		I("%s: NO enter update FW,  himax sesnor on\n", __func__);
		hx_sensor_on(true);
	}
}

#ifdef CONFIG_TOUCHSCREEN_HIMAX_CAPSENSOR_FW_UPDATE
int register_himax_touch_fw_update(void)
{
	himax_tp_notifier.fwupdate = himax_touch_fw_update_cap;
	himax_tp_notifier.flash_timeout = 200;
	scnprintf(himax_tp_notifier.fw_vendor, sizeof(himax_tp_notifier.fw_vendor), "%s", "HIMAX");
	scnprintf(himax_tp_notifier.fw_ver, sizeof(himax_tp_notifier.fw_ver), "0x%02X", private_ts->vendor_fw_ver_H);
	return register_fw_update(&himax_tp_notifier);
}
#endif

static int himax_input_register(struct himax_ts_data *ts)
{
	int ret;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		E("%s: Failed to allocate input device\n", __func__);
		return ret;
	}
	ts->input_dev->name = INPUT_DEV_NAME;

	set_bit(EV_SYN, ts->input_dev->evbit);
///	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
///	set_bit(KEY_MENU, ts->input_dev->keybit);
///	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#if defined(HX_SMART_WAKEUP)||defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
	set_bit(KEY_CUST_01, ts->input_dev->keybit);
	set_bit(KEY_CUST_02, ts->input_dev->keybit);
	set_bit(KEY_CUST_03, ts->input_dev->keybit);
	set_bit(KEY_CUST_04, ts->input_dev->keybit);
	set_bit(KEY_CUST_05, ts->input_dev->keybit);
	set_bit(KEY_CUST_06, ts->input_dev->keybit);
	set_bit(KEY_CUST_07, ts->input_dev->keybit);
	set_bit(KEY_CUST_08, ts->input_dev->keybit);
	set_bit(KEY_CUST_09, ts->input_dev->keybit);
	set_bit(KEY_CUST_10, ts->input_dev->keybit);
	set_bit(KEY_CUST_11, ts->input_dev->keybit);
	set_bit(KEY_CUST_12, ts->input_dev->keybit);
	set_bit(KEY_CUST_13, ts->input_dev->keybit);
	set_bit(KEY_CUST_14, ts->input_dev->keybit);
	set_bit(KEY_CUST_15, ts->input_dev->keybit);
#endif	
/*	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	set_bit(KEY_MENU, ts->input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		//ts->input_dev->mtsize = ts->nFinger_support;
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
		0, 3, 0, 0);
	} else {// PROTOCOL_TYPE_B
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		input_mt_init_slots(ts->input_dev, ts->nFinger_support,0);
	}

	I("input_set_abs_params: min_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);

//	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);
*/
	return input_register_device(ts->input_dev);
}

static void calcDataSize(uint8_t finger_num)
{
	struct himax_ts_data *ts_data = private_ts;
	ts_data->coord_data_size = 4 * finger_num;
	ts_data->area_data_size = ((finger_num / 4) + (finger_num % 4 ? 1 : 0)) * 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;
	ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
																					ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
															(((uint32_t)ts_data->x_channel * ts_data->y_channel +
		  																		ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size)? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4 ;

	if ( (HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4 ;
	else
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) +1) * 4 ;
}
void himax_touch_information_cap(void)
{
	char data[12] = {0};

	I("%s:IC_TYPE =%d\n", __func__,IC_TYPE);

	if(IC_TYPE == HX_85XX_ES_SERIES_PWON)
		{
	  		data[0] = 0x8C;
	  		data[1] = 0x14;
		  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		  	msleep(10);
		  	data[0] = 0x8B;
		  	data[1] = 0x00;
		  	data[2] = 0x70;
		  	i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
		  	msleep(10);
		  	i2c_himax_read(private_ts->client, 0x5A, data, 12, DEFAULT_RETRY_CNT);
			HX_RX_NUM = 2;				 // FE(70)
			HX_TX_NUM = 1;				 // FE(71)
			HX_MAX_PT = 2;				 // FE(72)
#ifdef HX_EN_SEL_BUTTON
		  	HX_BT_NUM = (data[2] & 0x0F); //FE(72)
#endif
		  	if((data[4] & 0x04) == 0x04) {//FE(74)
				HX_XY_REVERSE = true;
			HX_Y_RES = data[6]*256 + data[7]; //FE(76),FE(77)
			HX_X_RES = data[8]*256 + data[9]; //FE(78),FE(79)
		  	} else {
			HX_XY_REVERSE = false;
			HX_X_RES = data[6]*256 + data[7]; //FE(76),FE(77)
			HX_Y_RES = data[8]*256 + data[9]; //FE(78),FE(79)
		  	}
		  	data[0] = 0x8C;
		  	data[1] = 0x00;
		  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		  	msleep(10);
#ifdef HX_EN_MUT_BUTTON
			data[0] = 0x8C;
		  	data[1] = 0x14;
		  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		  	msleep(10);
		  	data[0] = 0x8B;
		  	data[1] = 0x00;
		  	data[2] = 0x64;
		  	i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
		  	msleep(10);
		  	i2c_himax_read(private_ts->client, 0x5A, data, 4, DEFAULT_RETRY_CNT);
		  	HX_BT_NUM = (data[0] & 0x03);
		  	data[0] = 0x8C;
		  	data[1] = 0x00;
		  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		  	msleep(10);
#endif
#if defined(HX_TP_PROC_2T2R) || defined (HX_TP_SYS_2T2R)
			data[0] = 0x8C;
			data[1] = 0x14;
			i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
			msleep(10);

			data[0] = 0x8B;
			data[1] = 0x00;
			data[2] = HX_2T2R_Addr;
			i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(10);

			i2c_himax_read(private_ts->client, 0x5A, data, 10, DEFAULT_RETRY_CNT);

			HX_RX_NUM_2 = data[0];
			HX_TX_NUM_2 = data[1];

			I("%s:Touch Panel Type=%d \n", __func__,data[2]);
			if(data[2]==HX_2T2R_en_setting)//2T2R type panel
				Is_2T2R = true;
			else
				Is_2T2R = false;

			data[0] = 0x8C;
			data[1] = 0x00;
			i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
			msleep(10);
#endif
		  	data[0] = 0x8C;
		  	data[1] = 0x14;
		  	i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
		  	msleep(10);
		  	data[0] = 0x8B;
		  	data[1] = 0x00;
		  	data[2] = 0x02;
		  	i2c_himax_master_write(private_ts->client, &data[0],3,DEFAULT_RETRY_CNT);
		  	msleep(10);
		  	i2c_himax_read(private_ts->client, 0x5A, data, 10, DEFAULT_RETRY_CNT);
		  	data[0] = 0x8C;
			data[1] = 0x00;
			i2c_himax_master_write(private_ts->client, &data[0],2,DEFAULT_RETRY_CNT);
			msleep(10);
			I("%s:HX_RX_NUM =%d,HX_TX_NUM =%d,HX_MAX_PT=%d \n", __func__,HX_RX_NUM,HX_TX_NUM,HX_MAX_PT);

			if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, data, 1, 3) < 0) {
				E("%s: i2c access fail!\n", __func__);
			}
			private_ts->vendor_config_ver = data[0];
			I("config_ver=%x.\n",private_ts->vendor_config_ver);
		}
		else
		{
			HX_RX_NUM				= 0;
			HX_TX_NUM				= 0;
			HX_BT_NUM				= 0;
			HX_X_RES				= 0;
			HX_Y_RES				= 0;
			HX_MAX_PT				= 0;
			HX_XY_REVERSE			= false;
			HX_INT_IS_EDGE			= false;
		}
}
static uint8_t himax_read_Sensor_ID(struct i2c_client *client)
{
	uint8_t val_high[1], val_low[1], ID0=0, ID1=0;
	uint8_t sensor_id;
	char data[3];

	data[0] = 0x56; data[1] = 0x02; data[2] = 0x02;/*ID pin PULL High*/
	i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
	msleep(1);

	//read id pin high
	i2c_himax_read(client, 0x57, val_high, 1, DEFAULT_RETRY_CNT);

	data[0] = 0x56; data[1] = 0x01; data[2] = 0x01;/*ID pin PULL Low*/
	i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
	msleep(1);

	//read id pin low
	i2c_himax_read(client, 0x57, val_low, 1, DEFAULT_RETRY_CNT);

	if((val_high[0] & 0x01) ==0)
		ID0=0x02;/*GND*/
	else if((val_low[0] & 0x01) ==0)
		ID0=0x01;/*Floating*/
	else
		ID0=0x04;/*VCC*/

	if((val_high[0] & 0x02) ==0)
		ID1=0x02;/*GND*/
	else if((val_low[0] & 0x02) ==0)
		ID1=0x01;/*Floating*/
	else
		ID1=0x04;/*VCC*/
	if((ID0==0x04)&&(ID1!=0x04))
		{
			data[0] = 0x56; data[1] = 0x02; data[2] = 0x01;/*ID pin PULL High,Low*/
			i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(1);

		}
	else if((ID0!=0x04)&&(ID1==0x04))
		{
			data[0] = 0x56; data[1] = 0x01; data[2] = 0x02;/*ID pin PULL Low,High*/
			i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(1);

		}
	else if((ID0==0x04)&&(ID1==0x04))
		{
			data[0] = 0x56; data[1] = 0x02; data[2] = 0x02;/*ID pin PULL High,High*/
			i2c_himax_master_write(client, &data[0],3,DEFAULT_RETRY_CNT);
			msleep(1);

		}
	sensor_id=(ID1<<4)|ID0;

	data[0] = 0xF3; data[1] = sensor_id;
	i2c_himax_master_write(client, &data[0],2,DEFAULT_RETRY_CNT);/*Write to MCU*/
	msleep(1);

	return sensor_id;

}
static int himax_power_on_initCMD(struct i2c_client *client)
{
	int retry_cnt = 0;
	char data[12] = {0};

	I("%s:\n", __func__);
retry_power_init:
	//Sense on to update the information
	hx_sensor_on(true);
	hx_sensor_on(false);
	himax_touch_information_cap();
	//Check R36 to check IC Status
	i2c_himax_read(client, 0x36, data, 2, 10);

	//Check R36 to check IC Status
	if(data[0] != 0x0F || data[1] != 0x53) {
		//IC is abnormal
		E("[HimaxError] %s R36 Fail : R36[0]=%d, R36[1]=%d\n", __func__, data[0], data[1]);
		if (retry_cnt++ < 3)
			goto retry_power_init;
		else
			return -1;
	}
	return 0;
}

#ifdef HX_AUTO_UPDATE_FW
static int i_update_FW(void)
{
	unsigned char* ImageBuffer = i_CTPM_FW;
	int fullFileLength = sizeof(i_CTPM_FW);

	I("IMAGE FW_VER=%x,%x.\n",i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR],i_CTPM_FW[FW_VER_MIN_FLASH_ADDR]);
	I("IMAGE CFG_VER=%x.\n",i_CTPM_FW[FW_CFG_VER_FLASH_ADDR]);
	if (( private_ts->vendor_fw_ver_H < i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR] )
		|| ( private_ts->vendor_fw_ver_L < i_CTPM_FW[FW_VER_MIN_FLASH_ADDR] )
		|| ( private_ts->vendor_config_ver < i_CTPM_FW[FW_CFG_VER_FLASH_ADDR] )
		|| ( himax_calculateChecksum(false) == 0 ))
		{
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset_cap(false,true);
#endif
			if(fts_ctpm_fw_upgrade_with_fs_cap(ImageBuffer,fullFileLength,true) == 0)
				E("%s: TP upgrade error\n", __func__);
			else
				{
					I("%s: TP upgrade OK\n", __func__);
					private_ts->vendor_fw_ver_H = i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR];
					private_ts->vendor_fw_ver_L = i_CTPM_FW[FW_VER_MIN_FLASH_ADDR];
					private_ts->vendor_config_ver = i_CTPM_FW[FW_CFG_VER_FLASH_ADDR];
				}
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset_cap(false,true);
#endif
			return 1;
		}
	else
		return 0;
}
#endif

#ifdef HX_AUTO_UPDATE_CONFIG
int fts_ctpm_fw_upgrade_with_i_file_flash_cfg(struct himax_config *cfg)
{
	unsigned char* ImageBuffer= i_CTPM_FW;//NULL ;//
	int fullFileLength = FLASH_SIZE;

	int i, j, k;
	uint8_t cmd[5], last_byte, prePage;
	uint8_t tmp[5];
	int FileLength;
	int Polynomial = 0x82F63B78;
	int CRC32 = 0xFFFFFFFF;
	int BinDataWord = 0;
	int current_index = 0;
	uint8_t checksumResult = 0;

	I(" %s: flash CONFIG only START!\n", __func__);

#if 0
	//=========Dump FW image from Flash========
	setSysOperation(1);
	setFlashCommand(0x0F);
	setFlashDumpProgress(0);
	setFlashDumpComplete(0);
	setFlashDumpFail(0);
	queue_work(private_ts->flash_wq, &private_ts->flash_work);
	for(i=0 ; i<1024 ; i++)
		{
			if(getFlashDumpComplete())
				{
					ImageBuffer = flash_buffer;
					fullFileLength = FLASH_SIZE;
					I(" %s: Load FW from flash OK! cycle=%d fullFileLength=%x\n", __func__,i,fullFileLength);
					break;
				}
			msleep(5);
		}
	if(i==400)
	{
		E(" %s: Load FW from flash time out fail!\n", __func__);
		return 0;
	}
	//===================================
#endif
	current_index = CFB_START_ADDR + CFB_INFO_LENGTH;

	//Update the Config Part from config array
	for(i=1 ; i<sizeof(cfg->c1)/sizeof(cfg->c1[0]) ; i++) ImageBuffer[current_index++] = cfg->c1[i];
	for(i=1 ; i<sizeof(cfg->c2)/sizeof(cfg->c2[0]) ; i++) ImageBuffer[current_index++] = cfg->c2[i];
	for(i=1 ; i<sizeof(cfg->c3)/sizeof(cfg->c3[0]) ; i++) ImageBuffer[current_index++] = cfg->c3[i];
	for(i=1 ; i<sizeof(cfg->c4)/sizeof(cfg->c4[0]) ; i++) ImageBuffer[current_index++] = cfg->c4[i];
	for(i=1 ; i<sizeof(cfg->c5)/sizeof(cfg->c5[0]) ; i++) ImageBuffer[current_index++] = cfg->c5[i];
	for(i=1 ; i<sizeof(cfg->c6)/sizeof(cfg->c6[0]) ; i++) ImageBuffer[current_index++] = cfg->c6[i];
	for(i=1 ; i<sizeof(cfg->c7)/sizeof(cfg->c7[0]) ; i++) ImageBuffer[current_index++] = cfg->c7[i];
	for(i=1 ; i<sizeof(cfg->c8)/sizeof(cfg->c8[0]) ; i++) ImageBuffer[current_index++] = cfg->c8[i];
	for(i=1 ; i<sizeof(cfg->c9)/sizeof(cfg->c9[0]) ; i++) ImageBuffer[current_index++] = cfg->c9[i];
	for(i=1 ; i<sizeof(cfg->c10)/sizeof(cfg->c10[0]) ; i++) ImageBuffer[current_index++] = cfg->c10[i];
	for(i=1 ; i<sizeof(cfg->c11)/sizeof(cfg->c11[0]) ; i++) ImageBuffer[current_index++] = cfg->c11[i];
	for(i=1 ; i<sizeof(cfg->c12)/sizeof(cfg->c12[0]) ; i++) ImageBuffer[current_index++] = cfg->c12[i];
	for(i=1 ; i<sizeof(cfg->c13)/sizeof(cfg->c13[0]) ; i++) ImageBuffer[current_index++] = cfg->c13[i];
	for(i=1 ; i<sizeof(cfg->c14)/sizeof(cfg->c14[0]) ; i++) ImageBuffer[current_index++] = cfg->c14[i];
	for(i=1 ; i<sizeof(cfg->c15)/sizeof(cfg->c15[0]) ; i++) ImageBuffer[current_index++] = cfg->c15[i];
	for(i=1 ; i<sizeof(cfg->c16)/sizeof(cfg->c16[0]) ; i++) ImageBuffer[current_index++] = cfg->c16[i];
	for(i=1 ; i<sizeof(cfg->c17)/sizeof(cfg->c17[0]) ; i++) ImageBuffer[current_index++] = cfg->c17[i];
	for(i=1 ; i<sizeof(cfg->c18)/sizeof(cfg->c18[0]) ; i++) ImageBuffer[current_index++] = cfg->c18[i];
	for(i=1 ; i<sizeof(cfg->c19)/sizeof(cfg->c19[0]) ; i++) ImageBuffer[current_index++] = cfg->c19[i];
	for(i=1 ; i<sizeof(cfg->c20)/sizeof(cfg->c20[0]) ; i++) ImageBuffer[current_index++] = cfg->c20[i];
	for(i=1 ; i<sizeof(cfg->c21)/sizeof(cfg->c21[0]) ; i++) ImageBuffer[current_index++] = cfg->c21[i];
	for(i=1 ; i<sizeof(cfg->c22)/sizeof(cfg->c22[0]) ; i++) ImageBuffer[current_index++] = cfg->c22[i];
	for(i=1 ; i<sizeof(cfg->c23)/sizeof(cfg->c23[0]) ; i++) ImageBuffer[current_index++] = cfg->c23[i];
	for(i=1 ; i<sizeof(cfg->c24)/sizeof(cfg->c24[0]) ; i++) ImageBuffer[current_index++] = cfg->c24[i];
	for(i=1 ; i<sizeof(cfg->c25)/sizeof(cfg->c25[0]) ; i++) ImageBuffer[current_index++] = cfg->c25[i];
	for(i=1 ; i<sizeof(cfg->c26)/sizeof(cfg->c26[0]) ; i++) ImageBuffer[current_index++] = cfg->c26[i];
	for(i=1 ; i<sizeof(cfg->c27)/sizeof(cfg->c27[0]) ; i++) ImageBuffer[current_index++] = cfg->c27[i];
	for(i=1 ; i<sizeof(cfg->c28)/sizeof(cfg->c28[0]) ; i++) ImageBuffer[current_index++] = cfg->c28[i];
	for(i=1 ; i<sizeof(cfg->c29)/sizeof(cfg->c29[0]) ; i++) ImageBuffer[current_index++] = cfg->c29[i];
	for(i=1 ; i<sizeof(cfg->c30)/sizeof(cfg->c30[0]) ; i++) ImageBuffer[current_index++] = cfg->c30[i];
	for(i=1 ; i<sizeof(cfg->c31)/sizeof(cfg->c31[0]) ; i++) ImageBuffer[current_index++] = cfg->c31[i];
	for(i=1 ; i<sizeof(cfg->c32)/sizeof(cfg->c32[0]) ; i++) ImageBuffer[current_index++] = cfg->c32[i];
	for(i=1 ; i<sizeof(cfg->c33)/sizeof(cfg->c33[0]) ; i++) ImageBuffer[current_index++] = cfg->c33[i];
	for(i=1 ; i<sizeof(cfg->c34)/sizeof(cfg->c34[0]) ; i++) ImageBuffer[current_index++] = cfg->c34[i];
	for(i=1 ; i<sizeof(cfg->c35)/sizeof(cfg->c35[0]) ; i++) ImageBuffer[current_index++] = cfg->c35[i];
	for(i=1 ; i<sizeof(cfg->c36)/sizeof(cfg->c36[0]) ; i++) ImageBuffer[current_index++] = cfg->c36[i];
	for(i=1 ; i<sizeof(cfg->c37)/sizeof(cfg->c37[0]) ; i++) ImageBuffer[current_index++] = cfg->c37[i];
	for(i=1 ; i<sizeof(cfg->c38)/sizeof(cfg->c38[0]) ; i++) ImageBuffer[current_index++] = cfg->c38[i];
	for(i=1 ; i<sizeof(cfg->c39)/sizeof(cfg->c39[0]) ; i++) ImageBuffer[current_index++] = cfg->c39[i];
	current_index=current_index+50;//dummy data
	//I(" %s: current_index=%d\n", __func__,current_index);
	for(i=1 ; i<sizeof(cfg->c40)/sizeof(cfg->c40[0]) ; i++) ImageBuffer[current_index++] = cfg->c40[i];
	for(i=1 ; i<sizeof(cfg->c41)/sizeof(cfg->c41[0]) ; i++) ImageBuffer[current_index++] = cfg->c41[i];

	//cal_checksum start//
	FileLength = fullFileLength;
	FileLength = (FileLength + 3) / 4;
	for (i = 0; i < FileLength; i++)
	{
		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
		if (i < (FileLength - 1))/*cal_checksum*/
		{
			for(k = 0; k < 4; k++)
			{
				BinDataWord |= cmd[k] << (k * 8);
			}

			CRC32 = BinDataWord ^ CRC32;
			for(k = 0; k < 32; k++)
			{
				if((CRC32 % 2) != 0)
				{
					CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF) ^ Polynomial;
				}
				else
				{
					CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF);
				}
			}
			BinDataWord = 0;
		}
	}
	//cal_checksum end//

	//Try 3 Times
	for (j = 0; j < 3; j++)
	{
		FileLength = fullFileLength;

		if( i2c_himax_write(private_ts->client, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			E(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		msleep(120);

		himax_lock_flash(0);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++)
		{
			last_byte = 0;

			if(i<0x20)
				continue;
			else if((i>0xBF)&&(i<(FileLength-0x20)))
				continue;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
			{
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;

			if( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			//I(" %s: CMD 44 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
			if (prePage != cmd[1] || i == 0x20)
			{
				prePage = cmd[1];
				//I(" %s: %d page erase\n",__func__,prePage);
				tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				tmp[0] = 0x05;tmp[1] = 0x2D;//tmp[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}
				msleep(30);
				tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				if( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				//I(" %s: CMD 44-47 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);

				tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				tmp[0] = 0x01;tmp[1] = 0x0D;//tmp[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}
				tmp[0] = 0x01;tmp[1] = 0x09;//tmp[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&tmp[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				if( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				//I(" %s: CMD 44-45 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
				//I(" %s: Erase Page num=%x\n", __func__,prePage);
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);

			if (i == (FileLength - 1))
			{
				tmp[0]= (CRC32 & 0xFF);
				tmp[1]= ((CRC32 >>8) & 0xFF);
				tmp[2]= ((CRC32 >>16) & 0xFF);
				tmp[3]= ((CRC32 >>24) & 0xFF);

				memcpy(&cmd[0], &tmp[0], 4);
				I("%s last_byte = 1, CRC32= %x, =[0,1,2,3] = %x,%x,%x,%x \n",__func__, CRC32,tmp[0],tmp[1],tmp[2],tmp[3]);
			}

			if( i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}
			//I(" %s: CMD 48 \n", __func__);
			cmd[0] = 0x01;cmd[1] = 0x0D;//cmd[2] = 0x02;
			if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;cmd[1] = 0x09;//cmd[2] = 0x02;
			if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				E(" %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (last_byte == 1)
			{
				cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x05;//cmd[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x01;//cmd[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x00;//cmd[2] = 0x02;
				if( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E(" %s: i2c access fail!\n", __func__);
					return 0;
				}

				msleep(10);
				if (i == (FileLength - 1))
				{
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(true);//
					//himax_ManualMode(0);
					himax_lock_flash(1);

					I(" %s: flash CONFIG only END!\n", __func__);
					if (checksumResult) //Success
					{
						return 1;
					}
					else //Fail
					{
						E("%s: checksumResult fail!\n", __func__);
						return 0;
					}
				}
			}
		}
	}
	return 0;
}

static int i_update_FWCFG(struct himax_config *cfg)
{
	I("%s: CHIP CONFG version=%x\n", __func__,private_ts->vendor_config_ver);
	I("%s: LOAD CONFG version=%x\n", __func__,cfg->c40[1]);

	if ( private_ts->vendor_config_ver != cfg->c40[1] )
		{
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset_cap(false,true);
#endif
			if(fts_ctpm_fw_upgrade_with_i_file_flash_cfg(cfg) == 0)
				E("%s: TP upgrade error\n", __func__);
			else
				I("%s: TP upgrade OK\n", __func__);
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset_cap(false,true);
#endif
			return 1;
		}
	else
		return 0;
}

#endif

#ifdef HX_RST_PIN_FUNC
void himax_HW_reset_cap(uint8_t doPowerInit, uint8_t int_off)
{
	struct himax_ts_data *cs = private_ts;
	int ret = 0;

	HW_RESET_ACTIVATE = 1;
do_reset_again:
	if (!int_off) {
		if (cs->use_irq)
			himax_int_enable_cap(cs->client->irq,0,true);
		else {
			hrtimer_cancel(&cs->timer);
			ret = cancel_work_sync(&cs->work);
			}
	}

	I("%s: Now reset the Touch chip.\n", __func__);

	himax_rst_gpio_set_cap(cs->rst_gpio, 0);
	msleep(20);
	himax_rst_gpio_set_cap(cs->rst_gpio, 1);
	msleep(20);

	if (doPowerInit && (himax_power_on_initCMD(cs->client) < 0)) {
		I("%s: \n", __func__);
		goto do_reset_again;
	}

	if (!int_off) {
		if (cs->use_irq)
			himax_int_enable_cap(cs->client->irq,1,true);
		else
			hrtimer_start(&cs->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
}
#endif

#ifdef HX_TP_SYS_DEBUG
static u8 himax_read_FW_ver(bool hw_reset)
{
	uint8_t cmd[3];

	memset(cmd, 0x00, sizeof(cmd));

#ifdef HX_RST_PIN_FUNC
	if (hw_reset) {
		himax_HW_reset_cap(false, false);
	}
#else
	if (ts->use_irq)
		himax_int_enable_cap(private_ts->client->irq, 0);
#endif

	msleep(120);
	if (i2c_himax_read(private_ts->client, HX_VER_FW_MAJ, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_H = cmd[0];
	if (i2c_himax_read(private_ts->client, HX_VER_FW_MIN, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_L = cmd[0];
	I("FW_VER : %d,%d \n", private_ts->vendor_fw_ver_H, private_ts->vendor_fw_ver_L);

	if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_config_ver = cmd[0];
	I("CFG_VER : %d \n", private_ts->vendor_config_ver);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true, false);
	hx_sensor_on(false);
	hx_sensor_on(true);
#else
	if (ts->use_irq)
		himax_int_enable_cap(private_ts->client->irq, 1);
#endif

	return 0;
}
#endif

#ifdef HX_TP_PROC_DEBUG
static u8 himax_read_FW_ver(bool hw_reset)
{
	uint8_t cmd[3];

	himax_int_enable_cap(private_ts->client->irq,0,true);

#ifdef HX_RST_PIN_FUNC
	if (hw_reset) {
		himax_HW_reset_cap(false,true);
	}
#endif

	msleep(120);
	if (i2c_himax_read(private_ts->client, HX_VER_FW_MAJ, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_H = cmd[0];
	if (i2c_himax_read(private_ts->client, HX_VER_FW_MIN, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_L = cmd[0];
	I("FW_VER : %d,%d \n",private_ts->vendor_fw_ver_H,private_ts->vendor_fw_ver_L);

	if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_config_ver = cmd[0];
	I("CFG_VER : %d \n",private_ts->vendor_config_ver);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true,true);
#endif

	himax_int_enable_cap(private_ts->client->irq,1,true);

	return 0;
}

#endif

static bool himax_ic_package_check(struct himax_ts_data *ts)
{
	uint8_t cmd[3];

	memset(cmd, 0x00, sizeof(cmd));

	if (i2c_himax_read(ts->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
		return false ;

	if(cmd[0] == 0x05 && cmd[1] == 0x85 &&
		(cmd[2] == 0x25 || cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28))
		{
			IC_TYPE			= HX_85XX_ES_SERIES_PWON;
			IC_CHECKSUM 		= HX_TP_BIN_CHECKSUM_CRC;
			//Himax: Set FW and CFG Flash Address
			FW_VER_MAJ_FLASH_ADDR   = 133;  //0x0085
			FW_VER_MAJ_FLASH_LENG   = 1;;
			FW_VER_MIN_FLASH_ADDR   = 134;  //0x0086
			FW_VER_MIN_FLASH_LENG   = 1;
			CFG_VER_MAJ_FLASH_ADDR 	= 160;   //0x00A0
			CFG_VER_MAJ_FLASH_LENG 	= 12;
			CFG_VER_MIN_FLASH_ADDR 	= 172;   //0x00AC
			CFG_VER_MIN_FLASH_LENG 	= 12;
			FW_CFG_VER_FLASH_ADDR	= 132;  //0x0084
#ifdef HX_AUTO_UPDATE_CONFIG
			CFB_START_ADDR 			= 0x80;
			CFB_LENGTH				= 638;
			CFB_INFO_LENGTH 		= 68;
#endif
			I("Himax IC package 852x ES: %02X %02X %02X\n", cmd[0], cmd[1], cmd[2]);
		}
		else
		{
			E("Himax IC package incorrect!!PKG[0]=%x,PKG[1]=%x,PKG[2]=%x\n",cmd[0],cmd[1],cmd[2]);
			return false ;
		}
	return true;
}

static void himax_read_TP_info(struct i2c_client *client)
{
	char data[12] = {0};

	//read fw version
	if (i2c_himax_read(client, HX_VER_FW_MAJ, data, 1, 3) < 0) {
		E("%s: i2c access fail! line:%d\n", __func__, __LINE__);
		return;
	}
	private_ts->vendor_fw_ver_H = data[0];

	if (i2c_himax_read(client, HX_VER_FW_MIN, data, 1, 3) < 0) {
		E("%s: i2c access fail! line:%d\n", __func__, __LINE__);
		return;
	}
	private_ts->vendor_fw_ver_L = data[0];
	//read config version
	if (i2c_himax_read(client, HX_VER_FW_CFG, data, 1, 3) < 0) {
		E("%s: i2c access fail! line:%d\n", __func__, __LINE__);
		return;
	}
	private_ts->vendor_config_ver = data[0];//********************************************get firmware_ver*********************

	//read sensor ID
	private_ts->vendor_sensor_id = himax_read_Sensor_ID(client);

	I("sensor_id=%x.\n",private_ts->vendor_sensor_id);
	I("fw_ver=%x,%x.\n",private_ts->vendor_fw_ver_H,private_ts->vendor_fw_ver_L);
	I("config_ver=%x.\n",private_ts->vendor_config_ver);
}

#ifdef HX_ESD_WORKAROUND
void ESD_HW_REST_cap(void)
{
	if (self_test_inter_flag == 1)	{
		I("In self test ,not  TP: ESD - Reset\n");
		return;
	}

	ESD_RESET_ACTIVATE = 1;
	ESD_COUNTER = 0;
	ESD_R36_FAIL = 0;
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT=0;
#endif
	I("START_Himax TP: ESD - Reset\n");
	while(ESD_R36_FAIL <= 3) {
		himax_rst_gpio_set_cap(private_ts->rst_gpio, 0);
		msleep(20);
		himax_rst_gpio_set_cap(private_ts->rst_gpio, 1);
		msleep(20);
		if(himax_power_on_initCMD(private_ts->client) < 0)
			ESD_R36_FAIL++;
		else
			break;
	}

	i2c_himax_write_command(private_ts->client, 0x83, DEFAULT_RETRY_CNT);
	mdelay(30);
	i2c_himax_write_command(private_ts->client, 0x81, DEFAULT_RETRY_CNT);
	mdelay(50);
	himax_int_enable_cap(private_ts->client->irq, 1, true);
	I("END_Himax TP: ESD - Reset\n");
}
#endif
#ifdef HX_CHIP_STATUS_MONITOR
static void himax_chip_monitor_function(struct work_struct *work) //for ESD solution
{
	int ret=0;

	I(" %s: POLLING_COUNT=%x, STATUS=%x \n", __func__,HX_CHIP_POLLING_COUNT,ret);
	if(HX_CHIP_POLLING_COUNT >= (HX_POLLING_TIMES-1))//POLLING TIME
	{
		HX_ON_HAND_SHAKING=1;
		ret = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
		HX_ON_HAND_SHAKING=0;
		if(ret == 2)
		{
			I(" %s: I2C Fail \n", __func__);
			ESD_HW_REST_cap();
		}
		else if(ret == 1)
		{
			I(" %s: MCU Stop \n", __func__);
			ESD_HW_REST_cap();
		}
		HX_CHIP_POLLING_COUNT=0;//clear polling counter
	}
	else
		HX_CHIP_POLLING_COUNT++;

	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);

	return;
}
#endif

#ifdef HX_DOT_VIEW
static void himax_set_cover_func(unsigned int enable)
{
	uint8_t cmd[4];

	if (enable)
		cmd[0] = 0x40;
	else
		cmd[0] = 0x00;
	if ( i2c_himax_write(private_ts->client, 0x8F ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
		E("%s i2c write fail.\n",__func__);

	return;
}

static int hallsensor_hover_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct himax_ts_data *ts = private_ts;

	pole_value = 0x1 & status;
	pole = (0x2 & status) >> 1;
	I("[HL] %s[%s]\n", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");

	if (pole == 1) {
		if (pole_value == 0)
			ts->cover_enable = 0;
		else{
				ts->cover_enable = 1;
			}

		himax_set_cover_func(ts->cover_enable);
		I("[HL] %s: cover_enable = %d.\n", __func__, ts->cover_enable);
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_hover_status_handler_func,
};
#endif

#ifdef HX_SMART_WAKEUP
static void gest_pt_log_coordinate(int rx, int tx)
{
	//driver report x y with range 0 - 255 , we scale it up to x/y pixel
	gest_pt_x[gest_pt_cnt] = rx*HX_X_RES/255;
	gest_pt_y[gest_pt_cnt] = tx*HX_Y_RES/255;
}

static int himax_parse_wake_event(struct himax_ts_data *ts)
{
	uint8_t buf[128];
	unsigned char check_sum_cal = 0;
	int tmp_max_x=0x00,tmp_min_x=0xFFFF,tmp_max_y=0x00,tmp_min_y=0xFFFF;
	int gest_len;
	int i=0, check_FC = 0, gesture_flag = 0;

	if (i2c_himax_read(ts->client, 0x86, buf, 128,DEFAULT_RETRY_CNT))
		E("%s: can't read data from chip!\n", __func__);

	for(i=0;i<GEST_PTLG_ID_LEN;i++)
	{
		if (check_FC==0)
		{
			if((buf[0]!=0x00)&&((buf[0]<=0x0F)||(buf[0]==0x80)))
			{
				check_FC = 1;
				gesture_flag = buf[i];
			}
			else
			{
				check_FC = 0;
				I("ID START at %x , value = %x skip the event\n", i, buf[i]);
				break;
			}
		}
		else
		{
			if(buf[i]!=gesture_flag)
			{
				check_FC = 0;
				I("ID NOT the same %x != %x So STOP parse event\n", buf[i], gesture_flag);
				break;
			}
		}

		I("0x%2.2X ", buf[i]);
		if (i % 8 == 7)
				I("\n");
	}
	I("Himax gesture_flag= %x\n",gesture_flag );
	I("Himax check_FC is %d\n", check_FC);

	if (check_FC == 0)
		return 0;
	if(buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
			buf[GEST_PTLG_ID_LEN+1] != GEST_PTLG_HDR_ID2)
		return 0;
	for(i=0;i<(GEST_PTLG_ID_LEN+GEST_PTLG_HDR_LEN);i++)
	{
		I("P[%x]=0x%2.2X \n", i, buf[i]);
		I("checksum=0x%2.2X \n", check_sum_cal);
		check_sum_cal += buf[i];
	}
	if ((check_sum_cal != 0x00) )
	{
		I(" %s : check_sum_cal: 0x%02X\n",__func__ ,check_sum_cal);
		return 0;
	}

	if(buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
			buf[GEST_PTLG_ID_LEN+1] == GEST_PTLG_HDR_ID2)
	{
		gest_len = buf[GEST_PTLG_ID_LEN+2];
		//parse key point
		gest_pt_cnt = 0;
		I("gest_len = %d \n",gest_len);
		for (i=0; i<3; i++)
			{
				gest_pt_log_coordinate(buf[GEST_PTLG_ID_LEN+4+i*2],buf[GEST_PTLG_ID_LEN+4+i*2+1]);
				gest_key_pt_x[i] = gest_pt_x[i];
				gest_key_pt_y[i] = gest_pt_y[i];
				gest_pt_cnt +=1;
				I("gest_key_pt_x[%d]=%d \n ", i, gest_key_pt_x[i]);
				I("gest_key_pt_y[%d]=%d \n ", i, gest_key_pt_y[i]);
			}
		//parse all point location
		i = 0;
		gest_pt_cnt = 0;
		I("gest doornidate start \n %s",__func__);

		while(i<(gest_len+1)/2)
		{
			gest_pt_log_coordinate(buf[GEST_PTLG_ID_LEN+4+8+i*2],buf[GEST_PTLG_ID_LEN+4+8+i*2+1]);
			i++;

			I("gest_pt_x[%d]=%d \n",gest_pt_cnt,gest_pt_x[gest_pt_cnt]);
			I("gest_pt_y[%d]=%d \n",gest_pt_cnt,gest_pt_y[gest_pt_cnt]);

			gest_pt_cnt +=1;
		}
		if(gest_pt_cnt)
			{
				for(i=0; i<gest_pt_cnt; i++)
					{
						if(tmp_max_x<gest_pt_x[i])
							tmp_max_x=gest_pt_x[i];
						if(tmp_min_x>gest_pt_x[i])
							tmp_min_x=gest_pt_x[i];
						if(tmp_max_y<gest_pt_y[i])
							tmp_max_y=gest_pt_y[i];
						if(tmp_min_y>gest_pt_y[i])
							tmp_min_y=gest_pt_y[i];
					}
				I("gest_point x_min= %d, x_max= %d, y_min= %d, y_max= %d\n",tmp_min_x,tmp_max_x,tmp_min_y,tmp_max_y);
				gest_start_x=gest_pt_x[0];
				gest_start_y=gest_pt_y[0];
				gest_end_x=gest_pt_x[gest_pt_cnt-1];
				gest_end_y=gest_pt_y[gest_pt_cnt-1];
				gest_left_top_x = tmp_min_x;
				gest_left_top_y = tmp_min_y;
				gest_right_down_x = tmp_max_x;
				gest_right_down_y = tmp_max_y;
				I("gest_start_x= %d, gest_start_y= %d, gest_end_x= %d, gest_end_y= %d\n",gest_start_x,gest_start_y,gest_end_x,gest_end_y);
			}

	}

	if(gesture_flag != 0x80)
	{
		if(!ts->gesture_cust_en[gesture_flag])
			{
				I("%s NOT report customer key \n ",__func__);
				return 0;//NOT report customer key
			}
	}
	else
	{
		if(!ts->gesture_cust_en[0])
			{
				I("%s NOT report report double click \n",__func__);
				return 0;//NOT report power key
			}
	}
	ts->gesture_id = gesture_flag;
	if(gesture_flag == 0x80)
		return EV_GESTURE_PWR;
	else
		return gesture_flag;
}

#endif

static void himax_ts_button_func(int tp_key_index,struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;
if ( tp_key_index != 0x00)
	{
		I("virtual key index =%x\n",tp_key_index);
		if ( tp_key_index == 0x01) {
			vk_press = 1;
			I("back key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[0].index) {
						x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
						y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
					}
					if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
//						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
//							100);
//						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
//							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
//						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
//							100);
//						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
//							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_BACK, 1);
		}
		else if ( tp_key_index == 0x02) {
			vk_press = 1;
			I("home key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[1].index) {
						x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
						y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
					}
						if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
//						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
//							100);
//						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
//							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
//						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
//							100);
//						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
//							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_HOME, 1);
		}
		else if ( tp_key_index == 0x04) {
			vk_press = 1;
			I("APP_switch key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[2].index) {
						x_position = (ts->button[2].x_range_min + ts->button[2].x_range_max) / 2;
						y_position = (ts->button[2].y_range_min + ts->button[2].y_range_max) / 2;
					}
						if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
//						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
//							100);
//						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
//							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
//						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
//							100);
//						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
//							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_MENU, 1);
		}
		input_sync(ts->input_dev);
	}
else/*tp_key_index =0x00*/
	{
		I("virtual key released\n");
		vk_press = 0;
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_mt_sync(ts->input_dev);
		}
		else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		input_report_key(ts->input_dev, KEY_MENU, 0);
	input_sync(ts->input_dev);
	}
}

static void recal_work_func(struct work_struct *work)
{
	himax_int_enable_cap(private_ts->client->irq,0,true);
	himax_HW_reset_cap(true, true);
	private_ts->K_counter = 0;
	//set chip to active mode
	i2c_himax_write_command(private_ts->client, 0x83, DEFAULT_RETRY_CNT);
	mdelay(30);
	i2c_himax_write_command(private_ts->client, 0x81, DEFAULT_RETRY_CNT);
	mdelay(50);
	himax_int_enable_cap(private_ts->client->irq, 1, true);
}

static uint8_t vkey_previous[2] = {0x00,0x00};
static void himax_cap_debounce_work_func_back(struct work_struct *work)
{
	struct himax_ts_data *cs = private_ts;
	unsigned long irqflags = 0;

	spin_lock_irqsave(&cs->lock, irqflags);
	I_TIME("KEY_BACK Pressed");
	input_report_key(cs->input_dev,	KEY_BACK, vkey_previous[0]);
	input_sync(cs->input_dev);
	cs->debounced[0] = false;
	spin_unlock_irqrestore(&cs->lock, irqflags);
}
static void himax_cap_debounce_work_func_app(struct work_struct *work)
{
	struct himax_ts_data *cs = private_ts;
	unsigned long irqflags = 0;

	spin_lock_irqsave(&cs->lock, irqflags);
	I_TIME("KEY_APPSELECT Pressed");
	input_report_key(cs->input_dev,	KEY_APPSELECT, vkey_previous[1]);
	input_sync(cs->input_dev);
	cs->debounced[1] = false;
	spin_unlock_irqrestore(&cs->lock, irqflags);
}
inline void himax_ts_work_cap(struct himax_ts_data *ts)
{
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
	int ret = 0;
#endif
	uint8_t buf[128], finger_num, hw_reset_check[2];
	uint16_t finger_pressed;
	uint8_t finger_on = 0;
	int32_t loop_i;
	uint8_t coordInfoSize = ts->coord_data_size + ts->area_data_size + 4;
	unsigned char check_sum_cal = 0;
	int RawDataLen = 0;
	int raw_cnt_max ;
	int raw_cnt_rmd ;
	int hx_touch_info_size;
	int base, x, y, w;
	uint8_t vkey_status;
	const uint16_t vkey_code[] = {KEY_BACK, KEY_APPSELECT};
#ifdef HX_TP_SYS_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int	i;
	int 	mul_num;
	int 	self_num;
	int 	index = 0;
	int	temp1, temp2;
	//coordinate dump start
	char coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 2];
	struct timeval t;
	struct tm broken;
	//coordinate dump end
#endif
#ifdef HX_CHIP_STATUS_MONITOR
	int j = 0;
#endif
	unsigned long irqflags = 0;
	uint16_t mutual_num, width;
	//initial alwayls K WA 's parameters
	struct timespec now;
	memset(&now, 0x00, sizeof(now));

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	if(HX_ON_HAND_SHAKING) { //chip on hand shaking,wait hand shaking
		for(j = 0; j < 100; j++) {
			if(HX_ON_HAND_SHAKING == 0) { //chip on hand shaking end
				I("%s:HX_ON_HAND_SHAKING OK check %d times\n", __func__, j);
				break;
			} else
				msleep(1);
		}
		if(j == 100) {
			E("%s:HX_ON_HAND_SHAKING timeout reject interrupt\n", __func__);
			return;
		}
	}
#endif
	raw_cnt_max = HX_MAX_PT / 4;
	raw_cnt_rmd = HX_MAX_PT % 4;

	if (raw_cnt_rmd != 0x00) { //more than 4 fingers
		RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
		hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 2) * 4;
	} else { //less than 4 fingers
		RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;
		hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 1) * 4;
	}

#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
	if ((diag_cmd) || (ESD_RESET_ACTIVATE) || (HW_RESET_ACTIVATE))
#else
	if ((diag_cmd) || (HW_RESET_ACTIVATE))
#endif
	{
		ret = i2c_himax_read(ts->client, 0x86, buf, 128, DEFAULT_RETRY_CNT);
	} else {
		if(touch_monitor_stop_flag != 0) {
			ret = i2c_himax_read(ts->client, 0x86, buf, 128, DEFAULT_RETRY_CNT);
			touch_monitor_stop_flag-- ;
		} else {
			ret = i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size, DEFAULT_RETRY_CNT);
		}
	}
	if (ret)
#else
	if (i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size, DEFAULT_RETRY_CNT))
#endif
	{
		E("%s: can't read data from chip!\n", __func__);
		goto err_workqueue_out;
	} else {
#ifdef HX_ESD_WORKAROUND
		for(i = 0; i < hx_touch_info_size; i++) {
			if (buf[i] == 0x00) { //case 2 ESD recovery flow-Disable
				check_sum_cal = 1;
			} else if(buf[i] == 0xED) { /*case 1 ESD recovery flow*/
				check_sum_cal = 2;
			} else {
				check_sum_cal = 0;
				i = hx_touch_info_size;
			}
		}

		//IC status is abnormal ,do hand shaking
#ifdef HX_TP_SYS_DIAG
		diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
		if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 && diag_cmd == 0)	//ESD Check
#else
		if (check_sum_cal != 0 && diag_cmd == 0)
#endif
#else
#ifdef HX_ESD_WORKAROUND
		if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0)	//ESD Check
#else
		if (check_sum_cal != 0)
#endif
#endif
		{
			ret = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
			if (ret == 2) {
				goto err_workqueue_out;
			}

			if (ts->suspended) {
				I("In suspend mode, ESD event checked - ALL Zero no need to do reset\n");
				return;
			}
			getnstimeofday(&now);
			if ((ret == 1) && (check_sum_cal == 1)) {
				if ((now.tv_sec - ts->ESD_time_start.tv_sec) > 2 ) {
					ts->ESD_counter = 0;
					ts->ESD_time_start.tv_sec = now.tv_sec;
				} else
					ts->ESD_counter++;

				I("[HIMAX TP MSG]: ESD event checked - ALL Zero. %u\n", ts->ESD_counter);
				if (ts->ESD_counter >= 4) {
					himax_int_enable_cap(ts->client->irq, 0, true);
					ESD_HW_REST_cap();
					ts->ESD_counter = 0;
				}
			} else if (check_sum_cal == 2) {
				if ((now.tv_sec - ts->ESD_time_start.tv_sec) > 2 ) {
					ts->ESD_counter = 0;
					ts->ESD_time_start.tv_sec = now.tv_sec;
				} else
					ts->ESD_counter++;

				I("[HIMAX TP MSG]: ESD event checked - ALL 0xED. %u\n", ts->ESD_counter);
				if (ts->ESD_counter >= 4) {
					himax_int_enable_cap(ts->client->irq, 0, true);
					ESD_HW_REST_cap();
					ts->ESD_counter = 0;
				}
			}

			return;
		} else if (ESD_RESET_ACTIVATE) {
			ESD_RESET_ACTIVATE = 0;/*drop 1st interrupts after chip reset*/
			I("[HIMAX TP MSG]:%s: Back from reset, ready to serve.\n", __func__);
			return;
		} else if (HW_RESET_ACTIVATE)
#else
		if (HW_RESET_ACTIVATE)
#endif
		{
			HW_RESET_ACTIVATE = 0; //drop 1st interrupts after chip reset
			I("[HIMAX TP MSG] : %s : HW_RST Back from reset, ready to serve.\n", __func__);
			return;
		}

		for (loop_i = 0, check_sum_cal = 0; loop_i < hx_touch_info_size; loop_i++)
			check_sum_cal += buf[loop_i];

		if ((check_sum_cal != 0x00) ) {
			I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
			return;
		}
	}

	if (ts->debug_log_level & BIT(0)) {
		I("%s: raw data:\n", __func__);
		for (loop_i = 0; loop_i < hx_touch_info_size; loop_i++) {
			I("0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				I("\n");
		}
	}

	//touch monitor raw data fetch
#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6) {
		//Check 128th byte CRC
		for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++) {
			check_sum_cal += buf[i];
		}
		if (check_sum_cal % 0x100 != 0) {
			goto bypass_checksum_failed_packet;
		}
#ifdef HX_TP_SYS_2T2R
		if(Is_2T2R && diag_cmd == 4) {
			mutual_data = getMutualBuffer_2();
			self_data 	= getSelfBuffer();

			// initiallize the block number of mutual and self
			mul_num = getXChannel_2() * getYChannel_2();

#ifdef HX_EN_SEL_BUTTON
			self_num = getXChannel_2() + getYChannel_2() + HX_BT_NUM;
#else
			self_num = getXChannel_2() + getYChannel_2();
#endif
		} else
#endif
		{
			mutual_data = getMutualBuffer();
			self_data 	= getSelfBuffer();

			// initiallize the block number of mutual and self
			mul_num = getXChannel() * getYChannel();

#ifdef HX_EN_SEL_BUTTON
			self_num = getXChannel() + getYChannel() + HX_BT_NUM;
#else
			self_num = getXChannel() + getYChannel();
#endif
		}

		//Himax: Check Raw-Data Header
		if (buf[hx_touch_info_size] == buf[hx_touch_info_size + 1] && buf[hx_touch_info_size + 1] == buf[hx_touch_info_size + 2]
		    && buf[hx_touch_info_size + 2] == buf[hx_touch_info_size + 3] && buf[hx_touch_info_size] > 0) {
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;
			//I("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
			for (i = 0; i < RawDataLen; i++) {
				temp1 = index + i;

				if (temp1 < mul_num) {
					//mutual
					mutual_data[index + i] = buf[i + hx_touch_info_size + 4];	//4: RawData Header
				} else {
					//self
					temp1 = i + index;
					temp2 = self_num + mul_num;

					if (temp1 >= temp2) {
						break;
					}

					self_data[i + index - mul_num] = buf[i + hx_touch_info_size + 4];	//4: RawData Header
				}
			}
		} else {
			I("[HIMAX TP MSG]%s: header format is wrong!\n", __func__);
		}
	} else if (diag_cmd == 7) {
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	//coordinate dump start
	if (coordinate_dump_enable == 1) {
		for(i = 0; i < (15 + (HX_MAX_PT + 5) * 2 * 5); i++) {
			coordinate_char[i] = 0x20;
		}
		coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 1] = 0xA;
	}
	//coordinate dump end
bypass_checksum_failed_packet:
#endif
	EN_NoiseFilter = (buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 3);
	//I("EN_NoiseFilter=%d\n",EN_NoiseFilter);
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
	//I("EN_NoiseFilter2=%d\n",EN_NoiseFilter);
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if(ts->pdata->proximity_bytp_enable) {
		if ((buf[HX_TOUCH_INFO_POINT_CNT] & 0x0F) == 0x00 && (buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 2 & 0x01)) {
			if(proximity_flag == 0) {
				I(" %s near event trigger\n", __func__);
				touch_report_psensor_input_event(0);
				proximity_flag = 1;
			}
			wake_lock_timeout(&ts->ts_wake_lock, TS_WAKE_LOCK_TIMEOUT);
			return;
		}
	}
#endif
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 4);
	if (tpd_key == 0x0F) { /*All (VK+AA)leave*/
		tpd_key = 0x00;
	}
	//I("[DEBUG] tpd_key: %x\r\n", tpd_key);
#else
	tpd_key = 0x00;
#endif

	p_point_num = hx_point_num;

	if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff
			&& buf[HX_TOUCH_INFO_POINT_CNT + 1] == 0xff) {
		/* All vkeys are released  */
		hx_point_num = 0;
		vkey_status = 0x00;
		/* Check which key status has been changed, then report this change event */
		if ((!((vkey_previous[0] ^ vkey_status) & BIT(0)) && !((vkey_previous[1] ^ vkey_status) & BIT(1)))) {
			/* get system time, WA for cap always K issue */
			if (ts->pdata->always_K_WA) {
				getnstimeofday(&now);
				if((now.tv_sec - ts->time_start.tv_sec) > 2 ) {
					ts->K_counter = 0;
					ts->time_start.tv_sec = now.tv_sec;
				} else
					ts->K_counter++;
				if (ts->K_counter >= 4) {
					I("All Finger leave, K counter %u\n", ts->K_counter);
					queue_work(private_ts->recal_work_wq, &private_ts->recal_work);
				} else
					I("All Finger leave, K counter %u\n", ts->K_counter);
			} else
				I("All Finger leave, K counter %u\n", ts->K_counter);
			return;
		}
		spin_lock_irqsave(&ts->lock, irqflags);
		if ((vkey_previous[1] ^ vkey_status) & BIT(1)) {
			if (!(ts->SW_timer_debounce) || !(ts->debounced[1])) {
				input_report_key(ts->input_dev,	vkey_code[1], 0);
				I_TIME("KEY_APPSELECT Released");
				input_sync(ts->input_dev);
			} else {
				/*** Delete timer ***/
				cancel_delayed_work(&ts->monitorDB[1]);
				ts->debounced[1] = false;
				I_TIME("KEY_APPSELECT Released (debounced)");
			}
			vkey_previous[1] = (vkey_status & BIT(1));
		}
		if ((vkey_previous[0] ^ vkey_status) & BIT(0)) {
			if (!(ts->SW_timer_debounce) || !(ts->debounced[0])) {
				input_report_key(ts->input_dev,	vkey_code[0], 0);
				I_TIME("KEY_BACK Released");
				input_sync(ts->input_dev);
			} else {
				/*** Delete timer ***/
				cancel_delayed_work(&ts->monitorDB[0]);
				ts->debounced[0] = false;
				I_TIME("BACK key Released (debounced)");
			}
			vkey_previous[0] = (vkey_status & BIT(0));
		}
		spin_unlock_irqrestore(&ts->lock, irqflags);

		if ((ts->debug_log_level & BIT(1)) > 0) {
			I("{PN, ID Inf*2, ID Inf*3}={0x%02X, 0x%02X, 0x%02X}; vkey_previous app=%d, vkey_previous back=%d, vkey_status=%d\n",
				buf[HX_TOUCH_INFO_POINT_CNT],
				buf[HX_TOUCH_INFO_POINT_CNT + 1],
				buf[HX_TOUCH_INFO_POINT_CNT + 2],
				vkey_previous[1],
				vkey_previous[0],
				vkey_status);
		}
		return;
	} else if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xf0
			&& buf[HX_TOUCH_INFO_POINT_CNT + 1] == 0x00) {
		vkey_status = (buf[HX_TOUCH_INFO_POINT_CNT + 2] & 0x30) >> 4;

		/* Check which key status has been changed, then print this change */
		spin_lock_irqsave(&ts->lock, irqflags);
		if ((vkey_previous[1] ^ vkey_status) & BIT(1)) {
			//** debounce mechanism start **//
			if (ts->SW_timer_debounce) {
				queue_delayed_work(ts->monitor_wq[1], &ts->monitorDB[1], msecs_to_jiffies(ts->SW_timer_debounce));
				ts->debounced[1] = true;
				I_TIME("KEY_APPSELECT queue work");
			} else {
				I_TIME("KEY_APPSELECT Pressed");

				// start to show out the raw data in adb shell
				if (diag_command >= 1 && diag_command <= 6) {
					if (diag_command <= 3) {
						mutual_num	= x_channel * y_channel;
						self_num	= x_channel + y_channel; //don't add KEY_COUNT
						width		= x_channel;
						I("ChannelStart: %4d, %4d\n", x_channel, y_channel);
						I(" %4d %4d %3d\n", diag_mutual[0], diag_mutual[1], diag_self[width + 1/width]);
						I(" %4d %4d\n", diag_self[0], diag_self[1]);
					}
				}
				input_report_key(ts->input_dev,	vkey_code[1], 1);
				input_sync(ts->input_dev);
			}
			vkey_previous[1] = (vkey_status & BIT(1));
			//** debounce mechanism end **//
		}
		if ((vkey_previous[0] ^ vkey_status) & BIT(0)) {
			//** debounce mechanism start **//
			if (ts->SW_timer_debounce) {
				queue_delayed_work(ts->monitor_wq[0], &ts->monitorDB[0], msecs_to_jiffies(ts->SW_timer_debounce));
				ts->debounced[0] = true;
				I_TIME("KEY_BACK queue work");
			} else {
				I_TIME("KEY_BACK Pressed");

				// start to show out the raw data in adb shell
				if (diag_command >= 1 && diag_command <= 6) {
					if (diag_command <= 3) {
						mutual_num	= x_channel * y_channel;
						self_num	= x_channel + y_channel; //don't add KEY_COUNT
						width		= x_channel;
						I("ChannelStart: %4d, %4d\n", x_channel, y_channel);
						I(" %4d %4d %3d\n", diag_mutual[0], diag_mutual[1], diag_self[width + 1/width]);
						I(" %4d %4d\n", diag_self[0], diag_self[1]);
					}
				}
				input_report_key(ts->input_dev,	vkey_code[0], 1);
				input_sync(ts->input_dev);
			}
			vkey_previous[0] = (vkey_status & BIT(0));
			//** debounce mechanism end **//
		}
		spin_unlock_irqrestore(&ts->lock, irqflags);

		if ((ts->debug_log_level & BIT(1)) > 0) {
			I("{PN, ID Inf*2, ID Inf*3}={0x%02X, 0x%02X, 0x%02X}; vkey_previous app=%d, vkey_previous back=%d,vkey_status=%d\n",
				buf[HX_TOUCH_INFO_POINT_CNT],
				buf[HX_TOUCH_INFO_POINT_CNT + 1],
				buf[HX_TOUCH_INFO_POINT_CNT + 2],
				vkey_previous[1],
				vkey_previous[0],
				vkey_status);
		}
		return;
	} else if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff) {
		hx_point_num = 0;
	} else
		hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

	// Touch Point information
	if (hx_point_num != 0 ) {
		if(vk_press == 0x00) {
			uint16_t old_finger = ts->pre_finger_mask;
			finger_num = buf[coordInfoSize - 4] & 0x0F;
			finger_pressed = buf[coordInfoSize - 2] << 8 | buf[coordInfoSize - 3];
			finger_on = 1;
			AA_press = 1;
			for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
				if (((finger_pressed >> loop_i) & 1) == 1) {
					base = loop_i * 4;
					x = buf[base] << 8 | buf[base + 1];
					y = (buf[base + 2] << 8 | buf[base + 3]);
					w = buf[(ts->nFinger_support * 4) + loop_i];
					finger_num--;

					if ((ts->debug_log_level & BIT(3)) > 0) {
						if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1) {
							if (ts->useScreenRes) {
								if ((x) > 65) {
									input_report_key(ts->input_dev, KEY_APPSELECT, 1);
									I("KEY_APPSELECT Pressed. X:%d\n", x);
								} else {
									input_report_key(ts->input_dev, KEY_BACK, 1);
									I("KEY_BACK Pressed. X:%d\n", x);
								}
///								I("status:%X, Screen:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
///								  finger_pressed, loop_i + 1, x * ts->widthFactor >> SHIFTBITS,
///								  y * ts->heightFactor >> SHIFTBITS, w, EN_NoiseFilter);
							} else {
								I("status:%X, Raw:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
								  finger_pressed, loop_i + 1, x, y, w, EN_NoiseFilter);
							}
						}
					}

					if (ts->protocol_type == PROTOCOL_TYPE_B) {
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						if (ts->hall_block_touch_event == 0)
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
							input_mt_slot(ts->input_dev, loop_i);
					}

#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
					if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
					}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif

					if (ts->protocol_type == PROTOCOL_TYPE_A) {
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
							input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
							input_mt_sync(ts->input_dev);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
					} else {
						ts->last_slot = loop_i;
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						if (ts->hall_block_touch_event == 0)
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
					}

					if (!ts->first_pressed) {
						ts->first_pressed = 1;
						I("S1@%d, %d\n", x, y);
					}

					ts->pre_finger_data[loop_i][0] = x;
					ts->pre_finger_data[loop_i][1] = y;


					if (ts->debug_log_level & BIT(1))
						I("Finger %d=> X:%d, Y:%d W:%d, Z:%d, F:%d, N:%d\n",
						  loop_i + 1, x, y, w, w, loop_i + 1, EN_NoiseFilter);
				} else {
					if (ts->protocol_type == PROTOCOL_TYPE_B) {
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
							input_mt_slot(ts->input_dev, loop_i);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
					}

					if (loop_i == 0 && ts->first_pressed == 1) {
						ts->first_pressed = 2;
						I("E1@%d, %d\n",
						  ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
					}
					if ((ts->debug_log_level & BIT(3)) > 0) {
						if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1) {
							if (ts->useScreenRes) {
								if ((ts->pre_finger_data[loop_i][0]) > 65) {
									input_report_key(ts->input_dev, KEY_APPSELECT, 0);
									I("KEY_APPSELECT Released. X:%d\n", ts->pre_finger_data[loop_i][0]);
								} else {
									input_report_key(ts->input_dev, KEY_BACK, 0);
									I("KEY_BACK Released. X:%d\n", ts->pre_finger_data[loop_i][0]);
								}
///								I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n",
///								  finger_pressed, loop_i + 1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
///								  ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
							} else {
								I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",
								  finger_pressed, loop_i + 1, ts->pre_finger_data[loop_i][0],
								  ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
							}
						}
					}
				}
			}
			ts->pre_finger_mask = finger_pressed;
		} else if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
			//temp_x[0] = 0xFFFF;
			//temp_y[0] = 0xFFFF;
			//temp_x[1] = 0xFFFF;
			//temp_y[1] = 0xFFFF;
			himax_ts_button_func(tpd_key, ts);
			finger_on = 0;
		}
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
		if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
		}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
	} else if (hx_point_num == 0) {
#if defined(HX_PALM_REPORT)
		loop_i = 0;
		base = loop_i * 4;
		x = buf[base] << 8 | buf[base + 1];
		y = (buf[base + 2] << 8 | buf[base + 3]);
		w = buf[(ts->nFinger_support * 4) + loop_i];
		I(" %s HX_PALM_REPORT_loopi=%d,base=%x,X=%x,Y=%x,W=%x \n", __func__, loop_i, base, x, y, w);

		if((!atomic_read(&ts->suspend_mode)) && (x == 0xFA5A) && (y == 0xFA5A) && (w == 0x00)) {
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
			if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
				I(" %s HX_PALM_REPORT KEY power event press\n", __func__);
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(100);
				I(" %s HX_PALM_REPORT KEY power event release\n", __func__);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
			}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
			return;
		}
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
		if ((ts->pdata->proximity_bytp_enable) && (proximity_flag)) { //Proximity Far event
			I(" %s far event trigger\n", __func__);
			touch_report_psensor_input_event(1);
			proximity_flag = 0;	//clear flag , avoid touch point cant leave.
			wake_lock_timeout(&ts->ts_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		} else if(AA_press)
#else
		if(AA_press)
#endif
		{
			// leave event
			finger_on = 0;
			AA_press = 0;
			if (ts->protocol_type == PROTOCOL_TYPE_A)
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
				if (ts->hall_block_touch_event == 0)
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
					input_mt_sync(ts->input_dev);

			for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
				if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
					if (ts->protocol_type == PROTOCOL_TYPE_B) {
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
							input_mt_slot(ts->input_dev, loop_i);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
						}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
					}
				}
			}
			if (ts->pre_finger_mask > 0) {
				for (loop_i = 0; loop_i < ts->nFinger_support && (ts->debug_log_level & BIT(3)) > 0; loop_i++) {
					if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
						if ((ts->pre_finger_data[loop_i][0]) > 65) {
							input_report_key(ts->input_dev, KEY_APPSELECT, 0);
							I("KEY_APPSELECT Released. X:%d\n", ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS);
						} else {
							input_report_key(ts->input_dev, KEY_BACK, 0);
							I("KEY_BACK Released. X:%d\n", ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS);
						}

						if (ts->protocol_type == PROTOCOL_TYPE_B) {
							input_mt_slot(ts->input_dev, loop_i);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
						}
					}
///					if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
///						if (ts->useScreenRes) {
///							I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i + 1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
///							  ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
///						} else {
///							I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i + 1, ts->pre_finger_data[loop_i][0], ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
///						}
///					}
				}
				ts->pre_finger_mask = 0;
			}

			if (ts->first_pressed == 1) {
				ts->first_pressed = 2;
				I("E1@%d, %d\n", ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
			}

			if (ts->debug_log_level & BIT(1))
				I("All Finger leave\n");

#ifdef HX_TP_SYS_DIAG
			//coordinate dump start
			if (coordinate_dump_enable == 1) {
				do_gettimeofday(&t);
				time_to_tm(t.tv_sec, 0, &broken);
				scnprintf(&coordinate_char[0], 17, "%2d:%2d:%2d:%lu,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec / 1000);
				scnprintf(&coordinate_char[15], 10, "Touch up!");
				coordinate_fn->f_op->write(coordinate_fn, &coordinate_char[0], 15 + (HX_MAX_PT + 5) * 2 * sizeof(char) * 5 + 2, &coordinate_fn->f_pos);
			}
			//coordinate dump end
#endif
		} else if (tpd_key != 0x00) {
			//report key
			//temp_x[0] = 0xFFFF;
			//temp_y[0] = 0xFFFF;
			//temp_x[1] = 0xFFFF;
			//temp_y[1] = 0xFFFF;
			himax_ts_button_func(tpd_key, ts);
			finger_on = 1;
		} else if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
			//temp_x[0] = 0xFFFF;
			//temp_y[0] = 0xFFFF;
			//temp_x[1] = 0xFFFF;
			//temp_y[1] = 0xFFFF;
			himax_ts_button_func(tpd_key, ts);
			finger_on = 0;
		}
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
		if (ts->hall_block_touch_event == 0) {
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
		}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif
	}
	tpd_key_old = tpd_key;
	Last_EN_NoiseFilter = EN_NoiseFilter;

workqueue_out:
	return;

err_workqueue_out:
	I("%s: Now reset the Touch chip.\n", __func__);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true, false);
	I("%s: himax sesnor on\n", __func__);
	i2c_himax_write_command(private_ts->client, 0x83, DEFAULT_RETRY_CNT);
	msleep(30);
	i2c_himax_write_command(private_ts->client, 0x81, DEFAULT_RETRY_CNT);
	msleep(50);
#endif

	goto workqueue_out;
}

static irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	struct himax_ts_data *ts = ptr;
	struct timespec timeStart, timeEnd, timeDelta;

	if (ts->debug_log_level & BIT(2)) {
		getnstimeofday(&timeStart);
		/*I(" Irq start time = %ld.%06ld s\n",
			timeStart.tv_sec, timeStart.tv_nsec/1000);*/
	}
#ifdef HX_SMART_WAKEUP
	if (atomic_read(&ts->suspend_mode) && (!FAKE_POWER_KEY_SEND) && (ts->SMWP_enable)) {
		wake_lock_timeout(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		if(himax_parse_wake_event((struct himax_ts_data *)ptr)) {
			I(" %s SMART WAKEUP KEY power event press\n", __func__);
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			msleep(100);
			I(" %s SMART WAKEUP KEY power event release\n", __func__);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			FAKE_POWER_KEY_SEND = true;
			return IRQ_HANDLED;
		}
	}
#endif
	himax_ts_work_cap((struct himax_ts_data *)ptr);
	if(ts->debug_log_level & BIT(2)) {
		getnstimeofday(&timeEnd);
		timeDelta.tv_nsec = (timeEnd.tv_sec * 1000000000 + timeEnd.tv_nsec)
				    - (timeStart.tv_sec * 1000000000 + timeStart.tv_nsec);
		/*I("Irq finish time = %ld.%06ld s\n",
			timeEnd.tv_sec, timeEnd.tv_nsec/1000);*/
		I("Touch latency = %ld us\n", timeDelta.tv_nsec / 1000);
	}
	return IRQ_HANDLED;
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
} 

static void himax_ts_work_cap_func(struct work_struct *work)
{
//	himax_ts_work_cap();
}

#ifdef MTK
#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *desc)
{
	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	himax_int_enable_cap(private_ts->client->irq,0,false);
	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}
#endif
#endif

int himax_ts_register_interrupt_cap(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	uint8_t i=0,j=0;
	uint8_t buf[256];
	int ret = 0;
	I("Himax_interrupt = %s, client->irq value: %d  gpop_irq: %d\n",__func__, client->irq, ts->pdata->gpio_irq);

	ts->irq_enabled = 0;
	ts->use_irq = 0;
	if(himax_int_gpio_read_cap(ts->pdata->gpio_irq)==0)
	{
		for (i=0;i<50;i++)
		{
			ret = i2c_himax_read(ts->client, 0x86, buf, 128,DEFAULT_RETRY_CNT);
			msleep(10);
			if(himax_int_gpio_read_cap(ts->pdata->gpio_irq))
			{
				I("%s event stack has been clear\n ",__func__);
				if(++j > 5)
					break;
			}
		}
	}
	//Work functon
	if (client->irq) {
		ts->use_irq = 1;
		I("%s level trigger low\n ",__func__);
		ret = request_threaded_irq(client->irq, NULL, himax_ts_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts);
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
#ifdef HX_SMART_WAKEUP
			irq_set_irq_wake(client->irq, 1);
#endif
		} else {
			ts->use_irq = 0;
			E("%s: request_irq failed, ret is : %d\n", __func__,ret);
		}
	} else {
		I("%s: client->irq is empty, use polling mode.\n", __func__);
	}
	if (!ts->use_irq) {
		ts->himax_wq = create_singlethread_workqueue("himax_touch-cap");

		INIT_WORK(&ts->work, himax_ts_work_cap_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		I("%s: polling mode enabled\n", __func__);
	}
	return 0;
}

#if defined(HX_USB_DETECT)
static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	I("Touch: cable change to %d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode)) {
			if ((!!connect_status) != ts->usb_connected) {
				if (!!connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

				i2c_himax_master_write(ts->client, ts->cable_config,
					sizeof(ts->cable_config), DEFAULT_RETRY_CNT);

				I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			} else
				I("%s: Cable status is the same as previous one, ignore.\n", __func__);
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;
			I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};

#endif


#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
							work_att.work);
	I(" %s in", __func__);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		E(" Unable to register fb_notifier: %d\n", ret);
}
#endif

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
int proximity_enable_from_ps(int on)
{
	char buf_tmp[5];
	if (on)
	{
		touch_report_psensor_input_event(1);//Workaround for screen off on phone APP
		buf_tmp[0] = 0x92;
		buf_tmp[1] = 0x01;
		g_proximity_en=1;
		enable_irq_wake(private_ts->client->irq);
	}
	else
	{
		buf_tmp[0] = 0x92;
		buf_tmp[1] = 0x00;
		g_proximity_en=0;
		disable_irq_wake(private_ts->client->irq);
	}

	I("Proximity=%d\n",on);
	i2c_himax_master_write(private_ts->client, buf_tmp, 2, DEFAULT_RETRY_CNT);

	return 0;
}
EXPORT_SYMBOL_GPL(proximity_enable_from_ps);
#endif

//=============================================================================================================
//
//	Segment : Himax SYS Debug Function
//
//=============================================================================================================
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
#if defined(HTC_FEATURE)
static void hx_sensor_on(bool enable) {
	if(enable && !(atomic_read(&private_ts->sensing_status))) {
		I("%s: ON\n", __func__);
		i2c_himax_write_command(private_ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
		msleep(30);
		i2c_himax_write_command(private_ts->client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
		msleep(50);
		atomic_set(&private_ts->sensing_status, 1);
	} else if((enable == false) && atomic_read(&private_ts->sensing_status)) {
		I("%s: OFF\n", __func__);
		i2c_himax_write_command(private_ts->client, HX_CMD_TSSOFF, DEFAULT_RETRY_CNT);
		msleep(50);
		i2c_himax_write_command(private_ts->client, HX_CMD_TSSLPIN, DEFAULT_RETRY_CNT);
		msleep(50);
		atomic_set(&private_ts->sensing_status, 0);
	} else
		I("%s: set sensing status fault! sesnor status %d\n", __func__, atomic_read(&private_ts->sensing_status));
}

static int hx_sensitivity_setup(struct himax_ts_data *cs, uint8_t level)
{
	uint8_t i;
	bool tmpStatus = false;
	int err = 0;

	D("%s: %x\n", __func__, level);
	if (atomic_read(&private_ts->sensing_status)) {
		I("%s: sensor already on, switch to off\n", __func__);
		hx_sensor_on(false);
		tmpStatus = true;
	}

	err = i2c_himax_write(cs->client, CS_SENSITIVITY_START_REG, &level, 1, DEFAULT_RETRY_CNT);
	if (err < 0)
		E("%s: I2C write fail. err %d, addr: 0x%02X.\n", __func__, err, i);

	if (tmpStatus)
		hx_sensor_on(true);

	return err;
}

#ifdef CONFIG_AK8789_HALLSENSOR
static int hallsensor_handler_callback(struct notifier_block *self,
				         unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct himax_ts_data *cs = container_of(self, struct himax_ts_data, hallsensor_handler);

	pole_value = status & 0x01;
	pole = (status & 0x02) >> HALL_POLE_BIT;
	if (pole == HALL_POLE_S) {
		if (pole_value == HALL_NEAR) {
			if (cs->suspended == false)
				himax_int_enable_cap(cs->client->irq,0,true);
			cs->cap_hall_s_pole_status = HALL_NEAR;
		} else {
			if (cs->suspended == false)
				himax_int_enable_cap(cs->client->irq,1,true);
			cs->cap_hall_s_pole_status = HALL_FAR;
		}
		I("[HL] %s[%s] cover_enable = %d\n",
				pole? "att_s" : "att_n", pole_value ? "Near" : "Far",
				cs->cap_hall_s_pole_status);
	} else
		I("[HL] %s[%s]\n", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");
	return NOTIFY_OK;
}
#endif

static ssize_t touch_vendor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	ret += scnprintf(buf, PAGE_SIZE, "%s_FW:%#x,%x_CFG:%#x_SensorId:%#x\n", HIMAX852xes_NAME_CAP,
			 ts_data->vendor_fw_ver_H, ts_data->vendor_fw_ver_L, ts_data->vendor_config_ver, ts_data->vendor_sensor_id);

	return ret;
}

static DEVICE_ATTR(vendor, (S_IRUGO), touch_vendor_show, NULL);

static ssize_t touch_attn_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	scnprintf(buf, PAGE_SIZE, "attn = %x\n", himax_int_gpio_read_cap(ts_data->pdata->gpio_irq));
	ret = strlen(buf) + 1;

	return ret;
}
static DEVICE_ATTR(attn, (S_IRUGO), touch_attn_show, NULL);

static ssize_t hx_glove_setting_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *cs = private_ts;
	I("%s\n", __func__);
	return scnprintf(buf, PAGE_SIZE, "%d\n", cs->cap_glove_mode_status);
}
static ssize_t hx_glove_setting_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *cs = private_ts;
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		E("%s: Failed to get the inout value\n", __func__);
			return -EINVAL;
	}
	if (input > 2) {
		E("%s: wrong parameter\n", __func__);
		return -EINVAL;
	}

	if (input == 1) {
		cs->cap_glove_mode_status = 1;
	} else if (input == 0) {
		cs->cap_glove_mode_status = 0;
	}

	I("%s: glove_setting status change to %d\n", __func__, cs->cap_glove_mode_status);

	return count;
}
static DEVICE_ATTR(glove_setting, (S_IRUGO|S_IWUSR), hx_glove_setting_show, hx_glove_setting_store);

static ssize_t himax_int_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;

	count += scnprintf(buf + count, PAGE_SIZE, "%d\n", ts->irq_enabled);

	return count;
}

static ssize_t himax_int_status_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts = private_ts;
	int value, ret = 0;

	if (sysfs_streq(buf, "0"))
		value = false;
	else if (sysfs_streq(buf, "1"))
		value = true;
	else
		return -EINVAL;

	if (value) {
		if(HX_INT_IS_EDGE) {
#ifdef MTK
#ifdef CONFIG_OF_TOUCH
			ret = request_irq(ts->client->irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING, HIMAX852xes_NAME_CAP, NULL);
			if(ret > 0){
				ret = -1;
				E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
			}
#else
//			mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
//			mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
#endif
#endif
#ifdef QCT
			ret = request_threaded_irq(ts->client->irq, NULL, himax_ts_thread,
						   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts->client->name, ts);
#endif
		} else {
#ifdef MTK
#ifdef CONFIG_OF_TOUCH
			ret = request_irq(ts->client->irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_LOW, HIMAX852xes_NAME_CAP, NULL);
			if(ret > 0){
				ret = -1;
				E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
			}
#else
//			mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
//			mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 1);
#endif
#endif
#ifdef QCT
			ret = request_threaded_irq(ts->client->irq, NULL, himax_ts_thread,
						   IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts->client->name, ts);
#endif
		}
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
		}
	} else {
		himax_int_enable_cap(ts->client->irq, 0, true);
		free_irq(ts->client->irq, ts);
		ts->irq_enabled = 0;
	}

	return count;
}

static DEVICE_ATTR(enabled, (S_IWUSR | S_IRUGO),
		   himax_int_status_show, himax_int_status_store);

static ssize_t himax_layout_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;

	count += scnprintf(buf + count, PAGE_SIZE, "%d %d %d %d\n", ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	return count;
}

static ssize_t himax_layout_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	unsigned long value;
	int layout[4] = {0};

	for (i = 0; i < 20; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				I("buffer size is over 5 char\n");
				return count;
			}
			j = i + 1;
			if (k < 4) {
				ret = kstrtol(buf_tmp, 10, &value);
				layout[k++] = value;
			}
		}
	}
	if (k == 4) {
		ts->pdata->abs_x_min = layout[0];
		ts->pdata->abs_x_max = layout[1];
		ts->pdata->abs_y_min = layout[2];
		ts->pdata->abs_y_max = layout[3];
		I("%d, %d, %d, %d\n",
		  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
		input_unregister_device(ts->input_dev);
		himax_input_register(ts);
	} else
		I("ERR@%d, %d, %d, %d\n",
		  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	return count;
}

static DEVICE_ATTR(layout, (S_IWUSR | S_IRUGO),
		   himax_layout_show, himax_layout_store);

static ssize_t himax_debug_level_show_cap(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	count += scnprintf(buf, PAGE_SIZE, "%d\n", ts_data->debug_log_level);

	return count;
}

static ssize_t himax_debug_level_dump_cap(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts;
	char buf_tmp[11];
	int i;
	ts = private_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memcpy(buf_tmp, buf, count);

	ts->debug_log_level = 0;
	for(i = 0; i < count - 1; i++) {
		if( buf_tmp[i] >= '0' && buf_tmp[i] <= '9' )
			ts->debug_log_level |= (buf_tmp[i] - '0');
		else if( buf_tmp[i] >= 'A' && buf_tmp[i] <= 'F' )
			ts->debug_log_level |= (buf_tmp[i] - 'A' + 10);
		else if( buf_tmp[i] >= 'a' && buf_tmp[i] <= 'f' )
			ts->debug_log_level |= (buf_tmp[i] - 'a' + 10);

		if(i != count - 2)
			ts->debug_log_level <<= 4;
	}

	if (ts->debug_log_level & BIT(3)) {
		if (ts->pdata->screenWidth > 0 && ts->pdata->screenHeight > 0 &&
		    (ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0 &&
		    (ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
			ts->widthFactor = (ts->pdata->screenWidth << SHIFTBITS) / (ts->pdata->abs_x_max - ts->pdata->abs_x_min);
			ts->heightFactor = (ts->pdata->screenHeight << SHIFTBITS) / (ts->pdata->abs_y_max - ts->pdata->abs_y_min);
			if (ts->widthFactor > 0 && ts->heightFactor > 0)
				ts->useScreenRes = 1;
			else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else
			I("Enable finger debug with raw position mode!\n");
	} else {
		ts->useScreenRes = 0;
		ts->widthFactor = 0;
		ts->heightFactor = 0;
	}

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR | S_IRUGO),
		   himax_debug_level_show_cap, himax_debug_level_dump_cap);

#ifdef HX_TP_SYS_REGISTER
static ssize_t himax_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i, loop_j;
	uint8_t data[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(data, 0x00, sizeof(data));

	I("Himax multi_register_command = %d \n", multi_register_command);

	if (multi_register_command == 1) {
		base = 0;

		for(loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1) {//config bank register
					outData[0] = 0x14;
					i2c_himax_write(private_ts->client, 0x8C , &outData[0], 1, DEFAULT_RETRY_CNT);
					msleep(10);

					outData[0] = 0x00;
					outData[1] = multi_register[loop_i];
					i2c_himax_write(private_ts->client, 0x8B , &outData[0], 2, DEFAULT_RETRY_CNT);
					msleep(10);

					i2c_himax_read(private_ts->client, 0x5A, data, 128, DEFAULT_RETRY_CNT);

					outData[0] = 0x00;
					i2c_himax_write(private_ts->client, 0x8C , &outData[0], 1, DEFAULT_RETRY_CNT);

					for(loop_j = 0; loop_j < 128; loop_j++)
						multi_value[base++] = data[loop_j];
				} else {//normal register
					i2c_himax_read(private_ts->client, multi_register[loop_i], data, 128, DEFAULT_RETRY_CNT);

					for(loop_j = 0; loop_j < 128; loop_j++)
						multi_value[base++] = data[loop_j];
				}
			}
		}

		base = 0;
		for(loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1)
					ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Register: FE(%x)\n", multi_register[loop_i]);
				else
					ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Register: %x\n", multi_register[loop_i]);

				for (loop_j = 0; loop_j < 128; loop_j++) {
					ret += scnprintf(buf + ret, PAGE_SIZE - ret, "0x%2.2X ", multi_value[base++]);
					if ((loop_j % 16) == 15)
						ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
				}
			}
		}
		return ret;
	}

	if (config_bank_reg) {
		I("%s: register_command = FE(%x)\n", __func__, register_command);

		//Config bank register read flow.

		outData[0] = 0x14;
		i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		outData[1] = register_command;
		i2c_himax_write(private_ts->client, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);

		i2c_himax_read(private_ts->client, 0x5A, data, 128, DEFAULT_RETRY_CNT);
		msleep(10);

		outData[0] = 0x00;
		i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
	} else {
		if (i2c_himax_read(private_ts->client, register_command, data, 128, DEFAULT_RETRY_CNT) < 0)
			return ret;
	}

	if (config_bank_reg)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "command: FE(%x)\n", register_command);
	else
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "command: %x\n", register_command);

	for (loop_i = 0; loop_i < 128; loop_i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length	= 0;
	unsigned long result	= 0;
	uint8_t loop_i		= 0;
	uint16_t base		= 5;
	uint8_t write_da[128];
	uint8_t outData[5];

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	I("himax %s \n", buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		I("himax multi register enter\n");

		multi_register_command = 1;

		base	= 2;
		loop_i	= 0;

		while(true) {
			if (buf[base] == '\n')
				break;

			if (loop_i >= 6 )
				break;

			if (buf[base] == ':' && buf[base + 1] == 'x' && buf[base + 2] == 'F' &&
			    buf[base + 3] == 'E' && buf[base + 4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!kstrtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!kstrtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		I("========================== \n");
		for(loop_i = 0; loop_i < 6; loop_i++)
			I("%d,%d:", multi_register[loop_i], multi_cfg_bank[loop_i]);
		I("\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			if (buf[3] == 'F' && buf[4] == 'E') {//Config bank register
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!kstrtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 7;

				I("CMD: FE(%x)\n", register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!kstrtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 5;
				I("CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x14;
							i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
							msleep(10);

							outData[0] = 0x00;
							outData[1] = register_command;
							i2c_himax_write(private_ts->client, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);

							msleep(10);
							i2c_himax_write(private_ts->client, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);

							msleep(10);
							outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);

							I("CMD: FE(%x), %x, %d\n", register_command, write_da[0], length);
						} else {
							i2c_himax_write(private_ts->client, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
							I("CMD: %x, %x, %d\n", register_command, write_da[0], length);
						}
					}
					I("\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!kstrtoul(buf_tmp, 16, &result)) {
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR | S_IRUGO), himax_register_show, himax_register_store);
#endif

#ifdef HX_TP_SYS_DIAG
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}
static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}
static uint8_t getXChannel(void)
{
	return x_channel;
}
static uint8_t getYChannel(void)
{
	return y_channel;
}
static uint8_t getDiagCommand(void)
{
	return diag_command;
}
static void setXChannel(uint8_t x)
{
	x_channel = x;
}
static void setYChannel(uint8_t y)
{
	y_channel = y;
}
static void setMutualBuffer(void)
{
	diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

#ifdef HX_TP_SYS_2T2R
static uint8_t *getMutualBuffer_2(void)
{
	return diag_mutual_2;
}
static uint8_t getXChannel_2(void)
{
	return x_channel_2;
}
static uint8_t getYChannel_2(void)
{
	return y_channel_2;
}
static void setXChannel_2(uint8_t x)
{
	x_channel_2 = x;
}
static void setYChannel_2(uint8_t y)
{
	y_channel_2 = y;
}
static void setMutualBuffer_2(void)
{
	diag_mutual_2 = kzalloc(x_channel_2 * y_channel_2 * sizeof(uint8_t), GFP_KERNEL);
}
#endif

static ssize_t himax_diag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
#ifdef HX_TP_SYS_2T2R
	if(Is_2T2R && diag_command == 4) {
		mutual_num 	= x_channel_2 * y_channel_2;
		self_num 	= x_channel_2 + y_channel_2; //don't add KEY_COUNT
		width 		= x_channel_2;
		count += scnprintf(buf + count, PAGE_SIZE - count, "ChannelStart: %4d, %4d\n\n", x_channel_2, y_channel_2);
	} else
#endif
	{
		mutual_num 	= x_channel * y_channel;
		self_num 	= x_channel + y_channel; //don't add KEY_COUNT
		width 		= x_channel;
		count += scnprintf(buf + count, PAGE_SIZE - count, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	}

	// start to show out the raw data in adb shell
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command <= 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					count += scnprintf(buf + count, PAGE_SIZE - count, " %3d\n", diag_self[width + loop_i / width]);
			}
			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			}
#ifdef HX_EN_SEL_BUTTON
			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_self[HX_RX_NUM + HX_TX_NUM + loop_i]);
#endif
#ifdef HX_TP_SYS_2T2R
		} else if(Is_2T2R && diag_command == 4 ) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_mutual_2[loop_i]);
				if ((loop_i % width) == (width - 1))
					count += scnprintf(buf + count, PAGE_SIZE - count, " %3d\n", diag_self[width + loop_i / width]);
			}
			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			}

#ifdef HX_EN_SEL_BUTTON
			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_self[HX_RX_NUM_2 + HX_TX_NUM_2 + loop_i]);
#endif
#endif
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
					count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			}
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "ChannelEnd\n");
	} else if (diag_command == 7) {
		for (loop_i = 0; loop_i < 128 ; loop_i++) {
			if ((loop_i % 16) == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "LineStart:");
			count += scnprintf(buf + count, PAGE_SIZE - count, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
	}
	return count;
}

static ssize_t himax_diag_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const uint8_t command_ec_128_raw_flag 		= 0x02;
	const uint8_t command_ec_24_normal_flag 	= 0x00;
	uint8_t command_ec_128_raw_baseline_flag 	= 0x01;
	uint8_t command_ec_128_raw_bank_flag 			= 0x03;

	uint8_t command_F1h[2] = {0xF1, 0x00};
	uint8_t receive[1];

	memset(receive, 0x00, sizeof(receive));

	diag_command = buf[0] - '0';

	I("[Himax]diag_command=0x%x\n", diag_command);
	if (diag_command == 0x01)	{//DC
		command_F1h[1] = command_ec_128_raw_baseline_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x02) {//IIR
		command_F1h[1] = command_ec_128_raw_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x03) {	//BANK
		command_F1h[1] = command_ec_128_raw_bank_flag;	//0x03
		i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x04 ) { // 2T3R IIR
		command_F1h[1] = 0x04; //2T3R IIR
		i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x00) {//Disable
		command_F1h[1] = command_ec_24_normal_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
		touch_monitor_stop_flag = touch_monitor_stop_limit;
	}

	//coordinate dump start
	else if (diag_command == 0x08)	{
		coordinate_fn = filp_open(DIAG_COORDINATE_FILE, O_CREAT | O_WRONLY | O_APPEND | O_TRUNC, 0666);
		if (IS_ERR(coordinate_fn)) {
			E("%s: coordinate_dump_file_create error\n", __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn, NULL);
		}
		coordinate_dump_enable = 1;
	} else if (diag_command == 0x09) {
		coordinate_dump_enable = 0;

		if (!IS_ERR(coordinate_fn)) {
			filp_close(coordinate_fn, NULL);
		}
	}
	//coordinate dump end
	else {
		E("[Himax]Diag command error!diag_command=0x%x\n", diag_command);
	}
	return count;
}
static DEVICE_ATTR(diag, (S_IWUSR | S_IRUGO), himax_diag_show, himax_diag_dump);
#endif

#ifdef HX_TP_SYS_RESET
static ssize_t himax_reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		himax_int_enable_cap(private_ts->client->irq, 0, true);
		himax_HW_reset_cap(true, true);
		hx_sensor_on(true);
		himax_int_enable_cap(private_ts->client->irq, 1, true);
	}

	return count;
}

static DEVICE_ATTR(reset, (S_IWUSR), NULL, himax_reset_set);
#endif

#ifdef HX_TP_SYS_DEBUG
static ssize_t himax_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	if (debug_level_cmd == 't') {
		if (fw_update_complete) {
			count += scnprintf(buf, PAGE_SIZE, "FW Update Complete ");
		} else {
			count += scnprintf(buf, PAGE_SIZE, "FW Update Fail ");
		}
	} else if (debug_level_cmd == 'h') {
		if (handshaking_result == 0) {
			count += scnprintf(buf, PAGE_SIZE, "Handshaking Result = %d (MCU Running)\n", handshaking_result);
		} else if (handshaking_result == 1) {
			count += scnprintf(buf, PAGE_SIZE, "Handshaking Result = %d (MCU Stop)\n", handshaking_result);
		} else if (handshaking_result == 2) {
			count += scnprintf(buf, PAGE_SIZE, "Handshaking Result = %d (I2C Error)\n", handshaking_result);
		} else {
			count += scnprintf(buf, PAGE_SIZE, "Handshaking Result = error \n");
		}
	} else if (debug_level_cmd == 'v') {
		count += scnprintf(buf + count, PAGE_SIZE - count, "FW_VER = 0x%2.2X, %2.2X \n", private_ts->vendor_fw_ver_H, private_ts->vendor_fw_ver_L);
		count += scnprintf(buf + count, PAGE_SIZE - count, "CONFIG_VER = 0x%2.2X \n", private_ts->vendor_config_ver);
	} else if (debug_level_cmd == 'd') {
		count += scnprintf(buf + count, PAGE_SIZE - count, "Himax Touch IC Information :\n");
		if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Type : D\n");
		} else if (IC_TYPE == HX_85XX_E_SERIES_PWON) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Type : E\n");
		} else if (IC_TYPE == HX_85XX_ES_SERIES_PWON) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Type : ES\n");
		} else {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Type error.\n");
		}

		if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Checksum : SW\n");
		} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Checksum : HW\n");
		} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Checksum : CRC\n");
		} else {
			count += scnprintf(buf + count, PAGE_SIZE - count, "IC Checksum error.\n");
		}

		if (HX_INT_IS_EDGE) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "Interrupt : EDGE TIRGGER\n");
		} else {
			count += scnprintf(buf + count, PAGE_SIZE - count, "Interrupt : LEVEL TRIGGER\n");
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "RX Num : %d\nTX Num : %d\nBT Num : %d\n"
				   "X Resolution : %d\nY Resolution : %d\nMax Point : %d\n",
				   HX_RX_NUM, HX_TX_NUM, HX_BT_NUM, HX_X_RES, HX_Y_RES, HX_MAX_PT);
#ifdef HX_TP_SYS_2T2R
		if(Is_2T2R)
			count += scnprintf(buf + count, PAGE_SIZE - count, "2T2R panel\nRX Num_2 : %d\nTX Num_2 : %d\n", HX_RX_NUM_2, HX_TX_NUM_2);
#endif
	} else if (debug_level_cmd == 'i')
		count += scnprintf(buf, PAGE_SIZE, "Himax Touch Driver Version:\n%s \n", HIMAX_DRIVER_VER);

	return count;
}

static ssize_t himax_debug_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct file* filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];

	if ( buf[0] == 'h') { //handshaking
		debug_level_cmd = buf[0];

		himax_int_enable_cap(private_ts->client->irq, 0, true);

		handshaking_result = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail

		himax_int_enable_cap(private_ts->client->irq, 1, true);

		return count;
	}

	else if ( buf[0] == 'v') { //firmware version
		debug_level_cmd = buf[0];
		himax_read_FW_ver(true);
		return count;
	}

	else if ( buf[0] == 'd') { //test
		debug_level_cmd = buf[0];
		return count;
	}

	else if ( buf[0] == 'i') { //driver version
		debug_level_cmd = buf[0];
		return count;
	}

	else if (buf[0] == 't') {

		himax_int_enable_cap(private_ts->client->irq, 0, true);
#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
		cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

		debug_level_cmd 	= buf[0];
		fw_update_complete	= false;

		memset(fileName, 0, 128);
		// parse the file name
		if (sizeof(fileName) > count - 2)
			scnprintf(fileName, count - 2, "%s", &buf[2]);
		else {
			E("%s: file name is too long\n", __func__);
			goto firmware_upgrade_done;
			//return count;
		}

		I("%s: upgrade from file(%s) start!\n", __func__, fileName);
		// open file
		filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(filp)) {
			E("%s: open firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return count;
		}
		oldfs = get_fs();
		set_fs(get_ds());

		// read the latest firmware binary file
		result = filp->f_op->read(filp, upgrade_fw, sizeof(upgrade_fw), &filp->f_pos);
		if (result < 0) {
			E("%s: read firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return count;
		}

		set_fs(oldfs);
		filp_close(filp, NULL);

		I("%s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

		if (result > 0) {
			// start to upgrade
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset_cap(false, true);
#endif
			if (fts_ctpm_fw_upgrade_with_sys_fs_cap(upgrade_fw, result, true) == 0) {
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			goto firmware_upgrade_done;
			//return count;
		}
	}

firmware_upgrade_done:

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true, false);
#endif
	hx_sensor_on(false);
	I("%s: himax read CAP info\n", __func__);
	himax_read_TP_info(private_ts->client);
	hx_sensor_on(true);

	himax_int_enable_cap(private_ts->client->irq, 1, true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES * HZ);
#endif
	//todo himax_chip->tp_firmware_upgrade_proceed = 0;
	//todo himax_chip->suspend_state = 0;
	//todo enable_irq(himax_chip->irq);
	return count;
}

static DEVICE_ATTR(debug, (S_IWUSR | S_IRUGO), himax_debug_show, himax_debug_dump);
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
static ssize_t himax_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step = 0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	I("flash_progress = %d \n", local_flash_progress);

	if (local_flash_fail) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashStart:Fail \nFlashEnd\n");
		return ret;
	}

	if (!local_flash_complete) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashStart:Ongoing:0x%2.2x \nFlashEnd\n", flash_progress);
		return ret;
	}

	if (local_flash_command == 1 && local_flash_complete) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashStart:Complete \nFlashEnd\n");
		return ret;
	}

	if (local_flash_command == 3 && local_flash_complete) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashStart: \n");
		for(loop_i = 0; loop_i < 128; loop_i++) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "x%2.2x", flash_buffer[loop_i]);
			if ((loop_i % 16) == 15) {
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
			}
		}
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashEnd\n");
		return ret;
	}

	//flash command == 0 , report the data
	local_flash_read_step = getFlashReadStep();

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashStart:%2.2x \n", local_flash_read_step);

	for (loop_i = 0; loop_i < 1024; loop_i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "x%2.2X", flash_buffer[local_flash_read_step * 1024 + loop_i]);

		if ((loop_i % 16) == 15) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
		}
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashEnd\n");
	return ret;
}

static ssize_t himax_flash_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	I("%s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1) {
		E("%s: SYS is busy , return!\n", __func__);
		return count;
	}

	if (buf[0] == '0') {
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x') {
			memcpy(buf_tmp, buf + 3, 2);
			I("%s: read_Step = %s\n", __func__, buf_tmp);
			if (!kstrtoul(buf_tmp, 16, &result)) {
				I("%s: read_Step = %lu \n", __func__, result);
				setFlashReadStep(result);
			}
		}
	} else if (buf[0] == '1') {
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '2') {
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '3') {
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!kstrtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!kstrtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		}

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '4') {
		I("%s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!kstrtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		} else {
			E("%s: command 4 , sector error.\n", __func__);
			return count;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!kstrtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		} else {
			E("%s: command 4 , page error.\n", __func__);
			return count;
		}

		base = 11;

		I("=========Himax flash page buffer start=========\n");
		for(loop_i = 0; loop_i < 128; loop_i++) {
			memcpy(buf_tmp, buf + base, 2);
			if (!kstrtoul(buf_tmp, 16, &result)) {
				flash_buffer[loop_i] = result;
				I("%d ", flash_buffer[loop_i]);
				if (loop_i % 16 == 15) {
					I("\n");
				}
			}
			base += 3;
		}
		I("=========Himax flash page buffer end=========\n");

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	return count;
}
static DEVICE_ATTR(flash_dump, (S_IWUSR | S_IRUGO), himax_flash_show, himax_flash_store);
#endif	/* #ifdef HX_TP_SYS_FLASH_DUMP */

#ifdef HX_TP_SYS_SELF_TEST
static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0x00;
	val = himax_chip_self_test();

	if (val == 0x00) {
		return scnprintf(buf, PAGE_SIZE, "Self_Test Pass\n");
	} else {
		return scnprintf(buf, PAGE_SIZE, "Self_Test Fail\n");
	}
}

static int himax_chip_self_test(void)
{
	uint8_t cmdbuf[11];
	uint8_t valuebuf[16];
	int i = 0, pf_value = 0x00;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(valuebuf, 0x00, sizeof(valuebuf));

	cmdbuf[0] = 0x06;
	i2c_himax_write(private_ts->client, 0xF1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, 0x83, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, 0x81, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(2000);

	i2c_himax_write(private_ts->client, 0x82, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, 0x80, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0xF1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_read(private_ts->client, 0xB1, valuebuf, 8, DEFAULT_RETRY_CNT);
	msleep(10);

	for(i = 0; i < 8; i++) {
		I("[Himax]: After slf test 0xB1 buff_back[%d] = 0x%x\n", i, valuebuf[i]);
	}

	msleep(30);

	if (valuebuf[0] == 0xAA) {
		I("[Himax]: self-test pass\n");
		pf_value = 0x0;
	} else {
		E("[Himax]: self-test fail\n");
		pf_value = 0x1;
	}

	return pf_value;
}

static DEVICE_ATTR(tp_self_test, (S_IWUSR | S_IRUGO), himax_chip_self_test_function, NULL);
#endif

#ifdef HX_TP_SYS_HITOUCH
static ssize_t himax_hitouch_show_cap(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if(hitouch_command == 0) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Driver Version:2.0 \n");
	}

	return ret;
}

//-----------------------------------------------------------------------------------
//himax_hitouch_store_cap
//command 0 : Get Driver Version
//command 1 : Hitouch Connect
//command 2 : Hitouch Disconnect
//-----------------------------------------------------------------------------------
static ssize_t himax_hitouch_store_cap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0] == '0') {
		hitouch_command = 0;
	} else if(buf[0] == '1') {
		hitouch_is_connect = true;
		I("hitouch_is_connect = true\n");
	} else if(buf[0] == '2') {
		hitouch_is_connect = false;
		I("hitouch_is_connect = false\n");
	}
	return count;
}

static DEVICE_ATTR(hitouch, (S_IWUSR | S_IRUGO), himax_hitouch_show_cap, himax_hitouch_store_cap);
#endif

#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
static ssize_t himax_cover_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *cs = private_ts;
	I("%s\n", __func__);
	return scnprintf(buf, PAGE_SIZE, "%d\n", cs->cap_hall_s_pole_status);
}

static ssize_t himax_cover_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t count)
{

	struct himax_ts_data *ts = private_ts;

	if (sysfs_streq(buf, "0"))
		ts->cover_enable = 0;
	else if (sysfs_streq(buf, "1"))
		ts->cover_enable = 1;
	else
		return -EINVAL;
	himax_set_cover_func(ts->cover_enable);

	I("%s: cover_enable = %d.\n", __func__, ts->cover_enable);

	return count;
}

static DEVICE_ATTR(cover, (S_IWUSR | S_IRUGO),
		   himax_cover_show, himax_cover_store);
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif

#ifdef HX_SMART_WAKEUP
static ssize_t himax_SMWP_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;
	count = scnprintf(buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);

	return count;
}

static ssize_t himax_SMWP_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{

	struct himax_ts_data *ts = private_ts;

	if (sysfs_streq(buf, "0"))
		ts->SMWP_enable = 0;
	else if (sysfs_streq(buf, "1"))
		ts->SMWP_enable = 1;
	else
		return -EINVAL;

	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, ts->SMWP_enable);

	return count;
}

static DEVICE_ATTR(SMWP, (S_IWUSR | S_IRUGO),
		   himax_SMWP_show, himax_SMWP_store);
#endif

enum SR_REG_STATE {
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};

static int register_sr_touch_device(void)
{
	struct himax_ts_data *ts = private_ts;
	ts->sr_input_dev = input_allocate_device();

	if (ts->sr_input_dev == NULL) {
		E("[SR]%s: Failed to allocate SR input device\n", __func__);
		return ALLOCATE_DEV_FAIL;
	}
	ts->sr_input_dev->name = "sr_touchscreen";
	set_bit(EV_SYN, ts->sr_input_dev->evbit);
	set_bit(EV_ABS, ts->sr_input_dev->evbit);
	set_bit(EV_KEY, ts->sr_input_dev->evbit);

	set_bit(KEY_BACK, ts->sr_input_dev->keybit);
	set_bit(KEY_HOME, ts->sr_input_dev->keybit);
	set_bit(KEY_MENU, ts->sr_input_dev->keybit);
	set_bit(KEY_SEARCH, ts->sr_input_dev->keybit);
	set_bit(BTN_TOUCH, ts->sr_input_dev->keybit);
	//set_bit(KEY_APP_SWITCH, ts->sr_input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->sr_input_dev->propbit);

	input_set_abs_params(ts->sr_input_dev, ABS_MT_TRACKING_ID,
			     0, 3, 0, 0);
	I("[SR]input_set_abs_params: mix_x %d, max_x %d,"
	  " min_y %d, max_y %d\n", ts->pdata->abs_x_min,
	  ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_X, ts->pdata->abs_x_min, ts->pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_Y, ts->pdata->abs_y_min, ts->pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TOUCH_MAJOR, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_PRESSURE, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_WIDTH_MAJOR, ts->pdata->abs_width_min, ts->pdata->abs_width_max, 0, 0);

	if (input_register_device(ts->sr_input_dev)) {
		input_free_device(ts->sr_input_dev);
		E("[SR]%s: Unable to register %s input device\n",
		  __func__, ts->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t himax_get_en_sr(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;

	if (ts->sr_input_dev) {
		count += scnprintf(buf + count, PAGE_SIZE, "%s \n", ts->sr_input_dev->name);
	} else
		count += scnprintf(buf + count, PAGE_SIZE, "0\n");
	return count;
}
static ssize_t himax_set_en_sr(struct device *dev, struct device_attribute * attr,
			       const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0]) {
		if(ts_data->sr_input_dev)
			I("[SR]%s: SR device already exist!\n", __func__);
		else
			I("[SR]%s: SR touch device enable result:%X\n", __func__, register_sr_touch_device());
	}
	return count;
}
static DEVICE_ATTR(sr_en, (S_IWUSR | S_IRUGO), himax_get_en_sr, himax_set_en_sr);
#endif	/* #if defined(HTC_FEATURE) */

static ssize_t himax_debug_level_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts_data;
	ssize_t ret = 0;

	ts_data = private_ts;
	if(!HX_PROC_SEND_FLAG)
		{
			ret += scnprintf(buf, PAGE_SIZE, "%d\n", ts_data->debug_log_level);
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_debug_level_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts;
	char buf_tmp[12]= {0};
	int i;

	if (len >= 12)
	{
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len))
	{
		return -EFAULT;
	}
	ts = private_ts;

	ts->debug_log_level = 0;
	for(i=0; i<len-1; i++)
	{
		if( buf_tmp[i]>='0' && buf_tmp[i]<='9' )
			ts->debug_log_level |= (buf_tmp[i]-'0');
		else if( buf_tmp[i]>='A' && buf_tmp[i]<='F' )
			ts->debug_log_level |= (buf_tmp[i]-'A'+10);
		else if( buf_tmp[i]>='a' && buf_tmp[i]<='f' )
			ts->debug_log_level |= (buf_tmp[i]-'a'+10);

		if(i!=len-2)
			ts->debug_log_level <<= 4;
	}

	if (ts->debug_log_level & BIT(3)) {
		if (ts->pdata->screenWidth > 0 && ts->pdata->screenHeight > 0 &&
		 (ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0 &&
		 (ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
			ts->widthFactor = (ts->pdata->screenWidth << SHIFTBITS)/(ts->pdata->abs_x_max - ts->pdata->abs_x_min);
			ts->heightFactor = (ts->pdata->screenHeight << SHIFTBITS)/(ts->pdata->abs_y_max - ts->pdata->abs_y_min);
			if (ts->widthFactor > 0 && ts->heightFactor > 0)
				ts->useScreenRes = 1;
			else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else
			I("Enable finger debug with raw position mode!\n");
	} else {
		ts->useScreenRes = 0;
		ts->widthFactor = 0;
		ts->heightFactor = 0;
	}

	return len;
}

static struct file_operations himax_proc_debug_level_ops =
{
	.owner = THIS_MODULE,
	.read = himax_debug_level_read,
	.write = himax_debug_level_write,
};

static ssize_t himax_vendor_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if(!HX_PROC_SEND_FLAG)
		{
			ret += scnprintf(buf, PAGE_SIZE, "HX8527E44_TXD.ver.0A.0E.%x\n", ts_data->vendor_config_ver);

			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static struct file_operations himax_proc_vendor_ops =
{
	.owner = THIS_MODULE,
	.read = himax_vendor_read,
};

static ssize_t himax_attn_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	if(!HX_PROC_SEND_FLAG)
		{
			scnprintf(buf, PAGE_SIZE, "attn = %x\n", himax_int_gpio_read_cap(ts_data->pdata->gpio_irq));
			ret = strlen(buf) + 1;
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static struct file_operations himax_proc_attn_ops =
{
	.owner = THIS_MODULE,
	.read = himax_attn_read,
};

static ssize_t himax_int_en_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d ", ts->irq_enabled);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
			HX_PROC_SEND_FLAG=1;
		}
		else
			HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_int_en_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[12]= {0};
	int value, ret=0;

	if (len >= 12)
	{
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len))
	{
		return -EFAULT;
	}

	if (buf_tmp[0] == '0')
		value = false;
	else if (buf_tmp[0] == '1')
		value = true;
	else
		return -EINVAL;
	if (value) {
				if(HX_INT_IS_EDGE)
				{
#ifdef MTK
#ifdef CONFIG_OF_TOUCH
					himax_int_enable_cap(ts->client->irq,1,true);
#else
					//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
					//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
					mt_eint_registration(ts->client->irq, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
#endif
#endif
				}
				else
				{
#ifdef MTK
#ifdef CONFIG_OF_TOUCH
					himax_int_enable_cap(ts->client->irq,1,true);
#else
					//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
					//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
					mt_eint_registration(ts->client->irq, EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 1);
#endif
#endif
				}
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
		}
	} else {
		himax_int_enable_cap(ts->client->irq,0,true);
		free_irq(ts->client->irq, ts);
		ts->irq_enabled = 0;
	}

	return len;
}

static struct file_operations himax_proc_int_en_ops =
{
	.owner = THIS_MODULE,
	.read = himax_int_en_read,
	.write = himax_int_en_write,
};

static ssize_t himax_layout_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d ", ts->pdata->abs_x_min);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d ", ts->pdata->abs_x_max);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d ", ts->pdata->abs_y_min);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d ", ts->pdata->abs_y_max);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_layout_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	unsigned long value;
	int layout[4] = {0};
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	for (i = 0; i < 20; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				I("buffer size is over 5 char\n");
				return len;
			}
			j = i + 1;
			if (k < 4) {
				ret = kstrtoul(buf_tmp, 10, &value);
				layout[k++] = value;
			}
		}
	}
	if (k == 4) {
		ts->pdata->abs_x_min=layout[0];
		ts->pdata->abs_x_max=layout[1];
		ts->pdata->abs_y_min=layout[2];
		ts->pdata->abs_y_max=layout[3];
		I("%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
		input_unregister_device(ts->input_dev);
		himax_input_register(ts);
	} else
		I("ERR@%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	return len;
}

static struct file_operations himax_proc_layout_ops =
{
	.owner = THIS_MODULE,
	.read = himax_layout_read,
	.write = himax_layout_write,
};

#ifdef HX_TP_PROC_RESET
static ssize_t himax_reset_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[12];

	if (len >= 12)
	{
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len))
	{
		return -EFAULT;
	}

	if (buf_tmp[0] == '1')
		himax_HW_reset_cap(true,false);
	//else if (buf_tmp[0] == '2')
	//	himax_HW_reset_cap(true,true);
	//else if (buf_tmp[0] == '3')
	//	himax_HW_reset_cap(false,true);
	//else if (buf_tmp[0] == '4')
	//	ESD_HW_REST_cap();
	return len;
}

static struct file_operations himax_proc_reset_ops =
{
	.owner = THIS_MODULE,
	.write = himax_reset_write,
};
#endif

#ifdef HX_TP_PROC_DIAG
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}
static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}
static uint8_t getXChannel(void)
{
	return x_channel;
}
static uint8_t getYChannel(void)
{
	return y_channel;
}
static uint8_t getDiagCommand(void)
{
	return diag_command;
}
static void setXChannel(uint8_t x)
{
	x_channel = x;
}
static void setYChannel(uint8_t y)
{
	y_channel = y;
}
static void setMutualBuffer(void)
{
	diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

#ifdef HX_TP_PROC_2T2R
static uint8_t *getMutualBuffer_2(void)
{
	return diag_mutual_2;
}
static uint8_t getXChannel_2(void)
{
	return x_channel_2;
}
static uint8_t getYChannel_2(void)
{
	return y_channel_2;
}
static void setXChannel_2(uint8_t x)
{
	x_channel_2 = x;
}
static void setYChannel_2(uint8_t y)
{
	y_channel_2 = y;
}
static void setMutualBuffer_2(void)
{
	diag_mutual_2 = kzalloc(x_channel_2 * y_channel_2 * sizeof(uint8_t), GFP_KERNEL);
}
#endif
static void *himax_diag_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos>=1) return NULL;
	return (void *)((unsigned long) *pos+1);
}

static void *himax_diag_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}
static void himax_diag_seq_stop(struct seq_file *s, void *v)
{
}
static int himax_diag_seq_read(struct seq_file *s, void *v)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
#ifdef HX_TP_PROC_2T2R
	if(Is_2T2R && diag_command == 4)
	{
		mutual_num	= x_channel_2 * y_channel_2;
		self_num	= x_channel_2 + y_channel_2; //don't add KEY_COUNT
		width		= x_channel_2;
		seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel_2, y_channel_2);
	}
	else
#endif
	{
		mutual_num	= x_channel * y_channel;
		self_num	= x_channel + y_channel; //don't add KEY_COUNT
		width		= x_channel;
		seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	}

	// start to show out the raw data in adb shell
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command <= 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, " %3d\n", diag_self[width + loop_i/width]);
			}
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					seq_printf(s, "\n");
			}
#ifdef HX_EN_SEL_BUTTON
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
					seq_printf(s, "%4d", diag_self[HX_RX_NUM + HX_TX_NUM + loop_i]);
#endif
#ifdef HX_TP_PROC_2T2R
		}else if(Is_2T2R && diag_command == 4 ) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual_2[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, " %3d\n", diag_self[width + loop_i/width]);
			}
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					seq_printf(s, "\n");
			}

#ifdef HX_EN_SEL_BUTTON
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
				seq_printf(s, "%4d", diag_self[HX_RX_NUM_2 + HX_TX_NUM_2 + loop_i]);
#endif
#endif
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
					seq_printf(s, "\n");
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, "\n");
			}
		}
		seq_printf(s, "ChannelEnd");
		seq_printf(s, "\n");
	} else if (diag_command == 7) {
		for (loop_i = 0; loop_i < 128 ;loop_i++) {
			if ((loop_i % 16) == 0)
				seq_printf(s, "LineStart:");
				seq_printf(s, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15)
				seq_printf(s, "\n");
		}
	}
	return count;
}

static struct seq_operations himax_diag_seq_ops =
{
	.start	= himax_diag_seq_start,
	.next	= himax_diag_seq_next,
	.stop	= himax_diag_seq_stop,
	.show	= himax_diag_seq_read,
};
static int himax_diag_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_diag_seq_ops);
};
static ssize_t himax_diag_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	const uint8_t command_ec_128_raw_flag 		= 0x02;
	const uint8_t command_ec_24_normal_flag 	= 0x00;
	uint8_t command_ec_128_raw_baseline_flag 	= 0x01;
	uint8_t command_ec_128_raw_bank_flag 		= 0x03;
	uint8_t command_F1h[2] = {0xF1, 0x00};
	char messages[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len))
	{
		return -EFAULT;
	}

	diag_command = messages[0] - '0';

	I("[Himax]diag_command=0x%x\n",diag_command);
	if (diag_command == 0x01)	{//DC
		command_F1h[1] = command_ec_128_raw_baseline_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x02) {//IIR
		command_F1h[1] = command_ec_128_raw_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x03) {	//BANK
		command_F1h[1] = command_ec_128_raw_bank_flag;	//0x03
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x04 ) { // 2T3R IIR
		command_F1h[1] = 0x04; //2T3R IIR
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
	} else if (diag_command == 0x00) {//Disable
		command_F1h[1] = command_ec_24_normal_flag;
		i2c_himax_write(private_ts->client, command_F1h[0] ,&command_F1h[1], 1, DEFAULT_RETRY_CNT);
		touch_monitor_stop_flag = touch_monitor_stop_limit;
	}

	//coordinate dump start
	else if (diag_command == 0x08)	{
		coordinate_fn = filp_open(DIAG_COORDINATE_FILE,O_CREAT | O_WRONLY | O_APPEND | O_TRUNC,0666);
		if (IS_ERR(coordinate_fn))
		{
			E("%s: coordinate_dump_file_create error\n", __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn,NULL);
		}
		coordinate_dump_enable = 1;
	}
	else if (diag_command == 0x09){
		coordinate_dump_enable = 0;

		if (!IS_ERR(coordinate_fn))
		{
			filp_close(coordinate_fn,NULL);
		}
	}
	//coordinate dump end
	else{
			E("[Himax]Diag command error!diag_command=0x%x\n",diag_command);
	}
	return len;
}

static struct file_operations himax_proc_diag_ops =
{
	.owner = THIS_MODULE,
	.open = himax_diag_proc_open,
	.read = seq_read,
	.write = himax_diag_write,
};
#endif

#ifdef HX_TP_PROC_REGISTER
static ssize_t himax_register_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i,loop_j;
	uint8_t inData[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(inData, 0x00, sizeof(inData));

	I("Himax multi_register_command = %d \n",multi_register_command);
	if(!HX_PROC_SEND_FLAG)
		{
			if (multi_register_command == 1) {
				base = 0;

				for(loop_i = 0; loop_i < 6; loop_i++) {
					if (multi_register[loop_i] != 0x00) {
						if (multi_cfg_bank[loop_i] == 1) {//config bank register
							outData[0] = 0x14;
							i2c_himax_write(private_ts->client, 0x8C ,&outData[0], 1, DEFAULT_RETRY_CNT);
							msleep(10);

							outData[0] = 0x00;
							outData[1] = multi_register[loop_i];
							i2c_himax_write(private_ts->client, 0x8B ,&outData[0], 2, DEFAULT_RETRY_CNT);
							msleep(10);

							i2c_himax_read(private_ts->client, 0x5A, inData, 128, DEFAULT_RETRY_CNT);

							outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0x8C ,&outData[0], 1, DEFAULT_RETRY_CNT);

							for(loop_j=0; loop_j<128; loop_j++)
								multi_value[base++] = inData[loop_j];
						} else {//normal register
							i2c_himax_read(private_ts->client, multi_register[loop_i], inData, 128, DEFAULT_RETRY_CNT);

							for(loop_j=0; loop_j<128; loop_j++)
								multi_value[base++] = inData[loop_j];
						}
					}
				}

				base = 0;
				for(loop_i = 0; loop_i < 6; loop_i++) {
					if (multi_register[loop_i] != 0x00) {
						if (multi_cfg_bank[loop_i] == 1)
							ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Register: FE(%x)\n", multi_register[loop_i]);
						else
							ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Register: %x\n", multi_register[loop_i]);

						for (loop_j = 0; loop_j < 128; loop_j++) {
							ret += scnprintf(buf + ret, PAGE_SIZE - ret, "0x%2.2X ", multi_value[base++]);
							if ((loop_j % 16) == 15)
								ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
						}
					}
				}
				return ret;
			}

			if (config_bank_reg) {
				I("%s: register_command = FE(%x)\n", __func__, register_command);

				//Config bank register read flow.

				outData[0] = 0x14;
				i2c_himax_write(private_ts->client, 0x8C,&outData[0], 1, DEFAULT_RETRY_CNT);

				msleep(10);

				outData[0] = 0x00;
				outData[1] = register_command;
				i2c_himax_write(private_ts->client, 0x8B,&outData[0], 2, DEFAULT_RETRY_CNT);
		    	msleep(10);

				i2c_himax_read(private_ts->client, 0x5A, inData, 128, DEFAULT_RETRY_CNT);
		    	msleep(10);

				outData[0] = 0x00;
				i2c_himax_write(private_ts->client, 0x8C,&outData[0], 1, DEFAULT_RETRY_CNT);
			} else {
				if (i2c_himax_read(private_ts->client, register_command, inData, 128, DEFAULT_RETRY_CNT) < 0)
					return ret;
			}

			if (config_bank_reg)
				ret += scnprintf(buf, PAGE_SIZE - ret, "command: FE(%x)\n", register_command);
			else
				ret += scnprintf(buf, PAGE_SIZE - ret, "command: %x\n", register_command);

			for (loop_i = 0; loop_i < 128; loop_i++) {
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "0x%2.2X ", inData[loop_i]);
				if ((loop_i % 16) == 15)
					ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
			}
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
			HX_PROC_SEND_FLAG=1;
		}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_register_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[6], length = 0;
	unsigned long result    = 0;
	uint8_t loop_i          = 0;
	uint16_t base           = 5;
	uint8_t write_da[128];
	uint8_t outData[5];
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	I("himax %s \n",buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		I("himax multi register enter\n");

		multi_register_command = 1;

		base 	= 2;
		loop_i 	= 0;

		while(true) {
			if (buf[base] == '\n')
				break;

			if (loop_i >= 6 )
				break;

			if (buf[base] == ':' && buf[base+1] == 'x' && buf[base+2] == 'F' &&
					buf[base+3] == 'E' && buf[base+4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!kstrtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!kstrtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		I("========================== \n");
		for(loop_i = 0; loop_i < 6; loop_i++)
			I("%d,%d:",multi_register[loop_i],multi_cfg_bank[loop_i]);
		I("\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			if (buf[3] == 'F' && buf[4] == 'E') {//Config bank register
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!kstrtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 7;

				I("CMD: FE(%x)\n", register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!kstrtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 5;
				I("CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x14;
							i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
              				msleep(10);

							outData[0] = 0x00;
							outData[1] = register_command;
							i2c_himax_write(private_ts->client, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);

							msleep(10);
							i2c_himax_write(private_ts->client, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);

							msleep(10);
              				outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);

							I("CMD: FE(%x), %x, %d\n", register_command,write_da[0], length);
						} else {
							i2c_himax_write(private_ts->client, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
							I("CMD: %x, %x, %d\n", register_command,write_da[0], length);
						}
					}
					I("\n");
					return len;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!kstrtoul(buf_tmp, 16, &result)) {
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return len;
}

static struct file_operations himax_proc_register_ops =
{
	.owner = THIS_MODULE,
	.read = himax_register_read,
	.write = himax_register_write,
};
#endif

#ifdef HX_TP_PROC_DEBUG
static ssize_t himax_debug_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
	{
		if (debug_level_cmd == 't')
		{
			if (fw_update_complete)
			{
				ret += scnprintf(buf, PAGE_SIZE - ret, "FW Update Complete ");
			}
			else
			{
				ret += scnprintf(buf, PAGE_SIZE - ret, "FW Update Fail ");
			}
		}
		else if (debug_level_cmd == 'h')
		{
			if (handshaking_result == 0)
			{
				ret += scnprintf(buf, PAGE_SIZE - ret, "Handshaking Result = %d (MCU Running)\n",handshaking_result);
			}
			else if (handshaking_result == 1)
			{
				ret += scnprintf(buf, PAGE_SIZE - ret, "Handshaking Result = %d (MCU Stop)\n",handshaking_result);
			}
			else if (handshaking_result == 2)
			{
				ret += scnprintf(buf, PAGE_SIZE - ret, "Handshaking Result = %d (I2C Error)\n",handshaking_result);
			}
			else
			{
				ret += scnprintf(buf, PAGE_SIZE - ret, "Handshaking Result = error \n");
			}
		}
		else if (debug_level_cmd == 'v')
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FW_VER = ");
	           ret += scnprintf(buf + ret, "0x%2.2X, %2.2X \n",private_ts->vendor_fw_ver_H,private_ts->vendor_fw_ver_L);

			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "CONFIG_VER = ");
	           ret += scnprintf(buf + ret, "0x%2.2X \n",private_ts->vendor_config_ver);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
		}
		else if (debug_level_cmd == 'd')
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Himax Touch IC Information :\n");
			if (IC_TYPE == HX_85XX_D_SERIES_PWON)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Type : D\n");
			}
			else if (IC_TYPE == HX_85XX_E_SERIES_PWON)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Type : E\n");
			}
			else if (IC_TYPE == HX_85XX_ES_SERIES_PWON)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Type : ES\n");
			}
			else
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Type error.\n");
			}

			if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Checksum : SW\n");
			}
			else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Checksum : HW\n");
			}
			else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Checksum : CRC\n");
			}
			else
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "IC Checksum error.\n");
			}

			if (HX_INT_IS_EDGE)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Interrupt : EDGE TIRGGER\n");
			}
			else
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Interrupt : LEVEL TRIGGER\n");
			}

			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "RX Num : %d\n",HX_RX_NUM);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "TX Num : %d\n",HX_TX_NUM);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "BT Num : %d\n",HX_BT_NUM);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "X Resolution : %d\n",HX_X_RES);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Y Resolution : %d\n",HX_Y_RES);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Max Point : %d\n",HX_MAX_PT);
#ifdef HX_TP_PROC_2T2R
			if(Is_2T2R)
			{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "2T2R panel\n");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "RX Num_2 : %d\n",HX_RX_NUM_2);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "TX Num_2 : %d\n",HX_TX_NUM_2);
			}
#endif
		}

		else if (debug_level_cmd == 'i')
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Himax Touch Driver Version:\n");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s \n", HIMAX_DRIVER_VER);
		}
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;
	return ret;
}

static ssize_t himax_debug_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct file* hx_filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if ( buf[0] == 'h') //handshaking
	{
		debug_level_cmd = buf[0];

		himax_int_enable_cap(private_ts->client->irq,0,true);

		handshaking_result = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail

		himax_int_enable_cap(private_ts->client->irq,1,true);

		return len;
	}

	else if ( buf[0] == 'v') //firmware version
	{
		debug_level_cmd = buf[0];
		himax_read_FW_ver(true);
		return len;
	}

	else if ( buf[0] == 'd') //test
	{
		debug_level_cmd = buf[0];
		return len;
	}

	else if ( buf[0] == 'i') //driver version
	{
		debug_level_cmd = buf[0];
		return len;
	}

	else if (buf[0] == 't')
	{

		himax_int_enable_cap(private_ts->client->irq,0,true);
		wake_lock(&private_ts->ts_flash_wake_lock);
#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
		cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

		debug_level_cmd 		= buf[0];
		fw_update_complete		= false;

		memset(fileName, 0, 128);
		// parse the file name
		scnprintf(fileName, len-2, "%s", &buf[2]);
		I("%s: upgrade from file(%s) start!\n", __func__, fileName);
		// open file
		hx_filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(hx_filp))
		{
			E("%s: open firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return len;
		}
		oldfs = get_fs();
		set_fs(get_ds());

		// read the latest firmware binary file
		result=hx_filp->f_op->read(hx_filp,upgrade_fw,sizeof(upgrade_fw), &hx_filp->f_pos);
		if (result < 0)
		{
			E("%s: read firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return len;
		}

		set_fs(oldfs);
		filp_close(hx_filp, NULL);

		I("%s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

		if (result > 0)
		{
		// start to upgrade
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset_cap(false,true);
#endif
			if (fts_ctpm_fw_upgrade_with_fs_cap(upgrade_fw, result, true) == 0)
			{
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			}
			else
			{
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			goto firmware_upgrade_done;
			//return len;
		}
	}

	firmware_upgrade_done:

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true,true);
#endif
	wake_unlock(&private_ts->ts_flash_wake_lock);
	himax_int_enable_cap(private_ts->client->irq,1,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif
	//todo himax_chip->tp_firmware_upgrade_proceed = 0;
	//todo himax_chip->suspend_state = 0;
	//todo enable_irq(himax_chip->irq);
	return len;
}

static struct file_operations himax_proc_debug_ops =
{
	.owner = THIS_MODULE,
	.read = himax_debug_read,
	.write = himax_debug_write,
};
#endif

#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)

static uint8_t getFlashCommand(void)
{
	return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
	return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
	return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
	return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
	return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
	return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
	return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
	return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
	return flash_dump_going;
}

static void setFlashBuffer(void)
{
	flash_buffer = kzalloc(FLASH_SIZE * sizeof(uint8_t), GFP_KERNEL);
	if(flash_buffer == NULL) {
		E("%s: flash_buffer kzalloc fail!", __func__);
	}
}

static void setSysOperation(uint8_t operation)
{
	sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
	flash_progress = progress;
	//I("setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress,flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
	flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
	flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
	flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
	flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
	flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
	flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
	flash_dump_going = going;
}

static void himax_ts_flash_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);

	uint8_t page_tmp[128];
	uint8_t x59_tmp[4] = {0,0,0,0};
	int i=0, j=0, k=0, l=0,/* j_limit = 0,*/ buffer_ptr = 0/*, flash_end_count = 0*/;
	uint8_t local_flash_command = 0;
	uint8_t sector = 0;
	uint8_t page = 0;

	/*uint8_t xAA_command[2] = {0xAA,0x00};*/
	uint8_t x81_command[2] = {0x81,0x00};
	uint8_t x82_command[2] = {0x82,0x00};
	uint8_t x35_command[2] = {0x35,0x00};
	uint8_t x43_command[4] = {0x43,0x00,0x00,0x00};
	uint8_t x44_command[4] = {0x44,0x00,0x00,0x00};
	uint8_t x45_command[5] = {0x45,0x00,0x00,0x00,0x00};
	uint8_t x46_command[2] = {0x46,0x00};
	/*uint8_t x4A_command[2] = {0x4A,0x00};*/
	uint8_t x4D_command[2] = {0x4D,0x00};
	/*uint8_t x59_command[2] = {0x59,0x00};*/

	himax_int_enable_cap(private_ts->client->irq,0,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

	setFlashDumpGoing(true);

	sector = getFlashDumpSector();
	page = getFlashDumpPage();

	local_flash_command = getFlashCommand();

#ifdef HX_RST_PIN_FUNC
	if(local_flash_command<0x0F)
		himax_HW_reset_cap(false,true);
#endif

	//if( i2c_himax_master_write(ts->client, xAA_command, 1, 3) < 0 )//sleep out
	//{
	//	E("%s i2c write AA fail.\n",__func__);
	//	goto Flash_Dump_i2c_transfer_error;
	//}
	//msleep(120);

	if ( i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0 )//sleep out
	{
		E("%s i2c write 81 fail.\n",__func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(120);
	if ( i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0 )
	{
		E("%s i2c write 82 fail.\n",__func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(100);

	I("%s: local_flash_command = %d enter.\n", __func__,local_flash_command);

	if ((local_flash_command == 1 || local_flash_command == 2)|| (local_flash_command==0x0F))
	{
		x43_command[1] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for( i=0 ; i<8 ;i++)
		{
			for(j=0 ; j<32 ; j++)
			{
				//I(" Step 2 i=%d , j=%d %s\n",i,j,__func__);
				//read page start
				for(k=0; k<128; k++)
				{
					page_tmp[k] = 0x00;
				}
				for(k=0; k<32; k++)
				{
					x44_command[1] = k;
					x44_command[2] = j;
					x44_command[3] = i;
					if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
					{
						E("%s i2c write 44 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0)
					{
						E("%s i2c write 46 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s i2c write 59 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					for(l=0; l<4; l++)
					{
						page_tmp[k*4+l] = x59_tmp[l];
					}
					//msleep(10);
				}
				//read page end

				for(k=0; k<128; k++)
				{
					flash_buffer[buffer_ptr++] = page_tmp[k];

				}
				setFlashDumpProgress(i*32 + j);
			}
		}
	}
	else if (local_flash_command == 3)
	{
		x43_command[1] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for(i=0; i<128; i++)
		{
			page_tmp[i] = 0x00;
		}

		for(i=0; i<32; i++)
		{
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;

			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 46 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			//msleep(2);
			if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 59 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			//msleep(2);
			for(j=0; j<4; j++)
			{
				page_tmp[i*4+j] = x59_tmp[j];
			}
			//msleep(10);
		}
		//read page end
		for(i=0; i<128; i++)
		{
			flash_buffer[buffer_ptr++] = page_tmp[i];
		}
	}
	else if (local_flash_command == 4)
	{
		//page write flow.
		//I("%s: local_flash_command = 4, enter.\n", __func__);

		// unlock flash
		himax_lock_flash(0);

		msleep(50);

		// page erase
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x02;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 44 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);
		if ( i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 4D fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		// enter manual mode

		x35_command[1] = 0x01;
		if( i2c_himax_write(ts->client, x35_command[0],&x35_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 35 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}

		msleep(100);

		// flash enable
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// set flash address
		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 44 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// manual mode command : 47 to latch the flash address when page address change.
		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x0D;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		for(i=0; i<32; i++)
		{
			I("himax :i=%d \n",i);
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;
			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x45_command[1] = flash_buffer[i*4 + 0];
			x45_command[2] = flash_buffer[i*4 + 1];
			x45_command[3] = flash_buffer[i*4 + 2];
			x45_command[4] = flash_buffer[i*4 + 3];
			if ( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 45 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			// manual mode command : 48 ,data will be written into flash buffer
			x43_command[1] = 0x01;
			x43_command[2] = 0x0D;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);
		}

		// manual mode command : 49 ,program data from flash buffer to this page
		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x05;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// flash disable
		x43_command[1] = 0x00;
		if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 43 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		// leave manual mode
		x35_command[1] = 0x01;
		if( i2c_himax_write(ts->client, x35_command[0],&x35_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
		{
			E("%s i2c write 35 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}

		msleep(10);

		// lock flash
		himax_lock_flash(1);
		msleep(50);

		buffer_ptr = 128;
		I("Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
	}

	I("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
	if(local_flash_command==0x01)
		{
			I(" buffer_ptr = %d \n",buffer_ptr);

			for (i = 0; i < buffer_ptr; i++)
			{
				I("%2.2X ", flash_buffer[i]);

				if ((i % 16) == 15)
				{
					I("\n");
				}
			}
			I("End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		}
	i2c_himax_master_write(ts->client, x43_command, 1, 3);
	msleep(50);

	if (local_flash_command == 2)
	{
		struct file *fn;

		fn = filp_open(FLASH_DUMP_FILE,O_CREAT | O_WRONLY ,0);
		if (!IS_ERR(fn))
		{
			fn->f_op->write(fn,flash_buffer,buffer_ptr*sizeof(uint8_t),&fn->f_pos);
			filp_close(fn,NULL);
		}
	}

#ifdef HX_RST_PIN_FUNC
	if(local_flash_command<0x0F)
		himax_HW_reset_cap(true,true);
#endif

	himax_int_enable_cap(private_ts->client->irq,1,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif
	setFlashDumpGoing(false);

	setFlashDumpComplete(1);
	setSysOperation(0);
	return;

	Flash_Dump_i2c_transfer_error:

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset_cap(true,true);
#endif

	himax_int_enable_cap(private_ts->client->irq,1,true);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
#endif
	setFlashDumpGoing(false);
	setFlashDumpComplete(0);
	setFlashDumpFail(1);
	setSysOperation(0);
	return;
}
#endif	/* #if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP) */


#ifdef HX_TP_PROC_FLASH_DUMP
static ssize_t himax_flash_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step=0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	I("flash_progress = %d \n",local_flash_progress);
	if(!HX_PROC_SEND_FLAG)
	{

		if (local_flash_fail)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "FlashStart:Fail \n");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashEnd");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		if (!local_flash_complete)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashStart:Ongoing:0x%2.2x \n",flash_progress);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashEnd");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		if (local_flash_command == 1 && local_flash_complete)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashStart:Complete \n");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashEnd");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		if (local_flash_command == 3 && local_flash_complete)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashStart: \n");
			for(loop_i = 0; loop_i < 128; loop_i++)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "x%2.2x", flash_buffer[loop_i]);
				if ((loop_i % 16) == 15)
				{
					ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
				}
			}
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashEnd");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
			HX_PROC_SEND_FLAG=1;
			return ret;
		}

		//flash command == 0 , report the data
		local_flash_read_step = getFlashReadStep();

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashStart:%2.2x \n",local_flash_read_step);

		for (loop_i = 0; loop_i < 1024; loop_i++)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "x%2.2X", flash_buffer[local_flash_read_step*1024 + loop_i]);

			if ((loop_i % 16) == 15)
			{
				ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
			}
		}

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "FlashEnd");
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "\n");
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;
	return ret;
}

static ssize_t himax_flash_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}
	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	I("%s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1)
	{
		E("%s: SYS is busy , return!\n", __func__);
		return len;
	}

	if (buf[0] == '0')
	{
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x')
		{
			memcpy(buf_tmp, buf + 3, 2);
			I("%s: read_Step = %s\n", __func__, buf_tmp);
			if (!kstrtoul(buf_tmp, 16, &result))
			{
				I("%s: read_Step = %lu \n", __func__, result);
				setFlashReadStep(result);
			}
		}
	}
	else if (buf[0] == '1')
	{
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '2')
	{
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '3')
	{
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpSector(result);
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpPage(result);
		}

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '4')
	{
		I("%s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpSector(result);
		}
		else
		{
			E("%s: command 4 , sector error.\n", __func__);
			return len;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!kstrtoul(buf_tmp, 16, &result))
		{
			setFlashDumpPage(result);
		}
		else
		{
			E("%s: command 4 , page error.\n", __func__);
			return len;
		}

		base = 11;

		I("=========Himax flash page buffer start=========\n");
		for(loop_i=0;loop_i<128;loop_i++)
		{
			memcpy(buf_tmp, buf + base, 2);
			if (!kstrtoul(buf_tmp, 16, &result))
			{
				flash_buffer[loop_i] = result;
				I("%d ",flash_buffer[loop_i]);
				if (loop_i % 16 == 15)
				{
					I("\n");
				}
			}
			base += 3;
		}
		I("=========Himax flash page buffer end=========\n");

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	return len;
}

static struct file_operations himax_proc_flash_ops =
{
	.owner = THIS_MODULE,
	.read = himax_flash_read,
	.write = himax_flash_write,
};
#endif	/* #ifdef HX_TP_PROC_FLASH_DUMP */

#ifdef HX_TP_PROC_SELF_TEST
static int himax_chip_self_test(void)
{
	uint8_t cmdbuf[11];
	uint8_t valuebuf[16];
	int i=0, pf_value=0x00;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(valuebuf, 0x00, sizeof(valuebuf));

	//i2c_himax_write(private_ts->client, 0x82,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	//msleep(120);
	//i2c_himax_write(private_ts->client, 0x80,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	//msleep(120);

	cmdbuf[0] = 0x06;
	i2c_himax_write(private_ts->client, 0xF1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, 0x83,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, 0x81,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	//msleep(2000);

	//i2c_himax_write(private_ts->client, 0x82,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	//msleep(120);

	//i2c_himax_write(private_ts->client, 0x80,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	//msleep(120);

	//cmdbuf[0] = 0x00;
	//i2c_himax_write(private_ts->client, 0xF1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	//msleep(120);

read_result_again:
	i2c_himax_read(private_ts->client, 0xB1, valuebuf, 8, DEFAULT_RETRY_CNT);
	msleep(10);

	for(i=0;i<8;i++) {
		I("[Himax]: After slf test 0xB1 buff_back[%d] = 0x%x\n",i,valuebuf[i]);
	}

	msleep(30);

	if (valuebuf[0]==0xAA) {
		I("[Himax]: self-test pass\n");
		pf_value = 0x0;
	} else if((valuebuf[0] == 0xf1) || (valuebuf[0] == 0xf2) || (valuebuf[0] == 0xf3)){
		E("[Himax]: self-test fail\n");
		pf_value = 0x1;
	}
	else{
		I("[Himax]:Not ready yet,wait \n");
		goto read_result_again;
	}

	return pf_value;
}

static ssize_t himax_self_test_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int val=0x00;
	int ret = 0;

	#ifdef HX_CHIP_STATUS_MONITOR
	int j=0;
       #endif
     himax_int_enable_cap(private_ts->client->irq,0,false);

     #ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT=0;
		if(HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
		{
			for(j=0; j<100; j++)
				{
					if(HX_ON_HAND_SHAKING==0)//chip on hand shaking end
						{
							I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,j);
							break;
						}
					else
						msleep(1);
				}

		}

		cancel_delayed_work_sync(&private_ts->himax_chip_monitor);

	#endif

	self_test_inter_flag= 1;

	msleep(10);

	val = himax_chip_self_test();

	//#ifdef HX_RST_PIN_FUNC
		//	himax_HW_reset_cap(true,false);
	//#endif

	//himax_int_enable_cap(private_ts->client->irq,1,true);
	#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES*HZ);
	#endif
	msleep(100);

	if(!HX_PROC_SEND_FLAG)
	{
		if (val == 0x00) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "Pass\n");
		} else {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "Failed\n");
		}
		HX_PROC_SEND_FLAG = 1;
	}
	else
		HX_PROC_SEND_FLAG = 0;

	self_test_inter_flag= 0;
	return ret;
}

static struct file_operations himax_proc_self_test_ops =
{
	.owner = THIS_MODULE,
	.read = himax_self_test_read,
};

#endif

#ifdef HX_TP_PROC_HITOUCH
static ssize_t himax_hitouch_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	int ret = 0;
	if(!HX_PROC_SEND_FLAG)
	{
		if(hitouch_command == 0)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "Himax Touch Driver Version:\n");
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "%s \n", HIMAX_DRIVER_VER);
		}
		else if(hitouch_command == 1)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "hitouch_is_connect = true\n");
		}
		else if(hitouch_command == 2)
		{
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,  "hitouch_is_connect = false\n");
		}
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

//-----------------------------------------------------------------------------------
//himax_hitouch_store_cap
//command 0 : Get Driver Version
//command 1 : Hitouch Connect
//command 2 : Hitouch Disconnect
//-----------------------------------------------------------------------------------
static ssize_t himax_hitouch_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0')
	{
		hitouch_command = 0;
	}
	else if(buf[0] == '1')
	{
		hitouch_command = 1;
		hitouch_is_connect = true;
		I("hitouch_is_connect = true\n");
	}
	else if(buf[0] == '2')
	{
		hitouch_command = 2;
		hitouch_is_connect = false;
		I("hitouch_is_connect = false\n");
	}
	return len;
}

static struct file_operations himax_proc_hitouch_ops =
{
	.owner = THIS_MODULE,
	.read = himax_hitouch_read,
	.write = himax_hitouch_write,
};
#endif

#ifdef HX_DOT_VIEW
static ssize_t himax_cover_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
	{
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", ts->cover_enable);
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_cover_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0')
		ts->cover_enable = 0;
	else if(buf[0] == '1')
		ts->cover_enable = 1;
	else
		return -EINVAL;
	himax_set_cover_func(ts->cover_enable);

	I("%s: cover_enable = %d.\n", __func__, ts->cover_enable);

	return len;
}

static struct file_operations himax_proc_cover_ops =
{
	.owner = THIS_MODULE,
	.read = himax_cover_read,
	.write = himax_cover_write,
};
#endif

#ifdef HX_SMART_WAKEUP
#if 0
static ssize_t himax_SMWP_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(!HX_PROC_SEND_FLAG)
	{
		ret = scnprintf(buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);
		HX_PROC_SEND_FLAG=1;
	}
	else
		HX_PROC_SEND_FLAG=0;

	return ret;
}

static ssize_t himax_SMWP_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i =0;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0')
		ts->SMWP_enable = 0;
	else if(buf[0] == '1')
		ts->SMWP_enable = 1;
	else
		return -EINVAL;

	if(ts->SMWP_enable)
	{
		for (i=0;i<16;i++)
			{
				ts->gesture_cust_en[i]= 1;
				I("gesture en[%d]=%d \n", i, ts->gesture_cust_en[i]);
			}
	}
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, ts->SMWP_enable);

	return len;
}

static struct file_operations himax_proc_SMWP_ops =
{
	.owner = THIS_MODULE,
	.read = himax_SMWP_read,
	.write = himax_SMWP_write,
};

static ssize_t himax_GESTURE_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;

	if(HX_PROC_SEND_FLAG<16)
	{
		ret = scnprintf(buf, PAGE_SIZE, "ges_en[%d]=%d \n",HX_PROC_SEND_FLAG ,ts->gesture_cust_en[HX_PROC_SEND_FLAG]);
		HX_PROC_SEND_FLAG++;
	}
	else
	{
		HX_PROC_SEND_FLAG = 0;
		ret = 0;
	}
	return ret;
}

static ssize_t himax_GESTURE_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i =0;
	char buf[80] = {0};

	if (len >= 80)
	{
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	for (i=0;i<16;i++)
		{
			if (buf[i] == '0')
	    		ts->gesture_cust_en[i]= 0;
			else if (buf[i] == '1')
	    		ts->gesture_cust_en[i]= 1;
			else
				ts->gesture_cust_en[i]= 0;
			I("gesture en[%d]=%d \n", i, ts->gesture_cust_en[i]);
		}
	return len;
}

static struct file_operations himax_proc_Gesture_ops =
{
	.owner = THIS_MODULE,
	.read = himax_GESTURE_read,
	.write = himax_GESTURE_write,
};
#endif
static unsigned char gesture_point_readbuf[32];
static struct kobject *gesture_debug_kobj = NULL;

static ssize_t himax_gesture_enable_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	int buf_len=0;

	buf_len += scnprintf(buf, PAGE_SIZE, "%d",ts->SMWP_enable);
    return buf_len;
}

static ssize_t himax_gesture_enable_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct himax_ts_data *ts = private_ts;
	int i=0;

    sscanf(buf, "%d", &ts->SMWP_enable);

	if(ts->SMWP_enable)
	{
		for (i=0;i<16;i++)
			{
				ts->gesture_cust_en[i]= 1;
				I("gesture en[%d]=%d \n", i, ts->gesture_cust_en[i]);
			}
	}
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, ts->SMWP_enable);

    return len;
}


static ssize_t himax_gesture_debug_write(struct kobject *kobj,struct kobj_attribute *attr, const char *buf, size_t len)
{
	//struct himax_ts_data *ts = private_ts;
	//int i=0;

	I("%s: SMART_WAKEUP_debug = %c.\n", __func__, *buf);

    return len;
}




static ssize_t himax_gesture_debug_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf)
{
    ssize_t len = 0;
	int i=0;
	int j=0;
	int num_read_chars = 0;

	struct himax_ts_data *ts = private_ts;

	gesture_point_readbuf[0] = ts->gesture_id;

	gesture_point_readbuf[2] = 	gest_start_x & 0xFF;
	gesture_point_readbuf[1] = 	(gest_start_x >> 8) & 0xFF;
	gesture_point_readbuf[4] = 	gest_start_y & 0xFF;
	gesture_point_readbuf[3] = 	(gest_start_y >> 8) & 0xFF;
	gesture_point_readbuf[6] = 	gest_end_x & 0xFF;
	gesture_point_readbuf[5] = 	(gest_end_x >> 8) & 0xFF;
	gesture_point_readbuf[8] = 	gest_end_y & 0xFF;
	gesture_point_readbuf[7] = 	(gest_end_y >> 8) & 0xFF;
	gesture_point_readbuf[10] =  gest_left_top_x & 0xFF;
	gesture_point_readbuf[9] = (gest_left_top_x >> 8) & 0xFF;
	gesture_point_readbuf[12] = gest_left_top_y & 0xFF;
	gesture_point_readbuf[11] = (gest_left_top_y >> 8) & 0xFF;
	gesture_point_readbuf[14] = gest_right_down_x & 0xFF;
	gesture_point_readbuf[13] = (gest_right_down_x >> 8) & 0xFF;
	gesture_point_readbuf[16] = gest_right_down_y & 0xFF;
	gesture_point_readbuf[15] = (gest_right_down_y >> 8) & 0xFF;
	for(i=0;i<3;i++)
		{
			gesture_point_readbuf[18+i*4] = gest_key_pt_x[i] & 0xFF;
			gesture_point_readbuf[17+i*4] = (gest_key_pt_x[i] >> 8) & 0xFF;
			gesture_point_readbuf[20+i*4] = gest_key_pt_y[i] & 0xFF;
			gesture_point_readbuf[19+i*4] = (gest_key_pt_y[i] >> 8) & 0xFF;
		}
	for(j = 0;j<=15;j++)
	{
		I("frank_zhonghua:gesture_point_readbuf[%d] = %c\n",j,gesture_point_readbuf[j]);

	}
	len = num_read_chars = 29;
	memcpy(buf,gesture_point_readbuf,num_read_chars);

	I("himax_gesture_debug_show--end %d\n", num_read_chars);

    return len;
}

static struct kobj_attribute gesture_debug_attr = __ATTR(debug, 0664, himax_gesture_debug_show, himax_gesture_debug_write);
static struct kobj_attribute gesture_enable_attr = __ATTR(gesture_enable, 0664, himax_gesture_enable_show, himax_gesture_enable_store);

int himax_gesture_sysfs_init(void)
{
	int ret = -1;

	gesture_debug_kobj = kobject_create_and_add("gesture", NULL);
	if (gesture_debug_kobj == NULL) {
			ret = -ENOMEM;
			E("register sysfs failed. ret = %d\n", ret);
			return ret;
	}

	ret = sysfs_create_file(gesture_debug_kobj, &gesture_debug_attr.attr);
	if (ret) {
			E("create sysfs failed. ret = %d\n", ret);
			return ret;
	}

	ret = sysfs_create_file(gesture_debug_kobj, &gesture_enable_attr.attr);
	if (ret) {
			E("create sysfs failed. ret = %d\n", ret);
			return ret;
	}

	I("Himax_gesture_sysfs success\n");
	return ret;
}
static void himax_gesture_sysfs_deinit(void)
{
	sysfs_remove_file(gesture_debug_kobj, &gesture_debug_attr.attr);
	sysfs_remove_file(gesture_debug_kobj, &gesture_enable_attr.attr);
	kobject_del(gesture_debug_kobj);
}
#endif

#if defined(HTC_FEATURE)
static int himax_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add(KOBJ_NAME, NULL);
	if (android_touch_kobj == NULL) {
		E("%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		E("%s: create_file dev_attr_debug_level failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		E("%s: sysfs_create_file dev_attr_vendor failed\n", __func__);
		return ret;
	}

#ifdef HX_TP_SYS_RESET
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
	if (ret) {
		E("%s: sysfs_create_file dev_attr_reset failed\n", __func__);
		return ret;
	}
#endif
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_attn.attr);
	if (ret) {
		E("%s: sysfs_create_file dev_attr_attn failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_enabled.attr);
	if (ret) {
		E("%s: sysfs_create_file dev_attr_enabled failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_layout.attr);
	if (ret) {
		E("%s: sysfs_create_file dev_attr_layout failed\n", __func__);
		return ret;
	}

#ifdef HX_TP_SYS_REGISTER
	register_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		E("create_file dev_attr_register failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_DIAG
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		E("sysfs_create_file dev_attr_diag failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_DEBUG
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug.attr);
	if (ret) {
		E("create_file dev_attr_debug failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr);
	if (ret) {
		E("sysfs_create_file dev_attr_flash_dump failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_SELF_TEST
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
	if (ret) {
		E("sysfs_create_file dev_attr_tp_self_test failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_HITOUCH
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_hitouch.attr);
	if (ret) {
		E("sysfs_create_file dev_attr_hitouch failed\n");
		return ret;
	}
#endif

#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
	if (private_ts->cover_support) {
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_cover.attr);
		if (ret) {
			E("sysfs_create_file dev_attr_cover failed\n");
			return ret;
		}
	}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif

#ifdef HX_SMART_WAKEUP
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_SMWP.attr);
	if (ret) {
		E("sysfs_create_file dev_attr_SMWP failed\n");
		return ret;
	}
#endif

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_sr_en.attr);
	if (ret) {
		E("[SR]%s: sysfs_create_file dev_attr_sr_en failed\n", __func__);
		return ret;
	}

	if (private_ts->pdata->glove_mode_en)  {
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_glove_setting.attr);
		if (ret) {
			E("%s: sysfs_create_file dev_attr_glove_setting failed\n", __func__);
			return ret;
		}
	}

	return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
#ifdef HX_TP_SYS_RESET
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
#endif
	sysfs_remove_file(android_touch_kobj, &dev_attr_attn.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_enabled.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_layout.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_sr_en.attr);

#ifdef HX_TP_SYS_REGISTER
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
#endif

#ifdef HX_TP_SYS_DIAG
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
#endif

#ifdef HX_TP_SYS_DEBUG
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug.attr);
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	sysfs_remove_file(android_touch_kobj, &dev_attr_flash_dump.attr);
#endif

#ifdef HX_TP_SYS_SELF_TEST
	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
#endif

#ifdef HX_TP_SYS_HITOUCH
	sysfs_remove_file(android_touch_kobj, &dev_attr_hitouch.attr);
#endif

#ifdef HX_DOT_VIEW
#if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2)
	if (private_ts->cover_support)
		sysfs_remove_file(android_touch_kobj, &dev_attr_cover.attr);
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) || defined(CONFIG_AK8789_HALLSENSOR_R2) */
#endif

#ifdef HX_SMART_WAKEUP
	sysfs_remove_file(android_touch_kobj, &dev_attr_SMWP.attr);
#endif

	kobject_del(android_touch_kobj);
}
#endif	/* #if defined(HTC_FEATURE) */

static int himax_touch_proc_init(void)
{
	himax_touch_proc_dir = proc_mkdir( HIMAX_PROC_TOUCH_FOLDER, NULL);
	if (himax_touch_proc_dir == NULL)
	{

		E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}

	himax_proc_debug_level_file = proc_create(HIMAX_PROC_DEBUG_LEVEL_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_debug_level_ops);
	if (himax_proc_debug_level_file == NULL)
	{
		E(" %s: proc debug_level file create failed!\n", __func__);
		goto fail_0;
	}

	himax_proc_vendor_file = proc_create(HIMAX_PROC_VENDOR_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_vendor_ops);
	if(himax_proc_vendor_file == NULL)
	{
		E(" %s: proc vendor file create failed!\n", __func__);
		goto fail_1;
	}

	himax_proc_attn_file = proc_create(HIMAX_PROC_ATTN_FILE, (S_IRUGO),
		himax_touch_proc_dir, &himax_proc_attn_ops);
	if(himax_proc_attn_file == NULL)
	{
		E(" %s: proc attn file create failed!\n", __func__);
		goto fail_2;
	}

	himax_proc_int_en_file = proc_create(HIMAX_PROC_INT_EN_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_int_en_ops);
	if(himax_proc_int_en_file == NULL)
	{
		E(" %s: proc int en file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_layout_file = proc_create(HIMAX_PROC_LAYOUT_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_layout_ops);
	if(himax_proc_layout_file == NULL)
	{
		E(" %s: proc layout file create failed!\n", __func__);
		goto fail_4;
	}
#ifdef HX_TP_PROC_RESET
	himax_proc_reset_file = proc_create(HIMAX_PROC_RESET_FILE, (0777),
		himax_touch_proc_dir, &himax_proc_reset_ops);
	if(himax_proc_reset_file == NULL)
	{
		E(" %s: proc reset file create failed!\n", __func__);
		goto fail_6;
	}
#endif

#ifdef HX_TP_PROC_DIAG
	himax_proc_diag_file = proc_create(HIMAX_PROC_DIAG_FILE, (0777),
		himax_touch_proc_dir, &himax_proc_diag_ops);
	if(himax_proc_diag_file == NULL)
	{
		E(" %s: proc diag file create failed!\n", __func__);
		goto fail_7;
	}
#endif

#ifdef HX_TP_PROC_REGISTER
	himax_proc_register_file = proc_create(HIMAX_PROC_REGISTER_FILE, (0777),
		himax_touch_proc_dir, &himax_proc_register_ops);
	if(himax_proc_register_file == NULL)
	{
		E(" %s: proc register file create failed!\n", __func__);
		goto fail_8;
	}
#endif

#ifdef HX_TP_PROC_DEBUG
	himax_proc_debug_file = proc_create(HIMAX_PROC_DEBUG_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_debug_ops);
	if(himax_proc_debug_file == NULL)
	{
		E(" %s: proc debug file create failed!\n", __func__);
		goto fail_9;
	}
#endif

#ifdef HX_TP_PROC_FLASH_DUMP
	himax_proc_flash_dump_file = proc_create(HIMAX_PROC_FLASH_DUMP_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_flash_ops);
	if(himax_proc_flash_dump_file == NULL)
	{
		E(" %s: proc flash dump file create failed!\n", __func__);
		goto fail_10;
	}
#endif
#ifdef HX_TP_PROC_SELF_TEST
	himax_proc_self_test_file = proc_create(HIMAX_PROC_SELF_TEST_FILE, (0777), //ITO Test proc*************************8
		himax_touch_proc_dir, &himax_proc_self_test_ops);
	if(himax_proc_self_test_file == NULL)
	{
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_11;
	}
#endif

#ifdef HX_TP_PROC_HITOUCH
	himax_proc_hitouch_file = proc_create(HIMAX_PROC_HITOUCH_FILE, (S_IWUSR|S_IRUGO),
		himax_touch_proc_dir, &himax_proc_hitouch_ops);
	if(himax_proc_hitouch_file == NULL)
	{
		E(" %s: proc hitouch file create failed!\n", __func__);
		goto fail_12;
	}
#endif

#ifdef HX_DOT_VIEW
	himax_proc_cover_file = proc_create(HIMAX_PROC_COVER_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_cover_ops);
	if(himax_proc_cover_file == NULL)
	{
		E(" %s: proc cover file create failed!\n", __func__);
		goto fail_13;
	}
#endif

#ifdef HX_SMART_WAKEUP
#if 0
	himax_proc_SMWP_file = proc_create(HIMAX_PROC_SMWP_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_SMWP_ops);
	if(himax_proc_SMWP_file == NULL)
	{
		E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_14;
	}
	himax_proc_GESTURE_file = proc_create(HIMAX_PROC_GESTURE_FILE, (S_IWUSR|S_IRUGO|S_IWUGO),
		himax_touch_proc_dir, &himax_proc_Gesture_ops);
	if(himax_proc_GESTURE_file == NULL)
	{
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_14;
	}
#endif
#endif

	return 0 ;

#ifdef HX_SMART_WAKEUP
//	fail_14:
#endif

#ifdef HX_DOT_VIEW
	fail_13:
	remove_proc_entry( HIMAX_PROC_COVER_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_HITOUCH
	fail_12:
	remove_proc_entry( HIMAX_PROC_HITOUCH_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_SELF_TEST
	fail_11:
	remove_proc_entry( HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_FLASH_DUMP
	fail_10:
	remove_proc_entry( HIMAX_PROC_FLASH_DUMP_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_DEBUG
	fail_9:
	remove_proc_entry( HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_REGISTER
	fail_8:
	remove_proc_entry( HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_DIAG
	fail_7:
	remove_proc_entry( HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir );
#endif

#ifdef HX_TP_PROC_RESET
	fail_6:
	remove_proc_entry( HIMAX_PROC_RESET_FILE, himax_touch_proc_dir );
#endif
	fail_4: remove_proc_entry( HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir );
	fail_3: remove_proc_entry( HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir );
	fail_2: remove_proc_entry( HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir );
	fail_1: remove_proc_entry( HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir );
	fail_0: remove_proc_entry( HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir );
        // remove_proc_entry( HIMAX_PROC_TOUCH_FOLDER, NULL );
		return -ENOMEM;
}

static void himax_touch_proc_deinit(void)
{
#ifdef HX_SMART_WAKEUP
#if 0
		remove_proc_entry( HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir );
#endif
#endif
#ifdef HX_DOT_VIEW
		remove_proc_entry( HIMAX_PROC_COVER_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_TP_PROC_HITOUCH
		remove_proc_entry( HIMAX_PROC_HITOUCH_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_TP_PROC_SELF_TEST
		remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
		remove_proc_entry(HIMAX_PROC_FLASH_DUMP_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_DEBUG
		remove_proc_entry( HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir );
#endif
#ifdef HX_TP_PROC_REGISTER
		remove_proc_entry(HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_DIAG
		remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_TP_PROC_RESET
		remove_proc_entry( HIMAX_PROC_RESET_FILE, himax_touch_proc_dir );
#endif
		remove_proc_entry( HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir );
		remove_proc_entry( HIMAX_PROC_TOUCH_FOLDER, NULL );
}

#endif

#ifdef CONFIG_OF
static int himax_parse_dt(struct himax_ts_data *ts,
				struct himax_i2c_platform_data *pdata)
{
	int rc, coords_size = 0;
	uint32_t coords[4] = {0};
	struct property *prop;
	struct device_node *dt = ts->client->dev.of_node;
	u32 data = 0;
	/*** Get PCBIDField start ***/
	/*uint32_t pcb_id = 0;
	uint32_t eng_id = 0;
	uint8_t htc_skuid[48] = {0};
	struct device_node *mfgnode = of_find_node_by_path("/chosen/radio");
	if (mfgnode) {
		if (of_property_read_u8_array(mfgnode, "sku_id", &htc_skuid[0], sizeof(htc_skuid)))
			I(" %s, Failed to get property: sku id\n", __func__);
		pcb_id = htc_skuid[7] << 24 | htc_skuid[6] << 16 | htc_skuid[5] << 8 | htc_skuid[4];
		eng_id = htc_skuid[47] << 24 | htc_skuid[46] << 16 | htc_skuid[45] << 8 | htc_skuid[44];
		I("PCBID=0x%X, ENG_ID=0x%X\n", pcb_id, eng_id);
		if ( pcb_id < 0x800301FF ) {
			HMX_FW_NAME = "cs_HMX.img";
			I("pcb_id < 0x800301FF\n");
		} else if (eng_id & BIT(0)){
			HMX_FW_NAME = "cs_HMX_B.img";
			I("pcb_id > 0x800301FF, eng id = 1, use FW type B\n");
		} else {
			HMX_FW_NAME = "cs_HMX_A.img";
			I("pcb_id > 0x800301FF, end id != 1, use FW type A\n");
		}
	} else
		I(" %s, Failed to find device node\n", __func__);*/
	/*** Get PCBIDField end ***/

	prop = of_find_property(dt, "himax,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid panel coords size %d\n", __func__, coords_size);
	}

	if (of_property_read_u32_array(dt, "himax,panel-coords", coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
		pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];
		I(" DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
				pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	}

	prop = of_find_property(dt, "himax,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid display coords size %d\n", __func__, coords_size);
	}
	rc = of_property_read_u32_array(dt, "himax,display-coords", coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		D(" %s:Fail to read display-coords %d\n", __func__, rc);
	}
	pdata->screenWidth  = coords[1];
	pdata->screenHeight = coords[3];
	I(" DT-%s:display-coords = (%d, %d)\n", __func__, pdata->screenWidth,
		pdata->screenHeight);

	if (of_property_read_bool(dt, "himax,en-gpio")) {
		pdata->gpio_en = of_get_named_gpio_flags(dt, "himax,en-gpio", 0, NULL);
		rc = gpio_request_one(pdata->gpio_en, GPIOF_OUT_INIT_HIGH, "himax-en_cap");
		if (rc < 0) {
			I("%s: request en pin failed, pin status: %d\n", __func__, gpio_get_value(pdata->gpio_en));
			pdata->gpio_en = 0;
		} else {
			gpio_set_value(pdata->gpio_en, 1);
			I("%s: en gpio direction output %d\n", __func__, gpio_get_value(pdata->gpio_en));
		}
	} else {
		pdata->gpio_en = 0;
		I("%s: no gpio_en value\n", __func__);
	}

	if (of_property_read_bool(dt, "himax,irq-gpio")) {
		pdata->gpio_irq = of_get_named_gpio_flags(dt, "himax,irq-gpio", 0, NULL);
		rc = gpio_request(pdata->gpio_irq, "himax_gpio_irq_cap");
		if (rc < 0) {
			E("unable to request gpio [%d]\n", pdata->gpio_irq);
		} else {
			I("%s: request himax,irq-gpio success\n", __func__);
		}
	} else {
		I(" DT:gpio_irq value is not valid\n");
		pdata->gpio_irq = 0;
	}

	if (of_property_read_bool(dt, "himax,rst-gpio")) {
		pdata->gpio_reset = of_get_named_gpio_flags(dt, "himax,rst-gpio", 0, NULL);
		rc = gpio_request_one(pdata->gpio_reset, GPIOF_OUT_INIT_HIGH, "himax-reset_cap");
		if (rc < 0) {
			E("%s: request reset pin failed\n", __func__);
		} else {
			gpio_set_value(pdata->gpio_reset, 1);
			I("%s: reset gpio direction output %d\n", __func__, gpio_get_value(pdata->gpio_reset));
			// HW Reset, vendor said it needs low 20ms,then high 20ms
			himax_rst_gpio_set_cap(pdata->gpio_reset, 0);
			msleep(20);
			himax_rst_gpio_set_cap(pdata->gpio_reset, 1);
			msleep(20);
			I("himax reset over\n");
		}
	} else {
		I(" DT:gpio_reset value is not valid\n");
		pdata->gpio_reset = 0;
	}

	if (of_property_read_u32(dt, "himax,SW_timer_debounce", &data) == 0) {
		pdata->SW_timer_debounce = (data);
		if (!gpio_is_valid(pdata->SW_timer_debounce))
			I(" DT:SW_timer_debounce value is not valid\n");
	} else
		pdata->SW_timer_debounce = 0;

	if (of_property_read_u32(dt, "himax,wake-up-reset", &data) == 0) {
		pdata->wake_up_reset = data;
		if (!gpio_is_valid(pdata->wake_up_reset))
			I(" DT:wake_up_reset value is not valid\n");
	} else
		pdata->wake_up_reset = 0;

	if (of_property_read_u32(dt, "himax,glove-mode", &data) == 0) {
		pdata->glove_mode_en = data;
		if (!gpio_is_valid(pdata->glove_mode_en))
			I(" DT:glove_mode_en value is not valid\n");
	} else
		pdata->glove_mode_en = 0;

	if (of_property_read_u32(dt, "himax,always-K-WA", &data) == 0) {
		pdata->always_K_WA = data;
		if (!gpio_is_valid(pdata->always_K_WA))
			I(" DT:glove_mode_en value is not valid\n");
	} else
		pdata->always_K_WA = 0;

	I(" DT:gpio_irq=%d, gpio_rst=%d, gpio_en=%d, SW_timer_debounce=%d,  wake_up_reset=%d, glove_mode_en=%d\n",
		pdata->gpio_irq, pdata->gpio_reset, pdata->gpio_en, pdata->SW_timer_debounce, pdata->wake_up_reset, pdata->glove_mode_en);

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		I(" DT:protocol_type=%d\n", pdata->protocol_type);
	}

	if (of_property_read_u32(dt, "irq-on-state", &data) == 0) {
		pdata->irq_on_state = data;
		I(" DT:irq-on-state=%d\n", pdata->irq_on_state);
	} else {
		pdata->irq_on_state = 0;
		I("No DT:irq-on-state=%d\n", pdata->irq_on_state);
	}

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
	if (of_property_read_u32(dt, "proximity_bytp_enable", &data) == 0) {
		pdata->proximity_bytp_enable = data;
		I(" DT:proximity_bytp_enable=%d\n", pdata->proximity_bytp_enable);
	}
#endif

	return 0;
}
#endif

static void himax_int_enable_cap(int irqnum, int enable, int logprint)
{
	if (enable == 1 && irq_enable_count == 0) {
		enable_irq(irqnum);
		irq_enable_count++;
	} else if (enable == 0 && irq_enable_count == 1) {
		disable_irq_nosync(irqnum);
		irq_enable_count--;
	}

	if(logprint)
		I("irq_enable_count = %d\n", irq_enable_count);
}

static void himax_rst_gpio_set_cap(int pinnum, uint8_t value)
{
	gpio_set_value(pinnum, value);
}

static int8_t himax_int_gpio_read_cap(int pinnum)
{
	return gpio_get_value(pinnum);
}

static int himax_pinctrl_init(struct himax_ts_data *cs)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	cs->pinctrl = devm_pinctrl_get(cs->dev);
	if (IS_ERR_OR_NULL(cs->pinctrl)) {
		retval = PTR_ERR(cs->pinctrl);
		D("Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	cs->pinctrl_state_active
		= pinctrl_lookup_state(cs->pinctrl, "pmx_cs_active");
	if (IS_ERR_OR_NULL(cs->pinctrl_state_active)) {
		retval = PTR_ERR(cs->pinctrl_state_active);
		D("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	cs->pinctrl_state_suspend
		= pinctrl_lookup_state(cs->pinctrl, "pmx_cs_suspend");
	if (IS_ERR_OR_NULL(cs->pinctrl_state_suspend)) {
		retval = PTR_ERR(cs->pinctrl_state_suspend);
		D("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	cs->pinctrl_state_release
		= pinctrl_lookup_state(cs->pinctrl, "pmx_cs_release");
	if (IS_ERR_OR_NULL(cs->pinctrl_state_release)) {
		retval = PTR_ERR(cs->pinctrl_state_release);
		D("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(cs->pinctrl);
err_pinctrl_get:
	cs->pinctrl = NULL;
	return retval;
}
static int himax852xes_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int err = -ENOMEM;

	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;
	char *mfg_mode;

	I("%s: +++\n", __func__);

	//Check I2C functionality
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

#ifdef MTK// Allocate the MTK's DMA memory
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, 4096, (dma_addr_t *)&gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va)
	{
		E("Allocate DMA I2C Buffer failed\n");
		goto err_alloc_MTK_DMA_failed;
	}
	memset(gpDMABuf_va, 0, 4096);
#endif
	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		E("%s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	i2c_set_clientdata(client, ts);
	I("client-addr = %x\n",client->addr);
	ts->client = client;
	ts->dev = &client->dev;

#if 0
	/* Remove this since this will cause request_firmware fail.
	   Thie line has no use to touch function. */
	dev_set_name(ts->dev, HIMAX852xes_NAME_CAP);
#endif

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
			goto err_dt_platform_data_fail;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) { /*DeviceTree Init Platform_data*/
		err = himax_parse_dt(ts, pdata);
		if (err < 0) {
			I(" pdata is NULL for DT\n");
			goto err_alloc_dt_pdata_failed;
		}
	} else {
			I("dev.of_node is NULL for DT\n");
	}

#else
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		I(" pdata is NULL(dev.platform_data)\n");
		goto err_get_platform_data_fail;
	}
#endif

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif
	ts->irq_gpio = pdata->gpio_irq;

#ifndef CONFIG_OF
	if (pdata->power) {
		err = pdata->power(1);
		if (err < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
#endif
	private_ts = ts;
	private_pdata = pdata;
	//Get Himax IC Type / FW information / Calculate the point number
	if (himax_ic_package_check(ts) == false) {
		E("Himax chip doesn NOT EXIST, retry_probe: %d\n", retry_probe);
		if (retry_probe++ < MAX_RETRY_CNT) {
			/** free pdata, ts data and power pin **/
			if ((pdata->gpio_irq != 0)) {
				gpio_free(pdata->gpio_irq);
			}
			if ((pdata->gpio_reset != 0)) {
				gpio_free(pdata->gpio_reset);
			}
			if ((pdata->gpio_en != 0)) {
				gpio_free(pdata->gpio_en);
			}
			kfree(pdata);
			kfree(ts);
			return -EPROBE_DEFER;
		} else
			goto err_ic_package_failed;
	}

	err = himax_pinctrl_init(ts);
	if (!err && ts->pinctrl) {
		/*
		* Pinctrl handle is optional. If pinctrl handle is found
		* let pins to be configured in active state. If not
		* found continue further without error.
		*/
		err = pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_active);
		if (err < 0) {
			E("%s: Failed to select %s pinstate %d\n",
				__func__, PINCTRL_STATE_ACTIVE, err);
		}
	}
	if (ts->pinctrl) {
		if (pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_active) < 0)
			E("Cannot get default pinctrl state\n");
	}

	mfg_mode = htc_get_bootmode();
	if((strcmp(mfg_mode, "offmode_charging") == 0) ||
		(strcmp(mfg_mode, "recovery") == 0) ||
		(strcmp(mfg_mode, "charger") == 0) ||
		(strcmp(mfg_mode, "MFG_MODE_OFFMODE_CHARGING") == 0) ||
		(strcmp(mfg_mode, "MFG_MODE_POWER_TEST") == 0) ||
		(strcmp(mfg_mode, "MFG_MODE_RECOVERY") == 0))
	{
		I(" %s: %s mode. cap chip default sleep mode and skip cap driver probe\n",
		__func__, mfg_mode);
		err = -ENODEV;
		E("%s: goto err_off_mode\n", __func__);
		goto err_off_mode;
	}

	if (pdata->virtual_key)
		ts->button = pdata->virtual_key;
#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)
		ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
		if (!ts->flash_wq)
		{
			E("%s: create flash workqueue failed\n", __func__);
			err = -ENOMEM;
			goto err_create_flash_wq_failed;
		}

		INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);

		setSysOperation(0);
		setFlashBuffer();
#endif

	himax_read_TP_info(client);
#ifdef HX_AUTO_UPDATE_FW
	if(i_update_FW()==false)//****************************************************fw auto update
		I("NOT Have new FW=NOT UPDATE=\n");
	else
		I("Have new FW=UPDATE=\n");
#endif
	//DO NOT load config, only Power On init, if fail, do reset 1 time
	if (himax_power_on_initCMD(client) < 0) {
		I(" power on init fail, do HW reset\n");
		himax_rst_gpio_set_cap(pdata->gpio_reset, 0);
		msleep(20);
		himax_rst_gpio_set_cap(pdata->gpio_reset, 1);
		msleep(20);
		himax_power_on_initCMD(client);
	}

	calculate_point_number();
#if defined(HX_TP_PROC_DIAG) || defined(HX_TP_SYS_DIAG)
	setXChannel(HX_RX_NUM); // X channel
	setYChannel(HX_TX_NUM); // Y channel

	setMutualBuffer();
	if (getMutualBuffer() == NULL) {
		E("%s: mutual buffer allocate fail failed\n", __func__);
		goto err_setchannel_failed;
	}
#if defined(HX_TP_PROC_2T2R) || defined (HX_TP_SYS_2T2R)
	if(Is_2T2R){
		setXChannel_2(HX_RX_NUM_2); // X channel
		setYChannel_2(HX_TX_NUM_2); // Y channel

		setMutualBuffer_2();

		if (getMutualBuffer_2() == NULL) {
			E("%s: mutual buffer 2 allocate fail failed\n", __func__);
			goto err_setchannel_failed;
		}
	}
#endif	
#endif
#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;
	ts->SW_timer_debounce = pdata->SW_timer_debounce;
	ts->x_channel = HX_RX_NUM;
	ts->y_channel = HX_TX_NUM;
	ts->nFinger_support = HX_MAX_PT;
	//calculate the i2c data size
	calcDataSize(ts->nFinger_support);
	I("%s: calcDataSize complete\n", __func__);

	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;

	ts->suspended                      = false;
#if defined(HX_USB_DETECT)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
#endif
	ts->protocol_type = pdata->protocol_type;
	I("%s: Use Protocol Type %c\n", __func__,
	ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	err = himax_input_register(ts);
	if (err) {
		E("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

//	if (get_tamper_sf() == 0) {
		ts->debug_log_level |= BIT(3);
		I("%s: Enable touch down/up debug log since not security-on device\n",
		  __func__);
		if (pdata->screenWidth > 0 && pdata->screenHeight > 0 &&
		    (pdata->abs_x_max - pdata->abs_x_min) > 0 &&
		    (pdata->abs_y_max - pdata->abs_y_min) > 0) {
			ts->widthFactor = (pdata->screenWidth << SHIFTBITS) / (pdata->abs_x_max - pdata->abs_x_min);
			ts->heightFactor = (pdata->screenHeight << SHIFTBITS) / (pdata->abs_y_max - pdata->abs_y_min);
			if (ts->widthFactor > 0 && ts->heightFactor > 0)
				ts->useScreenRes = 1;
			else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else
			I("Enable finger debug with raw position mode!\n");
//	}

#ifdef HX_CHIP_STATUS_MONITOR//for ESD solution
	ts->himax_chip_monitor_wq = create_singlethread_workqueue("himax_chip_monitor_wq");
	if (!ts->himax_chip_monitor_wq)
	{
		E(" %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_chip_monitor_wq_failed;
	}

	INIT_DELAYED_WORK(&ts->himax_chip_monitor, himax_chip_monitor_function);
	queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);
#endif

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
	if(pdata->proximity_bytp_enable)
	wake_lock_init(&ts->ts_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME_CAP);
#endif
#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable=0;
	wake_lock_init(&ts->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME_CAP);
#endif
wake_lock_init(&ts->ts_flash_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME_CAP);

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)//******************************************ITOtest proc  and gesture proc
	himax_touch_proc_init();
#if defined(HTC_FEATURE)
	himax_touch_sysfs_init();
#endif
#ifdef HX_SMART_WAKEUP
	himax_gesture_sysfs_init();
#endif
#endif

#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 0;
#endif
	HW_RESET_ACTIVATE = 0;


#if defined(HX_USB_DETECT)
	if (ts->cable_config)
		cable_detect_register_notifier(&himax_cable_status_handler);
#endif
#ifdef HX_DOT_VIEW
	register_notifier_by_hallsensor(&hallsensor_status_handler);
#endif


	if ((strcmp(mfg_mode, "ftm") == 0) ||
		(strcmp(mfg_mode, "factory") == 0))
	{
		I(" %s: %s mode. Set touch chip to sleep mode and skip himax_ts_register_interrupt\n",
		__func__, mfg_mode);

		hx_sensor_on(false);
	} else {
		I(" %s: %s mode, register FB\n",__func__, mfg_mode);
		himax_int_enable_cap(ts->client->irq,1,true);
		err = himax_ts_register_interrupt_cap(ts->client);
		if (err) {
			E("%s: himax_ts_register_interrupt failed\n", __func__);
			goto err_register_interrupt_failed;
		}
#ifdef CONFIG_FB
		ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");
		if (!ts->himax_att_wq) {
			E(" allocate HMX_att_wq failed\n");
			err = -ENOMEM;
			goto err_get_intr_bit_failed;
		}
		INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
		I(" %s: set FB register \n",__func__);
		queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
		ts->early_suspend.suspend = himax_ts_early_suspend;
		ts->early_suspend.resume = himax_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
		ts->cap_hall_s_pole_status = HALL_FAR;
		ts->hallsensor_handler.notifier_call = hallsensor_handler_callback;
		hallsensor_register_notifier(&ts->hallsensor_handler);
#endif
	}

	if(mfg_flag)
		private_ts->update_feature = mfg_flag;

	if (private_ts->update_feature){
		if ((strcmp(mfg_mode, "")  == 0) ||
			(strcmp(mfg_mode, "MFG_MODE_NORMAL") == 0) ||
			(strcmp(mfg_mode, "factory2") == 0) ||
			(strcmp(mfg_mode, "normal") == 0) /* Default ON!!! */ ||
			(strcmp(mfg_mode, "MFG_MODE_FACTORY2") == 0)) {
			ts->himax_FW_wq = create_singlethread_workqueue("HMX_FW_reuqest");
			if (!ts->himax_FW_wq) {
				E(" allocate HMX_FW_wq failed\n");
			} else {
				INIT_DELAYED_WORK(&ts->work_FW, updateFirmware);
				I(" %s: set FW update work queue\n",__func__);
				queue_delayed_work(ts->himax_FW_wq, &ts->work_FW, msecs_to_jiffies(10000));
				I("[FW] %s mode, process firmware update check.\n", mfg_mode);
			}
		} else
			I("[FW] skip firmware update since under %s mode.\n", mfg_mode);

	}
	mfg_flag = 0;

	/*** init spin lock & de-bounced related value status***/
	spin_lock_init(&ts->lock);
	ts->debounced[0] = false;
	ts->debounced[1] = false;
	ts->K_counter = 0;
	ts->ESD_counter = 0;
	memset(&ts->time_start, 0x00, sizeof(ts->time_start));
	memset(&ts->ESD_time_start, 0x00, sizeof(ts->ESD_time_start));
	getnstimeofday(&ts->time_start);
	getnstimeofday(&ts->ESD_time_start);

	/* Create singlethread workqueue */
	INIT_DELAYED_WORK(&ts->monitorDB[0], himax_cap_debounce_work_func_back);
	ts->monitor_wq[0] = create_singlethread_workqueue("monitorDB_wq_back");
	if (!ts->monitor_wq[0]) {
		E(" %s: create workqueue failed back\n", __func__);
		goto err_device_init_wq_debounce;
	}
	INIT_DELAYED_WORK(&ts->monitorDB[1], himax_cap_debounce_work_func_app);
	ts->monitor_wq[1] = create_singlethread_workqueue("monitorDB_wq_app");
	if (!ts->monitor_wq[1]) {
		E(" %s: create workqueue failed app\n", __func__);
		goto err_device_init_wq_debounce;
	}
	INIT_WORK(&ts->recal_work, recal_work_func);
	ts->recal_work_wq = create_singlethread_workqueue("recal_work");
	if (!ts->recal_work_wq) {
		E(" %s: create workqueue failed reset\n", __func__);
	        goto err_device_init_wq_debounce;
	}

	I("%s: ---\n", __func__);

	return 0;

err_register_interrupt_failed:
err_device_init_wq_debounce:
	if(ts->irq_enabled){
		himax_int_enable_cap(ts->client->irq,0,true);
		free_irq(ts->client->irq, ts);
	}
	if (!ts->use_irq){
		hrtimer_cancel(&ts->timer);
		destroy_workqueue(ts->himax_wq);
	}
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
#ifdef HX_SMART_WAKEUP
	himax_gesture_sysfs_deinit();
#endif
	himax_touch_proc_deinit();
#endif
	wake_lock_destroy(&ts->ts_flash_wake_lock);
#ifdef HX_SMART_WAKEUP
	wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
	destroy_workqueue(ts->monitor_wq[0]);
	destroy_workqueue(ts->monitor_wq[1]);
	destroy_workqueue(ts->recal_work_wq);
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
	if(pdata->proximity_bytp_enable)
		wake_lock_destroy(&ts->ts_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
	destroy_workqueue(ts->himax_chip_monitor_wq);
err_create_chip_monitor_wq_failed:
#endif


#ifdef CONFIG_FB
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
	if (fb_unregister_client(&ts->fb_notif))
			I("Error occurred while unregistering fb_notifier.\n");
err_get_intr_bit_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	input_free_device(ts->input_dev);

err_input_register_device_failed:
#if defined(HX_TP_PROC_DIAG) || defined(HX_TP_SYS_DIAG)
err_setchannel_failed:
#endif
#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)
	destroy_workqueue(ts->flash_wq);
err_create_flash_wq_failed:
#endif
err_off_mode:
	gpio_free(pdata->gpio_reset);
	gpio_free(pdata->gpio_irq);
err_ic_package_failed:
#ifndef CONFIG_OF
err_power_failed:
#endif
#ifdef CONFIG_OF
err_alloc_dt_pdata_failed:
#else
err_get_platform_data_fail:
#endif
	kfree(pdata);
err_dt_platform_data_fail:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	HX_DRIVER_PROBE_Fial=1;
	err =-1;
	return err;

}

static int himax852xes_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	if(ts->irq_enabled){
		himax_int_enable_cap(ts->client->irq,0,true);
			free_irq(ts->client->irq, ts);
		}
	if (!ts->use_irq){
			hrtimer_cancel(&ts->timer);
			destroy_workqueue(ts->himax_wq);
		}
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_CAP)
#ifdef HX_SMART_WAKEUP
		himax_gesture_sysfs_deinit();
#endif
#if defined(HTC_FEATURE)
		himax_touch_sysfs_deinit();
#endif
		himax_touch_proc_deinit();
#endif
#ifdef HX_SMART_WAKEUP
		wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
		if(pdata->proximity_bytp_enable)
			wake_lock_destroy(&ts->ts_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
		cancel_delayed_work_sync(&ts->himax_chip_monitor);
		destroy_workqueue(ts->himax_chip_monitor_wq);
#endif


#ifdef CONFIG_FB
		cancel_delayed_work_sync(&ts->work_att);
		destroy_workqueue(ts->himax_att_wq);
		if (fb_unregister_client(&ts->fb_notif))
			I("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		unregister_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
		hallsensor_unregister_notifier(&ts->hallsensor_handler);
#endif
		input_free_device(ts->input_dev);
#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)
		destroy_workqueue(ts->flash_wq);
#endif
		if (ts->pinctrl) {
			if (IS_ERR_OR_NULL(ts->pinctrl_state_release)) {
				devm_pinctrl_put(ts->pinctrl);
				ts->pinctrl = NULL;
			} else {
				if (pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_release))
					E("Failed to select release pinctrl state\n");
			}
		}
		gpio_free(ts->rst_gpio);
		gpio_free(ts->irq_gpio);
		kfree(ts);
#ifdef MTK
		if(gpDMABuf_va)
		{
			dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
			gpDMABuf_va = NULL;
			gpDMABuf_pa = NULL;
		}
		if(flash_buffer) {
			kfree(flash_buffer);
		}
#endif

	return 0;

}

void himax852xes_suspend_cap(struct device *dev)
{
	int ret;
#ifdef HX_SMART_WAKEUP
	uint8_t buf[2] = {0};
#endif
#ifdef HX_CHIP_STATUS_MONITOR
	int t=0;
#endif
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	if(HX_DRIVER_PROBE_Fial)
	{
		I("%s: Driver probe fail. \n", __func__);
		return;
	}
	I("%s: Enter suspended. \n", __func__);

	if(ts->suspended)
	{
		I("%s: Already suspended. Skipped. \n", __func__);
		return;
	}
	else
	{
		ts->suspended = true;
		I("%s: enter \n", __func__);
	}

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY_CAP)
	if(ts->pdata->proximity_bytp_enable){
			I("[Proximity],Proximity en=%d\r\n",g_proximity_en);
			if(g_proximity_en)
					I("[Proximity],Proximity %s\r\n",proximity_flag? "NEAR":"FAR");

			if((g_proximity_en)&&(proximity_flag)){
				I("[Proximity],Proximity on,and Near won't enter deep sleep now.\n");
				atomic_set(&ts->suspend_mode, 1);
				ts->first_pressed = 0;
				ts->pre_finger_mask = 0;
				return;
			}
		}
#endif
#if defined(HX_TP_PROC_FLASH_DUMP) || defined(HX_TP_SYS_FLASH_DUMP)
	if (getFlashDumpGoing())
	{
		I("[himax] %s: Flash dump is going, reject suspend\n",__func__);
		return;
	}
#endif
#ifdef HX_TP_PROC_HITOUCH
	if(hitouch_is_connect)
	{
		I("[himax] %s: Hitouch connect, reject suspend\n",__func__);
		return;
	}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
	if(HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
	{
		for(t=0; t<100; t++) {
				if(HX_ON_HAND_SHAKING==0)//chip on hand shaking end
				{
					I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,t);
					break;
				} else
					msleep(1);
		}
		if(t==100) {
			E("%s:HX_ON_HAND_SHAKING timeout reject suspend\n",__func__);
			return;
		}
	}

	HX_CHIP_POLLING_COUNT = 0;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif

#ifdef HX_SMART_WAKEUP
	if(ts->SMWP_enable)
	{
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		FAKE_POWER_KEY_SEND=false;
		buf[0] = 0x8F;
		buf[1] = 0x20;
		ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
		if (ret < 0)
		{
			E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
		}
		I("[himax] %s: SMART_WAKEUP enable, reject suspend\n",__func__);
		return;
	}
#endif

	himax_int_enable_cap(ts->client->irq,0,true);

	//Himax 852xes IC enter sleep mode
	hx_sensor_on(false);

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);
		if (ret)
			himax_int_enable_cap(ts->client->irq,1,true);
	}

	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(0);

	return;
}

static void himax_cap_post_resume_work_func(struct work_struct *dummy)
{
	struct himax_ts_data *cs = private_ts;
	if (cs->pdata->wake_up_reset) {
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset_cap(false, true);
#endif
	}
	if (cs->pdata->glove_mode_en) {
		if (cs->cap_glove_mode_status)
			hx_sensitivity_setup(cs, CS_SENS_HIGH);
		else if (cs->cap_glove_mode_status == 0)
			hx_sensitivity_setup(cs, CS_SENS_DEFAULT);
	}
	hx_sensor_on(true);
	atomic_set(&cs->suspend_mode, 0);
#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(cs->himax_chip_monitor_wq, &cs->himax_chip_monitor, HX_POLLING_TIMER*HZ); //for ESD solution
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
	if(cs->cap_hall_s_pole_status == HALL_FAR)
#endif
		himax_int_enable_cap(cs->client->irq, 1, true);

	cs->suspended = false;
	I("%s: cap sensor reset end----\n", __func__);
}
static DECLARE_DELAYED_WORK(himax_cap_post_resume_work, himax_cap_post_resume_work_func);

static void himax852xes_resume_cap(struct device *dev)
{
#ifdef HX_SMART_WAKEUP
	int ret = 0;
	uint8_t buf[2] = {0};
#endif
#ifdef HX_CHIP_STATUS_MONITOR
	int t=0;
#endif
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	if(HX_DRIVER_PROBE_Fial) {
		I("%s: Driver probe fail. \n", __func__);
		return;
	}
	I("%s: enter \n", __func__);

	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(1);
#ifdef HX_CHIP_STATUS_MONITOR
	if(HX_ON_HAND_SHAKING) {//chip on hand shaking,wait hand shaking
		for(t=0; t<100; t++) {
			if(HX_ON_HAND_SHAKING==0) {//chip on hand shaking end
					I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,t);
					break;
			} else
				msleep(1);
		}
		if(t==100) {
			E("%s:HX_ON_HAND_SHAKING timeout reject resume\n",__func__);
			return;
		}
	}
#endif
#ifdef HX_SMART_WAKEUP
	if(ts->SMWP_enable) {
		hx_sensor_on(false);
		buf[0] = 0x8F;
		buf[1] = 0x00;
		ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
		}
		msleep(50);
	}
#endif
	I("%s: cap sensor resume end, call post resume work----\n", __func__);
	schedule_delayed_work(&himax_cap_post_resume_work, msecs_to_jiffies(0));
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts=
		container_of(self, struct himax_ts_data, fb_notif);

	I(" %s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		switch (*blank) {
			case FB_BLANK_UNBLANK:
				himax852xes_resume_cap(&ts->client->dev);
			break;

			case FB_BLANK_POWERDOWN:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				himax852xes_suspend_cap(&ts->client->dev);
			break;
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);

#ifdef MTK
	himax852xes_suspend_cap(h);
#endif
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
#ifdef MTK
	himax852xes_resume_cap(h);
#endif
}
#endif

static const struct i2c_device_id himax852xes_ts_id[] = {
	{HIMAX852xes_NAME_CAP, 0 },
	{}
};

#ifdef MTK
//static struct i2c_board_info __initdata himax852xes_i2c = { I2C_BOARD_INFO(HIMAX852xes_NAME_CAP, (0x90 >> 1))};
static struct i2c_driver tpd_i2c_driver =
{
    .probe = himax852xes_probe,
    .remove = himax852xes_remove,
    .detect = himax852xes_detect,
    .driver.name = HIMAX852xes_NAME_CAP,
    .driver	= {
		.name = HIMAX852xes_NAME_CAP,
		.of_match_table = of_match_ptr(himax_match_table),
		},
    .id_table = himax852xes_ts_id,
};
#endif
#ifdef QCT
static struct i2c_driver himax852xes_driver = {
	.id_table       = himax852xes_ts_id,
	.probe          = himax852xes_probe,
	.remove         = himax852xes_remove,
	.driver         = {
	.name = HIMAX852xes_NAME_CAP,
	.owner = THIS_MODULE,
	.of_match_table = himax_match_table,
#ifdef CONFIG_PM
	//.pm = &himax852xes_pm_ops,
#endif
	},
};
#endif
#ifdef MTK
static int himax852xes_local_init(void)
{
	I("[Himax] Himax_ts I2C Touchscreen Driver local init\n");
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		I("unable to add i2c driver.\n");
		return -1;
	}

	I("end %s, %d\n", __func__, __LINE__);

	return 0;
}
#endif
#ifdef MTK
static int himax852xes_detect (struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, TPD_DEVICE, sizeof(info->type));
	 return 0;
}
static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = HIMAX852xes_NAME_CAP,
    .tpd_local_init = himax852xes_local_init,
    .suspend = himax852xes_suspend_cap,
    .resume = himax852xes_resume_cap,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};
#endif

static void __init himax852xes_init_async(void *unused, async_cookie_t cookie)
{
	I("%s:Enter \n", __func__);
#ifdef QCT
	if (i2c_add_driver(&himax852xes_driver) != 0)
		I("unable to add i2c driver.\n");
#endif
	I("End %s, %d\n", __func__, __LINE__);
}

static int __init himax852xes_init(void)
{
	I("Himax 852xES touch panel driver init\n");
	async_schedule(himax852xes_init_async, NULL);
	return 0;
}

static void __exit himax852xes_exit(void)
{
#ifdef MTK
	i2c_del_driver(&tpd_i2c_driver);
#endif
#ifdef QCT
	i2c_del_driver(&himax852xes_driver);
#endif
}

module_init(himax852xes_init);
module_exit(himax852xes_exit);
static int __init mfg_update_enable_flag(char *str)
{
	int ret = 0;
	mfg_flag = 1;
	I("MFG_BUILD flag enable: %d from %s\n", mfg_flag, str);
	return ret;
} early_param("androidboot.mfg.tpupdate", mfg_update_enable_flag);

static int __init get_tamper_flag(char *str)
{
	int ret = kstrtouint(str, 0, &tamper_flag);
	pr_info(" %d: %d from %s", ret, tamper_flag, str);
	return ret;
} early_param("td.sf", get_tamper_flag);

MODULE_DESCRIPTION("Himax852xes driver");
MODULE_LICENSE("GPL");
