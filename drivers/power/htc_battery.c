#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/power/htc_battery.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/of_batterydata.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif /* CONFIG_FB */
#include <supply/qcom/smb-reg.h>
#include <supply/qcom/fg-reg.h>

static struct htc_battery_info htc_batt_info;
static struct htc_battery_timer htc_batt_timer;

#define HTC_BATT_NAME "htc_battery"
#ifdef CONFIG_HTC_BATT_PCN0021
#define HTC_STATISTICS "htc_batt_stats_1.1"
#endif //CONFIG_HTC_BATT_PCN0021

static int screen_on_limit_chg = 1;
module_param_named(
	screen_on_limit_chg, screen_on_limit_chg, int, S_IRUSR | S_IWUSR
);

static unsigned int g_charger_ctrl_stat;

/* disable pwrsrc reason */
#define HTC_BATT_PWRSRC_DIS_BIT_MFG		(1)
#define HTC_BATT_PWRSRC_DIS_BIT_API		(1<<1)
#define HTC_BATT_PWRSRC_DIS_BIT_USB_OVERHEAT (1<<2)
#define HTC_BATT_PWRSRC_DIS_BIT_FTM	(1<<3)
static int g_pwrsrc_dis_reason;

/* disable charging reason */
#define HTC_BATT_CHG_DIS_BIT_EOC	(1)
#define HTC_BATT_CHG_DIS_BIT_ID		(1<<1)
#define HTC_BATT_CHG_DIS_BIT_TMP	(1<<2)
#define HTC_BATT_CHG_DIS_BIT_OVP	(1<<3)
#define HTC_BATT_CHG_DIS_BIT_TMR	(1<<4)
#define HTC_BATT_CHG_DIS_BIT_MFG	(1<<5)
#define HTC_BATT_CHG_DIS_BIT_USR_TMR	(1<<6)
#define HTC_BATT_CHG_DIS_BIT_STOP_SWOLLEN	(1<<7)
#define HTC_BATT_CHG_DIS_BIT_USB_OVERHEAT	(1<<8)
#define HTC_BATT_CHG_DIS_BIT_FTM	(1<<9)
static int g_chg_dis_reason;

/* limited charge reason */
#define HTC_BATT_CHG_LIMIT_BIT_TALK				(1)
#define HTC_BATT_CHG_LIMIT_BIT_NAVI				(1<<1)
#define HTC_BATT_CHG_LIMIT_BIT_THRML				(1<<2)
#define HTC_BATT_CHG_LIMIT_BIT_KDDI				(1<<3)
#define HTC_BATT_CHG_LIMIT_BIT_NET_TALK			(1<<4)
static int g_chg_limit_reason;

/* for suspend high frequency (5min) */
static int suspend_highfreq_check_reason;

/* MFG ftm mode feature */
static int g_ftm_charger_control_flag;

/* static int prev_charging_src; */
static int g_latest_chg_src = POWER_SUPPLY_TYPE_UNKNOWN;

/* Set true when all battery need file probe done */
static bool g_htc_battery_probe_done = false;
static bool g_is_unknown_charger = false;

/* fake soc when set this flag */
bool g_test_power_monitor;
/* 1.Disable temp protect. 2.Skip safety timer. */
bool g_flag_keep_charge_on;
/* Fake cable type to AC for ATS testing. */
bool g_flag_force_ac_chg;
/* Fake battery temperature not over 68 degree for PA testing. */
bool g_flag_pa_fake_batt_temp;
/* Disable safety timer */
bool g_flag_disable_safety_timer;
/* Disable battery temperature hot/cold protection*/
bool g_flag_disable_temp_protection;
/* Enable batterydebug log*/
bool g_flag_enable_batt_debug_log;
/* is battery fully charged with charging stopped */
bool g_flag_ats_limit_chg;

static int g_batt_full_eoc_stop;

/*Force to update power_supply*/
static bool gs_update_PSY = false;

/* for batt cycle info */
unsigned int g_total_level_raw;
unsigned int g_overheat_55_sec;
unsigned int g_batt_first_use_time;
unsigned int g_batt_cycle_checksum;


/*USB CONN temp*/
static bool g_usb_overheat = false;
static int g_usb_overheat_check_count = 0;
static int g_usb_conn_temp = 300;

/*cable impedance*/
static bool gs_measure_cable_impedance = true;
static int gs_R_cable_impedance = 0;
static int gs_cable_impedance = 0; 	// 0:init, 1:good, 2:not good, 3:bad, 4:calculating
static int gs_aicl_result = 0;
static bool g_need_to_check_impedance = true;

static int gs_prev_charging_enabled = 0;

/* To make sure the level to report is ready */
static bool g_is_rep_level_ready = true;
static bool g_is_consistent_level_ready = false;

#ifdef CONFIG_HTC_BATT_PCN0021
/* statistics of charging info */
bool g_htc_stats_charging = false;
struct htc_statistics_category g_htc_stats_category_all;
struct htc_statistics_category g_htc_stats_category_full_low;
struct htc_charging_statistics g_htc_stats_data;
const int HTC_STATS_SAMPLE_NUMBER = 5;

/* BI Data Thermal Battery Temperature */
static int g_thermal_batt_temp = 0;

/* BI for batt charging */
#define HTC_BATT_CHG_BI_BIT_CHGR                             (1)
#define HTC_BATT_CHG_BI_BIT_AGING                            (1<<1)
static int g_BI_data_ready = 0;
static struct timeval gs_batt_chgr_start_time = { 0, 0 };
static int g_batt_chgr_iusb = 0;
static int g_batt_chgr_ibat = 0;
static int g_batt_chgr_start_temp = 0;
static int g_batt_chgr_end_temp = 0;
static int g_batt_chgr_start_level = 0;
static int g_batt_chgr_end_level = 0;
static int g_batt_chgr_start_batvol = 0;
static int g_batt_chgr_end_batvol = 0;

/* BI for batt aging */
static int g_batt_aging_bat_vol = 0;
static int g_batt_aging_level = 0;
static unsigned int g_pre_total_level_raw = 0;

/* Quick charger detection */
static bool g_is_quick_charger = false;

/* Screen ON ibat limitation */
#define SCREEN_LIMIT_IBAT_DELAY_MS	60000
static bool g_is_screen_on_limit_ibat = false;

/* Thermal ibat limit */
static int g_thermal_limit_ma = 0;

/* reference from power_supply.h power_supply_type */
const char *cg_chr_src[] = { "NONE", "Battery", "UPS", "Mains", "USB", "AC(USB_DCP)",
			     "USB_CDP", "USB_ACA", "USB_HVDCP", "USB_HVDCP_3", "USB_PD",
			     "Wireless", "USB_FLOAT", "BMS", "Parallel", "Main", "Wipower",
			     "TYPEC", "TYPEC_UFP", "TYPEC_DFP" };

enum {
    USB_CONN_STAT_INIT = 0,
    USB_CONN_STAT_POLLING,
    USB_CONN_STAT_OVERHEAT,
    USB_CONN_STAT_POLLING_END_CHK,
};

/* USB connector overheat parameter */
static unsigned int g_htc_usb_overheat_check_state = USB_CONN_STAT_INIT;
static bool g_htc_usb_overheat = false;

//Cool Charger
static bool htcchg_on = false;
static bool htcchg_leave = false;
static bool not_entry_again = false;
static bool first_cool_charger = true;

enum {
    HTC_STATS_CATEGORY_ALL = 0,
    HTC_STATS_CATEGORY_FULL_LOW,
};
#endif //CONFIG_HTC_BATT_PCN0021

#define BATT_DEBUG(x...) do { \
	if (g_flag_enable_batt_debug_log) \
		printk(KERN_INFO"[BATT] " x); \
	else	\
		printk(KERN_DEBUG"[BATT] " x); \
} while (0)

struct dec_level_by_current_ua {
	int threshold_ua;
	int dec_level;
};

static struct dec_level_by_current_ua g_dec_level_curr_table[] = {
							{900000, 2},
							{600000, 4},
							{0, 6},
};

static char* htc_chr_type_data_str[] = {
        "NONE", //no charger
        "UNKNOWN", //unknown charger
        "UNKNOWN_TYPE", //unknown type
        "USB", //normal USB charger
        "USB_CDP", //USB 3.0 port, framework will show AC charging.
        "AC(USB_DCP)", //normal AC charger
        "USB_HVDCP", //QC2.0
        "USB_HVDCP_3", //QC3.0
        "PD_5V", //PD 5V
        "PD_9V", //PD 9V
        "PD_12V", //PD 12V
        "USB_TYPE_C" //USB TypeC charger
};

/*AI charging*/
#define IS_SCREEN_ON    1
#define IS_SCREEN_OFF   2
#define IS_CHARGING     3
#define IS_DISCHARGE    4
#define IS_CHARGE_FULL  5
#define IS_CROSS_DAY	6
#define BOOT_SCREEN_ON	7

#define AIC_Limit	1000

struct TimeStage{
        bool status;
        int charging;
        int screen_on_cnt;
        int screen_on_time;
};

static struct TimeStage aicdata[48]; //24hr
static struct TimeStage CalculateData[14][48];

static bool AIC_setting[48];

static int screen_on_begin;
static int charging_begin;
static int charging_full;
static int aicdata_index = 0;
static int Crossday_charging_begin = 0;

static bool aic_data_ready = false;
static bool aic_data_request_for_shutdown = false;
static bool aic_init_data = true;
static int data_req_current_time = 0;
static bool gbAICharging = false;

struct AIC_data{
	int current_time;
	int aicdata_index;
	int aicdata[48];
	int calculatedata[7][48];
};

union AIC_charing{
        struct AIC_data int_data;
        char char_data[sizeof(struct AIC_data)];
};

struct AIC_data_week{
	int calculatedata[7][48];
};

union AIC_charing_week{
	struct AIC_data_week int_data;
	char char_data[sizeof(struct AIC_data_week)];
};

static void update_AIC_setting(void);
static void set_screen_on_time(int end_hr, int end_min, int end_sec);
static void set_charging_time(int end_hr, int end_min, int end_sec);

static void dump_aicdata(void)
{
        int i = 0;
	/*TODO: open dump if necessary.*/
	if(1) return;
        for(i = 0; i <48; i++){
                pr_info("[AIC][aicdata: %d] charging: %d, screen_on: %d sec. [%d] \n", i,  aicdata[i].charging, aicdata[i].screen_on_time, aicdata[i].screen_on_cnt);
        }
}

static void dump_CalculateData(void)
{
        int i = 0,j = 0;
        /*TODO: open dump if necessary.*/
        if(1) return;
        for(j = 0; j < 14; j++){
                for(i = 0; i <48; i ++){
                        pr_info("[AIC][Calculate %2d-%2d] status: %d,  charging: %d, screen_on: %d sec. [%d] \n", j, i, CalculateData[j][i].status?1:0, CalculateData[j][i].charging, CalculateData[j][i].screen_on_time, CalculateData[j][i].screen_on_cnt);
                }
        }
}

static void dump_AIC_setting(void)
{
        int i = 0;
        /*TODO: open dump if necessary.*/
        //if(1) return;
        for(i = 0; i <48; i++){
                pr_info("[AIC][setting: %d] : %d\n", i, AIC_setting[i]?1:0);
        }
}

static ssize_t htc_aic_data(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        int len = 0;
	union AIC_charing data;
	int i = 0, j = 0;
        struct rtc_time tm;
        struct timeval time;
        unsigned long local_time;

	if((aic_data_ready == false) && (aic_data_request_for_shutdown==false)){
		return len;
	}
        pr_info("[AIC]Read SC data\n");
        if(strcmp(htc_get_bootmode(), "") != 0){
                return len;
        }

        do_gettimeofday(&time);
        local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
        rtc_time_to_tm(local_time, &tm);
	pr_info("[AIC]Attributes at (%04d-%02d-%02d %02d:%02d:%02d)\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	if(aic_data_ready == false){
		//Update the current data
		if(screen_on_begin >  0 ){
			pr_info("[AIC]Update the current screen on data.\n");
			set_screen_on_time(tm.tm_hour,tm.tm_min,tm.tm_sec);
                        screen_on_begin = tm.tm_hour *10000 + tm.tm_min * 100 + tm.tm_sec + 1;

                }
                if(charging_begin > 0 ){
                        pr_info("[AIC]Update the current charging data.\n");
			set_charging_time(tm.tm_hour,tm.tm_min,tm.tm_sec);
                        charging_begin = tm.tm_hour *10000 + tm.tm_min * 100 + tm.tm_sec + 1;
		}
	}

		data.int_data.current_time = (tm.tm_mon + 1)*1000000 + (tm.tm_mday)*10000 + (tm.tm_hour) *100 + tm.tm_min;
		data.int_data.aicdata_index = aicdata_index;
		for(j = 0; j < 7; j++){
			for(i = 0; i <48; i ++)
				data.int_data.calculatedata[j][i] = (CalculateData[j][i].status?1:0) + CalculateData[j][i].charging * 10 + CalculateData[j][i].screen_on_time *100 + CalculateData[j][i].screen_on_cnt *1000000;
		}

		for( i=0 ;i <48; i++){
			data.int_data.aicdata[i] = (aicdata[i].status?1:0) + aicdata[i].charging * 10 + aicdata[i].screen_on_time *100 + aicdata[i].screen_on_cnt *1000000;
		}
		len = sizeof(union AIC_charing);
		memcpy(buf, data.char_data, len);
        return len;
}

static ssize_t htc_aic_data1(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        int len = 0;
        union AIC_charing_week data;
        int i = 0, j = 0;

        if((aic_data_ready == false) && (aic_data_request_for_shutdown==false)){
                return len;
        }

        pr_info("[AIC]Read SC data 1.\n");
        if(strcmp(htc_get_bootmode(), "") != 0){
                return len;
        }

	for(j = 0; j < 7; j++){
		for(i = 0; i <48; i ++)
			data.int_data.calculatedata[j][i] = (CalculateData[j+7][i].status?1:0) + CalculateData[j+7][i].charging * 10 + CalculateData[j+7][i].screen_on_time *100 + CalculateData[j+7][i].screen_on_cnt *1000000;
	}

        len = sizeof(union AIC_charing_week);
        memcpy(buf, data.char_data, len);
        aic_data_ready = false;
	aic_data_request_for_shutdown = false;
        return len;
}


static ssize_t htc_aic_data_set(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
        union AIC_charing data;
	int len = sizeof(union AIC_charing);
        int i = 0, j = 0;

        pr_info("[AIC]Set SC data. len = %d\n", len);
        if(strcmp(htc_get_bootmode(), "") != 0){
                return count;
        }

	if(count < len){
		pr_info("[AIC]Set SC data fail. len = %d\n", len);
		aic_init_data = true;
		return count;
	}
        memcpy (data.char_data, buf , len);
	data_req_current_time = data.int_data.current_time;
	pr_info("[AIC]Last save time : %d\n", data_req_current_time);

        aicdata_index = data.int_data.aicdata_index;
        pr_info("[AIC][data index : %d\n", aicdata_index);

	for( i=0 ;i <48; i++){
		aicdata[i].status = (data.int_data.aicdata[i] % 10) == 1;
		aicdata[i].charging = (data.int_data.aicdata[i] % 100) / 10;
		aicdata[i].screen_on_time = (data.int_data.aicdata[i] % 1000000) / 100;
		aicdata[i].screen_on_cnt = data.int_data.aicdata[i] / 1000000;
        }
	dump_aicdata();
        for(j = 0; j < 7; j++){
                for(i = 0; i <48; i ++){
			CalculateData[j][i].status = (data.int_data.calculatedata[j][i] % 10) == 1;
			CalculateData[j][i].charging = (data.int_data.calculatedata[j][i] % 100) / 10;
			CalculateData[j][i].screen_on_time = (data.int_data.calculatedata[j][i] % 1000000) / 100;
			CalculateData[j][i].screen_on_cnt = data.int_data.calculatedata[j][i] / 1000000;
		}
        }

	return count;
}

static ssize_t htc_aic_data_set1(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
        union AIC_charing_week data;
        int len = sizeof(union AIC_charing_week);
        int i = 0, j = 0;

        pr_info("[AIC]Set SC data 1. len = %d\n", len);
        if(strcmp(htc_get_bootmode(), "") != 0){
                return count;
        }

        if(count < len){
                pr_info("[AIC]Set SC data 1 fail. len = %d\n", len);
                aic_init_data = true;
                return count;
        }
        memcpy (data.char_data, buf , len);

         for(j = 0; j < 7; j++){
                for(i = 0; i <48; i ++){
                        CalculateData[j+7][i].status = (data.int_data.calculatedata[j][i] % 10) == 1;
                        CalculateData[j+7][i].charging = (data.int_data.calculatedata[j][i] % 100) / 10;
                        CalculateData[j+7][i].screen_on_time = (data.int_data.calculatedata[j][i] % 1000000) / 100;
                        CalculateData[j+7][i].screen_on_cnt = data.int_data.calculatedata[j][i] / 1000000;
                }
        }
	aic_init_data = false;
	dump_CalculateData();
	return count;
}

static ssize_t htc_aic_ready(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "%s\n", aic_data_ready? "YES" : "NO");
}

static ssize_t htc_aic_data_request(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	aic_data_request_for_shutdown = true;
        return count;
}

static void init_aicdata(void)
{
        int i = 0,j = 0;
	aicdata_index = 13;
	pr_info("[AIC]No data, create new data.\n");
        for(j = 0; j < 14; j++){
                for(i = 0; i <48; i ++){
			if(( i < 12)|| (i >=44)){
				CalculateData[j][i].status = true;
				CalculateData[j][i].charging = 2;
				CalculateData[j][i].screen_on_time = 0;
				CalculateData[j][i].screen_on_cnt = 0;
			}else {
                                CalculateData[j][i].status = false;
                                CalculateData[j][i].charging = 0;
                                CalculateData[j][i].screen_on_time = 4;
                                CalculateData[j][i].screen_on_cnt = 15;
			}
                }
        }
        for(i = 0; i <48; i++){
		if(( i < 12)|| (i >=44)){
			AIC_setting[i] = true;
		}else
			AIC_setting[i] = false;
        }
}

static void update_AIC_setting(void)
{
        int i = 0,j = 0, n = 0, m = 0;
	int total_cnt = 0;
	bool check_charging = false;
	bool check_over_day = false;
        pr_info("[AIC]Update AIC setting. Index: %d\n", aicdata_index);
	if(aicdata_index < 13) return;
	for(i = 0; i < 48; i++){
		for(j = 0; j < 14; j++){
			total_cnt += CalculateData[j][i].status? 1 : 0;
			if(CalculateData[j][i].charging == 2)
				check_charging = true;
                }
		if((total_cnt > 11) && (check_charging))
			AIC_setting[i]= true;
		else
			AIC_setting[i]= false;
		total_cnt = 0;
		check_charging = false;
	}
	dump_AIC_setting();
	for(i = 0; i < 48; i++){
		if(AIC_setting[i]){
			for( j = i+1 ;j <48 ;j++){
				if(AIC_setting[j] == false) break;
			}
			if( i == 0) {
				for ( n = 47; n > 0; n--){
					if(AIC_setting[n] == false) break;
				}
				check_over_day = true;
			}else
				n = 47;

			if((j == 48) && (check_over_day)){
				pr_info("[AIC]Checking done. Ignore this.\n");
			}
			else if((j - i+ 47 - n) < 7){
					if( n < 47){
						for ( m = 47; m > n; m--){
							AIC_setting[m] = false;
						}
	                                }
					for(n =  i ; n < j; n++){
						AIC_setting[n] = false;
					}
				}
			i = j;
		}
	}
	aic_data_ready = true;
	dump_AIC_setting();
}

static void update_AIC_data(void)
{
        int i = 0,j = 0;
	pr_info("[AIC]Update AIC_data: index=%d\n", aicdata_index);
	dump_aicdata();
        if(aicdata_index < 13){
                for(i = 0; i <48; i ++){
			if((aicdata[i].screen_on_cnt <= 3) && (aicdata[i].screen_on_time <= 180))
	                        CalculateData[aicdata_index][i].status = true;
			else
                                CalculateData[aicdata_index][i].status = false;
                        aicdata[i].status = 0;
                        CalculateData[aicdata_index][i].charging = aicdata[i].charging;
                        aicdata[i].charging = 0;
                        CalculateData[aicdata_index][i].screen_on_cnt = aicdata[i].screen_on_cnt;
                        aicdata[i].screen_on_cnt = 0;
                        CalculateData[aicdata_index][i].screen_on_time  = aicdata[i].screen_on_time;
                        aicdata[i].screen_on_time = 0;
                }
		aicdata_index++;
        }else{
                for(j = 0; j < 14; j++){
                        for(i = 0; i <48; i ++){
                                if(j == 13){
					if((aicdata[i].screen_on_cnt <= 3) && (aicdata[i].screen_on_time <= 180))
						CalculateData[j][i].status = true;
					else
						CalculateData[j][i].status = false;
                                        aicdata[i].status = 0;
                                        CalculateData[j][i].charging = aicdata[i].charging;
                                        aicdata[i].charging = 0;
                                        CalculateData[j][i].screen_on_cnt = aicdata[i].screen_on_cnt;
                                        aicdata[i].screen_on_cnt = 0;
                                        CalculateData[j][i].screen_on_time  = aicdata[i].screen_on_time;
                                        aicdata[i].screen_on_time = 0;
                                }else{
                                        CalculateData[j][i].status = CalculateData[j+1][i].status;
                                        CalculateData[j][i].charging = CalculateData[j+1][i].charging;
                                        CalculateData[j][i].screen_on_cnt = CalculateData[j+1][i].screen_on_cnt;
                                        CalculateData[j][i].screen_on_time  = CalculateData[j+1][i].screen_on_time;
                                }
                        }
                }
        }
	dump_CalculateData();
	update_AIC_setting();
}

static void cross_day_checking(int entrymode)
{
        int old_hr = 0, old_min = 0, old_sec = 0;
        int i = 0;
        int begin_index = 0, begin_time = 0;
        int end_index = 0, end_time = 0;
	int cur_time = 0;

        pr_info("[AIC]Cross day update: mode = %d\n", entrymode);

	if(((entrymode == IS_DISCHARGE) || (entrymode == IS_CROSS_DAY)) && (screen_on_begin > 0)) {
		pr_info("[AIC]Update screen on data.\n");

                        old_hr = screen_on_begin / 10000;
                        old_min = (screen_on_begin % 10000) / 100;
                        old_sec = screen_on_begin % 100 - 1;

                        begin_index = old_hr * 2;
                        begin_index += (old_min < 30)? 0:1;
                        end_index = 47;

                        begin_time = old_hr * 3600 + old_min * 60 + old_sec;
                        end_time = 3600 * 24;

                        for (i = begin_index; i <= end_index; i++){
                                if ( i != begin_index)
                                        aicdata[i].screen_on_cnt += 1;
	                        cur_time = (i+1) * 30 * 60;
                                aicdata[i].screen_on_time += cur_time - begin_time;
                                begin_time = cur_time;
                        }
                        screen_on_begin = 1;

	}
        if(((entrymode == IS_SCREEN_OFF) || (entrymode == IS_CROSS_DAY)) && (charging_begin > 0)){
                pr_info("[AIC]Update charging data.\n");

                        old_hr = charging_begin / 10000;
                        old_min = (charging_begin % 10000) / 100;
                        old_sec = charging_begin % 100 - 1;
                        begin_index = old_hr * 2;
                        begin_index += (old_min < 30)? 0:1;
                        end_index = 47;

                        begin_time = old_hr * 3600 + old_min * 60 + old_sec;
                        end_time = 24 * 3600;
                        Crossday_charging_begin = end_time - begin_time;

                        for (i = begin_index; i <= end_index; i++){
	                        cur_time = (i+1) * 30 * 60;
                                aicdata[i].charging = (cur_time > charging_full)? 2:1;
                        }

                        charging_begin = 1;
                        charging_full = 1;
	}
	update_AIC_data();
}

static void set_screen_on_cnt(int hr,int min)
{
        int index = 0;
        index = hr * 2;
        index += (min < 30)? 0:1;
        aicdata[index].screen_on_cnt += 1;
        pr_info("[AIC]Set screen on : index:%d count:%d\n",index, aicdata[index].screen_on_cnt);
//	dump_aicdata();
        return;
}

static void set_screen_on_time(int end_hr, int end_min, int end_sec)
{
        int old_hr = 0, old_min = 0, old_sec = 0;
        int i = 0;
        int begin_index = 0, begin_time = 0;
        int end_index = 0, end_time = 0;
	int cur_time = 0;

        old_hr = screen_on_begin / 10000;
        old_min = (screen_on_begin % 10000) / 100;
        old_sec = screen_on_begin % 100 - 1;
        begin_index = old_hr * 2;
        begin_index += (old_min < 30)? 0:1;
        end_index = end_hr *2;
        end_index += (end_min <30) ? 0:1;

        begin_time = old_hr * 3600 + old_min * 60 + old_sec;
        end_time = end_hr * 3600 + end_min * 60 + end_sec;

        pr_info("[AIC]Set screen on time : begin : %6d end: %6d\n", begin_time, end_time);

        if(begin_index == end_index){
                aicdata[begin_index].screen_on_time += (end_min - old_min)*60 + end_sec - old_sec;
        }else if (begin_index > end_index){
                //Over 24hr.
                for(i = begin_index; i < 48; i++){
                        if ( i != begin_index)
                                aicdata[i].screen_on_cnt += 1;
                        cur_time = (i+1) * 30 * 60;
                        aicdata[i].screen_on_time += cur_time - begin_time;
                        begin_time = cur_time;
                }
		cross_day_checking(IS_SCREEN_OFF);

		begin_time = 0;
                for(i = 0; i <= end_index; i++){
                        aicdata[i].screen_on_cnt += 1;
                        cur_time = (i+1) * 30 * 60;
                        if(cur_time > end_time){
                                //the lastest time stage.
                                aicdata[i].screen_on_time += end_time - begin_time;
                        }else{
                                aicdata[i].screen_on_time += cur_time - begin_time;
                        }
                        begin_time = cur_time;
                }
        }else{
                for (i = begin_index; i <= end_index; i++){
                        if ( i != begin_index)
                                aicdata[i].screen_on_cnt += 1;
                        cur_time = (i+1) * 30 * 60;
                        if(cur_time > end_time){
                                //the lastest time stage.
                                aicdata[i].screen_on_time += end_time - begin_time;
                        }else{
                                aicdata[i].screen_on_time += cur_time - begin_time;
                        }
                        begin_time = cur_time;
                }
        }
	dump_aicdata();
        screen_on_begin = 0;
}

static void set_charging_time(int end_hr, int end_min, int end_sec)
{
        int old_hr = 0, old_min = 0, old_sec = 0;
        int i = 0;
        int begin_index = 0, begin_time = 0;
        int end_index = 0, end_time = 0;
	int cur_time = 0;

        old_hr = charging_begin / 10000;
        old_min = (charging_begin % 10000) / 100;
        old_sec = charging_begin % 100 - 1;
        begin_index = old_hr * 2;
        begin_index += (old_min < 30)? 0:1;
        end_index = end_hr *2;
        end_index += (end_min <30) ? 0:1;

        begin_time = old_hr * 3600 + old_min * 60 + old_sec;
        end_time = end_hr * 3600 + end_min * 60 + end_sec;

	pr_info("[AIC]set charging status : begin : %6d end: %6d\n",begin_time, end_time);

        if (begin_index > end_index){
                if ( (3600 * 24 - begin_time + end_time) >= 10800){
                //Over 24hr.
			for(i = begin_index; i < 48; i++){
				cur_time = (i+1) * 30 * 60;
				aicdata[i].charging = (cur_time > charging_full)? 2:1;
			}
			cross_day_checking(IS_DISCHARGE);
			begin_time = 0;
			for(i = 0; i <= end_index; i++){
				cur_time = (i+1) * 30 * 60;
	                        if(charging_full > cur_time)
					aicdata[i].charging = 2;
				else
					aicdata[i].charging = (cur_time > charging_full)? 2:1;
	                }
		}
        }else{
                if((end_time - begin_time + Crossday_charging_begin < 10800)&& (begin_time !=0)) {
			//Do nothing.
		}else{
	                for (i = begin_index; i <= end_index; i++){
				cur_time = (i+1) * 30 * 60;
				aicdata[i].charging = (cur_time > charging_full)? 2:1;
	               }
		}
        }
	dump_aicdata();
        charging_begin = 0;
	Crossday_charging_begin = 0;
        charging_full = 0;
}

static void check_AIC_setting(int screen_on_cnt)
{
	static int entry_AIC_charing_cnt = 0;
        int old_hr = 0, old_min = 0;
        int index = 0, index_now = 0;

        struct rtc_time tm;
        struct timeval time;
        unsigned long local_time;
        do_gettimeofday(&time);
        local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
        rtc_time_to_tm(local_time, &tm);

        old_hr = charging_begin / 10000;
        old_min = (charging_begin % 10000) / 100;
        index = old_hr * 2;
        index += (old_min < 30)? 0:1;

	index_now = tm.tm_hour * 2;
	index_now = tm.tm_min<30? 0:1;

	pr_info("[AIC]hour= %d(%d), minutes = %d(%d)\n", tm.tm_hour, old_hr, tm.tm_min, old_min);

	if(screen_on_cnt == 0) entry_AIC_charing_cnt = 0;

	if((AIC_setting[index]) && (AIC_setting[(index + 2)%48])&&(AIC_setting[index_now])){
		if((gbAICharging == false)&&(entry_AIC_charing_cnt == 0)){
			//Entry first plug in
			entry_AIC_charing_cnt = screen_on_cnt == 0? 1 : screen_on_cnt;
			pr_info("[AIC]Enable AIC charging.\n");
	                gbAICharging = true;

		}else if ((gbAICharging)&& (screen_on_cnt > entry_AIC_charing_cnt)){
			gbAICharging = false;
		}
	}else{
		entry_AIC_charing_cnt = 0;
		if(gbAICharging)
			pr_info("[AIC]leave AIC charging.\n");
                gbAICharging = false;
	}
}

static void set_aicdata(int datamode)
{
        static int last_screen = 0, last_charge = 0;
	static int last_hr = 0;
        struct rtc_time tm;

	static bool enable_checking = false;
	static int screen_on_after_charging = 0;
	struct timeval time;
	unsigned long local_time;
	static bool b_bootup = false;
        struct timespec xtime = CURRENT_TIME;
        static unsigned long currtime_s = 0;
	int i = 0;
	do_gettimeofday(&time);
	local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &tm);

	if(strcmp(htc_get_bootmode(), "") != 0){
		return;
	}

	if(currtime_s == 0){
		currtime_s = xtime.tv_sec;
		return;
	}

	if((xtime.tv_sec - currtime_s) <  3600 * 24 * 365)
	{
	        if(datamode == IS_SCREEN_ON){
                        last_screen = BOOT_SCREEN_ON;
	        }else if(datamode == IS_SCREEN_OFF){
			last_screen = IS_SCREEN_OFF;
		}
		return;
	}

        if((datamode == last_screen)||(datamode == last_charge)) {
		if((last_hr == 23) &&(tm.tm_hour == 0) && (enable_checking == false)){
			cross_day_checking(IS_CROSS_DAY);
		}
		last_hr = tm.tm_hour;
		if(datamode == IS_CHARGING)
			check_AIC_setting(screen_on_after_charging);
                return;
        }

        if(BOOT_SCREEN_ON == last_screen) {
		if(aic_init_data){
			pr_info("[AIC]Init the data for no files.\n");
			init_aicdata();
		}else{
			if((data_req_current_time/10000) != ((tm.tm_mon + 1)*100 + (tm.tm_mday))){
		                pr_info("[AIC]Ignore the aicdata\n ");
				for( i=0 ;i <48; i++){
					aicdata[i].status = false;
					aicdata[i].charging = 0;
					aicdata[i].screen_on_time = 0;
					aicdata[i].screen_on_cnt = 0;
			        }
			}
		}
		update_AIC_setting();
		//First time
                pr_info("[AIC]Boot up screen on: set last screen, count and begin time. \n") ;
		last_screen = IS_SCREEN_ON;
		screen_on_begin = tm.tm_hour *10000 + tm.tm_min * 100 + tm.tm_sec + 1;
		set_screen_on_cnt(tm.tm_hour,tm.tm_min);
		screen_on_after_charging++;
		if((datamode == IS_CHARGING) || (datamode == IS_CHARGE_FULL))
			b_bootup = true;
	}
	enable_checking = true;
        switch(datamode){
                case IS_SCREEN_ON:
			pr_info("[AIC] IS_SCREEN_ON [%d] \n", gbAICharging? 1: 0);
                        screen_on_begin = tm.tm_hour *10000 + tm.tm_min * 100 + tm.tm_sec + 1;
                        set_screen_on_cnt(tm.tm_hour,tm.tm_min);
			if((screen_on_after_charging > 0)&&(gbAICharging))
				gbAICharging = false;
			screen_on_after_charging++;
                        last_screen = IS_SCREEN_ON;
                        break;
                case IS_SCREEN_OFF:
                        pr_info("[AIC] IS_SCREEN_OFF[%d] \n", gbAICharging? 1: 0);
                        if(screen_on_begin >  0 ){
                                set_screen_on_time(tm.tm_hour,tm.tm_min,tm.tm_sec);
                        }
                        last_screen = IS_SCREEN_OFF;
                        break;
                case IS_CHARGING:
                        pr_info("[AIC] IS_CHARGING.[%d] \n", gbAICharging? 1: 0);
                        charging_begin = tm.tm_hour *10000 + tm.tm_min * 100 + tm.tm_sec + 1;
			if(b_bootup){
				b_bootup = false;
                                screen_on_after_charging = 1;
                        }else
				screen_on_after_charging = 0;
			check_AIC_setting(screen_on_after_charging);
                        last_charge = IS_CHARGING;
                        break;
                case IS_DISCHARGE:
                        pr_info("[AIC] IS_DISCHARGING[%d] \n", gbAICharging? 1: 0);
                        if(charging_begin > 0 ){
                                set_charging_time(tm.tm_hour,tm.tm_min,tm.tm_sec);
                        }
			gbAICharging = false;
                        last_charge = IS_DISCHARGE;
                        break;
                case IS_CHARGE_FULL:
                        pr_info("[AIC] IS_CHARGE_FULL[%d] \n", gbAICharging? 1: 0);
                        charging_full = tm.tm_hour *10000 + tm.tm_min * 100 + tm.tm_sec + 1;
			if(charging_begin == 0){
				charging_begin = charging_full;
				if(b_bootup){
					b_bootup = false;
		                        screen_on_after_charging = 1;
				}else
					screen_on_after_charging = 0;
				check_AIC_setting(screen_on_after_charging);
			}
                        last_charge = IS_CHARGE_FULL;
                        break;
                default:
                        break;
        }
	enable_checking = false;
}

static const int g_DEC_LEVEL_CURR_TABLE_SIZE = sizeof(g_dec_level_curr_table) / sizeof (g_dec_level_curr_table[0]);

#define BOUNDING_RECHARGE_NORMAL		5;
#define BOUNDING_RECHARGE_ATS		20;
static int is_bounding_fully_charged_level(void)
{
	static int s_pingpong = 1;
	int is_batt_chg_off_by_bounding = 0;
	int upperbd = htc_batt_info.rep.full_level;
	int current_level = htc_batt_info.rep.level;
	int lowerbd;

	if (g_flag_ats_limit_chg) {
		lowerbd = upperbd - BOUNDING_RECHARGE_ATS;	// 20% range
	} else {
		lowerbd = upperbd - BOUNDING_RECHARGE_NORMAL;	// 5% range
	}

	if (0 < htc_batt_info.rep.full_level &&
			htc_batt_info.rep.full_level < 100) {

		if (lowerbd < 0)
			lowerbd = 0;

		if (s_pingpong == 1 && upperbd <= current_level) {
			BATT_LOG("%s: lowerbd=%d, upperbd=%d, current=%d,"
					" pingpong:1->0 turn off\n", __func__, lowerbd, upperbd, current_level);
			is_batt_chg_off_by_bounding = 1;
			s_pingpong = 0;
		} else if (s_pingpong == 0 && lowerbd < current_level) {
			BATT_LOG("%s: lowerbd=%d, upperbd=%d, current=%d,"
					" toward 0, turn off\n", __func__, lowerbd, upperbd, current_level);
			is_batt_chg_off_by_bounding = 1;
		} else if (s_pingpong == 0 && current_level <= lowerbd) {
			BATT_LOG("%s: lowerbd=%d, upperbd=%d, current=%d,"
					" pingpong:0->1 turn on\n", __func__, lowerbd, upperbd, current_level);
			s_pingpong = 1;
		} else {
			BATT_LOG("%s: lowerbd=%d, upperbd=%d, current=%d,"
					" toward %d, turn on\n", __func__, lowerbd, upperbd, current_level, s_pingpong);
		}
	}

	return is_batt_chg_off_by_bounding;
}

static void batt_set_check_timer(u32 seconds)
{
	pr_debug("[BATT] %s(%u sec)\n", __func__, seconds);
	mod_timer(&htc_batt_timer.batt_timer,
			jiffies + msecs_to_jiffies(seconds * 1000));
}

static int get_property(struct power_supply *psy, enum power_supply_property prop)
{
	union power_supply_propval ret = {0, };
	int rc = 0;

	if (psy) {
		rc = power_supply_get_property(psy, prop, &ret);
		if (rc) {
			pr_err("[BATT] failed to retrieve value rc=%d\n", rc);
			return -1;
		}
	} else {
		pr_err("[BATT] batt_psy is null.\n");
		return -1;
	}

	return ret.intval;
}

static int set_batt_psy_property(enum power_supply_property prop, int value)
{
	union power_supply_propval ret = {0, };
	int rc = -1;

	if (htc_batt_info.batt_psy) {
		BATT_EMBEDDED("set_batt_psy_property. value(%d) prop(%d)", value, prop);
		ret.intval = value;
		rc = power_supply_set_property(htc_batt_info.batt_psy, prop, &ret);
	}

	return rc;
}

static int set_usb_psy_property(enum power_supply_property prop, int value)
{
	union power_supply_propval ret = {0, };
	int rc = -1;

	if (htc_batt_info.batt_psy) {
		BATT_EMBEDDED("set_usb_psy_property. value(%d) prop(%d)", value, prop);
		ret.intval = value;
		rc = power_supply_set_property(htc_batt_info.usb_psy, prop, &ret);
	}

	return rc;
}

static int set_main_psy_property(enum power_supply_property prop, int value)
{
	union power_supply_propval ret = {0, };
	int rc = -1;

	if (htc_batt_info.batt_psy) {
		BATT_EMBEDDED("set_main_psy_property. value(%d) prop(%d)", value, prop);
		ret.intval = value;
		rc = power_supply_set_property(htc_batt_info.main_psy, prop, &ret);
	}

	return rc;
}

#define CHG_ONE_PERCENT_LIMIT_PERIOD_MS	(1000 * 60)
#define LEVEL_GAP_BETWEEN_UI_AND_RAW		3
static void batt_check_overload(unsigned long time_since_last_update_ms)
{
	static unsigned int overload_count = 0;
	static unsigned long time_accumulation = 0;
	static unsigned int s_prev_level_raw = 0;

	pr_debug("[BATT] Chk overload by CS=%d V=%d I=%d count=%d overload=%d "
			"is_full=%d\n",
			htc_batt_info.rep.charging_source, htc_batt_info.rep.batt_vol,
			htc_batt_info.rep.batt_current, overload_count, htc_batt_info.rep.overload,
			htc_batt_info.rep.is_full);
	if ((htc_batt_info.rep.charging_source > 0) &&
			(!htc_batt_info.rep.is_full) &&
			((htc_batt_info.rep.batt_current / 1000) >
				htc_batt_info.overload_curr_thr_ma)) {
		time_accumulation += time_since_last_update_ms;
		if (time_accumulation >= CHG_ONE_PERCENT_LIMIT_PERIOD_MS) {
			if (overload_count++ < 3) {
				htc_batt_info.rep.overload = 0;
			} else
				htc_batt_info.rep.overload = 1;

			/* Treat overload is happening if UI/raw level gap > 3% */
			if ((htc_batt_info.rep.level - htc_batt_info.rep.level_raw)
					>= LEVEL_GAP_BETWEEN_UI_AND_RAW)
				htc_batt_info.rep.overload = 1;

			time_accumulation = 0;
		}
	} else {
		if ((htc_batt_info.rep.charging_source > 0) && (htc_batt_info.rep.overload == 1)) {
			if (htc_batt_info.rep.level_raw > s_prev_level_raw) {
				overload_count = 0;
				time_accumulation = 0;
				htc_batt_info.rep.overload = 0;
			}
		} else { /* Cable is removed */
			overload_count = 0;
			time_accumulation = 0;
			htc_batt_info.rep.overload = 0;
		}
	}
	s_prev_level_raw = htc_batt_info.rep.level_raw;
}

int batt_check_consistent(void)
{
	struct timespec xtime = CURRENT_TIME;
	unsigned long currtime_s = (xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC)/MSEC_PER_SEC;

	/* Restore battery data for keeping soc stable */
	if (htc_batt_info.store.batt_stored_magic_num == STORE_MAGIC_NUM
		&& htc_batt_info.rep.batt_temp > 20
                && htc_batt_info.store.batt_stored_temperature > 20
		&& (abs(htc_batt_info.rep.level_raw - htc_batt_info.store.batt_stored_soc) < 10)
		&& (htc_batt_info.rep.level_raw > 5 )) {
		htc_batt_info.store.consistent_flag = true;
	}

	BATT_EMBEDDED("%s: magic_num=0x%X, level_raw=%d, store_soc=%d, current_time:%lu, store_time=%lu,"
				" batt_temp=%d, store_temp=%d, consistent_flag=%d", __func__,
				htc_batt_info.store.batt_stored_magic_num, htc_batt_info.rep.level_raw,
				htc_batt_info.store.batt_stored_soc,currtime_s,
				htc_batt_info.store.batt_stored_update_time,htc_batt_info.rep.batt_temp,
				htc_batt_info.store.batt_stored_temperature,
				htc_batt_info.store.consistent_flag);

	return htc_batt_info.store.consistent_flag;
}

int htc_battery_level_adjust(void)
{
	while (!g_is_rep_level_ready)
		msleep(50);
	return htc_batt_info.rep.level;
}

static void batt_check_critical_low_level(int *dec_level, int batt_current)
{
	int	i;

	for(i = 0; i < g_DEC_LEVEL_CURR_TABLE_SIZE; i++) {
		if (batt_current > g_dec_level_curr_table[i].threshold_ua) {
			*dec_level = g_dec_level_curr_table[i].dec_level;

			pr_debug("%s: i=%d, dec_level=%d, threshold_ua=%d\n",
				__func__, i, *dec_level, g_dec_level_curr_table[i].threshold_ua);
			break;
		}
	}
}

static void adjust_store_level(int *store_level, int drop_raw, int drop_ui, int prev) {
	int store = *store_level;
	/* To calculate next store_vale between UI and Raw level*/
	store += drop_raw - drop_ui;
	if (store >= 0)
		htc_batt_info.rep.level = prev - drop_ui;
	else {
		htc_batt_info.rep.level = prev;
		store += drop_ui;
	}
	*store_level = store;
}

#define DISCHG_UPDATE_PERIOD_MS			(1000 * 60)
#define ONE_PERCENT_LIMIT_PERIOD_MS		(1000 * (60 + 10))
#define FIVE_PERCENT_LIMIT_PERIOD_MS	(1000 * (300 + 10))
#define ONE_MINUTES_MS					(1000 * (60 + 10))
#define FOURTY_MINUTES_MS				(1000 * (2400 + 10))
#define SIXTY_MINUTES_MS				(1000 * (3600 + 10))
#define CHG_ONE_PERCENT_LIMIT_PERIOD_MS	(1000 * 60)
#define QUICK_CHG_ONE_PERCENT_LIMIT_PERIOD_MS	(1000 * 30)
#define DEMO_GAP_WA						6
static void batt_level_adjust(unsigned long time_since_last_update_ms)
{
	static int s_first = 1;
	static int s_critical_low_enter = 0;
	static int s_store_level = 0;
	static int s_pre_five_digit = 0;
	static int s_five_digit = 0;
	static bool s_stored_level_flag = false;
	static bool s_allow_drop_one_percent_flag = false;
	int dec_level = 0;
	int dropping_level;
	int drop_raw_level;
	int allow_suspend_drop_level = 0;
	static unsigned long time_accumulated_level_change = 0;

	if (s_first) {
		htc_batt_info.rep.level_raw = get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_CAPACITY);
		if (batt_check_consistent())
			htc_batt_info.rep.level = htc_batt_info.store.batt_stored_soc;
		else
			htc_batt_info.rep.level = htc_batt_info.rep.level_raw;

		htc_batt_info.prev.level = htc_batt_info.rep.level;
		htc_batt_info.prev.level_raw= htc_batt_info.rep.level_raw;
		s_pre_five_digit = htc_batt_info.rep.level / 10;
		htc_batt_info.prev.charging_source = htc_batt_info.rep.charging_source;
		BATT_LOG("pre_level=%d,pre_raw_level=%d,pre_five_digit=%d,pre_chg_src=%d\n",
			htc_batt_info.prev.level,htc_batt_info.prev.level_raw,s_pre_five_digit,
			htc_batt_info.prev.charging_source);
	}
	drop_raw_level = htc_batt_info.prev.level_raw - htc_batt_info.rep.level_raw;
	time_accumulated_level_change += time_since_last_update_ms;

	if ((htc_batt_info.prev.charging_source > 0) &&
		htc_batt_info.rep.charging_source == 0 && htc_batt_info.prev.level == 100) {
		BATT_LOG("%s: Cable plug out when level 100, reset timer.\n",__func__);
		time_accumulated_level_change = 0;
		htc_batt_info.rep.level = htc_batt_info.prev.level;

		return;
	}

	/* In discharging case, to store the very first difference
	 * between UI and Raw level.
	 * In case of Overload follow the remap logic.
	 */
	if (((htc_batt_info.rep.charging_source == 0)
			&& (s_stored_level_flag == false)) ||
			htc_batt_info.rep.overload ) {
		s_store_level = htc_batt_info.prev.level - htc_batt_info.rep.level_raw;
		BATT_LOG("%s: Cable plug out, to store difference between"
			" UI & SOC. store_level:%d, prev_level:%d, raw_level:%d\n"
			,__func__, s_store_level, htc_batt_info.prev.level, htc_batt_info.rep.level_raw);
		s_stored_level_flag = true;
	} else if (htc_batt_info.rep.charging_source > 0)
		s_stored_level_flag = false;

	/* ++++ For cc pin no debounce done WA ++++ */
	if ((htc_batt_info.rep.charging_source == 0) && (htc_batt_info.rep.status == POWER_SUPPLY_STATUS_CHARGING) &&
			(get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_PRESENT))) {
		if ((htc_batt_info.rep.level_raw == 100) ||
				((htc_batt_info.rep.level_raw > htc_batt_info.prev.level_raw) &&
				(htc_batt_info.rep.level_raw >= (htc_batt_info.rep.level + 2)))) {
			// Set UI level = raw level when
			// 1. raw is 100%
			// 2. raw increase & raw > UI 2%
			s_store_level = 0;
			htc_batt_info.rep.level = htc_batt_info.rep.level_raw;

			htc_batt_info.prev.level = htc_batt_info.rep.level;
			htc_batt_info.prev.level_raw = htc_batt_info.rep.level_raw;
			return;
		}
	}
	/* ---- For cc pin no debounce done WA ---- */

	/* Let it enter charging state to prevent reporting 100% directly
	   if soc is 100% if cable is just inserted or boot up with cable in.
	   In case of Overload follow the remap logic.*/
	if ((!gs_prev_charging_enabled &&
			!((htc_batt_info.prev.charging_source == 0) &&
				htc_batt_info.rep.charging_source > 0)) || htc_batt_info.rep.overload) {
		/* battery discharging - level smoothen algorithm:
		 * Rule: always report 1% before report 0%
		 * IF VBATT < CRITICAL_LOW_VOLTAGE THEN
		 *		drop level by 6%
		 * ELSE
		 * - level cannot drop over 2% in 1 minute (here use 60 + 10 sec).
		 * - level cannot drop over 5% in 5 minute (here use 300 + 10 sec).
		 * - level cannot drop over 3% in 60 minute.
		 * - level cannot increase while discharging.
		 */
		if (time_accumulated_level_change < DISCHG_UPDATE_PERIOD_MS
				&& !s_first) {
			/* level should keep the previous one */
			BATT_DEBUG("%s: total_time since last batt level update = %lu ms.",
			__func__, time_accumulated_level_change);
			htc_batt_info.rep.level = htc_batt_info.prev.level;
			s_store_level += drop_raw_level;

			g_is_consistent_level_ready = true;
			return;
		}

		if ((htc_batt_info.rep.batt_vol < htc_batt_info.critical_low_voltage_mv)
				|| (htc_batt_info.rep.level_raw <= htc_batt_info.low_batt_check_soc_raw_threshold)) {
			s_critical_low_enter = 1;
			/* batt voltage is under critical low condition */
			if (htc_batt_info.decreased_batt_level_check)
				batt_check_critical_low_level(&dec_level,
					htc_batt_info.rep.batt_current);
			else
				dec_level = 6;

			if (htc_batt_info.rep.batt_vol < htc_batt_info.critical_low_voltage_mv)
				htc_batt_info.rep.level =
						(htc_batt_info.prev.level > dec_level) ? (htc_batt_info.prev.level - dec_level) : 0;
			else /* if (htc_batt_info.prev.level <= htc_batt_info.low_batt_check_soc_raw_threshold) */
				htc_batt_info.rep.level =
						(((int)htc_batt_info.prev.level - dec_level) > (int)htc_batt_info.rep.level_raw) ?
							(htc_batt_info.prev.level - dec_level) : htc_batt_info.rep.level_raw;

			BATT_LOG("%s: battery level force decreses %d%% from %d%%"
					" (soc=%d)on critical low (%d mV)(%d uA)\n", __func__, dec_level, htc_batt_info.prev.level,
						htc_batt_info.rep.level, htc_batt_info.critical_low_voltage_mv,
						htc_batt_info.rep.batt_current);
		} else {
			/* Always allows to drop stored 1% below 30% */
			/* Allows to drop stored 1% when UI - Raw > 10 */
			if ((htc_batt_info.rep.level_raw < 30) ||
					(htc_batt_info.prev.level > (htc_batt_info.prev.level_raw + 10)))
				s_allow_drop_one_percent_flag = true;

			/* Preset the UI Level as Pre-UI */
			htc_batt_info.rep.level = htc_batt_info.prev.level;

			if (time_since_last_update_ms <= ONE_PERCENT_LIMIT_PERIOD_MS) {
				if (1 <= drop_raw_level) {
					adjust_store_level(&s_store_level, drop_raw_level, 1, htc_batt_info.prev.level);
					BATT_LOG("%s: remap: normal soc drop = %d%% in %lu ms."
							" UI only allow -1%%, store_level:%d, ui:%d%%\n",
							__func__, drop_raw_level, time_since_last_update_ms,
							s_store_level, htc_batt_info.rep.level);
				}
			} else if ((suspend_highfreq_check_reason & SUSPEND_HIGHFREQ_CHECK_BIT_TALK) &&
				(time_since_last_update_ms <= FIVE_PERCENT_LIMIT_PERIOD_MS)) {
				if (5 < drop_raw_level) {
					adjust_store_level(&s_store_level, drop_raw_level, 5, htc_batt_info.prev.level);
				} else if (1 <= drop_raw_level && drop_raw_level <= 5) {
					adjust_store_level(&s_store_level, drop_raw_level, 1, htc_batt_info.prev.level);
				}
				BATT_LOG("%s: remap: phone soc drop = %d%% in %lu ms.\n"
						" UI only allow -1%% or -5%%, store_level:%d, ui:%d%%\n",
						__func__, drop_raw_level, time_since_last_update_ms,
						s_store_level, htc_batt_info.rep.level);
			} else {
				if (1 <= drop_raw_level) {
					if ((ONE_MINUTES_MS < time_since_last_update_ms) &&
							(time_since_last_update_ms <= FOURTY_MINUTES_MS)) {
						allow_suspend_drop_level = 4;
					} else if ((FOURTY_MINUTES_MS < time_since_last_update_ms) &&
							(time_since_last_update_ms <= SIXTY_MINUTES_MS)) {
						allow_suspend_drop_level = 6;
					} else if (SIXTY_MINUTES_MS < time_since_last_update_ms) {
						allow_suspend_drop_level = 8;
					}
					/* allow_suspend_drop_level (4/6/8) is temporary setting, original is (1/2/3) */
					if (allow_suspend_drop_level != 0) {
						if (allow_suspend_drop_level <= drop_raw_level) {
							adjust_store_level(&s_store_level, drop_raw_level, allow_suspend_drop_level, htc_batt_info.prev.level);
						} else {
							adjust_store_level(&s_store_level, drop_raw_level, drop_raw_level, htc_batt_info.prev.level);
						}
					}
					BATT_LOG("%s: remap: suspend soc drop: %d%% in %lu ms."
							" UI only allow -1%% to -8%%, store_level:%d, ui:%d%%, suspend drop:%d%%\n",
							__func__, drop_raw_level, time_since_last_update_ms,
							s_store_level, htc_batt_info.rep.level, allow_suspend_drop_level);
				}
			}

			if ((s_allow_drop_one_percent_flag == false)
					&& (drop_raw_level == 0)) {
				htc_batt_info.rep.level = htc_batt_info.prev.level;
				BATT_DEBUG("%s: remap: no soc drop and no additional 1%%,"
						" ui:%d%%\n", __func__, htc_batt_info.rep.level);
			} else if ((s_allow_drop_one_percent_flag == true)
					&& (drop_raw_level == 0)
					&& (s_store_level > 0)) {
				s_store_level--;
				htc_batt_info.rep.level = htc_batt_info.prev.level - 1;
				s_allow_drop_one_percent_flag = false;
				BATT_LOG("%s: remap: drop additional 1%%. store_level:%d,"
						" ui:%d%%\n", __func__, s_store_level
						, htc_batt_info.rep.level);
			} else if (drop_raw_level < 0) {
				/* soc increased in discharging state:
				 * do not allow level increase. */
				if (s_critical_low_enter) {
					BATT_LOG("%s: remap: level increase because of"
							" exit critical_low!\n", __func__);
				}
				s_store_level += drop_raw_level;
				htc_batt_info.rep.level = htc_batt_info.prev.level;
				BATT_LOG("%s: remap: soc increased. store_level:%d,"
						" ui:%d%%\n", __func__, s_store_level, htc_batt_info.rep.level);
			}

			/* Allow to minus additional 1% in every 5% */
			s_five_digit = htc_batt_info.rep.level / 5;
			if (htc_batt_info.rep.level != 100) {
				/* In every 5% */
				if ((s_pre_five_digit <= 18) && (s_pre_five_digit > s_five_digit)) {
					s_allow_drop_one_percent_flag = true;
					BATT_LOG("%s: remap: allow to drop additional 1%% at next"
							" level:%d%%.\n", __func__, htc_batt_info.rep.level - 1);
				}
			}
			s_pre_five_digit = s_five_digit;

			if (s_critical_low_enter) {
				s_critical_low_enter = 0;
				pr_warn("[BATT] exit critical_low without charge!\n");
			}

			/* A. To reduce the difference between UI & SOC
			 * while in low temperature condition & no drop_raw_level */
			if (htc_batt_info.rep.batt_temp < 0 &&
				drop_raw_level == 0 &&
				s_store_level >= 2) {
				dropping_level = htc_batt_info.prev.level - htc_batt_info.rep.level;
				if((dropping_level == 1) || (dropping_level == 0)) {
					s_store_level = s_store_level - (2 - dropping_level);
					htc_batt_info.rep.level = htc_batt_info.rep.level -
						(2 - dropping_level);
				}
				BATT_LOG("%s: remap: enter low temperature section, "
						"store_level:%d%%, dropping_level:%d%%, "
						"prev_level:%d%%, level:%d%%.\n", __func__
						, s_store_level, htc_batt_info.prev.level, dropping_level
						, htc_batt_info.rep.level);
			}

			/* B. To reduce the difference between UI & SOC
			 * while UI level <= 10%, reduce UI 2% maximum */
			if (s_store_level >= 2 && htc_batt_info.prev.level <= 10) {
				dropping_level = htc_batt_info.prev.level - htc_batt_info.rep.level;
				if((dropping_level == 1) || (dropping_level == 0)) {
					s_store_level = s_store_level - (2 - dropping_level);
					htc_batt_info.rep.level = htc_batt_info.rep.level -
						(2 - dropping_level);
				}
				BATT_LOG("%s: remap: UI level <= 10%% "
						"and allow drop 2%% maximum, "
						"store_level:%d%%, dropping_level:%d%%, "
						"prev_level:%d%%, level:%d%%.\n", __func__
						, s_store_level, dropping_level, htc_batt_info.prev.level
						, htc_batt_info.rep.level);
			}
		}
		/* always report 1% before report 0% in discharging stage
		    for entering quick boot off first rather than real off */
		if ((htc_batt_info.rep.level == 0) && (htc_batt_info.prev.level > 2)) {
			htc_batt_info.rep.level = 1;
			BATT_LOG("%s: battery level forcely report %d%%"
					" since prev_level=%d%%\n", __func__,
					htc_batt_info.rep.level, htc_batt_info.prev.level);
		}
	} else {
		/* battery charging - level smoothen algorithm:
		 * IF batt is not full THEN
		 *		- restrict level less then 100
		 * ELSE
		 *		- set level = 100
		 */

		if (htc_batt_info.rep.is_full) {
			if (htc_batt_info.smooth_chg_full_delay_min
					&& htc_batt_info.prev.level < 100) {
				/* keep prev level while time interval is less than 180s */
				if (time_accumulated_level_change <
						(htc_batt_info.smooth_chg_full_delay_min
						* CHG_ONE_PERCENT_LIMIT_PERIOD_MS)) {
					htc_batt_info.rep.level = htc_batt_info.prev.level;
				} else {
					htc_batt_info.rep.level = htc_batt_info.prev.level + 1;
				}
			} else {
				htc_batt_info.rep.level = 100; /* update to 100% */
			}
		} else {
			if (htc_batt_info.prev.level > htc_batt_info.rep.level) {
				/* Keep pre_level because overloading case didn't happen */
				if (!htc_batt_info.rep.overload) {
					BATT_LOG("%s: pre_level=%d, new_level=%d, "
						"level drop but overloading doesn't happen!\n",
					__func__ , htc_batt_info.prev.level, htc_batt_info.rep.level);
					htc_batt_info.rep.level = htc_batt_info.prev.level;
				}
			}
			else if (99 < htc_batt_info.rep.level && htc_batt_info.prev.level == 99)
				htc_batt_info.rep.level = 99; /* restrict at 99% */
			else if (htc_batt_info.prev.level < htc_batt_info.rep.level) {
				if(time_accumulated_level_change >
						QUICK_CHG_ONE_PERCENT_LIMIT_PERIOD_MS) {
					/* Let UI level increase at most 2% per minute, but
					     avoid level directly jumping to 100% from 98%      */
					if ((htc_batt_info.rep.level > (htc_batt_info.prev.level + 1))
							&& htc_batt_info.prev.level < 98)
						htc_batt_info.rep.level = htc_batt_info.prev.level + 2;
					else
						htc_batt_info.rep.level = htc_batt_info.prev.level + 1;
				} else
					htc_batt_info.rep.level = htc_batt_info.prev.level;

				if (htc_batt_info.rep.level > 100)
					htc_batt_info.rep.level = 100;
			} else {
				BATT_DEBUG("%s: pre_level=%d, new_level=%d, "
					"level would use raw level!\n", __func__,
					htc_batt_info.prev.level, htc_batt_info.rep.level);
			}
			/* WA: avoid battery level > limited level in charging case. */
			if (0 < htc_batt_info.rep.full_level &&
					htc_batt_info.rep.full_level < 100) {
				if((htc_batt_info.rep.level >
						htc_batt_info.rep.full_level) &&
						(htc_batt_info.rep.level <
						(htc_batt_info.rep.full_level + DEMO_GAP_WA))) {
					BATT_LOG("%s: block current_level=%d at "
							"full_level_dis_batt_chg=%d\n",
							__func__, htc_batt_info.rep.level,
							htc_batt_info.rep.full_level);
					htc_batt_info.rep.level =
								htc_batt_info.rep.full_level;
				}
			}
		}

		s_critical_low_enter = 0;
		s_allow_drop_one_percent_flag = false;
	}

	/* store_level updates everytime in the end of battery level adjust */
	s_store_level = htc_batt_info.rep.level - htc_batt_info.rep.level_raw;

	/* Do not power off when battery voltage is higher */
	if (htc_batt_info.rep.level == 0
			&& htc_batt_info.rep.batt_vol > htc_batt_info.allow_power_off_voltage
			&& htc_batt_info.rep.batt_temp > 0) {
		BATT_LOG("Not reach shutdown voltage, vol:%d\n", htc_batt_info.rep.batt_vol);
		htc_batt_info.rep.level = 1;
		s_store_level = 1;
	}

	/* Error handle for minus level */
	if ((int)htc_batt_info.rep.level < 0 ) {
		BATT_LOG("Adjust error level, level:%d\n", htc_batt_info.rep.level);
		htc_batt_info.rep.level = 1;
		s_store_level = 0;
	}

	if ((htc_batt_info.rep.batt_temp < 0) &&
			(htc_batt_info.rep.level_raw == 0)){
		BATT_LOG("Vbat=%d, level=%d\n", htc_batt_info.rep.batt_vol, htc_batt_info.rep.level);
		if ((htc_batt_info.rep.batt_vol < 3900)){
			BATT_LOG("Force report 0 for temp < 0 and raw = 0\n");
			htc_batt_info.rep.level = 0;
		} else if (htc_batt_info.rep.level == 0) {
			BATT_LOG("Force report 1 for temp < 0 and raw = 0 but vbat > 3.9V\n");
			htc_batt_info.rep.level = 1;
		}
	}

	g_is_consistent_level_ready = true;

	if (htc_batt_info.rep.level != htc_batt_info.prev.level){
		time_accumulated_level_change = 0;
		gs_update_PSY = true;
	}
	htc_batt_info.prev.level = htc_batt_info.rep.level;
	htc_batt_info.prev.level_raw = htc_batt_info.rep.level_raw;

	s_first = 0;
}

static void batt_update_info_from_charger(void)
{
	u8 chg_sts = 0;

	htc_batt_info.prev.batt_temp = htc_batt_info.rep.batt_temp;

	htc_batt_info.rep.batt_current =
		get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_CURRENT_NOW);

	htc_batt_info.rep.batt_vol =
		get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;

	htc_batt_info.rep.batt_temp =
		get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_TEMP);

	htc_batt_info.vbus =
		get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	// TODO: add eoc detection

	htc_batt_info.icharger->register_read(BATTERY_CHARGER_STATUS_1_REG, &chg_sts);
	if ((chg_sts & BATTERY_CHARGER_STATUS_MASK) == TERMINATE_CHARGE)
		g_batt_full_eoc_stop = true;
	else
		g_batt_full_eoc_stop = false;

}

static void batt_update_info_from_gauge(void)
{
	htc_batt_info.rep.level = htc_batt_info.rep.level_raw;
}

static void calculate_batt_cycle_info(unsigned long time_since_last_update_ms)
{
	time_t t = g_batt_first_use_time;
	struct tm timeinfo;
	const unsigned long timestamp_commit = 1483228800;	// timestamp of 2017/1/1
	struct timeval rtc_now;

    do_gettimeofday(&rtc_now);
	/* level_raw change times */
	if ( htc_batt_info.prev.charging_source && ( htc_batt_info.rep.level_raw > htc_batt_info.prev.level_raw ) )
		g_total_level_raw = g_total_level_raw + ( htc_batt_info.rep.level_raw - htc_batt_info.prev.level_raw );
	/* battery overheat time */
	if ( htc_batt_info.prev.batt_temp >= 550 )
		g_overheat_55_sec += time_since_last_update_ms/1000;
	/* battery first use time */
	if (((g_batt_first_use_time < timestamp_commit) || (g_batt_first_use_time > rtc_now.tv_sec))
									&& ( timestamp_commit < rtc_now.tv_sec )) {
		g_batt_first_use_time = rtc_now.tv_sec;
		BATT_LOG("%s: g_batt_first_use_time modify!\n", __func__);
	}
	/* calculate checksum */
	g_batt_cycle_checksum = g_batt_first_use_time + g_total_level_raw + g_overheat_55_sec;

	t = g_batt_first_use_time;
	time_to_tm(t, 0, &timeinfo);
	BATT_LOG("%s: g_batt_first_use_time = %04ld-%02d-%02d %02d:%02d:%02d, g_overheat_55_sec = %u, g_total_level_raw = %u\n",
		__func__, timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday,
		timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
		g_overheat_55_sec, g_total_level_raw);
}

static void read_batt_cycle_info(void)
{
	struct timeval rtc_now;
	unsigned int checksum_now;

	do_gettimeofday(&rtc_now);

	BATT_LOG("%s: (read from devtree) g_batt_first_use_time = %u, g_overheat_55_sec = %u, g_total_level_raw = %u, g_batt_cycle_checksum = %u\n",
                        __func__, g_batt_first_use_time, g_overheat_55_sec, g_total_level_raw, g_batt_cycle_checksum);

	checksum_now = g_batt_first_use_time + g_total_level_raw + g_overheat_55_sec;

        if ((checksum_now != g_batt_cycle_checksum) || (g_batt_cycle_checksum == 0)) {
                BATT_LOG("%s: Checksum error: reset battery cycle data.\n", __func__);
                g_batt_first_use_time = rtc_now.tv_sec;
                g_overheat_55_sec = 0;
                g_total_level_raw = 0;
                BATT_LOG("%s: g_batt_first_use_time = %u, g_overheat_55_sec = %u, g_total_level_raw = %u\n",
                        __func__, g_batt_first_use_time, g_overheat_55_sec, g_total_level_raw);
        }
}

void update_htc_extension_state(void)
{
	if (((g_batt_full_eoc_stop != 0) &&
		(htc_batt_info.rep.level == 100) &&
		(htc_batt_info.rep.level_raw == 100)))
		htc_batt_info.htc_extension |= HTC_EXT_CHG_FULL_EOC_STOP;
	else
		htc_batt_info.htc_extension &= ~HTC_EXT_CHG_FULL_EOC_STOP;

	if (g_is_unknown_charger)
		htc_batt_info.htc_extension |= HTC_EXT_UNKNOWN_USB_CHARGER;
	else
		htc_batt_info.htc_extension &= ~HTC_EXT_UNKNOWN_USB_CHARGER;

	if (g_is_quick_charger)
		htc_batt_info.htc_extension |= HTC_EXT_QUICK_CHARGER_USED;
	else
		htc_batt_info.htc_extension &= ~HTC_EXT_QUICK_CHARGER_USED;

	if (g_htc_usb_overheat)
		htc_batt_info.htc_extension |= HTC_EXT_USB_OVERHEAT;
	else
		htc_batt_info.htc_extension &= ~HTC_EXT_USB_OVERHEAT;

        if (gbAICharging)
                htc_batt_info.htc_extension |= HTC_EXT_AI_CHARGING;
        else
                htc_batt_info.htc_extension &= ~HTC_EXT_AI_CHARGING;
}

void update_htc_chg_src(void)
{
/* In bootable/offmode_charging/offmode_charging.c
 * The charging_source type is set as below,
        CHARGER_BATTERY = 0,
        CHARGER_USB,
        CHARGER_AC,
        CHARGER_9VAC,
        CHARGER_WIRELESS,
        CHARGER_MHL_AC,
        CHARGER_DETECTING,
        CHARGER_UNKNOWN_USB,
        CHARGER_PQM_FASTCHARGE,

*/
	int chg_src = 0;
	switch (htc_batt_info.rep.charging_source) {
		case POWER_SUPPLY_TYPE_UNKNOWN:
		case POWER_SUPPLY_TYPE_BATTERY:
			chg_src = 0;
			break;
		case POWER_SUPPLY_TYPE_USB:
		case POWER_SUPPLY_TYPE_USB_CDP:
			if (g_is_unknown_charger)
				chg_src = 7; /* UNKNOWN */
			else
				chg_src = 1; /* USB */
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
		case POWER_SUPPLY_TYPE_USB_ACA:
		case POWER_SUPPLY_TYPE_USB_HVDCP:
		case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		case POWER_SUPPLY_TYPE_USB_PD:
		case POWER_SUPPLY_TYPE_TYPEC:
			chg_src = 2; /* AC */
			break;
		case POWER_SUPPLY_TYPE_WIRELESS:
			chg_src = 4; /* WIRELESS */
			break;
		default:
			chg_src = 9;
			break;
        }
	htc_batt_info.rep.chg_src = chg_src;
}

#if defined(CONFIG_FB)
/* for htc_batt_info.state */
#define STATE_SCREEN_OFF		(1)
#endif /* CONFIG_FB */

#define ATS_IBAT_LIMIT_MA		1000
#define SCREEN_ON_LIMIT_MA		1000
#define IBAT_LIMITED_MA			300
#define IBAT_MAX			2600
#define MAX_IDX				4
#define HEALTH_LEVELS			6
static int ibat_map_5v[MAX_IDX][HEALTH_LEVELS] = {
/* IBAT_IDX =  < VOLTAGE-LIMITED, DISPLAY-ON > */
/*  #0 (00) */ { 1300,  1300,  IBAT_MAX, IBAT_MAX,  1800,  1300},
/*  #1 (01) */ { 1300,  1300,  IBAT_MAX, IBAT_MAX,  1800,  1300},
/*  #2 (10) */ {  250,   500,      1800,     1800,  1800,     0},
/*  #3 (11) */ {  250,   500,      1800,     1800,  1800,     0},
};

static int ibat_map_qc2[MAX_IDX][HEALTH_LEVELS] = {
/* IBAT_IDX =  < VOLTAGE-LIMITED, DISPLAY-ON > */
/*  #0 (00) */ { 1300,  1300,  IBAT_MAX, IBAT_MAX,  1800,  1300},
/*  #1 (01) */ { 1300,  1300,  IBAT_MAX, IBAT_MAX,  1800,  1300},
/*  #2 (10) */ {  250,   500,      1800,     1800,  1800,     0},
/*  #3 (11) */ {  250,   500,      1800,     1800,  1800,     0},
};

static int ibat_map_qc3[MAX_IDX][HEALTH_LEVELS] = {
/* IBAT_IDX =  < VOLTAGE-LIMITED, DISPLAY-ON > */
/*  #0 (00) */ { 1300,  1300,  IBAT_MAX, IBAT_MAX,  1800,  1300},
/*  #1 (01) */ { 1300,  1300,  IBAT_MAX, IBAT_MAX,  1800,  1300},
/*  #2 (10) */ {  250,   500,      1800,     1800,  1800,     0},
/*  #3 (11) */ {  250,   500,      1800,     1800,  1800,     0},
};

static struct htc_thermal_stage thermal_stage[HEALTH_LEVELS] =
{
	{	-INT_MAX,	 70},
	{	 50,		170},
	{	150,		220},
	{	420,		200},
	{	480,		390},
	{	INT_MAX,	460}
};

static int thermal_limit_vol[HEALTH_LEVELS] = {
	4100, 4100, 4300, 4300, 4300, 4100 };

static int thermal_limit_vol_recover[HEALTH_LEVELS] = {
	3900, 3900, 4100, 4100, 4100, 4000 };

enum {
	HEALTH_COOL3 = 0,
	HEALTH_COOL2,
	HEALTH_COOL1,
	HEALTH_GOOD,
	HEALTH_WARM1,
	HEALTH_WARM2,
};

int update_ibat_setting (void)
{
	static int batt_thermal = HEALTH_GOOD;
	static bool is_vol_limited = false;
	int idx = 0;
	bool is_screen_on = true;
	int batt_temp = htc_batt_info.rep.batt_temp;
	int batt_vol = htc_batt_info.rep.batt_vol;
	int chg_type = htc_batt_info.rep.charging_source;
	int ibat_ma = 0, ibat_max_ma = 0, batt_health_good = 0;
	int count = 0;

	ibat_max_ma = (htc_batt_info.fastchg_current_ma) ? htc_batt_info.fastchg_current_ma : IBAT_MAX;
	batt_health_good = (htc_batt_info.batt_health_good) ? htc_batt_info.batt_health_good : HEALTH_GOOD;

	if (g_flag_keep_charge_on)
		return ibat_max_ma * 1000;

	/* Step#1: Update Health status*/
	while (1) {
		if (batt_thermal >= batt_health_good) {	// normal & warm
			if (batt_temp >= thermal_stage[batt_thermal].nextTemp)
				batt_thermal++;
			else if (batt_temp <= thermal_stage[batt_thermal].recoverTemp)
				batt_thermal--;
			else
				break;
		} else {				// cool
			if (batt_temp <= thermal_stage[batt_thermal].nextTemp)
				batt_thermal--;
			else if (batt_temp >= thermal_stage[batt_thermal].recoverTemp)
				batt_thermal++;
			else
				break;
		}
		count++;
		if (count > 10) {
			BATT_LOG("%s: timeout: battTemp=%d,battThermal=%d,nextTemp=%d,recoverTemp=%d\n",
				__func__, batt_temp, batt_thermal,
				thermal_stage[batt_thermal].nextTemp, thermal_stage[batt_thermal].recoverTemp);
			break;
		}
	}
	/* Step#2: Update Voltage status */
	if (is_vol_limited || batt_vol > thermal_limit_vol[batt_thermal])
		is_vol_limited = true;
	if (!is_vol_limited ||
	    batt_vol < thermal_limit_vol_recover[batt_thermal])
		is_vol_limited = false;

	/* Step#3: Apply Screen ON configuartion */
	is_screen_on = !(htc_batt_info.state & STATE_SCREEN_OFF);

	/* Step#4: Get mapping index */
	idx = (	(2 * (is_vol_limited ? 1 : 0))	+
		(1 * (is_screen_on   ? 1 : 0))	);

	switch (chg_type) {
		case POWER_SUPPLY_TYPE_USB_HVDCP_3:
			ibat_ma = ibat_map_qc3[idx][batt_thermal];
			break;
		case POWER_SUPPLY_TYPE_USB_HVDCP:
			ibat_ma = ibat_map_qc2[idx][batt_thermal];
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
		case POWER_SUPPLY_TYPE_USB_PD:
		case POWER_SUPPLY_TYPE_TYPEC:
			ibat_ma = ibat_map_5v[idx][batt_thermal];
			break;
		default:
			ibat_ma = ibat_max_ma;
			break;
	}

	/* Limit charging to 1A when screen on & normal mode */
	if (!(htc_batt_info.state & STATE_SCREEN_OFF) &&
	    (strcmp(htc_get_bootmode(), "") == 0) &&
	    g_is_screen_on_limit_ibat &&
	    screen_on_limit_chg && (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_TYPEC))
		ibat_ma = min(ibat_ma, SCREEN_ON_LIMIT_MA);

	/*  Limit charging when talking */
	if ((g_chg_limit_reason) && (ibat_ma > IBAT_LIMITED_MA)) {
		BATT_EMBEDDED("Limit ibat under %dmA, reason: %d",
				IBAT_LIMITED_MA, g_chg_limit_reason);
		ibat_ma = IBAT_LIMITED_MA;
	}

	/* For ATS limit charge */
	if ((g_flag_ats_limit_chg) && (ibat_ma > ATS_IBAT_LIMIT_MA)) {
		BATT_EMBEDDED("Limit ibat under %dmA", ATS_IBAT_LIMIT_MA);
		ibat_ma = ATS_IBAT_LIMIT_MA;
	}

	/* For AI Charging */
	if((htc_batt_info.rep.level_raw >= 35) && htc_batt_info.ai_charge_enable){
		if(gbAICharging) {
			if (get_radio_flag() & BIT(3)) { /* Only enable under Radio flag = 8 */
				BATT_EMBEDDED("[AIC]AIC limit current.");
				ibat_ma = ibat_ma > AIC_Limit? AIC_Limit : ibat_ma;
			}
		}
	}

	/* For thermal limit charge */
	if ((g_thermal_limit_ma > 0) && (ibat_ma > g_thermal_limit_ma)) {
		BATT_EMBEDDED("Thermal limit ibat under %dmA", g_thermal_limit_ma);
		ibat_ma = g_thermal_limit_ma;
	}

        BATT_LOG("%s: batt_thermal=%d,is_vol_limited(>%dmV)=%d,is_screen_on=%d,idx=%d,ibat_ma=%d\n",
                        __func__, batt_thermal, thermal_limit_vol[batt_thermal],
                        is_vol_limited, is_screen_on, idx, ibat_ma);

	return ibat_ma*1000;
}

int htc_batt_schedule_batt_info_update(void)
{
        if (!g_htc_battery_probe_done)
                return 1;

        if (!work_pending(&htc_batt_timer.batt_work)) {
                wake_lock(&htc_batt_timer.battery_lock);
                queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
        }
        return 0;
}

#define USB_OVERHEAT_CHECK_PERIOD_MS 30000
static void is_usb_overheat_worker(struct work_struct *work)
{
        int usb_conn_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_USB_CONN_TEMP);
        static int last_usb_conn_temp = 0;
        if (g_latest_chg_src > POWER_SUPPLY_TYPE_UNKNOWN) {
                if(!g_usb_overheat){
                        if(usb_conn_temp > 650)
				g_usb_overheat = true;
                        if(g_usb_overheat_check_count == 0)
                                last_usb_conn_temp = usb_conn_temp;
                        else{
				if(((usb_conn_temp - last_usb_conn_temp) > 300) &&( g_usb_overheat_check_count <= 6))
					g_usb_overheat = true;
                        }
                }
                g_usb_overheat_check_count++;
                if(!g_usb_overheat)
                        schedule_delayed_work(&htc_batt_info.is_usb_overheat_work, msecs_to_jiffies(USB_OVERHEAT_CHECK_PERIOD_MS));
                else{
                        BATT_LOG("%s: USB overheat! cable_in_usb_temp:%d, usb_temp:%d, count = %d\n",
                                __func__, last_usb_conn_temp, usb_conn_temp, g_usb_overheat_check_count);
                        htc_batt_schedule_batt_info_update();
                }
        }else{
                g_usb_overheat = false;
                g_usb_overheat_check_count = 0;
                last_usb_conn_temp = 0;
        }
}

extern void htc_typec_enable(bool enable);

#define HTC_USB_OVERHEAT_POLLING_THRES 550
#define HTC_USB_OVERHEAT_POLLING_CONN_BATT_DELTA_TEMP_THRES 100
#define HTC_USB_OVERHEAT_POLLING_TIME_MS 5000
#define HTC_USB_OVERHEAT_TRIGGER_THRES 680
#define HTC_USB_OVERHEAT_TRIGGER_BATT_DELTA_TEMP_THRES 150
#define HTC_USB_OVERHEAT_RECOVER_THRES 600
static void htc_usb_overheat_routine(void)
{
	bool b_sched_quick_polling = false;
	unsigned int prev_state = g_htc_usb_overheat_check_state;
	int usb_conn_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_USB_CONN_TEMP);
	int batt_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_TEMP);
	int usb_conn_batt_delta_temp = usb_conn_temp - batt_temp;

	switch (g_htc_usb_overheat_check_state) {
		case USB_CONN_STAT_INIT:
			// start polling when temp reach 55 degree or delta temp between battery and connector reache 10 degree
			if ((usb_conn_temp >= HTC_USB_OVERHEAT_POLLING_THRES) ||
					(usb_conn_batt_delta_temp >= HTC_USB_OVERHEAT_POLLING_CONN_BATT_DELTA_TEMP_THRES)) {
				BATT_LOG("[USBOH] Trigger quick polling, usb_temp=%d, batt_tmp=%d\n", usb_conn_temp, batt_temp);
				b_sched_quick_polling = true;
				wake_lock(&htc_batt_info.check_overheat_lock);
				g_htc_usb_overheat_check_state = USB_CONN_STAT_POLLING;
			}
			break;

		case USB_CONN_STAT_POLLING:
			break;

		case USB_CONN_STAT_OVERHEAT:
			break;

		case USB_CONN_STAT_POLLING_END_CHK:
			// stop polling when temp < 55 degree and delta temp between battery and connector < 10 degree
			if ((usb_conn_temp >= HTC_USB_OVERHEAT_POLLING_THRES) ||
					(usb_conn_batt_delta_temp >= HTC_USB_OVERHEAT_POLLING_CONN_BATT_DELTA_TEMP_THRES)) {
				b_sched_quick_polling = true;
				g_htc_usb_overheat_check_state = USB_CONN_STAT_POLLING;
			} else {
				BATT_LOG("[USBOH] Quick polling was done! No overheat!\n");
				g_htc_usb_overheat_check_state = USB_CONN_STAT_INIT;
				wake_unlock(&htc_batt_info.check_overheat_lock);
			}
			break;

		default:
			panic("[BATT][USBOH]ERROR! SHOULD NOT GO HERE!\n");
			break;
	}

	if (b_sched_quick_polling) {
		if (!delayed_work_pending(&htc_batt_info.htc_usb_overheat_work)) {
			schedule_delayed_work(&htc_batt_info.htc_usb_overheat_work, 0);
		}
	}

	if ((prev_state != 0) || (g_htc_usb_overheat_check_state != 0))
		BATT_LOG("[USBOH] Prev state:%d, Curr state:%d, usb_temp=%d, batt_temp=%d\n",
			prev_state, g_htc_usb_overheat_check_state, usb_conn_temp, batt_temp);
}

static void htc_usb_overheat_worker(struct work_struct *work)
{
	int usb_conn_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_USB_CONN_TEMP);
	unsigned int prev_state = g_htc_usb_overheat_check_state;
	bool b_sched_next = false;
	static int s_polling_cnt = 0;
	int batt_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_TEMP);
	int usb_conn_batt_delta_temp = usb_conn_temp - batt_temp;

	switch (g_htc_usb_overheat_check_state) {
		case USB_CONN_STAT_POLLING:
			s_polling_cnt++;

			// trigger usb overheat when temp reach 68 degree or delta temp between battery and connector reache 15 degree
			if ((usb_conn_temp >= HTC_USB_OVERHEAT_TRIGGER_THRES) ||
					(usb_conn_batt_delta_temp >= HTC_USB_OVERHEAT_TRIGGER_BATT_DELTA_TEMP_THRES)){
				s_polling_cnt = 0;
				g_htc_usb_overheat = true;
				gs_update_PSY = true;
				BATT_LOG("[USBOH] USB OverHeat!!! usb_temp=%d, batt_temp=%d\n", usb_conn_temp, batt_temp);
				htc_typec_enable(false);
				htc_batt_schedule_batt_info_update();
				g_htc_usb_overheat_check_state = USB_CONN_STAT_OVERHEAT;
				b_sched_next = true;
			} else if (s_polling_cnt > 12) {
				// check temperature, if still >= 55 degree and delta temp > 10 degree, keep on polling else stop polling
				if ((usb_conn_temp >= HTC_USB_OVERHEAT_POLLING_THRES) ||
						(usb_conn_batt_delta_temp >= HTC_USB_OVERHEAT_POLLING_CONN_BATT_DELTA_TEMP_THRES)) {
					b_sched_next = true;
				} else {
					g_htc_usb_overheat_check_state = USB_CONN_STAT_POLLING_END_CHK;
				}
				s_polling_cnt = 0;
			} else {
				b_sched_next = true;
			}

			break;

		case USB_CONN_STAT_OVERHEAT:
			// stop alarm when temp < 60 degree or delta temp between battery and connector < 10 degree
			if ((usb_conn_temp < HTC_USB_OVERHEAT_RECOVER_THRES) &&
					(usb_conn_batt_delta_temp < HTC_USB_OVERHEAT_POLLING_CONN_BATT_DELTA_TEMP_THRES)){
				g_htc_usb_overheat = false;
				update_htc_extension_state();
				BATT_LOG("[USBOH] USB Cool Down!!! usb_temp=%d\n", usb_conn_temp);
				gs_update_PSY = true;
				htc_typec_enable(true);
				htc_batt_schedule_batt_info_update();
				g_htc_usb_overheat_check_state = USB_CONN_STAT_INIT;
				wake_unlock(&htc_batt_info.check_overheat_lock);
			} else {
				b_sched_next = true;
			}
			break;


		default:
			BATT_LOG("[USBOH] WARNING! SHOULD NOT GO HERE! State=%d\n", g_htc_usb_overheat_check_state);
			break;
	}

	if (b_sched_next)
		schedule_delayed_work(&htc_batt_info.htc_usb_overheat_work, msecs_to_jiffies(HTC_USB_OVERHEAT_POLLING_TIME_MS));

	BATT_LOG("[USBOH] Prev state:%d, Curr state:%d, conn_temp=%d, batt_temp=%d\n",
			prev_state, g_htc_usb_overheat_check_state, usb_conn_temp, batt_temp);
	return;
}

static void cable_impedance_worker(struct work_struct *work)
{
	int R_HW_MB_Impedance = 150;
	static int vbus_mv[3], iusb_ma[3], aicl_result;
	int i=0;
	int vbus[3], iusb[3];
	u8 stat;
	//int cnt = 0;
	if (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP){
		pr_err("[Cable impedance]Not correct charger source, ignore detection.!\n");
		return;
	}

	if (!g_need_to_check_impedance){
		pr_err("[Cable impedance]Charger ability enough, ignore detection.!\n");
		return;
	}

	//Check AICL done
	htc_batt_info.icharger->register_read(AICL_STATUS_REG, &stat);
	if (!(stat & AICL_DONE_BIT) && (gs_cable_impedance != 4)) {
		gs_cable_impedance = 4;
		pr_err("[Cable impedance]AICL(%dmA) is not ready, pending the detection 2 seconds!\n",
			get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000);
		schedule_delayed_work(&htc_batt_info.cable_impedance_work, msecs_to_jiffies(2000));
		return;
	}

	pr_err("[Cable impedance]Start to calculate cable impedance!\n");
	gs_cable_impedance = 4;

	aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
	gs_aicl_result = aicl_result;

	 /* if meet these condition, ignore cable impedance detection.
                1. aicl >= 1500
                2. USB_type != USB_DCP
                3. batt_health != POWER_SUPPLY_HEALTH_GOOD
                4. not in fastchg
        */
	if ((aicl_result >= 1500) || (htc_batt_info.rep.health != POWER_SUPPLY_HEALTH_GOOD) ||
		(get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CHARGE_TYPE) != POWER_SUPPLY_CHARGE_TYPE_FAST)){
		pr_err("[Cable impedance]Ignore the detecttion, aicl=%dmA\n", aicl_result);
		gs_cable_impedance = 1;
		return;
	}

	// Disable AICL
	htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, 0);
	htc_batt_info.icharger->register_read(USBIN_AICL_OPTIONS_CFG_REG, &stat);
	pr_err("[Cable impedance]AICL disable:0x%02x\n", stat);

	/* specific current values */
	// Set 500mA
	{
		set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, 500000);
		msleep(3000);
	}
	if (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP){
		pr_err("[Cable impedance]Not correct charger source, ignore detection.!\n");
		gs_cable_impedance = 1;
		goto endWorker;
	}

	for (i=0;i<3;i++) {
		iusb[i] = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000;
		vbus[i] = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	}
	vbus_mv[0] = (vbus[0] + vbus[1] + vbus[2])/3;
	iusb_ma[0] = (iusb[0] + iusb[1] + iusb[2])/3;
	pr_err("[Cable impedance] Iusb=500mA(%d),Vusb1-1=%d,Vusb1-2=%d,Vusb1-3=%d,Vusb1=%d\n",
			iusb_ma[0], vbus[0], vbus[1], vbus[2], vbus_mv[0]);

	// Set 400mA
	{
		set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, 400000);
		msleep(3000);
	}
	if (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP){
		pr_err("[Cable impedance]Not correct charger source, ignore detection.!\n");
		gs_cable_impedance = 1;
		goto endWorker;
	}

	for (i=0;i<3;i++) {
		iusb[i] = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000;
		vbus[i] = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	}
	vbus_mv[1] = (vbus[0] + vbus[1] + vbus[2])/3;
	iusb_ma[1] = (iusb[0] + iusb[1] + iusb[2])/3;
	pr_err("[Cable impedance] Iusb=400mA(%d),Vusb2-1=%d,Vusb2-2=%d,Vusb2-3=%d,Vusb2=%d\n",
			iusb_ma[1], vbus[0], vbus[1], vbus[2], vbus_mv[1]);

	// Set 300mA
	set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, 300000);
	msleep(3000);

	if (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP){
		pr_err("[Cable impedance]Not correct charger source, ignore detection.!\n");
		gs_cable_impedance = 1;
		goto endWorker;
	}

	for (i=0;i<3;i++) {
		iusb[i] = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000;
		vbus[i] = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	}
	vbus_mv[2] = (vbus[0] + vbus[1] + vbus[2])/3;
	iusb_ma[2] = (iusb[0] + iusb[1] + iusb[2])/3;
	pr_err("[Cable impedance] Iusb=300mA(%d),Vusb3-1=%d,Vusb3-2=%d,Vusb3-3=%d,Vusb3=%d\n",
			iusb_ma[2], vbus[0], vbus[1], vbus[2], vbus_mv[2]);

	pr_err("[Cable impedance] R1=%d,R2=%d,Vusb1=%d,Vusb2=%d,Vusb3=%d,AICL=%d\n",
		((vbus_mv[2]-vbus_mv[0])*1000/(iusb_ma[0]-iusb_ma[2]))-R_HW_MB_Impedance,
		((vbus_mv[2]-vbus_mv[1])*1000/(iusb_ma[1]-iusb_ma[2]))-R_HW_MB_Impedance,
		vbus_mv[0], vbus_mv[1], vbus_mv[2], aicl_result);

	gs_R_cable_impedance =
		( (((vbus_mv[2]-vbus_mv[0])*1000/(iusb_ma[0]-iusb_ma[2]))-R_HW_MB_Impedance)
		+ (((vbus_mv[2]-vbus_mv[1])*1000/(iusb_ma[1]-iusb_ma[2]))-R_HW_MB_Impedance) )/2;

	// bad cable=3, not good cable=2
	if (gs_R_cable_impedance >= 1000){
		gs_cable_impedance = 3;
		htc_batt_info.htc_extension |= HTC_EXT_BAD_CABLE_USED;
		BATT_LOG("This cable is bad to charge\n");
	}else{
		gs_cable_impedance = 2;
	}
	pr_err("[Cable impedance] cable_impedance: %d, R_cable_impedance: %d\n", gs_cable_impedance, gs_R_cable_impedance);

endWorker:
	gs_measure_cable_impedance = false;

	//impedance_set_iusb_max(1000000, false);
	set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, 1000000);

	// Enable AICL
	htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
	htc_batt_info.icharger->register_read(USBIN_AICL_OPTIONS_CFG_REG, &stat);
	pr_err("[Cable impedance]AICL Enable:0x%02x\n", stat);
	set_batt_psy_property(POWER_SUPPLY_PROP_RERUN_AICL, 1);

	pr_err("[Cable impedance]End.\n");
	htc_batt_schedule_batt_info_update();
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
        struct fb_event *evdata = data;
        int *blank;

        if (evdata && evdata->data && event == FB_EVENT_BLANK) {
                blank = evdata->data;
                switch (*blank) {
                        case FB_BLANK_UNBLANK:
                                htc_batt_info.state &= ~STATE_SCREEN_OFF;
                                BATT_LOG("%s-> display is On", __func__);
                                htc_batt_schedule_batt_info_update();
				set_aicdata(IS_SCREEN_ON);
                                break;
                        case FB_BLANK_POWERDOWN:
                        case FB_BLANK_HSYNC_SUSPEND:
                        case FB_BLANK_VSYNC_SUSPEND:
                        case FB_BLANK_NORMAL:
                                htc_batt_info.state |= STATE_SCREEN_OFF;
                                BATT_LOG("%s-> display is Off", __func__);
                                htc_batt_schedule_batt_info_update();
				set_aicdata(IS_SCREEN_OFF);
                                break;
                }
        }

        return 0;
}
#endif /* CONFIG_FB */

#define FG_BATT_INFO_IBATT_SENSING_CFG	0x4173
#define HTCCHG_VBUS_ADJUST_PERIOD_MS 	1000
#define HTCCHG_CHARGE_STATUS_PERIOD_MS	5000
#define HTCCHG_INIT_PERIOD_MS 		1000
#define VUSB_INIT_MV 		4600
#define VUSB_HIGH_BOUND_MV 	4600
#define VUSB_LOW_BOUND_MV 	4000
#define IBAT_HIGH_BOUND_MA 	2500
#define IBAT_LOWER_BOUND_MA 	1800
#define LEAVE_COOL_HIGH_TEMP	450
#define RE_ENTRY_COOL_HIGH_TEMP	430
#define ENTRY_COOL_TEMP_SRN_ON	390
#define ENTRY_COOL_TEMP		420
#define LEAVE_COOL_LOW_TEMP	360
#define LEAVE_COOL_VOLTAGE	4250
#define ENTRY_SOC_MAX		90
#define ENTRY_SOC_MIN		5
static void htcchg_cc_leave(void)
{
	htcchg_leave = true;
	pr_info("[CC] Leave Cool Charger.\n");

	// Reset screen on ibat limit
	g_is_screen_on_limit_ibat = false;
	if (htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		schedule_delayed_work(&htc_batt_info.screen_ibat_limit_enable_work, msecs_to_jiffies(SCREEN_LIMIT_IBAT_DELAY_MS));

	// Change IBAT SENSE
	htc_batt_info.icharger->register_write(FG_BATT_INFO_IBATT_SENSING_CFG, SOURCE_SELECT_MASK, SRC_SEL_BATFET);

	// Close Cool Charger path if leave HVDCP3
	set_batt_psy_property(POWER_SUPPLY_PROP_HTCCHG_GPIO_OPEN, 0);

	//Change ready to input to make sure the PMIC can get the cable out
	set_batt_psy_property(POWER_SUPPLY_PROP_HTCCHG_GPIO_OPEN, 2);

        htc_batt_info.icharger->register_write(TAPER_TIMER_SEL_CFG_REG, TYPEC_VBUS_1V_BIT, TYPEC_VBUS_1V_BIT);

        htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG , USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
        htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG , SUSPEND_ON_COLLAPSE_USBIN_BIT, SUSPEND_ON_COLLAPSE_USBIN_BIT);

	// 1. Turn on HVDCP
	htc_batt_info.icharger->register_write(USBIN_OPTIONS_1_CFG_REG, HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT, HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT);
	//Rerun APSD
	htc_batt_info.icharger->register_write(CMD_APSD_REG, APSD_RERUN_BIT, APSD_RERUN_BIT);

	htcchg_leave = false;
	first_cool_charger = true;
	htcchg_on = false;
}

static void htcchg_cc_entry(void)
{
	htcchg_on = true;
        pr_info("[CC] Entry Cool Charger.\n");

	htc_batt_info.icharger->register_write(TAPER_TIMER_SEL_CFG_REG, TYPEC_VBUS_1V_BIT, 0);

	htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG , USBIN_AICL_EN_BIT, 0);
	htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG , SUSPEND_ON_COLLAPSE_USBIN_BIT, 0);

	// 1. Turn of HVDCP daemon for preventing Vbus change.
	htc_batt_info.icharger->register_write(USBIN_OPTIONS_1_CFG_REG,	HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT, 0);
	// 2. Set IUSB as 100mA
	set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, HTCCHG_INIT_CURR);

	pr_info("[CC] STEP3 : ==========Create init queue==========\n");
	schedule_delayed_work(&htc_batt_info.htcchg_init_work,	msecs_to_jiffies(HTCCHG_INIT_PERIOD_MS));
}

static void htcchg_check_cc_status(void)
{
	static bool re_entry = false;
        bool is_screen_on;
	unsigned int entry_cool_temp = 0;

	if(strcmp(htc_get_bootmode(), "") != 0)
		 return;

	if((is_cool_charger() == false)||(not_entry_again)) return;

	if(htcchg_on){
		if(htcchg_leave) return;

                if(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_HVDCP_3){
			pr_info("[CC] ==Not QC3=\n");
			re_entry = false;
			htcchg_cc_leave();
		}else if(htc_batt_info.rep.batt_temp <= LEAVE_COOL_LOW_TEMP){
			pr_info("[CC] Leave temp: %d [%d]\n", htc_batt_info.rep.batt_temp, LEAVE_COOL_LOW_TEMP);
			re_entry = false;
                        htcchg_cc_leave();
                }else if(htc_batt_info.rep.batt_temp >= LEAVE_COOL_HIGH_TEMP){
                        pr_info("[CC] Leave temp: %d [%d]\n", htc_batt_info.rep.batt_temp, LEAVE_COOL_LOW_TEMP);
			re_entry = true;
                        htcchg_cc_leave();
		}else if(htc_batt_info.rep.batt_vol >= LEAVE_COOL_VOLTAGE){
                        pr_info("[CC] Leave voltage: %d [%d]\n", htc_batt_info.rep.batt_vol, LEAVE_COOL_VOLTAGE);
			not_entry_again = true;
			htcchg_cc_leave();
		}
	}else{
		if((htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_HVDCP_3)&&(g_is_quick_charger)){
			if((htc_batt_info.rep.level >= ENTRY_SOC_MIN)&&(htc_batt_info.rep.level <= ENTRY_SOC_MAX)){
			        is_screen_on = !(htc_batt_info.state & STATE_SCREEN_OFF);
				entry_cool_temp = is_screen_on? ENTRY_COOL_TEMP_SRN_ON : ENTRY_COOL_TEMP;
				if((htc_batt_info.rep.batt_temp > entry_cool_temp)&&(htc_batt_info.rep.batt_temp < LEAVE_COOL_HIGH_TEMP)){
					if(htc_batt_info.rep.batt_temp < RE_ENTRY_COOL_HIGH_TEMP){
						re_entry = false;
						htcchg_cc_entry();
					}else{
						if(re_entry == false)
							htcchg_cc_entry();
					}
				}
			}
		}
	}
}

static void htcchg_vbus_adjust_worker(struct work_struct *work)
{
	static int vbus_cnt = 0, ibat_cnt = 0;
	int ibat_for_cc_check = get_property(htc_batt_info.batt_psy,POWER_SUPPLY_PROP_HTC_CHARGE_NOW);
        int vbus_for_cc_check = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;

	if(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_HVDCP_3){
		pr_info("[CC] Not QC3, ignore...\n");
		return;
	}

	if(htcchg_on == false) return;

	if(vbus_cnt >= 5){
		//leave cool charger
		pr_info("[CC] VBUS cannot reach, leave cool charger!!!, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
		htcchg_cc_leave();
		ibat_cnt = 0;
		vbus_cnt = 0;
		return;
	}

	if(ibat_cnt >= 5){
                //leave cool charger
                pr_info("[CC] IBAT  cannot reach, leave cool charger!!!, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
		htcchg_cc_leave();
		ibat_cnt = 0;
		vbus_cnt = 0;
                return;
	}

	if (vbus_for_cc_check >= VUSB_LOW_BOUND_MV && vbus_for_cc_check <= VUSB_HIGH_BOUND_MV) {
		vbus_cnt = 0;
		if (ibat_for_cc_check > IBAT_HIGH_BOUND_MA) {
			pr_info("[CC][D-] IBAT is high, down it, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
			ibat_cnt++;
			htc_batt_info.icharger->register_write(CMD_HVDCP_2_REG,
				SINGLE_DECREMENT_BIT, SINGLE_DECREMENT_BIT);
			schedule_delayed_work(&htc_batt_info.htcchg_vbus_adjust_work,
				msecs_to_jiffies(HTCCHG_VBUS_ADJUST_PERIOD_MS));

		} else if (ibat_for_cc_check < IBAT_LOWER_BOUND_MA) {
			pr_info("[CC][D+] IBAT is low, pull it, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
			ibat_cnt++;
			htc_batt_info.icharger->register_write(CMD_HVDCP_2_REG,
				SINGLE_INCREMENT_BIT, SINGLE_INCREMENT_BIT);
			schedule_delayed_work(&htc_batt_info.htcchg_vbus_adjust_work,
				msecs_to_jiffies(HTCCHG_VBUS_ADJUST_PERIOD_MS));
		} else {
			pr_info("[CC] IBAT reach, delay 5 sec.!!!, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
			ibat_cnt = 0;
			vbus_cnt = 0;
			not_entry_again = false;
			// Do nothing, ibat reach
                        schedule_delayed_work(&htc_batt_info.htcchg_vbus_adjust_work,
                                msecs_to_jiffies(HTCCHG_CHARGE_STATUS_PERIOD_MS));
		}
	}else{
		if (vbus_for_cc_check > VUSB_HIGH_BOUND_MV){
                        pr_info("[CC][D-] VUSB is high, down it, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
			vbus_cnt++;
                        htc_batt_info.icharger->register_write(CMD_HVDCP_2_REG,
                                SINGLE_DECREMENT_BIT, SINGLE_DECREMENT_BIT);
                        schedule_delayed_work(&htc_batt_info.htcchg_vbus_adjust_work,
                                msecs_to_jiffies(HTCCHG_VBUS_ADJUST_PERIOD_MS));
		}else{
                        pr_info("[CC][D+] VBUS is low, pull it, Vbus : %d, ISEN : %d\n", vbus_for_cc_check, ibat_for_cc_check);
			vbus_cnt++;
                        htc_batt_info.icharger->register_write(CMD_HVDCP_2_REG,
                                SINGLE_INCREMENT_BIT, SINGLE_INCREMENT_BIT);
                        schedule_delayed_work(&htc_batt_info.htcchg_vbus_adjust_work,
                                msecs_to_jiffies(HTCCHG_VBUS_ADJUST_PERIOD_MS));
		}
	}
        if(first_cool_charger){
        //Change ready to input to make sure the PMIC can get the cable out
                set_batt_psy_property(POWER_SUPPLY_PROP_HTCCHG_GPIO_OPEN, 2);
		not_entry_again = true;
	}
	first_cool_charger = false;
}

#define HTCCHG_INIT_PERIOD_MS 1000
static void htcchg_init_worker(struct work_struct *work)
{
	int vbus_for_cc_init = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	static int init_cnt = 0;

        if(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_HVDCP_3){
                pr_info("[CC] Not QC3, ignore init...\n");
		init_cnt = 0;
                return;
        }

	if (vbus_for_cc_init > VUSB_INIT_MV) {
		pr_info("[CC][init] VBus > %d mV, down it. Vbus : %d\n", VUSB_INIT_MV, vbus_for_cc_init);
		// Pulse D-
		htc_batt_info.icharger->register_write(CMD_HVDCP_2_REG,
                                        SINGLE_DECREMENT_BIT, SINGLE_DECREMENT_BIT);
		init_cnt++;
		if  (init_cnt > 30) {
			not_entry_again = true;
			init_cnt = 0;
			pr_info("[CC][init] VBus cannot adjust!\n");
			htcchg_cc_leave();
		} else
			schedule_delayed_work(&htc_batt_info.htcchg_init_work,
				msecs_to_jiffies(HTCCHG_INIT_PERIOD_MS));
	} else {
		init_cnt = 0;
		pr_info("[CC][init] IBAT <= %d mV !!! Vbus : %d\n",VUSB_INIT_MV,  vbus_for_cc_init);
		// Open Cool Charger Path
		set_batt_psy_property(POWER_SUPPLY_PROP_HTCCHG_GPIO_OPEN, 1);
		// Change IBAT SENSE
		htc_batt_info.icharger->register_write(FG_BATT_INFO_IBATT_SENSING_CFG, SOURCE_SELECT_MASK, SRC_SEL_SENSERESISTOR);

		// Lauch Vbus_adjust_worker to modify vbus until IBAT goal reach
		pr_info("[CC] ========== Entry done, keep on the Vbus and IBAT checking==========\n");
		schedule_delayed_work(&htc_batt_info.htcchg_vbus_adjust_work, msecs_to_jiffies(HTCCHG_VBUS_ADJUST_PERIOD_MS));
	}
}

#define WA_5V_2A_DETECT_DELAY_MS		1000
#define ICL_ABILITY_CHECK_DELAY_MS	5000
#define WA_5V_2A_VBUS_THRES			4825000
#define R_DEFAULT					300
void htc_5v2a_pre_chk(void)
{
	int idx = 0;
	int chg_type;
	const unsigned int iusb_ma[] = {1000, 1100, 1200, 1400, 1500};
	int vbus[5], vbus_tmp1, vbus_tmp2, vbus_tmp3, vbus_cal;
	int r1, r2, r3, r4, r_avg;

	if(!g_htc_battery_probe_done) {
		BATT_LOG("%s: called before driver ready\n", __func__);
		return;
	}

	if (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP) {
		BATT_LOG("%s: Charger tyep not dcp, %s\n", __func__, cg_chr_src[htc_batt_info.rep.charging_source]);
		return;
	}

	g_need_to_check_impedance = false;

	// Disable HW AICL
	charger_register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, 0);

	// Get the vbus
	for (idx = 0; idx < 5; idx++) {
		// Set iusb
		chg_type = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_TYPE);
		if (chg_type == POWER_SUPPLY_TYPE_USB_DCP) {
			BATT_LOG("%s: set charging_current(%d)\n", __func__, iusb_ma[idx]);
			set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, (iusb_ma[idx] * 1000));
		} else {
			BATT_LOG("%s: TYPE CHANGED -> %d\n", __func__, chg_type);
			charger_register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
			return;
		}

		msleep(WA_5V_2A_DETECT_DELAY_MS);

		// Check charger type
		chg_type = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_TYPE);
		if (chg_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			// Re-Enable AICL
			charger_register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
			set_batt_psy_property(POWER_SUPPLY_PROP_RERUN_AICL, 1);
			return;
		}

		vbus_tmp1 = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		msleep(10);
		vbus_tmp2= get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		msleep(10);
		vbus_tmp3 = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		vbus[idx] = (vbus_tmp1 + vbus_tmp2 + vbus_tmp3)/3;

		BATT_LOG("%s: IUSB_MAX=%dmA,vbus1=%d,vbus2=%d,vbus3=%d\n", __func__, iusb_ma[idx], vbus_tmp1, vbus_tmp2, vbus_tmp3);
	}

	// Calculate cable resistence
	r1 = (vbus[0] - vbus[1]) / (iusb_ma[1] - iusb_ma[0]);
	r2 = (vbus[1] - vbus[2]) / (iusb_ma[2] - iusb_ma[1]);
	r3 = (vbus[2] - vbus[3]) / (iusb_ma[3] - iusb_ma[2]);
	r4 = (vbus[3] - vbus[4]) / (iusb_ma[4] - iusb_ma[3]);
	r_avg = (r1 + r2 + r3 + r4) / 4;

	if (r_avg < htc_batt_info.r_default_for_5v2a_pre_chk)
		r_avg = htc_batt_info.r_default_for_5v2a_pre_chk;

	// calculate vbus when IUSB=1.5A & use R_DEFAULT mohm cable to charge.
	vbus_cal = ((r_avg-htc_batt_info.r_default_for_5v2a_pre_chk)*1500) + vbus[4];


	BATT_LOG("%s: vbus_cal=%d,vbus_1A=%d,vbus_1.1A=%d,vbus_1.2A=%d,vbus_1.4A=%d,vbus_1.5A=%d\n",
			__func__, vbus_cal, vbus[0], vbus[1], vbus[2], vbus[3], vbus[4]);

	BATT_LOG("%s: r1=%d,r2=%d,r3=%d,r4=%d,r_avg=%d\n", __func__, r1, r2, r3, r4, r_avg);

	if (vbus_cal < WA_5V_2A_VBUS_THRES) {
		// Re-Enable AICL & Set Current Back
		charger_register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
		set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, WALL_CHARGE_CURR);
		set_batt_psy_property(POWER_SUPPLY_PROP_RERUN_AICL, 1);
		BATT_LOG("%s: Low vbus detected, keep 1.5A\n", __func__);
	} else {
		BATT_LOG("%s: Start 5v/2A\n", __func__);
		// Re-Enable AICL & Set Current 2A
		charger_register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
		set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, DCP_5V2A_CHARGE_CURR);
		set_batt_psy_property(POWER_SUPPLY_PROP_RERUN_AICL, 1);
		schedule_delayed_work(&htc_batt_info.iusb_5v_2a_ability_check_work,
				msecs_to_jiffies(ICL_ABILITY_CHECK_DELAY_MS));
	}

	return;
}

#define ICL_5V2A_THRESHOLD_MA	1900
static void htc_iusb_5v_2a_ability_check_work(struct work_struct *work)
{
	int current_aicl = 0, vbus = 0;

	if(!g_htc_battery_probe_done) {
		BATT_ERR("%s, called before driver ready\n", __func__);
		return;
	} else {
		current_aicl = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED) / 1000;
		vbus = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000;
	}

	BATT_LOG("current_aicl=%dmA, vbus=%duV\n", current_aicl, vbus);

	if (current_aicl > ICL_5V2A_THRESHOLD_MA) {
		BATT_LOG("5V/2A DETECTED\n");
	} else {
		BATT_LOG("5V/2A FAILED DOWNGRADE TO 1.5A\n");
		set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, WALL_CHARGE_CURR);
	}
}

static void htc_quick_charger_check_work(struct work_struct *work)
{
	int 	chg_type = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_TYPE);
	BATT_LOG("%s: charger type: %d", __func__, chg_type);

	if ((chg_type == POWER_SUPPLY_TYPE_USB_HVDCP) ||
			(chg_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) ||
			(chg_type == POWER_SUPPLY_TYPE_USB_PD)) {
		g_is_quick_charger = true;
		gs_update_PSY = true;
		htc_batt_schedule_batt_info_update();
	} else
		g_is_quick_charger = false;
}

static void screen_ibat_limit_enable_worker(struct work_struct *work)
{
	g_is_screen_on_limit_ibat = true;
	htc_batt_schedule_batt_info_update();
}

#define VFLOAT_COMP_WARM		0x10	// 0x10F4[5:0] - 0x10 = 4.1V
#define VFLOAT_COMP_COOL		0x0		// 0x10F4[5:0] - 0x0 = 4.4V
#define VFLOAT_COMP_NORMAL	0x0		// 0x10F4[5:0] - 0x0 = 4.4V
#define CHG_UNKNOWN_CHG_PERIOD_MS	9000
#define VBUS_VALID_THRESHOLD			4250000
#define THERMAL_BATT_TEMP_UPDATE_TIME_THRES	1800	//MIN 30min
#define THERMAL_BATT_TEMP_UPDATE_TIME_MAX	3610    //MAX 60min+10s tolarance
#define CC_DET_DEFAULT	1
#define BI_BATT_CHGE_UPDATE_TIME_THRES		1800	//30mins
#define BI_BATT_CHGE_CHECK_TIME_THRES		36000	//10HR
static void batt_worker(struct work_struct *work)
{
	static int s_first = 1;
	static int s_prev_pwrsrc_enabled = 1;
	static int s_prev_user_set_chg_curr = 0;
	int pwrsrc_enabled = s_prev_pwrsrc_enabled;
	int charging_enabled = gs_prev_charging_enabled;
	int user_set_chg_curr = s_prev_user_set_chg_curr;
	int src = 0;
	int ibat = 0;
	int ibat_setting = 0;
	int ibat_new = 0;
	int max_iusb = 0;
	unsigned long time_since_last_update_ms;
	unsigned long cur_jiffies;
	int cc_type = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	unsigned int aicl = 0;

	static struct timeval s_thermal_batt_update_time = { 0, 0 };
	struct timeval rtc_now;
	static bool batt_chgr_start_flag = false;

	/* STEP 1: print out and reset total_time since last update */
	cur_jiffies = jiffies;
	time_since_last_update_ms = htc_batt_timer.total_time_ms +
		((cur_jiffies - htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ);
	BATT_DEBUG("%s: total_time since last batt update = %lu ms.\n",
				__func__, time_since_last_update_ms);
	htc_batt_timer.total_time_ms = 0; /* reset total time */
	htc_batt_timer.batt_system_jiffies = cur_jiffies;

	/* STEP 2: setup next batt uptate timer (can put in the last step)*/
	del_timer_sync(&htc_batt_timer.batt_timer);
	batt_set_check_timer(htc_batt_timer.time_out);

	/* STEP 3: update charging_source */
	htc_batt_info.prev.charging_source = htc_batt_info.rep.charging_source;
	cc_type = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE);
	if ((g_latest_chg_src  != POWER_SUPPLY_TYPE_USB_PD) &&
			(g_latest_chg_src  != POWER_SUPPLY_TYPE_USB) && (g_latest_chg_src  != POWER_SUPPLY_TYPE_USB_CDP) &&
			(smblib_typec_first_debounce_result() != CC_DET_DEFAULT) &&
			((cc_type == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM) || (cc_type == POWER_SUPPLY_TYPEC_SOURCE_HIGH)))
		htc_batt_info.rep.charging_source = POWER_SUPPLY_TYPE_TYPEC;
	else
		htc_batt_info.rep.charging_source = g_latest_chg_src;

	/* STEP 4: fresh battery information from gauge/charger */
	/* In 8996, the battery infor report by smbchg */
	g_is_rep_level_ready = false;
	batt_update_info_from_gauge();
	batt_update_info_from_charger();
	update_htc_chg_src();
	g_usb_conn_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_USB_CONN_TEMP);

	/* STEP: Update safety timer setting when cable just plug in */
	// TODO: Add other charger type
	if (htc_batt_info.rep.charging_source != htc_batt_info.prev.charging_source) {
		if (((htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP) &&
			(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_HVDCP) &&
			(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_HVDCP_3) &&
			(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_PD) &&
			(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_TYPEC)) ||
			g_flag_keep_charge_on || g_flag_disable_safety_timer) {
			set_batt_psy_property(POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE, 0);
		} else {
			set_batt_psy_property(POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE, 1);
		}
		gs_update_PSY = true;
	}

	/* STEP 5: Error handling by changing USBIN mode
	FIXME: under reviewing, issue?
	if(htc_batt_info.rep.charging_source < CHARGER_MHL_UNKNOWN)
		batt_error_handle();
	*/

	/* STEP: update & print battery cycle information */
	if (s_first)
		read_batt_cycle_info();
	calculate_batt_cycle_info(time_since_last_update_ms);

	/* STEP 6: battery level smoothen adjustment */
	batt_level_adjust(time_since_last_update_ms);
	g_is_rep_level_ready = true;

	/* STEP 7: force level=0 to trigger userspace shutdown
	FIXME: need discussing with HW which condition needs force shutdown
	*/

	/* STEP 8: Update limited charge
	Dou to some returned device is cause by limit charge,
	We not porting this feature on 8996.
	batt_update_limited_charge();
	*/

	/* STEP 9: Check if overloading is happeneed during charging */
	batt_check_overload(time_since_last_update_ms);

	/* STEP 10: update htc_extension state */
	update_htc_extension_state();

	/* STEP 11: set the charger contorl depends on current status
	   batt id, batt temp, batt eoc, full_level
	   if charging source exist, determine charging_enable */
	if ((int)htc_batt_info.rep.charging_source > POWER_SUPPLY_TYPE_BATTERY) {

		if(htc_batt_info.rep.level == 100)
			set_aicdata(IS_CHARGE_FULL);
		else
			set_aicdata(IS_CHARGING);
		/*  STEP 11.1.1 check and update chg_dis_reason */
		if(!batt_chgr_start_flag){
			do_gettimeofday(&rtc_now);
			gs_batt_chgr_start_time = rtc_now;
			batt_chgr_start_flag = true;
		}

		if ((htc_batt_info.prev.charging_source == POWER_SUPPLY_TYPE_UNKNOWN) || (s_first)) {
			schedule_delayed_work(&htc_batt_info.quick_charger_check_work, msecs_to_jiffies(10000));
			schedule_delayed_work(&htc_batt_info.screen_ibat_limit_enable_work, msecs_to_jiffies(SCREEN_LIMIT_IBAT_DELAY_MS));
		}

		if ((htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_PD) ||
				(htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_TYPEC))
			s_prev_user_set_chg_curr = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_PD_CURRENT_MAX);
		else
			s_prev_user_set_chg_curr = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_SDP_CURRENT_MAX);

		if (g_ftm_charger_control_flag == FTM_FAST_CHARGE || g_flag_force_ac_chg) {
			if (htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB){
				user_set_chg_curr = FAST_CHARGE_CURR;
			} else {
				user_set_chg_curr = WALL_CHARGE_CURR;
			}
		} else if (g_ftm_charger_control_flag == FTM_SLOW_CHARGE) {
			user_set_chg_curr = SLOW_CHARGE_CURR;
		} else {
			if ((htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB)) {
				// For UNKNOWN charger and USB charger below 500mA
				if (s_prev_user_set_chg_curr < SLOW_CHARGE_CURR)
					user_set_chg_curr = SLOW_CHARGE_CURR;
				else
					user_set_chg_curr = s_prev_user_set_chg_curr;
			} else if (htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_HVDCP){
				user_set_chg_curr = HVDCP_CHARGE_CURR;
			} else if (htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_HVDCP_3){
				if(htcchg_on)
					user_set_chg_curr = s_prev_user_set_chg_curr;
				else
					user_set_chg_curr = htc_batt_info.qc3_current_ua;
			} else if (htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_CDP){
				user_set_chg_curr = WALL_CHARGE_CURR;
			} else {
				// DCP and OCP case
				if (s_prev_user_set_chg_curr != 0)
					user_set_chg_curr = s_prev_user_set_chg_curr;
				else
					user_set_chg_curr = OTHER_CHARG_CURR;
			}
		}

		if (g_ftm_charger_control_flag == FTM_STOP_CHARGER)
			g_pwrsrc_dis_reason |= HTC_BATT_PWRSRC_DIS_BIT_FTM;
		else
			g_pwrsrc_dis_reason &= ~HTC_BATT_PWRSRC_DIS_BIT_FTM;

		if (is_bounding_fully_charged_level()) {
			if (g_flag_ats_limit_chg)
				g_chg_dis_reason |= HTC_BATT_CHG_DIS_BIT_STOP_SWOLLEN;
			else
				g_pwrsrc_dis_reason |= HTC_BATT_PWRSRC_DIS_BIT_MFG;
		} else {
			if (g_flag_ats_limit_chg)
				g_chg_dis_reason &= ~HTC_BATT_CHG_DIS_BIT_STOP_SWOLLEN;
			else
				g_pwrsrc_dis_reason &= ~HTC_BATT_PWRSRC_DIS_BIT_MFG;
		}
		if (g_usb_overheat || g_htc_usb_overheat){
			g_chg_dis_reason |= HTC_BATT_CHG_DIS_BIT_USB_OVERHEAT;
			g_pwrsrc_dis_reason |= HTC_BATT_PWRSRC_DIS_BIT_USB_OVERHEAT;
		}else{
			g_pwrsrc_dis_reason &= ~HTC_BATT_PWRSRC_DIS_BIT_USB_OVERHEAT;
			g_chg_dis_reason &= ~HTC_BATT_CHG_DIS_BIT_USB_OVERHEAT;
		}

		/* STEP 11.2.1 determin charging_eanbled for charger control */
		if (g_chg_dis_reason)
			charging_enabled = 0;
		else
			charging_enabled = 1;

		/* STEP 11.2.2 determin pwrsrc_eanbled for charger control */
		if (g_pwrsrc_dis_reason)
			pwrsrc_enabled = 0;
		else
			pwrsrc_enabled = 1;

		if ((user_set_chg_curr != s_prev_user_set_chg_curr) || s_first) {
			if ((htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_PD) &&
					(htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_TYPEC)){
				BATT_EMBEDDED("set charging_current(%d)", user_set_chg_curr);
				set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, user_set_chg_curr);
			}
		}

		/* STEP 11.2.3 check vfloat setting */
		// TODO: modify vfloat setting

		/* STEP 11.2.4 check ibat setting */
		ibat = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX);
		ibat_new = update_ibat_setting();

		if (ibat != ibat_new) {
			BATT_EMBEDDED("set ibat(%d)", ibat_new);
			set_batt_psy_property(POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, ibat_new);
		}

		if (htc_batt_info.rep.full_level != 100) {
			BATT_EMBEDDED("set full level charging_enable(%d)", charging_enabled);
			set_batt_psy_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, charging_enabled);
		} else if ((charging_enabled != gs_prev_charging_enabled) ||
				(htc_batt_info.prev.charging_source == POWER_SUPPLY_TYPE_UNKNOWN)) {
			BATT_EMBEDDED("set charging_enable(%d)", charging_enabled);
			set_batt_psy_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, charging_enabled);
		}

		if (htc_batt_info.rep.full_level != 100) {
			BATT_EMBEDDED("set full level pwrsrc_enable(%d)", pwrsrc_enabled);
			set_batt_psy_property(POWER_SUPPLY_PROP_INPUT_SUSPEND, !pwrsrc_enabled);
		} else if ((pwrsrc_enabled != s_prev_pwrsrc_enabled) ||
				(htc_batt_info.prev.charging_source == POWER_SUPPLY_TYPE_UNKNOWN)){
			BATT_EMBEDDED("set pwrsrc_enable(%d)", pwrsrc_enabled);
			set_batt_psy_property(POWER_SUPPLY_PROP_INPUT_SUSPEND, !pwrsrc_enabled);
		}

	} else {
		set_aicdata(IS_DISCHARGE);
		/* TODO: check if we need to enable batfet while unplugged */
		if (htc_batt_info.prev.charging_source != htc_batt_info.rep.charging_source || s_first) {
			not_entry_again	= false;
			first_cool_charger = true;
			g_BI_data_ready &= ~HTC_BATT_CHG_BI_BIT_CHGR;
			batt_chgr_start_flag = false;
			g_batt_chgr_start_temp = 0;
			g_batt_chgr_start_level = 0;
			g_batt_chgr_start_batvol = 0;
			user_set_chg_curr = 0;
			charging_enabled = 0;
			pwrsrc_enabled = 0;
			g_is_quick_charger = false;
			g_usb_overheat = false;
			g_usb_overheat_check_count = 0;
			g_need_to_check_impedance = true;
			g_is_screen_on_limit_ibat = false;
			/* Symptom :
			 *   In the case that host device charged to client device :
			 *     When the device is host, the function will set CURRENT_MAX to 0.
			 *     But pd_active is TRUE, therefore, PD_SUSPEND_SUPPORTED_VOTER will vote USB_ICL to 0
			 *     and it will make device cannot be charged when switched to client.
			 * Solution :
			 *   Only set CURRENT_MAX to 0 when PD_ACTIVE is FALSE.
			 */
			if (!get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_PD_ACTIVE))
				set_usb_psy_property(POWER_SUPPLY_PROP_SDP_CURRENT_MAX, user_set_chg_curr);
			cancel_delayed_work_sync(&htc_batt_info.iusb_5v_2a_ability_check_work);
			cancel_delayed_work_sync(&htc_batt_info.quick_charger_check_work);
			cancel_delayed_work_sync(&htc_batt_info.screen_ibat_limit_enable_work);
			update_htc_extension_state();
		}
	}

	gs_prev_charging_enabled = charging_enabled;
	s_prev_pwrsrc_enabled = pwrsrc_enabled;
	s_prev_user_set_chg_curr = user_set_chg_curr;

	/* Disable charger when entering FTM mode only in a MFG ROM
	 * htc_battery_pwrsrc_disable would change pwrsrc_enabled, so
	 * it needs to be put in the bottom of batt_worker.
	 */
	 // TODO: wait for htc_get_bootmode() ready
#if 0
	if (s_first == 1 && !strcmp(htc_get_bootmode(),"ftm")
			&& (of_get_property(of_chosen, "is_mfg_build", NULL))) {
		pr_info("%s: Under FTM mode, disable charger first.", __func__);
		/* Set charger_control to DISABLE_PWRSRC */
		set_batt_psy_property(POWER_SUPPLY_PROP_CHARGING_ENABLED, 0);
	}
#endif

	s_first = 0;

	if (htc_batt_info.icharger) {
		/* dump charger status */
		htc_batt_info.icharger->dump_all();
	}

	if (htc_batt_info.rep.over_vchg || g_pwrsrc_dis_reason || g_chg_dis_reason ||
		get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CHARGING_ENABLED) == 0) {
		htc_batt_info.rep.chg_en = 0;
	} else {
		src = htc_batt_info.rep.charging_source;
		if (src == POWER_SUPPLY_TYPE_UNKNOWN) {
			htc_batt_info.rep.chg_en = 0;
		} else if (src == POWER_SUPPLY_TYPE_USB || src == POWER_SUPPLY_TYPE_USB_CDP ||
					user_set_chg_curr == SLOW_CHARGE_CURR) {
			htc_batt_info.rep.chg_en = 1;
		} else {
			htc_batt_info.rep.chg_en = 2;
		}
	}

	if (htc_batt_info.rep.charging_source != htc_batt_info.prev.charging_source){
		if (htc_batt_info.rep.charging_source != POWER_SUPPLY_TYPE_USB_DCP) {
			gs_cable_impedance = 0;
			htc_batt_info.htc_extension &= ~HTC_EXT_BAD_CABLE_USED;
			gs_R_cable_impedance = 0;
			gs_aicl_result = 0;
			gs_measure_cable_impedance = true;
			cancel_delayed_work_sync(&htc_batt_info.cable_impedance_work);
		}else{
			if (gs_measure_cable_impedance == true) {
				schedule_delayed_work(&htc_batt_info.cable_impedance_work, msecs_to_jiffies(30000));
				gs_measure_cable_impedance = false;
			}
		}
	}
	if (htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_DCP){
		BATT_EMBEDDED("cable_impedance: %d, R_cable_impedance: %d, aicl_result: %d",
		gs_cable_impedance, gs_R_cable_impedance, gs_aicl_result);
	}

	if ((htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_PD) ||
			(htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_TYPEC))
		max_iusb = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_PD_CURRENT_MAX)/1000;
	else
		max_iusb = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_SDP_CURRENT_MAX)/1000;

#if 0
	htc_dump_chg_reg();
	/* FIXME: htc_extension not ready */
	BATT_EMBEDDED("ID=%d,"
		"level=%d,"
		"level_raw=%d,"
		"vol=%dmV,"
		"temp=%d,"
		"current(mA)=%d,"
		"chg_name=%s,"
		"chg_src=%d,"
		"chg_en=%d,"
		"health=%d,"
		"overload=%d,"
		"vbus(mV)=%d,"
		"MAX_IUSB(mA)=%d,"
		"MAX_IBAT(mA)=%d,"
		"iusb_now(mA)=%d,"
		"ibat_now(mA)=%d,"
		"AICL=%d,"
		"chg_batt_en=%d,"
		"status=%d,"
		"chg_limit_reason=%d,"
		"chg_stop_reason=%d,"
		"consistent=%d,"
		"flag=0x%08X,"
		"htc_ext=0x%x,"
		"htcchg=%d,"
		"level_accu=%d,"
		"cc_uAh=%d,"
		"usb_temp=%d,"
		"usb_overheat=%d,"
		"usb_overheat_stat=%d,"
		"batt_state=%d",
		htc_batt_info.rep.batt_id,
		htc_batt_info.rep.level,
		htc_batt_info.rep.level_raw,
		htc_batt_info.rep.batt_vol,
		htc_batt_info.rep.batt_temp,
		(htc_batt_info.rep.batt_current/1000),
		cg_chr_src[htc_batt_info.rep.charging_source],
		htc_batt_info.rep.chg_src,
		htc_batt_info.rep.chg_en,
		htc_batt_info.rep.health,
		htc_batt_info.rep.overload,
		(htc_batt_info.vbus/1000),
		max_iusb,
		get_property(htc_batt_info.batt_psy,POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX)/1000,
		get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000,
		get_property(htc_batt_info.batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW)/1000,
		get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000,
		htc_batt_info.rep.chg_batt_en,
		htc_batt_info.rep.status,
		htc_batt_info.current_limit_reason,
		g_pwrsrc_dis_reason,
		(htc_batt_info.store.consistent_flag ? htc_batt_info.store.batt_stored_soc : (-1)),
		htc_batt_info.k_debug_flag,
		htc_batt_info.htc_extension,
		htcchg_on? 1 : 0,
		g_total_level_raw,
		get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER),
		g_usb_conn_temp,
		g_usb_overheat? 1 : 0,
		g_htc_usb_overheat_check_state,
		htc_batt_info.state
		);
#endif
	if(gs_update_PSY){
		gs_update_PSY = false;
		power_supply_changed(htc_batt_info.batt_psy);
	}

	if (!g_flag_keep_charge_on)
		htc_usb_overheat_routine();

	/* BI Data Thermal Battery Temperaturee */
	do_gettimeofday(&rtc_now);
	if (((rtc_now.tv_sec - s_thermal_batt_update_time.tv_sec) > THERMAL_BATT_TEMP_UPDATE_TIME_THRES)
			&& ((rtc_now.tv_sec - s_thermal_batt_update_time.tv_sec) < THERMAL_BATT_TEMP_UPDATE_TIME_MAX)) {
		pr_info("[BATT][THERMAL] Update period: %ld, batt_temp = %d.\n",
			(long)(rtc_now.tv_sec - s_thermal_batt_update_time.tv_sec), htc_batt_info.rep.batt_temp);
		g_thermal_batt_temp = htc_batt_info.rep.batt_temp;
	}
	s_thermal_batt_update_time = rtc_now;
	if((batt_chgr_start_flag) && (g_batt_chgr_start_batvol ==0)){
		g_batt_chgr_start_temp = htc_batt_info.rep.batt_temp;
		g_batt_chgr_start_level = htc_batt_info.rep.level_raw;
		g_batt_chgr_start_batvol = htc_batt_info.rep.batt_vol;
	}

        if ((g_batt_aging_bat_vol == 0) || (htc_batt_info.rep.level_raw < g_batt_aging_level)) {
                g_batt_aging_bat_vol = htc_batt_info.rep.batt_vol;
                g_batt_aging_level = htc_batt_info.rep.level_raw;
        }

	if((g_BI_data_ready & HTC_BATT_CHG_BI_BIT_CHGR) == 0){
		if(batt_chgr_start_flag){
			if((rtc_now.tv_sec - gs_batt_chgr_start_time.tv_sec) > BI_BATT_CHGE_CHECK_TIME_THRES) {
				gs_batt_chgr_start_time = rtc_now;
			} else if((rtc_now.tv_sec - gs_batt_chgr_start_time.tv_sec) > BI_BATT_CHGE_UPDATE_TIME_THRES){
                                g_batt_chgr_end_temp = htc_batt_info.rep.batt_temp;
                                g_batt_chgr_end_level = htc_batt_info.rep.level_raw;
                                g_batt_chgr_end_batvol = htc_batt_info.rep.batt_vol;
                                g_batt_chgr_iusb = get_property(htc_batt_info.usb_psy,POWER_SUPPLY_PROP_SDP_CURRENT_MAX)/1000;
                                g_batt_chgr_ibat = htc_batt_info.rep.batt_current;
				g_BI_data_ready |= HTC_BATT_CHG_BI_BIT_CHGR;
				BATT_EMBEDDED("Trigger batt_chgr event.");
			}
		}
	}

	ibat = get_property(htc_batt_info.batt_psy,POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX);
	ibat_setting = get_property(htc_batt_info.main_psy,POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX);
	if (ibat != ibat_setting) {
		set_main_psy_property(POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, ibat);
	}

	g_pre_total_level_raw = g_total_level_raw;

	htcchg_check_cc_status();

	aicl = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
	if (aicl == 25 && htc_batt_info.rep.overload == 1) {
		htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, 0);
		msleep(50);
		htc_batt_info.icharger->register_write(USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
		set_batt_psy_property(POWER_SUPPLY_PROP_RERUN_AICL, 1);
	}

	wake_unlock(&htc_batt_timer.battery_lock);

}

#define CHG_FULL_CHECK_PERIOD_MS	10000
#define CLEAR_FULL_STATE_BY_LEVEL_THR 97
#define CONSECUTIVE_COUNT             3
static void chg_full_check_worker(struct work_struct *work)
{
	u32 vol = 0;
	u32 max_vbat = 0;
	s32 curr = 0;
	static int chg_full_count = 0;
	static bool b_recharging_cycle = false;

	if (g_latest_chg_src > POWER_SUPPLY_TYPE_UNKNOWN) {
		if (htc_batt_info.rep.is_full) {
			if (htc_batt_info.rep.level_raw < CLEAR_FULL_STATE_BY_LEVEL_THR) {
				vol = get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
				max_vbat = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX)/1000;
				if (vol < (max_vbat - 100)) {
					chg_full_count = 0;
					htc_batt_info.rep.is_full = false;
				}
				BATT_EMBEDDED("%s: vol:%d, max_vbat=%d, is_full=%d", __func__,
					vol, max_vbat, htc_batt_info.rep.is_full);
			}
		} else {
			vol = get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
			curr = get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_CURRENT_NOW)/1000;
			if ((vol >= htc_batt_info.batt_full_voltage_mv)
				&& (curr < 0) && (abs(curr) <= htc_batt_info.batt_full_current_ma)) {
				chg_full_count++;
				if (chg_full_count >= CONSECUTIVE_COUNT)
					htc_batt_info.rep.is_full = true;
			} else {
				chg_full_count = 0;
				htc_batt_info.rep.is_full = false;
			}
			BATT_EMBEDDED("%s: vol:%d, curr=%d, is_full_count=%d, is_full=%d", __func__,
				vol, curr, chg_full_count, htc_batt_info.rep.is_full);
		}
	        if((htc_batt_info.htc_extension & 0x8)){
			b_recharging_cycle = true;
			wake_unlock(&htc_batt_info.charger_exist_lock);
	        }else{
			if((g_batt_full_eoc_stop != 0) && (b_recharging_cycle))
				wake_unlock(&htc_batt_info.charger_exist_lock);
			else{
				b_recharging_cycle = false;
				wake_lock(&htc_batt_info.charger_exist_lock);
			}
	        }
	} else {
		chg_full_count = 0;
		htc_batt_info.rep.is_full = false;
		wake_unlock(&htc_batt_info.charger_exist_lock);
		return;
	}

	schedule_delayed_work(&htc_batt_info.chg_full_check_work,
							msecs_to_jiffies(CHG_FULL_CHECK_PERIOD_MS));

}

#ifdef CONFIG_HTC_BATT_PCN0021
bool htc_stats_is_ac(int type)
{
    return type == POWER_SUPPLY_TYPE_USB_DCP ||
           type == POWER_SUPPLY_TYPE_USB_ACA ||
           type == POWER_SUPPLY_TYPE_USB_CDP ||
           type == POWER_SUPPLY_TYPE_USB_HVDCP ||
           type == POWER_SUPPLY_TYPE_USB_HVDCP_3;
}

const char* htc_stats_classify(unsigned long sum, int sample_count)
{
    char* ret;
    if (sum < sample_count * 5 * 60 * 60L) {
        ret = "A(0~5hr)";
    } else if (sum < sample_count * 8 * 60 * 60L ) {
        ret = "B(5~8hr)";
    } else if (sum < sample_count * 12 * 60 * 60L) {
        ret = "C(8~12hr)";
    } else if (sum < sample_count * 15 * 60 * 60L) {
        ret = "D(12~15hr)";
    } else if (sum < sample_count * 20 * 60 * 60L) {
        ret = "E(15~20hr)";
    } else {
        ret = "F(20hr~)";
    }
    return ret;
}

bool htc_stats_is_valid(int category, unsigned long chg_time, unsigned long dischg_time, int unplug_level, int plug_level)
{
    bool ret = false;
    switch (category)
    {
        case HTC_STATS_CATEGORY_ALL:
            ret = dischg_time >= 60 * 60L &&
                  chg_time >= 60 * 60L;
            break;
        case HTC_STATS_CATEGORY_FULL_LOW:
            ret = dischg_time >= 60 * 60L &&
                  chg_time >= 60 * 60L &&
                  unplug_level >= 80 &&
                  plug_level <= 30;
            break;
    }
    return ret;
}

void htc_stats_update(int category, unsigned long chg_time, unsigned long dischg_time)
{
    struct htc_statistics_category* category_ptr;
    switch (category)
    {
        case HTC_STATS_CATEGORY_ALL: category_ptr = &g_htc_stats_category_all; break;
        case HTC_STATS_CATEGORY_FULL_LOW: category_ptr = &g_htc_stats_category_full_low; break;
    }

    category_ptr->sample_count += 1;
    category_ptr->chg_time_sum += chg_time;
    category_ptr->dischg_time_sum += dischg_time;
}

const char* htc_stats_category2str(int category)
{
    const char* ret;
    switch (category)
    {
        case HTC_STATS_CATEGORY_ALL: ret = "all"; break;
        case HTC_STATS_CATEGORY_FULL_LOW: ret = "full_low"; break;
    }
    return ret;
}

void htc_stats_calculate_statistics_data(int category, unsigned long chg_time, unsigned long dischg_time)
{
    int unplug_level = g_htc_stats_data.end_chg_batt_level;
    int plug_level = g_htc_stats_data.begin_chg_batt_level;
    struct htc_statistics_category* category_ptr;

    switch (category) {
        case HTC_STATS_CATEGORY_ALL: category_ptr = &g_htc_stats_category_all; break;
        case HTC_STATS_CATEGORY_FULL_LOW: category_ptr = &g_htc_stats_category_full_low; break;
    }

    if (htc_stats_is_valid(category, chg_time, dischg_time, unplug_level, plug_level))
    {
        htc_stats_update(category, chg_time, dischg_time);
    }
    else
    {
        BATT_DEBUG("%s: statistics_%s: This sampling is invalid, chg_time=%ld, dischg_time=%ld\n",
            HTC_STATISTICS,
            htc_stats_category2str(category),
            chg_time,
            dischg_time);
    }

    if (category_ptr->sample_count >= HTC_STATS_SAMPLE_NUMBER)
    {
        BATT_DEBUG("%s: statistics_%s: group=%s, chg_time_sum=%ld, dischg_time_sum=%ld, sample_count=%d\n",
            HTC_STATISTICS,
            htc_stats_category2str(category),
            htc_stats_classify(category_ptr->dischg_time_sum, category_ptr->sample_count),
            category_ptr->chg_time_sum,
            category_ptr->dischg_time_sum,
            category_ptr->sample_count);

        // Reset sampling data
        category_ptr->sample_count = 0;
        category_ptr->chg_time_sum = 0L;
        category_ptr->dischg_time_sum = 0L;
     }
}

void htc_stats_update_charging_statistics(int latest, int prev)
{
    struct timeval rtc_now;
    struct tm time_info;
    char time_str[25];
    long dischg_time = 0L;
    long chg_time = 0L;
    char debug[10] = "[debug]";
    bool debug_flag = false;

    if (debug_flag) {
        BATT_DEBUG("%s: %s update_charging_statistics(): prev=%d, now=%d\n",
            HTC_STATISTICS, debug, prev, latest);
    }

    do_gettimeofday(&rtc_now);
    time_to_tm(rtc_now.tv_sec, 0, &time_info);
    snprintf(time_str, sizeof(time_str), "%04ld-%02d-%02d %02d:%02d:%02d UTC",
        time_info.tm_year+1900,
        time_info.tm_mon+1,
        time_info.tm_mday,
        time_info.tm_hour,
        time_info.tm_min,
        time_info.tm_sec);

    // When AC is plugged in
    if (prev == POWER_SUPPLY_TYPE_UNKNOWN && htc_stats_is_ac(latest) && !g_htc_stats_charging)
    {
        g_htc_stats_charging = true;

        if (0 != g_htc_stats_data.end_chg_time)
        {
            // Collect battery info for end uncharging
            dischg_time = rtc_now.tv_sec - g_htc_stats_data.end_chg_time;
            if (dischg_time < 0) dischg_time = 0L;
            BATT_DEBUG("%s: sampling: dischg_time=%ld(s), level=%d->%d, prev_chg_time=%ld(s)\n",
                HTC_STATISTICS,
                dischg_time,
                g_htc_stats_data.end_chg_batt_level,
                htc_batt_info.rep.level,
                g_htc_stats_data.end_chg_time - g_htc_stats_data.begin_chg_time);

            // Calculate charging time
            chg_time = g_htc_stats_data.end_chg_time - g_htc_stats_data.begin_chg_time;
            if (chg_time < 0) chg_time = 0L;

            htc_stats_calculate_statistics_data(HTC_STATS_CATEGORY_ALL, chg_time, dischg_time);
            htc_stats_calculate_statistics_data(HTC_STATS_CATEGORY_FULL_LOW, chg_time, dischg_time);
        }

        // Clear statistics data
        g_htc_stats_data.begin_chg_time = 0;
        g_htc_stats_data.end_chg_time = 0;
        g_htc_stats_data.begin_chg_batt_level = 0;
        g_htc_stats_data.end_chg_batt_level = 0;

        // Collect battery info for begin charging
        g_htc_stats_data.begin_chg_time = rtc_now.tv_sec;
        g_htc_stats_data.begin_chg_batt_level = htc_batt_info.rep.level;

        BATT_DEBUG("%s: begin charging: level=%d, at=%s\n",
            HTC_STATISTICS,
            g_htc_stats_data.begin_chg_batt_level,
            time_str);
    }

    // When AC is plugged out
    if (htc_stats_is_ac(prev) && latest == POWER_SUPPLY_TYPE_UNKNOWN && g_htc_stats_charging)
    {
        g_htc_stats_charging = false;

        // Collect battery info for end charging
        g_htc_stats_data.end_chg_time = rtc_now.tv_sec;
        g_htc_stats_data.end_chg_batt_level = htc_batt_info.rep.level;

        BATT_DEBUG("%s: end charging: level=%d, at=%s\n",
            HTC_STATISTICS,
            g_htc_stats_data.end_chg_batt_level,
            time_str);
    }
}
#endif //CONFIG_HTC_BATT_PCN0021

void htc_battery_info_update(enum power_supply_property prop, int intval)
{
	int present = 0;

	if (!g_htc_battery_probe_done)
		return;

	switch (prop) {
		case POWER_SUPPLY_PROP_STATUS:
			/* Get charger type from usb psy interface. */
			present = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_PRESENT);
			if (present)
				g_latest_chg_src = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_TYPE);
			else
				g_latest_chg_src = POWER_SUPPLY_TYPE_UNKNOWN;
			if (htc_batt_info.rep.status != intval ||
				g_latest_chg_src != htc_batt_info.rep.charging_source) {
#ifdef CONFIG_HTC_BATT_PCN0021
				htc_stats_update_charging_statistics(g_latest_chg_src, htc_batt_info.rep.charging_source);
#endif //CONFIG_HTC_BATT_PCN0021
				htc_batt_info.rep.status = intval;
				htc_batt_schedule_batt_info_update();
				if (!delayed_work_pending(&htc_batt_info.chg_full_check_work)
					&& (g_latest_chg_src > POWER_SUPPLY_TYPE_UNKNOWN)) {
						wake_lock(&htc_batt_info.charger_exist_lock);
					schedule_delayed_work(&htc_batt_info.chg_full_check_work,0);
				} else {
					if (delayed_work_pending(&htc_batt_info.chg_full_check_work)
						&& (g_latest_chg_src == POWER_SUPPLY_TYPE_UNKNOWN)) {
						BATT_LOG("cancel chg_full_check_work when plug out.\n");
						cancel_delayed_work_sync(&htc_batt_info.chg_full_check_work);
						schedule_delayed_work(&htc_batt_info.chg_full_check_work,0);
 					}
				}
				if (!g_flag_keep_charge_on){
					if(!delayed_work_pending(&htc_batt_info.is_usb_overheat_work)) {
						schedule_delayed_work(&htc_batt_info.is_usb_overheat_work, 0);
					}
				}
			}
			break;
		case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
			htc_batt_info.rep.chg_batt_en = intval;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if (htc_batt_info.rep.level_raw != intval) {
				htc_batt_info.rep.level_raw = intval;
				htc_batt_schedule_batt_info_update();
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			if (htc_batt_info.rep.health != intval) {
				htc_batt_info.rep.health = intval;
				htc_batt_schedule_batt_info_update();
			}
			break;
		case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
			htc_batt_schedule_batt_info_update();
			break;
		default:
			break;
	}
}

static int htc_batt_charger_control(enum charger_control_flag control)
{
	int ret = 0;

	BATT_EMBEDDED("%s: user switch charger to mode: %u", __func__, control);

	switch (control) {
		case STOP_CHARGER:
			g_chg_dis_reason |= HTC_BATT_CHG_DIS_BIT_USR_TMR;
			break;
		case ENABLE_CHARGER:
			g_chg_dis_reason &= ~HTC_BATT_CHG_DIS_BIT_USR_TMR;
			break;
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
		case DISABLE_PWRSRC_FINGERPRINT:
			set_batt_psy_property(POWER_SUPPLY_PROP_INPUT_SUSPEND, true);
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING
		case DISABLE_PWRSRC:
			g_pwrsrc_dis_reason |= HTC_BATT_PWRSRC_DIS_BIT_API;
			break;
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
		case ENABLE_PWRSRC_FINGERPRINT:
			set_batt_psy_property(POWER_SUPPLY_PROP_INPUT_SUSPEND, false);
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING
		case ENABLE_PWRSRC:
			g_pwrsrc_dis_reason &= ~HTC_BATT_PWRSRC_DIS_BIT_API;
			break;
		case ENABLE_LIMIT_CHARGER:
		case DISABLE_LIMIT_CHARGER:
			BATT_EMBEDDED("%s: skip charger_contorl(%d)", __func__, control);
			return ret;
			break;

		default:
			BATT_EMBEDDED("%s: unsupported charger_contorl(%d)", __func__, control);
			ret =  -1;
			break;

	}

	htc_batt_schedule_batt_info_update();

	return ret;
}

#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
int htc_battery_charger_switch_internal(int enable)
{
	int rc = 0;

	switch (enable) {
	case ENABLE_PWRSRC_FINGERPRINT:
	case DISABLE_PWRSRC_FINGERPRINT:
		rc = htc_batt_charger_control(enable);
		break;
	default:
		pr_info("%s: invalid type, enable=%d\n", __func__, enable);
		return -EINVAL;
	}

	if (rc < 0)
		BATT_ERR("FP control charger failed!");

	return rc;
}
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004

bool htc_battery_get_discharging_reason(void)
{
	return g_chg_dis_reason;
}

void htc_ftm_disable_charger(bool disable)
{
	if (disable)
		g_ftm_charger_control_flag = FTM_STOP_CHARGER;
	else
		g_ftm_charger_control_flag = FTM_ENABLE_CHARGER;
}

bool htc_get_htcchg_sts(void) {
	return htcchg_on;
}

static ssize_t htc_battery_set_full_level(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int rc = 0;
	int percent = 100;

	rc = kstrtoint(buf, 10, &percent);
	if (rc)
		return rc;

	htc_batt_info.rep.full_level = percent;

	BATT_LOG(" set full_level constraint as %d.\n", (int)percent);

    return count;
}
static ssize_t htc_battery_charger_stat(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "%d\n", g_charger_ctrl_stat);
}
static ssize_t htc_battery_charger_switch(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int enable = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &enable);
	if (rc)
		return rc;

	BATT_EMBEDDED("Set charger_control:%d", enable);
	if (enable >= END_CHARGER)
		return -EINVAL;

	rc = htc_batt_charger_control(enable);
	if (rc < 0) {
		BATT_ERR("charger control failed!");
		return rc;
	}
	g_charger_ctrl_stat = enable;

	return count;
}

static ssize_t htc_battery_set_phone_call(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int phone_call = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &phone_call);
	if (rc)
		return rc;

	BATT_LOG("set context phone_call=%d\n", phone_call);

	if (phone_call) {
		suspend_highfreq_check_reason |= SUSPEND_HIGHFREQ_CHECK_BIT_TALK;
		g_chg_limit_reason |= HTC_BATT_CHG_LIMIT_BIT_TALK;
		pmic_bob_regulator_pwm_mode_enable(true);
	} else {
		suspend_highfreq_check_reason &= ~SUSPEND_HIGHFREQ_CHECK_BIT_TALK;
		g_chg_limit_reason &= ~HTC_BATT_CHG_LIMIT_BIT_TALK;
		pmic_bob_regulator_pwm_mode_enable(false);
	}

    return count;
}

static ssize_t htc_battery_set_net_call(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int net_call = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &net_call);
	if (rc)
		return rc;

	BATT_LOG("set context net_call=%d\n", net_call);

	if (net_call) {
		suspend_highfreq_check_reason |= SUSPEND_HIGHFREQ_CHECK_BIT_SEARCH;
		g_chg_limit_reason |= HTC_BATT_CHG_LIMIT_BIT_NET_TALK;
	} else {
		suspend_highfreq_check_reason &= ~SUSPEND_HIGHFREQ_CHECK_BIT_SEARCH;
		g_chg_limit_reason &= ~HTC_BATT_CHG_LIMIT_BIT_NET_TALK;
	}


	return count;
}

static ssize_t htc_battery_set_play_music(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int play_music = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &play_music);
	if (rc)
		return rc;

	BATT_LOG("set context play music=%d\n", play_music);

	if (play_music)
		suspend_highfreq_check_reason |= SUSPEND_HIGHFREQ_CHECK_BIT_MUSIC;
	else
		suspend_highfreq_check_reason &= ~SUSPEND_HIGHFREQ_CHECK_BIT_MUSIC;

    return count;
}

static ssize_t htc_battery_rt_vol(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return sprintf(buf, "%d\n",
				get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW));
}

static ssize_t htc_battery_rt_current(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return sprintf(buf, "%d\n",
				get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_CURRENT_NOW));
}

static ssize_t htc_battery_rt_temp(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return sprintf(buf, "%d\n",
				get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_TEMP));
}

static ssize_t htc_battery_rt_id(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	if (g_test_power_monitor)
		htc_batt_info.rep.batt_id = 77;

    return sprintf(buf, "%d\n", htc_batt_info.rep.batt_id);
}

static ssize_t htc_battery_rt_id_mv(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return sprintf(buf, "%d\n",
				get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_RESISTANCE_ID));
}

static ssize_t htc_battery_ftm_charger_stat(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return sprintf(buf, "%d\n", g_ftm_charger_control_flag);
}

static ssize_t htc_battery_ftm_charger_switch(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int enable = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &enable);
	if (rc)
		return rc;

	BATT_EMBEDDED("Set ftm_charger_control:%d", enable);
	if (enable >= FTM_END_CHARGER)
		return -EINVAL;

	g_ftm_charger_control_flag = enable;

	htc_batt_schedule_batt_info_update();

    return count;
}

static ssize_t htc_battery_over_vchg(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	htc_batt_info.rep.over_vchg = (get_property(htc_batt_info.usb_psy,
		POWER_SUPPLY_PROP_HEALTH) == POWER_SUPPLY_HEALTH_OVERVOLTAGE)?1:0;

	return sprintf(buf, "%d\n", htc_batt_info.rep.over_vchg);
}

static ssize_t htc_battery_overload(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    return sprintf(buf, "%d\n", htc_batt_info.rep.overload);
}

static ssize_t htc_battery_show_htc_extension_attr(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "%d\n", htc_batt_info.htc_extension);;
}

static ssize_t htc_battery_show_usb_overheat(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "%d\n", g_usb_overheat? 1:0);
}

static ssize_t htc_battery_show_batt_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int len = 0;
	time_t t = g_batt_first_use_time;
	struct tm timeinfo;
	u8 reg, addr;
	int soc, vbat_mv, ibat_ma, iusb_ma, ibat_max, iusb_max, aicl_result,
	    vbus_uv, batt_temp, health, present, charger_type, charger_done,
	    usb_present, usb_online, charger_temp, charger_temp_max, cc_uah, usb_conn_temp;
	int isen_value, isen_value_adc;

	time_to_tm(t, 0, &timeinfo);

	usb_conn_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_USB_CONN_TEMP);

	isen_value = get_property(htc_batt_info.batt_psy,POWER_SUPPLY_PROP_HTC_CHARGE_NOW);
	isen_value_adc = get_property(htc_batt_info.batt_psy,POWER_SUPPLY_PROP_HTC_CHARGE_NOW_RAW);

	/* collect htc_battery vars */
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"charging_source: %s;\n"
			"charging_enabled: %d;\n"
			"overload: %d;\n"
			"Percentage(%%): %d;\n"
			"Percentage_raw(%%): %d;\n"
			"gs_cable_impedance: %d\n"
			"gs_R_cable_impedance: %d\n"
			"gs_aicl_result: %d\n"
			"batt_cycle_first_use: %04ld/%02d/%02d/%02d:%02d:%02d\n"
			"batt_cycle_level_raw: %u;\n"
			"batt_cycle_overheat(s): %u;\n"
			"htc_extension: 0x%x;\n"
			"usb_overheat_state: %d;\n"
			"USB_PWR_TEMP(degree): %d;\n"
			"ISEN_VALUE_ADC: %d;\n"
			"ISEN_VALUE: %d;\n",
			cg_chr_src[htc_batt_info.rep.charging_source],
			htc_batt_info.rep.chg_en,
			htc_batt_info.rep.overload,
			htc_batt_info.rep.level,
			htc_batt_info.rep.level_raw,
			gs_cable_impedance, gs_R_cable_impedance, gs_aicl_result,
			timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
			g_total_level_raw,
			g_overheat_55_sec,
			htc_batt_info.htc_extension,
			g_htc_usb_overheat_check_state,
			usb_conn_temp,
			isen_value_adc,
			isen_value
			);

	soc       = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CAPACITY);
	vbat_mv   = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	ibat_ma   = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW)/1000;
	iusb_ma   = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW)/1000;
	aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
	vbus_uv   = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	batt_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_TEMP);
	health    = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_HEALTH);
	present   = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_PRESENT);
	charger_type = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CHARGE_TYPE);
	charger_done = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CHARGE_DONE);
	usb_present  = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_PRESENT);
	usb_online   = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_ONLINE);
	ibat_max = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX)/1000;
	if ((htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_USB_PD) ||
			(htc_batt_info.rep.charging_source == POWER_SUPPLY_TYPE_TYPEC))
		iusb_max = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_PD_CURRENT_MAX)/1000;
	else
		iusb_max = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_SDP_CURRENT_MAX)/1000;
	cc_uah   = get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);
	charger_temp = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CHARGER_TEMP);
	charger_temp_max = get_property(htc_batt_info.batt_psy, POWER_SUPPLY_PROP_CHARGER_TEMP_MAX);

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"SOC(%%): %d;\n"
			"VBAT(mV): %d;\n"
			"IBAT(mA): %d;\n"
			"IUSB(mA): %d;\n"
			"MAX_IBAT(mA): %d;\n"
			"MAX_IUSB(mA): %d;\n"
			"AICL_RESULT: %d\n"
			"VBUS(uV): %d;\n"
			"BATT_TEMP: %d;\n"
			"HEALTH: %d;\n"
			"BATT_PRESENT(bool): %d;\n"
			"CHARGE_TYPE: %d;\n"
			"CHARGE_DONE: %d;\n"
			"USB_PRESENT: %d;\n"
			"USB_ONLINE: %d;\n"
			"CHARGER_TEMP: %d;\n"
			"CHARGER_TEMP_MAX: %d;\n"
			"CC_uAh: %d;\n",
			soc, vbat_mv, ibat_ma, iusb_ma, ibat_max, iusb_max, aicl_result,
			vbus_uv, batt_temp, health, present,
			charger_type, charger_done, usb_present, usb_online,
			charger_temp, charger_temp_max, cc_uah
			);

	// Dump pmi8998 register
	htc_batt_info.icharger->register_read(USBIN_CMD_IL_REG, &reg);			// 0x1340
	len += scnprintf(buf + len, PAGE_SIZE - len, "USB_CMD_IL_REG: 0x%02x;\n", reg);
	htc_batt_info.icharger->register_read(USBIN_CURRENT_LIMIT_CFG_REG, &reg);	// 0x1370
	len += scnprintf(buf + len, PAGE_SIZE - len, "USBIN_CURRENT_LIMIT_CFG: 0x%02x;\n", reg);
	htc_batt_info.icharger->register_read(USBIN_AICL_OPTIONS_CFG_REG, &reg);	// 0x1380
	len += scnprintf(buf + len, PAGE_SIZE - len, "USBIN_AICL_OPTIONS_CFG: 0x%02x;\n", reg);
	htc_batt_info.icharger->register_read(FAST_CHARGE_CURRENT_CFG_REG, &reg);	// 0x1061
	len += scnprintf(buf + len, PAGE_SIZE - len, "FAST_CHARGE_CURRENT_CFG: 0x%02x;\n", reg);
	htc_batt_info.icharger->register_read(FG_BCL_LMH_STS1, &reg);
	len += scnprintf(buf + len, PAGE_SIZE - len, "FG_BCL_LMH_STS1: 0x%02x;\n", reg);//0x4209

	/* charger peripheral */
	len += scnprintf(buf + len, PAGE_SIZE - len, "CHGR_STS ------;\n");
	for (addr = 0x6; addr <= 0xE; addr++){
		htc_batt_info.icharger->register_read(CHGR_BASE + addr, &reg);	//"CHGR Status"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", CHGR_BASE + addr, reg);
	}
	/* battery interface peripheral */

	/* usb charge path peripheral */
	len += scnprintf(buf + len, PAGE_SIZE - len,"USB_STS ------;\n");
	for (addr = 0x6; addr <= 0x10; addr++){
		htc_batt_info.icharger->register_read(USBIN_BASE + addr, &reg);	//"USB Status"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + addr, reg);
	}
	len += scnprintf(buf + len, PAGE_SIZE - len,"USB_CFG ------;\n");
	for (addr = 0x58; addr <= 0x66; addr++){
		htc_batt_info.icharger->register_read(USBIN_BASE + addr, &reg); //"USB Config"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + addr, reg);
	}

	htc_batt_info.icharger->register_read(USBIN_BASE + 0x70, &reg); //"USB Config"
	len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + 0x70, reg);

	for (addr = 0x80; addr <= 0x84; addr++){
		htc_batt_info.icharger->register_read(USBIN_BASE + addr, &reg); //"USB Config"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", USBIN_BASE + addr, reg);
	}

	/* misc peripheral */
	len += scnprintf(buf + len, PAGE_SIZE - len,"MISC_STS ------;\n");
	for (addr = 0x6; addr <= 0x10; addr++){
		htc_batt_info.icharger->register_read(MISC_BASE + addr, &reg);	//"MISC Status"
		len += scnprintf(buf + len, PAGE_SIZE - len,"0x%04x: 0x%02x;\n", MISC_BASE + addr, reg);
	}

	return len;
}

static ssize_t htc_battery_show_cc_attr(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "cc:%d\n", get_property(htc_batt_info.bms_psy,
		POWER_SUPPLY_PROP_CHARGE_NOW_RAW));
}

static ssize_t htc_battery_show_capacity_raw(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "%d\n", htc_batt_info.rep.level_raw);
}

static ssize_t htc_battery_batt_vol(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        return sprintf(buf, "%d\n", htc_batt_info.rep.batt_vol);
}

static ssize_t htc_battery_charging_source(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	return sprintf(buf, "%d\n", htc_batt_info.rep.chg_src);
}

static ssize_t htc_battery_temp(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        return sprintf(buf, "%d\n", htc_batt_info.rep.batt_temp);
}

#define VBUS_10_UV	10000000
#define VBUS_6_UV	6000000
static ssize_t htc_charger_type(struct device *dev,
                 struct device_attribute *attr,
                char *buf)
{
	int chg_type, aicl_result;
	int standard_cable;

	// TODO: Check workable cable
	//standard_cable = workable_charging_cable()==1? 1: 0;
	standard_cable = 1;

	if(g_is_unknown_charger){
		aicl_result = 500;
		chg_type = DATA_UNKNOWN_CHARGER;
	}else{
		switch(htc_batt_info.rep.charging_source){
			case POWER_SUPPLY_TYPE_UNKNOWN:
			case POWER_SUPPLY_TYPE_BATTERY:
				chg_type = DATA_NO_CHARGING;
				aicl_result = 0;
				break;
			case POWER_SUPPLY_TYPE_USB:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_USB;
				break;
			case POWER_SUPPLY_TYPE_USB_CDP:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_USB_CDP;
				break;
			case POWER_SUPPLY_TYPE_USB_HVDCP:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_QC2;
				break;
			case POWER_SUPPLY_TYPE_USB_HVDCP_3:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_QC3;
				break;
			case POWER_SUPPLY_TYPE_USB_DCP:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_AC;
				break;
			case POWER_SUPPLY_TYPE_USB_PD:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				if (htc_batt_info.vbus > VBUS_10_UV)
					chg_type = DATA_PD_12V;
				else if (htc_batt_info.vbus > VBUS_6_UV)
					chg_type = DATA_PD_9V;
				else
					chg_type = DATA_PD_5V;
				break;
			case POWER_SUPPLY_TYPE_TYPEC:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_TYPE_C;
				break;
			default:
				aicl_result = get_property(htc_batt_info.usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED)/1000;
				chg_type = DATA_UNKNOWN_TYPE;
				break;
		}
	}
	return sprintf(buf, "charging_source=%s;iusb_current=%d;workable=%d\n", htc_chr_type_data_str[chg_type], aicl_result, standard_cable);
}

static ssize_t htc_thermal_batt_temp(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf (buf, "%d\n", g_thermal_batt_temp);
}

static ssize_t htc_batt_chgr_data(char *buf)
{
	int chg_type, len = 0;

        if(((g_BI_data_ready & HTC_BATT_CHG_BI_BIT_CHGR) == 0)||(g_batt_chgr_end_batvol == 0)){
                return len;
	}

	if(g_is_unknown_charger){
                chg_type = DATA_UNKNOWN_CHARGER;
	}else{
		switch(htc_batt_info.rep.charging_source){
			case POWER_SUPPLY_TYPE_UNKNOWN:
			case POWER_SUPPLY_TYPE_BATTERY:
				chg_type = DATA_NO_CHARGING;
				break;
			case POWER_SUPPLY_TYPE_USB:
				chg_type = DATA_USB;
				break;
			case POWER_SUPPLY_TYPE_USB_CDP:
				chg_type = DATA_USB_CDP;
				break;
			case POWER_SUPPLY_TYPE_USB_HVDCP:
				chg_type = DATA_QC2;
				break;
			case POWER_SUPPLY_TYPE_USB_HVDCP_3:
				chg_type = DATA_QC3;
				break;
			case POWER_SUPPLY_TYPE_USB_DCP:
				chg_type = DATA_AC;
				break;
			case POWER_SUPPLY_TYPE_USB_PD:
				if (htc_batt_info.vbus > VBUS_10_UV)
					chg_type = DATA_PD_12V;
				else if (htc_batt_info.vbus > VBUS_6_UV)
					chg_type = DATA_PD_9V;
				else
					chg_type = DATA_PD_5V;
				break;
			case POWER_SUPPLY_TYPE_TYPEC:
				chg_type = DATA_TYPE_C;
				break;
			default:
				chg_type = DATA_UNKNOWN_TYPE;
				break;
		}
	}

	len = sprintf(buf, "appID: charging_log\n"
			"category: charging\n"
			"action: charging\n"
			"Attribute:\n"
			"{type, %s}\n"
			"{iusb, %d}\n"
			"{ibat, %d}\n"
			"{start_temp, %d}\n"
			"{end_temp, %d}\n"
			"{start_level, %d}\n"
			"{end_level, %d}\n"
			"{batt_vol_start, %d}\n"
			"{batt_vol_end, %d}\n"
			"{chg_cycle, %d}\n"
			"{overheat, %d}\n"
			"{batt_level, %d}\n"
			"{batt_vol, %d}\n"
			"{err_code, %d}\n",
			htc_chr_type_data_str[chg_type], g_batt_chgr_iusb, g_batt_chgr_ibat, g_batt_chgr_start_temp, g_batt_chgr_end_temp, g_batt_chgr_start_level, g_batt_chgr_end_level, g_batt_chgr_start_batvol, g_batt_chgr_end_batvol, g_total_level_raw, g_overheat_55_sec, g_batt_aging_level, g_batt_aging_bat_vol, 0);

	BATT_EMBEDDED("Charging_log:\n%s", buf);

	g_batt_chgr_end_batvol = 0;
	return len;
}

static ssize_t htc_batt_bidata(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int len = 0;

	len = htc_batt_chgr_data(buf);

	return len;

}

static ssize_t htc_consist_data(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int len = 0;
	struct timespec xtime = CURRENT_TIME;
	static int data[6];
	unsigned long currtime_s = (xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC)/MSEC_PER_SEC;

	if (!g_is_consistent_level_ready)
		data[0] = 0;
	else
		data[0] = STORE_MAGIC_NUM;			// magic number

	data[1] = htc_batt_info.rep.level;		// soc
	data[2] = 0;							// ocv_uv
	data[3] = 0;							// cc_uah
	data[4] = (int) currtime_s;				// current time
	data[5] = htc_batt_info.rep.batt_temp;	// batt temperature

	len = sizeof(data);
	memcpy(buf,(char*) data, len);


	return len;
}

static ssize_t htc_cycle_data(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int len = 0;
	static int data[4];

	data[0] = g_total_level_raw;
	data[1] = g_overheat_55_sec;
	data[2] = g_batt_first_use_time;
	data[3] = g_batt_cycle_checksum;

	len = sizeof(data);
	memcpy(buf,(char*) data, len);

	return len;
}

#define BATT_CHK_MAX				10
static const unsigned int sc_cycle_num[BATT_CHK_MAX] = { 5000, 7500, 10000, 12500, 15000, 17500, 20000, 30000, 40000, 50000};
#define BATT_CHK_OVERHEAT_TIME	7200 // 2HR
static ssize_t htc_batt_check(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	unsigned int idx = 0;
	int batt_status;

	while (idx < BATT_CHK_MAX) {
		if (g_total_level_raw < sc_cycle_num[idx])
			break;
		idx++;
	}

	if (g_overheat_55_sec > BATT_CHK_OVERHEAT_TIME)
		batt_status = BATT_CHK_MAX - idx -1;
	else
		batt_status = BATT_CHK_MAX - idx;

	if (batt_status < 0)
		batt_status = 0;

	return sprintf (buf, "%d\n", batt_status);
}

static ssize_t htc_clr_cycle(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	g_total_level_raw = 0;
	g_overheat_55_sec = 0;
	g_batt_first_use_time = 0;
	g_batt_cycle_checksum= 0;

	BATT_LOG("CLEAR BATT CYCLE DATA\n");
	return count;
}

static ssize_t htc_battery_state(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        return sprintf(buf, "%d\n", g_htc_battery_probe_done);
}

static ssize_t htc_thermal_limnit_ma(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int thermal_ma = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &thermal_ma);
	if (rc)
		return rc;

	BATT_LOG("Thermal set limit ma=%d\n", thermal_ma);

	if(thermal_ma < 100 ){
		g_thermal_limit_ma = 0;
		return count;
	}

	g_thermal_limit_ma = thermal_ma;

	htc_batt_schedule_batt_info_update();

	return count;
}

static struct device_attribute htc_battery_attrs[] = {
	__ATTR(batt_attr_text, S_IRUGO, htc_battery_show_batt_attr, NULL),
	__ATTR(full_level, S_IWUSR | S_IWGRP, NULL, htc_battery_set_full_level),
	__ATTR(full_level_dis_batt_chg, S_IWUSR | S_IWGRP, NULL, htc_battery_set_full_level),
	__ATTR(charger_control, S_IWUSR | S_IWGRP | S_IRUGO, htc_battery_charger_stat, htc_battery_charger_switch),
	__ATTR(phone_call, S_IWUSR | S_IWGRP, NULL, htc_battery_set_phone_call),
	__ATTR(net_call, S_IWUSR | S_IWGRP, NULL, htc_battery_set_net_call),
	__ATTR(play_music, S_IWUSR | S_IWGRP, NULL, htc_battery_set_play_music),
	__ATTR(batt_vol_now, S_IRUGO, htc_battery_rt_vol, NULL),
	__ATTR(batt_current_now, S_IRUGO, htc_battery_rt_current, NULL),
	__ATTR(batt_temp_now, S_IRUGO, htc_battery_rt_temp, NULL),
	__ATTR(batt_id, S_IRUGO, htc_battery_rt_id, NULL),
	__ATTR(batt_id_now, S_IRUGO, htc_battery_rt_id_mv, NULL),
	__ATTR(ftm_charger_control, S_IWUSR | S_IWGRP | S_IRUGO, htc_battery_ftm_charger_stat,
		htc_battery_ftm_charger_switch),
	__ATTR(over_vchg, S_IRUGO, htc_battery_over_vchg, NULL),
	__ATTR(overload, S_IRUGO, htc_battery_overload, NULL),
	__ATTR(htc_extension, S_IRUGO, htc_battery_show_htc_extension_attr, NULL),
	__ATTR(batt_power_meter, S_IRUGO, htc_battery_show_cc_attr, NULL),
	__ATTR(capacity_raw, S_IRUGO, htc_battery_show_capacity_raw, NULL),
	__ATTR(batt_vol, S_IRUGO, htc_battery_batt_vol, NULL),
	__ATTR(charging_source, S_IRUGO, htc_battery_charging_source, NULL),
	__ATTR(batt_temp, S_IRUGO, htc_battery_temp, NULL),
	__ATTR(batt_state, S_IRUGO, htc_battery_state, NULL),
	__ATTR(usb_overheat, S_IRUGO, htc_battery_show_usb_overheat, NULL),
	__ATTR(charger_type, S_IRUGO, htc_charger_type, NULL),
	__ATTR(thermal_batt_temp, S_IRUGO, htc_thermal_batt_temp, NULL),
	__ATTR(htc_batt_data, S_IRUGO, htc_batt_bidata, NULL),
	__ATTR(consist_data, S_IRUGO, htc_consist_data, NULL),
	__ATTR(cycle_data, S_IRUGO, htc_cycle_data, NULL),
	__ATTR(batt_chked, S_IRUGO, htc_batt_check, NULL),
	__ATTR(batt_asp_set, S_IWUSR | S_IWGRP, NULL, htc_clr_cycle),
	__ATTR(thermal_limit_ma, S_IWUSR | S_IWGRP, NULL, htc_thermal_limnit_ma),
	__ATTR(aic_ready, S_IRUGO, htc_aic_ready, NULL),
	__ATTR(aic_data, S_IRUGO, htc_aic_data, NULL),
	__ATTR(aic_data1, S_IRUGO, htc_aic_data1, NULL),
	__ATTR(aic_data_set, S_IWUSR | S_IWGRP, NULL, htc_aic_data_set),
	__ATTR(aic_data_set1, S_IWUSR | S_IWGRP, NULL, htc_aic_data_set1),
	__ATTR(aic_data_request, S_IWUSR | S_IWGRP, NULL, htc_aic_data_request),
};

int htc_battery_create_attrs(struct device *dev)
{
    int i = 0, rc = 0;

    for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
        rc = device_create_file(dev, &htc_battery_attrs[i]);
        if (rc)
            goto htc_attrs_failed;
    }

    goto succeed;

htc_attrs_failed:
    while (i--)
        device_remove_file(dev, &htc_battery_attrs[i]);
succeed:
    return rc;
}

#if 0
void htc_dump_chg_reg(void)
{
	u8 chg_sts[11], chg_42, chg_61, chg_70;
	u8 otg_09, otg_10, bat_10;
	u8 usb_sts[11], usb_40, usb_5x[5], usb_6x[9], usb_70, usb_8x[5];
	u8 misc_sts[11];
	u16 idx;

	/* Dump for CHGR part */
	for (idx = 0; idx < sizeof(chg_sts); idx++)
		htc_batt_info.icharger->register_read(BATTERY_CHARGER_STATUS_1_REG + idx, &chg_sts[idx]);
	htc_batt_info.icharger->register_read(CHARGING_ENABLE_CMD_REG, &chg_42);
	htc_batt_info.icharger->register_read(FAST_CHARGE_CURRENT_CFG_REG, &chg_61);
	htc_batt_info.icharger->register_read(FLOAT_VOLTAGE_CFG_REG, &chg_70);

	/* Dump for OTG part */
	htc_batt_info.icharger->register_read(OTG_STATUS_REG, &otg_09);
	htc_batt_info.icharger->register_read((OTG_BASE + INT_RT_STS_OFFSET), &otg_10);

	/* Dump for BATIF part */
	htc_batt_info.icharger->register_read((BATIF_BASE + INT_RT_STS_OFFSET), &bat_10);

	/* Dump for USBIN part */
	for (idx = 0; idx < sizeof(usb_sts); idx++)
		htc_batt_info.icharger->register_read(USBIN_INPUT_STATUS_REG + idx, &usb_sts[idx]);
	htc_batt_info.icharger->register_read(USBIN_CMD_IL_REG, &usb_40);
	for (idx = 0; idx < sizeof(usb_5x); idx++)
		htc_batt_info.icharger->register_read(TYPE_C_CFG_REG + idx, &usb_5x[idx]);
	for (idx = 0; idx < sizeof(usb_6x); idx++)
		htc_batt_info.icharger->register_read(USBIN_ADAPTER_ALLOW_CFG_REG + idx, &usb_6x[idx]);
	htc_batt_info.icharger->register_read(USBIN_CURRENT_LIMIT_CFG_REG, &usb_70);
	for (idx = 0; idx < sizeof(usb_8x); idx++)
		htc_batt_info.icharger->register_read(USBIN_AICL_OPTIONS_CFG_REG + idx, &usb_8x[idx]);

	/* Dump for MISC part */
	for (idx = 0; idx < sizeof(misc_sts); idx++)
		htc_batt_info.icharger->register_read(TEMP_RANGE_STATUS_REG + idx, &misc_sts[idx]);

	BATT_DUMP("CHG[06:10]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],CHG[42,61,70]=[%02x,%02x,%02x]\n",
				chg_sts[0], chg_sts[1], chg_sts[2], chg_sts[3], chg_sts[4], chg_sts[5],
				chg_sts[6],chg_sts[7], chg_sts[8], chg_sts[9], chg_sts[10],
				chg_42, chg_61, chg_70);

	BATT_DUMP("USB[06:10]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],USB[40]=[%02x],USB[58:5B]=[%02x,%02x,%02x,%02x,%02x]\n",
				usb_sts[0], usb_sts[1], usb_sts[2], usb_sts[3], usb_sts[4], usb_sts[5],
				usb_sts[6], usb_sts[7], usb_sts[8], usb_sts[9], usb_sts[10],
				usb_40, usb_5x[0], usb_5x[1], usb_5x[2], usb_5x[3], usb_5x[4]);
	BATT_DUMP("USB[60:68]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],USB[70]=[%02x],USB[80:84]=[%02x,%02x,%02x,%02x,%02x]\n",
				usb_6x[0], usb_6x[1], usb_6x[2], usb_6x[3], usb_6x[4], usb_6x[5],
				usb_6x[6], usb_6x[7], usb_6x[8],
				usb_70, usb_8x[0], usb_8x[1], usb_8x[2], usb_8x[3], usb_8x[4]);

	BATT_DUMP("MISC[06:10]=[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x],OTG[09,10]=[%02x,%02x],BAT[10]=[%02x]\n",
				misc_sts[0], misc_sts[1], misc_sts[2], misc_sts[3], misc_sts[4], misc_sts[5],
				misc_sts[6],misc_sts[7], misc_sts[8], misc_sts[9], misc_sts[10],
				otg_09, otg_10, bat_10);

	return;
}
#endif

void htc_notify_unknown_charger(bool is_unknown)
{
	g_is_unknown_charger = is_unknown;
	if (is_unknown) {
		BATT_EMBEDDED("Unknown Charger detected");
		htc_batt_schedule_batt_info_update();
	}
}

void htc_battery_probe_process(enum htc_batt_probe probe_type) {

	static int s_probe_finish_process = 0;
	union power_supply_propval prop = {0,};
	int rc = 0, id_kohms = 0;
	struct device_node *node;
	struct device_node *profile_node;

	s_probe_finish_process++;
	BATT_LOG("Probe process: (%d, %d)\n", probe_type, s_probe_finish_process);

	if (s_probe_finish_process == BATT_PROBE_MAX) {

		htc_batt_info.batt_psy = power_supply_get_by_name("battery");
		htc_batt_info.bms_psy = power_supply_get_by_name("bms");
		htc_batt_info.usb_psy = power_supply_get_by_name("usb");
		htc_batt_info.main_psy = power_supply_get_by_name("main");

		if (g_test_power_monitor) {
			set_batt_psy_property(POWER_SUPPLY_PROP_CAPACITY, POWER_MONITOR_BATT_CAPACITY);
			htc_batt_info.rep.level = POWER_MONITOR_BATT_CAPACITY;
			htc_batt_info.rep.level_raw = POWER_MONITOR_BATT_CAPACITY;
		}

		id_kohms = get_property(
				htc_batt_info.bms_psy,
				POWER_SUPPLY_PROP_RESISTANCE_ID) / 1000;
		node = of_find_node_by_name(NULL, "qcom,battery-data");
		if (!node) {
			BATT_LOG("%s: No batterydata available\n", __func__);
		} else {
			profile_node = of_batterydata_get_best_profile(
						node, id_kohms, NULL);
			if (!profile_node) {
				BATT_LOG(
					"%s: couldn't find profile handle\n",
					__func__);
			} else {
				rc = of_property_read_u32(
						profile_node,
						"qcom,fastchg-current-ma",
						&htc_batt_info.batt_fcc_ma);
				if (rc < 0) {
					BATT_LOG(
						"%s: error reading qcom,fastchg-current-ma. %d\n",
						__func__, rc);
					htc_batt_info.batt_fcc_ma = 3000;
				}
			}
		}

		/* Check battery id */
		rc = power_supply_get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		if (rc) {
			BATT_ERR("Unable to read battery-type rc=%d\n", rc);
			htc_batt_info.rep.batt_id = 255;
		} else {
			if (strstr(prop.strval, "_id1"))
				htc_batt_info.rep.batt_id = 1;
			else if (strstr(prop.strval, "_id2"))
				htc_batt_info.rep.batt_id = 2;
			else if (strstr(prop.strval, "_semi"))
				htc_batt_info.rep.batt_id = 3;
			else
				htc_batt_info.rep.batt_id = 255;

			BATT_LOG("%s: catch name %s, set batt id=%d, fcc_ma=%d\n",
				__func__, prop.strval, htc_batt_info.rep.batt_id, htc_batt_info.batt_fcc_ma);
		}

		BATT_LOG("Probe process done.\n");
		g_htc_battery_probe_done = true;
	}
}

static struct htc_battery_platform_data htc_battery_pdev_data = {
        /* charger */
	.icharger.dump_all = charger_dump_all,
	.icharger.register_write = charger_register_write,
	.icharger.register_read = charger_register_read,
};

static void batt_regular_timer_handler(unsigned long data) {
	htc_batt_schedule_batt_info_update();
}

static enum alarmtimer_restart
batt_check_alarm_handler(struct alarm *alarm, ktime_t time)
{
	//BATT_LOG("alarm handler, but do nothing.");
	return 0;
}
static int htc_battery_prepare(struct device *dev)
{
	ktime_t interval;
	ktime_t next_alarm;
	struct timespec xtime;
	unsigned long cur_jiffies;
	s64 next_alarm_sec = 0;
	int check_time = 0;

	xtime = CURRENT_TIME;
	cur_jiffies = jiffies;
	htc_batt_timer.total_time_ms += (cur_jiffies -
			htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_timer.batt_system_jiffies = cur_jiffies;
	htc_batt_timer.batt_suspend_ms = xtime.tv_sec * MSEC_PER_SEC +
					xtime.tv_nsec / NSEC_PER_MSEC;

	if (suspend_highfreq_check_reason)
		check_time = BATT_SUSPEND_HIGHFREQ_CHECK_TIME;
	else
		check_time = BATT_SUSPEND_CHECK_TIME;

	interval = ktime_set(check_time - htc_batt_timer.total_time_ms / 1000, 0);
	next_alarm_sec = div_s64(interval.tv64, NSEC_PER_SEC);

	/* check if alarm is over time or in 1 second near future */
	if (next_alarm_sec <= 1) {
		BATT_LOG("%s: passing time:%lu ms, trigger batt_work immediately."
			"(suspend_highfreq_check_reason=0x%x)\n", __func__,
			htc_batt_timer.total_time_ms,
			suspend_highfreq_check_reason);
		htc_batt_schedule_batt_info_update();
		/* interval = ktime_set(check_time, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&htc_batt_timer.batt_check_wakeup_alarm,
					next_alarm, ktime_add(next_alarm, slack));
		*/
		return -EBUSY;
	}

	BATT_EMBEDDED("%s: passing time:%lu ms, alarm will be triggered after %lld sec."
		"(suspend_highfreq_check_reason=0x%x, htc_batt_info.state=0x%x)",
		__func__, htc_batt_timer.total_time_ms, next_alarm_sec,
		suspend_highfreq_check_reason, htc_batt_info.state);

	next_alarm = ktime_add(ktime_get_real(), interval);
	alarm_start(&htc_batt_timer.batt_check_wakeup_alarm, next_alarm);

	return 0;
}

static void htc_battery_complete(struct device *dev)
{
	struct timespec xtime;
	unsigned long resume_ms;
	unsigned long sr_time_period_ms;
	unsigned long check_time;
	int	batt_vol;

	xtime = CURRENT_TIME;
	htc_batt_timer.batt_system_jiffies = jiffies;
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	sr_time_period_ms = resume_ms - htc_batt_timer.batt_suspend_ms;
	htc_batt_timer.total_time_ms += sr_time_period_ms;

	BATT_EMBEDDED("%s: sr_time_period=%lu ms; total passing time=%lu ms.",
			__func__, sr_time_period_ms, htc_batt_timer.total_time_ms);

	if (suspend_highfreq_check_reason)
		check_time = BATT_SUSPEND_HIGHFREQ_CHECK_TIME * MSEC_PER_SEC;
	else
		check_time = BATT_SUSPEND_CHECK_TIME * MSEC_PER_SEC;

	check_time -= CHECH_TIME_TOLERANCE_MS;

	/*
	 * When kernel resumes, battery driver should check total time to
	 * decide if do battery information update or just ignore.
	 */
	batt_vol = get_property(htc_batt_info.bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)/1000;
	if ((htc_batt_timer.total_time_ms >= check_time) || (batt_vol < 3250)) {
		BATT_LOG("trigger batt_work while resume."
				"(suspend_highfreq_check_reason=0x%x, batt_vol=%d\n",
				suspend_highfreq_check_reason, batt_vol);
		htc_batt_schedule_batt_info_update();
	}

	return;
}

static struct dev_pm_ops htc_battery_pm_ops = {
	.prepare = htc_battery_prepare,
	.complete = htc_battery_complete,
};

#if defined(CONFIG_FB)
static void htc_battery_fb_register(struct work_struct *work)
{
        int ret = 0;

        BATT_LOG("%s in", __func__);
        htc_batt_info.fb_notif.notifier_call = fb_notifier_callback;
        ret = fb_register_client(&htc_batt_info.fb_notif);
        if (ret)
                BATT_ERR("[warning]:Unable to register fb_notifier: %d\n", ret);
}
#endif /* CONFIG_FB */

static int htc_battery_probe(struct platform_device *pdev)
{
	struct htc_battery_platform_data *pdata = pdev->dev.platform_data;

	htc_batt_info.icharger = &pdata->icharger;
	INIT_WORK(&htc_batt_timer.batt_work, batt_worker);
	INIT_DELAYED_WORK(&htc_batt_info.cable_impedance_work, cable_impedance_worker);
	INIT_DELAYED_WORK(&htc_batt_info.chg_full_check_work, chg_full_check_worker);
	INIT_DELAYED_WORK(&htc_batt_info.is_usb_overheat_work, is_usb_overheat_worker);
	INIT_DELAYED_WORK(&htc_batt_info.htc_usb_overheat_work, htc_usb_overheat_worker);
	INIT_DELAYED_WORK(&htc_batt_info.iusb_5v_2a_ability_check_work, htc_iusb_5v_2a_ability_check_work);
	INIT_DELAYED_WORK(&htc_batt_info.quick_charger_check_work, htc_quick_charger_check_work);
	INIT_DELAYED_WORK(&htc_batt_info.htcchg_init_work, htcchg_init_worker);
	INIT_DELAYED_WORK(&htc_batt_info.htcchg_vbus_adjust_work, htcchg_vbus_adjust_worker);
	INIT_DELAYED_WORK(&htc_batt_info.screen_ibat_limit_enable_work, screen_ibat_limit_enable_worker);
	init_timer(&htc_batt_timer.batt_timer);
	htc_batt_timer.batt_timer.function = batt_regular_timer_handler;
	alarm_init(&htc_batt_timer.batt_check_wakeup_alarm, ALARM_REALTIME,
			batt_check_alarm_handler);
	htc_batt_timer.batt_wq = create_singlethread_workqueue("batt_timer");

	htc_batt_timer.time_out = BATT_TIMER_UPDATE_TIME;
	batt_set_check_timer(htc_batt_timer.time_out);

#if defined(CONFIG_FB)
        htc_batt_info.batt_fb_wq = create_singlethread_workqueue("HTC_BATTERY_FB");
        if (!htc_batt_info.batt_fb_wq) {
                BATT_ERR("allocate batt_fb_wq failed\n");
        }
        INIT_DELAYED_WORK(&htc_batt_info.work_fb, htc_battery_fb_register);
        queue_delayed_work(htc_batt_info.batt_fb_wq, &htc_batt_info.work_fb, 0);
#endif /* CONFIG_FB */

	htc_battery_probe_process(HTC_BATT_PROBE_DONE);

	return 0;
}

static struct platform_device htc_battery_pdev = {
	.name = HTC_BATT_NAME,
	.id = -1,
	.dev    = {
		.platform_data = &htc_battery_pdev_data,
	},
};

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= HTC_BATT_NAME,
		.owner	= THIS_MODULE,
		.pm = &htc_battery_pm_ops,
	},
};

static int __init htc_battery_init(void)
{
	struct device_node *node;
	int ret = -EINVAL, i = 0;
	u32 val;
	struct property *prop;
	const __be32 *cur;

	wake_lock_init(&htc_batt_timer.battery_lock, WAKE_LOCK_SUSPEND, "htc_battery");
	wake_lock_init(&htc_batt_info.charger_exist_lock, WAKE_LOCK_SUSPEND,"charger_exist_lock");
	wake_lock_init(&htc_batt_info.check_overheat_lock, WAKE_LOCK_SUSPEND,"check_overheat_lock");

	htc_batt_info.k_debug_flag = get_kernel_flag();

	g_test_power_monitor =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_TEST_PWR_SUPPLY) ? 1 : 0;
	g_flag_keep_charge_on =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_KEEP_CHARG_ON) ? 1 : 0;
	g_flag_force_ac_chg =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_ENABLE_FAST_CHARGE) ? 1 : 0;
	g_flag_pa_fake_batt_temp =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_FOR_PA_TEST) ? 1 : 0;
	g_flag_disable_safety_timer =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_DISABLE_SAFETY_TIMER) ? 1 : 0;
	g_flag_disable_temp_protection =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_DISABLE_TBATT_PROTECT) ? 1 : 0;
	g_flag_enable_batt_debug_log =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG) ? 1 : 0;
	g_flag_ats_limit_chg =
		(htc_batt_info.k_debug_flag & KERNEL_FLAG_ATS_LIMIT_CHARGE) ? 1 : 0;

	/* init battery parameters. */
	htc_batt_info.rep.batt_vol = 4000;
	htc_batt_info.rep.batt_id = 1;
	htc_batt_info.rep.batt_temp = 280;
	htc_batt_info.rep.batt_current = 0;
	htc_batt_info.rep.charging_source = POWER_SUPPLY_TYPE_UNKNOWN;
	htc_batt_info.rep.level = 33;
	htc_batt_info.rep.level_raw = 33;
	htc_batt_info.rep.full_level = 100;
	htc_batt_info.rep.status = POWER_SUPPLY_STATUS_UNKNOWN;
	htc_batt_info.rep.full_level_dis_batt_chg = 100;
	htc_batt_info.rep.overload = 0;
	htc_batt_info.rep.over_vchg = 0;
	htc_batt_info.rep.is_full = false;
	htc_batt_info.rep.health = POWER_SUPPLY_HEALTH_UNKNOWN;
	htc_batt_info.smooth_chg_full_delay_min = 1;
	htc_batt_info.decreased_batt_level_check = 1;
	htc_batt_info.critical_low_voltage_mv = 3200;
	htc_batt_info.batt_full_voltage_mv = 4350;
	htc_batt_info.batt_full_current_ma = 300;
	htc_batt_info.overload_curr_thr_ma = 0;
	htc_batt_info.store.batt_stored_magic_num = 0;
	htc_batt_info.store.batt_stored_soc = 0;
	htc_batt_info.store.batt_stored_temperature = 0;
	htc_batt_info.store.batt_stored_update_time = 0;
	htc_batt_info.store.consistent_flag = false;
	htc_batt_info.vbus = 0;
	htc_batt_info.current_limit_reason = 0;
	htc_batt_info.chgr_stop_reason = 0;
	htc_batt_info.qc3_current_ua = HVDCP_3_CHARGE_CURR;
	htc_batt_info.batt_fcc_ma = 3000;
	htc_batt_info.fastchg_current_ma = 0;
	htc_batt_info.health_level = 0;
	htc_batt_info.batt_health_good = 0;
	htc_batt_info.allow_power_off_voltage = 3400;
	htc_batt_info.low_batt_check_soc_raw_threshold = 0;
	htc_batt_info.r_default_for_5v2a_pre_chk = R_DEFAULT;
	htc_batt_info.ai_charge_enable = 0;

	node = of_find_compatible_node(NULL, NULL, "htc,htc_battery_store");
	if (node) {
		if (!of_device_is_available(node)) {
			pr_err("%s: unavailable node.\n", __func__);
		} else {
			/* Read magic number */
			ret = of_property_read_u32(node, "htc,stored-batt-magic-num", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-magic-num.\n", __func__);
			else
				htc_batt_info.store.batt_stored_magic_num = val;
			/* Read store battery soc */
			ret = of_property_read_u32(node, "htc,stored-batt-soc", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-soc.\n", __func__);
			else
				htc_batt_info.store.batt_stored_soc = val;
			/* Read store battery temperature */
			ret = of_property_read_u32(node, "htc,stored-batt-temperature", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-temperature.\n", __func__);
			else
				htc_batt_info.store.batt_stored_temperature = val;
			/* Read store time */
			ret = of_property_read_u32(node, "htc,stored-batt-update-time", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-update-time.\n", __func__);
			else
				htc_batt_info.store.batt_stored_update_time = val;
			/* Read store battery total level raw */
			ret = of_property_read_u32(node, "htc,stored-batt-total-level", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-total-level.\n", __func__);
			else
				g_total_level_raw = val;
			/* Read store batteryoverheat time */
			ret = of_property_read_u32(node, "htc,stored-batt-overheat-sec", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-overheat-sec.\n", __func__);
			else
				g_overheat_55_sec = val;
			/* Read store battery first use time */
			ret = of_property_read_u32(node, "htc,stored-batt-first-use", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-first-use.\n", __func__);
			else
				g_batt_first_use_time = val;
			/* Read store battery checksum */
			ret = of_property_read_u32(node, "htc,stored-batt-checksum", &val);
			if (ret)
				pr_err("%s: error reading htc,stored-batt-checksum.\n", __func__);
			else
				g_batt_cycle_checksum = val;
		}
	} else {
		pr_err("%s: can't find compatible 'htc,htc_battery_store'\n", __func__);
	}

	node = of_find_compatible_node(NULL, NULL, "qcom,qpnp-smb2");
	if (node) {
		if (!of_device_is_available(node)) {
			pr_err("%s: unavailable node.\n", __func__);
		} else {
			/* Read QC3 current */
			ret = of_property_read_u32(node, "htc,qc3-curr-limit-ma", &val);
			if (ret)
				pr_err("%s: error reading htc,qc3-curr-limit-ma.\n", __func__);
			else
				htc_batt_info.qc3_current_ua = val * 1000;
		}
	} else {
		pr_err("%s: can't find compatible 'qcom,qpnp-smb2'\n", __func__);
	}

	node = of_find_compatible_node(NULL, NULL, "htc,htc_battery_dt");
	if (node) {
		if (!of_device_is_available(node)) {
			pr_err("%s: unavailable node.\n", __func__);
		} else {
			ret = of_property_read_u32(node, "htc,batt-full-current-ma", &val);
			if (ret)
				pr_err("%s: error reading htc,batt-full-current-ma.\n", __func__);
			else
				htc_batt_info.batt_full_current_ma = val;

			ret = of_property_read_u32(node, "htc,fastchg-current-ma", &val);
			if (ret)
				pr_err("%s: error reading htc,fastchg-current-ma.\n", __func__);
			else {
				// read ibat_max for thermal table
				htc_batt_info.fastchg_current_ma = val;
				// read health level
				ret = of_property_read_u32(node, "htc,health-level", &val);
				if (ret)
					pr_err("%s: error reading htc,health-level.\n", __func__);
				else
					htc_batt_info.health_level = val;
				// read the idx for HEALTH_GOOD
				ret = of_property_read_u32(node, "htc,batt-health-good", &val);
				if (ret)
					 pr_err("%s: error reading htc,batt-health-good.\n", __func__);
				else
					htc_batt_info.batt_health_good = val;
				// read 5v thermal table
				ret = of_property_read_u32_array(node, "htc,chg-fcc-limits",
								 &ibat_map_5v[0][0], MAX_IDX * HEALTH_LEVELS);
				if (ret)
					pr_err("%s: error reading 5v thermal table.\n", __func__);
				// read qc2 thermal table
				ret = of_property_read_u32_array(node, "htc,qc2-chg-fcc-limits",
								 &ibat_map_qc2[0][0], MAX_IDX * HEALTH_LEVELS);
				if (ret)
					pr_err("%s: error reading qc2 thermal table.\n", __func__);
				// read qc3 thermal table
				ret = of_property_read_u32_array(node, "htc,qc3-chg-fcc-limits",
								 &ibat_map_qc3[0][0], MAX_IDX * HEALTH_LEVELS);
				if (ret)
					pr_err("%s: error reading qc3 thermal table.\n", __func__);
				// read vbat threshold
				ret = of_property_read_u32_array(node, "htc,chg-vbatt-thresholds",
								 &thermal_limit_vol[0], HEALTH_LEVELS);
				if (ret)
					pr_err("%s: error reading htc,chg-vbatt-thresholds.\n", __func__);
				// read vbat recover threshold
				ret = of_property_read_u32_array(node, "htc,chg-vbatt-recover-thresholds",
								 &thermal_limit_vol_recover[0], HEALTH_LEVELS);
				if (ret)
					pr_err("%s: error reading htc,chg-vbatt-recover-thresholds.\n", __func__);
				// read temp threshold
				i = 0;
				of_property_for_each_u32(node, "htc,chg-temp-thresholds", prop, cur, val) {
					if (i / 2 >= HEALTH_LEVELS)
						break;
					switch (i % 2) {
					case 0:
						thermal_stage[i / 2].nextTemp = val;
						break;
					case 1:
						thermal_stage[i / 2].recoverTemp = val;
						break;
					}
					i++;
				}
			}

			ret = of_property_read_u32(node, "htc,allow-power-off-voltage", &val);
			if (ret)
				pr_err("%s: error reading htc,allow-power-off-voltage.\n", __func__);
			else
				htc_batt_info.allow_power_off_voltage = val;

			ret = of_property_read_u32(node, "htc,low-batt-check-soc-threshold", &val);
			if (ret)
				pr_err("%s: error reading htc,low-batt-check-soc-threshold.\n", __func__);
			else
				htc_batt_info.low_batt_check_soc_raw_threshold = val;

			ret = of_property_read_u32(node, "htc,decreased-batt-level-check", &val);
			if (ret)
				pr_err("%s: error reading htc,decreased-batt-level-check.\n", __func__);
			else
				htc_batt_info.decreased_batt_level_check = val;

			ret = of_property_read_u32(node, "htc,r-default-for-5v2a-pre-chk", &val);
			if (ret)
				pr_err("%s: error reading htc,r-default-for-5v2a-pre-chk.\n", __func__);
			else
				htc_batt_info.r_default_for_5v2a_pre_chk = val;
			ret = of_property_read_u32(node, "htc,ai-charge-enable", &val);
			if (ret)
				pr_err("%s: error reading htc,ai-charge-enable.\n", __func__);
			else
				htc_batt_info.ai_charge_enable = val;
		}
	}
	platform_device_register(&htc_battery_pdev);
	platform_driver_register(&htc_battery_driver);

	/* Modify full level for ATS test */
	if (g_flag_ats_limit_chg)
		htc_batt_info.rep.full_level = 50;

	BATT_LOG("htc_battery_init done.\n");

	return 0;
}

module_init(htc_battery_init);

