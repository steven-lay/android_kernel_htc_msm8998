#define CONFIG_HTC_BATT_PCN0021 //Power team monitor battery charge/discharge

#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/wakelock.h>
#include <linux/module.h>
#include <linux/htc_flags.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define BATT_LOG(x...) do { \
printk(KERN_INFO "[BATT] " x); \
} while (0)

#define BATT_DUMP(x...) do { \
	printk(KERN_ERR "[BATT][DUMP] " x); \
} while (0)

#define BATT_ERR(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] err:" x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_EMBEDDED(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define POWER_MONITOR_BATT_CAPACITY	77
#define POWER_MONITOR_BATT_TEMP	330

/* stored consistent parameters */
#define STORE_MAGIC_NUM          0xDDAACC00
#define STORE_MAGIC_OFFSET       3104    /*0xC20*/
#define STORE_SOC_OFFSET         3108    /*0xC24*/
#define STORE_CURRTIME_OFFSET    3120    /*0xC30*/
#define STORE_TEMP_OFFSET		3140    /*0xC44*/

/* for batt cycle info */
#define HTC_BATT_TOTAL_LEVELRAW		3144
#define HTC_BATT_OVERHEAT_MSEC		3148
#define HTC_BATT_FIRST_USE_TIME		3152
#define HTC_BATT_CYCLE_CHECKSUM		3156

/* for htc_extension */
#define HTC_EXT_UNKNOWN_USB_CHARGER		(1<<0)
#define HTC_EXT_CHG_UNDER_RATING		(1<<1)
#define HTC_EXT_CHG_SAFTY_TIMEOUT		(1<<2)
#define HTC_EXT_CHG_FULL_EOC_STOP		(1<<3)
#define HTC_EXT_BAD_CABLE_USED			(1<<4)
#define HTC_EXT_QUICK_CHARGER_USED		(1<<5)
#define HTC_EXT_USB_OVERHEAT				(1<<6)
#define HTC_EXT_AI_CHARGING			(1<<8)

#define BATT_TIMER_UPDATE_TIME				(60)
#define BATT_SUSPEND_CHECK_TIME				(3600)
#define BATT_SUSPEND_HIGHFREQ_CHECK_TIME	(300)
#define BATT_TIMER_CHECK_TIME				(360)
#define CHECH_TIME_TOLERANCE_MS	(1000)

/* for suspend high frequency (5min) */
#define SUSPEND_HIGHFREQ_CHECK_BIT_TALK		(1)
#define SUSPEND_HIGHFREQ_CHECK_BIT_SEARCH	(1<<1)
#define SUSPEND_HIGHFREQ_CHECK_BIT_MUSIC	(1<<3)

struct battery_info_reply {
	u32 batt_vol;
	u32 batt_id;
	s32 batt_temp;
	s32 batt_current;
	u32 charging_source;
	u32 level;
	u32 level_raw;
	u32 full_level;
	u32 status;
	u32 chg_src;
	u32 chg_en;
	u32 chg_batt_en;
	u32 full_level_dis_batt_chg;
	u32 overload;
	u32 over_vchg;
	u32 health;
	bool is_full;
};

struct battery_info_previous {
	s32 batt_temp;
	u32 charging_source;
	u32 level;
	u32 level_raw;
};

struct htc_battery_store {
	u32 batt_stored_magic_num;
	u32 batt_stored_soc;
	u32 batt_stored_temperature;
	unsigned long batt_stored_update_time;
	u32 consistent_flag;
};

struct htc_thermal_stage {
	int nextTemp;
	int recoverTemp;
};

struct htc_charger {
	int (*dump_all)(void);
	int (*register_write)(u16 addr, u8 mask, u8 val);
	int (*register_read)(u16 addr, u8 *val);
//	int (*get_attr_text)(char *buf, int size);
//	int (*is_battery_full_eoc_stop)(int *result);
};

struct htc_battery_info {
	struct battery_info_reply rep;
	struct battery_info_previous prev;
	struct htc_battery_store store;
	struct htc_charger *icharger;
	struct power_supply		*batt_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*main_psy;
	int critical_low_voltage_mv;
	int smooth_chg_full_delay_min;
	int decreased_batt_level_check;
	int batt_full_voltage_mv;
	int batt_full_current_ma;
	int overload_curr_thr_ma;
	int batt_fcc_ma;
	struct wake_lock charger_exist_lock;
	struct wake_lock check_overheat_lock;
	struct delayed_work chg_full_check_work;
	struct delayed_work is_usb_overheat_work;
	struct delayed_work cable_impedance_work;
	struct delayed_work htc_usb_overheat_work;
	struct delayed_work iusb_5v_2a_ability_check_work;
	struct delayed_work quick_charger_check_work;
	struct delayed_work htcchg_init_work;
	struct delayed_work htcchg_vbus_adjust_work;
	struct delayed_work screen_ibat_limit_enable_work;
	int state;
	int vbus;
	int k_debug_flag;
	int current_limit_reason;
	int chgr_stop_reason;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *batt_fb_wq;
	struct delayed_work work_fb;
#endif
	unsigned int htc_extension;	/* for htc in-house sw */
	int qc3_current_ua;
	int fastchg_current_ma;
	int health_level;
	int batt_health_good;
	int allow_power_off_voltage;
	int low_batt_check_soc_raw_threshold;
	int r_default_for_5v2a_pre_chk;
	int ai_charge_enable;
};

struct htc_battery_timer {
	unsigned long batt_system_jiffies;
	unsigned long total_time_ms;	/* since last do batt_work */
	unsigned long batt_suspend_ms;
	struct alarm batt_check_wakeup_alarm;
	struct work_struct batt_work;
	struct timer_list batt_timer;
	struct workqueue_struct *batt_wq;
	struct wake_lock battery_lock;
	unsigned int time_out;
};

struct htc_battery_platform_data {
        struct htc_charger icharger;
};

#ifdef CONFIG_HTC_BATT_PCN0021
struct htc_charging_statistics {
        unsigned long begin_chg_time;
        unsigned long end_chg_time;
        int begin_chg_batt_level;
        int end_chg_batt_level;
};

struct htc_statistics_category {
        unsigned long chg_time_sum;
        unsigned long dischg_time_sum;
        int sample_count;
};
#endif //CONFIG_HTC_BATT_PCN0021

enum charger_control_flag {
	STOP_CHARGER = 0,
	ENABLE_CHARGER,
	ENABLE_LIMIT_CHARGER,
	DISABLE_LIMIT_CHARGER,
	DISABLE_PWRSRC,
	ENABLE_PWRSRC,
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
	DISABLE_PWRSRC_FINGERPRINT,
	ENABLE_PWRSRC_FINGERPRINT,
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
	END_CHARGER
};

/*
 * MFG ftm mode charger control
 *
 * FTM_ENABLE_CHARGER: default, ftm control disabled
 * FTM_STOP_CHARGER: ftm control to disable charging
 * FTM_FAST_CHARGE: ftm control to force fast charge
 * FTM_SLOW_CHARGE: ftm control to force slow charge
 * FTM_END_CHARGER: do nothing, value for flag out of bound check
 */
enum ftm_charger_control_flag {
	FTM_ENABLE_CHARGER = 0,
	FTM_STOP_CHARGER,
	FTM_FAST_CHARGE,
	FTM_SLOW_CHARGE,
	FTM_END_CHARGER
};

enum user_set_input_current {
	SLOW_CHARGE_CURR = 500000,
	FAST_CHARGE_CURR = 900000,
	WALL_CHARGE_CURR = 1500000,
	HVDCP_CHARGE_CURR = 1600000,
	HVDCP_3_CHARGE_CURR = 2200000,
	DCP_5V2A_CHARGE_CURR = 2000000,
	OTHER_CHARG_CURR = 1500000,
	HTCCHG_INIT_CURR = 100000,
};

enum htc_batt_probe {
	CHARGER_PROBE_DONE,
	GAUGE_PROBE_DONE,
	HTC_BATT_PROBE_DONE,
	BATT_PROBE_MAX,
};
enum htc_charger_request {
	CHARGER_ABILITY_DETECT_DONE,
	CHARGER_5V_2A_DETECT_DONE,
};


enum htc_chr_type_data {
	DATA_NO_CHARGING = 0,
	DATA_UNKNOWN_CHARGER,
	DATA_UNKNOWN_TYPE,
	DATA_USB,
	DATA_USB_CDP,
	DATA_AC,
	DATA_QC2,
	DATA_QC3,
	DATA_PD_5V,
	DATA_PD_9V,
	DATA_PD_12V,
	DATA_TYPE_C
};

static const struct qpnp_vadc_map_pt usb_conn_temp_adc_map[] = {
        {-200, 1668},
        {-190, 1659},
        {-180, 1651},
        {-170, 1641},
        {-160, 1632},
        {-150, 1622},
        {-140, 1611},
        {-130, 1600},
        {-120, 1589},
        {-110, 1577},
        {-100, 1565},
        {-90, 1552},
        {-80, 1539},
        {-70, 1525},
        {-60, 1511},
        {-50, 1496},
        {-40, 1481},
        {-30, 1466},
        {-20, 1449},
        {-10, 1433},
        {0, 1416},
        {10, 1398},
        {20, 1381},
        {30, 1362},
        {40, 1344},
        {50, 1325},
        {60, 1305},
        {70, 1286},
        {80, 1266},
        {90, 1245},
        {100, 1225},
        {110, 1204},
        {120, 1183},
        {130, 1161},
        {140, 1140},
        {150, 1118},
        {160, 1096},
        {170, 1075},
        {180, 1053},
        {190, 1031},
        {200, 1009},
        {210, 987},
        {220, 965},
        {230, 943},
        {240, 922},
        {250, 900},
        {260, 879},
        {270, 857},
        {280, 836},
        {290, 815},
        {300, 795},
        {310, 774},
        {320, 754},
        {330, 734},
        {340, 715},
        {350, 695},
        {360, 677},
        {370, 658},
        {380, 640},
        {390, 622},
        {400, 604},
        {410, 587},
        {420, 570},
        {430, 554},
        {440, 537},
        {450, 522},
        {460, 506},
        {470, 491},
        {480, 477},
        {490, 462},
        {500, 449},
        {510, 435},
        {520, 422},
        {530, 409},
        {540, 397},
        {550, 385},
        {560, 373},
        {570, 361},
        {580, 350},
        {590, 339},
        {600, 329},
        {610, 319},
        {620, 309},
        {630, 300},
        {640, 290},
        {650, 281},
        {660, 273},
        {670, 264},
        {680, 256},
        {690, 248},
        {700, 241},
        {710, 233},
        {720, 226},
        {730, 219},
        {740, 212},
        {750, 206},
        {760, 200},
        {770, 193},
        {780, 188},
        {790, 182},
        {800, 176},
        {810, 171},
        {820, 166},
        {830, 161},
        {840, 156},
        {850, 151},
        {860, 147},
        {870, 142},
        {880, 138},
        {890, 134},
        {900, 130},
        {910, 126},
        {920, 123},
        {930, 119},
        {940, 116},
        {950, 112},
        {960, 109},
        {970, 106},
        {980, 103},
        {990, 100},
        {1000, 97},
        {1010, 94},
        {1020, 91},
        {1030, 89},
        {1040, 86},
        {1050, 84},
        {1060, 82},
        {1070, 79},
        {1080, 77},
        {1090, 75},
        {1100, 73},
        {1110, 71},
        {1120, 69},
        {1130, 67},
        {1140, 65},
        {1150, 64},
        {1160, 62},
        {1170, 60},
        {1180, 59},
        {1190, 57},
        {1200, 56},
        {1210, 54},
        {1220, 53},
        {1230, 51},
        {1240, 50},
        {1250, 49}
};


int htc_battery_create_attrs(struct device *dev);
void htc_battery_info_update(enum power_supply_property prop, int intval);
void htc_battery_probe_process(enum htc_batt_probe probe_type);
int htc_battery_level_adjust(void);
int htc_battery_charger_switch_internal(int enable);
void htc_5v2a_pre_chk(void);
void htc_dump_chg_reg(void);
void htc_notify_unknown_charger(bool is_unknown);
bool htc_battery_get_discharging_reason(void);
void htc_ftm_disable_charger(bool disable);
bool htc_get_htcchg_sts(void);

int charger_dump_all(void);
int charger_register_write(u16 addr, u8 mask, u8 val);
int charger_register_read(u16 addr, u8 *val);
int smblib_typec_first_debounce_result(void);

bool is_cool_charger(void);
int pmic_bob_regulator_pwm_mode_enable(bool enable);
