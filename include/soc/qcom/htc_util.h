#ifndef __HTC_UTIL_H__
#define __HTC_UTIL_H__

#define k_pr_empty()
#define k_pr_info(args...)  is_commercial() ? k_pr_empty() : pr_info(args)
#define k_pr_embedded(args...)  pr_err(args)
#define k_pr_embedded_cond(cond, args...)  cond ? k_pr_embedded(args) : is_commercial() ? k_pr_empty() : k_pr_embedded(args)
#define safe_strcat(output, str)  strncat(output, str, sizeof(output) - strlen(output) - 1)

void htc_pm_monitor_init(void);
void htc_monitor_init(void);
void htc_idle_stat_add(int sleep_mode, u32 time);
bool is_commercial(void);
#endif
