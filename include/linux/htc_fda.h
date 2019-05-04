/* include/linux/htc_fda.h
 *
 * FDA (Fail Detect Algorithm)
 *
 * Copyright (C) 2015 HTC Corporation
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

#ifndef __HTC_FDA_H
#define __HTC_FDA_H

/* Performance maintained log trigger */
#define fda_log_cpu(fmt, ...)	pr_err("[Perf_FDA][CPU] " fmt, ##__VA_ARGS__)
#define fda_log_pnp(fmt, ...)	pr_err("[Perf_FDA][PNP] " fmt, ##__VA_ARGS__)
#define fda_log_sched(fmt, ...)	pr_err("[Perf_FDA][SCHED] " fmt, ##__VA_ARGS__)

#endif

