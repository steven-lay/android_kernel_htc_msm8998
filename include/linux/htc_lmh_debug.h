/*
 * HTC lmh debug Header
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

struct cpu_freq_table {
	int cpu_freq_table[CONFIG_NR_CPUS][64];
	int cpu_freq_steps_num[CONFIG_NR_CPUS];
	int max_cpu_freq[CONFIG_NR_CPUS];
};

extern struct cpu_freq_table cpu_freq_table_info;
