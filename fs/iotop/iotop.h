/*
 * iotop.h - Using kernel thread to print the process of top 5 read/write throughput.
 *
 * Copyright (C) HTC Corporation.
 */
#ifndef _IOTOP
#define _IOTOP

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/htc_flags.h>
#include <linux/list_sort.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/delay.h>

#define IOTOP_INTERVAL			4500
#define IOREAD_DUMP_THRESHOLD		10485760 /* 10MB */
#define IOWRITE_DUMP_THRESHOLD		1048576  /*  1MB */
#define IOREAD_DUMP_TOTAL_THRESHOLD	10485760 /* 10MB */
#define IOWRITE_DUMP_TOTAL_THRESHOLD	10485760 /* 10MB */

struct io_account {
	char task_name[TASK_COMM_LEN];
	char gtask_name[TASK_COMM_LEN]; /* group leader */
	char ptask_name[TASK_COMM_LEN]; /* parnet */
	unsigned int pid;
	unsigned int tgid;
	unsigned int ppid;
	u64 io_amount;
	struct list_head list;
};

static LIST_HEAD(ioread_list);
static LIST_HEAD(iowrite_list);
static spinlock_t iolist_lock;
static unsigned long jiffies_next_iotop = 0;

static struct task_struct * _task;
static int iotop_init_finish = 0;

#endif	/* _IOTOP */
