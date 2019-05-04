/*
 * iotop.c - Using kernel thread to print the process of top 5 read/write throughput.
 *
 * Copyright (C) HTC Corporation.
 */
#include "iotop.h"

static int iotop_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct io_account *ioa = container_of(a, struct io_account, list);
	struct io_account *iob = container_of(b, struct io_account, list);

	return !(ioa->io_amount > iob->io_amount);
}

void collect_io_stats(size_t rw_bytes, int type)
{
	struct task_struct *process = current;
	struct io_account *io_act, *tmp;
	int found;
	struct list_head *io_list;
	unsigned long flags;

	if (!iotop_init_finish || IS_ERR(_task))
		return;
	if (get_tamper_sf() == 1)
		return;

	if (!rw_bytes)
		return;

	if (!strcmp(current->comm, "sdcard"))
		return;

	if (type == READ)
		io_list = &ioread_list;
	else if (type == WRITE)
		io_list = &iowrite_list;
	else
		return;

	found = 0;
	spin_lock_irqsave(&iolist_lock, flags);
	list_for_each_entry_safe(io_act, tmp, io_list, list) {
		if ((process->pid == io_act->pid) && !strcmp(process->comm, io_act->task_name)) {
			io_act->io_amount += rw_bytes;
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&iolist_lock, flags);

	if (!found) {
		io_act = kmalloc(sizeof(struct io_account), GFP_ATOMIC);
		if (io_act) {
			snprintf(io_act->task_name, sizeof(io_act->task_name), "%s", process->comm);
			io_act->pid = process->pid;
			io_act->tgid = process->tgid;
			if (process->group_leader)
				snprintf(io_act->gtask_name, sizeof(io_act->gtask_name), "%s",
					process->group_leader->comm);
			if (process->parent) {
				snprintf(io_act->ptask_name, sizeof(io_act->ptask_name), "%s",
					process->parent->comm);
				io_act->ppid = process->parent->pid;
			}
			io_act->io_amount = rw_bytes;
			spin_lock_irqsave(&iolist_lock, flags);
			list_add(&io_act->list, io_list);
			spin_unlock_irqrestore(&iolist_lock, flags);
		}
	}
}
EXPORT_SYMBOL(collect_io_stats);

static void show_iotop(void)
{
	struct io_account *io_act, *tmp;
	int i = 0;
	unsigned int task_cnt = 0;
	unsigned long long total_bytes;
	unsigned long flags;

	if (!iotop_init_finish || IS_ERR(_task))
		return;
	if (get_tamper_sf() == 1)
		return;

	spin_lock_irqsave(&iolist_lock, flags);
	list_sort(NULL, &ioread_list, iotop_cmp);
	total_bytes = 0;
	list_for_each_entry_safe(io_act, tmp, &ioread_list, list) {
		list_del_init(&io_act->list);
		if (i++ < 5 && io_act->io_amount > IOREAD_DUMP_THRESHOLD)
			pr_info("[READ IOTOP%d] %s(pid %u, tgid %u(%s), ppid %u(%s)): %llu KB\n",
				i, io_act->task_name, io_act->pid, io_act->tgid, io_act->gtask_name,
				io_act->ppid, io_act->ptask_name, io_act->io_amount / 1024);
		task_cnt++;
		total_bytes += io_act->io_amount;
		kfree(io_act);
	}
	if (total_bytes > IOREAD_DUMP_TOTAL_THRESHOLD)
		pr_info("[IOTOP] READ total %u tasks, %llu KB\n", task_cnt, total_bytes / 1024);

	list_sort(NULL, &iowrite_list, iotop_cmp);
	i = 0;
	total_bytes = 0;
	task_cnt = 0;
	list_for_each_entry_safe(io_act, tmp, &iowrite_list, list) {
		list_del_init(&io_act->list);
		if (i++ < 5 && io_act->io_amount >= IOWRITE_DUMP_THRESHOLD)
			pr_info("[WRITE IOTOP%d] %s(pid %u, tgid %u(%s), ppid %u(%s)): %llu KB\n",
				i, io_act->task_name, io_act->pid, io_act->tgid, io_act->gtask_name,
				io_act->ppid, io_act->ptask_name, io_act->io_amount / 1024);
		task_cnt++;
		total_bytes += io_act->io_amount;
		kfree(io_act);
	}
	spin_unlock_irqrestore(&iolist_lock, flags);
	if (total_bytes > IOWRITE_DUMP_TOTAL_THRESHOLD)
		pr_info("[IOTOP] WRITE total %u tasks, %llu KB\n", task_cnt, total_bytes / 1024);
}

static int iotop_pull_thread(void *d)
{
	while(!kthread_should_stop())
	{
		msleep(5000);
		if (!jiffies_next_iotop || time_after(jiffies, jiffies_next_iotop)) {
			jiffies_next_iotop = jiffies + msecs_to_jiffies(IOTOP_INTERVAL);
			show_iotop();
		}
	}

	pr_info("IOTOP: pulling thread end!\n");
	iotop_init_finish = 0;
	return 0;
}

static int __init iotop_init(void)
{
	pr_info("IOTOP: module init.\n");

	_task = kthread_run(iotop_pull_thread, NULL, "iotop_fn");
	if (IS_ERR(_task)) {
		pr_info("IOTOP: create kthread failed!\n");
	} else {
		pr_info("IOTOP: create kthread success!\n");
		spin_lock_init(&iolist_lock);
		iotop_init_finish = 1;
	}

	return 0;
}

static void __exit iotop_exit(void)
{
	pr_info("IOTOP: module exit!\n");
	iotop_init_finish = 0;

	if (!IS_ERR(_task)) {
		int ret = kthread_stop(_task);
		pr_info("thread function has run %ds\n", ret);
	}
}

module_init(iotop_init);
module_exit(iotop_exit);
