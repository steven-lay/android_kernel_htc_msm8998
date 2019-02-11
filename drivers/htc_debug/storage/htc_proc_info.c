/*
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>

int cancel_fsync = 0;

static ssize_t htc_cancel_fsync_proc_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *ppos)
{
	int val;
	char buf[64];

        if (copy_from_user(buf, buffer, 64))
                return -EFAULT;

	sscanf(buf, "%d", &val);

	if (val == 1) {
		pr_info("Cancel fsync.\n");
		cancel_fsync = 1;
	} else if (val == 0) {
		pr_info("Stop canceling fsync.\n");
		cancel_fsync = 0;
	}

	return count;
}

static int htc_cancel_fsync_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d", cancel_fsync);
	return 0;
}

static int htc_cancel_fsync_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, htc_cancel_fsync_read, NULL);
}

static const struct file_operations htc_cancel_fsync_fops = {
	.open           = htc_cancel_fsync_proc_open,
	.write          = htc_cancel_fsync_proc_write,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init sysinfo_proc_init(void)
{
	struct proc_dir_entry *entry = NULL;

	pr_info("%s: Init HTC system info proc interface.\r\n", __func__);

	entry = proc_create_data("cancel_fsync", 0644, NULL, &htc_cancel_fsync_fops, NULL);
	if (entry == NULL)
		pr_info(KERN_ERR "%s: unable to create /proc entry of cancel_fsync\n", __func__);
	cancel_fsync = 0;

	return 0;
}

module_init(sysinfo_proc_init);
MODULE_DESCRIPTION("HTC Proc Info Interface");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
