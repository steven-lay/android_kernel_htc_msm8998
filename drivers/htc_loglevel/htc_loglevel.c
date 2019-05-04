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
//#include <mach/board.h>
#include <linux/seq_file.h>

#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/of.h>

mm_segment_t oldfs;

#define PROCNAME "driver/hdf"
#define FLAG_LEN 64
#define DEVICE_TREE_MISC_PATH "/chosen/misc"
#define ENABLE_LOG_PROPERTY "enable_log"

#if 0
#define SECMSG(s...) pr_info("[SECURITY] "s)

#else
#define SECMSG(s...) do{} while(0)
#endif

static char htc_debug_flag[FLAG_LEN+1]={0};
static int offset=2676;
static int first_read=1;
/**
MTK  LK  MISC_ENABLE_LOG_OFFSET :: 148084  = 147456 + 628
QCT  LK  MISC_ENABLE_LOG_OFFSET ::   2676  =  2048  + 628
QCT  HB  MISC_ENABLE_LOG_OFFSET ::    628  =     0  + 628

**/

static int htc_debug_read(struct seq_file *m, void *v)
{
    char RfMisc[FLAG_LEN+3]={0};
    struct file *filp = NULL;
    ssize_t nread;

    if(first_read){

        filp = filp_open("dev/block/bootdevice/by-name/misc", O_RDWR, 0);
        if (IS_ERR(filp)) {
            printk(KERN_ERR"read: unable to open file: dev/block/bootdevice/by-name/misc");
            return PTR_ERR(filp);
        }

        SECMSG("read: %s: offset :%d\n", __func__, offset);
        filp->f_pos = offset;

        nread = kernel_read(filp, filp->f_pos, RfMisc, FLAG_LEN+2);

        memset(htc_debug_flag,0,FLAG_LEN+1);
        memcpy(htc_debug_flag,RfMisc+2,FLAG_LEN);//RfMisc will have two bytes prefix "0x"

        SECMSG("read: %s: RfMisc        :%s (%zd)\n", __func__,RfMisc, nread);
        SECMSG("read: %s: htc_debug_flag:%s \n", __func__, htc_debug_flag);

        seq_printf(m, "0X%s\n",htc_debug_flag);

        if (filp)
            filp_close(filp, NULL);

        first_read = 0;
    }else{
        seq_printf(m, "0X%s\n",htc_debug_flag);
    }
    return 0;
}

static ssize_t htc_debug_write(struct file *file, const char __user *buffer,
                        size_t count, loff_t *ppos)
{
    char buf[FLAG_LEN+3];
    struct file *filp = NULL;
    ssize_t nread;

    SECMSG("write: %s called (count:%d)\n", __func__, (int)count);
    if ((count != FLAG_LEN + 2) && (count != FLAG_LEN + 3)) {
        // +2 for prefix "0x", +3 for prefix "0x" and suffix "\0" or "\n"
        printk(KERN_ERR"write: count:%d != buf\n",(int)count);
        return -EFAULT;
    }

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;

    memset(htc_debug_flag,0,FLAG_LEN+1);
    memcpy(htc_debug_flag,buf+2,FLAG_LEN);//buf will have two bytes prefix "0x"

    //printk(KERN_ERR"write: Receive :%s\n",buf);
    //printk(KERN_ERR"write: Flag    :%s\n",htc_debug_flag);

    filp = filp_open("dev/block/bootdevice/by-name/misc", O_RDWR, 0);
    if (IS_ERR(filp)) {
        printk(KERN_ERR"write: unable to open file: dev/block/bootdevice/by-name/misc\n");
        return PTR_ERR(filp);
    }

    filp->f_pos = offset;
    nread = kernel_write(filp, buf, FLAG_LEN+2, filp->f_pos);//Need to write two bytes prefix "0x" to misc

    if (filp)
        filp_close(filp, NULL);

    return count;
}

static int htc_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, htc_debug_read, NULL);
}

static const struct file_operations htc_debug_fops = {
    .owner      = THIS_MODULE,
    .open       = htc_debug_open,
    .write      = htc_debug_write,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static void init_from_device_tree(void)
{
    struct device_node *misc_node;
    char *data;
    int property_size;
    misc_node = of_find_node_by_path(DEVICE_TREE_MISC_PATH);

    if(NULL == misc_node)
        return;

	pr_info("/chosen/misc found name: %s\n", misc_node->name);

    data = (char *) of_get_property(misc_node, ENABLE_LOG_PROPERTY, &property_size);
	if(property_size < FLAG_LEN || data== NULL){
		pr_info("use \"misc\" of_get_property failed ! \n");
		return;
	}

    pr_info("%s - loglevel: %s\n", __func__, data);

    memset(htc_debug_flag, 0, (FLAG_LEN + 1));
    memcpy(htc_debug_flag, data, FLAG_LEN);

    //clear first_read to prevent from reading eMMC
    first_read = 0;
}

static int __init sysinfo_proc_init(void)
{
    struct proc_dir_entry *entry = NULL;

    pr_info("%s: Init HTC Debug Flag proc interface.\r\n", __func__);

    init_from_device_tree();

    /* NOTE: kernel 3.10 use proc_create_data to create /proc file node */
    entry = proc_create_data(PROCNAME, 0660, NULL, &htc_debug_fops, NULL);//registering functions
    if (entry == NULL) {
        printk(KERN_ERR "%s: unable to create /proc%s entry\n", __func__,PROCNAME);
        return -ENOMEM;
    }

    return 0;
}

module_init(sysinfo_proc_init);
MODULE_AUTHOR("HTC");
MODULE_DESCRIPTION("HTC LogLevel Interface");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
