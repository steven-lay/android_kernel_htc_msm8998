/* drivers/htc_ril/htc_port_list.c
 * Copyright (C) 2009-2015 HTC Corporation.
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

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/list.h>
#include <net/tcp.h>
#include <linux/htc_flags.h>
#include <soc/qcom/smem.h>

//Sync from mach/msm_iomap.h
#define IOMEM(x)     (x)
#define MSM_SHARED_RAM_BASE    IOMEM(0x86300000)       /*  1M  */
#define MSM8952_MSM_SHARED_RAM_PHYS     0x86300000

// In qct new release (after 8974) it forces to use 256k as segment size.
// So we need always enable this flag.
#define PACKET_FILTER_UDP

static struct mutex port_lock;
static struct wake_lock port_suspend_lock;
static uint16_t *port_list = NULL;
#ifdef PACKET_FILTER_UDP
static uint16_t *port_list_udp = NULL;
#endif
static int usb_enable = 0;
struct p_list {
	struct list_head list;
	int no;
};
static struct p_list curr_port_list;
#ifdef PACKET_FILTER_UDP
static struct p_list curr_port_list_udp;
static int port_updated = 0;
#endif
static int packet_filter_flag = 1;
struct class *p_class;
static struct miscdevice portlist_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-portlist",
};

static int ril_debug_flag = 0;//enable for debug

#define PF_LOG_DBG(fmt, ...) do {                           \
		if (ril_debug_flag)                                 \
			printk(KERN_DEBUG "[K]" pr_fmt(fmt), ##__VA_ARGS__);  \
	} while (0)

#define PF_LOG_INFO(fmt, ...) do {                         \
		if (ril_debug_flag)                                \
			printk(KERN_INFO "[K]" pr_fmt(fmt), ##__VA_ARGS__);  \
	} while (0)

#define PF_LOG_ERR(fmt, ...) do {                     \
	if (ril_debug_flag)                               \
		printk(KERN_ERR "[K]" pr_fmt(fmt), ##__VA_ARGS__);  \
	} while (0)


#define NIPQUAD(addr) \
    ((unsigned char *)&addr)[0], \
    ((unsigned char *)&addr)[1], \
    ((unsigned char *)&addr)[2], \
    ((unsigned char *)&addr)[3]

void port_list_dump_data(struct sock *sk)
{
	struct task_struct *task = current;
	struct inet_sock *inet = NULL;
	struct net *net = NULL;

	if ( ril_debug_flag == 0 ) {
		return;
	}

	if ( sk == NULL ) {
		return;
	}

	inet = inet_sk(sk);
	net = sock_net(sk);

	if ( net ) {
		PF_LOG_INFO("[Port list] %s: inuse=[%d]\n", __FUNCTION__, sock_prot_inuse_get(net, sk->sk_prot));
	} else {
		PF_LOG_INFO("[Port list] %s: net = null\n", __FUNCTION__);
	}

	if ( inet ) {
		PF_LOG_INFO("[Port list] %s: Local:%03d.%03d.%03d.%03d:%05d(0x%x) Remote:%03d.%03d.%03d.%03d:%05d(0x%x)\n", __FUNCTION__, NIPQUAD(inet->inet_rcv_saddr), ntohs(inet->inet_sport), inet->inet_rcv_saddr, NIPQUAD(inet->inet_daddr), ntohs(inet->inet_dport), inet->inet_daddr);
	} else {
		PF_LOG_INFO("[Port list] %s: inet = null\n", __FUNCTION__);
	}
	PF_LOG_INFO("[Port list] %s: sk->sk_shutdown = [%d], sock_flag(sk, SOCK_DONE)=[%d]\n", __FUNCTION__, sk->sk_shutdown, sock_flag(sk, SOCK_DONE));
	PF_LOG_INFO("[Port list] %s: sk->sk_socket->state=[%d]\n", __FUNCTION__, sk->sk_socket->state);
	PF_LOG_INFO("[Port list] %s: sk->sk_type=[%d]\n", __FUNCTION__, sk->sk_type);
	PF_LOG_INFO("[Port list] %s: sk->sk_family=[%d]\n", __FUNCTION__, sk->sk_family);
	PF_LOG_INFO("[Port list] %s: sk->sk_state=[%d]\n", __FUNCTION__, sk->sk_state);
	PF_LOG_INFO("[Port list] %s: sk->sk_reuse=[%d]\n", __FUNCTION__, sk->sk_reuse);
	PF_LOG_INFO("[Port list] %s: sk->sk_reuseport=[%d]\n", __FUNCTION__, sk->sk_reuseport);
	PF_LOG_INFO("[Port list] %s: sk->sk_flags=[%lu]\n", __FUNCTION__, sk->sk_flags);
	if (sock_flag(sk, SOCK_DEAD)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_DEAD]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_DONE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_DONE]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_URGINLINE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_URGINLINE]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_KEEPOPEN)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_KEEPOPEN]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_LINGER)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_LINGER]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_DESTROY)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_DESTROY]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_BROADCAST)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_BROADCAST]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_TIMESTAMP)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_TIMESTAMP]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_ZAPPED)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_ZAPPED]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_USE_WRITE_QUEUE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_USE_WRITE_QUEUE]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_DBG)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_DBG]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_RCVTSTAMP)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_RCVTSTAMP]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_RCVTSTAMPNS)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_RCVTSTAMPNS]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_LOCALROUTE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_LOCALROUTE]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_QUEUE_SHRUNK)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_QUEUE_SHRUNK]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_LOCALROUTE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_LOCALROUTE]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_MEMALLOC)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_MEMALLOC]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_TIMESTAMPING_RX_SOFTWARE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_TIMESTAMPING_RX_SOFTWARE]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_FASYNC)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_FASYNC]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_RXQ_OVFL)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_RXQ_OVFL]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_ZEROCOPY)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_ZEROCOPY]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_WIFI_STATUS)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_WIFI_STATUS]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_NOFCS)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_NOFCS]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_FILTER_LOCKED)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_FILTER_LOCKED]\n", __FUNCTION__);
	if (sock_flag(sk, SOCK_SELECT_ERR_QUEUE)) PF_LOG_INFO("[Port list] %s: sk->sk_flags[SOCK_SELECT_ERR_QUEUE]\n", __FUNCTION__);

	PF_LOG_INFO("[Port list] %s: task->state=[%d][%d]\n", __FUNCTION__, task->flags, task->flags & PF_EXITING);

	PF_LOG_INFO("=== Show stack ===\n");
	show_stack(0, 0);
	PF_LOG_INFO("=== Show stack end===\n");
}

static ssize_t htc_show(struct device *dev,  struct device_attribute *attr,  char *buf)
{
	char *s = buf;
	mutex_lock(&port_lock);
	s += snprintf(s, sizeof(buf), "%d\n", packet_filter_flag);
	mutex_unlock(&port_lock);
	return s - buf;
}

static ssize_t htc_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
	int ret;

	mutex_lock(&port_lock);
	if (!strncmp(buf, "0", strlen("0"))) {
		packet_filter_flag = 0;
		PF_LOG_INFO("[Port list] Disable Packet filter\n");
#ifdef PACKET_FILTER_UDP
		if (port_list_udp != NULL)
			port_list_udp[0] = packet_filter_flag;
		else
			PF_LOG_ERR("[Port list] port_list_udp == NULL\n");
#endif
		if (port_list != NULL)
			port_list[0] = packet_filter_flag;
		else
			PF_LOG_ERR("[Port list] port_list == NULL\n");
		ret = count;
	} else if (!strncmp(buf, "1", strlen("1"))) {
		packet_filter_flag = 1;
		PF_LOG_INFO("[Port list] Enable Packet filter\n");
#ifdef PACKET_FILTER_UDP
		if (port_list_udp != NULL)
			port_list_udp[0] = packet_filter_flag;
		else
			PF_LOG_ERR("[Port list] port_list_udp == NULL\n");
#endif
		if (port_list != NULL)
			port_list[0] = packet_filter_flag;
		else
			PF_LOG_ERR("[Port list] port_list == NULL\n");
		ret = count;
	} else {
		PF_LOG_ERR("[Port list] flag: invalid argument\n");
		ret = -EINVAL;
	}
	mutex_unlock(&port_lock);

	return ret;
}

static DEVICE_ATTR(flag, 0664, htc_show, htc_store);

static int port_list_enable(int enable)
{
	if (port_list[0] != enable) {
		port_list[0] = enable;
		if (enable)
			PF_LOG_INFO("[Port list] port_list is enabled.\n");
		else
			PF_LOG_INFO("[Port list] port_list is disabled.\n");
	}
	return 0;
}

#ifdef PACKET_FILTER_UDP
static int port_list_enable_udp(int enable)
{
	if (port_list_udp[0] != enable) {
		port_list_udp[0] = enable;
		if (enable)
			PF_LOG_INFO("[Port list] port_list_udp is enabled.\n");
		else
			PF_LOG_INFO("[Port list] port_list_udp is disabled.\n");
	}
	return 0;
}
#endif

static void update_port_list(void)
{
	size_t count = 0;
	size_t i = 0;
	struct list_head *listptr;
	struct p_list *entry;

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		count++;
		PF_LOG_INFO("[Port list] [%zu] = %d\n", count, entry->no);
		if (count <= 127)
			port_list[count] = entry->no;
	}
	if (count < 127)
		for (i = count + 1; i <= 127; i++)
			port_list[i] = 0;

	if (usb_enable) {
		port_list_enable(0);
	} else {
		if (count <= 127)
			port_list_enable(1);
		else
			port_list_enable(0);
	}

#ifdef PACKET_FILTER_UDP
	count = 0;
	list_for_each(listptr, &curr_port_list_udp.list) {
		entry = list_entry(listptr, struct p_list, list);
		count++;
		if (count <= 127)
			port_list_udp[count] = entry->no;
	}
	PF_LOG_INFO("[Port list] Total UDP amount in linked-list = %zu\n", count);

	if (count < 127)
		for (i = count + 1; i <= 127; i++)
			port_list_udp[i] = 0;

	if (usb_enable) {
		port_list_enable_udp(0);
	} else {
		if (count <= 127)
			port_list_enable_udp(1);
		else
			port_list_enable_udp(0);
	}
#endif
}

static struct p_list *add_list(int no)
{
	struct p_list *ptr = NULL;
	struct list_head *listptr;
	struct p_list *entry;
	int get_list = 0;

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		if (entry->no == no) {
			PF_LOG_INFO("[Port list] TCP port[%d] is already in the list!", entry->no);
			get_list = 1;
			break;
		}
	}
	if (!get_list) {
		ptr = kmalloc(sizeof(struct p_list), GFP_KERNEL);
		if (ptr) {
			ptr->no = no;
			list_add_tail(&ptr->list, &curr_port_list.list);
			PF_LOG_INFO("[Port list] TCP port[%d] added\n", no);
		}
	}
	return (ptr);
}
#ifdef PACKET_FILTER_UDP
static struct p_list *add_list_udp(int no)
{
	struct p_list *ptr = NULL;
	struct list_head *listptr;
	struct p_list *entry;
	int get_list = 0;

	list_for_each(listptr, &curr_port_list_udp.list) {
		entry = list_entry(listptr, struct p_list, list);
		if (entry->no == no) {
			PF_LOG_INFO("[Port list] UDP port[%d] is already in the list!", entry->no);
			get_list = 1;
			break;
		}
	}
	if (!get_list) {
		ptr = kmalloc(sizeof(struct p_list), GFP_KERNEL);
		if (ptr) {
			ptr->no = no;
			list_add_tail(&ptr->list, &curr_port_list_udp.list);
			PF_LOG_INFO("[Port list] UDP port[%d] added\n", no);
			port_updated = 1;
		}
	}
	return (ptr);
}
#endif

static int remove_list(int no)
{
	struct list_head *listptr;
	struct p_list *entry;
	int get_list = 0;

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		if (entry->no == no) {
			PF_LOG_INFO("[Port list] TCP port[%d] removed\n", entry->no);
			list_del(&entry->list);
			kfree(entry);
			get_list = 1;
			break;
		}
	}
	if (!get_list) {
		PF_LOG_INFO("[Port list] TCP port[%d] failed to remove. Port number is not in list!\n", no);
		return -1;
	} else {
		return 0;
	}
}
#ifdef PACKET_FILTER_UDP
static void remove_list_udp(int no)
{
	struct list_head *listptr;
	struct p_list *entry;
	int get_list = 0;

	list_for_each(listptr, &curr_port_list_udp.list) {
		entry = list_entry(listptr, struct p_list, list);
		if (entry->no == no) {
			PF_LOG_INFO("[Port list] UDP port[%d] removed\n", entry->no);
			list_del(&entry->list);
			kfree(entry);
			get_list = 1;
			port_updated = 1;
			break;
		}
	}
	/*
	if (!get_list)
		PF_LOG_INFO("[Port list] UDP port[%d] failed to remove. Port number is not in list!\n", no);
	*/
}
#endif

static int allocate_port_list(void)
{
	uint32_t port_list_phy_addr;

#if defined PACKET_FILTER_UDP
	port_list = smem_alloc(SMEM_ID_VENDOR2, sizeof(uint16_t)*256, 0, SMEM_ANY_HOST_FLAG);
#else
	port_list = smem_alloc(SMEM_ID_VENDOR2, sizeof(uint16_t)*128, 0, SMEM_ANY_HOST_FLAG);
#endif

	port_list_phy_addr = (uint32_t)MSM8952_MSM_SHARED_RAM_PHYS + ((uint32_t)(uintptr_t)port_list - (uint32_t)MSM_SHARED_RAM_BASE);
	if (port_list == NULL) {
		/*
		PF_LOG_INFO("[Port list] Error: Cannot allocate port_list in SMEM_ID_VENDOR2\n");
		*/
		return -1;
	} else {
		PF_LOG_INFO("[Port list] Virtual Address of port_list: [%p]\n", port_list);
		PF_LOG_INFO("[Port list] Physical Address of port_list: [%X]\n", port_list_phy_addr);

		port_list[0] = packet_filter_flag;
#ifdef PACKET_FILTER_UDP
		port_list_udp = port_list + 128;
		port_list_udp[0] = packet_filter_flag;
		PF_LOG_INFO("[Port list] Address of port_list: [%p]\n", port_list);
		PF_LOG_INFO("[Port list] Address of port_list_udp: [%p]\n", port_list_udp);
#endif
		return 0;
	}
}

int add_or_remove_port(struct sock *sk, int add_or_remove)
{
	struct inet_sock *inet = NULL;
	__be32 src = 0;
	__u16 srcp = 0;

	if ( sk == NULL ) {
		PF_LOG_INFO("[Port list] add_or_remove_port: sk = null\n");
		return 0;
	}

	inet = inet_sk(sk);
	if (!inet)
	{
		PF_LOG_INFO("[Port list] add_or_remove_port: inet = null\n");
		return 0;
	}

	src = inet->inet_rcv_saddr;
	srcp = ntohs(inet->inet_sport);

	if (!packet_filter_flag) {
		return 0;
	}
	/* Check port list memory allocation */
	if (port_list == NULL) {
		if(allocate_port_list()!=0) {
			return 0;
		}
	}


	if ( sk->sk_protocol == IPPROTO_TCP &&
		add_or_remove == 0 &&
		src != 0x0100007F &&
		sk->sk_state != TCP_CLOSE &&
		sk->sk_socket->state == SS_CONNECTED) {
			PF_LOG_INFO("[Port list] can't remove port:[%d]\n", srcp);
			port_list_dump_data(sk);
			return 0;
	}

	/* if TCP packet and source IP != 127.0.0.1 */
	if (sk->sk_protocol == IPPROTO_TCP && src != 0x0100007F && srcp != 0) {
		int err = 0;
		wake_lock(&port_suspend_lock);
		mutex_lock(&port_lock);
		PF_LOG_INFO("[Port list] %s TCP port#: [%d]\n", add_or_remove?"Add":"Remove", srcp);

		if (add_or_remove) {
			struct p_list *plist = NULL;
			plist = add_list(srcp);
			if ( !plist ) {
				err = 1;
				port_list_dump_data(sk);
			}
		} else {
			int ret = 0;
			ret = remove_list(srcp);
			if ( ret != 0 ) {
				err = 1;
				port_list_dump_data(sk);
			}
		}

		if ( !err )
			update_port_list();

		mutex_unlock(&port_lock);
		wake_unlock(&port_suspend_lock);
	}

#ifdef PACKET_FILTER_UDP
	/* UDP */
	if (sk->sk_protocol == IPPROTO_UDP && src != 0x0100007F && srcp != 0) {
		PF_LOG_INFO("[Port list] %s UTP port#: [%d]\n", add_or_remove?"Add":"Remove", srcp);
		wake_lock(&port_suspend_lock);
		mutex_lock(&port_lock);
		port_updated = 0;
		if (add_or_remove)
			add_list_udp(srcp);
		else
			remove_list_udp(srcp);
		if(port_updated)
			update_port_list();
		mutex_unlock(&port_lock);
		wake_unlock(&port_suspend_lock);
	}
#endif

	return 0;
}
EXPORT_SYMBOL(add_or_remove_port);

int update_port_list_charging_state(int enable)
{
	size_t count = 0;

	if (!packet_filter_flag) {
		return 0;
	}

	if (port_list == NULL) {
		PF_LOG_INFO("[Port list] port_list is NULL.\n");
		return 0;
	}

	usb_enable = enable;
	wake_lock(&port_suspend_lock);
	mutex_lock(&port_lock);
	if (usb_enable) {
		port_list_enable(0);
	} else {
		for (count = 1; count <= 127; count++) {
			if (!port_list[count])
				break;
		}
		if (count <= 127)
			port_list_enable(1);
		else
			port_list_enable(0);
	}
#ifdef PACKET_FILTER_UDP
	if (usb_enable) {
		port_list_enable_udp(0);
	} else {
		for (count = 1; count <= 127; count++) {
			if (!port_list_udp[count])
				break;
		}
		if (count <= 127)
			port_list_enable_udp(1);
		else
			port_list_enable_udp(0);
	}
#endif
	mutex_unlock(&port_lock);
	wake_unlock(&port_suspend_lock);
	return 0;
}
EXPORT_SYMBOL(update_port_list_charging_state);

static int __init port_list_init(void)
{
	int ret;
	wake_lock_init(&port_suspend_lock, WAKE_LOCK_SUSPEND, "port_list");
	mutex_init(&port_lock);

	PF_LOG_INFO("[Port list] init()\n");

	/* Print log only when debug flag (6) to 0x400000 */
	if (get_kernel_flag() & KERNEL_FLAG_RIL_DBG_MEMCPY)
		ril_debug_flag = 1;

	/* initial TCP port list linked-list struct */
	memset(&curr_port_list, 0, sizeof(curr_port_list));
	INIT_LIST_HEAD(&curr_port_list.list);

#ifdef PACKET_FILTER_UDP
	/* initial UDP port list linked-list struct */
	memset(&curr_port_list_udp, 0, sizeof(curr_port_list_udp));
	INIT_LIST_HEAD(&curr_port_list_udp.list);
#endif

	/* Check port list memory allocation */
	allocate_port_list();

	ret = misc_register(&portlist_misc);
	if (ret < 0) {
		PF_LOG_ERR("[Port list] failed to register misc device!\n");
		goto err_misc_register;
	}

	p_class = class_create(THIS_MODULE, "htc_portlist");
	if (IS_ERR(p_class)) {
		ret = PTR_ERR(p_class);
		p_class = NULL;
		PF_LOG_ERR("[Port list] class_create failed!\n");
		goto err_class_create;
	}

	portlist_misc.this_device = device_create(p_class, NULL, 0 , NULL, "packet_filter");
	if (IS_ERR(portlist_misc.this_device)) {
		ret = PTR_ERR(portlist_misc.this_device);
		portlist_misc.this_device = NULL;
		PF_LOG_ERR("[Port list] device_create failed!\n");
		goto err_device_create;
	}

	ret = device_create_file(portlist_misc.this_device, &dev_attr_flag);
	if (ret < 0) {
		PF_LOG_ERR("[Port list] devices_create_file failed!\n");
		goto err_device_create_file;
	}

	return 0;

err_device_create_file:
	device_destroy(p_class, 0);
err_device_create:
	class_destroy(p_class);
err_class_create:
	misc_deregister(&portlist_misc);
err_misc_register:
	return ret;
}

static void __exit port_list_exit(void)
{
	//int ret;
	struct list_head *listptr;
	struct p_list *entry;

	device_remove_file(portlist_misc.this_device, &dev_attr_flag);
	device_destroy(p_class, 0);
	class_destroy(p_class);

/*
commit f368ed6088ae9c1fbe1c897bb5f215ce5e63fa1e
Author: Greg Kroah-Hartman <gregkh@linuxfoundation.org>
Date:   Thu Jul 30 15:59:57 2015 -0700

    char: make misc_deregister a void function

    With well over 200+ users of this api, there are a mere 12 users that
    actually checked the return value of this function.  And all of them
    really didn't do anything with that information as the system or module
    was shutting down no matter what.

    So stop pretending like it matters, and just return void from
    misc_deregister().  If something goes wrong in the call, you will get a
    WARNING splat in the syslog so you know how to fix up your driver.
    Other than that, there's nothing that can go wrong.

	ret = misc_deregister(&portlist_misc);
	if (ret < 0)
		PF_LOG_ERR("[Port list] failed to unregister misc device!\n");
*/
	misc_deregister(&portlist_misc);

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		kfree(entry);
	}
#ifdef PACKET_FILTER_UDP
	list_for_each(listptr, &curr_port_list_udp.list) {
		entry = list_entry(listptr, struct p_list, list);
		kfree(entry);
	}
#endif
}

late_initcall(port_list_init);
module_exit(port_list_exit);

MODULE_AUTHOR("Mio Su <Mio_Su@htc.com>");
MODULE_DESCRIPTION("HTC port list driver");
MODULE_LICENSE("GPL");
