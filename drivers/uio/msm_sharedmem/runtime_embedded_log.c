/* Copyright (c) 2016, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/uio_driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/cma.h>
#include <soc/qcom/secure_buffer.h>
#include <linux/slab.h>
#include <linux/remote_spinlock.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/ramdump.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/subsystem_restart.h>
#include "sharedmem_qmi.h"

#define CLIENT_ID_PROP "qcom,client-id"

#define MPSS_RMTS_CLIENT_ID 1

#define DRIVER_NAME "rtel" /* runtime_embedded_log */
/* #define pr_fmt(fmt) DRIVER_NAME ": %s: " fmt, __func__ */

#if (IS_T_PROJECT == 0)
#define DEVICE_TREE_RADIO_PATH "/chosen/radio"
#define RADIO_SMLOG_FLAG BIT(27)
#define UT_LONG_SKU_LEN 5
#define UT_LONG_SKU_FIRST_NUM '9'
#define UT_SHORT_SKU_NUM "999"

static char *rom_version = NULL;
#endif /* IS_T_PROJECT == 0 */

struct restart_notifier_block {
	unsigned processor;
	char *name;
	struct notifier_block nb;
};

static struct ramdump_segment *rtel_ramdump_segments;
static void *rtel_ramdump_dev;

static void *uio_vaddr;
static struct device *uio_dev;

static int setup_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size);
static int clear_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size);

static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
#if defined(CONFIG_HTC_FEATURES_SSR)
	struct notif_data *notif = data;
	if (code == SUBSYS_RAMDUMP_NOTIFICATION &&
	    notif->enable_ramdump == ENABLE_RAMDUMP) {
#else
	if (code == SUBSYS_RAMDUMP_NOTIFICATION) {
#endif
		struct restart_notifier_block *notifier;

		notifier = container_of(this,
					struct restart_notifier_block, nb);
		pr_info("[rtel]%s: ssrestart for processor %d ('%s')\n",
				__func__, notifier->processor,
				notifier->name);

		if (rtel_ramdump_dev && rtel_ramdump_segments) {
			int ret;

			pr_info("[rtel]%s: saving runtime embedded log ramdump.\n",
					__func__);
			ret = do_ramdump(rtel_ramdump_dev,
					rtel_ramdump_segments, 1);
			if (ret < 0)
				pr_err("[rtel]%s: unable to dump runtime embedded log %d\n",
						__func__, ret);
		}
	}

	return NOTIFY_DONE;
}

static struct restart_notifier_block restart_notifiers[] = {
	{SMEM_MODEM, "modem", .nb.notifier_call = restart_notifier_cb},
};

#if (IS_T_PROJECT == 0)
static bool get_rom_version(void)
{
	struct device_node *dnp;

	if (!rom_version) {
		/* get ROM version */
		dnp = of_find_node_by_path(DEVICE_TREE_RADIO_PATH);
		if(dnp)
			rom_version = (char *) of_get_property(dnp,
						"htc_rom_ver", NULL);
		else {
			pr_err("[rtel]%s: cannot find path %s.\n",
				__func__, DEVICE_TREE_RADIO_PATH);
			return false;
		}
	}
	return true;
}

/*
* UT ROM Checking Rule
* 1. sku version = 999
* 2. length of sku version = 5 and first number is 9 (ex. 9xxxx)
*/
static bool is_ut_rom(void)
{
	int len = 0;
	int c = 0;
	int i = 0;
	char *main_version, *sku_version;

	if (!rom_version) {
		if (!get_rom_version()) {
			pr_err("[rtel]%s: no ROM version.\n", __func__);
			return false;
		}
	}

	main_version = strstr(rom_version, ".");
	if (!main_version || strlen(main_version + 1) == 0) {
		pr_err("[rtel]%s: no main version.\n", __func__);
		return false;
	}

	sku_version = strstr(main_version + 1, ".") + 1;
	if (!sku_version || strlen(sku_version + 1) == 0) {
		pr_err("[rtel]%s: no sku version.\n", __func__);
		return false;
	}

	len = strlen(sku_version);
	for (i = 0; i < len; i++) {
		if (isdigit(sku_version[i]))
			c++;
		else
			break;
	}

	if (sku_version[0] == UT_LONG_SKU_FIRST_NUM && c == UT_LONG_SKU_LEN)
		return true;

	if (strstr(sku_version, UT_SHORT_SKU_NUM) && c == strlen(UT_SHORT_SKU_NUM))
		return true;

	return false;
}

/*
* smlog Enabled Rule
* --------------------------------
*          |   RADIO_SMLOG_FLAG
* --------------------------------
*  UT ROM  |     Y     |    N
* --------------------------------
*     Y    |  disable  |  enable
* --------------------------------
*     N    |  enable   |  disable
* --------------------------------
*/
bool is_smlog_enabled(void)
{
	struct device_node *dnp;
	u32 boot_mode = 0, radioflag = 0;
	int ret;
	/* Get boot_mode and radioflag */
	dnp = of_find_node_by_path(DEVICE_TREE_RADIO_PATH);
	if(dnp) {
		ret = of_property_read_u32(dnp, "htc_smem_app_run_mode",
					&boot_mode);
		if (ret) {
			pr_err("[rtel]%s: cannot get boot mode.\n", __func__);
			return false;
		}
		ret = of_property_read_u32(dnp, "htc_smem_radio_dbg_flag",
					&radioflag);
		if (ret) {
			pr_err("[rtel]%s: cannot get radio flag.\n", __func__);
			return false;
		}
	} else {
		pr_err("[rtel]%s: cannot find path %s.\n",
			__func__, DEVICE_TREE_RADIO_PATH);
		return false;
	}

	if (boot_mode != 0x8) {
	/* boot_mode is not 0x8 (FTM_MODE) */
		if (is_ut_rom()) {
			if (radioflag & RADIO_SMLOG_FLAG)
				return false;
			else
				return true;
		} else if (radioflag & RADIO_SMLOG_FLAG)
			return true;
		else
			return false;
	} else
		return false;
}
#endif /* IS_T_PROJECT == 0 */

static ssize_t rtel_show(struct device *d,
			struct device_attribute *attr,
			char *buf)
{
	struct uio_info *info = dev_get_drvdata(d);

	return snprintf(buf, PAGE_SIZE, "size:0x%x paddr:0x%x vaddr:0x%p\n",
					(unsigned int)info->mem[0].size,
					(unsigned int)info->mem[0].addr,
					uio_vaddr);
}

static ssize_t rtel_store(struct device *d,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int ret = 0;
	u64 rmtfs_addr = 0, rtel_addr = 0;
	u32 rmtfs_size = 0, rtel_size = 0;
	struct uio_info *info = dev_get_drvdata(d);
	void *handle;
	struct restart_notifier_block *nb;

	pr_info("[rtel] %s entering\n", __func__);

#if (IS_T_PROJECT == 0) /* Follw inhouse rule to enable smlog */
        if (!is_smlog_enabled()) {
                pr_info("[rtel] is_smlog_enabled = %d, drop rtel store\n",
                         is_smlog_enabled());
                return count;
        }
#endif /* IS_T_PROJECT == 0 */

	if (!info) {
		pr_err("can't get correct uio info.\n");
		return count;
	}

	get_uio_addr_size_by_name("rmtfs", &rmtfs_addr, &rmtfs_size);
	rtel_addr = info->mem[0].addr;
	rtel_size = info->mem[0].size;

	if (!rmtfs_addr || !rmtfs_size || !rtel_addr || !rtel_size) {
		pr_err("can't get rmtfs/rtel address or size data.\n");
		return count;
	}

	/* Debug purpose, input userdebug/release to switch mem protect area */
	if ((buf[0] == '1' && !uio_vaddr) || !strncmp(buf, "enable", 6)) {
		uio_vaddr = dma_alloc_writecombine(uio_dev, rtel_size,
							&rtel_addr, GFP_KERNEL);

		if (uio_vaddr == NULL) {
			pr_err("Shared memalloc fail, client=%s, size=%x\n",
				info->name, rtel_size);
			return count;
		}

		clear_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rmtfs_addr,
					rmtfs_size);
		ret = setup_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rtel_addr,
					rtel_size);

		if (ret)
			pr_err("%s setup_shared_ram_perms fail.\n", info->name);
		else {
			/* Set ver 1.0 for userspace smlog enable detection */
			info->version = "1.0";
			dev_set_drvdata(d, info);
			pr_info("[rtel] Enable smlog, set %s device ver %s\n",
					info->name, info->version);
		}

		if (rtel_ramdump_dev)
			return count;

		rtel_ramdump_segments = kzalloc(sizeof(struct ramdump_segment),
						GFP_KERNEL);
		if (IS_ERR_OR_NULL(rtel_ramdump_segments)) {
			pr_err("[rtel]%s:rtel_ramdump_segments alloc fail\n",
					__func__);
			rtel_ramdump_segments = NULL;
		} else {
			rtel_ramdump_segments->address = rtel_addr +
							rmtfs_size;
			rtel_ramdump_segments->size = rtel_size - rmtfs_size;
			rtel_ramdump_segments->v_address = uio_vaddr +
							rmtfs_size;

			pr_info("[rtel] rtel addr= 0x%lx, size= 0x%lx, v_addr= 0x%p\n",
					rtel_ramdump_segments->address,
					rtel_ramdump_segments->size,
					rtel_ramdump_segments->v_address);
		}
#if (IS_T_PROJECT == 0)
		/* Create ramdump_smlog device for inhouse projects */
		rtel_ramdump_dev = create_ramdump_device("smlog", NULL);
#else
		/* Create ramdump_retel device for T-projects */
		rtel_ramdump_dev = create_ramdump_device("rtel", NULL);
#endif /* (IS_T_PROJECT == 0) */
		if (IS_ERR_OR_NULL(rtel_ramdump_dev)) {
			pr_err("[rtel]%s: Unable to create rtel ramdump device.\n",
					__func__);
			rtel_ramdump_dev = NULL;
		}

		if (rtel_ramdump_segments && rtel_ramdump_dev) {
			nb = &restart_notifiers[0];
			handle = subsys_notif_register_notifier(nb->name,
				    &nb->nb);
			if (IS_ERR_OR_NULL(handle)) {
				pr_err("[rtel]%s: Unable to register subsys notify.\n",
						__func__);
			} else
				pr_info("[rtel] registering notif for '%s', handle=0x%p\n",
						nb->name, handle);
		}
	} else if (uio_vaddr && !strncmp(buf, "disable", 7)) {
		clear_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rtel_addr,
		    rtel_size);
		ret = setup_shared_ram_perms(MPSS_RMTS_CLIENT_ID, rmtfs_addr,
			rmtfs_size);

		if (ret)
			pr_err("%s setup_shared_ram_perms fail!!\n",
					info->name);
		else {
			/* Set ver N/A for userspace smlog disable detection */
			info->version = "N/A";
			dev_set_drvdata(d, info);
			pr_info("[rtel] Disable smlog, set %s device ver %s\n",
					info->name, info->version);
		}

		dma_free_writecombine(uio_dev, rtel_size, uio_vaddr, rtel_addr);
		uio_vaddr = NULL;
	} else
		pr_info("[rtel] Invalid arg %s or uio_vaddr exists, do nothing\n",
				buf);

	return count;
}

static DEVICE_ATTR(rtel, 0664, rtel_show, rtel_store);
static struct attribute *rtel_attributes[] = {
	&dev_attr_rtel.attr,
	NULL
};

static const struct attribute_group rtel_group = {
	.name  = "rtel",
	.attrs = rtel_attributes,
};


static int uio_get_mem_index(struct uio_info *info, struct vm_area_struct *vma)
{
	if (vma->vm_pgoff >= MAX_UIO_MAPS)
		return -EINVAL;

	if (info->mem[vma->vm_pgoff].size == 0)
		return -EINVAL;

	return (int)vma->vm_pgoff;
}

static int sharedmem_mmap(struct uio_info *info, struct vm_area_struct *vma)
{
	int result;
	struct uio_mem *mem;
	int mem_index = uio_get_mem_index(info, vma);

	if (mem_index < 0) {
		pr_err("mem_index is invalid errno %d\n", mem_index);
		return mem_index;
	}

	mem = info->mem + mem_index;

	if (vma->vm_end - vma->vm_start > mem->size) {
		pr_err("vm_end[%lu] - vm_start[%lu] [%lu] > mem->size[%lu]\n",
			(unsigned long) vma->vm_end,
			(unsigned long) vma->vm_start,
			(unsigned long) (vma->vm_end - vma->vm_start),
			(unsigned long) mem->size);
		return -EINVAL;
	}
	pr_debug("Attempting to setup mmap.\n");

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	result = remap_pfn_range(vma, vma->vm_start,
				mem->addr >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);
	if (result != 0)
		pr_err("mmap Failed with errno %d\n", result);
	else
		pr_debug("mmap success\n");

	return result;
}

/* Setup the shared ram permissions.
 * This function currently supports the mpss client only.
 */
static int setup_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size)
{
	int ret;
	u32 source_vmlist[1] = {VMID_HLOS};
	int dest_vmids[2] = {VMID_HLOS, VMID_MSS_MSA};
	int dest_perms[2] = {PERM_READ|PERM_WRITE, PERM_READ|PERM_WRITE};

	if (client_id != MPSS_RMTS_CLIENT_ID)
		return -EINVAL;

	ret = hyp_assign_phys(addr, size, source_vmlist, 1, dest_vmids,
				dest_perms, 2);
	if (ret)
		pr_err("hyp_assign_phys failed addr=0x%pa size=%x err=%d\n",
			&addr, size, ret);
	return ret;
}

static int clear_shared_ram_perms(u32 client_id, phys_addr_t addr, u32 size)
{
	int ret;
	u32 source_vmlist[2] = {VMID_HLOS, VMID_MSS_MSA};
	int dest_vmids[1] = {VMID_HLOS};
	int dest_perms[1] = {PERM_READ|PERM_WRITE};

	if (client_id != MPSS_RMTS_CLIENT_ID)
		return -EINVAL;

	ret = hyp_assign_phys(addr, size, source_vmlist, 2, dest_vmids,
				dest_perms, 1);
	if (ret)
		pr_err("hyp_assign_phys failed addr=0x%pa size=%x err=%d\n",
			&addr, size, ret);
	return ret;
}

static int rtel_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct uio_info *info = NULL;
	struct resource *clnt_res = NULL;
	u32 client_id = ((u32)~0U);
	u32 shared_mem_size = 0;
	phys_addr_t shared_mem_pyhsical = 0;
	bool is_addr_dynamic = false;
	struct sharemem_qmi_entry qmi_entry;

	/* Get the addresses from platform-data */
	if (!pdev->dev.of_node) {
		pr_err("Node not found\n");
		ret = -ENODEV;
		goto out;
	}
	clnt_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!clnt_res) {
		pr_err("resource not found\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, CLIENT_ID_PROP,
				   &client_id);
	if (ret) {
		client_id = ((u32)~0U);
		pr_warn("qcom,client-id property not found\n");
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	shared_mem_size = resource_size(clnt_res);
	shared_mem_pyhsical = clnt_res->start;

	if (shared_mem_size == 0) {
		pr_err("Shared memory size is zero\n");
		return -EINVAL;
	}

	/* Setup device */
	info->mmap = sharedmem_mmap; /* Custom mmap function. */
	info->name = clnt_res->name;
	info->version = "N/A";
	info->mem[0].addr = shared_mem_pyhsical;
	info->mem[0].size = shared_mem_size;
	info->mem[0].memtype = UIO_MEM_PHYS;
	uio_dev = &pdev->dev;
	uio_vaddr = NULL;

	ret = uio_register_device(&pdev->dev, info);
	if (ret) {
		pr_err("uio register failed ret=%d\n", ret);
		goto out;
	}

	dev_set_drvdata(&pdev->dev, info);

	ret = sysfs_create_group(&pdev->dev.kobj, &rtel_group);
	if (ret)
		pr_err("failed to register rtel sysfs\n");

	qmi_entry.client_id = client_id;
	qmi_entry.client_name = info->name;
	qmi_entry.address = info->mem[0].addr;
	qmi_entry.size = info->mem[0].size;
	qmi_entry.is_addr_dynamic = is_addr_dynamic;

	sharedmem_qmi_add_entry(&qmi_entry);
	pr_info("Device created for client '%s'\n", clnt_res->name);
out:
	return ret;
}

static int rtel_remove(struct platform_device *pdev)
{
	struct uio_info *info = dev_get_drvdata(&pdev->dev);

	uio_unregister_device(info);

	return 0;
}

static const struct of_device_id rtel_of_match[] = {
	{.compatible = "htc,rtel-uio",},
	{}
};
MODULE_DEVICE_TABLE(of, rtel_of_match);

static struct platform_driver rtel_driver = {
	.probe	= rtel_probe,
	.remove	= rtel_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = rtel_of_match,
	},
};

static int __init rtel_init(void)
{
	int result;

	result = platform_driver_register(&rtel_driver);
	if (result != 0) {
		pr_err("Platform driver rtel_driver registration failed\n");
		return result;
	}
	return 0;
}

static void __exit rtel_exit(void)
{
	platform_driver_unregister(&rtel_driver);
}

module_init(rtel_init);
module_exit(rtel_exit);

MODULE_LICENSE("GPL v2");
