#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
//#include <linux/htc_flags.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>

#include "htc_radio_smem.h"
#include <soc/qcom/smem.h>

#define DEVICE_TREE_RADIO_PATH "/chosen/radio"

struct htc_smem_type *htc_radio_smem_via_smd;

#ifdef CONFIG_DEBUG_FS
static int dump_smem(char *buf, int max){
	int i = 0;

	if(!htc_radio_smem_via_smd){
		i += scnprintf(buf + i, max - i, "htc_radio_smem_via_smd is NULL.\n");
		pr_err("[smem]%s: htc_radio_smem_via_smd is NULL.\n", __func__);
		return i;
	}

	i += scnprintf(buf + i, max - i,
				   "version:                  %x\n"
				   "size:                     %d\n"
				   "boot mode:                %d\n"
				   "radioflag:                0x%x\n"
				   "radioflagex1:             0x%x\n"
				   "radioflagex2:             0x%x\n"
				   "ROM version:              %s\n"
				   "secure_flag               %d\n"
				   "htc_smem_pid              %d\n"
				   "htc_smem_cid              %s\n"
				   "htc_smem_skuid            %s\n"
				   "htc_smem_radio_parameter  %s\n"
				   "htc_smem_factory_reset    0x%x\n",
				   htc_radio_smem_via_smd->version,
				   htc_radio_smem_via_smd->struct_size,
				   htc_radio_smem_via_smd->htc_smem_app_run_mode,
				   htc_radio_smem_via_smd->htc_smem_ce_radio_dbg_flag,
				   htc_radio_smem_via_smd->htc_smem_ce_radio_dbg_flag_ext1,
				   htc_radio_smem_via_smd->htc_smem_ce_radio_dbg_flag_ext2,
				   htc_radio_smem_via_smd->htc_rom_ver,
				   htc_radio_smem_via_smd->secure_flag,
				   htc_radio_smem_via_smd->htc_smem_pid,
				   htc_radio_smem_via_smd->htc_smem_cid,
				   htc_radio_smem_via_smd->htc_smem_skuid,
				   htc_radio_smem_via_smd->htc_smem_radio_parameter,
				   htc_radio_smem_via_smd->htc_smem_factory_reset
				   );
	return i;
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
				struct dentry *dent,
				int (*fill)(char *buf, int max))
{
	struct dentry *file;

	file = debugfs_create_file(name, mode, dent, fill, &debug_ops);
	if (IS_ERR(file))
		pr_err("%s: debugfs create failed %d\n", __func__,
				(int)PTR_ERR(file));
}
#endif

static void smem_init(struct htc_smem_type *smem)
{
	int i = 0;

	smem->version = 0;
	smem->struct_size = 0;
	smem->htc_smem_app_run_mode = 0;
	smem->htc_smem_ce_radio_dbg_flag = 0;
	smem->htc_smem_ce_radio_dbg_flag_ext1 = 0;
	smem->htc_smem_ce_radio_dbg_flag_ext2 = 0;

	for(i = 0; i < sizeof(smem->htc_rom_ver); i++)
		smem->htc_rom_ver[i] = 0;

	smem->secure_flag = 0;
	smem->htc_smem_pid = 0;

	for(i = 0; i < sizeof(smem->htc_smem_cid); i++)
		smem->htc_smem_cid[i] = 0;

	for(i = 0; i < sizeof(smem->htc_smem_skuid); i++)
		smem->htc_smem_skuid[i] = 0;

	for(i = 0; i < sizeof(smem->htc_smem_radio_parameter); i++)
		smem->htc_smem_radio_parameter[i] = 0;

	for(i = 0; i < sizeof(smem->reserved); i++)
		smem->reserved[i] = 0;

	smem->htc_smem_factory_reset = 0;
}

static int htc_radio_smem_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct device_node *dnp;

	pr_info("[smem]%s: start.\n", __func__);

	htc_radio_smem_via_smd = smem_alloc( SMEM_ID_VENDOR0, sizeof(struct htc_smem_type), 0, SMEM_ANY_HOST_FLAG );

	if(htc_radio_smem_via_smd ) {
		pr_info("[smem]%s: smem_via_smd=0x%p.\n", __func__, htc_radio_smem_via_smd);
	}else{
		ret = -ENOMEM;
		pr_err("[smem]%s: smd alloc fail !!\n", __func__);
		return ret;
	}

	/* set smem init 0 */
	smem_init(htc_radio_smem_via_smd);

	/* get smem data from radio data note */
	dnp = of_find_node_by_path(DEVICE_TREE_RADIO_PATH);
	if(dnp) {
		htc_radio_smem_via_smd->version = HTC_RADIO_SMEM_VERSION;
		htc_radio_smem_via_smd->struct_size = sizeof(struct htc_smem_type);
		of_property_read_u32(dnp, "htc_smem_app_run_mode",
							&htc_radio_smem_via_smd->htc_smem_app_run_mode);
		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag",
							&htc_radio_smem_via_smd->htc_smem_ce_radio_dbg_flag);
		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag_ext1",
							&htc_radio_smem_via_smd->htc_smem_ce_radio_dbg_flag_ext1);
		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag_ext2",
							&htc_radio_smem_via_smd->htc_smem_ce_radio_dbg_flag_ext2);
		of_property_read_u8_array(dnp, "htc_rom_ver",&htc_radio_smem_via_smd->htc_rom_ver[0],
							sizeof(htc_radio_smem_via_smd->htc_rom_ver));
		of_property_read_u32(dnp, "htc_smem_pid", &htc_radio_smem_via_smd->htc_smem_pid);
		of_property_read_u8_array(dnp, "htc_smem_cid", &htc_radio_smem_via_smd->htc_smem_cid[0],
							sizeof(htc_radio_smem_via_smd->htc_smem_cid));
		of_property_read_u8_array(dnp, "sku_id", &htc_radio_smem_via_smd->htc_smem_skuid[0],
							sizeof(htc_radio_smem_via_smd->htc_smem_skuid));
		of_property_read_u8_array(dnp, "modem_reserve", &htc_radio_smem_via_smd->htc_smem_radio_parameter[0],
							sizeof(htc_radio_smem_via_smd->htc_smem_radio_parameter));
		of_property_read_u32(dnp, "htc_smem_factory_reset",
							&htc_radio_smem_via_smd->htc_smem_factory_reset);
	} else
		pr_err("[smem]%s: cannot find path %s.\n", __func__, DEVICE_TREE_RADIO_PATH);

	pr_info("[smem]%s: end.\n", __func__);
	return 0;
}

static struct of_device_id htc_radio_smem_of_match[] = {
	{.compatible = "htc,htc_radio_smem",}
};
MODULE_DEVICE_TABLE(of, htc_radio_smem_of_match);

static struct platform_driver htc_radio_smem_driver = {
	.probe = htc_radio_smem_probe,
	.driver = {
		.name = "htc_radio_smem",
		.owner = THIS_MODULE,
		.of_match_table = htc_radio_smem_of_match,
	}
};

static int __init htc_radio_smem_init(void)
{
	int ret = -1;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;
#endif

	pr_info("[smem]%s.\n", __func__);

	ret = platform_driver_register(&htc_radio_smem_driver);
	if(ret < 0 ) {
		pr_err("[smem]%s platform_driver register fail. ret:%d\n", __func__, ret);
		goto register_fail;
	}

#ifdef CONFIG_DEBUG_FS
	dent = debugfs_create_dir("htc_radio_smem", 0);
	if (!IS_ERR(dent)) {
		debug_create("dump_smem", 0444, dent, dump_smem);
	}
#endif

register_fail:
	return ret;
}

static void __exit htc_radio_smem_exit(void)
{
	platform_driver_unregister(&htc_radio_smem_driver);
}

module_init(htc_radio_smem_init);
module_exit(htc_radio_smem_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("htc radio smem driver");
