/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include "mdss_htc_util.h"
#include "mdss_dsi.h"
#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_panel.h"

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
	extern void lcm_rst_callback_register( void (*lcm_rst_func)(void) );
#endif

struct attribute_status htc_attr_status[] = {
	{"cabc_level_ctl", 0, 0, 0},
	{"burst_switch", 0, 0, 0},
	{"color_temp_ctl", 8, UNDEF_VALUE, UNDEF_VALUE},
	{"bklt_cali_enable", 1, UNDEF_VALUE, UNDEF_VALUE},
	{"disp_cali_enable", 1, 0, 0},
	{"color_profile_ctl", 0, 0, 0},
};

#define DEBUG_BUF   2048
#define MIN_COUNT   9
#define DCS_MAX_CNT   128

static struct delayed_work dimming_work;
static void dimming_do_work(struct work_struct *work);
static struct msm_fb_data_type *g_mfd = NULL;
static char debug_buf[DEBUG_BUF];
struct mdss_dsi_ctrl_pdata *ctrl_instance = NULL;
static char dcs_cmds[DCS_MAX_CNT];
static char *tmp;
static struct dsi_cmd_desc debug_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 1}, dcs_cmds
};
static char dsi_rbuf[4];
static void dsi_read_cb(int len)
{
	unsigned *lp;

	lp = (uint32_t *)dsi_rbuf;
	pr_info("%s: data=0x%x len=%d\n", __func__,*lp, len);
}
static ssize_t dsi_cmd_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	u32 type = 0, value = 0;
	int cnt, i;
	struct dcs_cmd_req cmdreq;

	if (count >= sizeof(debug_buf) || count < MIN_COUNT)
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	if (!ctrl_instance)
		return count;

	/* end of string */
	debug_buf[count] = 0;

	/* Format:
	ex: echo 39 51 ff > dsi_cmd
	read ex: echo 06 52 00 > dsi_cmd
	     [type] space [addr] space [value]
	     +--+--+-----+--+--+------+--+--+-
	bit   0  1    2   3  4     5   6  7
	ex:    39          51           ff
	*/
	/* Calc the count, min count = 9, format: type addr value */
	cnt = (count) / 3 - 1;
	debug_cmd.dchdr.dlen = cnt;

	/* Get the type */
	sscanf(debug_buf, "%x", &type);

	if (type == DTYPE_DCS_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_DCS_LWRITE;
	else if (type == DTYPE_GEN_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_GEN_LWRITE;
	else if (type == DTYPE_DCS_READ)
		debug_cmd.dchdr.dtype = DTYPE_DCS_READ;
	else
		return -EFAULT;

	pr_info("%s: cnt=%d, type=0x%x\n", __func__, cnt, type);

	/* Get the cmd and value */
	for (i = 0; i < cnt; i++) {
		if (i >= DCS_MAX_CNT) {
			pr_info("%s: DCS command count over DCS_MAX_CNT, Skip these commands.\n", __func__);
			break;
		}
		tmp = debug_buf + (3 * (i + 1));
		sscanf(tmp, "%x", &value);
		dcs_cmds[i] = value;
		pr_info("%s: value=0x%x\n", __func__, dcs_cmds[i]);
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	memset(&dsi_rbuf, 0, sizeof(dsi_rbuf));

	if (type == DTYPE_DCS_READ){
		cmdreq.cmds = &debug_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_REQ_RX;
		cmdreq.rlen = 4;
		cmdreq.rbuf = dsi_rbuf;
		cmdreq.cb = dsi_read_cb; /* call back */

		mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
	} else {
		cmdreq.cmds = &debug_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
		pr_info("%s %ld\n", __func__, count);
	}
	return count;
}


static void htc_report_panel_dead(void)
{
	if (g_mfd){
		mdss_fb_report_panel_dead(g_mfd);
		pr_err("%s:do panel reset\n", __func__);
	}else{
		pr_err("%s:no mfd resource, skip panel reset\n", __func__);
	}

}

static ssize_t panel_dead_write( struct file *file, const char __user *buff,
												size_t count,loff_t *ppos)
{
	unsigned long res;
	int rc;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	rc = kstrtoul(debug_buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buff, rc);
		count = -EINVAL;
		goto err_out;
	}

	if (res == 1){
		htc_report_panel_dead();
	}

err_out:
	return count;
}

static const struct file_operations dsi_cmd_fops = {
	.write = dsi_cmd_write,
};

static const struct file_operations panel_dead_fops = {
	.write = panel_dead_write,
};

void htc_debugfs_init(struct msm_fb_data_type *mfd)
{
	struct dentry *dent = debugfs_create_dir("htc_debug", NULL);

	if (mfd->panel_info->pdest != DISPLAY_1)
		return;

	if (!g_mfd)
		g_mfd = mfd;

	INIT_DELAYED_WORK(&dimming_work, dimming_do_work);

	if (IS_ERR(dent)) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return;
	}

	if (debugfs_create_file("report_panel_dead", 0644, dent, 0, &panel_dead_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
	}

	return;
}

void htc_debugfs_deinit(struct msm_fb_data_type *mfd){
	if (mfd->panel_info->pdest == DISPLAY_1) {
		ctrl_instance = NULL;
		g_mfd = NULL;
	}
}

static struct calibration_gain aux_gain;
#define RGB_MIN_COUNT   9
static ssize_t rgb_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%x\n%s%x\n%s%x\n", "GAIN_R=0x", aux_gain.R, "GAIN_G=0x",
				aux_gain.G, "GAIN_B=0x", aux_gain.B);
	return ret;
}

static ssize_t rgb_gain_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned temp, temp1, temp2;

	if (count < RGB_MIN_COUNT)
		return -EFAULT;

	/* Format:
	ex: echo f8 f0 ef > disp_cali
	    [gain.R] space [gain.G] space [gain.B]
	    +---+---+------+---+---+------+---+---+-
	bit   0   1    2     3   4     5    6   7
	ex:     f8             f0             ef
	*/
	/* min count = 9, format: gain.R gain.G gain.B */

	if (sscanf(buf, "%x %x %x ", &temp, &temp1, &temp2) != 3) {
		pr_err("%s sscanf buf fail\n",__func__);
	} else if (RGB_GAIN_CHECK(temp) && RGB_GAIN_CHECK(temp1) && RGB_GAIN_CHECK(temp2)) {
		aux_gain.R = temp;
		aux_gain.G = temp1;
		aux_gain.B = temp2;
		pr_info("%s %d, gain_r=%x, gain_g=%x, gain_b=%x \n",__func__, __LINE__,
				aux_gain.R, aux_gain.G, aux_gain.B);
	}

	return count;
}

static ssize_t bklt_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%d\n", "GAIN_BKLT=", aux_gain.BKL);
	return ret;
}

static ssize_t bklt_gain_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int temp = 0;

	if (sscanf(buf, "%d", &temp) != 1) {
		pr_err("%s sscanf buf fail\n",__func__);
	} else if(BRI_GAIN_CHECK(temp)) {
		aux_gain.BKL = temp;
		pr_info("[DISP]%s %d, gain_bkl=%d \n",__func__, __LINE__, aux_gain.BKL);
	}

	return count;
}

static unsigned camera_backlight_value = 0;
static ssize_t camera_bl_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%u\n", "BL_CAM_MIN=", camera_backlight_value);
	return ret;
}

static unsigned hal_color_feature_enabled = 0;
static ssize_t hal_color_feature_enabled_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", hal_color_feature_enabled);
	return ret;
}

static unsigned ddic_color_mode_supported = 0;
static ssize_t ddic_color_mode_supported_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", ddic_color_mode_supported);
	return ret;
}

static ssize_t attrs_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", htc_attr_status[i].cur_value);
			break;
		}
	}

	return ret;
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned long res;
	int rc, i;

	rc = kstrtoul(buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			htc_attr_status[i].req_value = res;
			break;
		}
	}

err_out:
	return count;
}

/*
HTC native mipi command format :

	"format string", < sleep ms >,  <cmd leng>, ['cmd bytes'...] ;

	"format string" :
		"DTYPE_DCS_WRITE"  : 0x05
		"DTYPE_DCS_WRITE1" : 0x15
		"DTYPE_DCS_LWRITE" : 0x39
		"DTYPE_GEN_WRITE"  : 0x03
		"DTYPE_GEN_WRITE1" : 0x13
		"DTYPE_GEN_WRITE2" : 0x23
		"DTYPE_GEN_LWRITE" : 0x29

	Example :
		htc-fmt,mdss-dsi-off-command =
					"DTYPE_DCS_WRITE", <1>, <0x02>, [28 00],
					"DTYPE_DCS_WRITE", <120>, <0x02>, [10 00];
		htc-fmt,display-on-cmds = "DTYPE_DCS_WRITE", <0x0A>, <0x02>, [29 00];
		htc-fmt,cabc-off-cmds = "DTYPE_DCS_LWRITE", <1>, <0x02>, [55 00];
		htc-fmt,cabc-ui-cmds =
			"DTYPE_DCS_LWRITE", <5>, <0x02>, [55 02],
			"DTYPE_DCS_LWRITE", <1>, <0x02>, [5E 11],
			"DTYPE_DCS_LWRITE", <1>, <0x0A>, [CA 2D 27 26 25 24 22 21 21 20],
			"DTYPE_DCS_LWRITE", <1>, <0x23>, [CE 00 00 00 00 10 10 16 16 16 16 16 16 16 16 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00];

*/

#define SLEEPMS_OFFSET(strlen) (strlen+1) /* < sleep ms>, [cmd len ... */
#define CMDLEN_OFFSET(strlen)  (SLEEPMS_OFFSET(strlen)+sizeof(const __be32))
#define CMD_OFFSET(strlen)     (CMDLEN_OFFSET(strlen)+sizeof(const __be32))

static struct __dsi_cmd_map{
	char *cmdtype_str;
	int  cmdtype_strlen;
	int  dtype;
} dsi_cmd_map[] = {
	{ "DTYPE_DCS_WRITE", 0, DTYPE_DCS_WRITE },
	{ "DTYPE_DCS_WRITE1", 0, DTYPE_DCS_WRITE1 },
	{ "DTYPE_DCS_LWRITE", 0, DTYPE_DCS_LWRITE },
	{ "DTYPE_GEN_WRITE", 0, DTYPE_GEN_WRITE },
	{ "DTYPE_GEN_WRITE1", 0, DTYPE_GEN_WRITE1 },
	{ "DTYPE_GEN_WRITE2", 0, DTYPE_GEN_WRITE2 },
	{ "DTYPE_GEN_LWRITE", 0, DTYPE_GEN_LWRITE },
	{ NULL, 0, 0 }
};

int htc_mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len = 0;
	char *buf;
	struct property *prop;
	struct dsi_ctrl_hdr *pdchdr;
	int i, cnt;
	int curcmdtype;

	i = 0;
	while (dsi_cmd_map[i].cmdtype_str) {
		if (!dsi_cmd_map[i].cmdtype_strlen) {
			dsi_cmd_map[i].cmdtype_strlen = strlen(dsi_cmd_map[i].cmdtype_str);
		}
		i++;
	}

	prop = of_find_property( np, cmd_key, &len);
	if (!prop || !len || !(prop->length) || !(prop->value)) {
		pr_err("%s: failed, key=%s  [%d : %d : %pK]\n", __func__, cmd_key,
			len, (prop ? prop->length : -1), (prop ? prop->value : 0) );
		//pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	data = prop->value;
	blen = 0;
	cnt = 0;
	while (len > 0) {
		curcmdtype = 0;
		while (dsi_cmd_map[curcmdtype].cmdtype_strlen) {
			if( !strncmp( data, dsi_cmd_map[curcmdtype].cmdtype_str,
						dsi_cmd_map[curcmdtype].cmdtype_strlen ) &&
				data[dsi_cmd_map[curcmdtype].cmdtype_strlen] == '\0' )
				break;
			curcmdtype++;
		};
		if( !dsi_cmd_map[curcmdtype].cmdtype_strlen ) /* no matching */
			break;

		i = be32_to_cpup((__be32 *)&data[CMDLEN_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]);
		blen += i;
		cnt++;

		data = data + CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) + i;
		len = len - CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) - i;
	}

	if(len || !cnt || !blen){
		pr_err("%s: failed, key[%s] : %d cmds, remain=%d bytes \n", __func__, cmd_key, cnt, len);
		return -ENOMEM;
	}

	i = (sizeof(char)*blen+sizeof(struct dsi_ctrl_hdr)*cnt);
	buf = kzalloc( i, GFP_KERNEL);
	if (!buf){
		pr_err("%s: create dsi ctrl oom failed \n", __func__);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pcmds->cmds){
		pr_err("%s: create dsi commands oom failed \n", __func__);
		goto exit_free;
	}

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = i;
	data = prop->value;
	for(i=0; i<cnt; i++){
		pdchdr = &pcmds->cmds[i].dchdr;

		curcmdtype = 0;
		while(dsi_cmd_map[curcmdtype].cmdtype_strlen){
			if( !strncmp( data, dsi_cmd_map[curcmdtype].cmdtype_str,
						dsi_cmd_map[curcmdtype].cmdtype_strlen ) &&
				data[dsi_cmd_map[curcmdtype].cmdtype_strlen] == '\0' ){
				pdchdr->dtype = dsi_cmd_map[curcmdtype].dtype;
				break;
			}
			curcmdtype ++;
		}

		pdchdr->last = 0x01;
		pdchdr->vc = 0x00;
		pdchdr->ack = 0x00;
		pdchdr->wait = be32_to_cpup((__be32 *)&data[SLEEPMS_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]) & 0xff;
		pdchdr->dlen = be32_to_cpup((__be32 *)&data[CMDLEN_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]);
		memcpy( buf, pdchdr, sizeof(struct dsi_ctrl_hdr) );
		buf += sizeof(struct dsi_ctrl_hdr);
		memcpy( buf, &data[CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)], pdchdr->dlen);
		pcmds->cmds[i].payload = buf;
		buf += pdchdr->dlen;
		data = data + CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) + pdchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (data) {
		if (!strncmp(data, "dsi_hs_mode", 11))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	} else {
		pcmds->link_state = DSI_HS_MODE;
	}
	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}

static DEVICE_ATTR(backlight_info, S_IRUGO, camera_bl_show, NULL);
static DEVICE_ATTR(cabc_level_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(burst_switch, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_temp_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_profile_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(bklt_cali, S_IRUGO | S_IWUSR, bklt_gain_show, bklt_gain_store);
static DEVICE_ATTR(bklt_cali_enable, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(disp_cali, S_IRUGO | S_IWUSR, rgb_gain_show, rgb_gain_store);
static DEVICE_ATTR(disp_cali_enable, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(hal_disp_color_enable, S_IRUGO, hal_color_feature_enabled_show, NULL);
static DEVICE_ATTR(ddic_color_mode_supported, S_IRUGO, ddic_color_mode_supported_show, NULL);
static struct attribute *htc_extend_attrs[] = {
	&dev_attr_backlight_info.attr,
	&dev_attr_cabc_level_ctl.attr,
	&dev_attr_burst_switch.attr,
	&dev_attr_color_temp_ctl.attr,
	&dev_attr_color_profile_ctl.attr,
	&dev_attr_bklt_cali.attr,
	&dev_attr_bklt_cali_enable.attr,
	&dev_attr_disp_cali.attr,
	&dev_attr_disp_cali_enable.attr,
	&dev_attr_hal_disp_color_enable.attr,
	&dev_attr_ddic_color_mode_supported.attr,
	NULL,
};

static struct attribute_group htc_extend_attr_group = {
	.attrs = htc_extend_attrs,
};

void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd)
{
	int rc;
	struct mdss_panel_data *pdata = dev_get_platdata(&mfd->pdev->dev);
	struct calibration_gain *gain = &(pdata->panel_info.cali_gain);

	pr_info("htc_register_attrs\n");

	rc = sysfs_create_group(led_kobj, &htc_extend_attr_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	/* rgb calibration initial value*/
	if (RGB_GAIN_CHECK(gain->R) && RGB_GAIN_CHECK(gain->G) && RGB_GAIN_CHECK(gain->B)) {
		aux_gain.R = gain->R;
		aux_gain.G = gain->G;
		aux_gain.B = gain->B;
	}

	/* backlight calibration initial value*/
	if (BRI_GAIN_CHECK(gain->BKL))
		aux_gain.BKL = gain->BKL;

	return;
}

void htc_reset_status(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		htc_attr_status[i].cur_value = htc_attr_status[i].def_value;
	}

	return;
}

void htc_register_camera_bkl(int level)
{
	camera_backlight_value = level;
}

void htc_set_cabc(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (htc_attr_status[CABC_INDEX].req_value > 2)
		return;

	if (!ctrl_pdata->cabc_off_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_ui_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_video_cmds.cmds)
		return;

	if (!force && (htc_attr_status[CABC_INDEX].req_value == htc_attr_status[CABC_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[CABC_INDEX].req_value == 0) {
		cmdreq.cmds = ctrl_pdata->cabc_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_off_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 1) {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 2) {
		cmdreq.cmds = ctrl_pdata->cabc_video_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_video_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[CABC_INDEX].cur_value = htc_attr_status[CABC_INDEX].req_value;
	pr_info("%s: cabc mode=%d\n", __func__, htc_attr_status[CABC_INDEX].cur_value);
	return;
}

bool htc_is_burst_bl_on(struct msm_fb_data_type *mfd, int value)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct htc_backlight1_table *brt_bl_table = &panel_info->brt_bl_table;
	int size = brt_bl_table->size;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	/* Support burst mode backlight  */
	if (mfd->panel_info->burst_bl_value == 0)
		return false;

	if(!size || size < 2 || !brt_bl_table->brt_data)
		return false;

	if (ctrl_pdata->burst_on_level == 0 || ctrl_pdata->burst_off_level == 0)
		return false;

	if (htc_attr_status[BURST_SWITCH_INDEX].req_value < 0)
		return false;

	htc_attr_status[BURST_SWITCH_INDEX].cur_value = htc_attr_status[BURST_SWITCH_INDEX].req_value;

	pr_debug("%s burst level=%d, value=%d, max brt=%d\n", __func__,
		htc_attr_status[BURST_SWITCH_INDEX].cur_value, value, brt_bl_table->brt_data[size - 1]);

	if (htc_attr_status[BURST_SWITCH_INDEX].cur_value >= ctrl_pdata->burst_on_level) {
		if(value >= brt_bl_table->brt_data[size - 1])
			return true;
	}

	return false;
}

void htc_update_bl_cali_data(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct calibration_gain *gain = NULL;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct htc_backlight1_table *brt_bl_table = &panel_info->brt_bl_table;
	int size = brt_bl_table->size;
	int bl_lvl = 0;
	u16 *bl_data_raw;
	u16 tmp_cali_value = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	gain = &(pdata->panel_info.cali_gain);

	if (htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value == htc_attr_status[BL_CALI_ENABLE_INDEX].req_value)
		return;

	htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value = htc_attr_status[BL_CALI_ENABLE_INDEX].req_value;

	/* update backlight calibration data from user change*/
	if ((aux_gain.BKL != gain->BKL)) {
		gain->BKL = aux_gain.BKL;
		pr_info("%s change bkl calibration value, bkl=%d\n", __func__, gain->BKL);
	}

	if (!BRI_GAIN_CHECK(gain->BKL)) {
		pr_info("%s bkl=%d out of range\n", __func__, gain->BKL);
		return;
	}

	brt_bl_table->apply_cali = htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value;

	/* free the old calibrated data first, then restore raw bl data again */
	kfree(brt_bl_table->bl_data);
	brt_bl_table->bl_data = kzalloc(size * sizeof(u16), GFP_KERNEL);
	if (!brt_bl_table->bl_data) {
		pr_err("unable to allocate memory for bl_data\n");
		return;
	}
	memcpy(brt_bl_table->bl_data, brt_bl_table->bl_data_raw, size * sizeof(u16));

	/* Not define brt table */
	if(!size || size < 2 || !brt_bl_table->brt_data || !brt_bl_table->bl_data)
		return;

	bl_data_raw = brt_bl_table->bl_data_raw;

	/* calibrates on Min and Max node */
	tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[0], gain->BKL);
	brt_bl_table->bl_data[0] = VALID_CALI_BKLT(tmp_cali_value, 0, bl_data_raw[1]);
	tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[size - 1], gain->BKL);
	brt_bl_table->bl_data[size - 1] = VALID_CALI_BKLT(tmp_cali_value, bl_data_raw[size - 2], panel_info->bl_max);

	/* Calibrate default brightness */
	if (brt_bl_table->apply_cali) {
		tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[1], gain->BKL);
		brt_bl_table->bl_data[1] = VALID_CALI_BKLT(tmp_cali_value, bl_data_raw[0], bl_data_raw[2]);
	}

	if (mfd->bl_level && mfd->last_bri1) {
		/* always calibrates based on last time brightness value rather than calibrated brightness */
		bl_lvl = mdss_backlight_trans(mfd->last_bri1, mfd->panel_info, true);

		/* Update the brightness when bl_cali be set */
		if (bl_lvl) {
			mfd->allow_bl_update = false;
			mfd->unset_bl_level = bl_lvl;
		}
	}

	pr_info("%s bl_cali=%d, unset_bl_level=%d \n", __func__, gain->BKL,  mfd->unset_bl_level);
}

int htc_update_rgb_cali_data(struct msm_fb_data_type *mfd, struct dsi_panel_cmds raw_cmds)
{
	struct mdss_panel_data *pdata;
	struct calibration_gain *gain = NULL;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	char *rgb_raw_payload = NULL, *rgbcmy_raw_payload = NULL;
	char *rgb_cali_payload = NULL, *rgbcmy_cali_payload = NULL;
	int rgb_cmd_loca= 0, rgbcmy_cmd_loca = 0, rgb_size = 0, rgbcmy_size = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	gain = &(pdata->panel_info.cali_gain);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->disp_cali_cmds.cmds || !ctrl_pdata->color_rgb_loca)
		return false;

	htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value = htc_attr_status[RGB_CALI_ENABLE_INDEX].req_value;

	/* update rgb calibration data from user change*/
	if ((aux_gain.R != gain->R) || (aux_gain.G != gain->G) || (aux_gain.B != gain->B)) {
		gain->R = aux_gain.R;
		gain->G = aux_gain.G;
		gain->B = aux_gain.B;
		pr_info("%s change calibration value, RGB(0x%x, 0x%x, 0x%x) \n",
				__func__, gain->R, gain->G, gain->B);
	}

	if (!RGB_GAIN_CHECK(gain->R) || !RGB_GAIN_CHECK(gain->G) || !RGB_GAIN_CHECK(gain->B)) {
		pr_info("%s RGB(0x%x, 0x%x, 0x%x) out of range\n", __func__, gain->R, gain->G, gain->B);
		return false;
	}

	rgb_cmd_loca = ctrl_pdata->color_rgb_loca;
	rgbcmy_cmd_loca = ctrl_pdata->color_rgbcmy_loca;
	if (!rgb_cmd_loca || !rgbcmy_cmd_loca) {
		pr_err("invalid rgb cmd location\n");
		return false;
	}
	rgb_raw_payload = raw_cmds.cmds[rgb_cmd_loca].payload;
	rgb_cali_payload = ctrl_pdata->disp_cali_cmds.cmds[rgb_cmd_loca].payload;
	rgb_size = ctrl_pdata->disp_cali_cmds.cmds[rgb_cmd_loca].dchdr.dlen;
	rgbcmy_raw_payload = raw_cmds.cmds[rgbcmy_cmd_loca].payload;
	rgbcmy_cali_payload = ctrl_pdata->disp_cali_cmds.cmds[rgbcmy_cmd_loca].payload;
	rgbcmy_size = ctrl_pdata->disp_cali_cmds.cmds[rgbcmy_cmd_loca].dchdr.dlen;
	if (!rgb_raw_payload || !rgb_cali_payload || !rgb_size || !rgb_raw_payload || !rgb_cali_payload || !rgb_size) {
		pr_err("invalid rgb cali data\n");
		return false;
	}
	memcpy(rgb_cali_payload, rgb_raw_payload, rgb_size * sizeof(char));
	memcpy(rgbcmy_cali_payload, rgbcmy_raw_payload, rgbcmy_size * sizeof(char));

	/* Apply calibration data to cmds only if calibration enabled */
	if (htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value) {
		/* rgb commands include address:C3 data:R G B */
		if (ctrl_pdata->disp_cali_cmds.cmds[rgb_cmd_loca].dchdr.dlen == 4) {
			rgb_cali_payload[1] = RGB_CALIBRATION(rgb_raw_payload[1], gain->R);
			rgb_cali_payload[2] = RGB_CALIBRATION(rgb_raw_payload[2], gain->G);
			rgb_cali_payload[3] = RGB_CALIBRATION(rgb_raw_payload[3], gain->B);
		} else{
			pr_info("%s rgb command length invalid\n",__func__);
			return false;
		}

		/* rgb commands include address:C3 data:R 00 00 00 G 00 00 00 B 00 G B R 00 B R G 00 */
		if (ctrl_pdata->disp_cali_cmds.cmds[rgbcmy_cmd_loca].dchdr.dlen == 19) {
			rgbcmy_cali_payload[1] = rgbcmy_cali_payload[13] = rgbcmy_cali_payload[16] = RGB_CALIBRATION(rgb_raw_payload[1], gain->R);
			rgbcmy_cali_payload[5] = rgbcmy_cali_payload[11] = rgbcmy_cali_payload[17] = RGB_CALIBRATION(rgb_raw_payload[2], gain->G);
			rgbcmy_cali_payload[9] = rgbcmy_cali_payload[12] = rgbcmy_cali_payload[15] = RGB_CALIBRATION(rgb_raw_payload[3], gain->B);
		} else{
			pr_info("%s rgbcmy command length invalid\n",__func__);
			return false;
		}
	}

	pr_info("%s apply=%d calibration value, RGB(0x%x, 0x%x, 0x%x) \n",
			__func__, htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value, gain->R, gain->G, gain->B);
	return true;
}

void htc_set_color_temp(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;
	int req_mode = 0;
	int i = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_temp_cnt)
		return;

	for (i = 0; i < ctrl_pdata->color_temp_cnt ; i++) {
		if (!ctrl_pdata->color_temp_cmds[i].cmds)
			return;
	}

	if (htc_attr_status[COLOR_TEMP_INDEX].req_value >= ctrl_pdata->color_temp_cnt)
		return;

	if (!force && (htc_attr_status[COLOR_TEMP_INDEX].req_value == htc_attr_status[COLOR_TEMP_INDEX].cur_value) &&
			(htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value == htc_attr_status[RGB_CALI_ENABLE_INDEX].req_value))
		return;

	req_mode = htc_attr_status[COLOR_TEMP_INDEX].req_value;

	htc_attr_status[COLOR_TEMP_INDEX].cur_value = htc_attr_status[COLOR_TEMP_INDEX].req_value;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if(htc_update_rgb_cali_data(mfd, ctrl_pdata->color_temp_cmds[req_mode])) {
		cmdreq.cmds = ctrl_pdata->disp_cali_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->disp_cali_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->color_temp_cmds[req_mode].cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_temp_cmds[req_mode].cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	pr_info("%s: color temp mode=%d\n", __func__, htc_attr_status[COLOR_TEMP_INDEX].cur_value);
	return;
}

void htc_set_color_profile(struct mdss_panel_data *pdata, bool force)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	if (!ctrl_pdata->color_default_cmds.cmd_cnt || !ctrl_pdata->color_srgb_cmds.cmd_cnt)
		return;

	if ((htc_attr_status[COLOR_PROFILE_INDEX].req_value != SRGB_MODE) && (htc_attr_status[COLOR_PROFILE_INDEX].req_value != DEFAULT_MODE))
		return;

	if (!force && (htc_attr_status[COLOR_PROFILE_INDEX].req_value == htc_attr_status[COLOR_PROFILE_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[COLOR_PROFILE_INDEX].req_value == SRGB_MODE) {
		cmdreq.cmds = ctrl_pdata->color_srgb_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_srgb_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->color_default_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_default_cmds.cmd_cnt;
	}

	if (force)
		cmdreq.flags = CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	else
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_PROFILE_INDEX].cur_value = htc_attr_status[COLOR_PROFILE_INDEX].req_value;
	pr_info("%s: color profile mode=%d\n", __func__, htc_attr_status[COLOR_PROFILE_INDEX].cur_value);
	return;
}

static DEFINE_MUTEX(dsi_notify_sem);
int dsi_register_notifier(struct dsi_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func) {
		pr_err("%s: invalid parameter\n",__func__);
		return -EINVAL;
	}

	mutex_lock(&dsi_notify_sem);
	list_add(&notifier->dsi_notifier_link,
		&g_dsi_notifier_list);
	mutex_unlock(&dsi_notify_sem);
	return 0;
}
EXPORT_SYMBOL(dsi_register_notifier);

void send_dsi_status_notify(int status)
{
	static struct dsi_status_notifier *dsi_notifier;

	mutex_lock(&dsi_notify_sem);
	list_for_each_entry(dsi_notifier,
		&g_dsi_notifier_list,
		dsi_notifier_link) {
		dsi_notifier->func(status);
	}
	mutex_unlock(&dsi_notify_sem);

	pr_info("%s: status=%d\n",__func__, status);
}

void htc_vreg_vol_switch(struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool enable)
{
	int ret = 0;

	if (ctrl_pdata->lcmio_src_vreg.vreg)
	{
		if (ctrl_pdata->lcmio_src_enabled == enable)
			return;

		if (enable) {
			regulator_set_voltage(
				ctrl_pdata->lcmio_src_vreg.vreg,
				ctrl_pdata->lcmio_src_vreg.on_min_voltage,
				ctrl_pdata->lcmio_src_vreg.max_voltage);
			ret = regulator_enable(ctrl_pdata->lcmio_src_vreg.vreg);
		} else {
			ret = regulator_disable(ctrl_pdata->lcmio_src_vreg.vreg);
			regulator_set_voltage(
				ctrl_pdata->lcmio_src_vreg.vreg,
				ctrl_pdata->lcmio_src_vreg.off_min_voltage,
				ctrl_pdata->lcmio_src_vreg.max_voltage);
			regulator_set_load(ctrl_pdata->lcmio_src_vreg.vreg, 0);
		}

		if (ret < 0) {
			pr_err("%pS->%s: %s %s failed\n",
					__builtin_return_address(0), __func__,
					ctrl_pdata->lcmio_src_vreg.vreg_name,
					enable?"enable":"disable");
		} else
			ctrl_pdata->lcmio_src_enabled = enable;
	}
}

void htc_vreg_init(struct platform_device *ctrl_pdev,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (ctrl_pdata->lcmio_src_vreg.vreg_name && ctrl_pdata->lcmio_src_vreg.on_min_voltage &&
			ctrl_pdata->lcmio_src_vreg.off_min_voltage && ctrl_pdata->lcmio_src_vreg.max_voltage)
	{
		ctrl_pdata->lcmio_src_vreg.vreg = regulator_get(&ctrl_pdev->dev,
				ctrl_pdata->lcmio_src_vreg.vreg_name);

		rc = PTR_RET(ctrl_pdata->lcmio_src_vreg.vreg);
		if (rc) {
			pr_err("%pS->%s: %s get failed. rc=%d\n",
					__builtin_return_address(0), __func__,
					ctrl_pdata->lcmio_src_vreg.vreg_name, rc);
		} else {
			regulator_set_voltage(
				ctrl_pdata->lcmio_src_vreg.vreg,
				ctrl_pdata->lcmio_src_vreg.on_min_voltage,
				ctrl_pdata->lcmio_src_vreg.max_voltage);
			rc = regulator_enable(ctrl_pdata->lcmio_src_vreg.vreg);
			if (rc < 0) {
				pr_err("%pS->%s: %s enable failed\n",
					__builtin_return_address(0), __func__,
					ctrl_pdata->lcmio_src_vreg.vreg_name);
			} else
				ctrl_pdata->lcmio_src_enabled = true;
		}
	}
}


void htc_mdss_dsi_parse_esd_params(struct device_node *np)
{

	int rc;
	const char *string;

	rc = of_property_read_string(np,"htc,esd-status-check-mode", &string);

	if (!rc){

		if (!strcmp(string, "tp_reg_check")) {
#ifdef	CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
			lcm_rst_callback_register(htc_report_panel_dead);
#endif
		}else{
			pr_err("No valid htc esd-status-check-mode string\n");
		}

	}else{
		pr_err("No valid htc esd-status-check-mode property in DTS \n");
	}

}


void htc_hal_color_feature_enabled(bool enabled)
{
	if (enabled){
		hal_color_feature_enabled = 1;
	}
}

void htc_ddic_color_mode_supported(bool enabled)
{
	if (enabled){
		ddic_color_mode_supported = 1;
	}
}

static void dimming_do_work(struct work_struct *work)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	if (!g_mfd) {
		pr_err("%s no mfd resource \n" , __func__);
		return;
	}

	pdata = dev_get_platdata(&g_mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	memset(&cmdreq, 0, sizeof(cmdreq));

	cmdreq.cmds = ctrl_pdata->dimming_on_cmds.cmds;
	cmdreq.cmds_cnt = ctrl_pdata->dimming_on_cmds.cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	pr_debug("dimming on\n");
}

void htc_dimming_on(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->dimming_on_cmds.cmds)
		return;


	schedule_delayed_work(&dimming_work, msecs_to_jiffies(1000));
	return;
}

void htc_dimming_off(void)
{
	/* Delete dimming workqueue */
	cancel_delayed_work_sync(&dimming_work);
	pr_debug("dimming off\n");
}

