/*
 * Copyright (C) 2015 HTC Corporation.  All rights reserved.
 *
 *     @file   /kernel/drivers/htc_debug/stability/htc_reboot_block.c
 *
 * This software is distributed under dual licensing. These include
 * the GNU General Public License version 2 and a commercial
 * license of HTC.  HTC reserves the right to change the license
 * of future releases.
 *
 * Unless you and HTC execute a separate written software license
 * agreement governing use of this software, this software is licensed
 * to you under the terms of the GNU General Public License version 2,
 * available at {link to GPL license term} (the "GPL").
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/reboot.h>
#include <linux/fcntl.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>

/* This constant is used in bootloader to decide actions. */
#define RESTART_REASON_BOOT_BASE			0x77665500
#define RESTART_REASON_BOOTLOADER			(RESTART_REASON_BOOT_BASE | 0x00)
#define RESTART_REASON_REBOOT				(RESTART_REASON_BOOT_BASE | 0x01)
#define RESTART_REASON_DM_VERITY_DEVICE_CORRUPTED	(RESTART_REASON_BOOT_BASE | 0x04)
#define RESTART_REASON_RECOVERY				(RESTART_REASON_BOOT_BASE | 0x02)
#define RESTART_REASON_RAMDUMP				(RESTART_REASON_BOOT_BASE | 0xAA)
#define RESTART_REASON_ERASE_FLASH			(RESTART_REASON_BOOT_BASE | 0xEF)

static char pname[128];
static uint64_t poffset;
const struct device *dev;

struct reboot_block {
	uint32_t reason;
	char     msg[64];
};

static int reboot_block_command(int reason, const char* msg)
{
	struct reboot_block block;
	char filename[128];
	struct file *filp;
	ssize_t nwrite;
	int ret = 0;

	msg = msg ? : "";

	/*DM-verity cannot access misc for it violates selinux policy*/
	if (reason == RESTART_REASON_DM_VERITY_DEVICE_CORRUPTED) {
		dev_info(dev, "dm-verity device corrupted, skipping writing to misc.\n");
		return ret;
	}

	dev_info(dev, "reason=%08x msg=%s save to /%s+%08llx\n",
			reason, msg, pname, poffset);

	block.reason = reason;
	memset(&block.msg[0], 0, sizeof(block.msg));
	strlcpy(&block.msg[0], msg, strlen(msg));

	snprintf(filename, sizeof(filename),
			"/dev/block/bootdevice/by-name/%s", pname);

	filp = filp_open(filename, O_RDWR | O_SYNC, 0);
	if (IS_ERR(filp)) {
		dev_err(dev, "unable to open file: %s\n", filename);
		return PTR_ERR(filp);
	}

	nwrite = kernel_write(filp,
			(const char*) &block, sizeof(block),
			poffset);
	if (nwrite == sizeof(block))
		dev_info(dev, "wrote reason: %08x\n", reason);
	else {
		dev_err(dev, "kernel_write failed: %zd\n", nwrite);
		ret = -1;
		goto err;
	}

err:
	filp_close(filp, NULL);
	return ret;
}

/*
 * This restart constant is used for oem commands.
 * The actual value is parsed from reboot commands.
 * RIL FATAL will use oem-99 to restart a device.
 */
#define RESTART_REASON_OEM_BASE         0x6f656d00
#define RESTART_REASON_DOWNLOAD         (RESTART_REASON_OEM_BASE | 0xE0)
#define RESTART_REASON_FTM              (RESTART_REASON_OEM_BASE | 0xE1)

static struct cmd_reason_map {
	char* cmd;
	u32 reason;
} cmd_reason_map[] = {
	{ "",               RESTART_REASON_REBOOT },
	{ "bootloader",     RESTART_REASON_BOOTLOADER },
	{ "recovery",       RESTART_REASON_RECOVERY },
	{ "force-hard",     RESTART_REASON_RAMDUMP },
	{ "force-dog-bark", RESTART_REASON_RAMDUMP },
	{ "eraseflash",     RESTART_REASON_ERASE_FLASH },
	{ "download",       RESTART_REASON_DOWNLOAD },
	{ "ftm",            RESTART_REASON_FTM },
	{ "force-dog-bark", RESTART_REASON_RAMDUMP },
	{ "power-key-force-hard", RESTART_REASON_RAMDUMP },
};

#define OEM_CMD_FMT "oem-%02x"

static int reboot_block_command_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int i;
	int code;
	const char* cmd = (char*) data;

	if (event != SYS_RESTART)
		goto end;

	/*
	 * NOTE: `data' is NULL when reboot w/o command or shutdown
	 */
	cmd = cmd ? : "";

	/* standard reboot command */
	for (i = 0; i < ARRAY_SIZE(cmd_reason_map); i++)
		if (!strcmp(cmd, cmd_reason_map[i].cmd)) {
			reboot_block_command(
					cmd_reason_map[i].reason,
					cmd_reason_map[i].cmd);
			goto end;
		}

	/* oem reboot command */
	if (1 == sscanf(cmd, OEM_CMD_FMT, &code)) {
		/* oem-93, 94, 95, 96, 97, 98, 99 are RIL fatal */
		if ((code >= 0x93) && (code <= 0x98))
			code = 0x99;

		reboot_block_command(RESTART_REASON_OEM_BASE | code, cmd);
		goto end;
	}

	/* unknown reboot command */
	dev_warn(dev, "Unknown restart command: %s\n", cmd);
	reboot_block_command(RESTART_REASON_REBOOT, "");

end:
	return NOTIFY_DONE;
}

static struct notifier_block reboot_block_command_nb = {
	.notifier_call = reboot_block_command_call,
};

static int reboot_block_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const char *tmp_str;
	int ret;

	dev = &pdev->dev;

	ret = of_property_read_string(node, "pname", &tmp_str);
	if (ret) {
		dev_err(dev, "can't read fdt prop `pname', error = %d\n", ret);
		return ret;
	}
	strlcpy(pname, tmp_str, sizeof(pname));

	ret = of_property_read_u64(node, "poffset", &poffset);
	if (ret) {
		dev_err(dev, "can't read fdt prop `poffset', error = %d\n", ret);
		return ret;
	}

	ret = register_reboot_notifier(&reboot_block_command_nb);
	if (ret) {
		dev_err(dev, "can't register reboot notifier, error = %d\n", ret);
		return ret;
	}

	return 0;
}

static struct of_device_id reboot_block_match[] = {
	{.compatible = "htc,reboot_block",},
	{}
};

static struct platform_driver reboot_block_driver = {
	.probe = reboot_block_probe,
	.driver = {
		.name = "htc_reboot_block",
		.owner = THIS_MODULE,
		.of_match_table = reboot_block_match,
	},
};

static int htc_reboot_block_init(void)
{
	return platform_driver_register(&reboot_block_driver);
}
/* MMC driver is ready after subsys_initcall */
fs_initcall(htc_reboot_block_init);
