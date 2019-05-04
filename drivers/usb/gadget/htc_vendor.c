/*
 * Copyright (C) 2011 HTC, Inc.
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

struct diag_context _context;
int vendor_com_type = 0;

static struct diag_context *get_modem_ctxt(void)
{
	return &_context;
}

static int if_route_to_userspace(struct diag_context *ctxt, unsigned int cmd)
{
	//unsigned long flags;
	int rc = -1;
	unsigned short tmp;
	unsigned char cmd_id, cmd_num;

	tmp = (unsigned short)cmd;

	cmd_num = (unsigned char)(tmp >> 8);
	cmd_id = (unsigned char)(tmp & 0x00ff);

	if (!ctxt->opened || cmd_id == 0)
		rc = 0;

	if (cmd_id == 0xfb && cmd_num == 0x2)
		rc = 2;
	else if (cmd_id == 0xfb && cmd_num == 0x3)
		rc = 3;

	return rc;
}

int get_vendor_request(void)
{
	return vendor_com_type;
}

void clear_hcmd_cmd(void)
{
	vendor_com_type = 0;
	return;
}
