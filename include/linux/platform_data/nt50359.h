/*
 * NOVATEK NT50359 Panel Power IC
 *
 *			Copyright (C) 2017 HTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _NT50359_H
#define _NT50359_H

enum NT50359_chip_id {
	NT50359,
	NT_MAX,
};

enum power_status{
	REGULATOR_POWER_UNKNOW,
	REGULATOR_POWER_ON,
	REGULATOR_POWER_OFF,
};

/* regcmd function struct*/
struct nt50359_regcmd{
	unsigned int address;
	unsigned int parameter1;
};

struct nt50359_regcmd_sets{
	int power_on_cmds_num;
	struct nt50359_regcmd *power_on_check_cmds;
	struct regmap *regmap;
};

struct nt50359{
	struct regmap *regmap;
	struct i2c_client *client;
	struct device *dev;
	bool reg_check_enable;
	int retry_times;
	int avee_gpio;
	struct nt50359_regcmd_sets power_on_check_cmd_sets;
};


#endif
