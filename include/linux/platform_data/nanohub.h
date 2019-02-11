#ifndef __LINUX_PLATFORM_DATA_NANOHUB_H
#define __LINUX_PLATFORM_DATA_NANOHUB_H

#include <linux/types.h>

struct nanohub_flash_bank {
	int bank;
	u32 address;
	size_t length;
};

struct nanohub_platform_data {
	u32 wakeup_gpio;
	u32 nreset_gpio;
	u32 boot0_gpio;
	u32 irq1_gpio;
	u32 irq2_gpio;
	u32 spi_cs_gpio;
	u32 bl_addr;
	u32 flash_page_size;
	u32 num_flash_banks;
	struct nanohub_flash_bank *flash_banks;
	u32 num_shared_flash_banks;
	struct nanohub_flash_bank *shared_flash_banks;

/* HTC_START */
	u32 handshaking_gpio;
	u32 vibrate_ms;
	u8  motion_sensor_placement;
#ifdef CONFIG_NANOHUB_FLASH_GPIO_CONTROL
	u32 gpio1;
	u32 gpio2;
#endif
/* HTC_END */
};

#endif /* __LINUX_PLATFORM_DATA_NANOHUB_H */
