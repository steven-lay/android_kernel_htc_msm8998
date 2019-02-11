/*
 * Copyright (C) 2016 Google, Inc.
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

#include <linux/platform_data/nanohub.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>

#include "main.h"
#include "bl.h"

#define MAX_BUFFER_SIZE		1024
#define MAX_FLASH_BANKS		16
#define READ_ACK_TIMEOUT	100000

#define FLASH_BL_OP_INSTRUMENTED


#if defined(FLASH_BL_OP_INSTRUMENTED)

#include <linux/ktime.h>


struct nanohub_stat_bl_op {
	uint64_t terase;	/* accumulated time in ns */
	uint32_t serase;    /* accumulated size in item/byte*/
	uint32_t cerase;    /* number of operation */
	uint64_t twrite;
	uint32_t swrite;
	uint32_t cwrite;
	uint64_t tread;
	uint32_t sread;
	uint32_t cread;
};

struct nanohub_stat_bl_op stat;

static void stat_bl_reset(void) {
	stat.terase = stat.twrite = stat.tread = 0;
	stat.serase = stat.swrite = stat.sread = 0;
	stat.cerase = stat.cwrite = stat.cread = 0;
}

static void stat_bl_log(void) {
	pr_info("nanohub: MCU flash (Execution time) E:%lldms/%ld/%ld W:%lldms/%ld/%ld R:%lldms/%ld/%ld\n",
	  (long long)stat.terase/1000000, (long)stat.serase, (long)stat.cerase,
	  (long long)stat.twrite/1000000, (long)stat.swrite, (long)stat.cwrite,
	   (long long)stat.tread/1000000, (long)stat.sread, (long)stat.cread);
}

static inline void stat_bl_inc_erase(s64 dur, uint32_t l)
{
	stat.terase += dur;
	stat.serase += l;
	stat.cerase += 1;
}

static inline void stat_bl_inc_write(s64 dur, uint32_t l)
{
	stat.twrite += dur;
	stat.swrite += l;
	stat.cwrite += 1;

}

static inline void stat_bl_inc_read(s64 dur, uint32_t l)
{
	stat.tread += dur;
	stat.sread += l;
	stat.cread += 1;
}


#define LOG_STAT_TIME_RESET() \
		stat_bl_reset()

#define LOG_STAT_TIME_LOG() \
		stat_bl_log()

#define LOG_STAT_TIME_DEC       ktime_t _ts, _te;
#define LOG_STAT_TIME_ENTRY() 	_ts = ktime_get();
#define LOG_STAT_TIME_EXIT()    _te = ktime_get();

#define LOG_STAT_TIME_INC_ERASE(a) \
		stat_bl_inc_erase(ktime_to_ns(ktime_sub(_te, _ts)), (a))

#define LOG_STAT_TIME_INC_WRITE(a) \
		stat_bl_inc_write(ktime_to_ns(ktime_sub(_te, _ts)), (a))


#define LOG_STAT_TIME_INC_READ(a) \
		stat_bl_inc_read(ktime_to_ns(ktime_sub(_te, _ts)), (a))

#else

#define LOG_STAT_TIME_RESET()
#define LOG_STAT_TIME_LOG()

#define LOG_STAT_TIME_DEC
#define LOG_STAT_TIME_ENTRY()
#define LOG_STAT_TIME_EXIT()

#define LOG_STAT_TIME_INC_ERASE(a)
#define LOG_STAT_TIME_INC_WRITE(a)
#define LOG_STAT_TIME_INC_READ(a)

#endif


static uint8_t write_len(struct nanohub_data *data, int len)
{
	uint8_t buffer[sizeof(uint8_t) + 1];

	buffer[0] = len - 1;

	return data->bl.write_data(data, buffer, sizeof(uint8_t));
}

static uint8_t write_cnt(struct nanohub_data *data, uint16_t cnt)
{
	uint8_t buffer[sizeof(uint16_t) + 1];

	buffer[0] = (cnt >> 8) & 0xFF;
	buffer[1] = (cnt >> 0) & 0xFF;

	return data->bl.write_data(data, buffer, sizeof(uint16_t));
}

static uint8_t write_addr(struct nanohub_data *data, uint32_t addr)
{
	uint8_t buffer[sizeof(uint32_t) + 1];

	buffer[0] = (addr >> 24) & 0xFF;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;

	return data->bl.write_data(data, buffer, sizeof(uint32_t));
}

/* write length followed by the data */
static uint8_t write_len_data(struct nanohub_data *data, int len,
			      const uint8_t *buf)
{
	uint8_t buffer[sizeof(uint8_t) + 256 + sizeof(uint8_t)];

	buffer[0] = len - 1;

	memcpy(&buffer[1], buf, len);

	return data->bl.write_data(data, buffer, sizeof(uint8_t) + len);
}

/* keep checking for ack until we receive a ack or nack */
static uint8_t read_ack_loop(struct nanohub_data *data)
{
	uint8_t ret;
	int32_t timeout = READ_ACK_TIMEOUT;

	do {
		ret = data->bl.read_ack(data);
		if (ret != CMD_ACK && ret != CMD_NACK)
			schedule();
	} while (ret != CMD_ACK && ret != CMD_NACK && timeout-- > 0);

	return ret;
}

uint8_t nanohub_bl_sync(struct nanohub_data *data)
{
	return data->bl.sync(data);
}

int nanohub_bl_open(struct nanohub_data *data)
{
	int ret = -1;

	data->bl.tx_buffer = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
	if (!data->bl.tx_buffer)
		goto out;

	data->bl.rx_buffer = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
	if (!data->bl.rx_buffer)
		goto free_tx;

	ret = data->bl.open(data);
	if (!ret)
		goto out;

	kfree(data->bl.rx_buffer);
free_tx:
	kfree(data->bl.tx_buffer);
out:
	return ret;
}

void nanohub_bl_close(struct nanohub_data *data)
{
	data->bl.close(data);
	kfree(data->bl.tx_buffer);
	kfree(data->bl.rx_buffer);
}

static uint8_t write_bank(struct nanohub_data *data, int bank, uint32_t addr,
			  const uint8_t *buf, size_t length)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status = CMD_ACK;
	uint32_t offset;

	if (addr <= pdata->flash_banks[bank].address) {
		offset = pdata->flash_banks[bank].address - addr;
		if (addr + length >
		    pdata->flash_banks[bank].address +
		    pdata->flash_banks[bank].length)
			status =
			    nanohub_bl_write_memory(data,
						    pdata->flash_banks[bank].
						    address,
						    pdata->flash_banks[bank].
						    length, buf + offset);
		else
			status =
			    nanohub_bl_write_memory(data,
						    pdata->flash_banks[bank].
						    address, length - offset,
						    buf + offset);
	} else {
		if (addr + length >
		    pdata->flash_banks[bank].address +
		    pdata->flash_banks[bank].length)
			status =
			    nanohub_bl_write_memory(data, addr,
						    pdata->flash_banks[bank].
						    address +
						    pdata->flash_banks[bank].
						    length - addr, buf);
		else
			status =
			    nanohub_bl_write_memory(data, addr, length, buf);
	}

	return status;
}

static uint8_t nanohub_bl_get_id_cmd(struct nanohub_data *data, uint16_t *id)
{
	uint8_t status;
	uint8_t buffer[256];

	data->bl.write_cmd(data, data->bl.cmd_get_id);
	status = data->bl.read_ack(data);
	if (status == CMD_ACK) {
			status = data->bl.read_data(data, buffer, 3);
			if ((status == CMD_ACK) && (buffer[0] == 1))
					*id = (buffer[1] << 8) | buffer[2];
			else
				*id = 0;
	}

	if (!*id)
		status = CMD_NACK;

	status = data->bl.read_ack(data);

	return status;
}

static uint8_t nanohub_bl_get_version_cmd(struct nanohub_data *data, uint8_t *version)
{
	uint8_t status;

	data->bl.write_cmd(data, data->bl.cmd_get_version);
	status = data->bl.read_ack(data);
	if (status == CMD_ACK)
		data->bl.read_data(data, version, 1);
	status = data->bl.read_ack(data);

	return status;
}

static uint8_t nanohub_bl_get_page_idx(struct nanohub_data *data,
										uint32_t addr, uint32_t psize,
										uint32_t *page)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	if ((!psize) || (addr < pdata->flash_banks[0].address) ||
			(((addr - pdata->flash_banks[0].address) % psize) != 0)) {
			pr_err("%s: ERR addr is not page-aligned (@0x%08x/@0x%08x),%d)\n",
				   __func__, addr, pdata->flash_banks[0].address, (int)psize);
			return CMD_NACK;
	}

	*page = (addr - pdata->flash_banks[0].address) / psize;

	return CMD_ACK;
}

static uint32_t nanohub_bl_cmp(uint8_t *ptr, const uint8_t *image,
							   size_t length)
{
	int i;
	uint32_t nbdiff = 0;


	for (i=0; i<length; i++) {
			if (ptr[i] != image[i]) {
					if (nbdiff < 16) {
							pr_debug("nanohub: CMP offset=%d %x instead %x\n",
									i, ptr[i], image[i]);
					}
					nbdiff++;
#if !defined(DEBUG)
					break;
#endif
			}
	}
	if (nbdiff) {
			pr_debug("nanohub: CMP nbdiff = %d/%d\n", (int)nbdiff, (int)length);
	}

	return nbdiff;
}

static uint8_t nanohub_bl_erase_page(struct nanohub_data *data, uint32_t addr,
									 uint32_t psize)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status;
	uint32_t page = 0;

	status = nanohub_bl_get_page_idx(data, addr, psize, &page);

	if (status != CMD_ACK)
		return status;

	pr_debug("nanohub: ERASE page %d (@0x%08X/@0x%08X)\n",
			page, addr, pdata->flash_banks[0].address);

	status = nanohub_bl_erase_sector(data, page);

	return status;
}

static uint8_t nanohub_bl_write_page(struct nanohub_data *data, uint32_t addr,
									 const uint8_t *image, uint32_t len,
									 uint32_t psize)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status;

	uint32_t offset;
    uint32_t lw;

	if ((!psize) || (len > psize) ||
			(((addr - pdata->flash_banks[0].address) % psize) != 0)) {
			pr_err("%s: ERR addr is not page-aligned (@0x%08x,%d) (psize=%d)\n",
				   __func__, addr, (int)len, (int)psize);
			return CMD_NACK;
	}

	pr_debug("nanohub: WRITE page (@0x%08X/%d)\n", addr, len);

	offset = 0;
	status = CMD_ACK;
	lw = 256;

	while ((status == CMD_ACK) && (offset < len)) {
			status = nanohub_bl_write_memory(data, addr, lw, &image[offset]);
			/* next */
			addr   += 256;
			offset += 256;

			if (((offset < len)) && (offset + lw > len)) {
					lw = len - offset;
			}
	}

	return status;
}

static int nanohub_bl_page_in_sectors(struct nanohub_data *data, uint32_t addr,
									  size_t length)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	int i;

	for (i = 0; i < pdata->num_flash_banks; i++) {
			if (addr >= pdata->flash_banks[i].address &&
					(addr + length) <=
					pdata->flash_banks[i].address +
					pdata->flash_banks[i].length) {
					return (int)pdata->flash_banks[i].bank;
			}
	}

	return -1;
}

/*
 * Following table is a STM32L4 code to guarantee that the PEMPTY bit
 * (FLASH_SR) register is correctly defined when the FLASH has been
 * programmed. It avoids a POR cycle on the empty device after the
 * FLASH programming sequence (see RM0394)
 * Off-line build process is used to generate (base @ and used registers
 * are device dependents).
 */

#define BASE_SRAM_CODE (uint32_t)0x20003100
const char sram_code[] =
	"\x00\x31\x00\x20\x09\x31\x00\x20\x4f\xf0\x00\x62\x08\x49\x12\x68"
	"\x0b\x68\x01\x32\xc3\xf3\x40\x43\x18\xbf\x01\x22\x9a\x42\x04\xbf"
	"\x4f\xf4\x00\x33\x0b\x60\x03\x4b\x03\x4a\x1a\x60\xfe\xe7\x00\xbf"
	"\x10\x20\x02\x40\x0c\xed\x00\xe0\x04\x00\xfa\x05"
	;

const int sram_code_size = 60;

/* end of STM32L4 generated code */


static uint8_t nanohub_bl_toggle_pempty_bit(struct nanohub_data *data)
{
	uint8_t status;

	pr_info("nanohub: Loading service to manage the FLASH PEMTY bit...\n");

	status = nanohub_bl_write_memory(data, BASE_SRAM_CODE, sram_code_size,
									 &sram_code[0]);

	pr_info("nanohub: Writing code (@0x%08x,%d) status=%02x\n",
			BASE_SRAM_CODE, sram_code_size, status);

	if (status == CMD_ACK) {
			status = nanohub_bl_go(data, BASE_SRAM_CODE);

			pr_info("nanohub: GO command (@0x%08x) status=%02x\n",
					BASE_SRAM_CODE, status);
	}

	return status;
}

static uint8_t nanohub_bl_cmp_page(struct nanohub_data *data, uint32_t addr,
								   const uint8_t *image, uint32_t len,
								   uint32_t *diff)
{
	uint8_t status;
	uint32_t offset;
	uint8_t tmpbuff[256];
	uint32_t lr;
	uint32_t nbdiff;

	offset = 0;

	status = CMD_ACK;
	nbdiff = 0;

	if (len > 256)
		lr = 256;
	else
		lr = len;

	while ((status == CMD_ACK) && (offset < len) && (nbdiff == 0)) {
			status = nanohub_bl_read_memory(data, addr, lr, tmpbuff);

			if (status == CMD_ACK) {
					nbdiff = nanohub_bl_cmp(tmpbuff, &image[offset], lr);
					if (nbdiff)
						pr_debug("nanohub: diff256 (@0x%08x,%d) %d\n", addr,
								 (int)lr, nbdiff);
			}

			/* next */
			addr   += 256;
			offset += 256;

			/* length of remaining data */
			if (((offset < len)) && (offset + lr > len)) {
					lr = len - offset;
					pr_debug("nanohub: diff256 Last packet (@0x%08x,%d)\n",
							 addr, (int)lr);
			}
	}

	if ((status == CMD_ACK) && (diff))
		*diff = nbdiff;

	return status;
}


static uint8_t nanohub_bl_download_by_page(struct nanohub_data *data,
										uint32_t addr, const uint8_t *image,
										uint8_t* ptr, size_t length)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status;
	uint32_t offset;
	uint32_t psize;
	uint32_t lw;
	uint32_t nbdiff;

	psize = pdata->flash_page_size;
	offset = 0;

	if (length > psize)
		lw = psize;
	else
		lw = length;

	if ((!psize) || (!pdata->num_flash_banks) ||
			(((addr - pdata->flash_banks[0].address) % psize) != 0)) {
			pr_err("%s: ERR addr is not page-aligned (@0x%08x,%d) Bk0(@0x%08x)\n",
				   __func__, addr, (int)psize,
				   pdata->num_flash_banks?pdata->flash_banks[0].address:0);
			return CMD_NACK;
	}

	pr_info("nanohub: MCU flash memory base address (@0x%08x)\n",
			pdata->flash_banks[0].address);

	status = CMD_ACK;
	nbdiff = 0;

	while ((status == CMD_ACK) && (offset < length)) {
			int secn = nanohub_bl_page_in_sectors(data, addr, lw);
			if (secn >= 0) {
					uint32_t diff;
					status = nanohub_bl_cmp_page(data, addr,
												&image[offset], lw, &diff);
					if ((status == CMD_ACK) && (diff)) {
							uint32_t page = 0;

							nanohub_bl_get_page_idx(data, addr, psize, &page);

							pr_info("nanohub: ERASE/WRITE page %d (@0x%08X/%d, sector:%d diff:%d)\n",
									page, addr, psize, secn, diff);
							nbdiff += diff;
							status = nanohub_bl_erase_page(data, addr, psize);
							if (status == CMD_ACK) {
									status = nanohub_bl_write_page(data, addr,
																&image[offset],
																lw, psize);
							}
					} else
						pr_debug("nanohub: NO DIFF (@0x%08x,%d)\n", addr, lw);
			}

			/* next */
			addr += psize;
			offset += psize;

			if (((offset < length)) && (offset + lw > length)) {
					lw = length - offset;
					pr_debug("nanohub: Last packet (@0x%08x,%d)\n", addr,
							 (int)lw);
			}
	}

	if (!nbdiff)
		pr_info("nanohub: MCU flash is not updated (no diff found)...\n");
	else
		nanohub_bl_toggle_pempty_bit(data);

	return CMD_ACK;
}

static uint8_t nanohub_bl_download_by_sector(struct nanohub_data *data,
									uint32_t addr, const uint8_t *image,
									uint8_t* ptr, size_t length)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	int i, j;
	uint8_t status = CMD_ACK;
	uint32_t offset;
	uint8_t erase_mask[MAX_FLASH_BANKS] = { 0 };
	uint8_t erase_write_mask[MAX_FLASH_BANKS] = { 0 };
	uint8_t write_mask[MAX_FLASH_BANKS] = { 0 };

	for (i = 0; i < pdata->num_flash_banks; i++) {
			if (addr >= pdata->flash_banks[i].address &&
					addr <
					pdata->flash_banks[i].address +
					pdata->flash_banks[i].length) {
					break;
			}
	}

	offset = (uint32_t) (addr - pdata->flash_banks[i].address);
	j = 0;
	while (j < length && i < pdata->num_flash_banks) {
		if (image[j] != 0xFF)
			erase_write_mask[i] = true;

		if ((ptr[j] & image[j]) != image[j]) {
			erase_mask[i] = true;
			if (erase_write_mask[i]) {
				j += pdata->flash_banks[i].length - offset;
				offset = pdata->flash_banks[i].length;
			} else {
				j++;
				offset++;
			}
		} else {
			if (ptr[j] != image[j])
				write_mask[i] = true;
			j++;
			offset++;
		}

		if (offset == pdata->flash_banks[i].length) {
			i++;
			offset = 0;
			if (i < pdata->num_flash_banks)
				j += (pdata->flash_banks[i].address -
				      pdata->flash_banks[i - 1].address -
				      pdata->flash_banks[i - 1].length);
			else
				j = length;
		}
	}

	for (i = 0; status == CMD_ACK && i < pdata->num_flash_banks; i++) {
		pr_info("nanohub: i=%d, erase=%d, erase_write=%d, write=%d\n",
			i, erase_mask[i], erase_write_mask[i], write_mask[i]);
		if (erase_mask[i]) {
			status =
			    nanohub_bl_erase_sector(data,
						    pdata->flash_banks[i].bank);
			if (status == CMD_ACK && erase_write_mask[i])
				status =
				    write_bank(data, i, addr, image, length);
		} else if (write_mask[i]) {
			status = write_bank(data, i, addr, image, length);
		}
	}

	return status;
}

static uint8_t nanohub_bl_sync_and_check(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status;
	uint16_t bl_pid;
	uint8_t bl_version;

	status = nanohub_bl_sync(data);

	if (status != CMD_ACK) {
			pr_err("%s: sync=%02x\n", __func__, status);
			goto out;
	}

	/* read device infos */
	pr_info("nanohub: Retrieve MCU device information...\n");

	status = nanohub_bl_get_id_cmd(data, &bl_pid);
	if (status != CMD_ACK) {
			pr_err("%s: nanohub_bl_get_id_cmd() status=0x%02x\n", __func__,
				   status);
			goto out;
	}

	status = nanohub_bl_get_version_cmd(data, &bl_version);
	if (status != CMD_ACK) {
			pr_err("%s: nanohub_bl_get_version_cmd() status=0x%02x\n", __func__,
				   status);
			goto out;
	}

	pr_info("nanohub: MCU Device Product ID:0x%04x (BL protocol version:0x%02x)\n",
			bl_pid, bl_version);

	/* check PID and flash organization */
	if ((bl_pid != 0x462) && (pdata->flash_page_size)) {
			pr_err("%s: ERROR - Flash PAGE organization not supported (STM32L45xxx family only)", __func__);
			status = CMD_NACK;
	}

out:
	return status;
}


uint8_t nanohub_bl_download(struct nanohub_data *data, uint32_t addr,
			    const uint8_t *image, size_t length)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status;
	uint8_t *ptr;

	struct timeval ts,te;

	if (pdata->num_flash_banks > MAX_FLASH_BANKS) {
		status = CMD_NACK;
		goto out;
	}

	pr_info("nanohub: Upgrading full nanohub image (@0x%08x,%zd)...\n",
			addr, length);

	status = nanohub_bl_sync_and_check(data);
	if (status != CMD_ACK)
		goto out;

	if (pdata->flash_page_size) {
			pr_info("nanohub: MCU flash memory with PAGE organization (page_size:%d)\n",
					pdata->flash_page_size);
	} else {
			pr_info("nanohub: MCU flash memory with SECTOR organization\n");
	}

	ptr = vmalloc(length);
	if (!ptr) {
		status = CMD_NACK;
		goto out;
	}

	LOG_STAT_TIME_RESET();
	do_gettimeofday(&ts);

	if (pdata->flash_page_size)
		status = CMD_ACK;
	else
		status = nanohub_bl_read_memory(data, addr, length, ptr);

	if (status != CMD_ACK)
		pr_err("%s: nanohub_bl_read_memory() status=0x%02x\n", __func__,
			   status);
	else {
			if (pdata->flash_page_size)
				status = nanohub_bl_download_by_page(data, addr, image, ptr,
													 length);
			else
				status = nanohub_bl_download_by_sector(data, addr, image, ptr,
													   length);
	}
	do_gettimeofday(&te);

	LOG_STAT_TIME_LOG();

	pr_info("nanohub: MCU flash global execution time: %ldms\n",
			1000 * (te.tv_sec - ts.tv_sec) + (te.tv_usec - ts.tv_usec) / 1000);

	vfree(ptr);
out:
	return status;
}

uint8_t nanohub_bl_erase_shared_by_page(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	int i;
	uint8_t status;
	uint32_t addr;
	uint32_t psize;

	psize = pdata->flash_page_size;

	status = CMD_ACK;
	for (i = 0;
			(status == CMD_ACK) && (i < pdata->num_shared_flash_banks);
			i++) {

			pr_info("nanohub: ERASE sector %d (@0x%08X/%zd) %zd pages\n",
					pdata->shared_flash_banks[i].bank,
					pdata->shared_flash_banks[i].address,
					pdata->shared_flash_banks[i].length,
					pdata->shared_flash_banks[i].length / psize);

			addr = pdata->shared_flash_banks[i].address;

			while ((status == CMD_ACK) &&
					(addr < pdata->shared_flash_banks[i].address +
					pdata->shared_flash_banks[i].length)) {

					status = nanohub_bl_erase_page(data, addr, psize);

					addr += psize;
			}
	}

	return status;
}

uint8_t nanohub_bl_erase_shared(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	uint8_t status;


	if (pdata->num_shared_flash_banks > MAX_FLASH_BANKS) {
		status = CMD_NACK;
		goto out;
	}

	pr_info("nanohub: Erasing all MCU shared flash sectors...\n");

	status = nanohub_bl_sync_and_check(data);
	if (status != CMD_ACK)
		goto out;

	if (pdata->flash_page_size) {
			pr_info("nanohub: MCU flash memory with PAGE organization (page_size:%d)\n",
					pdata->flash_page_size);
			status = nanohub_bl_erase_shared_by_page(data);
	} else {
			int i;
			pr_info("nanohub: MCU flash memory with SECTOR organization\n");

			for (i = 0;
					status == CMD_ACK && i < pdata->num_shared_flash_banks;
					i++) {
					status = nanohub_bl_erase_sector(data,
								pdata->shared_flash_banks[i].bank);
			}
	}

out:
	return status;
}

uint8_t nanohub_bl_erase_shared_bl(struct nanohub_data *data)
{
	uint8_t status;

	status = nanohub_bl_sync(data);

	if (status != CMD_ACK) {
		pr_err("nanohub_bl_erase_shared_bl: sync=%02x\n", status);
		goto out;
	}

	status = nanohub_bl_erase_special(data, 0xFFF0);
out:
	return status;
}

/* erase a single sector */
uint8_t nanohub_bl_erase_sector(struct nanohub_data *data, uint16_t sector)
{
	uint8_t ret;
	LOG_STAT_TIME_DEC;

	LOG_STAT_TIME_ENTRY();

	data->bl.write_cmd(data, data->bl.cmd_erase);
	ret = data->bl.read_ack(data);
	if (ret == CMD_ACK)
		ret = write_cnt(data, 0x0000);
	if (ret != CMD_NACK)
		ret = read_ack_loop(data);
	if (ret == CMD_ACK)
		ret = write_cnt(data, sector);
	if (ret != CMD_NACK)
		ret = read_ack_loop(data);

	LOG_STAT_TIME_EXIT();
	LOG_STAT_TIME_INC_ERASE(1);

	return ret;
}

/* erase special */
uint8_t nanohub_bl_erase_special(struct nanohub_data *data, uint16_t special)
{
	uint8_t ret;

	data->bl.write_cmd(data, data->bl.cmd_erase);
	ret = data->bl.read_ack(data);
	if (ret == CMD_ACK)
		ret = write_cnt(data, special);
	if (ret != CMD_NACK)
		ret = read_ack_loop(data);

	return ret;
}

/* read memory - this will chop the request into 256 byte reads */
uint8_t nanohub_bl_read_memory(struct nanohub_data *data, uint32_t addr,
			       uint32_t length, uint8_t *buffer)
{
	uint8_t ret = CMD_ACK;
	uint32_t offset = 0;
	LOG_STAT_TIME_DEC;

	LOG_STAT_TIME_ENTRY();
	while (ret == CMD_ACK && length > offset) {
		data->bl.write_cmd(data, data->bl.cmd_read_memory);
		ret = data->bl.read_ack(data);
		if (ret == CMD_ACK) {
			write_addr(data, addr + offset);
			ret = read_ack_loop(data);
			if (ret == CMD_ACK) {
				if (length - offset >= 256) {
					write_len(data, 256);
					ret = read_ack_loop(data);
					if (ret == CMD_ACK) {
						data->bl.read_data(data,
								   &buffer
								   [offset],
								   256);
						offset += 256;
					}
				} else {
					write_len(data, length - offset);
					ret = read_ack_loop(data);
					if (ret == CMD_ACK) {
						data->bl.read_data(data,
								   &buffer
								   [offset],
								   length -
								   offset);
						offset = length;
					}
				}
			}
		}
	}
	LOG_STAT_TIME_EXIT();
	LOG_STAT_TIME_INC_READ(length);

	return ret;
}

/* write memory - this will chop the request into 256 byte writes */
uint8_t nanohub_bl_write_memory(struct nanohub_data *data, uint32_t addr,
				uint32_t length, const uint8_t *buffer)
{
	uint8_t ret = CMD_ACK;
	uint32_t offset = 0;
	LOG_STAT_TIME_DEC;

	LOG_STAT_TIME_ENTRY();
	while (ret == CMD_ACK && length > offset) {
		data->bl.write_cmd(data, data->bl.cmd_write_memory);
		ret = data->bl.read_ack(data);
		if (ret == CMD_ACK) {
			write_addr(data, addr + offset);
			ret = read_ack_loop(data);
			if (ret == CMD_ACK) {
				if (length - offset >= 256) {
					write_len_data(data, 256,
						       &buffer[offset]);
					offset += 256;
				} else {
					write_len_data(data, length - offset,
						       &buffer[offset]);
					offset = length;
				}
				/* STM WA (#438795) */
				usleep_range(3000, 4000);
				/* end of STM WA */
				ret = read_ack_loop(data);
			}
		}
	}
	LOG_STAT_TIME_EXIT();
	LOG_STAT_TIME_INC_WRITE(length);

	return ret;
}

/* */
uint8_t nanohub_bl_go(struct nanohub_data *data, uint32_t addr)
{
	uint8_t ret;

	data->bl.write_cmd(data, CMD_GO);
	ret = data->bl.read_ack(data);
	if (ret == CMD_ACK) {
			write_addr(data, addr);
			ret = read_ack_loop(data);
	}

	return ret;
}

uint8_t nanohub_bl_get_version(struct nanohub_data *data, uint8_t *version)
{
	uint8_t status;

	status = nanohub_bl_sync(data);
	if (status != CMD_ACK) {
		pr_err("nanohub_bl_get_version: sync=%02x\n", status);
		goto out;
	}

	data->bl.write_cmd(data, data->bl.cmd_get_version);
	status = data->bl.read_ack(data);
	if (status == CMD_ACK)
		data->bl.read_data(data, version, 1);
	status = data->bl.read_ack(data);
out:
	return status;
}

uint8_t nanohub_bl_get_id(struct nanohub_data *data, uint16_t *id)
{
	uint8_t status;
	uint8_t len;
	uint8_t buffer[256];

	status = nanohub_bl_sync(data);
	if (status != CMD_ACK) {
		pr_err("nanohub_bl_get_id: sync=%02x\n", status);
		goto out;
	}

	data->bl.write_cmd(data, data->bl.cmd_get_id);
	status = data->bl.read_ack(data);
	if (status == CMD_ACK) {
		data->bl.read_data(data, &len, 1);
		data->bl.read_data(data, buffer, len+1);
		*id = (buffer[0] << 8) | buffer[1];
	}
	status = data->bl.read_ack(data);
out:
	return status;
}

uint8_t nanohub_bl_lock(struct nanohub_data *data)
{
	uint8_t status;

	status = nanohub_bl_sync(data);

	if (status != CMD_ACK) {
		pr_err("nanohub_bl_lock: sync=%02x\n", status);
		goto out;
	}

	data->bl.write_cmd(data, data->bl.cmd_readout_protect);
	status = data->bl.read_ack(data);
	if (status == CMD_ACK)
		status = read_ack_loop(data);
out:
	return status;
}

uint8_t nanohub_bl_unlock(struct nanohub_data *data)
{
	uint8_t status;

	status = nanohub_bl_sync(data);

	if (status != CMD_ACK) {
		pr_err("nanohub_bl_lock: sync=%02x\n", status);
		goto out;
	}

	data->bl.write_cmd(data, data->bl.cmd_readout_unprotect);
	status = data->bl.read_ack(data);
	if (status == CMD_ACK)
		status = read_ack_loop(data);
out:
	return status;
}

uint8_t nanohub_bl_update_finished(struct nanohub_data *data)
{
	uint8_t ret;

	data->bl.write_cmd(data, data->bl.cmd_update_finished);
	ret = read_ack_loop(data);

	return ret;
}
