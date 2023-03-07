/*
 * Copyright (c) 2023, Yunjae Lim <launius@gmail.com>
 *
 * Firmware update for internal MCU using external I2C
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define DEBUG

unsigned int bySpiLen = 31;

static inline int i2c_write(int file, unsigned char *buf, int size)
{
	if (write(file, buf, size) != size) {
		printf("%s: failed %d\n", __func__, size);
		return -1;
	}

	return 0;
}

int check_chip_id(int file)
{
	__u8 reg = 0x00; /* Device register to access */
	unsigned char buf[2];
	unsigned short chip_id;

	buf[0] = 0xff;	buf[1] = 0xe0;
	i2c_write(file, buf, 2);

	buf[0] = 0xee;	buf[1] = 0x01;
	i2c_write(file, buf, 2);

	buf[0] = 0xff;	buf[1] = 0xe1;
	i2c_write(file, buf, 2);

	reg = 0x00;
	buf[0] = reg;
	i2c_write(file, buf, 1);

	if (read(file, buf, 2) != 2) {
		printf("read error\n");
		return -1;
	}
	
	chip_id = (buf[0] << 8) | buf[1];
	printf("%s: 0x%02x 0x%02x, 0x%x\n", __func__, buf[0], buf[1], chip_id);

	return (chip_id == 0x2003);
}

int config_settings(int file)
{
	unsigned char buf[16] = {
		0xff, 0xe0,
		0xee, 0x01,
		0x5e, 0xc1,
		0x58, 0x00,
		0x59, 0x50,
		0x5a, 0x10,
		0x5a, 0x00,
		0x58, 0x21 };
	int i;

	for (i = 0 ; i < 16 ; i += 2) {
		i2c_write(file, buf + i, 2);

#ifdef DEBUG
		printf("%s: 0x%02x %02x\n", __func__, buf[i], buf[i+1]);
#endif
	}

	return 0;
}

int erase_block(int file)
{
	unsigned int erase_addr = 0;
	unsigned char buf[2];
	unsigned char addr[3] = {0,};
	int i;

	for (i = 0 ; i < 2 ; i++) {
		erase_addr = (i == 1) ? 0x8000 : 0;

		buf[0] = 0x5a;	buf[1] = 0x04;
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x00;
		i2c_write(file, buf, 2);

		addr[0] = (erase_addr & 0xFF0000) >> 16;
		addr[1] = (erase_addr & 0xFF00) >> 8;
		addr[2] = (erase_addr & 0xFF);

		printf("%s: erase_addr 0x%x, addr = 0x%x %x %x\n", __func__, erase_addr, addr[0], addr[1], addr[2]);

		buf[0] = 0x5b;	buf[1] = addr[0];
		i2c_write(file, buf, 2);

		buf[0] = 0x5c;	buf[1] = addr[1];
		i2c_write(file, buf, 2);

		buf[0] = 0x5d;	buf[1] = addr[2];
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x01;
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x00;
		i2c_write(file, buf, 2);

		usleep(500*1000);
	}

	return 0;
}

int program_firmware(int file, char *prog_data, unsigned int data_len)
{
	unsigned int m_block = 256;
	unsigned int write_addr = 0;
	unsigned char addr[3] = {0,};

	int start_block;
	int num_blocks;

	int i;
	unsigned char buf[2];
	unsigned char write_data[32 + 1] = {0,};

	addr[0] = (write_addr & 0xFF0000) >> 16;
	addr[1] = (write_addr & 0xFF00) >> 8;
	addr[2] = (write_addr & 0xFF);

	start_block = write_addr / m_block;
	num_blocks = (data_len % m_block) ? (data_len / m_block + 1) : (data_len / m_block);

	unsigned int start_addr = write_addr;
	unsigned int end_addr = write_addr;
	int spi_data_len = 32;

	printf("%s: prog_data 0x%p, data_len %d, num_blocks %d\n", __func__, prog_data, data_len, num_blocks);

	for (i = 0 ; i < num_blocks ; ++i) {
		end_addr = ((i + start_block + 1) * m_block > (write_addr + data_len)) ?
				 (write_addr + data_len) : ((i + start_block + 1) * m_block);

		int num_pages = ((end_addr - start_addr) % spi_data_len) ?
					 ((end_addr - start_addr) / spi_data_len + 1) : ((end_addr - start_addr) / spi_data_len);

#ifdef DEBUG
//		printf("%s: start_block %d, write_addr %d\n", __func__, start_block, write_addr);
		printf("%s: block %d, start_addr %d, end_addr %d, num_pages %d\r", __func__, i, start_addr, end_addr, num_pages);
#endif

		for (int j = 0 ; j < num_pages ; ++j) {
			// wren
			buf[0] = 0xff;	buf[1] = 0xe1;
			i2c_write(file, buf, 2);

			buf[0] = 0x03;	buf[1] = 0x2e;
			i2c_write(file, buf, 2);

			buf[0] = 0x03;	buf[1] = 0xee;
			i2c_write(file, buf, 2);

			buf[0] = 0xff;	buf[1] = 0xe0;
			i2c_write(file, buf, 2);

			buf[0] = 0x5a;	buf[1] = 0x04;
			i2c_write(file, buf, 2);

			buf[0] = 0x5a;	buf[1] = 0x00;
			i2c_write(file, buf, 2);

			// i2c data to fifo
			buf[0] = 0x5e;	buf[1] = 0xdf;
			i2c_write(file, buf, 2);

			buf[0] = 0x5a;	buf[1] = 0x20;
			i2c_write(file, buf, 2);

			buf[0] = 0x5a;	buf[1] = 0x00;
			i2c_write(file, buf, 2);

			buf[0] = 0x58;	buf[1] = 0x21;
			i2c_write(file, buf, 2);

			int cur_spi_data_len = ((end_addr - start_addr) / spi_data_len == 0) ?
								((end_addr - start_addr) % spi_data_len) : spi_data_len;

			memset(write_data, 0x00, bySpiLen + 1 + 1);
			for (int k = 0 ; k < cur_spi_data_len ; ++k)
				write_data[k + 1] = *(prog_data + start_addr - write_addr + k);

#if 0
			printf("%s: page %d, len %d, start_addr %d, write_addr %d, addr 0x%02x %02x %02x\n",
				 __func__, j, cur_spi_data_len, start_addr, write_addr, addr[0], addr[1], addr[2]);

			for (int t = 0 ; t < 32 + 1 ; t++)
				printf("%02x ", write_data[t]);
			printf("\n");
#endif

			write_data[0] = 0x59;
			i2c_write(file, write_data, cur_spi_data_len + 1);		//TODO: need to verify

			// fifo to flash
			buf[0] = 0x5b;	buf[1] = addr[0];
			i2c_write(file, buf, 2);

			buf[0] = 0x5c;	buf[1] = addr[1];
			i2c_write(file, buf, 2);

			buf[0] = 0x5d;	buf[1] = addr[2];
			i2c_write(file, buf, 2);

			buf[0] = 0x5a;	buf[1] = 0x10;
			i2c_write(file, buf, 2);

			buf[0] = 0x5a;	buf[1] = 0x00;
			i2c_write(file, buf, 2);

			start_addr += (bySpiLen + 1);
			addr[0] = (start_addr & 0xFF0000) >> 16;
			addr[1] = (start_addr & 0xFF00) >> 8;
			addr[2] = (start_addr & 0xFF);
		}

		start_addr = (i + start_block + 1) * m_block;
		addr[0] = (start_addr & 0xFF0000) >> 16;
		addr[1] = (start_addr & 0xFF00) >> 8;
		addr[2] = (start_addr & 0xFF);
	}

#ifdef DEBUG
	printf("\n");
#endif

	buf[0] = 0x5a;	buf[1] = 0x88;		//TODO: make sure the values
//	buf[0] = 0x5a;	buf[1] = 0x08;
	i2c_write(file, buf, 2);

	buf[0] = 0x5a;	buf[1] = 0x80;
//	buf[0] = 0x5a;	buf[1] = 0x00;
	i2c_write(file, buf, 2);

	return 0;
}

int read_firmware(int file, char *read_data, unsigned int read_len)
{
	unsigned int read_addr = 0;
	unsigned char addr[3] = {0,};

	unsigned char buf[2];
	unsigned char cur_read_data[32] = {0,};
	int cur_read_len;

	addr[0] = (read_addr & 0xFF0000) >> 16;
	addr[1] = (read_addr & 0xFF00) >> 8;
	addr[2] = (read_addr & 0xFF);

	int n_page = read_len / (32);

	if(read_len % 32 != 0)
		++n_page;
	
	for (int i = 0 ; i < n_page ; ++i) {
		buf[0] = 0xff;	buf[1] = 0xe0;
		i2c_write(file, buf, 2);

		buf[0] = 0x5e;	buf[1] = 0x5f;
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x20;
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x00;
		i2c_write(file, buf, 2);

		buf[0] = 0x5b;	buf[1] = addr[0];
		i2c_write(file, buf, 2);

		buf[0] = 0x5c;	buf[1] = addr[1];
		i2c_write(file, buf, 2);

		buf[0] = 0x5d;	buf[1] = addr[2];
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x10;
		i2c_write(file, buf, 2);

		buf[0] = 0x5a;	buf[1] = 0x00;
		i2c_write(file, buf, 2);

		buf[0] = 0x58;	buf[1] = 0x21;
		i2c_write(file, buf, 2);

		cur_read_len = (read_len - i*(32) < 32) ? (read_len - i*(32)) : (32);

		buf[0] = 0x5f;
		i2c_write(file, buf, 1);

		if (read(file, cur_read_data, cur_read_len) != cur_read_len) {
			printf("read error\n");
			return -1;
		}

#ifdef DEBUG
		printf("%s: page %d, read_addr %d, addr 0x%02x %02x %02x\r", __func__, i, read_addr, addr[0], addr[1], addr[2]);

		// for (int t = 0 ; t < 32 ; t++)
		// 	printf("%02x ", cur_read_data[t]);
		// printf("\n");
#endif

		for (int j = 0 ; j < cur_read_len ; ++j)
			read_data[i*(32) + j] = cur_read_data[j];

		read_addr += cur_read_len;
		addr[0] = (read_addr & 0xFF0000) >> 16;
		addr[1] = (read_addr & 0xFF00) >> 8;
		addr[2] = (read_addr & 0xFF);
	}

#ifdef DEBUG
	printf("\n");
#endif

	buf[0] = 0x5a;	buf[1] = 0x08;
	i2c_write(file, buf, 2);

	buf[0] = 0x5a;	buf[1] = 0x00;
	i2c_write(file, buf, 2);

	return 0;
}

int load_firmware(char *filename, char **data, char **result, int *size)
{
	FILE *fp;

	fp = fopen(filename, "r");
	if (fp == NULL) {
		printf("%s open fail!\n", filename);
		return -1;
	}

	fseek(fp, 0L, SEEK_END);
	*size = ftell(fp);
	fseek(fp, 0L, SEEK_SET);

	*data = (char *)calloc(*size, sizeof(char));
	*result = (char *)calloc(*size, sizeof(char));

	fread(*data, sizeof(char), *size, fp);
	fclose(fp);

	return 0;
}

int save_firmware(char *data, int size)
{
	FILE *fp;

	fp = fopen("/usr/bin/LT86204UX_512K.bin", "w");
	if (fp == NULL) {
		printf("LT86204UX_512K.bin open fail\n");
		return -1;
	}

	fwrite(data, sizeof(char), size, fp);
	fclose(fp);

	return 0;
}

int main(int argc, char *argv[])
{
	int file;
	int adapter_nr = 4; /* probably dynamically determined */
	char filename[20];
	int addr = 0x38; /* The I2C address */

	int size;
	char *w_data, *r_data;
	int i;

	if (argc < 2) {
		printf("Firmware filename error!\n");
		exit(1);
	}

	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	file = open(filename, O_RDWR);
	if (file < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}

	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}

	if (check_chip_id(file) != 1) {
		printf("check_chip_id fail\n");
		close(file);
		exit(1);
	}

	config_settings(file);

	erase_block(file);

	if (load_firmware(argv[1], &w_data, &r_data, &size))
		exit(1);

	program_firmware(file, w_data, size);

	read_firmware(file, r_data, size);

	if (memcmp(w_data, r_data, size))
		printf("verification fail\n");
	else
		printf("verification successful\n");

#ifdef DEBUG
	printf("written:\n");
	for (i = 0 ; i < 500 ; i++)
		printf("%02x ", w_data[i]);
	printf("\n");

	printf("written:\n");
	for (i = size - 500 ; i < size ; i++)
		printf("%02x ", w_data[i]);
	printf("\n");

	printf("read:\n");
	for (i = 0 ; i < 500 ; i++)
		printf("%02x ", r_data[i]);
	printf("\n");

	printf("read:\n");
	for (i = size - 500 ; i < size ; i++)
		printf("%02x ", r_data[i]);
	printf("\n");
#endif

	close(file);

	free(w_data);
	free(r_data);
}
