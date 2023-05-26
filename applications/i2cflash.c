/*
 * Copyright (c) 2023, Yunjae Lim <launius@gmail.com>
 *
 * Firmware update for internal MCU using external I2C
 */

#include "i2cflash.h"

#define DEBUG

static const unsigned int bySpiLen = 31;

int i2c_read(int fd, uint8_t *buf, size_t size)
{
	return read(fd, buf, size);
}

int i2c_write(int fd, uint8_t *buf, size_t size)
{
	if (write(fd, buf, size) != size) {
		print_error("write failed %lu\n", size);
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
	print_debug("0x%02x 0x%02x, 0x%x\n", buf[0], buf[1], chip_id);

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

int erase_block(int file, bool backup)
{
	unsigned int erase_addr = 0;
	unsigned char buf[2];
	unsigned char addr[3] = {0,};
	int i;

	for (i = 0 ; i < 2 ; i++) {
		if (!backup)
			erase_addr = (i == 1) ? 0x008000 : 0x000000;
		else
			erase_addr = (i == 1) ? 0x048000 : 0x040000;	// backup addr

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

int program_firmware(int file, unsigned char *prog_data, unsigned int data_len, bool backup)
{
	unsigned int m_block = 256;
	unsigned int write_addr = 0;
	unsigned char addr[3] = {0, 0, 0};

	int start_block;
	int num_blocks;

	int i;
	unsigned char buf[2];
	unsigned char write_data[32 + 1] = {0,};

	if (backup)
		write_addr = 0x040000;		// backup addr

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

int read_firmware(int file, unsigned char *read_data, unsigned int read_len, bool backup)
{
	unsigned int read_addr = 0;
	unsigned char addr[3] = {0, 0, 0};

	unsigned char buf[2];
	unsigned char cur_read_data[32] = {0,};
	int cur_read_len;

	if(backup)
		read_addr = 0x040000;		// backup addr

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

int load_firmware(char *filename, unsigned char **data, unsigned char **result, int *size)
{
	FILE *fp;

	fp = fopen(filename, "r");
	if (fp == NULL) {
		print_error("%s open fail!\n", filename);
		return -1;
	}

	fseek(fp, 0L, SEEK_END);
	*size = ftell(fp);
	fseek(fp, 0L, SEEK_SET);

	*data = (unsigned char *)calloc(*size, sizeof(char));
	*result = (unsigned char *)calloc(*size, sizeof(char));	//TODO: move to reading position

	fread(*data, sizeof(char), *size, fp);
	fclose(fp);

	return 0;
}

int save_firmware(unsigned char *data, int size, char *filename)
{
	FILE *fp;

	if ((fp = fopen(filename, "w")) == NULL) {
		print_error("save file open error!");
		return 1;
	}

	fwrite(data, sizeof(char), size, fp);
	fclose(fp);

	return 0;
}

void dump_memory(unsigned char *data1, unsigned char *data2, int size)
{
	int i;

	if (data1) {
		printf("first n-bytes written of %dbytes:\n", size);
		for (i = 0 ; i < 500 ; i++)
			printf("%02x ", data1[i]);
		printf("\n");

		printf("last n-bytes written:\n");
		for (i = size - 500 ; i < size ; i++)
			printf("%02x ", data1[i]);
		printf("\n");
	}

	if (data2) {
		printf("first n-bytes read:\n");
		for (i = 0 ; i < 500 ; i++)
			printf("%02x ", data2[i]);
		printf("\n");

		printf("last n-bytes read:\n");
		for (i = size - 500 ; i < size ; i++)
			printf("%02x ", data2[i]);
		printf("\n");
	}

	if (data1 && data2) {
		if (memcmp(data1, data2, size))
			print_debug("verification fail\n");
		else
			print_debug("verification success\n");
	}
}

static int A_Firmware_Extract(int fd, unsigned char *data, int size)
{
//	char *data = (char *)calloc(size, sizeof(char));

	if (check_chip_id(fd) != 1) {
		print_error("check chip ID fail!\n");
		return 1;
	}

	read_firmware(fd, data, size, false);
	save_firmware(data, size, "/usr/bin/firmware.saved");

	return 0;
}

static int A_Firmware_Upgrade(int fd, unsigned char *w_data, unsigned char *r_data, int size)
{
	if (check_chip_id(fd) != 1) {
		print_error("check chip ID fail!\n");
		return 1;
	}

	config_settings(fd);

	erase_block(fd, false);

	program_firmware(fd, w_data, size, false);

	read_firmware(fd, r_data, size, false);

#ifdef UPDATE_BACKUP_FW
	config_settings(fd);

	erase_block(fd, true);

	program_firmware(fd, w_data, size, true);

	read_firmware(fd, r_data, size, true);

	if (memcmp(w_data, r_data, size))
		print_debug("backup verification fail\n");
	else
		print_debug("backup verification success\n");
#endif

	return 0;
}

static void help(void) __attribute__ ((noreturn));

static void help(void)
{
	fprintf(stderr, "Usage: i2cflash MODE FIRMWARE-FILE\n"
				"  MODE is one of:\n"
				"    1 (HDMI to MIPI1)\n"
				"    2 (HDMI to MIPI2)\n"
				"    3 (HDMI to MIPI3)\n"
				"    4 (HDMI to MIPI4)\n"
				"    5 (HDMI to MIPI5)\n"
				"    8 (A, Extract Firmware)\n"
				"    9 (HDMI Matrix)\n"
				"    0 (B, Extract Firmware)\n"
				"  FIRMWARE-FILE is a firmware file name including the path\n"
				"e.g. # i2cflash 1 /usr/bin/firmware.bin\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int flags = 0;
	char *mode, *fw_file;

	char dev_file[16];
	int addr;
	int fd;

	int size;
	unsigned char *w_data, *r_data;

	if (argc < flags + 3)
		help();

	mode = argv[flags+1];
	fw_file = argv[flags+2];
	print_debug("mode %s, %s flashing...\n", mode, fw_file);

	if (*mode == '1') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_1);
		addr = 0x2b;
	}
	else if (*mode == '2') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_2);
		addr = 0x2b;
	}
	else if (*mode == '3') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_3);
		addr = 0x2b;
	}
	else if (*mode == '4') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_4);
		addr = 0x2b;
	}
	else if (*mode == '5') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_5);
		addr = 0x2b;
	}
	else if (*mode == '7') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_1);
		addr = 0x2b;
	}
	else if (*mode == '8') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_CHIP_1);
		addr = 0x2b;
	}
	else if (*mode == '9') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_LTM);
		addr = 0x38;
	}
	else if (*mode == '0') {
		snprintf(dev_file, 15, "/dev/i2c-%d", I2CBUS_LTM);
		addr = 0x38;
	}
	else
		help();

	fd = open(dev_file, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "i2c device file open error!\n");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		fprintf(stderr, "I2C slave address set error!\n");
		close(fd);
		exit(1);
	}

	if (load_firmware(fw_file, &w_data, &r_data, &size)) {
		close(fd);
		exit(1);
	}

	if (*mode == '9')
		A_Firmware_Upgrade(fd, w_data, r_data, size);
	else if (*mode == '0')
		A_Firmware_Extract(fd, r_data, size);
	else if (*mode == '7')
		B_Firmware_Extract(fd, r_data, size);
	else if (*mode == '8')
		B_Firmware_Extract(fd, NULL, 22336);
	else
		B_Firmware_Upgrade(fd, w_data, r_data, size);

#ifdef DEBUG
	dump_memory(w_data, r_data, size);
#endif

	free(w_data);
	free(r_data);
	close(fd);

	exit(0);
}
