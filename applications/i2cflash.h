/*
 * Copyright (c) 2023, Yunjae Lim <launius@gmail.com>
 *
 * Firmware update for internal MCU using external I2C
 */

#ifndef I2CFLASH_H
#define I2CFLASH_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define BYTE	unsigned char
#define UINT	unsigned int
#define ULONG	unsigned long

#define I2CBUS_CHIP_1		5
#define I2CBUS_CHIP_2		3
#define I2CBUS_CHIP_3		7
#define I2CBUS_CHIP_4		2
#define I2CBUS_CHIP_5		4

#define I2CBUS_LTM		4

#define PRINT_ERROR		1
#define PRINT_DEBUG		1

#define print_error(fmt, ...) \
		do { if (PRINT_ERROR) fprintf(stderr, "%s: " fmt, \
									__func__, ##__VA_ARGS__); } while (0)
#define print_debug(fmt, ...) \
		do { if (PRINT_DEBUG) fprintf(stdout, "%s: " fmt, \
									__func__, ##__VA_ARGS__); } while (0)

void CHIP2_Firmware_Upgrade(int fd, BYTE* byProgData, BYTE* byReadData, ULONG ulDataLen);
void CHIP2_Firmware_Extract(int fd, BYTE* byReadData, ULONG ulDataLen);

int i2c_read(int fd, uint8_t *buf, size_t size);
int i2c_write(int fd, uint8_t *buf, size_t size);

int save_firmware(unsigned char *data, int size, char *filename);

#endif /* I2CFLASH_H */
