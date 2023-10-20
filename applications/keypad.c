// Copyright (c) Yunjae Lim <launius@gmail.com>

/*
 * This application polls interrupts from GPIO driver and scans the key/rotary input using sysfs.
 * The microcontroller is connected by I2C and registers have been mapped in the GPIO driver
 * between Linux driver and MCU firmware.
 *
./meta-layer/recipes-apps/keypad/
├── files
│   └── src
│       ├── keypad.c
└── keypad_0.1.bb
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#define FILEPATH	"/sys/devices/platform/feab0000.i2c/i2c-3/3-0012/device-gpio/gpio_keypad"
#define FILE_IRQ	"key_irq"
#define FILE_REG	"key_reg"

int main()
{
	int gpio_fd;
	int key_fd[5];
	int key_val[5];
	int state;
	int n;
	int i;

	char irq_file[128], reg_file[128];
	char buf[5] = {0,};

	fd_set exceptfds;

	sprintf(irq_file, "%s/%s", FILEPATH, FILE_IRQ);

	for(;;) {
		if ((gpio_fd = open(irq_file, O_RDONLY)) == -1) {
			perror("file open error : ");
			exit(0);
		}

		for (i = 0 ; i < 5 ; i++) {
			sprintf(reg_file, "%s/%s%d", FILEPATH, FILE_REG, i);
			if ((key_fd[i] = open(reg_file, O_RDONLY)) == -1) {
				perror("file open error : ");
				exit(0);
			}
		}

		FD_ZERO(&exceptfds);
		FD_SET(gpio_fd, &exceptfds);
		memset (buf, 0x00, sizeof(buf));

		state = select(gpio_fd+1, NULL, NULL, &exceptfds, NULL);
		switch(state) {
			case -1:
				perror("select error : ");
				exit(0);
				break;
			default:
				if (FD_ISSET(gpio_fd, &exceptfds))
					while ((n = read(gpio_fd, buf, sizeof(buf))) > 0)
						printf("read gpio irq %s\n", buf);
				break;
		}
		usleep(1000);

		FD_ZERO(&exceptfds);
		FD_SET(gpio_fd, &exceptfds);

		state = select(gpio_fd+1, NULL, NULL, &exceptfds, NULL);
		switch(state) {
			case -1:
				perror("select error : ");
				exit(0);
				break;
			default:
				if (FD_ISSET(gpio_fd, &exceptfds)) {
					for (i = 0 ; i < 3 ; i++) {
						memset (buf, 0x00, sizeof(buf));
						while ((n = read(key_fd[i], buf, sizeof(buf))) > 0) {
							key_val[i] = strtol(buf, NULL, 16);
							printf("read key_reg%d 0x%x\n", i, key_val[i]);
						}
					}

					for (i = 3 ; i < 5 ; i++) {
						memset (buf, 0x00, sizeof(buf));
						while ((n = read(key_fd[i], buf, sizeof(buf))) > 0) {
							key_val[i] = strtol(buf, NULL, 10);
							printf("read key_reg%d %d\n", i, key_val[i]);
						}
					}
				}
				break;
		}

		printf("\n");
		if (key_val[0] & 1<<0)
			printf("Key1 on\n");
		if (key_val[0] & 1<<1)
			printf("Key2 on\n");
		if (key_val[0] & 1<<2)
			printf("Key3 on\n");
		if (key_val[0] & 1<<3)
			printf("Key4 on\n");
		if (key_val[0] & 1<<4)
			printf("Key5 on\n");
		if (key_val[0] & 1<<5)
			printf("Key6 on\n");
		if (key_val[0] & 1<<6)
			printf("Key7 on\n");

		if (key_val[1] & 1<<0)
			printf("Key8 on\n");
		if (key_val[1] & 1<<4)
			printf("Key9 on\n");

		// ... divided by bit operation

		if (key_val[0] & 1<<7)
			printf("Encoder1 Switch on\n");
		if (key_val[1] & 1<<7)
			printf("Encoder2 Switch on\n");

		printf("Encoder1 steps %d\n", key_val[3]);
		printf("Encoder2 steps %d\n", key_val[4]);

		close(gpio_fd);
		for (i = 0 ; i < 5 ; i++)
			close(key_fd[i]);
    }

    return 0;
}
