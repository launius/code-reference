// Copyright (c) Yunjae Lim <launius@gmail.com>

/*
 * This application set LED colors via sysfs using regmap
 * between Linux driver and microcontroller firmware
 *
./meta-layer/recipes-apps/ledcolor/
├── files
│   └── src
│       ├── ledcolor.c
└── ledcolor_0.1.bb
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#define FILEPATH	"/sys/devices/platform/feab0000.i2c/i2c-3/3-0012/device-gpio/gpio_keypad"
#define FILE_LED	"led_color"

/* LED color number
enum led_color
{
    LED_WHITE = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_TEAL,
    LED_PLUM,
    LED_BLACK,
    LED_COLOR7,
    LED_COLOR8,
    LED_COLOR9,
    LED_COLOR10 = 10,

    LED_WHITE_DIMMED = 11,
    LED_RED_DIMMED,
    LED_GREEN_DIMMED,
    LED_BLUE_DIMMED,
    LED_COLOR_DIMMED,

    LED_COLOR_RESERVED = 255
};
*/
#define LED_COLOR_MAX	256

__uint16_t get_color_index(int key, int color)
{
	return LED_COLOR_MAX*key + color;
}

int main()
{
	int key, clr;
	int fd;
	int bytes;
	__uint16_t index;

	char led_file[128];
	char buf[6] = {0,};

	printf("start LED color app...\n");

	sprintf(led_file, "%s/%s", FILEPATH, FILE_LED);

	while(1) {
		printf("input #LED(0-62) #colour(0-255) (exit: #99 #0): ");

		assert(scanf("%d %d", &key, &clr) > 0);

		if (key == 99)
			break;

		if ((fd = open(led_file, O_WRONLY)) == -1) {
			perror("file open error : ");
			exit(0);
		}

		index = get_color_index(key, clr);
		bytes = sprintf(buf, "%d", index);
		write(fd, buf, bytes);

		close(fd);
	}

    return 0;
}
