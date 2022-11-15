// (c) 2022 Yunjae Lim <launius@gmail.com> 

/*
 * This application controls the LED pad.
 */

/*
 * How to test ledpad
 * $ /usr/bin/ledpad
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <linux/version.h>
#include <linux/input.h>

#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>

#define BITS_PER_LONG			(sizeof(long) * 8)
#define NBITS(x)				((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)					((x)%BITS_PER_LONG)
#define BIT(x)					(1UL<<OFF(x))
#define LONG(x)					((x)/BITS_PER_LONG)
#define test_bit(bit, array)	((array[LONG(bit)] >> OFF(bit)) & 1)

#define DEV_ROTARY_ENC1			"/dev/input/event0"
#define DEV_ROTARY_ENC2			"/dev/input/event1"

#define DEV_MISC_SPI			"/dev/misc_spi_ledpad"
#define SYS_LEDS				"/sys/class/leds/multicolor:status/brightness"

#define LED_ENC1_CW				0x33
#define LED_ENC1_CCW			0x34
#define LED_ENC2_CW				0x35
#define LED_ENC2_CCW			0x36

static const char * const absval[6] = { "Value", "Min  ", "Max  ", "Fuzz ", "Flat ", "Resolution "};

static volatile sig_atomic_t stop = 0;

void control_led(char c) {
	int fd;

	fd = open(SYS_LEDS, O_WRONLY);
	write(fd, &c, 1);

	close(fd);
}

int control_keypad(bool on)
{
	int fd;
	char *cmd;
	int size;
	// TODO: consider mutex lock
	static bool state = false;
	
	if (state == on)
		return EXIT_SUCCESS;
	
	if (state) {
		cmd = "stop";
		size = 5;
	} else {
		cmd = "thread";
		size = 7;
	}

	if ((fd = open(DEV_MISC_SPI, O_WRONLY)) < 0) {
		perror("ledpad misc_spi");
		if (errno == EACCES && getuid() != 0)
			fprintf(stderr, "You do not have access to %s.\n", DEV_MISC_SPI);
		return EXIT_FAILURE;
	}

	write(fd, cmd, size);
	close(fd);

	state ^= true;
	
	return EXIT_SUCCESS;
}

static void interrupt_handler(int sig)
{
	stop = 1;
}

static void print_absdata(int fd, int axis)
{
	int abs[6] = {0};
	int k;

	ioctl(fd, EVIOCGABS(axis), abs);
	for (k = 0; k < 6; k++)
		if ((k < 3) || abs[k])
			printf("      %s %6d\n", absval[k], abs[k]);
}

static int print_rotary_info(int *fds)
{
	unsigned int type, code;
	int i, version;
	unsigned short id[4];
	char name[256] = "Unknown";
	unsigned long bit[EV_MAX][NBITS(KEY_MAX)];

	if (ioctl(fds[0], EVIOCGVERSION, &version)) {
		perror("evtest: can't get version");
		return 1;
	}

	printf("Input driver version is %d.%d.%d\n",
		version >> 16, (version >> 8) & 0xff, version & 0xff);

	ioctl(fds[0], EVIOCGID, id);
	printf("Input device ID: bus 0x%x vendor 0x%x product 0x%x version 0x%x\n",
		id[ID_BUS], id[ID_VENDOR], id[ID_PRODUCT], id[ID_VERSION]);

	for (i = 0 ; i < 2 ; i++) {
		ioctl(fds[i], EVIOCGNAME(sizeof(name)), name);
		printf("Input device name: \"%s\"\n", name);

		memset(bit, 0, sizeof(bit));
		ioctl(fds[i], EVIOCGBIT(0, EV_MAX), bit[0]);
		printf("Supported events:\n");

		for (type = 0; type < EV_MAX; type++) {
			if (test_bit(type, bit[0]) && type != EV_REP) {
				if (type == EV_SYN) continue;
				ioctl(fds[i], EVIOCGBIT(type, KEY_MAX), bit[type]);
				for (code = 0; code < KEY_MAX; code++)
					if (test_bit(code, bit[type])) {
						printf("    Event type %d code %d\n", type, code);
						if (type == EV_ABS)
							print_absdata(fds[i], code);
					}
			}
		}
	}

	return 0;
}

static int read_rotary_events(int *fds)
{
	struct input_event ev[64];
	int i, j, rd;
	fd_set rdfs, dup;

	FD_ZERO(&rdfs);
	FD_SET(fds[0], &rdfs);
	FD_SET(fds[1], &rdfs);

	while (!stop) {
		dup = rdfs;
		if (select(fds[1] + 1, &dup, NULL, NULL, NULL) < 0) {
			perror("\nledpad: select");
			return 1;
		}

		if (stop)
			break;
		
		for (i = 0 ; i < 2 ; i++)
			if (FD_ISSET(fds[i], &dup)) {
				rd = read(fds[i], ev, sizeof(ev));
				if (rd < (int) sizeof(struct input_event)) {
					perror("\nledpad: error reading");
					return 1;
				}

				for (j = 0; j < rd / sizeof(struct input_event); j++) {
					unsigned int type, code;

					type = ev[j].type;
					code = ev[j].code;

					printf("%s: %d, Event %d: time %ld.%06ld, ", __func__, fds[i], j, ev[j].time.tv_sec, ev[j].time.tv_usec);

					if (type == EV_SYN) {
						printf("EV_SYN: type %d code %d\n", type, code);
					} else {
						switch(code) {
							case ABS_WHEEL:
								printf("ABS_WHEEL type %d code %d value %d\n", type, code, ev[j].value);
								//TODO: temporary demo codes
								if (i == 0)
									(ev[j].value > 0) ? control_led(LED_ENC1_CW) : control_led(LED_ENC1_CCW);
								else
									(ev[j].value > 0) ? control_led(LED_ENC2_CW) : control_led(LED_ENC2_CCW);
								break;
							case KEY_ENTER:
								printf("KEY_ENTER type %d code %d value %d\n", type, code, ev[j].value);
								//TODO: temporary demo codes
								if (i == 0 && ev[j].value)
									control_keypad(true);
								else if (i == 1 && ev[j].value)
									control_keypad(false);
								break;
							default:
								printf("EVENT: type %d code %d value %d\n", type, code, ev[j].value);
						}
					}
				}
			}
	}

	ioctl(fds[0], EVIOCGRAB, (void*)0);
	ioctl(fds[1], EVIOCGRAB, (void*)0);

	return EXIT_SUCCESS;
}

static int capture_rotary_events()
{
	int fd[2];
	
	if ((fd[0] = open(DEV_ROTARY_ENC1, O_RDONLY)) < 0 ||
		(fd[1] = open(DEV_ROTARY_ENC2, O_RDONLY)) < 0) {
		perror("ledpad: no device input event file");
		if (errno == EACCES && getuid() != 0)
			fprintf(stderr, "You do not have access to %s, %s.\n", DEV_ROTARY_ENC1, DEV_ROTARY_ENC2);
		return EXIT_FAILURE;
	}
	
	if (!isatty(fileno(stdout)))
		setbuf(stdout, NULL);

	if (print_rotary_info(fd))
		return EXIT_FAILURE;

	printf("%s: Capturing %d %d ... (interrupt to exit)\n", __func__, fd[0], fd[1]);
	
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	return read_rotary_events(fd);
}

int main(int argc, char *argv[]) {
    printf("Start LED pad app... reading pads.. \n");

	control_keypad(true);

	capture_rotary_events();

    return 0;
}
