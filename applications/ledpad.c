// (c) 2022 Yunjae Lim <launius@gmail.com> 

/*
 * This application is handling LED control by GPIO, key input events by reading SPI
 * and rotary encoder events from linux input driver.
 */

#include "ledpad.h"
#include "ledpad-api.h"

#include <linux/version.h>
#include <linux/input.h>

#include <signal.h>
#include <assert.h>

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

int control_led(char c)
{
	int fd, rc;

	fd = open(SYS_LEDS, O_WRONLY);
	rc = write(fd, &c, 1);

	close(fd);
	return rc;
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
			print_debug("      %s %6d\n", absval[k], abs[k]);
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

	print_debug("Input driver version is %d.%d.%d\n",
		version >> 16, (version >> 8) & 0xff, version & 0xff);

	ioctl(fds[0], EVIOCGID, id);
	print_debug("Input device ID: bus 0x%x vendor 0x%x product 0x%x version 0x%x\n",
		id[ID_BUS], id[ID_VENDOR], id[ID_PRODUCT], id[ID_VERSION]);

	for (i = 0 ; i < 2 ; i++) {
		ioctl(fds[i], EVIOCGNAME(sizeof(name)), name);
		print_debug("Input device name: \"%s\"\n", name);

		memset(bit, 0, sizeof(bit));
		ioctl(fds[i], EVIOCGBIT(0, EV_MAX), bit[0]);
		print_debug("Supported events:\n");

		for (type = 0; type < EV_MAX; type++) {
			if (test_bit(type, bit[0]) && type != EV_REP) {
				if (type == EV_SYN) continue;
				ioctl(fds[i], EVIOCGBIT(type, KEY_MAX), bit[type]);
				for (code = 0; code < KEY_MAX; code++)
					if (test_bit(code, bit[type])) {
						print_debug("    Event type %d code %d\n", type, code);
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
					// TODO: refactoring to separate into handle_event() function
					unsigned int type, code;

					type = ev[j].type;
					code = ev[j].code;

					print_debug("%d, Event %d: time %ld.%06ld, ", fds[i], j, ev[j].time.tv_sec, ev[j].time.tv_usec);

					if (type == EV_SYN) {
						print_debug("EV_SYN: type %d code %d\n", type, code);
					} else {
						switch(code) {
							case ABS_WHEEL:
								print_debug("ABS_WHEEL type %d code %d value %d\n", type, code, ev[j].value);

								leds_init();
								if (i == 0)
									(ev[j].value > 0) ? leds_ring1_on(ENC_CW) : leds_ring1_on(ENC_CCW);
								else
									(ev[j].value > 0) ? leds_ring2_on(ENC_CW) : leds_ring1_on(ENC_CCW);
								leds_activate();
								break;
							case KEY_ENTER:
								print_debug("KEY_ENTER type %d code %d value %d\n", type, code, ev[j].value);
								//TODO: temporary demo codes
								if (i == 0 && ev[j].value) {
									// do something by rotary push1
								}
								else if (i == 1 && ev[j].value) {
									// do something by rotary push2
								}
								break;
							default:
								print_debug("EVENT: type %d code %d value %d\n", type, code, ev[j].value);
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
			print_debug("You do not have access to %s, %s.\n", DEV_ROTARY_ENC1, DEV_ROTARY_ENC2);
		return EXIT_FAILURE;
	}

	if (!isatty(fileno(stdout)))
		setbuf(stdout, NULL);

	if (print_rotary_info(fd))
		return EXIT_FAILURE;

	print_debug("%s: Capturing %d %d ... (interrupt to exit)\n", __func__, fd[0], fd[1]);

	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	return read_rotary_events(fd);
}

int control_keypad_events(bool on)
{
	int fd, rc;
	char *cmd;
	unsigned int size;
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
			print_debug("You do not have access to %s.\n", DEV_MISC_SPI);
		return EXIT_FAILURE;
	}

	rc = write(fd, cmd, size);
	close(fd);

	state ^= true;

	return rc;
}

int read_keypad_events(unsigned char *buf)
{
	int fd, rc;
	unsigned int size = 1;

	if ((fd = open(DEV_MISC_SPI, O_RDONLY)) < 0) {
		perror("ledpad misc_spi");
		if (errno == EACCES && getuid() != 0)
			print_debug("You do not have access to %s.\n", DEV_MISC_SPI);
		return EXIT_FAILURE;
	}

	rc = read(fd, buf, size);
	close(fd);

	print_info("%d, 0x%x 0x%x 0x%x..\n", rc, buf[0], buf[1], buf[2]);
	return rc;
}

int main(int argc, char *argv[])
{
	int opt;

    printf("Start LED pad app...\n");

	while(1) {
		printf("== ledpad APIs test menu ==\n");
		printf("1. leds_all_off\n");
		printf("2. leds_all_on\n");
		printf("3-17. leds_pad_on(pad#, color#)\n");
		printf("4-18. leds_pad_off(pad#)\n");
		printf("20. leds_pad1234_on\n");

		printf("21. leds_ring1_on cw\n");
		printf("22. leds_ring1_on ccw\n");
		printf("23. leds_ring2_on cw\n");
		printf("24. leds_ring2_on ccw\n");

		printf("31. leds_inc_on\n");
		printf("32. leds_dec_on\n");

		printf("99. exit\n");
		printf("input: ");
		assert(scanf("%d", &opt) > 0);

		leds_init();

		switch(opt) {
			case 1:			leds_all_off();				break;
			case 2:			leds_all_on();				break;

			case 3:			leds_pad_on(PAD1, PAD1_ON_RGB);			break;
			case 4:			leds_pad_off(PAD1);						break;
			case 5:			leds_pad_on(PAD2, PAD2_ON_RGB);			break;
			case 6:			leds_pad_off(PAD2);						break;
			case 7:			leds_pad_on(PAD3, PAD3_ON_RGB);			break;
			case 8:			leds_pad_off(PAD3);						break;
			case 9:			leds_pad_on(PAD4, PAD7_ON_RGB);			break;
			case 10:		leds_pad_off(PAD4);						break;

			case 11:		leds_pad_on(PAD_AUDIO_IN, PAD1_ON_RGB);			break;
			case 12:		leds_pad_off(PAD_AUDIO_IN);						break;
			case 13:		leds_pad_on(PAD_VIDEO_IN, PAD2_ON_RGB);			break;
			case 14:		leds_pad_off(PAD_VIDEO_IN);						break;
			case 15:		leds_pad_on(PAD_LEFT_IN, PAD3_ON_RGB);			break;
			case 16:		leds_pad_off(PAD_LEFT_IN);						break;
			case 17:		leds_pad_on(PAD_RIGHT_IN, PAD7_ON_RGB);			break;
			case 18:		leds_pad_off(PAD_RIGHT_IN);						break;

			case 20:
				leds_pad_on(PAD1, PAD1_ON_RGB);
				leds_pad_on(PAD2, PAD2_ON_RGB);
				leds_pad_on(PAD3, PAD3_ON_RGB);
				leds_pad_on(PAD4, PAD7_ON_RGB);
				break;

			case 21:		leds_ring1_on(ENC_CW);		break;
			case 22:		leds_ring1_on(ENC_CCW);		break;
			case 23:		leds_ring2_on(ENC_CW);		break;
			case 24:		leds_ring2_on(ENC_CCW);		break;

			case 31:		leds_inc_on(PAD1_ON_RGB);		break;
			case 32:		leds_dec_on(PAD2_ON_RGB);		break;

			case 99:
				goto rotary_test;
				break;
			default:
				printf("input error!\n");
				break;
		}

		leds_activate();
	}

rotary_test:
	control_keypad_events(true);

	capture_rotary_events();

    return 0;
}
