// SPDX-License-Identifier: GPL-2.0
// Copyright (c) launius@gmail.com

/*
 * The driver supports the LED pad.
 */

/* dts config
&spi1 {
	status = "okay";
	spi-max-frequency = <25000000>;
	pinctrl-names = "default";
	pinctrl-0 = <&spi1m1_clk &spi1m1_miso &spi1m1_mosi>;

	leds {
		compatible = "<manufacturer>,<model>";
		reg = <0>;	// chip select  0:cs0  1:cs1
		spi-max-frequency = <3000000>;
		latch-gpios = <&gpio2 RK_PB1 GPIO_ACTIVE_HIGH>;

		led@0 {
			label = "multicolor:status";
			default-state = "on";
		};
	};
};
*/

/* drivers/leds/Makefile
# LED SPI Drivers
leds-ledpad-objs                        := leds-ledpad-main.o leds-spi.o
obj-$(CONFIG_LEDPAD)                    += leds-ledpad.o
*/

/* How to test SPI LEDs
 * enable CONFIG_LEDPAD kernel config
 * echo 0 > /sys/class/leds/multicolor\:status/brightness (LEDs off)
 * echo 1 > /sys/class/leds/multicolor\:status/brightness (LEDs on, set all leds from user space)
 * echo 127 > /sys/class/leds/multicolor\:status/brightness (LEDs on, set leds in kernel space)
 * echo <num> > /sys/class/leds/multicolor\:status/brightness (LEDs on, write random)
 * 
 * How to test Keypads
 * echo read 1000 1 > /dev/misc_spi_ledpad (read keypads n times)
 * echo timer > /dev/misc_spi_ledpad (start timer to read keypads)
 * echo thread > /dev/misc_spi_ledpad (start thread to read keypads)
 * echo stop > /dev/misc_spi_ledpad (stop reading keypads)
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include <uapi/linux/uleds.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/timer.h>

#define KEYPAD1			0xfe
#define KEYPAD2			0xfb
#define KEYPAD3			0xfd
#define KEYPAD4			0xdf
#define KEYPAD5			0xef
#define KEYPAD6			0xf7
#define KEYPAD7			0x7f
#define KEYPAD8			0xbf

#define SPI_BUF_SIZE		680
#define SPI_BUF_PADDING		50

struct ledpad_chipdef {
	/* SPI byte that will be send to switch the LED off */
	u8	off_value;
	/* SPI byte that will be send to switch the LED to maximum brightness */
	u8	max_value;
};

struct ledpad {
	struct led_classdev		ldev;
	spinlock_t				spi_lock;
	struct spi_device		*spi;
	char					name[LED_MAX_NAME_SIZE];
	struct mutex			mutex;
	const struct ledpad_chipdef	*cdef;
	void					*txbuf;
	unsigned int			txlen;
	unsigned int			speed_hz;
	struct gpio_chip		chip;
	struct gpio_desc		*gpiod;
	char					gpio_read_buf[8];
};

static struct ledpad *xlp_data;

static const struct ledpad_chipdef ledpad_cdef = {
	.off_value = 0x0,
	.max_value = 0x7F,
};

struct control_surface_gpio_map {
    unsigned int state;
    unsigned int event;
};

struct control_surface_gpio_map control_gpios_map[] = {
	{0x01, 0x00},
    {0x01, 0x00},
    {0x01, 0x00},
    {0x01, 0x00},
    {0x01, 0x00},
    {0x01, 0x00},
    {0x01, 0x00},
    {0x01, 0x00},
};

int spi_poll_interval = 50;
static struct timer_list spi_poll_timer;
static struct task_struct *task = NULL;

static int ledpad_brightness_set_blocking(struct led_classdev *dev,
								enum led_brightness brightness);

static int control_surface_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	//gpiochip_get_data(chip);
	unsigned int state = 0;

	state = control_gpios_map[offset].state;

	return !state;
}

static void control_surface_gpio_set(struct gpio_chip *chip, unsigned int offset, int val)
{
	//Not supported
}

#define CONFIG_SPI_SYNC
static int ledpad_spi_read(void *rxbuf, size_t len, unsigned int speed_hz)
{
	int ret = -1;
	struct spi_device *spi = NULL;

#ifdef CONFIG_SPI_SYNC
	struct spi_transfer t = {
			.rx_buf		= rxbuf,
			.len		= len,
			.speed_hz	= speed_hz,
		};
	struct spi_message m;
#endif

	if (!xlp_data) {
		pr_err("ledpad data is NULL\n");
		return ret;
	} else {
		spin_lock_irq(&xlp_data->spi_lock);
		spi = xlp_data->spi;
		spin_unlock_irq(&xlp_data->spi_lock);
	}

	// set to ACTIVE_LOW
	gpiod_set_value_cansleep(xlp_data->gpiod, 1);
	gpiod_set_value_cansleep(xlp_data->gpiod, 0);

	// Perform a single byte read (per shift register)
#ifdef CONFIG_SPI_SYNC
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);
#else
	ret = spi_read(spi, rxbuf, len);
#endif

	return ret;
}

static void ledpad_keypad_handle(void)
{
	static char pre_key = 0xff;
	char key;

	mutex_lock(&xlp_data->mutex);
	ledpad_spi_read(xlp_data->gpio_read_buf, 1, xlp_data->speed_hz);
	key = xlp_data->gpio_read_buf[0];
	mutex_unlock(&xlp_data->mutex);

	if (pre_key != key) {
		printk(KERN_INFO "%s: gpio value %x -> %x\n", __func__, pre_key, key);

		switch (key) {
			case KEYPAD1:
			case KEYPAD2:
			case KEYPAD3:
			case KEYPAD4:
				// TODO: no keypad action in kernel space
				ledpad_brightness_set_blocking(&xlp_data->ldev, LED_HALF);
				break;
			case KEYPAD5:
			case KEYPAD6:
			case KEYPAD7:
			case KEYPAD8:
				ledpad_brightness_set_blocking(&xlp_data->ldev, LED_OFF);
				break;
		}
		
		pre_key = key;
	}

	control_gpios_map[0].state = (key) & 0x01;
	control_gpios_map[1].state = (key >> 1) & 0x01;
	control_gpios_map[2].state = (key >> 2) & 0x01;
	control_gpios_map[3].state = (key >> 3) & 0x01;
	control_gpios_map[4].state = (key >> 4) & 0x01;
	control_gpios_map[5].state = (key >> 5) & 0x01;
	control_gpios_map[6].state = (key >> 6) & 0x01;
	control_gpios_map[7].state = (key >> 7) & 0x01;
}

static void SpiPollTimerHandler(struct timer_list *unused)
{
	ledpad_keypad_handle();

    /*Restarting the timer...*/
    mod_timer( &spi_poll_timer, jiffies + msecs_to_jiffies(spi_poll_interval));
}

static int ledpad_threadfn(void *unused)
{
	while(!kthread_should_stop()) {
		ledpad_keypad_handle();
		msleep(50);
    }

	return 0;
}

static ssize_t misc_spi_read(struct file *file,
			char __user *buf, size_t n, loff_t *offset)
{
	ssize_t status;

	mutex_lock(&xlp_data->mutex);

	status = ledpad_spi_read(xlp_data->gpio_read_buf, n, xlp_data->speed_hz);
	if (status)
		return -EFAULT;

	status = copy_to_user(buf, xlp_data->gpio_read_buf, 1);
	mutex_unlock(&xlp_data->mutex);

	printk(KERN_INFO "%s: 0x%x, status %ld\n", __func__, buf[0], status);
	return status;
}

static ssize_t misc_spi_write(struct file *file,
			const char __user *buf, size_t n, loff_t *offset)
{
	int argc = 0, i;
	char *argv[16];
	char tmp[64];
	char *cmd, *data;
	unsigned int times = 0, size = 0;
	unsigned long us = 0, bytes = 0;

	ktime_t start_time;
	ktime_t end_time;
	ktime_t cost_time;

	if (n >= SPI_BUF_SIZE) {
		memset(xlp_data->txbuf, 0, SPI_BUF_SIZE);
		if (copy_from_user(xlp_data->txbuf, buf, n))
			return -EFAULT;
		xlp_data->txlen = n;

		printk(KERN_INFO "%s: %dbytes > /dev/misc\n", __func__, xlp_data->txlen);

		ledpad_brightness_set_blocking(&xlp_data->ldev, LED_ON);
	} else {
		memset(tmp, 0, sizeof(tmp));
		if (copy_from_user(tmp, buf, n))
			return -EFAULT;

		cmd = tmp;
		data = tmp;

		while (data < (tmp + n)) {
			data = strstr(data, " ");
			if (!data)
				break;
			*data = 0;
			argv[argc] = ++data;
			argc++;
			if (argc >= 16)
				break;
		}
		tmp[n - 1] = 0;

		printk(KERN_INFO "%s: %s > /dev/misc\n", __func__, cmd);

		if (!strcmp(cmd, "read")) {
			// TODO: temporary spi_read test code
			sscanf(argv[0], "%d", &times);
			sscanf(argv[1], "%d", &size);

			start_time = ktime_get();
			for (i = 0; i < times; i++) {
				ledpad_spi_read(xlp_data->gpio_read_buf, 1, xlp_data->speed_hz);
				printk(KERN_INFO "%s: %*ph\n", __func__, 8, xlp_data->gpio_read_buf);
				msleep(25);
			}

			end_time = ktime_get();
			cost_time = ktime_sub(end_time, start_time);
			us = ktime_to_us(cost_time);

			bytes = size * times * 1;
			bytes = bytes * 1000 / us;
			printk(KERN_INFO "spi read %d*%d cost %ldus speed:%ldKB/S\n", size, times, us, bytes);
		} else if (!strcmp(cmd, "timer")) {
			/*Starting the timer.*/
			timer_setup(&spi_poll_timer, SpiPollTimerHandler, 0);
			mod_timer( &spi_poll_timer, jiffies + msecs_to_jiffies(spi_poll_interval));
		} else if (!strcmp(cmd, "thread")) {
			task = kthread_create(ledpad_threadfn, NULL, "readspi");
			if (task)
				wake_up_process(task);
			else
				printk(KERN_ERR "%s: cannot create kthread\n", __func__);
		} else if (!strcmp(cmd, "stop")) {
			if (task) {
				kthread_stop(task);
				task = NULL;
			}

			// TODO: use the thread instead of timer
			del_timer(&spi_poll_timer);
		} else {
			printk(KERN_INFO "echo cmd times size value > /dev/misc_spi_ledpad\n");
			printk(KERN_INFO "echo read 1000 1 > /dev/misc_spi_ledpad\n");
			printk(KERN_INFO "echo timer > /dev/misc_spi_ledpad\n");
			printk(KERN_INFO "echo thread > /dev/misc_spi_ledpad\n");
			printk(KERN_INFO "echo stop > /dev/misc_spi_ledpad\n");
		}
	}

	return n;
}

static int ledpad_brightness_set_blocking(struct led_classdev *dev,
								enum led_brightness brightness)
{
	struct ledpad *lp = container_of(dev, struct ledpad, ldev);
	struct spi_device *spi;
	int ret;
	int i;

	printk(KERN_INFO "%s: %d > %s\n", __func__, brightness, dev->name);

	spin_lock_irq(&lp->spi_lock);
	spi = lp->spi;
	spin_unlock_irq(&lp->spi_lock);

	mutex_lock(&lp->mutex);

	switch(brightness) {
		case LED_OFF:
			memset(lp->txbuf, 0, SPI_BUF_SIZE);
			for (i = SPI_BUF_PADDING + 2 ; i < SPI_BUF_SIZE - SPI_BUF_PADDING - 2 ; i++)
				((u8 *)(lp->txbuf))[i] = 0x11;
			lp->txlen = SPI_BUF_SIZE;
			break;
		case LED_ON:
			for (i = 0 ; i < 680 ; i++)
				printk("%d: 0x%x", i, ((u8 *)(lp->txbuf))[i]);
			break;
		case LED_HALF:
			memset(lp->txbuf, 0, SPI_BUF_SIZE);
			for (i = SPI_BUF_PADDING + 2 ; i < SPI_BUF_SIZE - SPI_BUF_PADDING - 2 ; i++)
				((u8 *)(lp->txbuf))[i] = 0x11;

			// Red colour
			((u8 *)(lp->txbuf))[582] = 0x13;
			((u8 *)(lp->txbuf))[584] = 0x33;
			((u8 *)(lp->txbuf))[585] = 0x33;
			((u8 *)(lp->txbuf))[586] = 0x33;
			((u8 *)(lp->txbuf))[587] = 0x33;
			((u8 *)(lp->txbuf))[590] = 0x31;
			((u8 *)(lp->txbuf))[591] = 0x31;

			((u8 *)(lp->txbuf))[594] = 0x13;
			((u8 *)(lp->txbuf))[596] = 0x33;
			((u8 *)(lp->txbuf))[597] = 0x33;
			((u8 *)(lp->txbuf))[598] = 0x33;
			((u8 *)(lp->txbuf))[599] = 0x33;
			((u8 *)(lp->txbuf))[602] = 0x31;
			((u8 *)(lp->txbuf))[603] = 0x31;

			lp->txlen = SPI_BUF_SIZE;
			break;
		case LED_FULL:
		default:
			memset(lp->txbuf, 0, SPI_BUF_SIZE);
			for (i = SPI_BUF_PADDING ; i < SPI_BUF_SIZE - SPI_BUF_PADDING ; i++)
				((u8 *)(lp->txbuf))[i] = i % 256;
			lp->txlen = SPI_BUF_SIZE;
			break;
	}

	printk(KERN_INFO "%s: hexdump %*ph\n", __func__, lp->txlen, (uint8_t *)lp->txbuf + 50);

	ret = spi_write(spi, lp->txbuf, lp->txlen);
	mutex_unlock(&lp->mutex);

	return ret;
}

static int ledpad_probe(struct spi_device *spi)
{
	struct device_node *child;
	struct device *dev = &spi->dev;
	struct ledpad *data;
	const char *name = "leds-ledpad::";
	const char *state;
	int ret;

	if (!spi)
		return -ENOMEM;

	if (!spi->dev.of_node)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "failed to alloc ledpad data memory.");
		return -ENOMEM;
	}

	child = of_get_next_available_child(dev_of_node(dev), NULL);
	of_property_read_string(child, "label", &name);
	strlcpy(data->name, name, sizeof(data->name));

	data->spi = spi;
	spin_lock_init(&data->spi_lock);
	mutex_init(&data->mutex);
	data->cdef = device_get_match_data(dev);
	data->ldev.name = data->name;
	data->ldev.brightness = LED_OFF;
	data->ldev.max_brightness = data->cdef->max_value - data->cdef->off_value;
	data->ldev.brightness_set_blocking = ledpad_brightness_set_blocking;

	data->txbuf = devm_kzalloc(dev, SPI_BUF_SIZE, GFP_KERNEL);
	if (!data->txbuf) {
		dev_err(dev, "failed to alloc tx memory.");
		return -ENOMEM;
	}

	state = of_get_property(child, "default-state", NULL);
	if (state) {
		if (!strcmp(state, "on")) {
			data->ldev.brightness = data->ldev.max_brightness;
		} else if (strcmp(state, "off")) {
			/* all other cases except "off" */
			dev_err(dev, "default-state can only be 'on' or 'off'");
			mutex_destroy(&data->mutex);
			return -EINVAL;
		}
	}

	ledpad_brightness_set_blocking(&data->ldev, data->ldev.brightness);

	ret = devm_led_classdev_register(&spi->dev, &data->ldev);
	if (ret) {
		mutex_destroy(&data->mutex);
		return ret;
	}

	data->chip.parent = &spi->dev;
	data->chip.owner = THIS_MODULE;
	data->chip.label = dev_name(&spi->dev);
	data->chip.base = -1;
	data->chip.ngpio = ARRAY_SIZE(control_gpios_map);
	data->chip.get = control_surface_gpio_get;
	data->chip.direction_input = NULL;
	data->chip.set = control_surface_gpio_set;
	data->chip.direction_output = NULL;

	ret = devm_gpiochip_add_data(&spi->dev, &data->chip, data);

	data->gpiod = devm_gpiod_get_optional(&spi->dev, "latch", GPIOD_OUT_LOW);
	
	data->speed_hz = spi->max_speed_hz;
	spi_set_drvdata(spi, data);
	xlp_data = data;

	printk(KERN_INFO "%s: name=%s, bus_num=%d, cs=%d, mode=%d, speed=%d\n", __func__,
		spi->modalias, spi->master->bus_num, spi->chip_select, spi->mode, spi->max_speed_hz);

	// TODO: start on boot
	// printk(KERN_INFO "%s: start keypad reading...\n", __func__);
	// task = kthread_run(ledpad_threadfn, NULL, "read spi");
	// if (!task)
	// 	printk(KERN_ERR "%s: cannot create kthread\n", __func__);

	return 0;
}

static int ledpad_remove(struct spi_device *spi)
{
	struct ledpad *data = spi_get_drvdata(spi);

	// TODO: how to check started timer
	del_timer(&spi_poll_timer);

	if (task) {
		kthread_stop(task);
		task = NULL;
	}

	devm_led_classdev_unregister(&spi->dev, &data->ldev);
	mutex_destroy(&data->mutex);

	spin_lock_irq(&data->spi_lock);
	data->spi = NULL;
	spin_unlock_irq(&data->spi_lock);

	printk(KERN_INFO "%s: \n", __func__);

	return 0;
}

static const struct of_device_id ledpad_dt_ids[] = {
	{ .compatible = "<manufacturer>,<model>", .data = &ledpad_cdef },
	{},
};

MODULE_DEVICE_TABLE(of, ledpad_dt_ids);

static struct spi_driver ledpad_driver = {
	.probe		= ledpad_probe,
	.remove		= ledpad_remove,
	.driver = {
		.name			= KBUILD_MODNAME,
		.of_match_table	= ledpad_dt_ids,
	},
};

static const struct file_operations misc_spi_fops = {
	.read = misc_spi_read,
	.write = misc_spi_write,
};

static struct miscdevice misc_spi_ledpad = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "misc_spi_ledpad",
	.fops = &misc_spi_fops,
};

static int __init ledpad_init(void)
{
	int ret = 0;

	printk(KERN_DEBUG "%s\n", __func__);
	
	misc_register(&misc_spi_ledpad);
	ret = spi_register_driver(&ledpad_driver);

	return ret;
}
module_init(ledpad_init);

static void __exit ledpad_exit(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	misc_deregister(&misc_spi_ledpad);
	return spi_unregister_driver(&ledpad_driver);
}
module_exit(ledpad_exit);

//module_spi_driver(ledpad_driver);

MODULE_DESCRIPTION("LED Pad driver");
MODULE_AUTHOR("Yunjae Lim <launius@gmail.com>");
MODULE_ALIAS("spi:leds-ledpad");
MODULE_LICENSE("GPL v2");
