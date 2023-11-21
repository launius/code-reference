/*
 * CST820 Touchscreen Driver
 *
 * Copyright (c) 2023 Yunjae Lim <launius@gmail.com>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

/*
 * 1. How to build the driver
 *

+++ b/arch/arm64/boot/dts/rockchip/rk3588-product.dts
+       touch@15 {
+               compatible = "vendor,cst820";
+               reg = <0x15>;
+               interrupt-parent = <&gpio1>;
+               interrupts = <RK_PB5 IRQ_TYPE_EDGE_RISING>;
+               irq-gpios = <&gpio1 RK_PB5 GPIO_ACTIVE_HIGH>;
+               status = "okay";
+        };

+++ b/drivers/input/touchscreen/Kconfig
+config TOUCHSCREEN_CST820
+       tristate "CST820 touchscreen support"
+       depends on I2C
+       help
+         Say Y here if you want to use a touchscreen using CST820 controller.
+
+         If unsure, say N.
+
+         To compile this driver as a module, choose M here: the module
+         will be called cst820.

+++ b/drivers/input/touchscreen/Makefile
+obj-$(CONFIG_TOUCHSCREEN_CST820)       += cst820.o

 *
 * 2. How to test the driver
 *

root@dev-rk3588:~# dmesg |grep cst820
[    9.972336] cst820 4-0015: probing for CST820 on I2C addr 0x15
[    9.973478] cst820 4-0015: version: 00 b7 16 01
[    9.973758] input: CST820 Touchscreen as /devices/platform/feac0000.i2c/i2c-4/4-0015/input/input2
[    9.973868] cst820 4-0015: CST820 touchscreen initialized, IRQ 180.

root@dev-rk3588:~# cat /proc/config.gz |gunzip |grep CONFIG_DYNAMIC_DEBUG
CONFIG_DYNAMIC_DEBUG=y
CONFIG_DYNAMIC_DEBUG_CORE=y

root@dev-rk3588:~# mount -t debugfs none /sys/kernel/debug
root@dev-rk3588:~# mount |grep debugfs
debugfs on /sys/kernel/debug type debugfs (rw,relatime)

root@dev-rk3588:~# echo 'file cst820.c +p' > /sys/kernel/debug/dynamic_debug/control
root@dev-rk3588:~# echo 8 > /proc/sys/kernel/printk
root@dev-rk3588:~# cat /proc/sys/kernel/printk
8       4       1       7

root@dev-rk3588:~# evtest
No device specified, trying to scan all of /dev/input/event*
Available devices:
/dev/input/event2:      CST820 Touchscreen
Select the device event number [0-2]: 2
Input driver version is 1.0.1
Input device ID: bus 0x18 vendor 0x0 product 0x0 version 0x0
Input device name: "CST820 Touchscreen"
Supported events:
  Event type 0 (EV_SYN)
  Event type 1 (EV_KEY)
    Event code 330 (BTN_TOUCH)
  Event type 3 (EV_ABS)
    Event code 0 (ABS_X)
      Value    135
      Min        0
      Max      240
    Event code 1 (ABS_Y)
      Value    297
      Min        0
      Max      320
Properties:
Testing ... (interrupt to exit)
Event: time 1700459846.834936, type 3 (EV_ABS), code 1 (ABS_Y), value 11
Event: time 1700459846.834936, -------------- SYN_REPORT ------------
[  931.000595] cst820 4-0015: touch 00 01 80 08 00 0b, (232, 11)
Event: time 1700459846.850637, type 3 (EV_ABS), code 1 (ABS_Y), value 13
Event: time 1700459846.850637, -------------- SYN_REPORT ------------
[  931.016295] cst820 4-0015: touch 00 01 80 08 00 0d, (232, 13)
Event: time 1700459846.866406, type 3 (EV_ABS), code 1 (ABS_Y), value 15
Event: time 1700459846.866406, -------------- SYN_REPORT ------------
[  931.032064] cst820 4-0015: touch 00 01 80 08 00 0f, (232, 15)
Event: time 1700459846.882344, type 3 (EV_ABS), code 1 (ABS_Y), value 18
Event: time 1700459846.882344, -------------- SYN_REPORT ------------
[  931.048003] cst820 4-0015: touch 00 01 80 08 00 12, (232, 18)

 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#define CST820_WIDTH			240
#define CST820_HEIGHT			320

#define CST820_MESSAGE_SIZE		6

struct cst820_ts {
	struct i2c_client *client;
	struct input_dev *input;
};

static int cst820_ts_i2c_read(struct i2c_client *client,
						u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &reg;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

static int cst820_ts_i2c_readwrite(struct i2c_client *client,
			   u16 wr_len, u8 *wr_buf, u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	dev_info(&client->dev, "wr_buf 0x%x, rd_buf 0x%x\n", *(wrmsg[0].buf), *(wrmsg[1].buf));

	return 0;
}

static int cst820_ts_get_touch(struct i2c_client *client, u8 *buf, int len)
{
	int error;
	u8 reg;

	reg = 0x01;
	error = cst820_ts_i2c_read(client, reg, buf, len);
	if (error) {
		dev_err(&client->dev, "read touch data failed %d\n", error);
		return error;
	}

	return 0;
}

static int cst820_ts_check_version(struct i2c_client *client)
{
	int error;
	u8 reg;
	u8 buf1, buf2[3];

	reg = 0x15;
	error = cst820_ts_i2c_read(client, reg, &buf1, 1);
	if (error) {
		dev_err(&client->dev, "read version failed %d\n", error);
		return error;
	}

	reg = 0xa7;
	error = cst820_ts_i2c_read(client, reg, buf2, 3);
	if (error) {
		dev_err(&client->dev, "read version failed %d\n", error);
		return error;
	}

	dev_info(&client->dev, "version: %02x %02x %02x %02x\n",
								buf1, buf2[0], buf2[1], buf2[2]);

	return 0;
}

static irqreturn_t cst820_ts_irq_handler(int irq, void *dev_id)
{
	struct cst820_ts *ts = dev_id;
	int error;

	u8 buf[CST820_MESSAGE_SIZE];
	unsigned int x = 0, y = 0;

	error = cst820_ts_get_touch(ts->client, buf, sizeof(buf));
	if (error) {
		dev_err_ratelimited(&ts->client->dev, "Unable to read I2C data %d\n", error);
		goto out;
	}

	x = CST820_WIDTH - buf[3];
	y = (buf[4] << 8) | buf[5];

	dev_dbg(&ts->client->dev, "touch %02x %02x %02x %02x %02x %02x, (%d, %d)\n",
						buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], x, y);

	input_report_abs(ts->input, ABS_X, x);
	input_report_abs(ts->input, ABS_Y, y);

	input_sync(ts->input);

out:
	return IRQ_HANDLED;
}

static int cst820_ts_open(struct input_dev *dev)
{
	struct cst820_ts *ts = input_get_drvdata(dev);
	struct i2c_client *client = ts->client;

	enable_irq(client->irq);

	return 0;
}

static void cst820_ts_close(struct input_dev *dev)
{
	struct cst820_ts *ts = input_get_drvdata(dev);
	struct i2c_client *client = ts->client;

	disable_irq(client->irq);
}

static int cst820_ts_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct cst820_ts *ts;
	struct input_dev *input;

	u8 cmd;
	u8 buf[3] = {0};
	int error;

	dev_info(&client->dev, "probing for CST820 on I2C addr 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENODEV;
	}

	error = cst820_ts_check_version(client);
	if (error) {
		dev_err_ratelimited(&client->dev, "Unable to read I2C data %d\n", error);
		return error;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	ts->client = client;
	ts->input = input;

	input->name = "CST820 Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = cst820_ts_open;
	input->close = cst820_ts_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, CST820_WIDTH, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, CST820_HEIGHT, 0, 0);

	input_set_drvdata(input, ts);

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				cst820_ts_irq_handler, IRQF_ONESHOT, client->name, ts);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		return error;
	}

	/* Disable the IRQ, we'll enable it in cst820_ts_open() */
	disable_irq(client->irq);

	error = input_register_device(input);
	if (error) {
		dev_err(&client->dev, "Failed to register input device %d\n", error);
		return error;
	}

	i2c_set_clientdata(client, ts);

	dev_info(&client->dev, "CST820 touchscreen initialized, IRQ %d.\n", client->irq);

	return 0;
}

static int cst820_ts_remove(struct i2c_client *client)
{
	struct cst820_ts *ts = i2c_get_clientdata(client);

	dev_err(&client->dev, "removing for CST820 touchscreen\n");

	return 0;
}

static const struct i2c_device_id cst820_ts_id[] = {
	{ "cst820", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, cst820_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id cst820_ts_of_match[] = {
	{ .compatible = "vendor,cst820" },
	{ }
};
MODULE_DEVICE_TABLE(of, cst820_ts_of_match);
#endif

static struct i2c_driver cst820_ts_driver = {
	.driver = {
		.name = "cst820",
		.of_match_table = cst820_ts_of_match,
	},
	.id_table = cst820_ts_id,
	.probe    = cst820_ts_probe,
	.remove   = cst820_ts_remove,
};
module_i2c_driver(cst820_ts_driver);

MODULE_AUTHOR("Yunjae Lim <launius@gmail.com>");
MODULE_DESCRIPTION("CST820 Touchscreen Driver");
MODULE_LICENSE("GPL v2");
