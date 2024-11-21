/*
 * Front panel MCU's GPIO Driver for regmaps between MCU registers and Linux sysfs
 *
 * Copyright (c) 2024 Yunjae Lim <launius@gmail.com>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

/*
 * 1. Device tree configurations
 *

&i2c1 {
	status = "okay";
	pinctrl-0 = <&i2c1_z_pins>;
	pinctrl-names = "default";

	frpanel_mcu: frpanel_mcu@12 {
		compatible = "manufacturer,model-frpanel-mcu";
		reg = <0x12>;

	};
};

 *
 * 2. Build configurations
 *

+++ b/drivers/gpio/Kconfig
+config GPIO_DEVICE_NAME
+       tristate "DEVICE_NAME front panel MCU GPIO support"
+       depends on MFD_DEVICE_NAME_MCU
+       help
+          Say Y here to support DEVICE_NAME MCU GPIOs
+

+++ b/drivers/gpio/Makefile
+obj-$(CONFIG_GPIO_DEVICE_NAME)     += gpio-devname.o

 *
 * 3. Changes in parent driver
 *

+++ b/drivers/mfd/devname-mcu.c
#define MCU_DEVNAME_MAX_REG					0x50

static const struct regmap_config devname_mcu_regmap_config = {
	.reg_bits	= 8,
	.reg_stride	= 1,
	.val_bits	= 8,
	.max_register	= MCU_DEVNAME_MAX_REG,
	.volatile_reg	= devname_mcu_reg_volatile,
	.writeable_reg	= devname_mcu_reg_writeable,
	.cache_type	= REGCACHE_RBTREE,
};

static struct mfd_cell devname_frpanel_mcu_cells[] = {
+       { .name = "device-name-gpio", },
};

static struct devname_mcu_conf devname_frpanel_mcu_conf_data = {
	.cells = devname_frpanel_mcu_cells,
	.count = ARRAY_SIZE(devname_frpanel_mcu_cells),
	.mcu_type = FRPANEL_MCU,
};

static const struct of_device_id devname_mcu_of_match[] = {
	{ .compatible = "manufacturer,devname-frpanel-mcu", .data = &devname_frpanel_mcu_conf_data },
	{},
};

static int devname_mcu_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	data->regmap = devm_regmap_init_i2c(client, &devname_mcu_regmap_config);

 *
 * 4. Load driver module
 *

root@system:~# lsmod
Module                  Size  Used by
gpio_devname           16384  0

root@system:~# dmesg |grep -i device-name
[    1.934368] devname-gpio device-name-gpio: Registered DEVICE_NAME GPIO driver

 *
 * 5. Read/Write MCU registers via sysfs
 *

root@system:/sys/devices/platform/soc/ffd00000.bus/ffd1e000.i2c/i2c-2/2-0012/device-name-gpio# ls -l
total 0
lrwxrwxrwx 1 root root    0 Nov 21 01:09 driver -> ../../../../../../../../bus/platform/drivers/devname-gpio
-rw-r--r-- 1 root root 4096 Nov 21 01:09 driver_override
-r--r--r-- 1 root root 4096 Nov 21 01:09 modalias
drwxr-xr-x 2 root root    0 Nov 21 01:08 power
drwxr-xr-x 2 root root    0 Nov 21 01:08 mcu_gpio_frpanel
lrwxrwxrwx 1 root root    0 Nov 21 01:09 subsystem -> ../../../../../../../../bus/platform
-rw-r--r-- 1 root root 4096 Jan  1  1970 uevent

root@system:/sys/devices/platform/soc/ffd00000.bus/ffd1e000.i2c/i2c-2/2-0012/device-name-gpio# ls -l mcu_gpio_frpanel/
total 0
-rw-r--r-- 1 root root 4096 Nov 21 01:11 button_reg
-r--r--r-- 1 root root 4096 Nov 21 01:11 enc_in0
-r--r--r-- 1 root root 4096 Nov 21 01:11 enc_in1
-r--r--r-- 1 root root 4096 Nov 21 01:11 enc_in2
-rw-r--r-- 1 root root 4096 Nov 21 01:11 enc_sw_reg
-rw-r--r-- 1 root root 4096 Nov 21 01:11 led_brightness
-rw-r--r-- 1 root root 4096 Nov 21 01:11 led_color
-rw-r--r-- 1 root root 4096 Nov 21 01:11 led_enc_pos0
-rw-r--r-- 1 root root 4096 Nov 21 01:11 led_enc_pos1
-rw-r--r-- 1 root root 4096 Nov 21 01:11 led_enc_pos2

root@system:/sys/devices/platform/soc/ffd00000.bus/ffd1e000.i2c/i2c-2/2-0012/device-name-gpio# cat mcu_gpio_frpanel/button_reg
64 (8bits ON/OFF status, e.g. 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80)
root@system:/sys/devices/platform/soc/ffd00000.bus/ffd1e000.i2c/i2c-2/2-0012/device-name-gpio# echo 257 > mcu_gpio_frpanel/led_color (divided by max 256 colors)

 */

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/devname-mcu.h>

#define MCU_DEVNAME_FRPANEL_BTN_REG				0x07
#define MCU_DEVNAME_FRPANEL_ENC_SW_REG			0x08
#define MCU_DEVNAME_FRPANEL_ENC_IN				0x09
#define MCU_DEVNAME_FRPANEL_LED_COLOR			0x0C
#define MCU_DEVNAME_FRPANEL_LED_BRIGHTNESS		0x16
#define MCU_DEVNAME_FRPANEL_LED_ENC_POS			0x17

struct devname_gpio_priv {
	struct gpio_chip chip;
	struct devname_mcu *mcu;
};

struct devname_gpio_map {
	unsigned int reg;
	unsigned int bit;
};

const struct devname_gpio_map gpios_map[] = {
	{MCU_DEVNAME_FRPANEL_BTN_REG, 0},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 1},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 2},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 3},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 4},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 5},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 6},
	{MCU_DEVNAME_FRPANEL_BTN_REG, 7},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 0},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 1},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 2},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 3},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 4},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 5},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 6},
	{MCU_DEVNAME_FRPANEL_ENC_SW_REG, 7},
};

#define STORE_FUNC(_name, _reg) \
static ssize_t \
_name##_store(struct device *dev, struct device_attribute *attr, \
		 const char *buf, size_t count) \
{ \
	int ret; \
	ret = devname_reg_set(dev, buf, _reg); \
	if (ret != 0) \
		return ret; \
	return count; \
}

#define SHOW_FUNC(_name, _reg) \
static ssize_t \
_name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	return devname_reg_get(dev, buf, _reg); \
}

#define ENC_IN_SHOW(_index) \
static ssize_t \
enc_in##_index##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	return devname_reg_get(dev, buf, MCU_DEVNAME_FRPANEL_ENC_IN+_index); \
}

#define LED_ENC_POS_STORE(_index) \
static ssize_t \
led_enc_pos##_index##_store(struct device *dev, struct device_attribute *attr, \
		 const char *buf, size_t count) { \
	struct devname_gpio_priv *priv = dev_get_drvdata(dev); \
	int ret; \
	long data; \
	\
	ret = kstrtol(buf, 10, &data); \
	if (ret != 0) \
		return ret; \
	\
	ret = regmap_write(priv->mcu->regmap, MCU_DEVNAME_FRPANEL_LED_ENC_POS+_index, (unsigned int)data); \
	if (ret != 0) \
		return ret; \
	\
	return count; \
}

#define LED_ENC_POS_SHOW(_index) \
static ssize_t \
led_enc_pos##_index##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	return devname_reg_get(dev, buf, MCU_DEVNAME_FRPANEL_LED_ENC_POS+_index); \
}

static int devname_reg_get(struct device *dev, char *buf, unsigned int reg)
{
	struct devname_gpio_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->mcu->regmap, reg, &val);
	if (ret)
		return ret;

//	printk(KERN_INFO "%s: reg %d val %#x\n", __func__, reg, val);

	return snprintf(buf, 4, "%u\n", val);
}

static int devname_reg_set(struct device *dev, const char *buf, unsigned int reg)
{
	struct devname_gpio_priv *priv = dev_get_drvdata(dev);
	int ret;
	long data;

	ret = kstrtol(buf, 10, &data);
	if (ret != 0)
		return ret;

	return regmap_write(priv->mcu->regmap, reg, data);
}

static ssize_t
led_color_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct devname_gpio_priv *priv = dev_get_drvdata(dev);
	int ret;
	long data;
	const int num_of_clr = 256;
	unsigned int ledIndex = 0;
	unsigned int colorToSet = 0;

	ret = kstrtol(buf, 10, &data);
	if (ret != 0)
		return ret;

	ledIndex = data/num_of_clr;
	colorToSet = data%num_of_clr;

	ret = regmap_write(priv->mcu->regmap, MCU_DEVNAME_FRPANEL_LED_COLOR+ledIndex, colorToSet);
	if (ret != 0)
		return ret;

	return count;
}
static ssize_t
led_color_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return devname_reg_get(dev, buf, MCU_DEVNAME_FRPANEL_LED_COLOR);
}

static ssize_t
led_brightness_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct devname_gpio_priv *priv = dev_get_drvdata(dev);
	int ret;
	long data;

	ret = kstrtol(buf, 10, &data);
	if (ret != 0)
		return ret;

	ret = regmap_write(priv->mcu->regmap, MCU_DEVNAME_FRPANEL_LED_BRIGHTNESS, (unsigned int)data);
	if (ret != 0)
		return ret;

	return count;
}
static ssize_t
led_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return devname_reg_get(dev, buf, MCU_DEVNAME_FRPANEL_LED_BRIGHTNESS);
}

STORE_FUNC(button_reg, MCU_DEVNAME_FRPANEL_BTN_REG)
SHOW_FUNC(button_reg, MCU_DEVNAME_FRPANEL_BTN_REG)

STORE_FUNC(enc_sw_reg, MCU_DEVNAME_FRPANEL_ENC_SW_REG)
SHOW_FUNC(enc_sw_reg, MCU_DEVNAME_FRPANEL_ENC_SW_REG)

ENC_IN_SHOW(0)
ENC_IN_SHOW(1)
ENC_IN_SHOW(2)

LED_ENC_POS_STORE(0)
LED_ENC_POS_SHOW(0)
LED_ENC_POS_STORE(1)
LED_ENC_POS_SHOW(1)
LED_ENC_POS_STORE(2)
LED_ENC_POS_SHOW(2)

static DEVICE_ATTR_RW(button_reg);
static DEVICE_ATTR_RW(enc_sw_reg);
static DEVICE_ATTR_RO(enc_in0);
static DEVICE_ATTR_RO(enc_in1);
static DEVICE_ATTR_RO(enc_in2);
static DEVICE_ATTR_RW(led_color);
static DEVICE_ATTR_RW(led_brightness);
static DEVICE_ATTR_RW(led_enc_pos0);
static DEVICE_ATTR_RW(led_enc_pos1);
static DEVICE_ATTR_RW(led_enc_pos2);

static struct attribute *mcu_gpio_frpanel_attrs[] = {
	&dev_attr_button_reg.attr,
	&dev_attr_enc_sw_reg.attr,
	&dev_attr_enc_in0.attr,
	&dev_attr_enc_in1.attr,
	&dev_attr_enc_in2.attr,
	&dev_attr_led_color.attr,
	&dev_attr_led_brightness.attr,
	&dev_attr_led_enc_pos0.attr,
	&dev_attr_led_enc_pos1.attr,
	&dev_attr_led_enc_pos2.attr,
	NULL,
};

static struct attribute_group mcu_gpio_frpanel_attr_group = {
	.attrs = mcu_gpio_frpanel_attrs,
	.name = "mcu_gpio_frpanel"
};

static int devname_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct devname_gpio_priv *priv = gpiochip_get_data(chip);
	unsigned int reg, bit = 0;
	unsigned int val;
	int ret;

	reg = gpios_map[offset].reg;
	bit = gpios_map[offset].bit;

	ret = regmap_read(priv->mcu->regmap, reg, &val);
	if (ret)
		return ret;

	return !!(val & BIT(bit));
}

static void devname_gpio_set(struct gpio_chip *chip, unsigned int offset, int val)
{
	struct devname_gpio_priv *priv = gpiochip_get_data(chip);
	unsigned int reg, bit = 0;

	reg = gpios_map[offset].reg;
	bit = gpios_map[offset].bit;
	
	if (val > 0)
		regmap_update_bits(priv->mcu->regmap, reg, BIT(bit), BIT(bit));
	else
		regmap_update_bits(priv->mcu->regmap, reg, BIT(bit), 0);
}

static int devname_gpio_probe(struct platform_device *pdev)
{
	struct devname_mcu *mcu = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct devname_gpio_priv *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mcu = mcu;
	priv->chip.parent = pdev->dev.parent;
	priv->chip.owner = THIS_MODULE;
	priv->chip.label = dev_name(pdev->dev.parent);
	priv->chip.base = -1;
	priv->chip.ngpio = ARRAY_SIZE(gpios_map);
	priv->chip.get = devname_gpio_get;
	priv->chip.direction_input = NULL;
 	priv->chip.set = devname_gpio_set;
	priv->chip.direction_output = NULL;

	platform_set_drvdata(pdev, priv);

	err = sysfs_create_group(&dev->kobj, &mcu_gpio_frpanel_attr_group);
	if (err)
		return err;

    dev_info(&pdev->dev, "Registered DEVICE_NAME GPIO driver\n");

	return devm_gpiochip_add_data(&pdev->dev, &priv->chip, priv);
}

static const struct platform_device_id devname_gpio_id_table[] = {
	{ .name = "device-name-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(platform, devname_gpio_id_table);

static struct platform_driver devname_gpio_driver = {
	.driver	= {
		.name = "devname-gpio",
		.owner = THIS_MODULE,
	},
	.id_table = devname_gpio_id_table,
	.probe	= devname_gpio_probe,
};
module_platform_driver(devname_gpio_driver);

MODULE_AUTHOR("Yunjae Lim <launius@gmail.com>");
MODULE_DESCRIPTION("DEVICE_NAME MCU GPIO driver");
MODULE_LICENSE("GPL");
