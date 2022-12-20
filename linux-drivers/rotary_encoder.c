// SPDX-License-Identifier: GPL-2.0-only
/*
 * rotary_encoder.c
 *
 * (c) 2009 Daniel Mack <daniel@caiaq.de>
 * Copyright (C) 2011 Johan Hovold <jhovold@gmail.com>
 *
 * state machine code inspired by code from Tim Ruetz
 *
 * A generic driver for rotary encoders connected to GPIO lines.
 * See file:Documentation/input/devices/rotary-encoder.rst for more information
 */

/*
 * (c) 2022 Yunjae Lim <launius@gmail.com>
 *
 * The expanded device tree and driver supports the 2 rotary encoders and push-buttons.
 *
 * kernel-5.10.17/arch/arm64/boot/dts/rockchip/rk3308-product.dts
/{
	rotary0: rotary-encoder-1 {
		compatible = "rotary-encoder";
		pinctrl-names = "default";
		pinctrl-0 = <&rotary0_gpios &rotary0_push>;

		gpios = <&gpio4 RK_PA0 GPIO_ACTIVE_HIGH>, <&gpio4 RK_PA1 GPIO_ACTIVE_HIGH>;
		linux,axis = <8>;	// REL_WHEEL
		rotary-encoder,encoding = "gray";
		rotary-encoder,relative-axis;
		rotary-encoder,steps-per-period = <2>;
		wakeup-source;

		push-gpios = <&gpio4 RK_PA2 GPIO_ACTIVE_HIGH>;
		linux,push-code = <KEY_ENTER>;
		linux,push-type = <EV_KEY>;
	};

	rotary1: rotary-encoder-2 {
		compatible = "rotary-encoder";
		pinctrl-names = "default";
		pinctrl-0 = <&rotary1_gpios &rotary1_push>;

		gpios = <&gpio4 RK_PA3 GPIO_ACTIVE_HIGH>, <&gpio4 RK_PA4 GPIO_ACTIVE_HIGH>;
		linux,axis = <8>;	// ABS_WHEEL
		rotary-encoder,steps = <30>;
		rotary-encoder,encoding = "gray";
		rotary-encoder,rollover;
		rotary-encoder,steps-per-period = <2>;
		wakeup-source;

		push-gpios = <&gpio4 RK_PA5 GPIO_ACTIVE_HIGH>;
		linux,push-code = <KEY_ENTER>;
		linux,push-type = <EV_KEY>;
	};
};

&pinctrl {
	rotary {
		rotary0_gpios: rotary0-gpios {
			rockchip,pins =
				<4 RK_PA0 RK_FUNC_GPIO &pcfg_pull_up>,
				<4 RK_PA1 RK_FUNC_GPIO &pcfg_pull_up>;
		};
		
		rotary0_push: rotary0-push {
			rockchip,pins =
				<4 RK_PA2 RK_FUNC_GPIO &pcfg_pull_up>;
		};

		rotary1_gpios: rotary1-gpios {
			rockchip,pins =
				<4 RK_PA3 RK_FUNC_GPIO &pcfg_pull_up>,
				<4 RK_PA4 RK_FUNC_GPIO &pcfg_pull_up_2ma>;
		};

		rotary1_push: rotary1-push {
			rockchip,pins =
				<4 RK_PA5 RK_FUNC_GPIO &pcfg_pull_up_2ma>;
		};
	};
};
 *
 * How to test rotary input devices
 *
 * enable CONFIG_INPUT_GPIO_ROTARY_ENCODER from kernel configs
 * include evtest package in yocto image.bb file
 *
 * $ insmod /lib/modules/5.10.17-yocto/kernel/drivers/input/misc/rotary_encoder.ko
 * $ evtest /dev/input/event0
 * $ evtest /dev/input/event1
 *
Input driver version is 1.0.1
Input device ID: bus 0x19 vendor 0x0 product 0x0 version 0x0
Input device name: "rotary-encoder-2"
Supported events:
  Event type 0 (EV_SYN)
  Event type 1 (EV_KEY)
    Event code 28 (KEY_ENTER)
  Event type 3 (EV_ABS)
    Event code 8 (ABS_WHEEL)
      Value      0
      Min        0
      Max       30
      Flat       1
Properties:
Testing ... (interrupt to exit)
Event: time 1520599071.1520599071, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 1
Event: time 1520599071.1520599071, -------------- SYN_REPORT ------------
Event: time 1520599072.1520599072, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 2
Event: time 1520599072.1520599072, -------------- SYN_REPORT ------------
Event: time 1520599072.1520599072, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 3
Event: time 1520599072.1520599072, -------------- SYN_REPORT ------------
Event: time 1520599073.1520599073, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 4
Event: time 1520599073.1520599073, -------------- SYN_REPORT ------------
Event: time 1520599073.1520599073, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 3
Event: time 1520599073.1520599073, -------------- SYN_REPORT ------------
Event: time 1520599074.1520599074, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 2
Event: time 1520599074.1520599074, -------------- SYN_REPORT ------------
Event: time 1520599074.1520599074, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 1
Event: time 1520599074.1520599074, -------------- SYN_REPORT ------------
Event: time 1520599075.1520599075, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 0
Event: time 1520599075.1520599075, -------------- SYN_REPORT ------------
Event: time 1520599076.1520599076, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 29
Event: time 1520599076.1520599076, -------------- SYN_REPORT ------------
Event: time 1520599077.1520599077, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 28
Event: time 1520599077.1520599077, -------------- SYN_REPORT ------------
Event: time 1520599078.1520599078, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 27
Event: time 1520599078.1520599078, -------------- SYN_REPORT ------------
Event: time 1520599078.1520599078, type 3 (EV_ABS), code 8 (ABS_WHEEL), value 26
Event: time 1520599078.1520599078, -------------- SYN_REPORT ------------
Event: time 1520599116.1520599116, type 1 (EV_KEY), code 28 (KEY_ENTER), value 1
Event: time 1520599116.1520599116, -------------- SYN_REPORT ------------
Event: time 1520599116.1520599116, type 1 (EV_KEY), code 28 (KEY_ENTER), value 0
Event: time 1520599116.1520599116, -------------- SYN_REPORT ------------
Event: time 1520599116.1520599116, type 1 (EV_KEY), code 28 (KEY_ENTER), value 1
Event: time 1520599116.1520599116, -------------- SYN_REPORT ------------
Event: time 1520599117.1520599117, type 1 (EV_KEY), code 28 (KEY_ENTER), value 0
Event: time 1520599117.1520599117, -------------- SYN_REPORT ------------
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/property.h>

#define DRV_NAME "rotary-encoder"

enum rotary_encoder_encoding {
	ROTENC_GRAY,
	ROTENC_BINARY,
};

struct rotary_encoder {
	struct input_dev *input;

	struct mutex access_mutex;

	u32 steps;
	u32 axis;
	bool relative_axis;
	bool rollover;
	enum rotary_encoder_encoding encoding;

	unsigned int pos;

	struct gpio_descs *gpios;

	unsigned int *irq;

	struct gpio_desc *push_gpio;
	unsigned int push_code;
	unsigned int push_type;
	unsigned int push_irq;

	bool armed;
	signed char dir;	/* 1 - clockwise, -1 - CCW */

	unsigned int last_stable;
};

static unsigned int rotary_encoder_get_state(struct rotary_encoder *encoder)
{
	int i;
	unsigned int ret = 0;

	for (i = 0; i < encoder->gpios->ndescs; ++i) {
		int val = gpiod_get_value_cansleep(encoder->gpios->desc[i]);

		/* convert from gray encoding to normal */
		if (encoder->encoding == ROTENC_GRAY && ret & 1)
			val = !val;

		ret = ret << 1 | val;
	}

	return ret & 3;
}

static void rotary_encoder_report_event(struct rotary_encoder *encoder)
{
	if (encoder->relative_axis) {
		input_report_rel(encoder->input,
				 encoder->axis, encoder->dir);
	} else {
		unsigned int pos = encoder->pos;

		if (encoder->dir < 0) {
			/* turning counter-clockwise */
			if (encoder->rollover)
				pos += encoder->steps;
			if (pos)
				pos--;
		} else {
			/* turning clockwise */
			if (encoder->rollover || pos < encoder->steps)
				pos++;
		}

		if (encoder->rollover)
			pos %= encoder->steps;

		encoder->pos = pos;
		input_report_abs(encoder->input, encoder->axis, encoder->pos);
	}

	input_sync(encoder->input);

	printk(KERN_DEBUG "%s: steps %d dir %d pos %d\n", __func__, encoder->steps, encoder->dir, encoder->pos);
}

static irqreturn_t rotary_encoder_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	unsigned int state;

	mutex_lock(&encoder->access_mutex);

	state = rotary_encoder_get_state(encoder);

	switch (state) {
	case 0x0:
		if (encoder->armed) {
			rotary_encoder_report_event(encoder);
			encoder->armed = false;
		}
		break;

	case 0x1:
	case 0x3:
		if (encoder->armed)
			encoder->dir = 2 - state;
		break;

	case 0x2:
		encoder->armed = true;
		break;
	}

	printk(KERN_DEBUG "%s: irq %d, state %d dir %d armed %d\n", __func__, irq, state, encoder->dir, encoder->armed);
	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t rotary_encoder_half_period_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	unsigned int state;

	mutex_lock(&encoder->access_mutex);

	state = rotary_encoder_get_state(encoder);

	if (state & 1) {
		encoder->dir = ((encoder->last_stable - state + 1) % 4) - 1;
	} else {
		if (state != encoder->last_stable) {
			rotary_encoder_report_event(encoder);
			encoder->last_stable = state;
		}
	}

	printk(KERN_DEBUG "%s: irq %d, state %d dir %d\n", __func__, irq, state, encoder->dir);
	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t rotary_encoder_quarter_period_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	unsigned int state;

	mutex_lock(&encoder->access_mutex);

	state = rotary_encoder_get_state(encoder);

	if ((encoder->last_stable + 1) % 4 == state)
		encoder->dir = 1;
	else if (encoder->last_stable == (state + 1) % 4)
		encoder->dir = -1;
	else
		goto out;

	rotary_encoder_report_event(encoder);

out:
	encoder->last_stable = state;
	printk(KERN_DEBUG "%s: irq %d, state %d dir %d\n", __func__, irq, state, encoder->dir);
	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t rotary_push_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	int val;

	mutex_lock(&encoder->access_mutex);

	val = gpiod_get_value_cansleep(encoder->push_gpio);

	input_report_key(encoder->input, encoder->push_code, val);
	input_sync(encoder->input);

	printk(KERN_DEBUG "%s: irq %d, val %d\n", __func__, irq, val);
	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static int rotary_encoder_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rotary_encoder *encoder;
	struct input_dev *input;
	irq_handler_t handler;
	u32 steps_per_period;
	unsigned int i;
	int err;

	encoder = devm_kzalloc(dev, sizeof(struct rotary_encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	mutex_init(&encoder->access_mutex);

	device_property_read_u32(dev, "rotary-encoder,steps", &encoder->steps);

	err = device_property_read_u32(dev, "rotary-encoder,steps-per-period",
				       &steps_per_period);
	if (err) {
		/*
		 * The 'half-period' property has been deprecated, you must
		 * use 'steps-per-period' and set an appropriate value, but
		 * we still need to parse it to maintain compatibility. If
		 * neither property is present we fall back to the one step
		 * per period behavior.
		 */
		steps_per_period = device_property_read_bool(dev,
					"rotary-encoder,half-period") ? 2 : 1;
	}

	encoder->rollover =
		device_property_read_bool(dev, "rotary-encoder,rollover");

	if (!device_property_present(dev, "rotary-encoder,encoding") ||
	    !device_property_match_string(dev, "rotary-encoder,encoding",
					  "gray")) {
		dev_info(dev, "gray");
		encoder->encoding = ROTENC_GRAY;
	} else if (!device_property_match_string(dev, "rotary-encoder,encoding",
						 "binary")) {
		dev_info(dev, "binary");
		encoder->encoding = ROTENC_BINARY;
	} else {
		dev_err(dev, "unknown encoding setting\n");
		return -EINVAL;
	}

	device_property_read_u32(dev, "linux,axis", &encoder->axis);
	encoder->relative_axis =
		device_property_read_bool(dev, "rotary-encoder,relative-axis");

	encoder->gpios = devm_gpiod_get_array(dev, NULL, GPIOD_IN);
	if (IS_ERR(encoder->gpios)) {
		err = PTR_ERR(encoder->gpios);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "unable to get gpios: %d\n", err);
		return err;
	}
	if (encoder->gpios->ndescs < 2) {
		dev_err(dev, "not enough gpios found\n");
		return -EINVAL;
	}

	/* push-gpios setup */
	encoder->push_gpio = devm_gpiod_get_optional(dev, "push", GPIOD_IN);
	if (IS_ERR(encoder->push_gpio))
		return dev_err_probe(dev, PTR_ERR(encoder->push_gpio), "failed to get push-gpios\n");

	encoder->push_code = KEY_ENTER;
	device_property_read_u32(dev, "linux,push-code", &encoder->push_code);

	encoder->push_type = EV_KEY;
	device_property_read_u32(dev, "linux,push-type", &encoder->push_type);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	encoder->input = input;

	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->dev.parent = dev;

	if (encoder->relative_axis)
		input_set_capability(input, EV_REL, encoder->axis);
	else
		input_set_abs_params(input,
				     encoder->axis, 0, encoder->steps, 0, 1);

	switch (steps_per_period >> (encoder->gpios->ndescs - 2)) {
	case 4:
		handler = &rotary_encoder_quarter_period_irq;
		encoder->last_stable = rotary_encoder_get_state(encoder);
		break;
	case 2:
		handler = &rotary_encoder_half_period_irq;
		encoder->last_stable = rotary_encoder_get_state(encoder);
		break;
	case 1:
		handler = &rotary_encoder_irq;
		break;
	default:
		dev_err(dev, "'%d' is not a valid steps-per-period value\n",
			steps_per_period);
		return -EINVAL;
	}

	encoder->irq =
		devm_kcalloc(dev,
			     encoder->gpios->ndescs, sizeof(*encoder->irq),
			     GFP_KERNEL);
	if (!encoder->irq)
		return -ENOMEM;

	for (i = 0; i < encoder->gpios->ndescs; ++i) {
		encoder->irq[i] = gpiod_to_irq(encoder->gpios->desc[i]);
		dev_info(dev, "gpio %d to irq %d", desc_to_gpio(encoder->gpios->desc[i]), encoder->irq[i]);

		err = devm_request_threaded_irq(dev, encoder->irq[i],
				NULL, handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				DRV_NAME, encoder);
		if (err) {
			dev_err(dev, "unable to request IRQ %d (gpio#%d)\n",
				encoder->irq[i], i);
			return err;
		}
	}

	if (encoder->push_gpio) {
		input_set_capability(encoder->input, encoder->push_type, encoder->push_code);

		encoder->push_irq = gpiod_to_irq(encoder->push_gpio);
		dev_info(dev, "push-gpio %d to irq %d\n", desc_to_gpio(encoder->push_gpio), encoder->push_irq);

		err = devm_request_threaded_irq(dev, encoder->push_irq,
				NULL, rotary_push_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				DRV_NAME, encoder);
		if (err)
			return dev_err_probe(dev, err, "unable to request push IRQ\n");
	}

	err = input_register_device(input);
	if (err) {
		dev_err(dev, "failed to register input device\n");
		return err;
	}

	device_init_wakeup(dev,
			   device_property_read_bool(dev, "wakeup-source"));

	platform_set_drvdata(pdev, encoder);

	printk(KERN_DEBUG "%s: irqs %d %d %d\n", __func__, encoder->irq[0], encoder->irq[1], encoder->push_irq);

	return 0;
}

static int __maybe_unused rotary_encoder_suspend(struct device *dev)
{
	struct rotary_encoder *encoder = dev_get_drvdata(dev);
	unsigned int i;

	printk(KERN_DEBUG "%s: \n", __func__);

	if (device_may_wakeup(dev)) {
		for (i = 0; i < encoder->gpios->ndescs; ++i)
			enable_irq_wake(encoder->irq[i]);
	}

	return 0;
}

static int __maybe_unused rotary_encoder_resume(struct device *dev)
{
	struct rotary_encoder *encoder = dev_get_drvdata(dev);
	unsigned int i;

	printk(KERN_DEBUG "%s: \n", __func__);

	if (device_may_wakeup(dev)) {
		for (i = 0; i < encoder->gpios->ndescs; ++i)
			disable_irq_wake(encoder->irq[i]);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rotary_encoder_pm_ops,
			 rotary_encoder_suspend, rotary_encoder_resume);

#ifdef CONFIG_OF
static const struct of_device_id rotary_encoder_of_match[] = {
	{ .compatible = "rotary-encoder", },
	{ },
};
MODULE_DEVICE_TABLE(of, rotary_encoder_of_match);
#endif

static struct platform_driver rotary_encoder_driver = {
	.probe		= rotary_encoder_probe,
	.driver		= {
		.name	= DRV_NAME,
		.pm	= &rotary_encoder_pm_ops,
		.of_match_table = of_match_ptr(rotary_encoder_of_match),
	}
};
module_platform_driver(rotary_encoder_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("GPIO rotary encoder driver");
MODULE_AUTHOR("Daniel Mack <daniel@caiaq.de>, Johan Hovold");
MODULE_LICENSE("GPL v2");
