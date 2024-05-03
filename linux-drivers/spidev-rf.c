// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SPI-RF driver
 *
 * Copyright (C) 2024 Yunjae Lim <launius@gmail.com>
 *
 * Description:
 * This Misc Device Driver is developed for SPI RF devices.
 * It implements a double Ring Buffer for processing RF RX data.
 * The driver exposes userspace interfaces to support file operations
 * such as open(), close(), read(), write() and poll().
 *
 */

/*
 * Userspace application pseudo code:
 *
#define DEV_FILE1   "/dev/spidev-rf0"
#define DEV_FILE2   "/dev/spidev-rf1"

class SPIRFRunThread
{
private:
    int bufFd[2]{};
    struct pollfd pfds[2];

public:
	void run()
	{
		bufFd[0] = open(DEV_FILE1, O_RDWR);
		bufFd[1] = open(DEV_FILE2, O_RDWR);

		pfds[0].fd = bufFd[0];
		pfds[0].events = POLLERR | POLLPRI;
		pfds[1].fd = bufFd[1];
		pfds[1].events = POLLERR | POLLPRI;
		
		while(1) {
			ssize_t bytesRead, bytesWritten;
			int ret = poll(pfds, 2, 1000);		//wait for 1sec
			if (ret > 0) {
				if (pfds[0].revents & (POLLERR | POLLPRI)) {
					bytesRead = pread(bufFd[0], rx_packet, RX_PACKET_SIZE, 0);
					bytesWritten = pwrite(bufFd[0], tx_packet, TX_PACKET_SIZE, 0);
				}

				if (pfds[0].revents & (POLLERR | POLLPRI)) {
					bytesRead = pread(bufFd[1], rx_packet, RX_PACKET_SIZE, 0);
					bytesWritten = pwrite(bufFd[1], tx_packet, TX_PACKET_SIZE, 0);
				}
			}
		}

		close(bufFd[0]);
		close(bufFd[1]);
	}
}
 *
 */

#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

//#define SUPPORT_SYSFS_RW
#ifdef SUPPORT_SYSFS_RW
#include <linux/sysfs.h>
#endif

//#define WAIT_IN_READ

#define SPI_MAX_BUF_SIZE		128
#define RADIO_PACKET_SIZE		100
#define RADIO_RX_PACKET_SIZE	105
#define RADIO_TX_PACKET_SIZE	105

#define PACKET_BUF_MAX			10
#define PACKET_READ_COUNT		2

struct spidev_rf_priv {
	unsigned char			tx_buff[SPI_MAX_BUF_SIZE];
	unsigned char			init_buff[SPI_MAX_BUF_SIZE];
	unsigned char			rx_buff_1[SPI_MAX_BUF_SIZE * PACKET_BUF_MAX];
	unsigned char			rx_buff_2[SPI_MAX_BUF_SIZE * PACKET_BUF_MAX];
	struct spi_device 		*spi;
	struct gpio_desc		*gpiod_radio_reset;
	struct gpio_desc		*gpiod_radio_sync;
	unsigned long			irq_flags;
	int    					sync_irq;
	int						gpio_state;
	int						buf_updated;
	int						packet_cnt[2];
	int						packet_read[2];
	int						packet_write[2];
	struct miscdevice		miscdev[2];
	wait_queue_head_t		waitq;
	spinlock_t				data_lock;
	unsigned long			lock_flags;
	unsigned char			rx_buf[SPI_MAX_BUF_SIZE * PACKET_READ_COUNT];
};

#ifdef SUPPORT_SYSFS_RW
static ssize_t spidev_rf_init_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct spidev_rf_priv *priv = dev_get_drvdata(dev);

	ssize_t ret = sprintf(buf, "%d\n", priv->init_buff[1]);

	return ret;
}

static ssize_t spidev_rf_init_store(struct device *dev, 
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct spidev_rf_priv *priv = dev_get_drvdata(dev);

	memset(priv->init_buff, 0x00, SPI_MAX_BUF_SIZE);
	memcpy(priv->init_buff, buf, count < RADIO_TX_PACKET_SIZE ? count : RADIO_TX_PACKET_SIZE);

	gpiod_direction_output(priv->gpiod_radio_reset, 1);
	msleep(5);

	gpiod_direction_output(priv->gpiod_radio_reset, 0);
	msleep(10);

	return count;
}
static DEVICE_ATTR_RW(spidev_rf_init);

static struct attribute *spidev_rf_attribute[] = {
	&dev_attr_spidev_rf_init.attr,
	NULL
};

static const struct attribute_group radio_attribute_gr = {
	.name = "spidev-rf",
	.attrs = spidev_rf_attribute
};
#endif

int radio_spi_tansceive(struct spidev_rf_priv *priv, int state)
{
	struct spi_message msg;
	int status;

	struct spi_transfer xfer = {
		.tx_buf		= priv->tx_buff,
		.rx_buf		= priv->rx_buff_1 + priv->packet_write[0],
		.len		= RADIO_PACKET_SIZE,
		.bits_per_word = 16,
	};

	if (state == 1)
		xfer.rx_buf = priv->rx_buff_2 + priv->packet_write[1];

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	status = spi_nothread_sync(priv->spi, &msg);
	
	return status;
}

static irqreturn_t radio_gpio_irqhandler(int irq, void *data)
{
	struct spidev_rf_priv *priv = (struct spidev_rf_priv *)data;
	int state;

	if(priv->gpiod_radio_sync)
	{
		state = gpiod_get_value(priv->gpiod_radio_sync);

		//TODO: consider spinlock
		radio_spi_tansceive(priv, state);
		
		if(state == 0)
		{
#ifdef SUPPORT_SYSFS_RW
			sysfs_notify(&priv->spi->dev.kobj, radio_attribute_gr.name, spidev_rf_attribute[1]->name);
#endif

			spin_lock_irqsave(&priv->data_lock, priv->lock_flags);
			priv->gpio_state = state;
			priv->buf_updated = 1;
			priv->packet_cnt[0]++;

//			if (priv->rx_buff_1[1])
			printk(KERN_INFO "%s: channel: %d packets %d %d, %*ph\n", __func__,
				priv->gpio_state, priv->packet_cnt[0], priv->packet_cnt[1], 20, priv->rx_buff_1 + 80);
			spin_unlock_irqrestore(&priv->data_lock, priv->lock_flags);

			if (priv->packet_cnt[1] % PACKET_READ_COUNT == 0)
				wake_up_interruptible(&priv->waitq);

			if (priv->packet_write[0] < SPI_MAX_BUF_SIZE * (PACKET_BUF_MAX - 1))
				priv->packet_write[0] += SPI_MAX_BUF_SIZE;
			else
				priv->packet_write[0] = 0;
		}
		else
		{
#ifdef SUPPORT_SYSFS_RW
			sysfs_notify(&priv->spi->dev.kobj, radio_attribute_gr.name, spidev_rf_attribute[2]->name);
#endif

			spin_lock_irqsave(&priv->data_lock, priv->lock_flags);
			priv->gpio_state = state;
			priv->buf_updated = 2;
			priv->packet_cnt[1]++;

//			if (priv->rx_buff_2[1])
			printk(KERN_INFO "%s: channel: %d packets %d %d, %*ph\n", __func__,
				priv->gpio_state, priv->packet_cnt[0], priv->packet_cnt[1], 20, priv->rx_buff_2 + 80);
			spin_unlock_irqrestore(&priv->data_lock, priv->lock_flags);

			if (priv->packet_cnt[1] % PACKET_READ_COUNT == 0)
				wake_up_interruptible(&priv->waitq);

			if (priv->packet_write[1] < SPI_MAX_BUF_SIZE * (PACKET_BUF_MAX - 1))
				priv->packet_write[1] += SPI_MAX_BUF_SIZE;
			else
				priv->packet_write[1] = 0;
		}
	}

	return IRQ_HANDLED;
}

static int spidev_rf_open1(struct inode *inode, struct file *filp)
{
	struct spidev_rf_priv *priv = NULL;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[0]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return -EFAULT;
	}

	//TODO: dynamic buffer mem allocation
	//	rxbuf = kzalloc(SPI_MAX_BUF_SIZE, GFP_KERNEL);

	spin_lock_irqsave(&priv->data_lock, priv->lock_flags);
	priv->buf_updated = 0;
	priv->packet_cnt[0] = 0;
	priv->packet_read[0] = priv->packet_write[0];

	printk(KERN_DEBUG "%s: minor %d packets %d %d\n", __func__, MINOR(filp->f_path.dentry->d_inode->i_rdev),
		 priv->packet_cnt[0], priv->packet_cnt[1]);
	spin_unlock_irqrestore(&priv->data_lock, priv->lock_flags);

	return 0;
}

static int spidev_rf_release1(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, MINOR(filp->f_path.dentry->d_inode->i_rdev));

//	kfree(rxbuf);

	return 0;
}

static ssize_t
spidev_rf_write1(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_rf_priv *priv = NULL;

	if (count > RADIO_TX_PACKET_SIZE)
		count = RADIO_TX_PACKET_SIZE;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[0]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return -EFAULT;
	}

	memset(priv->init_buff, 0x00, SPI_MAX_BUF_SIZE);
	if (copy_from_user(priv->init_buff, buf, count))
		return -EFAULT;

	gpiod_direction_output(priv->gpiod_radio_reset, 1);
	msleep(5);

	gpiod_direction_output(priv->gpiod_radio_reset, 0);
	msleep(10);

	// printk(KERN_INFO "%s: minor %d buf %d %d count %d\n", __func__, MINOR(filp->f_path.dentry->d_inode->i_rdev),
	// 	 priv->init_buff[0], priv->init_buff[1], count);
	return count;
}

static ssize_t
spidev_rf_read1(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_rf_priv *priv = NULL;
	ssize_t status;
	unsigned long missing;
	int i = 0;
	int offset, cnt;
	char *rxbuf = NULL;

	if (count > RADIO_RX_PACKET_SIZE * PACKET_READ_COUNT)
		count = RADIO_RX_PACKET_SIZE * PACKET_READ_COUNT;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[0]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return -EFAULT;
	}

	rxbuf = priv->rx_buf;
	memset(rxbuf, 0, SPI_MAX_BUF_SIZE * PACKET_READ_COUNT);

#ifdef WAIT_IN_READ
	wait_event_interruptible(priv->waitq, priv->buf_updated != 0);
#endif

	cnt = 0;
	while (cnt < PACKET_READ_COUNT) {
		offset = priv->packet_read[0];

		memcpy(rxbuf + cnt*RADIO_RX_PACKET_SIZE, priv->rx_buff_1 + offset, RADIO_RX_PACKET_SIZE);
		for (i = 0; i < 64; i++) {
			uint16_t fifoEntry = swab16(((uint16_t *)(priv->rx_buff_1+offset+(i*2)))[0]);
			((uint16_t *)(rxbuf+cnt*RADIO_RX_PACKET_SIZE+(i*2)))[0] = fifoEntry;
		}
		cnt++;

		if (priv->packet_read[0] < SPI_MAX_BUF_SIZE * (PACKET_BUF_MAX - 1))
			priv->packet_read[0] += SPI_MAX_BUF_SIZE;
		else
			priv->packet_read[0] = 0;
	}

	missing = copy_to_user(buf, rxbuf, count);
	if (missing == count)
		status = -EFAULT;
	else
		status = count - missing;

	spin_lock_irqsave(&priv->data_lock, priv->lock_flags);
	priv->buf_updated = 0;
	priv->packet_cnt[0] -= PACKET_READ_COUNT;

//	if (rxbuf[1])
	printk(KERN_INFO "%s: minor %d-%d packets %d %d, %*ph\n", __func__,
		MINOR(filp->f_path.dentry->d_inode->i_rdev), priv->gpio_state,
		priv->packet_cnt[0], priv->packet_cnt[1], 20, rxbuf + 80);
	spin_unlock_irqrestore(&priv->data_lock, priv->lock_flags);

	return status;
}

static __poll_t spidev_rf_poll1(struct file *filp, poll_table *wait)
{
	__poll_t mask = 0;
	struct spidev_rf_priv *priv;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[0]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return EPOLLNVAL | EPOLLHUP;
	}

	// printk(KERN_INFO "%s: minor %d %d packets %d, waiting..\n", __func__,
	// 	MINOR(filp->f_path.dentry->d_inode->i_rdev), priv->gpio_state, priv->packet_cnt[0]);

	poll_wait(filp, &priv->waitq, wait);
	if (priv->buf_updated == 1)
		mask |= EPOLLIN | EPOLLPRI;

	return mask;
}

static int spidev_rf_open2(struct inode *inode, struct file *filp)
{
	struct spidev_rf_priv *priv = NULL;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[1]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return -EFAULT;
	}

	spin_lock_irqsave(&priv->data_lock, priv->lock_flags);
	priv->buf_updated = 0;
	priv->packet_cnt[1] = 0;
	priv->packet_read[1] = priv->packet_write[1];

	printk(KERN_DEBUG "%s: minor %d packets %d %d\n", __func__, MINOR(filp->f_path.dentry->d_inode->i_rdev),
		 priv->packet_cnt[0], priv->packet_cnt[1]);
	spin_unlock_irqrestore(&priv->data_lock, priv->lock_flags);

	return 0;
}

static int spidev_rf_release2(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, MINOR(filp->f_path.dentry->d_inode->i_rdev));
	return 0;
}

static ssize_t
spidev_rf_write2(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_rf_priv *priv = NULL;

	if (count > RADIO_TX_PACKET_SIZE)
		count = RADIO_TX_PACKET_SIZE;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[1]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return -EFAULT;
	}

	memset(priv->tx_buff, 0x00, SPI_MAX_BUF_SIZE);
	if (copy_from_user(priv->tx_buff, buf, count))
		return -EFAULT;

	// printk(KERN_INFO "%s: minor %d buf %d-%d count %d\n", __func__, MINOR(filp->f_path.dentry->d_inode->i_rdev),
	// 	 priv->tx_buff[0], priv->tx_buff[1], count);
	return count;
}

static ssize_t
spidev_rf_read2(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_rf_priv *priv = NULL;
	ssize_t status;
	unsigned long missing;
	int i = 0;
	int offset, cnt;
	char *rxbuf = NULL;

	if (count > RADIO_RX_PACKET_SIZE * PACKET_READ_COUNT)
		count = RADIO_RX_PACKET_SIZE * PACKET_READ_COUNT;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[1]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return -EFAULT;
	}

	rxbuf = priv->rx_buf;
	memset(rxbuf, 0, SPI_MAX_BUF_SIZE * PACKET_READ_COUNT);

#ifdef WAIT_IN_READ
	wait_event_interruptible(priv->waitq, priv->buf_updated != 0);
#endif

	for (cnt = 0 ; cnt < PACKET_READ_COUNT ; cnt++) {
		offset = priv->packet_read[1];

		memcpy(rxbuf + cnt*RADIO_RX_PACKET_SIZE, priv->rx_buff_2 + offset, RADIO_RX_PACKET_SIZE);
		for (i = 0; i < 64; i++) {
			uint16_t fifoEntry = swab16(((uint16_t *)(priv->rx_buff_2+offset+(i*2)))[0]);
			((uint16_t *)(rxbuf+cnt*RADIO_RX_PACKET_SIZE+(i*2)))[0] = fifoEntry;
		}

		if (priv->packet_read[1] < SPI_MAX_BUF_SIZE * (PACKET_BUF_MAX - 1))
			priv->packet_read[1] += SPI_MAX_BUF_SIZE;
		else
			priv->packet_read[1] = 0;
	}

	missing = copy_to_user(buf, rxbuf, count);
	if (missing == count)
		status = -EFAULT;
	else
		status = count - missing;

	spin_lock_irqsave(&priv->data_lock, priv->lock_flags);
	priv->buf_updated = 0;
	priv->packet_cnt[1] -= PACKET_READ_COUNT;

//	if (rxbuf[1])
	printk(KERN_INFO "%s: minor %d-%d packets %d %d, %*ph\n", __func__,
		MINOR(filp->f_path.dentry->d_inode->i_rdev), priv->gpio_state,
		priv->packet_cnt[0], priv->packet_cnt[1], 20, rxbuf + 80);
	spin_unlock_irqrestore(&priv->data_lock, priv->lock_flags);

	return status;
}

static __poll_t spidev_rf_poll2(struct file *filp, poll_table *wait)
{
	__poll_t mask = 0;
	struct spidev_rf_priv *priv;

	priv = container_of(filp->private_data, struct spidev_rf_priv, miscdev[1]);
	if (!priv) {
		printk(KERN_INFO "%s: spidev_rf_priv error!\n", __func__);
		return EPOLLNVAL | EPOLLHUP;
	}

	// printk(KERN_INFO "%s: minor %d %d packets %d, waiting..\n", __func__,
	// 	MINOR(filp->f_path.dentry->d_inode->i_rdev), priv->gpio_state, priv->packet_cnt[1]);

	poll_wait(filp, &priv->waitq, wait);
	if (priv->buf_updated == 2)
		mask |= EPOLLIN | EPOLLPRI;

	return mask;
}

static const struct file_operations spidev_rf_fops1 = {
	.owner = THIS_MODULE,
	.open = spidev_rf_open1,
	.release = spidev_rf_release1,
	.write = spidev_rf_write1,
	.read = spidev_rf_read1,
	.poll = spidev_rf_poll1,
};

static const struct file_operations spidev_rf_fops2 = {
	.owner = THIS_MODULE,
	.open = spidev_rf_open2,
	.release = spidev_rf_release2,
	.write = spidev_rf_write2,
	.read = spidev_rf_read2,
	.poll = spidev_rf_poll2,
};

static int spidev_rf_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct spidev_rf_priv *priv;
	const char *const name = dev_name(dev);
	int ret;

	spi->bits_per_word = 16;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "Failed to setup SPI dev RF\n");
		return ret;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi=spi;

	priv->gpiod_radio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_radio_reset)) {
		ret = PTR_ERR(priv->gpiod_radio_reset);
		dev_err(dev, "Failed to get gpiod_radio_reset: %d\n", ret);
		return ret;
	}

	priv->gpiod_radio_sync = devm_gpiod_get(dev, "sync", GPIOD_IN);
	if (IS_ERR(priv->gpiod_radio_sync)) {
		ret = PTR_ERR(priv->gpiod_radio_sync);
		dev_err(dev, "Failed to get gpiod_radio_sync: %d\n", ret);
		return ret;
	}

	gpiod_direction_output(priv->gpiod_radio_reset, 1);
	msleep(5);
	gpiod_direction_output(priv->gpiod_radio_reset, 0);
	msleep(10);

	priv->sync_irq = irq_of_parse_and_map(spi->dev.of_node, 0);
	if (!priv->sync_irq) {
		dev_err(dev, "Failed to get IRQ for sync\n");
		return -EACCES;
	}

	spi_set_drvdata(spi, priv);

#ifdef SUPPORT_SYSFS_RW
	ret = sysfs_create_group(&dev->kobj, &radio_attribute_gr);
	if (ret){
		dev_err(dev, "SYSFS registration failed\n");
		return ret;
	}
#endif

	memset(priv->tx_buff, 0x00, SPI_MAX_BUF_SIZE);

	spin_lock_init(&priv->data_lock);
	init_waitqueue_head(&priv->waitq);
	priv->buf_updated = 0;
	priv->packet_cnt[0] = priv->packet_cnt[1] = 0;
	priv->packet_read[0] = priv->packet_read[1] = 0;
	priv->packet_write[0] = priv->packet_write[1] = 0;

	priv->irq_flags = IRQ_TYPE_EDGE_BOTH | IRQF_ONESHOT;
	ret = devm_request_threaded_irq(dev, priv->sync_irq, NULL, radio_gpio_irqhandler, priv->irq_flags, name, priv);
	if (ret) {
		dev_err(dev, "IRQ handler registering failed (%d)\n", ret);
		return ret;
	}

	priv->miscdev[0].minor = MISC_DYNAMIC_MINOR;
	priv->miscdev[0].name = "spidev-rf0";
	priv->miscdev[0].fops = &spidev_rf_fops1;
	priv->miscdev[0].parent = dev;
	priv->miscdev[0].mode = S_IRUGO | S_IWUGO;
	ret = misc_register(&priv->miscdev[0]);
	if (ret) {
		dev_err(dev, "error:%d. Unable to register spidev-rf0", ret);
		return ret;
	}

	priv->miscdev[1].minor = MISC_DYNAMIC_MINOR;
	priv->miscdev[1].name = "spidev-rf1";
	priv->miscdev[1].fops = &spidev_rf_fops2;
	priv->miscdev[1].parent = dev;
	priv->miscdev[1].mode = S_IRUGO | S_IWUGO;
	ret = misc_register(&priv->miscdev[1]);
	if (ret) {
		dev_err(dev, "error:%d. Unable to register spidef-rf1", ret);
		return ret;
	}

	dev_info(dev, "SPI RF driver probed successfully. got minor %d %d",
		priv->miscdev[0].minor, priv->miscdev[1].minor);
	return 0;
}

static int spidev_rf_remove(struct spi_device *spi)
{
	struct spidev_rf_priv *priv = spi_get_drvdata(spi);

	misc_deregister(&priv->miscdev[0]);
	misc_deregister(&priv->miscdev[1]);

#ifdef SUPPORT_SYSFS_RW
	sysfs_remove_group(&spi->dev.kobj, &radio_attribute_gr);
#endif

	gpiod_direction_output(priv->gpiod_radio_reset, 1);
	return 0;
}

static const struct spi_device_id spidev_rf_id[] = {
	{ "spidev-rf", 0 },
	{ }
};

MODULE_DEVICE_TABLE(spi, spidev_rf_id);

#ifdef CONFIG_OF
static const struct of_device_id spidev_rf_of_match[] = {
	{ .compatible = "vendor,spidev-rf" },
	{ }
};
MODULE_DEVICE_TABLE(of, spidev_rf_of_match);
#endif

static struct spi_driver spidev_rf_driver = {
	.driver = {
		.name = "spidev-rf",
	},
	.probe = spidev_rf_probe,
	.remove = spidev_rf_remove,
	.id_table = spidev_rf_id,
};

module_spi_driver(spidev_rf_driver);

MODULE_AUTHOR("Yunjae Lim <launius@gmail.com>");
MODULE_DESCRIPTION("SPI-RF device driver");
MODULE_LICENSE("GPL");
