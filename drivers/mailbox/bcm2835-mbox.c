/*
 *  Copyright (C) 2010 Broadcom
 *  Copyright (C) 2013-2014 Lubomir Rintel
 *  Copyright (C) 2013 Craig McGeachie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a mechanism for writing to the mailboxes,
 * that are shared between the ARM and the VideoCore processor
 *
 * Parts of the driver are based on:
 *  - arch/arm/mach-bcm2708/vcio.c file written by Gray Girling that was
 *    obtained from branch "rpi-3.6.y" of git://github.com/raspberrypi/
 *    linux.git
 *  - drivers/mailbox/bcm2835-ipc.c by Lubomir Rintel at
 *    https://github.com/hackerspace/rpi-linux/blob/lr-raspberry-pi/drivers/
 *    mailbox/bcm2835-ipc.c
 *  - documentation available on the following web site:
 *    https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/err.h>
#include <linux/spinlock.h>

/* Mailboxes */
#define ARM_0_MAIL0	0x00
#define ARM_0_MAIL1	0x20

/* Mailbox registers. We basically only support mailbox 0 & 1. We deliver to
 * the VC in mailbox 1, it delivers to us in mailbox 0. See BCM2835 ARM
 * Peripherals section 1.3 for an explanation about the placement of memory
 * barriers. */
#define MAIL0_RD	(ARM_0_MAIL0 + 0x00)
#define MAIL0_POL	(ARM_0_MAIL0 + 0x10)
#define MAIL0_STA	(ARM_0_MAIL0 + 0x18)
#define MAIL0_CNF	(ARM_0_MAIL0 + 0x1C)
#define MAIL1_WRT	(ARM_0_MAIL1 + 0x00)
#define MAIL1_STA	(ARM_0_MAIL1 + 0x18)

#define MBOX_CHAN_COUNT		16

/* Status register: FIFO state. */
#define ARM_MS_FULL		0x80000000
#define ARM_MS_EMPTY		0x40000000

/* Configuration register: Enable interrupts. */
#define ARM_MC_IHAVEDATAIRQEN	0x00000001

#define MBOX_MSG(chan, data28)		(((data28) & ~0xf) | ((chan) & 0xf))
#define MBOX_CHAN(msg)			((msg) & 0xf)
#define MBOX_DATA28(msg)		((msg) & ~0xf)

struct bcm2835_mbox;

struct bcm2835_channel {
	struct bcm2835_mbox *mbox;
	struct mbox_chan *link;
	u32 chan_num;
	bool started;
};

struct bcm2835_mbox {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *regs;
	spinlock_t lock;
	struct bcm2835_channel channel[MBOX_CHAN_COUNT];
	struct mbox_controller controller;
};

#define to_channel(link) ((struct bcm2835_channel *)link->con_priv)

static irqreturn_t bcm2835_mbox_irq(int irq, void *dev_id)
{
	struct bcm2835_mbox *mbox = (struct bcm2835_mbox *) dev_id;
	struct device *dev = mbox->dev;
	while (!(readl(mbox->regs + MAIL0_STA) & ARM_MS_EMPTY)) {
		u32 msg = readl(mbox->regs + MAIL0_RD);
		unsigned int chan = MBOX_CHAN(msg);

		if (!mbox->channel[chan].started) {
			dev_err(dev, "Reply on stopped channel %d\n", chan);
			continue;
		}
		dev_dbg(dev, "Reply 0x%08X\n", msg);
		mbox_chan_received_data(mbox->channel[chan].link,
			(void *) MBOX_DATA28(msg));
	}
	rmb(); /* Finished last mailbox read. */
	return IRQ_HANDLED;
}

static int bcm2835_send_data(struct mbox_chan *link, void *data)
{
	struct bcm2835_channel *chan = to_channel(link);
	struct bcm2835_mbox *mbox = chan->mbox;
	int ret = 0;

	if (!chan->started)
		return -ENODEV;
	spin_lock(&mbox->lock);
	if (readl(mbox->regs + MAIL0_STA) & ARM_MS_FULL) {
		rmb(); /* Finished last mailbox read. */
		ret = -EBUSY;
		goto end;
	}
	wmb(); /* About to write to the mail box. */
	writel(MBOX_MSG(chan->chan_num, (u32) data), mbox->regs + MAIL1_WRT);
	dev_dbg(mbox->dev, "Request 0x%08X\n", MBOX_MSG(chan->chan_num,
		(u32) data));
end:
	spin_unlock(&mbox->lock);
	return ret;
}

static int bcm2835_startup(struct mbox_chan *link)
{
	struct bcm2835_channel *chan = to_channel(link);
	chan->started = true;
	return 0;
}

static void bcm2835_shutdown(struct mbox_chan *link)
{
	struct bcm2835_channel *chan = to_channel(link);
	chan->started = false;
}

static bool bcm2835_last_tx_done(struct mbox_chan *link)
{
	struct bcm2835_channel *chan = to_channel(link);
	struct bcm2835_mbox *mbox = chan->mbox;
	bool ret;

	if (!chan->started)
		return false;
	spin_lock(&mbox->lock);
	ret = !!(readl(mbox->regs + MAIL1_STA) & ARM_MS_EMPTY);
	rmb();
	spin_unlock(&mbox->lock);
	return ret;
}

static struct mbox_chan_ops bcm2835_mbox_chan_ops = {
	.send_data	= bcm2835_send_data,
	.startup	= bcm2835_startup,
	.shutdown	= bcm2835_shutdown,
	.last_tx_done	= bcm2835_last_tx_done
};

static int request_mailbox_iomem(struct bcm2835_mbox *mbox)
{
	struct platform_device *pdev = mbox->pdev;
	struct resource *iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_dbg(&pdev->dev, "iomem 0x%08X-0x%08X\n", iomem->start, iomem->end);
	mbox->regs = devm_ioremap_resource(&pdev->dev, iomem);
	dev_dbg(&pdev->dev, "registers at [0x%p]\n", mbox->regs);
	if (IS_ERR(mbox->regs)) {
		dev_err(&pdev->dev, "Failed to remap mailbox regs\n");
		return PTR_ERR(mbox->regs);
	}
	dev_dbg(&pdev->dev, "iomem registered\n");
	return 0;
}

static int request_mailbox_irq(struct bcm2835_mbox *mbox)
{
	int ret;
	struct device *dev = mbox->dev;
	struct device_node *np = dev->of_node;
	int irq = irq_of_parse_and_map(np, 0);

	if (irq <= 0) {
		dev_err(dev, "Can't get IRQ number for mailbox\n");
		return -ENODEV;
	}
	ret = devm_request_irq(dev, irq, bcm2835_mbox_irq, 0, dev_name(dev),
		mbox);
	if (ret) {
		dev_err(dev, "Failed to register a mailbox IRQ handler\n");
		return -ENODEV;
	}
	dev_dbg(dev, "Registered IRQ\n");
	return 0;
}

static int bcm2835_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_mbox *mbox;
	int i;
	int ret = 0;

	dev_dbg(dev, "Probing\n");
	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (mbox == NULL) {
		dev_err(dev, "Failed to allocate mailbox memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, mbox);
	mbox->pdev = pdev;
	mbox->dev = dev;
	spin_lock_init(&mbox->lock);

	dev_dbg(dev, "Requesting IRQ\n");
	ret = request_mailbox_irq(mbox);
	if (ret)
		return ret;
	dev_dbg(dev, "Requesting iomem\n");
	ret = request_mailbox_iomem(mbox);
	if (ret)
		return ret;

	dev_dbg(dev, "Initializing mailbox controller\n");
	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = 5;
	mbox->controller.ops = &bcm2835_mbox_chan_ops;
	mbox->controller.dev = dev;
	mbox->controller.num_chans = MBOX_CHAN_COUNT;
	mbox->controller.chans = devm_kzalloc(dev,
		sizeof(struct mbox_chan) * MBOX_CHAN_COUNT,
		GFP_KERNEL);
	if (!mbox->controller.chans) {
		dev_err(dev, "Failed to alloc mbox_chans\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "Initializing mailbox channels\n");
	for (i = 0; i != MBOX_CHAN_COUNT; ++i) {
		mbox->channel[i].mbox = mbox;
		mbox->channel[i].link = &mbox->controller.chans[i];
		mbox->channel[i].chan_num = i;
		mbox->controller.chans[i].con_priv =
			(void *)&mbox->channel[i];
	}

	ret  = mbox_controller_register(&mbox->controller);
	if (ret)
		return ret;

	/* Enable the interrupt on data reception */
	writel(ARM_MC_IHAVEDATAIRQEN, mbox->regs + MAIL0_CNF);
	dev_info(dev, "mailbox enabled\n");

	return ret;
}

static int bcm2835_mbox_remove(struct platform_device *pdev)
{
	struct bcm2835_mbox *mbox = platform_get_drvdata(pdev);
	mbox_controller_unregister(&mbox->controller);
	return 0;
}

static const struct of_device_id bcm2835_mbox_of_match[] = {
	{ .compatible = "brcm,bcm2835-mbox", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_mbox_of_match);

static struct platform_driver bcm2835_mbox_driver = {
	.driver = {
		.name = "bcm2835-mbox",
		.owner = THIS_MODULE,
		.of_match_table = bcm2835_mbox_of_match,
	},
	.probe		= bcm2835_mbox_probe,
	.remove		= bcm2835_mbox_remove,
};
module_platform_driver(bcm2835_mbox_driver);

MODULE_AUTHOR("Craig McGeachie");
MODULE_DESCRIPTION("BCM2835 mailbox IPC driver");
MODULE_LICENSE("GPL v2");
