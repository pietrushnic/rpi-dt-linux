/*
 * This driver dynamically manages the CPU Frequency of the ARM processor.
 * Messages are sent to Videocore either setting or requesting the
 * frequency of the ARM in order to match an appropiate frequency to the
 * current usage of the processor. The policy which selects the frequency
 * to use is defined in the kernel .config file, but can be changed during
 * runtime.
 *
 * Copyright 2011 Broadcom Corporation.  All rights reserved.
 * Copyright 2013 Lubomir Rintel
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>

#define VCMSG_SET_CLOCK_RATE	0x00038002
#define VCMSG_GET_CLOCK_RATE	0x00030002
#define VCMSG_GET_MIN_CLOCK	0x00030007
#define VCMSG_GET_MAX_CLOCK	0x00030004
#define VCMSG_ID_ARM_CLOCK	0x00000003	/* Clock/Voltage ID's */

struct mbox_client bcm2835_property_client;
struct mbox_chan *bcm2835_property_chan;

/* tag part of the message */
struct vc_msg_tag {
	u32 tag_id;		/* the message id */
	u32 buffer_size;	/* size of the buffer (8 bytes) */
	u32 data_size;		/* amount of data being sent or received */
	u32 dev_id;		/* the ID of the clock/voltage to get or set */
	u32 val;		/* the value (e.g. rate (in Hz)) to set */
} __packed;

/* message structure to be sent to videocore */
struct vc_msg {
	u32 msg_size;		/* simply, sizeof(struct vc_msg) */
	u32 request_code;	/* holds various information like thei
				 * success and number of bytes returned */
	struct vc_msg_tag tag;	/* the tag structure above to make */
	u32 end_tag;		/* an end identifier, should be set to NULL */
} __packed;

/* clk_rate either gets or sets the clock rates.  */
static u32 bcm2835_cpufreq_set_clock(int cur_rate, int arm_rate)
{
	int ret = 0;
	dma_addr_t msg_bus;
	struct vc_msg *msg;

	msg = dma_alloc_coherent(NULL, PAGE_ALIGN(sizeof(*msg)), &msg_bus,
							GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	/* wipe all previous message data */
	memset(msg, 0, sizeof(*msg));

	msg->msg_size = sizeof(*msg);

	msg->tag.tag_id = VCMSG_SET_CLOCK_RATE;
	msg->tag.buffer_size = 8;
	msg->tag.data_size = 8;	/* we're sending the clock ID and the
				 * knew rates which is a total of 2 words */
	msg->tag.dev_id = VCMSG_ID_ARM_CLOCK;
	msg->tag.val = arm_rate * 1000;

	/* send the message */
	ret = mbox_send_message(bcm2835_property_chan, (void *)msg_bus);

	/* check if it was all ok and return the rate in KHz */
	if (ret == 0 && (msg->request_code & 0x80000000))
		ret = msg->tag.val/1000;

	dma_free_coherent(NULL, PAGE_ALIGN(sizeof(*msg)), msg, msg_bus);
	return ret;
}

static u32 bcm2835_cpufreq_get_clock(int tag)
{
	int ret;
	int arm_rate = 0;
	dma_addr_t msg_bus;
	struct vc_msg *msg;

	msg = dma_alloc_coherent(NULL, PAGE_ALIGN(sizeof(*msg)), &msg_bus,
							GFP_KERNEL);
	if (!msg) {
		pr_err("Out of DMA memory\n");
		return -ENOMEM;
	}

	/* wipe all previous message data */
	memset(msg, 0, sizeof(*msg));

	msg->msg_size = sizeof(*msg);
	msg->tag.tag_id = tag;
	msg->tag.buffer_size = 8;
	msg->tag.data_size = 4;	/* we're just sending the clock ID which
				 * is one word long */
	msg->tag.dev_id = VCMSG_ID_ARM_CLOCK;

	/* send the message */
	ret = mbox_send_message(bcm2835_property_chan, (void *)msg_bus);

	/* check if it was all ok and return the rate in KHz */
	if (ret >= 0 && (msg->request_code & 0x80000000))
		arm_rate = msg->tag.val/1000;

	dma_free_coherent(NULL, PAGE_ALIGN(sizeof(*msg)), msg, msg_bus);
	return arm_rate;
}

/* Initialisation function sets up the CPU policy for first use */
static int bcm2835_cpufreq_init(struct cpufreq_policy *policy)
{
	/* measured value of how long it takes to change frequency */
	policy->cpuinfo.transition_latency = 355000; /* ns */

	/* now find out what the maximum and minimum frequencies are */
	policy->min = bcm2835_cpufreq_get_clock(VCMSG_GET_MIN_CLOCK);
	policy->max = bcm2835_cpufreq_get_clock(VCMSG_GET_MAX_CLOCK);
	policy->cur = bcm2835_cpufreq_get_clock(VCMSG_GET_CLOCK_RATE);

	policy->cpuinfo.min_freq = policy->min;
	policy->cpuinfo.max_freq = policy->max;

	return 0;
}

/* Target function chooses the most appropriate frequency from the table
 * to enable */
static int bcm2835_cpufreq_target(struct cpufreq_policy *policy,
						unsigned int target_freq,
						unsigned int relation)
{
	unsigned int target = target_freq;
	u32 cur;

	/* if we are above min and using ondemand, then just use max */
	if (strcmp("ondemand", policy->governor->name) == 0 &&
					target > policy->min)
		target = policy->max;

	/* if the frequency is the same, just quit */
	if (target == policy->cur)
		return 0;

	/* otherwise were good to set the clock frequency */
	policy->cur = bcm2835_cpufreq_set_clock(policy->cur, target);

	cur = bcm2835_cpufreq_set_clock(policy->cur, target);
	if (!cur)
		return -EINVAL;

	policy->cur = cur;
	return 0;
}

static unsigned int bcm2835_cpufreq_get(unsigned int cpu)
{
	return bcm2835_cpufreq_get_clock(VCMSG_GET_CLOCK_RATE);
}

/* Verify ensures that when a policy is changed, it is suitable for the
 * CPU to use */
static int bcm2835_cpufreq_verify(struct cpufreq_policy *policy)
{
	return 0;
}

/* the CPUFreq driver */
static struct cpufreq_driver bcm2835_cpufreq = {
	.name = "bcm2835-cpufreq",
	.init = bcm2835_cpufreq_init,
	.verify = bcm2835_cpufreq_verify,
	.target = bcm2835_cpufreq_target,
	.get = bcm2835_cpufreq_get
};

static int bcm2835_cpufreq_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	bcm2835_property_client.tx_block = true;
	bcm2835_property_client.dev = dev;

	bcm2835_property_chan =
		mbox_request_channel(&bcm2835_property_client, 0);
	if (IS_ERR(bcm2835_property_chan)) {
		dev_err(dev, "Could not get the property mailbox channel\n");
		return PTR_ERR(bcm2835_property_chan);
	}

	ret = cpufreq_register_driver(&bcm2835_cpufreq);
	if (ret) {
		mbox_free_channel(bcm2835_property_chan);
		dev_err(dev, "Could not register cpufreq driver\n");
		return ret;
	}

	dev_info(dev, "Broadcom BCM2835 CPU frequency control driver\n");
	return 0;
}

static int bcm2835_cpufreq_remove(struct platform_device *dev)
{
	mbox_free_channel(bcm2835_property_chan);
	cpufreq_unregister_driver(&bcm2835_cpufreq);
	return 0;
}

static const struct of_device_id bcm2835_cpufreq_of_match[] = {
	{ .compatible = "brcm,bcm2835-cpufreq", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_cpufreq_of_match);

static struct platform_driver bcm2835_cpufreq_driver = {
	.probe = bcm2835_cpufreq_probe,
	.remove = bcm2835_cpufreq_remove,
	.driver = {
		.name = "bcm2835-cpufreq",
		.owner = THIS_MODULE,
		.of_match_table = bcm2835_cpufreq_of_match,
	},
};
module_platform_driver(bcm2835_cpufreq_driver);

MODULE_AUTHOR("Dorian Peake and Dom Cobley and Lubomir Rintel");
MODULE_DESCRIPTION("BCM2835 CPU frequency control driver");
MODULE_LICENSE("GPL v2");
