/*
 *  Copyright (C) 2013 Craig McGeachie
 *  Copyright (C) 2014 Lubomir Rintel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device presents the BCM2835 SoC temperature sensor as a thermal
 * device.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/dma-mapping.h>
#include <linux/mailbox_client.h>
#include <linux/mutex.h>

#define VC_TAG_GET_TEMP 0x00030006
#define VC_TAG_GET_MAX_TEMP 0x0003000A
#define VC_SUCCESS 0x80000000

struct prop_msg {
	u32 msg_size;
	u32 code;
};

struct prop_tag_hdr {
	u32 tag_id;
	u32 buf_size;
	u32 indicator;
};

struct prop_tmp_tag {
	struct prop_tag_hdr hdr;
	u32 id;
	u32 val;
};

static void *propmsg_init(void *buf, size_t size)
{
	u32 num_cells = size >> 2;
	if (num_cells < 3)
		return 0;
	memset(buf, 0, size);
	((u32 *)buf)[0] = num_cells - 1;
	((u32 *)buf)[num_cells - 1] = 2;
	return buf;
}

static void *propmsg_addtag(void *buf, size_t size, u32 tag_id)
{
	u32 num_cells = ((size + 3) & -4) >> 2;
	u32 last_cell = ((u32 *)buf)[0];
	u32 nxt_cell = ((u32 *)buf)[last_cell];
	u32 free_cells = last_cell - nxt_cell;
	if (free_cells < num_cells || num_cells <= 3) {
		return 0;
	} else {
		void *ret = &((u32 *)buf)[nxt_cell];
		((struct prop_tag_hdr *) ret)->tag_id = tag_id;
		((struct prop_tag_hdr *) ret)->buf_size = (num_cells - 3) << 2;
		((struct prop_tag_hdr *) ret)->indicator = (num_cells - 3) << 2;
		((u32 *)buf)[last_cell] = nxt_cell + num_cells;
		return ret;
	}
}

static void propmsg_finish(void *buf)
{
	u32 last_cell = ((u32 *)buf)[0];
	u32 nxt_cell = ((u32 *)buf)[last_cell];
	((u32 *)buf)[nxt_cell] = 0;
	((u32 *)buf)[0] = (nxt_cell + 1) << 2;
	((u32 *)buf)[1] = 0;
}

#if 0

static void *propmsg_firsttag(void *msg)
{
	return ((struct prop_msg *) msg) + 1;
}

static void *propmsg_nexttag(void *curr)
{
	u32 tag_size = ((struct prop_tag_hdr *)curr)->buf_size;
	void *next_tag = (char *)curr + sizeof(struct prop_tag_hdr) + tag_size;
	return ((struct prop_tag_hdr *) next_tag)->tag_id == 0 ? 0 : next_tag;
}

#endif

struct bcm2835_therm {
	struct device *dev;
	struct thermal_zone_device *thermal_dev;
	struct completion comp;
	struct mutex lock;
};

static void bcm2835_rx_callback(struct mbox_client *cl, void *mssg)
{
	struct bcm2835_therm *therm = dev_get_drvdata(cl->dev);
	complete(&therm->comp);
}

static int do_message(struct bcm2835_therm *therm, dma_addr_t msg_handle)
{
	struct device *dev = therm->dev;
	struct mbox_client therm_mbox;
	struct mbox_chan *channel;
	int ret = 0;

	memset(&therm_mbox, 0, sizeof(therm_mbox));
	therm_mbox.rx_callback = bcm2835_rx_callback;
	therm_mbox.dev = dev;
	mutex_lock(&therm->lock);
	reinit_completion(&therm->comp);
	channel = mbox_request_channel(&therm_mbox, 0);
	if (IS_ERR(channel)) {
		dev_err(dev, "No mbox channel\n");
		ret = PTR_ERR(channel);
		goto exit;
	}
	mbox_send_message(channel, (void *) msg_handle);
	wait_for_completion(&therm->comp);
	mbox_free_channel(channel);
exit:
	mutex_unlock(&therm->lock);
	return ret;
}

static int bcm2835_get_temp_common(struct thermal_zone_device *thermal_dev,
	unsigned long *temp, u32 temp_type)
{
	struct bcm2835_therm *therm = thermal_dev->devdata;
	struct device *dev = therm->dev;
	void *buf = 0, *msg;
	int msgsize = sizeof(struct prop_msg) + sizeof(struct prop_tmp_tag)
		+ sizeof(u32);
	dma_addr_t dma_handle;
	int alignment = max(16, dma_get_cache_alignment());
	struct prop_tmp_tag *tmp_tag;
	int ret = 0;

	buf = kzalloc(msgsize + alignment, GFP_KERNEL | GFP_DMA);
	if (!buf) {
		dev_err(dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto exit;
	}
	msg = PTR_ALIGN(buf, alignment);
	propmsg_init(msg, msgsize);
	tmp_tag = propmsg_addtag(msg, sizeof(struct prop_tmp_tag), temp_type);
	BUG_ON(!tmp_tag); /* msgsize should be large enough to ensure success */
	propmsg_finish(msg);
	dma_handle = dma_map_single(dev, msg, msgsize, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, dma_handle)) {
		dev_err(dev, "No DMA address\n");
		ret = -ENOMEM;
		goto exit;
	}
	print_hex_dump(KERN_DEBUG, "req - ", DUMP_PREFIX_ADDRESS, 16, 4, msg,
		msgsize, false);
	ret = do_message(therm, dma_handle);
	print_hex_dump(KERN_DEBUG, "rep - ", DUMP_PREFIX_ADDRESS, 16, 4, msg,
		msgsize, false);
	dma_unmap_single(dev, dma_handle, msgsize, DMA_BIDIRECTIONAL);
	if (ret)
		goto exit;
	if (!(tmp_tag->hdr.indicator & VC_SUCCESS)) {
		dev_err(dev, "VC request failed 0x%08X\n",
			tmp_tag->hdr.indicator);
		ret = -EAGAIN;
	} else {
		*temp = tmp_tag->val;
	}
exit:
	kfree(buf);
	return ret;
}

static int bcm2835_get_temp(struct thermal_zone_device *thermal_dev,
	unsigned long *temp)
{
	return bcm2835_get_temp_common(thermal_dev, temp, VC_TAG_GET_TEMP);
}

static int bcm2835_get_max_temp(struct thermal_zone_device *thermal_dev,
	int trip_num, unsigned long *temp)
{
	return bcm2835_get_temp_common(thermal_dev, temp, VC_TAG_GET_MAX_TEMP);
}

static int bcm2835_get_trip_type(struct thermal_zone_device *thermal_dev,
	int trip_num, enum thermal_trip_type *trip_type)
{
	*trip_type = THERMAL_TRIP_HOT;
	return 0;
}


static int bcm2835_get_mode(struct thermal_zone_device *thermal_dev,
	enum thermal_device_mode *dev_mode)
{
	*dev_mode = THERMAL_DEVICE_ENABLED;
	return 0;
}

static struct thermal_zone_device_ops ops  = {
	.get_temp = bcm2835_get_temp,
	.get_trip_temp = bcm2835_get_max_temp,
	.get_trip_type = bcm2835_get_trip_type,
	.get_mode = bcm2835_get_mode,
};

static int bcm2835_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_therm *therm = devm_kzalloc(dev, sizeof(*therm),
		GFP_KERNEL);
	if (!therm) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	therm->dev = dev;
	dev_set_drvdata(dev, therm);
	mutex_init(&therm->lock);
	init_completion(&therm->comp);
	therm->thermal_dev = thermal_zone_device_register("bcm2835_thermal", 1,
		0, therm, &ops, NULL, 0, 0);
	if (IS_ERR(therm->thermal_dev)) {
		dev_err(dev, "Unable to register the thermal device");
		return PTR_ERR(therm->thermal_dev);
	}
	dev_info(dev, "%s enabled\n", pdev->name);
	return 0;
}


static int bcm2835_thermal_remove(struct platform_device *pdev)
{
	struct bcm2835_therm *therm = dev_get_drvdata(&pdev->dev);

	thermal_zone_device_unregister(therm->thermal_dev);
	return 0;
}

static const struct of_device_id bcm2835_thermal_of_match[] = {
	{ .compatible = "brcm,bcm2835-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_thermal_of_match);

static struct platform_driver bcm2835_thermal_driver = {
	.driver = {
		.name = "bcm2835_thermal",
		.owner = THIS_MODULE,
		.of_match_table = bcm2835_thermal_of_match,
	},
	.probe = bcm2835_thermal_probe,
	.remove = bcm2835_thermal_remove,
};

module_platform_driver(bcm2835_thermal_driver);

MODULE_AUTHOR("Craig McGeachie");
MODULE_DESCRIPTION("BCM2835 thermal driver");
MODULE_LICENSE("GPL v2");
