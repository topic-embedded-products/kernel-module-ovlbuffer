/*
 * AXI VDMA Overlay buffer driver
 * Based on: vdmfb.c
 *
 * Copyright (C) 2017 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 *
 * Licensed under GPL-2.
 *
 * Example devicetree contents:
	axi_overlay_mixer: axi_overlay_mixer@43c10000 {
		compatible = "topic,axi-overlay-mixer";
		reg = <0x43c10000 0x10000>;
		dmas = <&axi_vdma_0 0>, <&axi_vdma_0 1>;
		dma-names = "frombuf", "tobuf";
		hsize = <960>;
		vsize = <960>;
	};
 */

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma/xilinx_dma.h>

/* Register locations */
#define OVLBUF_CONTROL	0x00

#define OVLBUF_BASE_FRAME_WIDTH 0x10
#define OVLBUF_BASE_FRAME_HEIGHT 0x14

#define OVLBUF_INSERT_WIDTH 0x20
#define OVLBUF_INSERT_HEIGHT 0x24
#define OVLBUF_INSERT_POSX 0x28
#define OVLBUF_INSERT_POSY 0x2C

/* Register control flags */
#define OVLBUF_CONTROL_ENABLE 1

struct ovlbuf_dev {
	struct device *device;
	void __iomem *regs;
	/* Physical and virtual addresses of framebuffer */
	phys_addr_t fb_phys;
	void __iomem *fb_virt;
	/* VDMA handle */
	struct dma_chan *dma_tobuf;
	struct dma_interleaved_template *dma_template_tobuf;
	struct dma_chan *dma_frombuf;
	struct dma_interleaved_template *dma_template_frombuf;
	/* Configuration */
	u32 hsize; /* In bytes, currently using 8bpp */
	u32 vsize; /* Scanlines */
	u32 frames;
	u32 pos_x;
	u32 pos_y;
	u32 screen_width;
	u32 screen_height;
};

static inline u32 ovlbuf_readreg(struct ovlbuf_dev *fbdev, loff_t offset)
{
	return ioread32(fbdev->regs + offset);
}

static inline void ovlbuf_writereg(struct ovlbuf_dev *fbdev, loff_t offset, u32 data)
{
	iowrite32(data, fbdev->regs + offset);
}

/**
 * struct xilinx_vdma_config - VDMA Configuration structure
 * @frm_dly: Frame delay
 * @gen_lock: Whether in gen-lock mode
 * @master: Master that it syncs to
 * @frm_cnt_en: Enable frame count enable
 * @park: Whether wants to park
 * @park_frm: Frame to park on
 * @coalesc: Interrupt coalescing threshold
 * @delay: Delay counter
 * @reset: Reset Channel
 * @ext_fsync: External Frame Sync source
 */
static void ovlbuf_configure_vdma(struct ovlbuf_dev *fbdev)
{
	struct xilinx_vdma_config vdma_config;

	memset(&vdma_config, 0, sizeof(vdma_config));
	vdma_config.gen_lock = 1;
	vdma_config.coalesc = 240; /* Get rid of useless interrupts */
	/* Setting "reset" makes the call return immediately without applying */
	vdma_config.reset = 1;
	xilinx_vdma_channel_set_config(fbdev->dma_tobuf, &vdma_config);
	xilinx_vdma_channel_set_config(fbdev->dma_frombuf, &vdma_config);
	vdma_config.reset = 0; /* Now actually apply the settings... */
	xilinx_vdma_channel_set_config(fbdev->dma_tobuf, &vdma_config);
	xilinx_vdma_channel_set_config(fbdev->dma_frombuf, &vdma_config);
}

static int ovlbuf_setupfb(struct ovlbuf_dev *fbdev)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_interleaved_template *dma_template = fbdev->dma_template_frombuf;
	u32 frame;
	u32 fbsize = fbdev->hsize * fbdev->vsize;

	/* Disable display */
	ovlbuf_writereg(fbdev, OVLBUF_CONTROL, 0);

	dmaengine_terminate_all(fbdev->dma_tobuf);
	dmaengine_terminate_all(fbdev->dma_frombuf);

	if (!fbdev->hsize || !fbdev->vsize) {
		dev_info(fbdev->device, "Nul sized area, not starting.\n");
		return 0;
	}

	/* Configure */
	ovlbuf_writereg(fbdev, OVLBUF_BASE_FRAME_WIDTH, fbdev->screen_width);
	ovlbuf_writereg(fbdev, OVLBUF_BASE_FRAME_HEIGHT, fbdev->screen_height);
	ovlbuf_writereg(fbdev, OVLBUF_INSERT_WIDTH, fbdev->hsize);
	ovlbuf_writereg(fbdev, OVLBUF_INSERT_HEIGHT, fbdev->vsize);
	/* Quirk in logic requires us to add +1 to the position */
	ovlbuf_writereg(fbdev, OVLBUF_INSERT_POSX, fbdev->pos_x + 1);
	ovlbuf_writereg(fbdev, OVLBUF_INSERT_POSY, fbdev->pos_y + 1);

	/* Setup VDMA address etc */
	ovlbuf_configure_vdma(fbdev);

	/*
	* Interleaved DMA:
	* Each interleaved frame is a row (hsize) implemented in ONE
	* chunk (sgl has len 1).
	* The number of interleaved frames is the number of rows (vsize).
	* The icg in used to pack data to the HW, so that the buffer len
	* is fb->piches[0], but the actual size for the hw is somewhat less
	*/
	dma_template->dir = DMA_MEM_TO_DEV;
	dma_template->src_start = fbdev->fb_phys;
	/* sgl list have just one entry (each interleaved frame have 1 chunk) */
	dma_template->frame_size = 1;
	/* the number of interleaved frame, each has the size specified in sgl */
	dma_template->numf = fbdev->vsize;
	dma_template->src_sgl = 1;
	dma_template->src_inc = 1;
	/* vdma IP does not provide any addr to the hdmi IP */
	dma_template->dst_inc = 0;
	dma_template->dst_sgl = 0;
	/* horizontal size */
	dma_template->sgl[0].size = fbdev->hsize;
	/* the vdma driver seems to look at icg, and not src_icg */
	dma_template->sgl[0].icg = 0; /*  gap is stride - hsize */

	for (frame = 0; frame < fbdev->frames; ++frame) {
		desc = dmaengine_prep_interleaved_dma(fbdev->dma_frombuf, dma_template, 0);
		if (!desc) {
			dev_err(fbdev->device, "Failed to prepare from-buffer DMA descriptor %u\n", frame);
			return -ENOMEM;
		}
		dmaengine_submit(desc);
		dma_template->src_start += fbsize;
	}
	dma_async_issue_pending(fbdev->dma_frombuf);

	dma_template = fbdev->dma_template_tobuf;
	dma_template->dir = DMA_DEV_TO_MEM;
	dma_template->dst_start = fbdev->fb_phys;
	/* sgl list have just one entry (each interleaved frame have 1 chunk) */
	dma_template->frame_size = 1;
	dma_template->numf = fbdev->vsize;
	dma_template->src_sgl = 0;
	dma_template->src_inc = 0;
	dma_template->dst_inc = 1;
	dma_template->dst_sgl = 1;
	dma_template->sgl[0].size = fbdev->hsize;
	dma_template->sgl[0].icg = 0; /*  stride - hsize */

	for (frame = 0; frame < fbdev->frames; ++frame) {
		desc = dmaengine_prep_interleaved_dma(fbdev->dma_tobuf, dma_template, 0);
		if (!desc) {
			dev_err(fbdev->device, "Failed to prepare to-buffer DMA descriptor %u\n", frame);
			return -ENOMEM;
		}
		dmaengine_submit(desc);
		dma_template->dst_start += fbsize;
	}
	dma_async_issue_pending(fbdev->dma_tobuf);

	/* Enable output */
	ovlbuf_writereg(fbdev, OVLBUF_CONTROL, OVLBUF_CONTROL_ENABLE);

	return 0;
}


static int ovlbuf_probe(struct platform_device *pdev)
{
	struct ovlbuf_dev *fbdev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	int ret;
	void __iomem *fb;
	u32 fbsize;
	u32 i;
	u32 j;
	u8 pixel;

	fbdev = devm_kzalloc(&pdev->dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, fbdev);
	fbdev->device = &pdev->dev;

	fbdev->dma_template_tobuf = devm_kzalloc(&pdev->dev,
		sizeof(struct dma_interleaved_template) +
		sizeof(struct data_chunk), GFP_KERNEL);
	if (!fbdev->dma_template_tobuf)
		return -ENOMEM;
	fbdev->dma_template_frombuf = devm_kzalloc(&pdev->dev,
		sizeof(struct dma_interleaved_template) +
		sizeof(struct data_chunk), GFP_KERNEL);
	if (!fbdev->dma_template_frombuf)
		return -ENOMEM;

	fbdev->hsize = 960;
	fbdev->vsize = 960;
	fbdev->frames = 3;
	fbdev->screen_width = 1920;
	fbdev->screen_height = 1080;

	ret = of_property_read_u32(np, "num-frames", &fbdev->frames);
	if (ret)
		dev_warn(&pdev->dev, "Missing num-frames property, assume %u\n", fbdev->frames);
	ret = of_property_read_u32(np, "hsize", &fbdev->hsize);
	if (ret)
		dev_warn(&pdev->dev, "Missing hsize property, assume %u\n", fbdev->hsize);
	ret = of_property_read_u32(np, "vsize", &fbdev->hsize);
	if (ret)
		dev_warn(&pdev->dev, "Missing vsize property, assume %u\n", fbdev->vsize);

	ret = of_property_read_u32(np, "screen-width", &fbdev->screen_width);
	if (ret)
		dev_warn(&pdev->dev, "Missing screen-width property, assume %u\n", fbdev->screen_width);

	ret = of_property_read_u32(np, "screen-height", &fbdev->screen_height);
	if (ret)
		dev_warn(&pdev->dev, "Missing screen-height property, assume %u\n", fbdev->screen_height);

	/* Put in center by default.*/
	ret = of_property_read_u32(np, "pos-x", &fbdev->pos_x);
	if (ret) {
		fbdev->pos_x = (fbdev->screen_width - fbdev->hsize) / 2;
		dev_warn(&pdev->dev, "Missing pos-x property, assume %u\n", fbdev->pos_x);
	}
	ret = of_property_read_u32(np, "pos-y", &fbdev->pos_y);
	if (ret) {
		fbdev->pos_y = (fbdev->screen_height - fbdev->vsize) / 2;
		dev_warn(&pdev->dev, "Missing pos-y property, assume %u\n", fbdev->pos_y);
	}

	/* Request I/O resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "I/O resource request failed\n");
		return -ENXIO;
	}
	res->flags &= ~IORESOURCE_CACHEABLE;
	fbdev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fbdev->regs))
		return PTR_ERR(fbdev->regs);

	/* Allocate framebuffer memory */
	fbsize = fbdev->hsize * fbdev->vsize * fbdev->frames;
	fbdev->fb_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(fbsize),
					    &fbdev->fb_phys, GFP_KERNEL);
	if (!fbdev->fb_virt) {
		dev_err(&pdev->dev,
			"Frame buffer memory allocation failed\n");
		return -ENOMEM;
	}

	dev_info(fbdev->device, "virt=%p phys=%#x size=%u\n",
		fbdev->fb_virt, (u32)fbdev->fb_phys, fbsize);

	/* Fill framebuffer with gradient */
	fb = fbdev->fb_virt;
	for (j = 0; j < fbdev->frames; ++j)
	{
		pixel = 0x0;
		dev_info(fbdev->device, "frame: %u virt=%p\n", j, fb);
		for (i = fbdev->vsize; i != 0; --i) {
			memset_io(fb, pixel, fbdev->hsize);
			++pixel;
			fb += fbdev->hsize;
		}
	}

	fbdev->dma_tobuf = dma_request_slave_channel(&pdev->dev, "tobuf");
	if (IS_ERR_OR_NULL(fbdev->dma_tobuf)) {
		dev_err(&pdev->dev, "Failed to allocate DMA channel (%d).\n", ret);
		if (fbdev->dma_tobuf)
			ret = PTR_ERR(fbdev->dma_tobuf);
		else
			ret = -EPROBE_DEFER;
		goto err_dma_free;
	}
	fbdev->dma_frombuf = dma_request_slave_channel(&pdev->dev, "frombuf");
	if (IS_ERR_OR_NULL(fbdev->dma_frombuf)) {
		dev_err(&pdev->dev, "Failed to allocate DMA channel (%d).\n", ret);
		if (fbdev->dma_frombuf)
			ret = PTR_ERR(fbdev->dma_frombuf);
		else
			ret = -EPROBE_DEFER;
		goto err_channel_tobuf_free;
	}

	/* Setup and enable the framebuffer */
	ret = ovlbuf_setupfb(fbdev);
	if (ret)
		return ret;

	dev_info(fbdev->device, "probed, buffer size: %u\n", fbsize);

	return 0;

err_channel_tobuf_free:
	dma_release_channel(fbdev->dma_tobuf);
err_dma_free:
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(fbsize), fbdev->fb_virt,
			  fbdev->fb_phys);

	return ret;
}

static int ovlbuf_remove(struct platform_device *pdev)
{
	struct ovlbuf_dev *fbdev = platform_get_drvdata(pdev);
	u32 fbsize = fbdev->hsize * fbdev->vsize * fbdev->frames;

	/* Disable display */
	ovlbuf_writereg(fbdev, OVLBUF_CONTROL, 0);

	dma_release_channel(fbdev->dma_tobuf);
	dma_release_channel(fbdev->dma_frombuf);
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(fbsize),
			  fbdev->fb_virt, fbdev->fb_phys);
	return 0;
}

static struct of_device_id ovlbuf_match[] = {
	{ .compatible = "topic,axi-overlay-mixer", },
	{},
};
MODULE_DEVICE_TABLE(of, ovlbuf_match);

static struct platform_driver ovlbuf_driver = {
	.probe  = ovlbuf_probe,
	.remove	= ovlbuf_remove,
	.driver = {
		.name = "axi-overlay-mixer",
		.of_match_table = ovlbuf_match,
	}
};
module_platform_driver(ovlbuf_driver);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_DESCRIPTION("AXI Overlay buffer driver");
MODULE_LICENSE("GPL v2");
