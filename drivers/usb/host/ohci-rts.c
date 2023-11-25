/*
 * OHCI-compliant USB host controller driver for Realtek IPCam SoCs
 *
 * Copyright 2018 Realtek Corp.
 *
 * Derived from the ohci-platform driver
 * Copyright 2007 Michael Buesch <m@bues.ch>
 * Copyright 2011-2012 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Derived from the OCHI-SSB driver
 * Derived from the OHCI-PCI driver
 * Copyright 1999 Roman Weissgaerber
 * Copyright 2000-2002 David Brownell
 * Copyright 1999 Linus Torvalds
 * Copyright 1999 Gregory P. Smith
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/ohci_pdriver.h>
#include <linux/usb/phy.h>
#include <linux/clk.h>
#include <linux/reset.h>

#include "ohci.h"

#define DRIVER_DESC "ohci-rts platform driver"

static const char hcd_name[] = "ohci-rts";

#define RTS_USB_DMA_ALIGN 32

struct dma_aligned_buffer {
	void *kmalloc_ptr;
	void *old_xfer_buffer;
	dma_addr_t old_xfer_dma;
	dma_addr_t phy;
	size_t kmalloc_size;
	u8 data[0];
};

static void free_dma_aligned_buffer(struct urb *urb)
{
	struct dma_aligned_buffer *temp;

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))
		return;

	temp = container_of(urb->transfer_buffer,
		struct dma_aligned_buffer, data);

	if (usb_urb_dir_in(urb))
		memcpy(temp->old_xfer_buffer, temp->data,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->old_xfer_buffer;
	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) {
		urb->transfer_dma = temp->old_xfer_dma;
		usb_free_coherent(urb->dev, temp->kmalloc_size,
			temp->kmalloc_ptr, temp->phy);
	} else {
		kfree(temp->kmalloc_ptr);
	}

	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;
}

static int alloc_dma_aligned_buffer(struct urb *urb, gfp_t mem_flags)
{
	struct dma_aligned_buffer *temp, *kmalloc_ptr;
	size_t kmalloc_size;
	dma_addr_t phy;

	if (urb->num_sgs || urb->sg ||
	    urb->transfer_buffer_length == 0 ||
	    !((uintptr_t)urb->transfer_buffer & (RTS_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	kmalloc_size = urb->transfer_buffer_length +
		sizeof(struct dma_aligned_buffer) +
		((RTS_USB_DMA_ALIGN - 1) << 1);

	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
		kmalloc_ptr = usb_alloc_coherent(urb->dev, kmalloc_size,
				mem_flags, &phy);
	else
		kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);
	if (!kmalloc_ptr)
		return -ENOMEM;

	/* Position our struct dma_aligned_buffer such that data is aligned */
	temp = PTR_ALIGN(kmalloc_ptr + 1, RTS_USB_DMA_ALIGN) - 1;

	temp->kmalloc_ptr = kmalloc_ptr;
	temp->old_xfer_buffer = urb->transfer_buffer;
	if (usb_urb_dir_out(urb))
		memcpy(temp->data, urb->transfer_buffer,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->data;
	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) {
		temp->old_xfer_dma = urb->transfer_dma;
		urb->transfer_dma = ((unsigned long)temp->data -
				(unsigned long)kmalloc_ptr) + phy;
		temp->phy = phy;
		temp->kmalloc_size = kmalloc_size;
	}

	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;

	return 0;
}

static int ohci_rts_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				      gfp_t mem_flags)
{
	int ret;

	ret = alloc_dma_aligned_buffer(urb, mem_flags);
	if (ret)
		return ret;

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
	if (ret)
		free_dma_aligned_buffer(urb);

	return ret;
}

static void ohci_rts_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	free_dma_aligned_buffer(urb);
}

static struct hc_driver __read_mostly ohci_rts_hc_driver;

static const struct ohci_driver_overrides platform_overrides;

static struct usb_ohci_pdata ohci_rts_defaults;

static int ohci_rts_probe(struct platform_device *dev)
{
	struct usb_hcd *hcd;
	struct resource *res_mem;
	struct usb_ohci_pdata *pdata = dev->dev.platform_data;
	int irq;
	int err = -ENOMEM;
#ifndef CONFIG_USB_EHCI_RTS
	static struct usb_phy *phy;
	struct clk *phy_clk;
	struct reset_control *rst;
#endif
/*
	if (!pdata) {
		WARN_ON(1);
		return -ENODEV;
	}
*/
	if (!dev->dev.platform_data)
		dev->dev.platform_data = &ohci_rts_defaults;
	if (!dev->dev.dma_mask)
		dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
	if (!dev->dev.coherent_dma_mask)
		dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	pdata = dev->dev.platform_data;

	if (usb_disabled())
		return -ENODEV;

	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "no irq provided");
		return irq;
	}

	res_mem = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res_mem) {
		dev_err(&dev->dev, "no memory resource provided");
		return -ENXIO;
	}

	if (pdata->power_on) {
		err = pdata->power_on(dev);
		if (err < 0)
			return err;
	}

	hcd = usb_create_hcd(&ohci_rts_hc_driver, &dev->dev,
			dev_name(&dev->dev));
	if (!hcd) {
		err = -ENOMEM;
		goto err_power;
	}

	hcd->rsrc_start = res_mem->start;
	hcd->rsrc_len = resource_size(res_mem);

	hcd->regs = devm_ioremap_resource(&dev->dev, res_mem);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		goto err_put_hcd;
	}

#ifndef CONFIG_USB_EHCI_RTS
	/* enable DEV_CLK_EN */
	phy_clk = clk_get(NULL, "usbphy_host_ck");
	if (!phy_clk) {
		dev_err(&dev->dev, "get clk usbphy host clk fail\n");
		return -1;
	}
	clk_prepare(phy_clk);
	clk_enable(phy_clk);
	clk_put(phy_clk);
	/**/

	phy = devm_usb_get_phy(&dev->dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(phy)) {
		dev_err(&dev->dev, "unable to find usb phy\n");
		err = -ENODEV;
		goto err_put_hcd;
	}

	rst = devm_reset_control_get(&dev->dev, "reset-usb-host");
	if (!rst) {
		dev_err(&dev->dev, "can't find reset usb host control");
		err = -ENODEV;
		goto err_put_hcd;
	}

	/* Reset USB Host */
	reset_control_reset(rst);

	usb_phy_init(phy);
#endif

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err)
		goto err_put_hcd;

	platform_set_drvdata(dev, hcd);

	return err;

err_put_hcd:
	usb_put_hcd(hcd);
err_power:
	if (pdata->power_off)
		pdata->power_off(dev);

	return err;
}

static int ohci_rts_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct usb_ohci_pdata *pdata = dev->dev.platform_data;

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	platform_set_drvdata(dev, NULL);

	if (pdata->power_off)
		pdata->power_off(dev);

	if (pdata == &ohci_rts_defaults)
		dev->dev.platform_data = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int ohci_rts_suspend(struct device *dev)
{
	struct usb_ohci_pdata *pdata = dev->platform_data;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	if (pdata->power_suspend)
		pdata->power_suspend(pdev);

	return 0;
}

static int ohci_rts_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct usb_ohci_pdata *pdata = dev->platform_data;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	if (pdata->power_on) {
		int err = pdata->power_on(pdev);

		if (err < 0)
			return err;
	}

	ohci_resume(hcd, false);
	return 0;
}

#else /* !CONFIG_PM */
#define ohci_rts_suspend	NULL
#define ohci_rts_resume	NULL
#endif /* CONFIG_PM */

static const struct platform_device_id ohci_rts_table[] = {
	{ "ohci-platform", 0 },
	{ }
};
MODULE_DEVICE_TABLE(platform, ohci_rts_table);

static const struct dev_pm_ops ohci_rts_pm_ops = {
	.suspend	= ohci_rts_suspend,
	.resume		= ohci_rts_resume,
};

static const struct of_device_id rts_ohci_dt_ids[] = {
	{ .compatible = "realtek,rts3903-ohci", },
	{ /* sentinel */ },
};

static struct platform_driver ohci_rts_driver = {
	.id_table	= ohci_rts_table,
	.probe		= ohci_rts_probe,
	.remove		= ohci_rts_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ohci-platform",
		.of_match_table = of_match_ptr(rts_ohci_dt_ids),
		.pm	= &ohci_rts_pm_ops,
	}
};

static int __init ohci_rts_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);

	ohci_init_driver(&ohci_rts_hc_driver, &platform_overrides);
	ohci_rts_hc_driver.map_urb_for_dma = ohci_rts_map_urb_for_dma;
	ohci_rts_hc_driver.unmap_urb_for_dma = ohci_rts_unmap_urb_for_dma;

	return platform_driver_register(&ohci_rts_driver);
}
module_init(ohci_rts_init);

static void __exit ohci_rts_cleanup(void)
{
	platform_driver_unregister(&ohci_rts_driver);
}
module_exit(ohci_rts_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("hanley ge");
MODULE_LICENSE("GPL");
