/*
 * EHCI-compliant USB host controller driver for Realtek IPCam SoCs
 *
 * Copyright 2014 Realtek Corp.
 *
 * Derived from the ehci-platform driver
 * Copyright 2007 Steven Brown <sbrown@cortland.com>
 * Copyright 2010-2012 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Derived from the ohci-ssb driver
 * Copyright 2007 Michael Buesch <m@bues.ch>
 *
 * Derived from the EHCI-PCI driver
 * Copyright (c) 2000-2004 by David Brownell
 *
 * Derived from the ohci-pci driver
 * Copyright 1999 Roman Weissgaerber
 * Copyright 2000-2002 David Brownell
 * Copyright 1999 Linus Torvalds
 * Copyright 1999 Gregory P. Smith
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_pdriver.h>
#include <linux/usb/phy.h>
#include <linux/clk.h>
#include <linux/reset.h>

#include "ehci.h"

#define DRIVER_DESC "ehci-rts platform driver"

static const char hcd_name[] = "ehci-rts";
static struct usb_phy *phy;

struct ehci_dw_ext_regs {
	u32		insnreg[9];
};

#define DW_EXT_REGS_OFFSET	0x90

static int ehci_rts_reset(struct usb_hcd *hcd)
{
	struct platform_device *pdev = to_platform_device(hcd->self.controller);
	struct usb_ehci_pdata *pdata = pdev->dev.platform_data;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;
	struct ehci_dw_ext_regs __iomem *dw_ext_regs = NULL;

	hcd->has_tt = pdata->has_tt;
	ehci->has_synopsys_hc_bug = pdata->has_synopsys_hc_bug;
	ehci->big_endian_desc = pdata->big_endian_desc;
	ehci->big_endian_mmio = pdata->big_endian_mmio;

	ehci->caps = hcd->regs + pdata->caps_offset;
	retval = ehci_setup(hcd);
	if (retval)
		return retval;

	if (pdata->no_io_watchdog)
		ehci->need_io_watchdog = 0;

	dw_ext_regs = (void __iomem *)ehci->caps + DW_EXT_REGS_OFFSET;

	/* set packet buffer Out/In threshold */
	ehci_writel(ehci, 0x00800080, &dw_ext_regs->insnreg[1]);

	return 0;
}

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

static int ehci_rts_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
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

static void ehci_rts_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	free_dma_aligned_buffer(urb);
}

static struct hc_driver __read_mostly ehci_rts_hc_driver;

static const struct ehci_driver_overrides platform_overrides __initconst = {
	.reset =	ehci_rts_reset,
};

static struct usb_ehci_pdata ehci_rts_defaults;

static int ehci_rts_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res_mem;
	struct usb_ehci_pdata *pdata;
	int irq;
	int err = -ENOMEM;
	struct clk *phy_clk;
	struct reset_control *rst;

	if (usb_disabled())
		return -ENODEV;

	/*
	 * use reasonable defaults so platforms don't have to provide these.
	 * with DT probing on ARM, none of these are set.
	 */
	if (!pdev->dev.platform_data)
		pdev->dev.platform_data = &ehci_rts_defaults;
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	pdata = pdev->dev.platform_data;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq provided");
		return irq;
	}
	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem) {
		dev_err(&pdev->dev, "no memory resource provided");
		return -ENXIO;
	}

	if (pdata->power_on) {
		err = pdata->power_on(pdev);
		if (err < 0)
			return err;
	}

	hcd = usb_create_hcd(&ehci_rts_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd) {
		err = -ENOMEM;
		goto err_power;
	}

	hcd->rsrc_start = res_mem->start;
	hcd->rsrc_len = resource_size(res_mem);

	hcd->regs = devm_ioremap_resource(&pdev->dev, res_mem);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		goto err_put_hcd;
	}

	/* enable HOST_CLK_EN */
	phy_clk = clk_get(NULL, "usbphy_host_ck");
	if (!phy_clk) {
		dev_err(&pdev->dev, "get clk usbphy host clk fail\n");
		return -1;
	}
	clk_prepare(phy_clk);
	clk_enable(phy_clk);
	clk_put(phy_clk);

	phy = devm_usb_get_phy(&pdev->dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(phy)) {
		dev_err(&pdev->dev, "unable to find usb phy\n");
		err = -ENODEV;
		goto err_put_hcd;
	}

	rst = devm_reset_control_get(&pdev->dev, "reset-usb-host");
	if (!rst) {
		dev_err(&pdev->dev, "can't find reset usb host control");
		err = -ENODEV;
		goto err_put_hcd;
	}

	/* Reset USB Host */
	reset_control_reset(rst);

	usb_phy_init(phy);

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err)
		goto err_put_hcd;

	platform_set_drvdata(pdev, hcd);

	return err;

err_put_hcd:
	usb_put_hcd(hcd);
err_power:
	if (pdata->power_off)
		pdata->power_off(pdev);

	return err;
}

static int ehci_rts_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct usb_ehci_pdata *pdata = dev->dev.platform_data;

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	platform_set_drvdata(dev, NULL);

	if (pdata->power_off)
		pdata->power_off(dev);

	if (pdata == &ehci_rts_defaults)
		dev->dev.platform_data = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int ehci_rts_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct usb_ehci_pdata *pdata = dev->platform_data;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	bool do_wakeup = device_may_wakeup(dev);
	int ret;

	ret = ehci_suspend(hcd, do_wakeup);

	if (pdata->power_suspend)
		pdata->power_suspend(pdev);

	return ret;
}

static int ehci_rts_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct usb_ehci_pdata *pdata = dev->platform_data;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	if (pdata->power_on) {
		int err = pdata->power_on(pdev);

		if (err < 0)
			return err;
	}

	ehci_resume(hcd, false);
	return 0;
}

#else /* !CONFIG_PM */
#define ehci_rts_suspend	NULL
#define ehci_rts_resume		NULL
#endif /* CONFIG_PM */

static const struct dev_pm_ops ehci_rts_pm_ops = {
	.suspend	= ehci_rts_suspend,
	.resume		= ehci_rts_resume,
};

static const struct of_device_id rts_ehci_dt_ids[] = {
	{ .compatible = "realtek,rts3903-ehci", },
	{ /* sentinel */ },
};

static struct platform_driver ehci_rts_driver = {
	.probe		= ehci_rts_probe,
	.remove		= ehci_rts_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ehci-platform",
		.of_match_table = of_match_ptr(rts_ehci_dt_ids),
		.pm	= &ehci_rts_pm_ops,
	}
};

static int __init ehci_rts_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);

	ehci_init_driver(&ehci_rts_hc_driver, &platform_overrides);
	ehci_rts_hc_driver.map_urb_for_dma = ehci_rts_map_urb_for_dma;
	ehci_rts_hc_driver.unmap_urb_for_dma = ehci_rts_unmap_urb_for_dma;

	return platform_driver_register(&ehci_rts_driver);
}
module_init(ehci_rts_init);

static void __exit ehci_rts_cleanup(void)
{
	platform_driver_unregister(&ehci_rts_driver);
}
module_exit(ehci_rts_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Wei WANG");
MODULE_LICENSE("GPL");
