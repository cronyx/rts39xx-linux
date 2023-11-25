#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/video.h>

#include "f_uvc.h"
#include <linux/usb/composite.h>


static int rtsx_uvc_config_bind(struct usb_configuration *c);


#include "android_uvc.c"

#define RTSX_UVC_VENDOR_ID		0x0bda	/* Linux Foundation */
#define RTSX_UVC_PRODUCT_ID		0x3901	/* Webcam A/V gadget */
#define RTSX_UVC_DEVICE_BCD		0x3901	/* 0.10 */

static const char rtsx_uvc_vendor_label[] = "Generic";
static const char rtsx_uvc_product_label[] = "USB Camera";
static const char rtsx_uvc_serial_label[] = "200901010001";
static const char rtsx_uvc_config_label[] = "USB Camera";

/* string IDs are assigned dynamically */

#define CONFIG_DESCRIPTION_IDX		USB_GADGET_FIRST_AVAIL_IDX

static struct usb_string rtsx_uvc_strings[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = rtsx_uvc_vendor_label,
	[USB_GADGET_PRODUCT_IDX].s = rtsx_uvc_product_label,
	[USB_GADGET_SERIAL_IDX].s = rtsx_uvc_serial_label,
	[CONFIG_DESCRIPTION_IDX].s = rtsx_uvc_config_label,
	{}
};

static struct usb_gadget_strings rtsx_uvc_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = rtsx_uvc_strings,
};

static struct usb_gadget_strings *rtsx_uvc_device_strings[] = {
	&rtsx_uvc_stringtab,
	NULL,
};

static struct usb_device_descriptor rtsx_uvc_device_descriptor = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_MISC,
	.bDeviceSubClass = 0x02,
	.bDeviceProtocol = 0x01,
	.bMaxPacketSize0 = 64,	/* dynamic */
	.idVendor = cpu_to_le16(RTSX_UVC_VENDOR_ID),
	.idProduct = cpu_to_le16(RTSX_UVC_PRODUCT_ID),
	.bcdDevice = cpu_to_le16(RTSX_UVC_DEVICE_BCD),
	.iManufacturer = 0,	/* dynamic */
	.iProduct = 0,		/* dynamic */
	.iSerialNumber = 0,	/* dynamic */
	.bNumConfigurations = 0,	/* dynamic */
};

static struct usb_configuration rtsx_uvc_config_driver = {
	.label = rtsx_uvc_config_label,
	.bConfigurationValue = 1,
	.iConfiguration = 0,	/* dynamic */
	.bmAttributes = USB_CONFIG_ATT_SELFPOWER,
	.MaxPower = CONFIG_USB_GADGET_VBUS_DRAW,
};

static struct usb_function_instance *fi_uvc;
static struct usb_function *f_uvc;

static unsigned int trace;
module_param(trace, uint, 0644);
MODULE_PARM_DESC(trace, "Trace level bitmask");

static int rtsx_uvc_config_bind(struct usb_configuration *c)
{
	int status = 0;

	f_uvc = usb_get_function(fi_uvc);
	if (IS_ERR(f_uvc))
		return PTR_ERR(f_uvc);

	status = usb_add_function(c, f_uvc);
	if (status < 0)
		usb_put_function(f_uvc);

	return status;
}

static int
rtsx_uvc_unbind(struct usb_composite_dev *cdev)
{
	if (!IS_ERR_OR_NULL(f_uvc))
		usb_put_function(f_uvc);
	if (!IS_ERR_OR_NULL(fi_uvc))
		usb_put_function_instance(fi_uvc);
	return 0;
}

static int rtsx_uvc_bind(struct usb_composite_dev *cdev)
{
	int ret;
	/* Register our configuration. */

	struct f_uvc_opts *uvc_opts;

	fi_uvc = usb_get_function_instance("uvc");
	if (IS_ERR(fi_uvc))
		return PTR_ERR(fi_uvc);

	uvc_opts = container_of(fi_uvc, struct f_uvc_opts, func_inst);

	uvc_set_trace_param(trace);

	uvc_opts->fs_control =
		(const struct uvc_descriptor_header * const	*)
		uvc_fs_control_cls;

	ret = usb_string_ids_tab(cdev, rtsx_uvc_strings);
	if (ret < 0)
		goto error;

	rtsx_uvc_device_descriptor.iManufacturer =
	    rtsx_uvc_strings[USB_GADGET_MANUFACTURER_IDX].id;
	rtsx_uvc_device_descriptor.iProduct =
	    rtsx_uvc_strings[USB_GADGET_PRODUCT_IDX].id;
	rtsx_uvc_device_descriptor.iSerialNumber =
	    rtsx_uvc_strings[USB_GADGET_SERIAL_IDX].id;
	rtsx_uvc_config_driver.iConfiguration =
	    rtsx_uvc_strings[CONFIG_DESCRIPTION_IDX].id;
	ret = rtsx_uvc_add_config(cdev, &rtsx_uvc_config_driver,
				  rtsx_uvc_config_bind);
	if (ret < 0)
		goto error;
	rlxprintk(RLX_TRACE_DEBUG, "rtsx_uvc Video Gadget\n");
	return 0;

error:
	usb_put_function_instance(fi_uvc);
	return ret;
}

/* --------------------------------------------------------------------------
 * Driver
 */

static __refdata struct usb_composite_driver rtsx_uvc_driver = {
	.name = "rtsx_uvc",
	.dev = &rtsx_uvc_device_descriptor,
	.strings = rtsx_uvc_device_strings,
	.max_speed = USB_SPEED_HIGH,
	.bind = rtsx_uvc_bind,
	.unbind = rtsx_uvc_unbind,
};

static int __init rtsx_uvc_init(void)
{
	int ret;

	bypass_usb_pullup();
	rtsx_uvc_function_init();
	rtsx_uvc_function_enable();

	ret = usb_composite_probe(&rtsx_uvc_driver);

	return ret;
}

static void __exit rtsx_uvc_cleanup(void)
{
	rtsx_uvc_function_cleanup();
	usb_composite_unregister(&rtsx_uvc_driver);
}

module_init(rtsx_uvc_init);
module_exit(rtsx_uvc_cleanup);

DECLARE_USB_FUNCTION_INIT(uvc, uvc_alloc_inst, uvc_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Laurent Pinchart");

