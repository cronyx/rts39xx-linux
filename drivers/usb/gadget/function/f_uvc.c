/*
 *	uvc_gadget.c  --  USB Video Class Gadget driver
 *
 *	Copyright (C) 2009-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/video.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "uvc_queue.c"
#include "uvc_video.c"
#include "uvc_v4l2.c"

#include "f_uvc.h"
#include "u_uvc.h"
#include "uvc.h"
#include "uvc_configfs.h"
#include "uvc_v4l2.h"
#include "uvc_video.h"

unsigned int uvc_gadget_trace_param;
unsigned int int_alt_setting;

/* --------------------------------------------------------------------------
 * Function descriptors
 */

/* string IDs are assigned dynamically */

#define UVC_STRING_CONTROL_IDX			0
#define UVC_STRING_STREAMING_IDX		1

#define UVC_INTF_VIDEO_CONTROL			0
#define UVC_INTF_VIDEO_STREAMING		1
#define H264_INTF_VIDEO_STREAMING		2

#define UVC_STATUS_MAX_PACKET_SIZE		16	/* 16 bytes status */

static struct usb_interface_assoc_descriptor uvc_iad = {
	.bLength		= sizeof(uvc_iad),
	.bDescriptorType	= USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface	= 0,
	.bInterfaceCount	= 2,
	.bFunctionClass		= USB_CLASS_VIDEO,
	.bFunctionSubClass	= UVC_SC_VIDEO_INTERFACE_COLLECTION,
	.bFunctionProtocol	= 0x00,
	.iFunction		= 0,
};

static struct usb_interface_descriptor uvc_control_intf = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_CONTROL,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOCONTROL,
#ifndef CONFIG_USB_RTSX_UVC_15
	.bInterfaceProtocol = 0x00,
#else
	.bInterfaceProtocol = 0x01,
#endif
	.iInterface		= 0,
};

static struct usb_ss_ep_comp_descriptor uvc_ss_control_comp = {
	.bLength		= sizeof(uvc_ss_control_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The following 3 values can be tweaked if necessary. */
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	.wBytesPerInterval	= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
};

static struct usb_endpoint_descriptor uvc_control_cs_ep = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
	.bInterval = 6,
};

static struct uvc_control_endpoint_descriptor uvc_status_endpoint = {
	.bLength = UVC_DT_CONTROL_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_CS_ENDPOINT,
	.bDescriptorSubType = UVC_EP_INTERRUPT,
	.wMaxTransferSize = cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
};

static struct usb_interface_descriptor uvc_streaming_intf_alt0 = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UVC_INTF_VIDEO_STREAMING,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_VIDEO,
	.bInterfaceSubClass = UVC_SC_VIDEOSTREAMING,
#ifndef CONFIG_USB_RTSX_UVC_15
	.bInterfaceProtocol = 0x00,
#else
	.bInterfaceProtocol = 0x01,
#endif
	.iInterface = 0,
};

static struct usb_endpoint_descriptor uvc_fs_streaming_ep = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
	.wMaxPacketSize = 64,
	.bInterval = 0,
};

static struct usb_endpoint_descriptor uvc_hs_streaming_ep = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
	.wMaxPacketSize = 512,
	.bInterval = 0,
};

static struct usb_endpoint_descriptor uvc_ss_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,

	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
};

static struct usb_ss_ep_comp_descriptor uvc_ss_streaming_comp = {
	.bLength		= sizeof(uvc_ss_streaming_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The bMaxBurst, bmAttributes and wBytesPerInterval values will be
	 * initialized from module parameters.
	 */
};

static const struct usb_descriptor_header * const uvc_fs_streaming[] = {
	(struct usb_descriptor_header *) &uvc_fs_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_hs_streaming[] = {
	(struct usb_descriptor_header *) &uvc_hs_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_ss_streaming[] = {
	(struct usb_descriptor_header *) &uvc_ss_streaming_ep,
	(struct usb_descriptor_header *) &uvc_ss_streaming_comp,
	NULL,
};

void uvc_set_trace_param(unsigned int trace)
{
	uvc_gadget_trace_param = trace;
}
EXPORT_SYMBOL(uvc_set_trace_param);

static int outdatastall;


/* --------------------------------------------------------------------------
 * Control requests
 */

static void
uvc_function_ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct uvc_device *uvc = req->context;
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;

	if (uvc->event_setup_out) {
		uvc->event_setup_out = 0;

		memset(&v4l2_event, 0, sizeof(v4l2_event));
		v4l2_event.type = UVC_EVENT_DATA;
		uvc_event->data.length = req->actual;
		memcpy(&uvc_event->data.data, req->buf, req->actual);
		v4l2_event_queue(&uvc->vdev, &v4l2_event);
	}
}

static void
uvc_function_epc_complete(struct usb_ep *ep, struct usb_request *req)
{

}

static int uvc_function_stream_off(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	if (uvc->state == UVC_STATE_DISCONNECTED)
		return 0;

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_STREAMOFF;
	v4l2_event_queue(&uvc->vdev, &v4l2_event);

	uvc->state = UVC_STATE_CONNECTED;
	return 0;
}

static int videocontrolreq(uint8_t reqtype, uint8_t req, uint16_t index,
			   uint8_t valhigh)
{
	int ret = 0;

	switch (index >> 8) {
	case 0:
		break;
	case ENT_ID_CAMERA_IT:
		if (valhigh != CT_EXPOSURE_TIME_ABSOLUTE_CONTROL &&
		    valhigh != CT_AE_MODE_CONTROL &&
		    valhigh != CT_AE_PRIORITY_CONTROL &&
		    valhigh != CT_PANTILT_ABSOLUTE_CONTROL &&
		    valhigh != CT_ZOOM_ABSOLUTE_CONTROL &&
		    valhigh != CT_ROLL_ABSOLUTE_CONTROL)
			ret = -1;
		break;
	case ENT_ID_PROCESSING_UNIT:
		if (valhigh != PU_BRIGHTNESS_CONTROL &&
		    valhigh != PU_CONTRAST_CONTROL &&
		    valhigh != PU_HUE_CONTROL &&
		    valhigh != PU_SATURATION_CONTROL &&
		    valhigh != PU_SHARPNESS_CONTROL &&
		    valhigh != PU_GAMMA_CONTROL &&
		    valhigh != PU_BACKLIGHT_COMPENSATION_CONTROL &&
		    valhigh != PU_GAIN_CONTROL &&
		    valhigh != PU_POWER_LINE_FREQUENCY_CONTROL &&
		    valhigh != PU_WHITE_BALANCE_TEMPERATURE_CONTROL &&
		    valhigh != PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL)
			ret = -1;
		break;
	case ENT_ID_EXTENSION_UNIT_DBG:
		break;
	case ENT_ID_OUTPUT_TRM:
		ret = -1;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}

static int usbclsreqproc(uint8_t reqtype, uint8_t req, uint16_t index,
			 uint8_t valhigh)
{
	int ret = 0;

	switch (req) {
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_LEN:
	case GET_INFO:
	case GET_DEF:
		ret = videocontrolreq(reqtype, req, index, valhigh);
		break;
	default:
		break;
	}
	return ret;
}

static int usbprosetuppkt(uint8_t reqtype, uint8_t req, uint16_t index,
			  uint8_t valhigh)
{
	int ret = 0;

	switch (reqtype & REQUEST_TYPE) {
	case CLASS_REQUEST:
		if ((index & 0xff) == IF_IDX_VIDEOCONTROL)
			ret = usbclsreqproc(reqtype, req, index, valhigh);
		break;
	default:
		break;
	}

	return ret;
}

static int
uvc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
	int ret = 0;
	uint8_t value_high = le16_to_cpu(ctrl->wValue) >> 8;

	ret = usbprosetuppkt(ctrl->bRequestType, ctrl->bRequest,
			     le16_to_cpu(ctrl->wIndex), value_high);
	if (ret < 0) {
		rlxprintk(RLX_TRACE_DEBUG, "stall request\n");
		return -1;
	}
	if (getstsepactive(4) == 0) {
		rlxprintk(RLX_TRACE_DEBUG, "enable epc\n");
		ret =
		    config_ep_by_speed(f->config->cdev->gadget, &(uvc->func),
				       uvc->video.csep);
		if (ret)
			return ret;
		usb_ep_disable(uvc->video.csep);
		usb_ep_enable(uvc->video.csep);
	}

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		if (ctrl->bRequest == USB_REQ_CLEAR_FEATURE) {
			rlxprintk(RLX_TRACE_DEBUG,
				  "setup USB_REQ_CLEAR_FEATURE\n");
			uvc_function_stream_off(f);
			f->config->cdev->delayed_status++;
			return USB_GADGET_DELAYED_STATUS;
		}
	}

	if ((ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS &&
	    (ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_VENDOR) {
		INFO(f->config->cdev, "invalid request type\n");
		return -EINVAL;
	}

	/* Stall too big requests. */
	if (le16_to_cpu(ctrl->wLength) > UVC_MAX_REQUEST_SIZE)
		return -EINVAL;

	/* Tell the complete callback to generate an event for the next request
	 * that will be enqueued by UVCIOC_SEND_RESPONSE.
	 */
	uvc->event_setup_out = !(ctrl->bRequestType & USB_DIR_IN);
	uvc->event_length = le16_to_cpu(ctrl->wLength);

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_SETUP;
	memcpy(&uvc_event->req, ctrl, sizeof(uvc_event->req));
	v4l2_event_queue(&uvc->vdev, &v4l2_event);
	return ret;
}

int uvc_outdata_setstall(void)
{
	outdatastall = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(uvc_outdata_setstall);

static int
uvc_function_filteroutdata(struct usb_function *f,
			   const struct usb_ctrlrequest *ctrl,
			   unsigned char *buf, int len)
{
	unsigned char brequesttype;
	unsigned char brequest;
	unsigned short wvalue;
	unsigned short windex;
	unsigned short wlength;
	unsigned char wvalue_h;
	unsigned char wvalue_l;
	unsigned char windex_h;
	unsigned char windex_l;

	brequesttype = ctrl->bRequestType;
	brequest = ctrl->bRequest;
	wvalue = ctrl->wValue;
	windex = ctrl->wIndex;
	wlength = ctrl->wLength;
	wvalue_h = ((char *)&(ctrl->wValue))[1];
	wvalue_l = ((char *)&(ctrl->wValue))[0];
	windex_h = ((char *)&(ctrl->wIndex))[1];
	windex_l = ((char *)&(ctrl->wIndex))[0];

	if ((brequesttype == 0x82) && (brequest == 0x0c))
		return 0;
	if ((brequesttype & REQUEST_TYPE) != CLASS_REQUEST)
		return 0;
	if (brequest != SET_CUR)
		return 0;
	if (((brequesttype & 0x83) != 0x1) && ((brequesttype & 0x83) != 0x81))
		return 0;
	if (windex_l != IF_IDX_VIDEOCONTROL)
		return 0;

	if ((windex_h == ENT_ID_EXTENSION_UNIT_DBG) &&
		(wvalue_h == EXU1_CMD_STS) &&
		(brequest == SET_CUR) &&
		(buf[0] == VCMD_DBG)) {
		switch (buf[1]) {
		case VSCMD_IR_MODE_GET:
			return -1;
		default:
			return 0;
		}
	}

	if ((windex_h == ENT_ID_CAMERA_IT) && (brequest == SET_CUR))
		switch (wvalue_h) {
		case CT_PANTILT_ABSOLUTE_CONTROL:
			if ((*(int *)buf > 57600)
				|| (*(int *)buf < -57600))
				return -1;
			if ((*(int *)(buf+4) > 43200)
				|| (*(int *)(buf+4) < -43200))
				return -1;
			break;
		case CT_ROLL_ABSOLUTE_CONTROL:
			if (*(unsigned short *)buf > 3)
				return -1;
			break;
		case CT_ZOOM_ABSOLUTE_CONTROL:
			if (*(short *)buf > 3)
				return -1;
			break;
		}

	if (outdatastall) {
		outdatastall = 0;
		return -1;
	}

	return 0;
}

void uvc_function_setup_continue(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;

	usb_composite_setup_continue(cdev);
}

static int
uvc_function_get_alt(struct usb_function *f, unsigned int interface)
{
	struct uvc_device *uvc = to_uvc(f);

	INFO(f->config->cdev, "uvc_function_get_alt(%u)\n", interface);

	if (interface == uvc->control_intf)
		return 0;
	else if (interface != uvc->streaming_intf)
		return -EINVAL;
	else
		return int_alt_setting;
}

static int
uvc_function_set_alt(struct usb_function *f,
	unsigned int interface, unsigned int alt)
{
	struct uvc_device *uvc = to_uvc(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
	static int init;
	int ret;

	INFO(cdev, "uvc_function_set_alt(%u, %u)\n", interface, alt);

	if (interface == uvc->control_intf) {
		if (alt)
			return -EINVAL;

		if (uvc->state == UVC_STATE_DISCONNECTED) {
			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_CONNECT;
			uvc_event->speed = cdev->gadget->speed;
			v4l2_event_queue(&uvc->vdev, &v4l2_event);

			uvc->state = UVC_STATE_CONNECTED;
		}

		dwc_otg_pcd_wait_clearfeature(1);
		return 0;
	}

	if (interface != uvc->streaming_intf)
		return -EINVAL;

	switch (alt) {
	case 0:
		ret =
		    config_ep_by_speed(f->config->cdev->gadget, &(uvc->func),
				       uvc->video.csep);
		if (ret)
			return ret;
		if (init)
			usb_ep_disable(uvc->video.csep);
		usb_ep_enable(uvc->video.csep);

		ret =
		    config_ep_by_speed(f->config->cdev->gadget, &(uvc->func),
				       uvc->video.ep);
		if (ret)
			return ret;
		if (init)
			usb_ep_disable(uvc->video.ep);
		usb_ep_enable(uvc->video.ep);

		init = 1;
		int_alt_setting = 0;
		uvc->state = UVC_STATE_CONNECTED;
		return 0;

	case 1:
		if (uvc->state != UVC_STATE_CONNECTED)
			return 0;

		if (uvc->video.ep) {
			ret = config_ep_by_speed(f->config->cdev->gadget,
						 &(uvc->func), uvc->video.ep);
			if (ret)
				return ret;
			usb_ep_enable(uvc->video.ep);
		}

		memset(&v4l2_event, 0, sizeof(v4l2_event));
		v4l2_event.type = UVC_EVENT_STREAMON;
		v4l2_event_queue(&uvc->vdev, &v4l2_event);
		int_alt_setting = 1;
		return USB_GADGET_DELAYED_STATUS;

	default:
		return -EINVAL;
	}
}

static void
uvc_function_disable(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	if (uvc->state == UVC_STATE_STREAMING) {

		INFO(f->config->cdev, "uvc_function_disable\n");

		memset(&v4l2_event, 0, sizeof(v4l2_event));
		v4l2_event.type = UVC_EVENT_DISCONNECT;
		v4l2_event_queue(&uvc->vdev, &v4l2_event);

		uvc->state = UVC_STATE_DISCONNECTED;
	}
}

/* --------------------------------------------------------------------------
 * Connection / disconnection
 */

void
uvc_function_connect(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	int ret;

	ret = usb_function_activate(&uvc->func);
	if (ret < 0)
		INFO(cdev, "UVC connect failed with %d\n", ret);
}

void
uvc_function_disconnect(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	int ret;

	ret = usb_function_deactivate(&uvc->func);
	if (ret < 0)
		INFO(cdev, "UVC disconnect failed with %d\n", ret);
}

/* --------------------------------------------------------------------------
 * USB probe and disconnect
 */

static int
uvc_register_video(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;

	/* TODO reference counting. */
	uvc->vdev.v4l2_dev = &uvc->v4l2_dev;
	uvc->vdev.fops = &uvc_v4l2_fops;
	uvc->vdev.ioctl_ops = &uvc_v4l2_ioctl_ops;
	uvc->vdev.release = video_device_release_empty;
	uvc->vdev.vfl_dir = VFL_DIR_TX;
	uvc->vdev.lock = &uvc->video.mutex;
	strlcpy(uvc->vdev.name, cdev->gadget->name, sizeof(uvc->vdev.name));

	video_set_drvdata(&uvc->vdev, uvc);

	return video_register_device(&uvc->vdev, VFL_TYPE_GRABBER, 41);
}

#define UVC_COPY_DESCRIPTOR(mem, dst, desc) \
	do { \
		memcpy(mem, desc, (desc)->bLength); \
		*(dst)++ = mem; \
		mem += (desc)->bLength; \
	} while (0)

#define UVC_COPY_DESCRIPTORS(mem, dst, src) \
	do { \
		const struct usb_descriptor_header * const *__src; \
		for (__src = src; *__src; ++__src) { \
			memcpy(mem, *__src, (*__src)->bLength); \
			*dst++ = mem; \
			mem += (*__src)->bLength; \
		} \
	} while (0)

static struct usb_descriptor_header **
uvc_copy_descriptors(struct uvc_device *uvc, enum usb_device_speed speed)
{
	struct uvc_input_header_descriptor *uvc_streaming_header;
	struct uvc_header_descriptor *uvc_control_header;
	const struct uvc_descriptor_header * const *uvc_control_desc;
	const struct uvc_descriptor_header * const *uvc_streaming_cls;
	const struct usb_descriptor_header * const *uvc_streaming_std;
	const struct usb_descriptor_header * const *src;
	struct usb_descriptor_header **dst;
	struct usb_descriptor_header **hdr;
	unsigned int control_size;
	unsigned int streaming_size;
	unsigned int n_desc;
	unsigned int bytes;
	struct desc_entry *q;
	void *mem;

	switch (speed) {
	case USB_SPEED_SUPER:
		uvc_control_desc = uvc->desc.ss_control;
		uvc_streaming_cls = uvc->desc.ss_streaming;
		uvc_streaming_std = uvc_ss_streaming;
		break;

	case USB_SPEED_HIGH:
		uvc_control_desc = uvc->desc.fs_control;
		uvc_streaming_cls = uvc->desc.hs_streaming;
		uvc_streaming_std = uvc_hs_streaming;
		break;

	case USB_SPEED_FULL:
	default:
		uvc_control_desc = uvc->desc.fs_control;
		uvc_streaming_cls = uvc->desc.fs_streaming;
		uvc_streaming_std = uvc_fs_streaming;
		break;
	}

	/* Descriptors layout
	 *
	 * uvc_iad
	 * uvc_control_intf
	 * Class-specific UVC control descriptors
	 * uvc_control_ep
	 * uvc_control_cs_ep
	 * uvc_ss_control_comp (for SS only)
	 * uvc_streaming_intf_alt0
	 * Class-specific UVC streaming descriptors
	 * uvc_{fs|hs}_streaming
	 */

	/* Count descriptors and compute their size. */
	control_size = 0;
	streaming_size = 0;
	bytes = uvc_iad.bLength + uvc_control_intf.bLength
	    + uvc_status_endpoint.bLength
	    + uvc_control_cs_ep.bLength + uvc_streaming_intf_alt0.bLength;

	if (speed == USB_SPEED_SUPER) {
		bytes += uvc_ss_control_comp.bLength;
		n_desc = 6;
	} else {
		n_desc = 5;
	}

	for (src = (const struct usb_descriptor_header **)uvc_control_desc;
	     *src; ++src) {
		control_size += (*src)->bLength;
		bytes += (*src)->bLength;
		n_desc++;
	}

	n_desc += uvc_get_stream_descs_num();
	bytes += uvc_get_stream_descs_len();

	for (src = uvc_streaming_std; *src; ++src) {
		bytes += (*src)->bLength;
		n_desc++;
	}

	mem = kmalloc((n_desc + 1) * sizeof(*src) + bytes, GFP_KERNEL);
	if (mem == NULL)
		return NULL;

	hdr = mem;
	dst = mem;
	mem += (n_desc + 1) * sizeof(*src);

	/* Copy the descriptors. */
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_iad);
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_intf);

	uvc_control_header = mem;

	UVC_COPY_DESCRIPTORS(mem, dst, (const struct usb_descriptor_header **)
			     uvc_control_desc);

	uvc_control_header->wTotalLength = cpu_to_le16(control_size);

	if (speed == USB_SPEED_SUPER)
		UVC_COPY_DESCRIPTOR(mem, dst, &uvc_ss_control_comp);

	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_cs_ep);
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_status_endpoint);

	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_streaming_intf_alt0);

	UVC_COPY_DESCRIPTORS(mem, dst, uvc_streaming_std);

	uvc_streaming_header = mem;

	list_for_each_entry(q, &uvc_desc_list, list) {
		streaming_size += q->desc->bLength;
		UVC_COPY_DESCRIPTOR(mem, dst, q->desc);
	}
	uvc_streaming_header->wTotalLength = cpu_to_le16(streaming_size);
	uvc_streaming_header->bEndpointAddress = uvc->video.ep->address;

	*dst = NULL;
	return hdr;
}

static int
uvc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct uvc_device *uvc = to_uvc(f);
	unsigned int max_packet_mult;
	unsigned int max_packet_size;
	struct usb_ep *ep;
	struct f_uvc_opts *opts;
	int ret = -EINVAL;

	INFO(cdev, "uvc_function_bind\n");

	opts = fi_to_f_uvc_opts(f->fi);
	/* Sanity check the streaming endpoint module parameters.
	 */
	opts->streaming_interval = clamp(opts->streaming_interval, 1U, 16U);
	opts->streaming_maxpacket = clamp(opts->streaming_maxpacket, 1U, 3072U);
	opts->streaming_maxburst = min(opts->streaming_maxburst, 15U);

	/* Fill in the FS/HS/SS Video Streaming specific descriptors from the
	 * module parameters.
	 *
	 * NOTE: We assume that the user knows what they are doing and won't
	 * give parameters that their UDC doesn't support.
	 */
	if (opts->streaming_maxpacket <= 1024) {
		max_packet_mult = 1;
		max_packet_size = opts->streaming_maxpacket;
	} else if (opts->streaming_maxpacket <= 2048) {
		max_packet_mult = 2;
		max_packet_size = opts->streaming_maxpacket / 2;
	} else {
		max_packet_mult = 3;
		max_packet_size = opts->streaming_maxpacket / 3;
	}

	uvc_fs_streaming_ep.bInterval = opts->streaming_interval;
	uvc_hs_streaming_ep.bInterval = opts->streaming_interval;

	uvc_ss_streaming_ep.bInterval = opts->streaming_interval;
	uvc_ss_streaming_comp.bmAttributes = max_packet_mult - 1;
	uvc_ss_streaming_comp.bMaxBurst = opts->streaming_maxburst;
	uvc_ss_streaming_comp.wBytesPerInterval =
		cpu_to_le16(max_packet_size * max_packet_mult *
			    (opts->streaming_maxburst + 1));

	if (gadget_is_superspeed(c->cdev->gadget))
		ep = usb_ep_autoconfig_ss(cdev->gadget, &uvc_ss_streaming_ep,
					  &uvc_ss_streaming_comp);
	else if (gadget_is_dualspeed(cdev->gadget)) {
		ep = usb_ep_autoconfig(cdev->gadget, &uvc_hs_streaming_ep);
		uvc_hs_streaming_ep.wMaxPacketSize = 512;
	} else
		ep = usb_ep_autoconfig(cdev->gadget, &uvc_fs_streaming_ep);

	if (!ep) {
		INFO(cdev, "Unable to allocate streaming EP\n");
		goto error;
	}
	uvc->video.ep = ep;

	uvc_fs_streaming_ep.bEndpointAddress = uvc->video.ep->address;
	uvc_hs_streaming_ep.bEndpointAddress = uvc->video.ep->address;
	uvc_ss_streaming_ep.bEndpointAddress = uvc->video.ep->address;

	dwc_otg_pcd_wait_clearfeature(1);

	if (gadget_is_superspeed(c->cdev->gadget))
		ep = usb_ep_autoconfig_ss(cdev->gadget, &uvc_control_cs_ep,
					  &uvc_ss_streaming_comp);
	else if (gadget_is_dualspeed(cdev->gadget)) {
		ep = usb_ep_autoconfig(cdev->gadget, &uvc_control_cs_ep);
	} else
		ep = usb_ep_autoconfig(cdev->gadget, &uvc_control_cs_ep);

	if (!ep) {
		INFO(cdev, "Unable to allocate Interrupt EP\n");
		goto error;
	}
	uvc->video.csep = ep;

	/* Allocate interface IDs. */
	ret = usb_interface_id(c, f);
	if (ret < 0)
		goto error;
	uvc_iad.bFirstInterface = ret;
	uvc_control_intf.bInterfaceNumber = ret;
	uvc->control_intf = ret;

	ret = usb_interface_id(c, f);
	if (ret < 0)
		goto error;
	uvc_streaming_intf_alt0.bInterfaceNumber = ret;
	uvc->streaming_intf = ret;

	/* Copy descriptors */
	f->fs_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_FULL);
	if (IS_ERR(f->fs_descriptors)) {
		ret = PTR_ERR(f->fs_descriptors);
		f->fs_descriptors = NULL;
		goto error;
	}
	if (gadget_is_dualspeed(cdev->gadget)) {
		f->hs_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_HIGH);
		if (IS_ERR(f->hs_descriptors)) {
			ret = PTR_ERR(f->hs_descriptors);
			f->hs_descriptors = NULL;
			goto error;
		}
	}
	if (gadget_is_superspeed(c->cdev->gadget)) {
		f->ss_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_SUPER);
		if (IS_ERR(f->ss_descriptors)) {
			ret = PTR_ERR(f->ss_descriptors);
			f->ss_descriptors = NULL;
			goto error;
		}
	}

	/* Preallocate control endpoint request. */
	uvc->control_req = usb_ep_alloc_request(cdev->gadget->ep0, GFP_KERNEL);
	uvc->control_buf = kmalloc(UVC_MAX_REQUEST_SIZE, GFP_KERNEL);
	if (uvc->control_req == NULL || uvc->control_buf == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	uvc->control_req->buf = uvc->control_buf;
	uvc->control_req->complete = uvc_function_ep0_complete;
	uvc->control_req->context = uvc;

	if (v4l2_device_register(&cdev->gadget->dev, &uvc->v4l2_dev)) {
		rlxprintk(RLX_TRACE_ERROR, "v4l2_device_register failed\n");
		goto error;
	}

	/* Preallocate control endpoint request. */
	uvc->int_req = usb_ep_alloc_request(uvc->video.csep, GFP_KERNEL);
	uvc->int_buf = kmalloc(UVC_MAX_REQUEST_SIZE, GFP_KERNEL);
	if (uvc->int_req == NULL || uvc->int_buf == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	uvc->int_req->buf = uvc->int_buf;
	uvc->int_req->complete = uvc_function_epc_complete;
	uvc->int_req->context = uvc;


	/* Initialise video. */
	ret = uvcg_video_init(&uvc->video);
	if (ret < 0)
		goto error;

	/* Register a V4L2 device. */
	ret = uvc_register_video(uvc);
	if (ret < 0) {
		rlxprintk(RLX_TRACE_ERROR, "Unable to register video device\n");
		goto error;
	}
	return 0;

error:
	v4l2_device_unregister(&uvc->v4l2_dev);

	if (uvc->control_req)
		usb_ep_free_request(cdev->gadget->ep0, uvc->control_req);
	kfree(uvc->control_buf);

	usb_free_all_descriptors(f);
	return ret;
}

/* --------------------------------------------------------------------------
 * USB gadget function
 */

static void uvc_free_inst(struct usb_function_instance *f)
{
	struct f_uvc_opts *opts = fi_to_f_uvc_opts(f);

	mutex_destroy(&opts->lock);
	kfree(opts);
}

static struct usb_function_instance *uvc_alloc_inst(void)
{
	struct f_uvc_opts *opts;
	struct uvc_camera_terminal_descriptor *cd;
	struct uvc_processing_unit_descriptor *pd;
	struct uvc_output_terminal_descriptor *od;
	struct uvc_color_matching_descriptor *md;
	struct uvc_descriptor_header **ctl_cls;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.free_func_inst = uvc_free_inst;
	mutex_init(&opts->lock);

	cd = &opts->uvc_camera_terminal;
	cd->bLength			= UVC_DT_CAMERA_TERMINAL_SIZE(3);
	cd->bDescriptorType		= USB_DT_CS_INTERFACE;
	cd->bDescriptorSubType		= UVC_VC_INPUT_TERMINAL;
	cd->bTerminalID			= 1;
	cd->wTerminalType		= cpu_to_le16(0x0201);
	cd->bAssocTerminal		= 0;
	cd->iTerminal			= 0;
	cd->wObjectiveFocalLengthMin	= cpu_to_le16(0);
	cd->wObjectiveFocalLengthMax	= cpu_to_le16(0);
	cd->wOcularFocalLength		= cpu_to_le16(0);
	cd->bControlSize		= 3;
	cd->bmControls[0]		= 2;
	cd->bmControls[1]		= 0;
	cd->bmControls[2]		= 0;

	pd = &opts->uvc_processing;
	pd->bLength			= UVC_DT_PROCESSING_UNIT_SIZE(2);
	pd->bDescriptorType		= USB_DT_CS_INTERFACE;
	pd->bDescriptorSubType		= UVC_VC_PROCESSING_UNIT;
	pd->bUnitID			= 2;
	pd->bSourceID			= 1;
	pd->wMaxMultiplier		= cpu_to_le16(16*1024);
	pd->bControlSize		= 2;
	pd->bmControls[0]		= 1;
	pd->bmControls[1]		= 0;
	pd->iProcessing			= 0;

	od = &opts->uvc_output_terminal;
	od->bLength			= UVC_DT_OUTPUT_TERMINAL_SIZE;
	od->bDescriptorType		= USB_DT_CS_INTERFACE;
	od->bDescriptorSubType		= UVC_VC_OUTPUT_TERMINAL;
	od->bTerminalID			= 3;
	od->wTerminalType		= cpu_to_le16(0x0101);
	od->bAssocTerminal		= 0;
	od->bSourceID			= 2;
	od->iTerminal			= 0;

	md = &opts->uvc_color_matching;
	md->bLength			= UVC_DT_COLOR_MATCHING_SIZE;
	md->bDescriptorType		= USB_DT_CS_INTERFACE;
	md->bDescriptorSubType		= UVC_VS_COLORFORMAT;
	md->bColorPrimaries		= 1;
	md->bTransferCharacteristics	= 1;
	md->bMatrixCoefficients		= 4;

	/* Prepare fs control class descriptors for configfs-based gadgets */
	ctl_cls = opts->uvc_fs_control_cls;
	ctl_cls[0] = NULL;	/* assigned elsewhere by configfs */
	ctl_cls[1] = (struct uvc_descriptor_header *)cd;
	ctl_cls[2] = (struct uvc_descriptor_header *)pd;
	ctl_cls[3] = (struct uvc_descriptor_header *)od;
	ctl_cls[4] = NULL;	/* NULL-terminate */
	opts->fs_control =
		(const struct uvc_descriptor_header * const *)ctl_cls;

	/* Prepare hs control class descriptors for configfs-based gadgets */
	ctl_cls = opts->uvc_ss_control_cls;
	ctl_cls[0] = NULL;	/* assigned elsewhere by configfs */
	ctl_cls[1] = (struct uvc_descriptor_header *)cd;
	ctl_cls[2] = (struct uvc_descriptor_header *)pd;
	ctl_cls[3] = (struct uvc_descriptor_header *)od;
	ctl_cls[4] = NULL;	/* NULL-terminate */
	opts->ss_control =
		(const struct uvc_descriptor_header * const *)ctl_cls;

	opts->streaming_interval = 1;
	opts->streaming_maxpacket = 1024;

	return &opts->func_inst;
}

static void uvc_free(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct f_uvc_opts *opts = container_of(f->fi, struct f_uvc_opts,
					       func_inst);
	--opts->refcnt;
	kfree(uvc);
}

static void uvc_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct uvc_device *uvc = to_uvc(f);

	INFO(cdev, "%s\n", __func__);

	video_unregister_device(&uvc->vdev);
	v4l2_device_unregister(&uvc->v4l2_dev);

	usb_ep_free_request(cdev->gadget->ep0, uvc->control_req);
	kfree(uvc->control_buf);

	usb_free_all_descriptors(f);
}

static struct usb_function *uvc_alloc(struct usb_function_instance *fi)
{
	struct uvc_device *uvc;
	struct f_uvc_opts *opts;
	struct uvc_descriptor_header **strm_cls;

	uvc = kzalloc(sizeof(*uvc), GFP_KERNEL);
	if (uvc == NULL)
		return ERR_PTR(-ENOMEM);

	mutex_init(&uvc->video.mutex);
	uvc->state = UVC_STATE_DISCONNECTED;
	opts = fi_to_f_uvc_opts(fi);

	mutex_lock(&opts->lock);
	if (opts->uvc_fs_streaming_cls) {
		strm_cls = opts->uvc_fs_streaming_cls;
		opts->fs_streaming =
			(const struct uvc_descriptor_header * const *)strm_cls;
	}
	if (opts->uvc_hs_streaming_cls) {
		strm_cls = opts->uvc_hs_streaming_cls;
		opts->hs_streaming =
			(const struct uvc_descriptor_header * const *)strm_cls;
	}
	if (opts->uvc_ss_streaming_cls) {
		strm_cls = opts->uvc_ss_streaming_cls;
		opts->ss_streaming =
			(const struct uvc_descriptor_header * const *)strm_cls;
	}

	uvc->desc.fs_control = opts->fs_control;
	uvc->desc.ss_control = opts->ss_control;
	uvc->desc.fs_streaming = opts->fs_streaming;
	uvc->desc.hs_streaming = opts->hs_streaming;
	uvc->desc.ss_streaming = opts->ss_streaming;
	++opts->refcnt;
	mutex_unlock(&opts->lock);

	/* Register the function. */
	uvc->func.name = "uvc";
	uvc->func.bind = uvc_function_bind;
	uvc->func.unbind = uvc_unbind;
	uvc->func.get_alt = uvc_function_get_alt;
	uvc->func.set_alt = uvc_function_set_alt;
	uvc->func.disable = uvc_function_disable;
	uvc->func.setup = uvc_function_setup;
	uvc->func.free_func = uvc_free;
	uvc->func.bind_deactivated = true;
	uvc->func.filteroutdata = uvc_function_filteroutdata;

	return &uvc->func;
}


