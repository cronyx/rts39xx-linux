#ifndef _ANDROID_UVC_H
#define _ANDROID_UVC_H

int rtsx_uvc_function_bind(struct usb_configuration *c);
int rtsx_uvc_function_init(void);
void rtsx_uvc_function_cleanup(void);
void rtsx_uvc_function_enable(void);
void bypass_usb_pullup(void);
int dwc_otg_pcd_wait_in_nak(void);

#endif
