/*!
    \file  usbd_msc_core.c
    \brief USB MSC device class core functions

    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "usbd_msc_mem.h"
#include "usbd_msc_core.h"
#include "usbd_msc_bbb.h"
#include "usbd_enum.h"

#define USBD_VID 0x28E9U
#define USBD_PID 0x028FU

static uint8_t msc_core_init(usb_dev * pudev, uint8_t config_index);
static uint8_t msc_core_deinit(usb_dev * pudev, uint8_t config_index);
static uint8_t msc_core_req(usb_dev * pudev, usb_req * req);
static uint8_t msc_core_in(usb_dev * pudev, uint8_t ep_num);
static uint8_t msc_core_out(usb_dev * pudev, uint8_t ep_num);

usb_class_core msc_class = {.init = msc_core_init,
                            .deinit = msc_core_deinit,

                            .req_proc = msc_core_req,

                            .data_in = msc_core_in,
                            .data_out = msc_core_out};

/* note: it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
const usb_desc_dev msc_dev_desc = {
    .header = {.bLength = USB_DEV_DESC_LEN, .bDescriptorType = USB_DESCTYPE_DEV},
    .bcdUSB = 0x0200U,
    .bDeviceClass = 0x00U,
    .bDeviceSubClass = 0x00U,
    .bDeviceProtocol = 0x00U,
    .bMaxPacketSize0 = USB_FS_EP0_MAX_LEN,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100U,
    .iManufacturer = STR_IDX_MFC,
    .iProduct = STR_IDX_PRODUCT,
    .iSerialNumber = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM};

/* USB device configuration descriptor */
const usb_desc_config_set msc_config_desc = {
    .config = {.header = {.bLength = sizeof(usb_desc_config),
                          .bDescriptorType = USB_DESCTYPE_CONFIG},
               .wTotalLength = USB_MSC_CONFIG_DESC_SIZE,
               .bNumInterfaces = 0x01U,
               .bConfigurationValue = 0x01U,
               .iConfiguration = 0x00U,
               .bmAttributes = 0xC0U,
               .bMaxPower = 0x32U},

    .msc_itf = {.header = {.bLength = sizeof(usb_desc_itf), .bDescriptorType = USB_DESCTYPE_ITF},
                .bInterfaceNumber = 0x00U,
                .bAlternateSetting = 0x00U,
                .bNumEndpoints = 0x02U,
                .bInterfaceClass = USB_CLASS_MSC,
                .bInterfaceSubClass = USB_MSC_SUBCLASS_SCSI,
                .bInterfaceProtocol = USB_MSC_PROTOCOL_BBB,
                .iInterface = 0x00U},

    .msc_epin = {.header = {.bLength = sizeof(usb_desc_ep), .bDescriptorType = USB_DESCTYPE_EP},
                 .bEndpointAddress = MSC_IN_EP,
                 .bmAttributes = USB_EP_ATTR_BULK,
                 .wMaxPacketSize = MSC_EPIN_SIZE,
                 .bInterval = 0x00U},

    .msc_epout = {.header = {.bLength = sizeof(usb_desc_ep), .bDescriptorType = USB_DESCTYPE_EP},
                  .bEndpointAddress = MSC_OUT_EP,
                  .bmAttributes = USB_EP_ATTR_BULK,
                  .wMaxPacketSize = MSC_EPOUT_SIZE,
                  .bInterval = 0x00U}};

/* USB language ID descriptor */
const usb_desc_LANGID usbd_language_id_desc = {
    .header = {.bLength = sizeof(usb_desc_LANGID), .bDescriptorType = USB_DESCTYPE_STR},
    .wLANGID = ENG_LANGID};

/* usb string descriptor */
void * const usbd_msc_strings[] = {[STR_IDX_LANGID] = (uint8_t *)&usbd_language_id_desc,
                                   [STR_IDX_MFC] = USBD_STRING_DESC("GigaDevice"),
                                   [STR_IDX_PRODUCT] = USBD_STRING_DESC("GD32 USB MSC in FS Mode"),
                                   [STR_IDX_SERIAL] =
                                       USBD_STRING_DESC("GD32VF103-V1.0.0-ka4db5ec")};

static uint8_t usbd_msc_maxlun = 0;

/*!
    \brief      initialize the MSC device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t msc_core_init(usb_dev * pudev, uint8_t config_index) {
    /* configure MSC Tx endpoint */
    usbd_ep_setup(pudev, &(msc_config_desc.msc_epin));

    /* configure MSC Rx endpoint */
    usbd_ep_setup(pudev, &(msc_config_desc.msc_epout));

    /* init the BBB layer */
    msc_bbb_init(pudev);

    return USBD_OK;
}

/*!
    \brief      de-initialize the MSC device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t msc_core_deinit(usb_dev * pudev, uint8_t config_index) {
    /* clear MSC endpoints */
    usbd_ep_clear(pudev, MSC_IN_EP);
    usbd_ep_clear(pudev, MSC_OUT_EP);

    /* un-init the BBB layer */
    msc_bbb_deinit(pudev);

    return USBD_OK;
}

/*!
    \brief      handle the MSC class-specific and standard requests
    \param[in]  pudev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t msc_core_req(usb_dev * pudev, usb_req * req) {
    usb_transc * transc = &pudev->dev.transc_in[0];

    switch (req->bRequest) {
    case BBB_GET_MAX_LUN:
        if ((0U == req->wValue) && (1U == req->wLength) &&
            (0x80U == (req->bmRequestType & 0x80U))) {
            usbd_msc_maxlun = usbd_mem_fops->mem_maxlun();

            transc->xfer_buf = &usbd_msc_maxlun;
            transc->remain_len = 1;
        } else {
            return USBD_FAIL;
        }
        break;

    case BBB_RESET:
        if ((0U == req->wValue) && (0U == req->wLength) &&
            (0x80U != (req->bmRequestType & 0x80U))) {
            msc_bbb_reset(pudev);
        } else {
            return USBD_FAIL;
        }
        break;

    case USB_CLEAR_FEATURE:
        msc_bbb_clrfeature(pudev, (uint8_t)req->wIndex);
        break;

    default:
        return USBD_FAIL;
    }

    return USBD_OK;
}

/*!
    \brief      handle data in stage
    \param[in]  pudev: pointer to USB device instance
    \param[in]  ep_num: the endpoint number
    \param[out] none
    \retval     none
*/
static uint8_t msc_core_in(usb_dev * pudev, uint8_t ep_num) {
    if ((MSC_IN_EP & 0x7FU) == ep_num) {
        msc_bbb_data_in(pudev, ep_num);
    }

    return USBD_OK;
}

/*!
    \brief      handle data out stage
    \param[in]  pudev: pointer to USB device instance
    \param[in]  ep_num: the endpoint number
    \param[out] none
    \retval     none
*/
static uint8_t msc_core_out(usb_dev * pudev, uint8_t ep_num) {
    if (MSC_OUT_EP == ep_num) {
        msc_bbb_data_out(pudev, ep_num);
    }

    return USBD_OK;
}
