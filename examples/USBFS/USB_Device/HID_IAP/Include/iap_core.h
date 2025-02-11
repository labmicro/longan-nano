/*!
    \file  iap_core.h
    \brief the header file of IAP driver

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

#ifndef IAP_CORE_H
#define IAP_CORE_H

#include "usbd_enum.h"
#include "usb_ch9_std.h"
#include "usbd_transc.h"

#define IAP_CONFIG_DESC_SIZE     9
#define USB_SERIAL_STRING_SIZE   0x06

#define DEVICE_ID                (0x40022100)

#define IAP_REPORT_DESC_SIZE     35
#define IAP_CONFIG_SET_DESC_SIZE 41

#define HID_DESCTYPE             0x21
#define HID_REPORT_DESCTYPE      0x22

#define GET_REPORT               0x01
#define GET_IDLE                 0x02
#define GET_PROTOCOL             0x03
#define SET_REPORT               0x09
#define SET_IDLE                 0x0A
#define SET_PROTOCOL             0x0B
#define NO_CMD                   0xFF

/* special commands with download request */
#define IAP_OPTION_BYTE1   0x01
#define IAP_ERASE          0x02
#define IAP_DNLOAD         0x03
#define IAP_LEAVE          0x04
#define IAP_GETBIN_ADDRESS 0x05
#define IAP_OPTION_BYTE2   0x06

typedef void (*pAppFunction)(void);

#pragma pack(1)

typedef struct {
    usb_desc_header
        header; /*!< regular descriptor header containing the descriptor's type and length */

    uint16_t bcdHID;      /*!< BCD encoded version that the HID descriptor and device complies to */
    uint8_t bCountryCode; /*!< country code of the localized device, or zero if universal */
    uint8_t bNumDescriptors;    /*!< total number of HID report descriptors for the interface */
    uint8_t bDescriptorType;    /*!< type of HID report */
    uint16_t wDescriptorLength; /*!< length of the associated HID report descriptor, in bytes */
} usb_hid_descriptor_hid_struct;

#pragma pack()

typedef struct {
    usb_desc_config Config;
    usb_desc_itf HID_Interface;
    usb_hid_descriptor_hid_struct HID_VendorHID;
    usb_desc_ep HID_ReportINEndpoint;
    usb_desc_ep HID_ReportOUTEndpoint;
} usb_descriptor_configuration_set_struct;

extern void * const usbd_strings[USB_STRING_COUNT];
extern const usb_desc_dev device_descripter;
extern const usb_descriptor_configuration_set_struct configuration_descriptor;

extern usb_class_core usbd_hid_cb;

/* function declarations */
/* initialize the HID device */
uint8_t iap_init(usb_dev * pudev, uint8_t config_index);
/* de-initialize the HID device */
uint8_t iap_deinit(usb_dev * pudev, uint8_t config_index);
/* handle the HID class-specific requests */
uint8_t iap_req_handler(usb_dev * pudev, usb_req * req);
/* handle data stage */
uint8_t iap_data_handler(usb_dev * pudev, uint8_t ep_id);

/* send iap report */
uint8_t iap_report_send(usb_dev * pudev, uint8_t * report, uint16_t Len);

#endif /* IAP_CORE_H */
