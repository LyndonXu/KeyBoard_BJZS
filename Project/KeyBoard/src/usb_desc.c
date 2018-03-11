/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptors for Joystick Mouse Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USB Standard Device Descriptor */
const uint8_t Joystick_DeviceDescriptor[JOYSTICK_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize 64*/
    0x83,                       /*idVendor (0x0483)*/
    0x04,
    0x1F,                       /*idProduct = 0x571F*/
    0x57,
    0x00,                       /*bcdDevice rel. 1.00*/
    0x01,
    1,                          /*Index of string descriptor describing
                                                  manufacturer */
    2,                          /*Index of string descriptor describing
                                                 product*/
    3,                          /*Index of string descriptor describing the
                                                 device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* Joystick_DeviceDescriptor */


const uint8_t Joystick_ConfigDescriptor[JOYSTICK_SIZ_CONFIG_DESC] =
{
    0x09, /* bLength: Configuation Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    JOYSTICK_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /*bNumInterfaces: 1 interface*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
                                     the configuration*/
    0xE0,         /*bmAttributes: bus powered */ //0xE0
    0xC8,         /*MaxPower 400 mA: this current is used for detecting Vbus*/
    /* 09 */

    /************** Descriptor of Joystick KeyBoard interface ****************/
    0x09,         /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    0x00,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /* 18 */
    /******************** Descriptor of Joystick KeyBoard HID ********************/
    0x09,         /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x00,         /*bcdHID: HID Class Spec release number*/
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    KeyBoard_SIZ_REPORT_DESC,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /* 27 */
    /******************** Descriptor of Joystick KeyBoard endpoint ********************/
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x81,          /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    0x40,          /*wMaxPacketSize: 64 Byte max */
    0x00,
    0x0A,          /*bInterval: Polling Interval (10 ms)*/
    /* 34 */
    /******************** Descriptor of Joystick KeyBoard endpoint ********************/
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x01,          /*bEndpointAddress: Endpoint Address (OUT)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    0x40,          /*wMaxPacketSize: 64 Byte max */
    0x00,
    0x0A,          /*bInterval: Polling Interval (10 ms)*/
    /* 41 */

};  

const uint8_t KeyBoard_ReportDescriptor[KeyBoard_SIZ_REPORT_DESC] =
{
	/**35**/
	0x06, 0xFF, 0x00,     		   /* USAGE_PAGE (Vendor Page: 0xFF00) */
	0x09, 0x01,                    // USAGE (Consumer Control)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x85, REPORT_ID,               //   REPORT_ID (1)
	0x19, 0x01,                    //   USAGE_MINIMUM (1)
	0x29, 0x08,    		           //   USAGE_MAXIMUM (8)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x95, REPORT_IN_SIZE,          //   REPORT_COUNT (19)
	0x75, 0x08,                    //   REPORT_SIZE (1 Byte)
	0x81, 0x02,                    //   INPUT (Data,Ary,Abs)
	0x19, 0x01,                    //   USAGE_MINIMUM (1)
	0x29, 0x08,    		           //   USAGE_MAXIMUM (8)
	0x95, REPORT_OUT_SIZE,         //   REPORT_COUNT (9)
	0x75, 0x08,                    //   REPORT_SIZE (1 Byte)
	0x91, 0x02,                    //   OUTPUT (Data,Ary,Abs)
	0xc0                           // END_COLLECTION 

}; /* Joystick_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t Joystick_StringLangID[JOYSTICK_SIZ_STRING_LANGID] =
  {
    JOYSTICK_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t Joystick_StringVendor[JOYSTICK_SIZ_STRING_VENDOR] =
  {
    JOYSTICK_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "STMicroelectronics" */
	0x29, 0x59, //天
	0x25, 0x6d, //津
	0x02, 0x5e, //市
	0xD5, 0x71, //燕诺安\u71d5\u8bfa\u5b89
	0xFA, 0x8B, //
	0x89, 0x5B, //
	0xd1, 0x79, //科
	0x80, 0x62, //技
	0x09, 0x67, //有
	0x50, 0x96, //限
	0x6c, 0x51, //公
	0xf8, 0x53  //司
  };

const uint8_t Joystick_StringProduct[JOYSTICK_SIZ_STRING_PRODUCT] =
  {
    JOYSTICK_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	0xD5, 0x71, //燕诺安\u71d5\u8bfa\u5b89
	0xFA, 0x8B, //
	0x89, 0x5B, //
	0x5A, 0x86, //虚
	0xDF, 0x62, //拟
	0x35, 0x75, //电
	0x11, 0x81, //脑
	0x2e, 0x95, //键
	0xd8, 0x76  //盘
  };
uint8_t Joystick_StringSerial[JOYSTICK_SIZ_STRING_SERIAL] =
  {
    JOYSTICK_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	0xD5, 0x71, //燕诺安\u71d5\u8bfa\u5b89
	0xFA, 0x8B, //
	0x89, 0x5B, //
	0x2e, 0x95, //键
	0xd8, 0x76, //盘
	0xfb, 0x7c, //系
	0x17, 0x52, //列
	0x56, 0x00, //V
	0x31, 0x00, //1
	0x2e, 0x00, //.
	0x33, 0x00  //0
  };

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
