/**
  ******************************************************************************
  * @file    usb_desc.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptor Header for Joystick Mouse Demo
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DESC_H
#define __USB_DESC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define HID_DESCRIPTOR_TYPE                     0x21
#define JOYSTICK_SIZ_HID_DESC                   0x09
#define JOYSTICK_OFF_HID_DESC                   0x12

#define HID_DESCRIPTOR_TYPE                     0x21
#define JOYSTICK_SIZ_HID_DESC                   0x09
#define KeyBoard_OFF_HID_DESC                   0x12
#define Mouse_OFF_HID_DESC                 		41

#define JOYSTICK_SIZ_DEVICE_DESC                18
#define JOYSTICK_SIZ_CONFIG_DESC                41
#define KeyBoard_SIZ_REPORT_DESC                35
#define Mouse_SIZ_REPORT_DESC                   86
//157 171
#define JOYSTICK_SIZ_STRING_LANGID              4
#define JOYSTICK_SIZ_STRING_VENDOR              26
#define JOYSTICK_SIZ_STRING_PRODUCT             20

#define JOYSTICK_SIZ_STRING_SERIAL              24

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

#define REPORT_ID								1
#define REPORT_IN_SIZE							3					/* for PC */
#define REPORT_OUT_SIZE							3					/* for PC */
#define REPORT_IN_SIZE_WITH_ID					REPORT_IN_SIZE + 1 	/* for PC */
#define REPORT_OUT_SIZE_WITH_ID					REPORT_OUT_SIZE + 1 /* for PC */

/* Exported functions ------------------------------------------------------- */
extern const uint8_t Joystick_DeviceDescriptor[JOYSTICK_SIZ_DEVICE_DESC];
extern const uint8_t Joystick_ConfigDescriptor[JOYSTICK_SIZ_CONFIG_DESC];
extern const uint8_t KeyBoard_ReportDescriptor[KeyBoard_SIZ_REPORT_DESC];
extern const uint8_t Mouse_ReportDescriptor[Mouse_SIZ_REPORT_DESC];
extern const uint8_t Joystick_StringLangID[JOYSTICK_SIZ_STRING_LANGID];
extern const uint8_t Joystick_StringVendor[JOYSTICK_SIZ_STRING_VENDOR];
extern const uint8_t Joystick_StringProduct[JOYSTICK_SIZ_STRING_PRODUCT];
extern uint8_t Joystick_StringSerial[JOYSTICK_SIZ_STRING_SERIAL];


#endif /* __USB_DESC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
