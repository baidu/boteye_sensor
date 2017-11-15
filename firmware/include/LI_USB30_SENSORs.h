/******************************************************************************
 * Copyright 2017 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef FIRMWARE_INCLUDE_LI_USB30_SENSORS_H_
#define FIRMWARE_INCLUDE_LI_USB30_SENSORS_H_

#define LI_USB30_UVC_FEATURE_DEBUG_LEVEL 4

/************************************************************************************************
 * Image sensor selection,LI-CAM-M114 by default
 ***********************************************************************************************/

#define LI_USB30_SENSOR_V034_RAW
#define SENSOR_GPIO_CONTROL

/*Currently multi res switch is supportted only in USB3.0 Descriptor*/
#define MULTI_RESOLUTION_SUPPORT_U30

#ifdef MULTI_RESOLUTION_SUPPORT_U30
// Resolution supportted,Manually change Descriptor size:TOTAL_SIZE_DESCRIPTOR and
// TOTAL_SIZE_CS_DESCRIPTOR,also ADD or SUBSTRUCT the details for frame descriptor in cyfxuvcdscr.c
#define RES_NUM                   1
#define SUPPORTED_FRAMERATE_NUM   4  // 60,30,15,10FPS
#endif
/* Configuration Descriptor Type */
#define CONFIGURATION_DESCRIPTOR_LENGTH             0x9
/* Interface Association Descriptor */
#define INTERFACE_ASSOCIATION__DESCRIPTOR_LENGTH    0x8
/* Standard Video Control Interface Descriptor */
#define STD_VIDEO_CTR_INTERFACE_DESCRIPTOR_LENGTH   0x9
/* Class specific VC Interface Header Descriptor */
#define CS_VS_INTERFACE_HEADER_DESCRIPTOR_LENGTH    0x0D
/* Input (Camera) Terminal Descriptor */
#define INPUT_TERMINAL_DESCRIPTOR_LENGTH            0x12
/* Processing Unit Descriptor */
#define PROCESS_UNIT_DESCRIPTOR_LENGTH              0x0B
/* Extension Unit Descriptor */
#define EXTENSION_UNIT_DESCRIPTOR_LENGTH            0x1C
/* Output Terminal Descriptor */
#define OUTPUT_TERMINAL_DESCRIPTOR_LENGTH           0x09
/* Video Control Status Interrupt Endpoint Descriptor */
#define VC_STATUS_INT_ENDPOINT_DESCRIPTOR_LENGTH    0x07
/* Super Speed Endpoint Companion Descriptor */
#define SS_ENDPOINT_COMPANION_DESCRIPTOR_LENGTH     0x06
/* Class Specific Interrupt Endpoint Descriptor */
#define CS_INT_ENDPOINT_DESCRIPTOR_LENGTH           0x05
/* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
#define STD_VS_INTERFACE_DESCRIPTOR_LENGTH          0x09
/* Class-specific Video Streaming Input Header Descriptor */
#define CS_VS_INPUT_HEADER_DESCRIPTOR_LENGTH        0x0E
/* Class specific Uncompressed VS format descriptor */
#define CS_UNCOMPRESSED_VS_FORMAT_DESCRIPTOR_LENGTH 0x1B
/* Class specific Uncompressed VS frame descriptoR*/
#define CS_UNCOMPRESSED_VS_FRAME_DESCRIPTOR_LENGTH  (26 + 4 * SUPPORTED_FRAMERATE_NUM)
/* Endpoint Descriptor for BULK Streaming Video Data */
#define ENDPOINT_DESCRIPTOR_BULK_STREAMING_VIDEO_DATA_LENGTH  0x07
/* Super Speed Endpoint Companion Descriptor */
#define SS_ENDPOINT_COMPANION_DESCRIPTOR_LENGTH     0x06

#define TOTAL_SIZE_DESCRIPTOR CONFIGURATION_DESCRIPTOR_LENGTH +\
INTERFACE_ASSOCIATION__DESCRIPTOR_LENGTH +\
STD_VIDEO_CTR_INTERFACE_DESCRIPTOR_LENGTH +\
CS_VS_INTERFACE_HEADER_DESCRIPTOR_LENGTH+\
INPUT_TERMINAL_DESCRIPTOR_LENGTH+\
PROCESS_UNIT_DESCRIPTOR_LENGTH+\
EXTENSION_UNIT_DESCRIPTOR_LENGTH+\
OUTPUT_TERMINAL_DESCRIPTOR_LENGTH+\
VC_STATUS_INT_ENDPOINT_DESCRIPTOR_LENGTH+\
SS_ENDPOINT_COMPANION_DESCRIPTOR_LENGTH+\
CS_INT_ENDPOINT_DESCRIPTOR_LENGTH+\
STD_VS_INTERFACE_DESCRIPTOR_LENGTH+\
CS_VS_INPUT_HEADER_DESCRIPTOR_LENGTH+\
CS_UNCOMPRESSED_VS_FORMAT_DESCRIPTOR_LENGTH+\
CS_UNCOMPRESSED_VS_FRAME_DESCRIPTOR_LENGTH+\
ENDPOINT_DESCRIPTOR_BULK_STREAMING_VIDEO_DATA_LENGTH+\
SS_ENDPOINT_COMPANION_DESCRIPTOR_LENGTH

#define TOTAL_SIZE_CS_DESCRIPTOR CS_VS_INPUT_HEADER_DESCRIPTOR_LENGTH+\
CS_UNCOMPRESSED_VS_FORMAT_DESCRIPTOR_LENGTH+\
CS_UNCOMPRESSED_VS_FRAME_DESCRIPTOR_LENGTH


#define TOTAL_SIZE_CS_DESCRIPTOR_EX CS_VS_INTERFACE_HEADER_DESCRIPTOR_LENGTH+\
INPUT_TERMINAL_DESCRIPTOR_LENGTH+\
PROCESS_UNIT_DESCRIPTOR_LENGTH+\
EXTENSION_UNIT_DESCRIPTOR_LENGTH+\
OUTPUT_TERMINAL_DESCRIPTOR_LENGTH
#endif  // FIRMWARE_INCLUDE_LI_USB30_SENSORS_H_
