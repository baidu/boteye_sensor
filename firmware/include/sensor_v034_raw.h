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

#ifndef FIRMWARE_INCLUDE_SENSOR_V034_RAW_H_
#define FIRMWARE_INCLUDE_SENSOR_V034_RAW_H_

#include <cyu3types.h>

#define DVK_SDK_PORT
#define HD720P
#define UVC

typedef unsigned char  BYTE;
typedef unsigned int  uint;

#define V034_PLATFORM_TAHOE

#define SENSOR_ADDR_WR    0x90
#define SENSOR_ADDR_RD    0x91

#define L_SENSOR_ADDR_WR 0x98
#define L_SENSOR_ADDR_RD 0x99
#define R_SENSOR_ADDR_WR 0x90
#define R_SENSOR_ADDR_RD 0x91

#define V034_AUTO_EXPOSURE_MODE        8
#define V034_MANUAL_EXPOSURE_MODE      1

#define IMGS_CHIP_ID                   (0x1324)

/*****************************************************************************
**                                          function declaration
******************************************************************************/
CyU3PReturnStatus_t V034_SensorWrite2B(uint8_t SlaveAddr, uint8_t HighAddr, uint8_t LowAddr,
                                       uint8_t HighData, uint8_t LowData);
CyU3PReturnStatus_t V034_SensorWrite(uint8_t SlaveAddr, uint8_t HighAddr, uint8_t LowAddr,
                                     uint8_t count, uint8_t *buf);
CyU3PReturnStatus_t V034_SensorRead2B(uint8_t SlaveAddr, uint8_t HighAddr, uint8_t LowAddr,
                                      uint8_t *buf);
CyU3PReturnStatus_t V034_SensorRead(uint8_t SlaveAddr, uint8_t HighAddr, uint8_t LowAddr,
                                    uint8_t count, uint8_t *buf);
CyU3PReturnStatus_t V034_RegisterWrite(uint8_t HighAddr, uint8_t LowAddr, uint8_t HighData,
                                       uint8_t LowData);
void V034_delay(volatile int time);
void DRV_imgsSetRegs(void);
void V034_sensor_init(void);
void V034_soft_reset(uint8_t SlaveAddr);
BYTE V034_I2C_BUS_TEST(void);
void V034_stream_start(uint8_t SlaveAddr);
void V034_stream_stop(uint8_t SlaveAddr);
void dump_sensor_registers(uint8_t SlaveAddr);
uint16_t V034_RegisterRead(uint8_t HighAddr, uint8_t LowAddr);

/* Function    : V034_SensorGetBrightness
   Description : Get the current brightness setting from the MT9M114 sensor.
   Parameters  : None
 */
extern uint8_t V034_SensorGetBrightness(void);

/* Function    : V034_SensorSetBrightness
   Description : Set the desired brightness setting on the MT9M114 sensor.
   Parameters  :
                 brightness - Desired brightness level.
 */
extern void V034_SensorSetBrightness(uint8_t brightness);
/* Function    : V034_SensorGetContrast
   Description : Get the current Contrast setting from the MT9M114 sensor.
   Parameters  : None
 */
extern uint8_t V034_SensorGetContrast(void);

/* Function    : V034_SensorSetContrast
   Description : Set the desired Contrast setting on the MT9M114 sensor.
   Parameters  :
                 Contrast - Desired Contrast level.
 */
extern void V034_SensorSetContrast(uint8_t Contrast);

/*HUE Control*/
extern uint8_t V034_SensorGetHUE(void);
extern void V034_SensorSetHUE(uint8_t HUE);
extern uint8_t V034_SensorGetAWB(void);
extern void V034_SensorSetAWB(uint8_t enable);

/*AWB TMP Control*/
extern uint16_t V034_SensorGetAWB_TMP(void);
extern void V034_SensorSetAWB_TMP(uint16_t tmp);

/*Saturation Control*/
extern uint8_t V034_SensorGetSaturation(void);
extern void V034_SensorSetSaturation(uint8_t tmp);

/*Sharpness Control*/
extern void V034_SensorSetSharpness(uint8_t tmp);
extern uint8_t V034_SensorGetSharpness(void);

/*Gamma Control*/
extern void V034_SensorSetGamma(uint8_t tmp);
extern uint8_t V034_SensorGetGamma(void);

/*Backlight Control*/
extern uint8_t V034_SensorGetBacklight(void);
void V034_SensorSetBacklight(uint8_t tmp);

/*AE Mode Control*/
extern uint8_t V034_SensorGetAEMode(void);
extern void V034_SensorSetAEMode(uint8_t enable);

/*Exposuretime Control*/
extern uint16_t V034_SensorGetExposuretime(void);
void V034_SensorSetExposuretime(uint16_t tmp);

/*
   Power Line Frequency Control
 */
extern uint8_t V034_SensorGetPowerLineFreq(void);
extern void V034_SensorSetPowerLineFreq(uint8_t enable);

/*
  Gain Control
 */
extern uint8_t V034_SensorGetGain(void);
extern void V034_SensorSetGain(uint8_t tmp);

#endif  // FIRMWARE_INCLUDE_SENSOR_V034_RAW_H_
