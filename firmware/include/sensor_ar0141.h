/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
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

#ifndef FIRMWARE_INCLUDE_SENSOR_AR0141_H_
#define FIRMWARE_INCLUDE_SENSOR_AR0141_H_

#include <cyu3types.h>
#define AR0141_ADDR_WR    0x20
#define AR0141_ADDR_RD    0x21

#define L_AR0141_ADDR_WR  0x20
#define L_AR0141_ADDR_RD  0x21
#define R_AR0141_ADDR_WR  0x30
#define R_AR0141_ADDR_RD  0x31

// AR0141 register address map
#define GREEN1_GAIN       0x3056
#define BLUE_GAIN         0x3058
#define RED_GAIN          0x305A
#define GREEN2_GAIN       0x305C
#define GLOBAL_GAIN       0x305E
#define ANALOG_GAIN       0x3060

extern CyU3PReturnStatus_t AR0141_RegisterWrite(uint8_t HighAddr, uint8_t LowAddr,
                                                uint8_t HighData, uint8_t LowData);
extern uint16_t AR0141_RegisterRead(uint8_t HighAddr, uint8_t LowAddr);
extern void AR0141_sensor_init(void);
extern void AR0141_stream_start(uint8_t SlaveAddr);
extern void AR0141_stream_stop(uint8_t SlaveAddr);
#endif  // FIRMWARE_INCLUDE_SENSOR_AR0141_H_
