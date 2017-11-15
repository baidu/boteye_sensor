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

#ifndef FIRMWARE_INCLUDE_FX3_BSP_H_
#define FIRMWARE_INCLUDE_FX3_BSP_H_


/* macro declaration */
#define CAMERA_RST_GPIO        17  // CTL[0]
#define IMU_AD0_GPIO           18  // CTL[1]
#define IMU_NCS_GPIO           19  // CTL[2]
#define CAMERA_OE_GPIO         20  // CTL[3]
#define CAMERA_STANDBY_GPIO    21  // CTL[4]
/* warning: not use two pin Now. */
#define CAMERA_EXPOSURE_GPIO   23  // CTL[6]
#define IMU_INT_GPIO           24  // CTL[7]
#define IMU_G_FSYNC_GPIO       25  // CTL[8]

#define SENSOR_LED_GPIO        45  // GPIO45
#define CAMERA_SADR_GPIO       50  // I2S-CLK
#define CAMPWR_CONTROL_GPIO    52  // I2S-WS

#define DEBUG_GPIO1            37  // DQ20
#define DEBUG_GPIO2            38  // DQ21
#define DEBUG_GPIO3            39  // DQ22
#define DEBUG_GPIO4            40  // DQ23

#define HARD_VERSION_A3        41  // DQ24
#define HARD_VERSION_A2        42  // DQ25
#define HARD_VERSION_A1        43  // DQ26
#define HARD_VERSION_A0        44  // DQ27

#define SINGLE_V034_WITH_DIFF_ADDR

/* LED blink type */

enum LED_TYPE {
  LED_OFF = 0,
  LED_ON  = 1,
};
/* variable declaration*/
extern int hardware_version_num;
char* Baidu_ProductDscr[16];

/* function declaration */
extern void fx3_gpio_module_init(void);
extern void sensor_gpio_init(void);
extern void sensor_led_blink(enum LED_TYPE led_value);
extern void camera_hard_reset(void);
extern void fx3_IO_matrix_config(void);
extern void CyFxAppErrorHandler(CyU3PReturnStatus_t apiRetStatus);
extern int hadrware_version_detect(void);
extern void v034_set_unified_addr(void);
extern void v034_power_on(void);
extern void v034_power_off(void);
#endif  // FIRMWARE_INCLUDE_FX3_BSP_H_
