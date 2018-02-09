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

#ifndef FIRMWARE_INCLUDE_TLC59116_H_
#define FIRMWARE_INCLUDE_TLC59116_H_
// TLC59116 channel has four mode: OFF, ON, PWM, Group PWM
enum  StateMode {
  TLC_CH_OFF = 0,
  TLC_CH_ON = 1,
  TLC_CH_PWM = 2,
  TLC_CH_G_PWM = 3
  };
// Macro Declare
#define TLC59116_ADDR             0xC0
#define TLC59116_SWRST_ADDR       0b1101011
#define TLC59116_DEFAULT_PWM      120          // 256
#define TLC59116_DEFAULT_OPEN_CH  0x0707       // open 0,1,2 8,9,10 channel
#define TLC59116_DEFAULT_CLOSE_CH 0xFFFF       // close all channel
// register map
#define TLC59116_Mode1      0x00
#define TLC59116_Mode2      0x01
#define TLC59116_PWM0       0x02
#define TLC59116_PWM1       0x03
#define TLC59116_PWM2       0x04
#define TLC59116_PWM3       0x05
#define TLC59116_PWM4       0x06
#define TLC59116_PWM5       0x07
#define TLC59116_PWM6       0x08
#define TLC59116_PWM7       0x09
#define TLC59116_PWM8       0x0A
#define TLC59116_PWM9       0x0B
#define TLC59116_PWM10      0x0C
#define TLC59116_PWM11      0x0D
#define TLC59116_PWM12      0x0E
#define TLC59116_PWM13      0x0F
#define TLC59116_PWM14      0x10
#define TLC59116_PWM15      0x11
#define TLC59116_GRPPWM     0x12
#define TLC59116_GRPFREQ    0x13
#define TLC59116_LEDOUT0    0x14
#define TLC59116_LEDOUT1    0x15
#define TLC59116_LEDOUT2    0x16
#define TLC59116_LEDOUT3    0x17
#define TLC59116_SUBADR1    0x18
#define TLC59116_SUBADR2    0x19
#define TLC59116_SUBADR3    0x1A
#define TLC59116_ALLCALLADR 0x1B
#define TLC59116_IREF       0x1C
#define TLC59116_EFLAG1     0x1D
#define TLC59116_EFLAG2     0x1E

// Variable Declare
struct tlc59116_ctl_t {
    uint8_t UpdateBit: 1;
    uint8_t dumpRegister: 1;
    uint8_t WriteRegister: 1;
    uint8_t SetCH_on_mode: 1;
    uint8_t SetCH_pwm_mode: 1;
    uint8_t : 3;
    uint8_t pwm_value;
    uint16_t channel_value;
};
extern struct tlc59116_ctl_t tl59116_ctrl;
// Function Declare
void tlc59116_init(void);

uint8_t tlc59116_reg_read(uint8_t reg_addr);
void tlc59116_reg_write(uint8_t reg_addr, uint8_t reg_val);
void tlc59116_set_channel(uint8_t channel, enum  StateMode mode);
void tlc59116_set_pwm(uint16_t channel, uint8_t PWM_value);
void tlc59116_off(uint16_t ch_value);
void tlc59116_CH_on(uint16_t ch_value);
void tlc59116_dump_register(void);
#endif  // FIRMWARE_INCLUDE_TLC59116_H_
