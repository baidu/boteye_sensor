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

#ifndef FIRMWARE_INCLUDE_TLC59108_H_
#define FIRMWARE_INCLUDE_TLC59108_H_
// Macro Declare
#define TLC59108_ADDR             0x80
#define TLC59108_SWRST_ADDR       0b1101011
// TLC PWM value range is 0 - 255
#define TLC59108_DEFAULT_HEPTAGON_PWM      120
// Eevery channel current range of TLC59108 is 10 to 120mA For XPIRL3:
// 0 - 1 channel control LIMA
// 2 - 4 channel control Heptagon-A, required current 350mA (range: 320 - 440mA)
// 5 - 7 channel control Heptagon-B, required current 350mA (range: 320 - 440mA)
// register map
#define TLC59108_Mode1      0x00
#define TLC59108_Mode2      0x01
#define TLC59108_PWM0       0x02
#define TLC59108_PWM1       0x03
#define TLC59108_PWM2       0x04
#define TLC59108_PWM3       0x05
#define TLC59108_PWM4       0x06
#define TLC59108_PWM5       0x07
#define TLC59108_PWM6       0x08
#define TLC59108_PWM7       0x09
#define TLC59108_GRPPWM     0x0A
#define TLC59108_GRPFREQ    0x0B
#define TLC59108_LEDOUT0    0x0C
#define TLC59108_LEDOUT1    0x0D
#define TLC59108_SUBADR1    0x0E
#define TLC59108_SUBADR2    0x0F
#define TLC59108_SUBADR3    0x10
#define TLC59108_ALLCALLADR 0x11
#define TLC59108_IREF       0x12
#define TLC59108_EFLAG      0x13

// Function Declare
void tlc59108_init(void);

uint8_t tlc59108_reg_read(uint8_t reg_addr);
void tlc59108_reg_write(uint8_t reg_addr, uint8_t reg_val);
void tlc59108_set_channel(uint8_t channel, enum  StateMode mode);
void tlc59108_all_close(void);
void tlc59108_heptagon_close(void);
void tlc59108_LIMA_close(void);
void tlc59108_LIMA_open(uint8_t pwm_value);
void tlc59108_LIMA_ON(void);
void tlc59108_heptagon_open(uint8_t PWM_value);
void tlc59108_dump_register(void);
void xpril3_proc_ir_ctl(struct IR_ctl_t* IR_ctrl);
#endif  // FIRMWARE_INCLUDE_TLC59108_H_
