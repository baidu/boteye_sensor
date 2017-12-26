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
#include <cyu3error.h>
#include <cyu3gpio.h>
#include "include/i2c.h"
#include "include/debug.h"
#include "include/tlc59116.h"

struct tlc59116_ctl_t tl59116_ctrl;
/**
 *  @brief      tlc59116 init.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_init(void) {
  sensor_dbg("tlc59116 init\r\n");

  tlc59116_reg_write(TLC59116_Mode1, 0x00);
  tlc59116_reg_write(TLC59116_Mode2, 0x00);
}

/**
 *  @brief      tlc59116 register read.
 *  @param[out] NULL.
 *  @return     NULL.
 */
uint8_t tlc59116_reg_read(uint8_t reg_addr) {
  uint8_t reg_val;
  // reg_addr = 0x1F & reg_addr;
  Sensors_I2C_ReadReg(TLC59116_ADDR, reg_addr, 1, &reg_val);
  return reg_val;
}

/**
 *  @brief      tlc59116 register write.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_reg_write(uint8_t reg_addr, uint8_t reg_val) {
  // reg_addr = 0x1F & reg_addr;
  Sensors_I2C_WriteReg(TLC59116_ADDR, reg_addr, 1, &reg_val);
}

/**
 *  @brief      tlc59116 open channel.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_set_channel(uint8_t channel, enum  StateMode mode) {
  uint8_t reg_num = channel >> 2;
  uint8_t shift_num = channel % 4;
  uint8_t reg_val = 0;

  reg_val = tlc59116_reg_read(TLC59116_LEDOUT0 + reg_num);
  // sensor_dbg("TLC59116 read register:%d ,reg_val:%d\r\n", reg_num, reg_val);
  // clear
  reg_val = reg_val & (~(0x03 << (2 * shift_num)));
  // set
  reg_val = reg_val | (mode << shift_num);
  // sensor_info("TLC59116 writeregister:%d ,reg_val:%d\r\n", reg_num, reg_val);
  tlc59116_reg_write(TLC59116_LEDOUT0 + reg_num, reg_val);
}

/**
 *  @brief      tlc59116 set Light PWM mode.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_set_pwm(uint16_t channel, uint8_t PWM_value) {
  int i = 0;
  for (i = 0; i < 16; i++) {
    if (((channel >> i) & 0x0001) == 1) {
      tlc59116_reg_write(TLC59116_PWM0 + i, PWM_value);
      tlc59116_set_channel(i,  TLC_CH_PWM);
    }
  }
}

/**
 *  @brief      tlc59116 close output.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_off(uint16_t ch_value) {
  int i = 0;
  for (i = 0; i < 16; i++) {
    if (((ch_value >> i) & 0x0001) == 1)
      tlc59116_set_channel(i, TLC_CH_OFF);
  }
}

/**
 *  @brief      tlc59116 channel on state.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_CH_on(uint16_t ch_value) {
  int i = 0;
  for (i = 0; i < 16; i++) {
    if (((ch_value >> i) & 0x0001) == 1)
      tlc59116_set_channel(i, TLC_CH_ON);
  }
}

/**
 *  @brief      tlc59116 dump all register.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59116_dump_register(void) {
  uint8_t reg_val = 0;

  reg_val = tlc59116_reg_read(TLC59116_Mode1);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_Mode1, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_Mode2);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_Mode2, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM0);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM0, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM1);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM1, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM2);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM2, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM3);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM3, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM4);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM4, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM5);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM5, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM6);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM6, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM7);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM7, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM8);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM8, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM9);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM9, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM10);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM10, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM11);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM11, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM12);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM12, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM13);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM13, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM14);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM14, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_PWM15);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_PWM15, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_GRPPWM);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_GRPPWM, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_GRPFREQ);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_GRPFREQ, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_LEDOUT0);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_LEDOUT0, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_LEDOUT1);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_LEDOUT1, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_LEDOUT2);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_LEDOUT2, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_LEDOUT3);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_LEDOUT3, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_SUBADR1);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_SUBADR1, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_SUBADR2);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_SUBADR2, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_SUBADR3);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_SUBADR3, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_ALLCALLADR);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_ALLCALLADR, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_IREF);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_IREF, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_EFLAG1);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_EFLAG1, reg_val);
  reg_val = tlc59116_reg_read(TLC59116_EFLAG2);
  sensor_dbg("tl59116 0x%x register value: 0x%x\r\n", TLC59116_EFLAG2, reg_val);
}
