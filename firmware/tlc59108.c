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
#include <cyu3error.h>
#include <cyu3gpio.h>
#include "include/i2c.h"
#include "include/debug.h"
#include "include/tlc59116.h"
#include "include/tlc59108.h"
#include "include/fx3_bsp.h"
#include "include/uvc.h"

/**
 *  @brief      tlc59108 init.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_init(void) {
  sensor_dbg("tlc59108 init\r\n");

  tlc59108_reg_write(TLC59108_Mode1, 0x00);
  tlc59108_reg_write(TLC59108_Mode2, 0x00);
  tlc59108_reg_write(TLC59108_PWM0, 0xFE);
  tlc59108_reg_write(TLC59108_PWM1, 0xFE);
  tlc59108_reg_write(TLC59108_LEDOUT0, 0xAA);
  tlc59108_reg_write(TLC59108_LEDOUT1, 0xAA);
}

/**
 *  @brief      tlc59108 register read.
 *  @param[out] NULL.
 *  @return     NULL.
 */
uint8_t tlc59108_reg_read(uint8_t reg_addr) {
  uint8_t reg_val;
  // reg_addr = 0x1F & reg_addr;
  Sensors_I2C_ReadReg(TLC59108_ADDR, reg_addr, 1, &reg_val);
  return reg_val;
}

/**
 *  @brief      tlc59108 register write.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_reg_write(uint8_t reg_addr, uint8_t reg_val) {
  // reg_addr = 0x1F & reg_addr;
  Sensors_I2C_WriteReg(TLC59108_ADDR, reg_addr, 1, &reg_val);
}

/**
 *  @brief      tlc59108 open channel.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_set_channel(uint8_t channel, enum  StateMode mode) {
  uint8_t reg_num = channel >> 2;
  uint8_t shift_num = channel % 4;
  uint8_t reg_val = 0;

  reg_val = tlc59108_reg_read(TLC59108_LEDOUT0 + reg_num);
  // sensor_dbg("TLC59108 read register:%d ,reg_val:0x%x\r\n", reg_num, reg_val);
  // clear
  reg_val = reg_val & (~(0x03 << (2 * shift_num)));
  // set
  reg_val = reg_val | (mode << (2 * shift_num));
  // sensor_info("TLC59108 write register:%d ,reg_val:0x%x\r\n", reg_num, reg_val);
  tlc59108_reg_write(TLC59108_LEDOUT0 + reg_num, reg_val);
}

/**
 *  @brief      tlc59108 close output.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_all_close(void) {
  int i = 0;
  for (i = 0; i < 8; i++) {
      tlc59108_set_channel(i, TLC_CH_OFF);
  }
}

/**
 *  @brief      tlc59108 close LIMA light output.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_LIMA_close(void) {
// Just open 0 1 channel of TLC59108 for LIMA Light
  tlc59108_set_channel(0, TLC_CH_OFF);
  tlc59108_set_channel(1, TLC_CH_OFF);
}

/**
 *  @brief      tlc59108 PWM control LIMA output.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_LIMA_open(uint8_t PWM_value) {
// Just open 0 1 channel of tlc59108 for LIMA Light
  tlc59108_reg_write(TLC59108_PWM0 + 0, PWM_value);
  tlc59108_set_channel(0,  TLC_CH_PWM);
  tlc59108_reg_write(TLC59108_PWM0 +1, PWM_value);
  tlc59108_set_channel(1,  TLC_CH_PWM);
}

/**
 *  @brief      tlc59108 ON control LIMA output.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_LIMA_ON(void) {
// Just open 0 1 channel of tlc59108 for LIMA Light
  tlc59108_set_channel(0,  TLC_CH_ON);
  tlc59108_set_channel(1,  TLC_CH_ON);
}

/**
 *  @brief      tlc59108 set Light PWM mode for channel.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_heptagon_open(uint8_t PWM_value) {
  // Just open 2-7 channel of TLC59108 for Heptagon Light
  int i = 0;
  for (i = 2 ; i < 8 ; i++) {
      tlc59108_reg_write(TLC59108_PWM0 + i, PWM_value);
      tlc59108_set_channel(i, TLC_CH_PWM);
  }
}

/**
 *  @brief      tlc59108 close LIMA light output.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_heptagon_close(void) {
// Just open 2-7 channel of tlc59108 for Heptagon Light
  int i = 0;
  for (i = 2 ; i < 8 ; i++)
  tlc59108_set_channel(i, TLC_CH_OFF);
}

/**
 *  @brief      TLC59108 dump all register.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void tlc59108_dump_register(void) {
  uint8_t reg_val = 0;

  reg_val = tlc59108_reg_read(TLC59108_Mode1);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_Mode1, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_Mode2);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_Mode2, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM0);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM0, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM1);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM1, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM2);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM2, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM3);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM3, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM4);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM4, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM5);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM5, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM6);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM6, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_PWM7);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_PWM7, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_GRPPWM);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_GRPPWM, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_GRPFREQ);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_GRPFREQ, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_LEDOUT0);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_LEDOUT0, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_LEDOUT1);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_LEDOUT1, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_SUBADR1);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_SUBADR1, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_SUBADR2);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_SUBADR2, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_SUBADR3);
  sensor_dbg("tlc59108 0x%x tlc59116_initregister value: 0x%x\r\n", TLC59108_SUBADR3, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_ALLCALLADR);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_ALLCALLADR, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_IREF);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_IREF, reg_val);
  reg_val = tlc59108_reg_read(TLC59108_EFLAG);
  sensor_dbg("tlc59108 0x%x register value: 0x%x\r\n", TLC59108_EFLAG, reg_val);
}

/**
 *  @brief      process ir control.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void xpril3_proc_ir_ctl(struct IR_ctl_t* IR_ctrl) {
  if (IR_ctrl->Set_infrared_mode == 1 && IR_ctrl->Set_structured_mode == 1) {
    sensor_info("open Both structured and infrared Light, pwm:0x%x\r\n", IR_ctrl->pwm_value);
    tlc59108_heptagon_open(IR_ctrl->pwm_value);
    tlc59108_LIMA_open(IR_ctrl->pwm_value);
  } else if (IR_ctrl->Set_infrared_mode == 1 && IR_ctrl->Set_structured_mode == 0) {
    sensor_info("Only open Infrared Light, pwm:0x%x\r\n", IR_ctrl->pwm_value);
    tlc59108_LIMA_open(IR_ctrl->pwm_value);
    tlc59108_heptagon_close();
  } else if (IR_ctrl->Set_infrared_mode == 0 && IR_ctrl->Set_structured_mode == 1) {
    sensor_info("Only open structured Light, pwm:0x%x\r\n", IR_ctrl->pwm_value);
    tlc59108_heptagon_open(IR_ctrl->pwm_value);
    tlc59108_LIMA_close();
  } else {
    sensor_info("close all IR light\r\n");
    // close current control output
    tlc59108_all_close();
    // close vcc control output
    IR_LED_OFF();
  }
}
