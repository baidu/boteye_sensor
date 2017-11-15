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

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include "include/sensor_v034_raw.h"
#include "include/fx3_bsp.h"
#include "include/debug.h"
/*****************************************************************************
**                               Global data & Function declaration
******************************************************************************/

static BYTE Buf[2];
uint8_t AE_MODE_CUR = 0;
// #define SENSOR_GAIN_CHECK

static void V034_ChipID_Check(uint8_t SlaveAddr);
void DRV_imgsSetRegs(void);
/*****************************************************************************
**                                  Function definition
******************************************************************************/
/* V034,reg address 8 bits,data 16bits */
CyU3PReturnStatus_t V034_SensorWrite2B(uint8_t SlaveAddr, uint8_t HighAddr,
                                       uint8_t LowAddr, uint8_t HighData, uint8_t LowData) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PI2cPreamble_t preamble;
  uint8_t Buf[2];

  if (SlaveAddr != L_SENSOR_ADDR_WR && SlaveAddr != R_SENSOR_ADDR_WR) {
    sensor_err("I2C Slave address is not valid!\r\n");
    return 1;
  }

  preamble.buffer[1] = LowAddr;
  preamble.buffer[0] = SlaveAddr; /* Slave address: Write operation */
  preamble.length = 2;
  preamble.ctrlMask = 0x0000;
  Buf[0] = HighData;
  Buf[1] = LowData;
  apiRetStatus = CyU3PI2cTransmitBytes(&preamble, Buf, 2, 0);
  if (apiRetStatus == CY_U3P_SUCCESS) {
    V034_delay(800);  /* known issue for SDK I2C */
  } else {
    sensor_err("W2B I2C write error\r\n");
  }
  return apiRetStatus;
}

CyU3PReturnStatus_t V034_SensorWrite(uint8_t SlaveAddr, uint8_t HighAddr,
                                     uint8_t LowAddr, uint8_t count, uint8_t *buf) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PI2cPreamble_t preamble;

  if (SlaveAddr != L_SENSOR_ADDR_WR && SlaveAddr != R_SENSOR_ADDR_WR) {
    sensor_err("I2C Slave address is not valid!\r\n");
    return 1;
  }

  if (count > 64) {
    sensor_err("ERROR: V034_SensorWrite count > 64\r\n");
    return 1;
  }
  preamble.buffer[1] = LowAddr;
  preamble.buffer[0] = SlaveAddr; /* Slave address: Write operation */
  preamble.length = 2;
  preamble.ctrlMask = 0x0000;
  apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, count, 0);
  if (apiRetStatus == CY_U3P_SUCCESS) {
    V034_delay(800);    /* known issue for SDK I2C */
  } else {
    sensor_err("W I2C write error\r\n");
  }
  return apiRetStatus;
}

CyU3PReturnStatus_t V034_RegisterWrite(uint8_t HighAddr, uint8_t LowAddr,
  uint8_t HighData, uint8_t LowData) {
  return V034_SensorWrite2B(SENSOR_ADDR_WR, HighAddr, LowAddr, HighData, LowData);
}

uint16_t V034_RegisterRead(uint8_t HighAddr, uint8_t LowAddr) {
  uint8_t buf[2];
  int16_t tmp;

  V034_SensorRead2B(SENSOR_ADDR_RD, HighAddr, LowAddr, buf);
  tmp = (buf[0] << 8) | buf[1];
  // sensor_dbg("read highaddr: 0x%x, lowaddr: 0x%x, buf[0]: 0x%x, buf[1] 0x%x\r\n",
  //             HighAddr, LowAddr, buf[0], buf[1]);

  return (uint16_t)tmp;
}

CyU3PReturnStatus_t V034_SensorRead2B(uint8_t SlaveAddr, uint8_t HighAddr,
                                      uint8_t LowAddr, uint8_t *buf) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PI2cPreamble_t preamble;

  if (SlaveAddr != L_SENSOR_ADDR_RD && SlaveAddr != R_SENSOR_ADDR_RD) {
    sensor_err("I2C Slave address is not valid!\r\n");
    return 1;
  }

  preamble.buffer[1] = LowAddr;
  preamble.buffer[2] = SlaveAddr;      /* Slave address: Read operation */
  preamble.buffer[0] = SlaveAddr - 1;  /* Slave address: Write operation */
  preamble.length = 3;
  preamble.ctrlMask = 0x0002;  // After the second byte,need to restart the I2C communication
  apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, 2, 0);
  if (apiRetStatus == CY_U3P_SUCCESS) {
    V034_delay(800);
  } else {
    sensor_err("R2B I2C read error\r\n");
  }
  return apiRetStatus;
}

CyU3PReturnStatus_t V034_SensorRead(uint8_t SlaveAddr, uint8_t HighAddr,
                                    uint8_t LowAddr, uint8_t count, uint8_t *buf) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PI2cPreamble_t preamble;

  if (SlaveAddr != L_SENSOR_ADDR_RD && SlaveAddr != R_SENSOR_ADDR_RD) {
    sensor_err("I2C Slave address is not valid!\r\n");
    return 1;
  }

  if ( count > 64 ) {
    sensor_err("ERROR: V034_SensorWrite count > 64\n");
    return 1;
  }

  preamble.buffer[1] = LowAddr;
  preamble.buffer[2] = SlaveAddr;      /* Slave address: Read operation */
  preamble.buffer[0] = SlaveAddr - 1;  /* Slave address: Write operation */
  preamble.length = 3;
  preamble.ctrlMask = 0x0002;
  apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, count, 0);
  if (apiRetStatus == CY_U3P_SUCCESS) {
    V034_delay(800);
  } else {
    sensor_err("R I2C read error\r\n");
  }
  return apiRetStatus;
}

void V034_delay(volatile int time) {
  while (time--);
}

#define XP_IMG_WIDTH  640
#define XP_START_COL  ((752 - 640) / 2)   // vaule is 56
#define XP_IMG_HEIGHT 480

#define XP_H_BLANK 206
#define XP_ROW_TIME (XP_IMG_WIDTH + XP_H_BLANK)  // 846

#define XP_IMG_FRAMERATE 25
#define XP_OSC_FREQ 27000000
#define XP_V_BLANK (XP_OSC_FREQ * 1 / XP_IMG_FRAMERATE - 4 - \
                    XP_ROW_TIME * XP_IMG_HEIGHT) / XP_ROW_TIME
/******************************************************************************************
**                                    sensor initialization
********************************************************************************************/
/* This is 640*480,from 752*480(Enable line 162 and line 166), Now it's 60.9Hz from Sensor */
#define SENSOR_VGA
const uint16_t MT9V034_Parallel[] = {
#ifdef SENSOR_VGA
      0x01, XP_START_COL,   // COL_WINDOW_START_CONTEXTA_REG
#else
      0x01, 0x0001,         // COL_WINDOW_START_CONTEXTA_REG
#endif
      0x02, 0x0004,         // ROW_WINDOW_START_CONTEXTA_REG
      0x03, XP_IMG_HEIGHT,  // ROW_WINDOW_SIZE_CONTEXTA_REG
#ifdef SENSOR_VGA
      0x04, XP_IMG_WIDTH,   // COL_WINDOW_SIZE_CONTEXTA_REG
#else
      0x04, 0x02F0,         // COL_WINDOW_SIZE_CONTEXTA_REG
#endif
      0x05, XP_H_BLANK,     // HORZ_BLANK_CONTEXTA_REG
      0x06, XP_V_BLANK,     // VERT_BLANK_CONTEXTA_REG
      0x07, 0x0188,   // CONTROL_MODE_REG
      0x08, 0x0190,   // COARSE_SHUTTER_WIDTH_1_CONTEXTA
      0x09, 0x01BD,   // COARSE_SHUTTER_WIDTH_2_CONTEXTA
      0x0A, 0x0164,   // SHUTTER_WIDTH_CONTROL_CONTEXTA
      0x0B, 0x01C2,   // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
      0x0C, 0x0000,   // RESET_REG
      0x0D, 0x0300,   // READ_MODE_REG
      0x0E, 0x0000,   // READ_MODE2_REG
      0x0F, 0x0000,   // PIXEL_OPERATION_MODE
      0x10, 0x0040,   // RESERVED_CORE_10

      0x11, 0x8042,   // RESERVED_CORE_11
      0x12, 0x0022,   // RESERVED_CORE_12
      0x13, 0x2D2E,   // RESERVED_CORE_13
      0x14, 0x0E02,   // RESERVED_CORE_14
      0x15, 0x0E32,   // RESERVED_CORE_15
      0x16, 0x2802,   // RESERVED_CORE_16
      0x17, 0x3E38,   // RESERVED_CORE_17
      0x18, 0x3E38,   // RESERVED_CORE_18
      0x19, 0x2802,   // RESERVED_CORE_19
      0x1A, 0x0428,   // RESERVED_CORE_1A
      // close LED_out when exposure
      0x1B, 0x0001,   // LED_OUT_CONTROL
      0x1C, 0x0302,   // DATA_COMPRESSION
      0x1D, 0x0040,   // RESERVED_CORE_1D
      0x1E, 0x0000,   // RESERVED_CORE_1E
      0x1F, 0x0000,   // RESERVED_CORE_1F

      0x20, 0x03C7,   // RESERVED_CORE_20
      0x21, 0x0020,   // RESERVED_CORE_21
      0x22, 0x0020,   // RESERVED_CORE_22
      0x23, 0x0010,   // RESERVED_CORE_23
      0x24, 0x001B,   // RESERVED_CORE_24
      0x25, 0x001A,   // RESERVED_CORE_25
      0x26, 0x0004,   // RESERVED_CORE_26
      0x27, 0x000C,   // RESERVED_CORE_27
      0x28, 0x0010,   // RESERVED_CORE_28
      0x29, 0x0010,   // RESERVED_CORE_29
      0x2A, 0x0020,   // RESERVED_CORE_2A
      0x2B, 0x0003,   // RESERVED_CORE_2B
      0x2C, 0x0004,   // VREF_ADC_CONTROL
      0x2D, 0x0004,   // RESERVED_CORE_2D
      0x2E, 0x0007,   // RESERVED_CORE_2E
      0x2F, 0x0003,   // RESERVED_CORE_2F

      0x30, 0x0003,   // RESERVED_CORE_30
      0x31, 0x001F,   // V1_CONTROL_CONTEXTA
      0x32, 0x001A,   // V2_CONTROL_CONTEXTA
      0x33, 0x0012,   // V3_CONTROL_CONTEXTA
      0x34, 0x0003,   // V4_CONTROL_CONTEXTA
      0x35, 0x0020,   // GLOBAL_GAIN_CONTEXTA_REG
      0x36, 0x0010,   // GLOBAL_GAIN_CONTEXTB_REG
      0x37, 0x0000,   // RESERVED_CORE_37
      0x38, 0x0000,   // RESERVED_CORE_38
      0x39, 0x0025,   // V1_CONTROL_CONTEXTB
      0x3A, 0x0020,   // V2_CONTROL_CONTEXTB
      0x3B, 0x0003,   // V3_CONTROL_CONTEXTB
      0x3C, 0x0003,   // V4_CONTROL_CONTEXTB

      0x46, 0x231D,   // DARK_AVG_THRESHOLDS
      0x47, 0x0080,   // CALIB_CONTROL_REG
      0x4C, 0x0002,   // STEP_SIZE_AVG_MODE

      0x70, 0x0000,   // ROW_NOISE_CONTROL
      0x71, 0x002A,   // NOISE_CONSTANT
      0x72, 0x0000,   // PIXCLK_CONTROL
      0x7F, 0x0000,   // TEST_DATA

      0x80, 0x04F4,   // TILE_X0_Y0
      0x81, 0x04F4,   // TILE_X1_Y0
      0x82, 0x04F4,   // TILE_X2_Y0
      0x83, 0x04F4,   // TILE_X3_Y0
      0x84, 0x04F4,   // TILE_X4_Y0
      0x85, 0x04F4,   // TILE_X0_Y1
      0x86, 0x04F4,   // TILE_X1_Y1
      0x87, 0x04F4,   // TILE_X2_Y1
      0x88, 0x04F4,   // TILE_X3_Y1
      0x89, 0x04F4,   // TILE_X4_Y1
      0x8A, 0x04F4,   // TILE_X0_Y2
      0x8B, 0x04F4,   // TILE_X1_Y2
      0x8C, 0x04F4,   // TILE_X2_Y2
      0x8D, 0x04F4,   // TILE_X3_Y2
      0x8E, 0x04F4,   // TILE_X4_Y2
      0x8F, 0x04F4,   // TILE_X0_Y3

      0x90, 0x04F4,   // TILE_X1_Y3
      0x91, 0x04F4,   // TILE_X2_Y3
      0x92, 0x04F4,   // TILE_X3_Y3
      0x93, 0x04F4,   // TILE_X4_Y3
      0x94, 0x04F4,   // TILE_X0_Y4
      0x95, 0x04F4,   // TILE_X1_Y4
      0x96, 0x04F4,   // TILE_X2_Y4
      0x97, 0x04F4,   // TILE_X3_Y4
      0x98, 0x04F4,   // TILE_X4_Y4
      0x99, 0x0000,   // X0_SLASH5
      0x9A, 0x0096,   // X1_SLASH5
      0x9B, 0x012C,   // X2_SLASH5
      0x9C, 0x01C2,   // X3_SLASH5
      0x9D, 0x0258,   // X4_SLASH5
      0x9E, 0x02F0,   // X5_SLASH5
      0x9F, 0x0000,   // Y0_SLASH5

      0xA0, 0x0060,   // Y1_SLASH5
      0xA1, 0x00C0,   // Y2_SLASH5
      0xA2, 0x0120,   // Y3_SLASH5
      0xA3, 0x0180,   // Y4_SLASH5
      0xA4, 0x01E0,   // Y5_SLASH5
      0xA5, 0x003A,   // DESIRED_BIN
      0xA6, 0x0002,   // EXP_SKIP_FRM_H
      0xA8, 0x0000,   // EXP_LPF
      0xA9, 0x0002,   // GAIN_SKIP_FRM
      0xAA, 0x0002,   // GAIN_LPF_H
      0xAB, 0x0040,   // MAX_GAIN
      0xAC, 0x0001,   // MIN_COARSE_EXPOSURE
      0xAD, 0x01E0,   // MAX_COARSE_EXPOSURE
      0xAE, 0x0014,   // BIN_DIFF_THRESHOLD
      0xAF, 0x0000,   // AUTO_BLOCK_CONTROL

      0xB0, 0xABE0,   // PIXEL_COUNT
      0xB1, 0x0002,   // LVDS_MASTER_CONTROL
      0xB2, 0x0010,   // LVDS_SHFT_CLK_CONTROL
      0xB3, 0x0010,   // LVDS_DATA_CONTROL
      0xB4, 0x0000,   // LVDS_DATA_STREAM_LATENCY
      0xB5, 0x0000,   // LVDS_INTERNAL_SYNC
      0xB6, 0x0000,   // LVDS_USE_10BIT_PIXELS
      0xB7, 0x0000,   // STEREO_ERROR_CONTROL
      0xBF, 0x0016,   // INTERLACE_FIELD_VBLANK

      0xC0, 0x000A,   // IMAGE_CAPTURE_NUM
      0xC2, 0x18D0,   // ANALOG_CONTROLS
      0xC3, 0x007F,   // RESERVED_CORE_C3
      0xC4, 0x007F,   // RESERVED_CORE_C4
      0xC5, 0x007F,   // RESERVED_CORE_C5
      0xC6, 0x0000,   // NTSC_FV_CONTROL
      0xC7, 0x4416,   // NTSC_HBLANK
      0xC8, 0x4421,   // NTSC_VBLANK
      0xC9, 0x0002,   // COL_WINDOW_START_CONTEXTB_REG
      0xCA, 0x0004,   // ROW_WINDOW_START_CONTEXTB_REG
      0xCB, 0x01E0,   // ROW_WINDOW_SIZE_CONTEXTB_REG
      0xCC, 0x02EE,   // COL_WINDOW_SIZE_CONTEXTB_REG
      0xCD, 0x0100,   // HORZ_BLANK_CONTEXTB_REG
      0xCE, 0x0100,   // VERT_BLANK_CONTEXTB_REG
      0xCF, 0x0190,   // COARSE_SHUTTER_WIDTH_1_CONTEXTB

      0xD0, 0x01BD,   // COARSE_SHUTTER_WIDTH_2_CONTEXTB
      0xD1, 0x0064,   // SHUTTER_WIDTH_CONTROL_CONTEXTB
      0xD2, 0x01C2,   // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTB
      0xD3, 0x0000,   // FINE_SHUTTER_WIDTH_1_CONTEXTA
      0xD4, 0x0000,   // FINE_SHUTTER_WIDTH_2_CONTEXTA
      0xD5, 0x0000,   // FINE_SHUTTER_WIDTH_TOTAL_CONTEXTA
      0xD6, 0x0000,   // FINE_SHUTTER_WIDTH_1_CONTEXTB
      0xD7, 0x0000,   // FINE_SHUTTER_WIDTH_2_CONTEXTB
      0xD8, 0x0000,   // FINE_SHUTTER_WIDTH_TOTAL_CONTEXTB
      0xD9, 0x0001,   // MONITOR_MODE_CONTROL
    };

void V034_stream_start(uint8_t SlaveAddr) {
  uint16_t Stream_Start_Addr  = 0xD9;
  uint16_t Stream_Start_Value = 0x00;
  BYTE AddrH, AddrL, ValH, ValL;

  AddrH  = (Stream_Start_Addr >> 8) & 0xff;
  AddrL  = (Stream_Start_Addr) & 0xff;
  ValH   = (Stream_Start_Value >> 8) & 0xff;
  ValL   = (Stream_Start_Value) & 0xff;
  V034_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
}

void V034_stream_stop(uint8_t SlaveAddr) {
  uint16_t Stream_Stop_Addr  = 0xD9;
  uint16_t Stream_Stop_Value = 0x01;
  BYTE AddrH, AddrL, ValH, ValL;

  AddrH  = (Stream_Stop_Addr >> 8) & 0xff;
  AddrL  = (Stream_Stop_Addr) & 0xff;
  ValH   = (Stream_Stop_Value >> 8) & 0xff;
  ValL   = (Stream_Stop_Value) & 0xff;
  V034_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
}

void V034_soft_reset(uint8_t SlaveAddr) {
  uint16_t Soft_Reset_Addr = 0x0C;
  uint16_t Soft_Reset_Value = 0x01;
  BYTE AddrH, AddrL, ValH, ValL;

  AddrH  = (Soft_Reset_Addr >> 8) & 0xff;
  AddrL  = (Soft_Reset_Addr) & 0xff;
  ValH   = (Soft_Reset_Value >> 8) & 0xff;
  ValL   = (Soft_Reset_Value) & 0xff;
  V034_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
  /* delay a little time for regitster setting */
  V034_delay(100*1000);

  Soft_Reset_Value = 0x00;
  ValH   = (Soft_Reset_Value >> 8) & 0xff;
  ValL   = (Soft_Reset_Value) & 0xff;
  V034_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
}
void V034_sensor_init(void) {
#ifdef SENSOR_GAIN_CHECK
  uint16_t tmp;
#endif

  sensor_dbg("V034_sensor_init \r\n");
  DRV_imgsSetRegs();

#ifdef SENSOR_GAIN_CHECK
  V034_SensorRead2B(SENSOR_ADDR_RD, 0x30, 0x56, Buf);
  tmp = (Buf[0] << 8) | Buf[1];
  sensor_dbg("ReadBack= 0x%x \r\n", tmp);

  V034_SensorRead2B(SENSOR_ADDR_RD, 0x30, 0x58, Buf);
  tmp = (Buf[0] << 8) | Buf[1];
  sensor_dbg("ReadBack= 0x%x \r\n", tmp);

  V034_SensorRead2B(SENSOR_ADDR_RD, 0x30, 0x5a, Buf);
  tmp = (Buf[0] << 8) | Buf[1];
  sensor_dbg("ReadBack= 0x%x \r\n", tmp);

  V034_SensorRead2B(SENSOR_ADDR_RD, 0x30, 0x5c, Buf);
  tmp = (Buf[0] << 8) | Buf[1];
  sensor_dbg("ReadBack= 0x%x \r\n", tmp);

  V034_SensorRead2B(SENSOR_ADDR_RD, 0x30, 0xb0, Buf);
  tmp = (Buf[0] << 8) | Buf[1];
  sensor_dbg("ReadBack= 0x%x \r\n", tmp);
#endif
  sensor_dbg("V034_sensor_init done \r\n");
}

static void V034_ChipID_Check(uint8_t SlaveAddr) {
  uint16_t ChipID;

  sensor_dbg("V034_ChipID_Check\r\n");
  V034_SensorRead2B(SlaveAddr, 0x30, 0x00, (uint8_t *)Buf);
  ChipID = (Buf[0] << 8) | Buf[1];
  sensor_dbg("%s Sensor addr: 0x%x ID:0x%x \r\n", SlaveAddr == L_SENSOR_ADDR_RD ? "left": "right",
              SlaveAddr, ChipID);
}

void DRV_imgsSetRegs(void) {
  int j;
  BYTE AddrH, AddrL, ValH, ValL;

  /* init Left image sensor*/
  sensor_dbg("init Left image sensor regitster.\r\n");
  V034_soft_reset(L_SENSOR_ADDR_WR);
  V034_ChipID_Check(L_SENSOR_ADDR_RD);

  for (j = 0; j < sizeof(MT9V034_Parallel) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (MT9V034_Parallel[j] >> 8) & 0xff;
    AddrL  = (MT9V034_Parallel[j]) & 0xff;
    ValH   = (MT9V034_Parallel[j + 1] >> 8) & 0xff;
    ValL   = (MT9V034_Parallel[j + 1]) & 0xff;

    V034_SensorWrite2B(L_SENSOR_ADDR_WR, AddrH, AddrL, ValH, ValL);
  }
  // dump_sensor_registers(L_SENSOR_ADDR_RD);
  /* init Right image sensor*/
  V034_soft_reset(R_SENSOR_ADDR_WR);
  V034_ChipID_Check(R_SENSOR_ADDR_RD);
  sensor_dbg("init Right image sensor regitster.\r\n");

  for (j = 0; j < sizeof(MT9V034_Parallel) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (MT9V034_Parallel[j] >> 8) & 0xff;
    AddrL  = (MT9V034_Parallel[j]) & 0xff;
    ValH   = (MT9V034_Parallel[j + 1] >> 8) & 0xff;
    ValL   = (MT9V034_Parallel[j + 1]) & 0xff;

    V034_SensorWrite2B(R_SENSOR_ADDR_WR, AddrH, AddrL, ValH, ValL);
  }
  // dump_sensor_registers(R_SENSOR_ADDR_RD);
  /* reset two hard address to one addr, so we use SENSOR_ADDR_WR and SENSOR_ADDR_RD later  */
  v034_set_unified_addr();
}
void dump_sensor_registers(uint8_t SlaveAddr) {
  int j = 0;
  BYTE AddrH, AddrL;
  BYTE buf[2];
  uint16_t reg_value;

  sensor_info("=================dump start=============================\r\n");
  for (j = 0; j < sizeof(MT9V034_Parallel) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (MT9V034_Parallel[j] >> 8) & 0xff;
    AddrL  = (MT9V034_Parallel[j]) & 0xff;
    V034_SensorRead2B(SlaveAddr, AddrH, AddrL, buf);
    reg_value = (buf[0] << 8) | buf[1];
    sensor_info("%s image sensor register 0x%x value: 0x%x \r\n",
                SlaveAddr == L_SENSOR_ADDR_RD ? "left": "right",
                MT9V034_Parallel[j], reg_value);
  }
  sensor_info("=================dump   end=============================\r\n");
}

/*
   Get the current brightness setting from the AP0100+MT9V034 sensor.
 */
uint8_t V034_SensorGetBrightness(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x0A, buf);
  return (uint8_t)buf[1];
}

/*
   Update the brightness setting for the AP0100+MT9V034 sensor.
 */
void V034_SensorSetBrightness(uint8_t brightness) {
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x0A, 0x00, brightness);
}

/*Contrast 16~64 for sensor,64~255 from UVC*/
/*
   Get the current contrast setting from the AP0100+MT9V034 sensor.
 */
uint8_t V034_SensorGetContrast(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x0C, buf);
  buf[1] = buf[1] << 2;

  return (uint8_t)buf[1];
}

/*
   Update the contrast setting for the AP0100+MT9V034 sensor,the min value is 16
 */
void V034_SensorSetContrast(uint8_t contrast) {
  if (contrast > 255)contrast = 255;
  if (contrast < 64)contrast = 64;

  contrast = contrast >> 2;

  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x0C, 0x00, contrast);
}
/*
   Get the current HUE setting from the AP0100+MT9V034 sensor,step by 256
 */
uint8_t V034_SensorGetHUE(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x10, buf);
  // return high byte,low byte will always be 0
  return (uint8_t)buf[0];
}

/*
   Update the HUE setting for the AP0100+MT9V034 sensor
 */
void V034_SensorSetHUE(uint8_t HUE) {
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x10, HUE, 0);
}
/*AWB AUTO Control */
uint8_t V034_SensorGetAWB(void) {
  uint8_t buf[2];
  uint8_t tmp = 1;

  V034_SensorRead(SENSOR_ADDR_RD, 0xC9, 0x7D, 1, buf);
  // manual white balance
  if (buf[0] == 0x02)
    tmp = 0;
  // auto white balance
  if (buf[0] == 0x00)
    tmp = 1;

  return (uint8_t)tmp;
}
void V034_SensorSetAWB(uint8_t enable) {
  uint8_t buf[2];

  if (enable == 0x01)
    buf[0] = 0x00;
  if (enable == 0x00)
    buf[0] = 0x02;

  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x09, 0x8E, 0xC9, 0x7D);

  V034_SensorWrite(SENSOR_ADDR_WR, 0xC9, 0x7D, 1, buf);
  // COMMAND_REGISTER
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x40, 0x86, 0x06);
}
/*
   AWB AUTO Control
 */
uint16_t V034_SensorGetAWB_TMP(void) {
  uint8_t buf[2];
  int16_t tmp;

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xC9, 0x28, buf);

  tmp = (buf[0] << 8) | (buf[1]);

  return (uint16_t)tmp;
}
void V034_SensorSetAWB_TMP(uint16_t tmp) {
  if (tmp > 6500 || tmp < 2500)
    return;
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xC9, 0x28, tmp >> 8, tmp & 0xff);
}

/*
  Saturation Control
 */
uint8_t V034_SensorGetSaturation(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x12, buf);
  return (uint8_t)buf[1];
}

void V034_SensorSetSaturation(uint8_t tmp) {
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x12, 0, tmp);
}

/*
  Sharpness Control
 */
uint8_t V034_SensorGetSharpness(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x14, buf);

  return (uint8_t)buf[1];
}

void V034_SensorSetSharpness(uint8_t tmp) {
  if (tmp > 7)
    tmp = 7;
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x14, 0, tmp);
}

/*
  Gamma Control
 */
uint8_t V034_SensorGetGamma(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x14, buf);
  return (uint8_t)buf[1];
}

void V034_SensorSetGamma(uint8_t tmp) {
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x14, 0, tmp);
}

/*
  Gain Control,1X~4X
 */
uint8_t V034_SensorGetGain(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x35, buf);

  return (uint8_t)(buf[1] & 0x7F);
}

void V034_SensorSetGain(uint8_t tmp) {
  uint8_t buf[2];

  V034_SensorSetAEMode(V034_MANUAL_EXPOSURE_MODE);
  V034_SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x35, buf);

  buf[1] &= 0x80;

  if (tmp == 0)
    buf[1] |= 16;
  if (tmp == 1)
    buf[1] |= 32;
  if (tmp == 2)
    buf[1] |= 48;
  if (tmp == 3)
    buf[1] |= 64;

  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x35, buf[0], buf[1]);

  CyU3PThreadSleep(1);
}
/*
  Backlight Control
 */
uint8_t V034_SensorGetBacklight(void) {
  uint8_t buf[2];

  V034_SensorRead2B(SENSOR_ADDR_RD, 0xCC, 0x08, buf);

  return (uint8_t)buf[1];
}
void V034_SensorSetBacklight(uint8_t tmp) {
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xCC, 0x08, 0, tmp);
}

/*
  AE Mode Control
 */
uint8_t V034_SensorGetAEMode(void) {
  uint8_t buf[2];
  uint8_t tmp;
  V034_SensorRead2B(SENSOR_ADDR_RD, 0x00, 0xAF, buf);

  if (buf[1] & 0x01)
    tmp = 8;  // auto exposure
  else
    tmp = 1;  // manual exposure

  return (uint8_t)tmp;
}
void V034_SensorSetAEMode(uint8_t enable) {
  uint8_t buf[2];

  if (enable != AE_MODE_CUR) {
    AE_MODE_CUR = enable;

    V034_SensorRead2B(SENSOR_ADDR_RD, 0x00, 0xAF, buf);

    if (enable == 8) {
      buf[1] |= 0x01;  // enable exposure
    } else {
      buf[1] &= 0xfe;  // manual exposure
    }

    V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0xAF, buf[0], buf[1]);
  }
}

/*
  Exposuretime Control
 */
uint16_t V034_SensorGetExposuretime(void) {
  uint8_t buf[2];
  int16_t tmp;

  V034_SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x0B, buf);

  tmp = (buf[0] << 8) | (buf[1]);

  return (uint16_t)tmp;
}
void V034_SensorSetExposuretime(uint16_t tmp) {
  // CAM_EXP_CTRL_COARSE_INTEGRATION_TIME
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x0B, tmp >> 8, tmp & 0xff);

  CyU3PThreadSleep(1);
}

/*
   Power Line Frequency Control,UVC protocal,1:50Hz;2:60Hz
 */
uint8_t V034_SensorGetPowerLineFreq(void) {
  uint8_t buf[2];
  uint8_t tmp = 0x2;

  V034_SensorRead(SENSOR_ADDR_RD, 0x94, 0x04, 1, buf);
  if (buf[0] == 0x32)tmp = 1;
  if (buf[0] == 0x3c)tmp = 2;

  return tmp;
}
void V034_SensorSetPowerLineFreq(uint8_t enable) {
  uint8_t buf[2];

  if (enable == 2)
    buf[0] = 0x3c;   // 60Hz
  if (enable == 1)
    buf[0] = 0x32;   // 50Hz
  // COMMAND_REGISTER
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x40, 0x81, 0x01);
  // LOGICAL_ADDRESS_ACCESS [NTSC_AET_FLICKER_FREQ_HZ]
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x09, 0x8E, 0x94, 0x04);
  // NTSC_AET_FLICKER_FREQ_HZ
  V034_SensorWrite(SENSOR_ADDR_WR, 0x94, 0x04, 1, buf);

  // Flicker avoidance frequency is 60
  // Requesting a Change-Config
  // CMD_HANDLER_PARAMS_POOL_0
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0xFC, 0x00, 0x28, 0x00);
  // COMMAND_REGISTER
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x40, 0x81, 0x00);
  // COMMAND_REGISTER
  V034_SensorWrite2B(SENSOR_ADDR_WR, 0x00, 0x40, 0x81, 0x01);
}
