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
#include "include/sensor_ar0141.h"
#include "include/sensor_v034_raw.h"
#include "include/debug.h"
#include "include/fx3_bsp.h"
// AR0141 register address map
// Frame rate Fps = 1/Tframe
// Tframe = 1 /(CLK_PIX) * [frame_length_lines * line_length_pck + extra_delay]
// Minimumframe_length_lines = (y_addr_end - y_addr_start + 1) /
//                             ((y_odd_inc + 1) / 2) + min_vertical_blanking
// TODO(zhoury): max frame rate is 23Hz at 27MHz
#define CLK_PIX            27000000
#define FRAME_FPS          25
#define FRAME_LENGTH_LINES 750    // defalut: 750(0x02EE)  0x00E2=226 0x05DC=1500
#define EXTRA_DELAY        0
#define LINE_LENGTH_PCK    1550   // ((CLK_PIX / FRAME_FPS -  EXTRA_DELAY)/ FRAME_LENGTH_LINES)

// Exposure/Integration register
#define COARSE_INTEGRATION_TIME       500  // default:16(0X0010)
#define FINE_INTEGRATION_TIME         0       // default:0
// #define FLASH_CONTROL         0x0138
// #define FLASH_CONTROL         0x0108
#define FLASH_CONTROL         0x0108
#define FLASH_PERIODS         0x0F00
#define GRR_CONTROL1          0x0081
// Gain relative register
// The digital channel gain format is xxxx.yyyyyyy where
// xxxx refers integer gain of 1 -15
// yyyyyyy refer to fractional gain from 0/128 to 127/128
#define GREEN1_INTEGER        1
#define GREEN1_FRACTIONAL     0
#define GREEN1_GAIN           (GREEN1_INTEGER << 7 | GREEN1_FRACTIONAL)  // default:128(0X80)
#define BLUE_INTEGER          1
#define BLUE_FRACTIONAL       56
#define BLUE_GAIN             (BLUE_INTEGER << 7 | BLUE_FRACTIONAL)      // default:128(0X80)
#define RED_INTEGER           1
#define RED_FRACTIONAL        56
#define RED_GAIN              (RED_INTEGER << 7 | RED_FRACTIONAL)        // default:128(0X80)
#define GREEN2_INTEGER        1
#define GREEN2_FRACTIONAL     0
#define GREEN2_GAIN           (GREEN2_INTEGER << 7 | GREEN2_FRACTIONAL)  // default:128(0X80)
#define GLOBAL_INTEGER        1
#define GLOBAL_FRACTIONAL     0
#define GLOBAL_GAIN           (GLOBAL_INTEGER << 7 | GLOBAL_FRACTIONAL)
// 2^coarse_gain * (1 + fine_gain/16)
#define COARSE_GAIN           0
#define FINE_GAIN             14
#define ANALOG_GAIN           (COARSE_GAIN << 4 | FINE_GAIN)

#define AR0141_WINDOW_X_MAX    1344
#define AR0141_WINDOW_Y_MAX    848
#define AR0141_IMG_WIDTH       1280   // x
#define AR0141_IMG_HEIGHT      720    // y

#define AR0141_X_ADDR_START    ((AR0141_WINDOW_X_MAX - AR0141_IMG_WIDTH) / 2)
#define AR0141_X_ADDR_END      (AR0141_X_ADDR_START + AR0141_IMG_WIDTH - 1)
#define AR0141_Y_ADDR_START    ((AR0141_WINDOW_Y_MAX - AR0141_IMG_HEIGHT) / 2)
#define AR0141_Y_ADDR_END      (AR0141_Y_ADDR_START + AR0141_IMG_HEIGHT - 1)

uint16_t AR0141_Parallel_seq[] = {
  0x3088, 0x8000,
  0x3086, 0x4558,
  0x3086, 0x6E9B,
  0x3086, 0x4A31,
  0x3086, 0x4342,
  0x3086, 0x8E03,
  0x3086, 0x2714,
  0x3086, 0x4578,
  0x3086, 0x7B3D,
  0x3086, 0xFF3D,
  0x3086, 0xFF3D,
  0x3086, 0xEA27,
  0x3086, 0x043D,
  0x3086, 0x1027,
  0x3086, 0x0527,
  0x3086, 0x1535,
  0x3086, 0x2705,
  0x3086, 0x3D10,
  0x3086, 0x4558,
  0x3086, 0x2704,
  0x3086, 0x2714,
  0x3086, 0x3DFF,
  0x3086, 0x3DFF,
  0x3086, 0x3DEA,
  0x3086, 0x2704,
  0x3086, 0x6227,
  0x3086, 0x288E,
  0x3086, 0x0036,
  0x3086, 0x2708,
  0x3086, 0x3D64,
  0x3086, 0x7A3D,
  0x3086, 0x0444,
  0x3086, 0x2C4B,
  0x3086, 0x8F01,
  0x3086, 0x4372,
  0x3086, 0x719F,
  0x3086, 0x4643,
  0x3086, 0x166F,
  0x3086, 0x9F92,
  0x3086, 0x1244,
  0x3086, 0x1646,
  0x3086, 0x4316,
  0x3086, 0x9326,
  0x3086, 0x0426,
  0x3086, 0x848E,
  0x3086, 0x0327,
  0x3086, 0xFC5C,
  0x3086, 0x0D57,
  0x3086, 0x5417,
  0x3086, 0x0955,
  0x3086, 0x5649,
  0x3086, 0x5F53,
  0x3086, 0x0553,
  0x3086, 0x0728,
  0x3086, 0x6C4C,
  0x3086, 0x0928,
  0x3086, 0x2C72,
  0x3086, 0xA37C,
  0x3086, 0x9728,
  0x3086, 0xA879,
  0x3086, 0x6026,
  0x3086, 0x9C5C,
  0x3086, 0x1B45,
  0x3086, 0x4845,
  0x3086, 0x0845,
  0x3086, 0x8826,
  0x3086, 0xBE8E,
  0x3086, 0x0127,
  0x3086, 0xF817,
  0x3086, 0x0227,
  0x3086, 0xFA17,
  0x3086, 0x095C,
  0x3086, 0x0B17,
  0x3086, 0x1026,
  0x3086, 0xBA5C,
  0x3086, 0x0317,
  0x3086, 0x1026,
  0x3086, 0xB217,
  0x3086, 0x065F,
  0x3086, 0x2888,
  0x3086, 0x9060,
  0x3086, 0x27F2,
  0x3086, 0x1710,
  0x3086, 0x26A2,
  0x3086, 0x26A3,
  0x3086, 0x5F4D,
  0x3086, 0x2808,
  0x3086, 0x1A27,
  0x3086, 0xFA84,
  0x3086, 0x69A0,
  0x3086, 0x785D,
  0x3086, 0x2888,
  0x3086, 0x8710,
  0x3086, 0x8C82,
  0x3086, 0x8926,
  0x3086, 0xB217,
  0x3086, 0x036B,
  0x3086, 0x9C60,
  0x3086, 0x9417,
  0x3086, 0x2926,
  0x3086, 0x8345,
  0x3086, 0xA817,
  0x3086, 0x0727,
  0x3086, 0xFB17,
  0x3086, 0x2945,
  0x3086, 0x8820,
  0x3086, 0x1708,
  0x3086, 0x27FA,
  0x3086, 0x5D87,
  0x3086, 0x108C,
  0x3086, 0x8289,
  0x3086, 0x170E,
  0x3086, 0x4826,
  0x3086, 0x9A28,
  0x3086, 0x884C,
  0x3086, 0x0B79,
  0x3086, 0x1730,
  0x3086, 0x2692,
  0x3086, 0x1709,
  0x3086, 0x9160,
  0x3086, 0x27F2,
  0x3086, 0x1710,
  0x3086, 0x2682,
  0x3086, 0x2683,
  0x3086, 0x5F4D,
  0x3086, 0x2808,
  0x3086, 0x1A27,
  0x3086, 0xFA84,
  0x3086, 0x69A1,
  0x3086, 0x785D,
  0x3086, 0x2888,
  0x3086, 0x8710,
  0x3086, 0x8C80,
  0x3086, 0x8A26,
  0x3086, 0x9217,
  0x3086, 0x036B,
  0x3086, 0x9D95,
  0x3086, 0x2603,
  0x3086, 0x5C01,
  0x3086, 0x4558,
  0x3086, 0x8E00,
  0x3086, 0x2798,
  0x3086, 0x170A,
  0x3086, 0x4A0A,
  0x3086, 0x4316,
  0x3086, 0x0B43,
  0x3086, 0x5B43,
  0x3086, 0x1659,
  0x3086, 0x4316,
  0x3086, 0x8E03,
  0x3086, 0x279C,
  0x3086, 0x4578,
  0x3086, 0x1707,
  0x3086, 0x279D,
  0x3086, 0x1722,
  0x3086, 0x5D87,
  0x3086, 0x1028,
  0x3086, 0x0853,
  0x3086, 0x0D8C,
  0x3086, 0x808A,
  0x3086, 0x4558,
  0x3086, 0x1708,
  0x3086, 0x8E01,
  0x3086, 0x2798,
  0x3086, 0x8E00,
  0x3086, 0x76A2,
  0x3086, 0x77A2,
  0x3086, 0x4644,
  0x3086, 0x1616,
  0x3086, 0x967A,
  0x3086, 0x2644,
  0x3086, 0x5C05,
  0x3086, 0x1244,
  0x3086, 0x4B71,
  0x3086, 0x759E,
  0x3086, 0x8B86,
  0x3086, 0x184A,
  0x3086, 0x0343,
  0x3086, 0x1606,
  0x3086, 0x4316,
  0x3086, 0x0743,
  0x3086, 0x1604,
  0x3086, 0x4316,
  0x3086, 0x5843,
  0x3086, 0x165A,
  0x3086, 0x4316,
  0x3086, 0x4558,
  0x3086, 0x8E03,
  0x3086, 0x279C,
  0x3086, 0x4578,
  0x3086, 0x7B17,
  0x3086, 0x0727,
  0x3086, 0x9D17,
  0x3086, 0x2245,
  0x3086, 0x5822,
  0x3086, 0x1710,
  0x3086, 0x8E01,
  0x3086, 0x2798,
  0x3086, 0x8E00,
  0x3086, 0x1710,
  0x3086, 0x1244,
  0x3086, 0x4B8D,
  0x3086, 0x602C,
  0x3086, 0x2C2C,
  0x3086, 0x2C00
};

uint16_t AR0141_Parallel_init[] = {
  0x301A, 0x10D8,  // RESET_REGISTER = 4312
  0x302A, 0x0009,  // VT_PIX_CLK_DIV = 6       default: 6    Range: 4-16
  0x302C, 0x0001,  // VT_SYS_CLK_DIV = 1       Range: 1,2,4,6,8,10,11,12,14,16
  0x302E, 0x0003,  // PRE_PLL_CLK_DIV = 3      default: 4 Range: 1-64
  0x3030, 0x001B,  // PLL_MULTIPLIER = 27      default: 66 Range: 32-384
  0x3036, 0x000C,  // OP_PIX_CLK_DIV = 12
  0x3038, 0x0002,  // OP_SYS_CLK_DIV = 2

  0x31AC, 0x0C0C,  // DATA_FORMAT_BITS = 3084
  0x31AE, 0x0301,  // SERIAL_FORMAT = 769
  0x30B0, 0x4000,  // DIGITAL_TEST = 0
  0x30BA, 0x002C,  // DIGITAL_CTRL = 300
  0x3002, AR0141_Y_ADDR_START,  // Y_ADDR_START  default: 64(0x0040)
  0x3004, AR0141_X_ADDR_START,  // X_ADDR_START  default: 18(0x0012)
  0x3006, AR0141_Y_ADDR_END,    // Y_ADDR_END    defalut: 791(0x0317)  0x01DF=479 0x02CF=719
  0x3008, AR0141_X_ADDR_END,    // X_ADDR_END    defalut: 1305(0x0519) 0x027F=639 0x04FF=1279
  0x300A, FRAME_LENGTH_LINES,  // FRAME_LENGTH_LINES
  0x300C, LINE_LENGTH_PCK,  // LINE_LENGTH_PCK
  0x3042, EXTRA_DELAY,  // EXTRA_DELAY

  0x3012, COARSE_INTEGRATION_TIME,  // COARSE_INTEGRATION_TIME
  0x3014, FINE_INTEGRATION_TIME,   // FINE_INTEGRATION_TIME
  0x3046, FLASH_CONTROL,  // FLASH_CONTROL
  // 0x3048, FLASH_PERIODS,  // FLASH_PERIODS
  0x30DA, 0x00FF,  // FLASH_PERIODS
  // disable embedded data
  0x3056, GREEN1_GAIN,  // GREEN1_GAIN   defalut: 128(0x0080)
  0x3058, BLUE_GAIN,    // BLUE_GAIN     defalut: 128(0x0080)
  0x305A, RED_GAIN,     // RED_GAIN      defalut: 128(0x0080)
  0x305C, GREEN2_GAIN,  // GREEN2_GAIN   defalut: 128(0x0080)
  // 0x305E, GLOBAL_GAIN,  // GLOBAL_GAIN   defalut: 128(0x0080)
  0x3060, ANALOG_GAIN,  // ANALOG_GAIN   defalut: 0(0x0000)
  0x3064, 0x1802,  // SMIA_TEST
  0x30A2, 0x0001,  // X_ODD_INC = 1
  0x30A6, 0x0001,  // Y_ODD_INC = 1
  0x3082, 0x000D,  // OPERATION_MODE_CTRL = 1
  0x3084, 0x000D,  // OPERATION_MODE_CTRL_CB = 1
  0x308C, 0x00B8,  // Y_ADDR_START_CB = 6
  0x308A, 0x0160,  // X_ADDR_START_CB = 24
  0x3090, 0x0290,  // Y_ADDR_END_CB = 847
  0x308E, 0x03DF,  // X_ADDR_END_CB = 1319
  0x30AA, 0x01ED,  // FRAME_LENGTH_LINES_CB = 1500
  0x303E, 0x13A1,  // LINE_LENGTH_PCK_CB = 1650
  0x3016, 0x000E,  // COARSE_INTEGRATION_TIME_CB = 45
  0x30AE, 0x0001,  // X_ODD_INC_CB = 1
  0x30A8, 0x0001,  // Y_ODD_INC_CB = 1
  0x3040, 0x0000,  // READ_MODE = 0
  0x30CE, GRR_CONTROL1,  // GRR_CONTROL1
  0x318E, 0x0000,  // HDR_MC_CTRL3 = 0
  0x3100, 0x0004,
  0x301A, 0x10D8   // RESET_REGISTER = 4316
};

CyU3PReturnStatus_t AR0141_SensorRead2B(uint8_t SlaveAddr, uint8_t HighAddr,
                                      uint8_t LowAddr, uint8_t *buf) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PI2cPreamble_t preamble;

  if (SlaveAddr != L_AR0141_ADDR_RD && SlaveAddr != R_AR0141_ADDR_RD) {
    sensor_err("I2C Slave address is not valid!\r\n");
    return 1;
  }
  preamble.length    = 4;
  preamble.buffer[0] = SlaveAddr - 1;
  preamble.buffer[1] = HighAddr;
  preamble.buffer[2] = LowAddr;
  preamble.buffer[3] = SlaveAddr;
  preamble.ctrlMask  = 0x0004;
  apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, 2, 0);
  if (apiRetStatus == CY_U3P_SUCCESS) {
    V034_delay(800);
  } else {
    sensor_err("R2B I2C read error\r\n");
  }
  return apiRetStatus;
}

CyU3PReturnStatus_t AR0141_SensorWrite2B(uint8_t SlaveAddr, uint8_t HighAddr,
                                       uint8_t LowAddr, uint8_t HighData, uint8_t LowData) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PI2cPreamble_t preamble;
  uint8_t Buf[2];

  if (SlaveAddr != L_AR0141_ADDR_WR && SlaveAddr != R_AR0141_ADDR_WR) {
    sensor_err("I2C Slave address is not valid!\r\n");
    return 1;
  }
  preamble.buffer[0] = SlaveAddr; /* Slave address: Write operation */
  preamble.buffer[1] = HighAddr;
  preamble.buffer[2] = LowAddr;
  preamble.length = 3;
  preamble.ctrlMask = 0x0000;
  Buf[0] = HighData;
  Buf[1] = LowData;
  apiRetStatus = CyU3PI2cTransmitBytes(&preamble, Buf, 2, 0);
  if (apiRetStatus == CY_U3P_SUCCESS) {
    V034_delay(800);  /* known issue for SDK I2C */
  } else {
    sensor_err("W2B I2C write error, reg addr: 0x%x, reg value: 0x%x\r\n", HighAddr << 8 | LowAddr,
              HighData << 8 | LowData);
  }
  return apiRetStatus;
}

CyU3PReturnStatus_t AR0141_RegisterWrite(uint8_t HighAddr, uint8_t LowAddr,
  uint8_t HighData, uint8_t LowData) {
  return AR0141_SensorWrite2B(SENSOR_ADDR_WR, HighAddr, LowAddr, HighData, LowData);
}

uint16_t AR0141_RegisterRead(uint8_t HighAddr, uint8_t LowAddr) {
  uint8_t buf[2];
  int16_t tmp;

  AR0141_SensorRead2B(SENSOR_ADDR_RD, HighAddr, LowAddr, buf);
  tmp = (buf[0] << 8) | buf[1];

  return (uint16_t)tmp;
}

static void AR0141_register_Check(uint8_t SlaveAddr) {
  uint16_t revision_number;
  static uint8_t read_Buf[2];

  sensor_dbg("========AR0141 Revision number read Check===========\r\n");
  AR0141_SensorRead2B(SlaveAddr, 0x30, 0x1A, read_Buf);
  revision_number = (read_Buf[0] << 8) | read_Buf[1];
  sensor_dbg("%s Sensor addr: 0x%x reg addr:0x%x value:0x%x \r\n",
            SlaveAddr == L_SENSOR_ADDR_RD ? "left": "right", SlaveAddr, 0x301A, revision_number);
  sensor_dbg("========AR0141 Revision number write Check===========\r\n");
  AR0141_SensorWrite2B(L_AR0141_ADDR_WR, 0x30, 0x1A, 0x10, 0xDC);
  CyU3PThreadSleep(10);

  sensor_dbg("========AR0141 Revision number read Check===========\r\n");
  AR0141_SensorRead2B(SlaveAddr, 0x30, 0x1A, read_Buf);
  revision_number = (read_Buf[0] << 8) | read_Buf[1];
  sensor_dbg("%s Sensor addr: 0x%x reg addr:0x%x value:0x%x \r\n",
             SlaveAddr == L_SENSOR_ADDR_RD ? "left": "right", SlaveAddr, 0x301A, revision_number);
}

static void AR0141_ChipID_Check(uint8_t SlaveAddr) {
  uint16_t ChipID;
  static uint8_t read_Buf[2];

  sensor_dbg("AR0141 ChipID Check\r\n");
  AR0141_SensorRead2B(SlaveAddr, 0x30, 0x00, read_Buf);
  ChipID = (read_Buf[0] << 8) | read_Buf[1];
  sensor_dbg("%s Sensor addr: 0x%x ID:0x%x \r\n", SlaveAddr == L_SENSOR_ADDR_RD ? "left": "right",
              SlaveAddr, ChipID);
}

void dump_AR0141_registers(uint8_t SlaveAddr) {
  int j = 0;
  BYTE AddrH, AddrL;
  BYTE buf[2];
  uint16_t reg_value;

  sensor_info("=================dump start=============================\r\n");
  for (j = 0; j < sizeof(AR0141_Parallel_init) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (AR0141_Parallel_init[j] >> 8) & 0xff;
    AddrL  = (AR0141_Parallel_init[j]) & 0xff;
    AR0141_SensorRead2B(SlaveAddr, AddrH, AddrL, buf);
    reg_value = (buf[0] << 8) | buf[1];
    sensor_info("%s image sensor register 0x%x value: 0x%x \r\n",
                SlaveAddr == L_AR0141_ADDR_RD ? "left": "right",
                AR0141_Parallel_init[j], reg_value);
  }
  sensor_info("=================dump   end=============================\r\n");
}

void AR0141_soft_reset(uint8_t SlaveAddr) {
  uint16_t Soft_Reset_Addr = 0x301A;
  uint16_t Soft_Reset_Value = 0x10D9;
  BYTE AddrH, AddrL, ValH, ValL;

  AddrH  = (Soft_Reset_Addr >> 8) & 0xff;
  AddrL  = (Soft_Reset_Addr) & 0xff;
  ValH   = (Soft_Reset_Value >> 8) & 0xff;
  ValL   = (Soft_Reset_Value) & 0xff;
  AR0141_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
  /* delay a little time for regitster setting */
  V034_delay(100*1000);

  // Soft_Reset_Value = 0x00;
  // ValH   = (Soft_Reset_Value >> 8) & 0xff;
  // ValL   = (Soft_Reset_Value) & 0xff;
  // V034_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
  // sensor_info("%s image sensor register 0x%x value: 0x%x \r\n",
  //               SlaveAddr == L_AR0141_ADDR_RD ? "left": "right",
  //               Soft_Reset_Addr], reg_value);
}

void AR0141_SetRegs(void) {
  int j;
  BYTE AddrH, AddrL, ValH, ValL;

  /* init Left image sensor*/
  sensor_dbg("init Left image sensor regitster.\r\n");
  AR0141_soft_reset(L_AR0141_ADDR_WR);
  AR0141_ChipID_Check(L_AR0141_ADDR_RD);
  // dump_AR0141_registers(L_AR0141_ADDR_RD);
  // seq register init
  for (j = 0; j < sizeof(AR0141_Parallel_seq) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (AR0141_Parallel_seq[j] >> 8) & 0xff;
    AddrL  = (AR0141_Parallel_seq[j]) & 0xff;
    ValH   = (AR0141_Parallel_seq[j + 1] >> 8) & 0xff;
    ValL   = (AR0141_Parallel_seq[j + 1]) & 0xff;

    AR0141_SensorWrite2B(L_AR0141_ADDR_WR, AddrH, AddrL, ValH, ValL);
  }
  // config register init
  for (j = 0; j < sizeof(AR0141_Parallel_init) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (AR0141_Parallel_init[j] >> 8) & 0xff;
    AddrL  = (AR0141_Parallel_init[j]) & 0xff;
    ValH   = (AR0141_Parallel_init[j + 1] >> 8) & 0xff;
    ValL   = (AR0141_Parallel_init[j + 1]) & 0xff;

    AR0141_SensorWrite2B(L_AR0141_ADDR_WR, AddrH, AddrL, ValH, ValL);
  }
  dump_AR0141_registers(L_AR0141_ADDR_RD);

  /* init Right image sensor*/
  sensor_dbg("init Right image sensor regitster.\r\n");
  AR0141_soft_reset(R_AR0141_ADDR_WR);
  // dump_AR0141_registers(R_AR0141_ADDR_RD);
  AR0141_ChipID_Check(R_AR0141_ADDR_RD);

  // seq register init
  for (j = 0; j < sizeof(AR0141_Parallel_seq) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (AR0141_Parallel_seq[j] >> 8) & 0xff;
    AddrL  = (AR0141_Parallel_seq[j]) & 0xff;
    ValH   = (AR0141_Parallel_seq[j + 1] >> 8) & 0xff;
    ValL   = (AR0141_Parallel_seq[j + 1]) & 0xff;

    AR0141_SensorWrite2B(R_AR0141_ADDR_WR, AddrH, AddrL, ValH, ValL);
  }
  // config register init
  for (j = 0; j < sizeof(AR0141_Parallel_init) / sizeof(uint16_t); j = j + 2) {
    AddrH  = (AR0141_Parallel_init[j] >> 8) & 0xff;
    AddrL  = (AR0141_Parallel_init[j]) & 0xff;
    ValH   = (AR0141_Parallel_init[j + 1] >> 8) & 0xff;
    ValL   = (AR0141_Parallel_init[j + 1]) & 0xff;

    AR0141_SensorWrite2B(R_AR0141_ADDR_WR, AddrH, AddrL, ValH, ValL);
  }
  dump_AR0141_registers(R_AR0141_ADDR_RD);
  /* reset two hard address to one addr, so we use SENSOR_ADDR_WR and SENSOR_ADDR_RD later  */
  v034_set_unified_addr();
}

void AR0141_stream_start(uint8_t SlaveAddr) {
  uint16_t Stream_Start_Addr  = 0x301A;
  uint16_t Stream_Start_Value = 0x10DC;
  BYTE AddrH, AddrL, ValH, ValL;

  AddrH  = (Stream_Start_Addr >> 8) & 0xff;
  AddrL  = (Stream_Start_Addr) & 0xff;
  ValH   = (Stream_Start_Value >> 8) & 0xff;
  ValL   = (Stream_Start_Value) & 0xff;
  AR0141_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
}

void AR0141_stream_stop(uint8_t SlaveAddr) {
  uint16_t Stream_Stop_Addr  = 0x301A;
  uint16_t Stream_Stop_Value = 0x10DC;
  BYTE AddrH, AddrL, ValH, ValL;

  AddrH  = (Stream_Stop_Addr >> 8) & 0xff;
  AddrL  = (Stream_Stop_Addr) & 0xff;
  ValH   = (Stream_Stop_Value >> 8) & 0xff;
  ValL   = (Stream_Stop_Value) & 0xff;
  AR0141_SensorWrite2B(SlaveAddr, AddrH, AddrL, ValH, ValL);
}
void AR0141_sensor_init(void) {
  sensor_dbg("sensor AR0141 register init \r\n");
  AR0141_SetRegs();
}
