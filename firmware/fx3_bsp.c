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
#include "include/fx3_bsp.h"
#include "include/debug.h"
#include "include/uvc.h"
#include "include/tlc59116.h"

int hardware_version_num = 0x00;

char *Baidu_ProductDscr[16] = {
  "Baidu_Robotics_vision_XP/XP2",
  "Baidu_Robotics_vision_XP2S",
  "Baidu_Robotics_vision_XP3",
  "Baidu_Robotics_vision_XP3S",
  "Baidu_Robotics_vision_XPIRL",
  "Baidu_Robotics_vision_XPIRL2",
  "Baidu_Robotics_vision_XPIRL3",
  "Baidu_Robotics_vision_undefined_0111",
  "Baidu_Robotics_vision_undefined_1000",
  "Baidu_Robotics_vision_undefined_1001",
  "Baidu_Robotics_vision_undefined_1010",
  "Baidu_Robotics_vision_undefined_1011",
  "Baidu_Robotics_vision_undefined_1100",
  "Baidu_Robotics_vision_undefined_1101",
  "Baidu_Robotics_vision_undefined_1110",
  "Baidu_Robotics_vision_undefined_1111"
};
/**
 *  @brief      Application Error Handler.
 *  @param[in]  apiRetStatus    API return status.
 *  @return     no return.
 */
void CyFxAppErrorHandler(CyU3PReturnStatus_t apiRetStatus) {
/* This function is hit when we have hit a critical application error. This is not
   expected to happen, and the current implementation of this function does nothing
   except stay in a loop printing error messages through the UART port.

   This function can be modified to take additional error handling actions such
   as cycling the USB connection or performing a warm reset.
*/
  int i = 0;

  for (i = 0; i < 5; i++) {
    sensor_err("App Error handler...\r\n");
    sensor_led_blink(LED_OFF);
    CyU3PThreadSleep(500);
    sensor_led_blink(LED_ON);
    CyU3PThreadSleep(500);
  }
}
/**
 *  @brief      FX3 whole GPIO module Init.
 *  @param[] NULL.
 *  @return     NULL.
 */
void fx3_gpio_module_init(void) {
  CyU3PGpioClock_t             gpioClock;
  CyU3PGpioSimpleConfig_t      gpioConfig;
  CyU3PReturnStatus_t          apiRetStatus;

  /* Init the GPIO module */
  gpioClock.fastClkDiv = 2;
  gpioClock.slowClkDiv = 2;
  gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
  gpioClock.clkSrc     = CY_U3P_SYS_CLK;
  gpioClock.halfDiv    = 0;

  /* Initialize Gpio interface */
  apiRetStatus = CyU3PGpioInit(&gpioClock, CyFx_GpioIntrCb);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("GPIO Init failed, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* CTL pins are restricted and cannot be configured using I/O matrix configuration function,
   * must use GpioOverride to configure it. */
  apiRetStatus = CyU3PDeviceGpioOverride(CAMERA_OE_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(CAMERA_RST_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(CAMERA_EXPOSURE_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(CAMERA_SADR_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(CAMERA_STANDBY_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(SENSOR_LED_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(IMU_AD0_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(IMU_NCS_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(CAMPWR_CONTROL_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(HARD_VERSION_A0, CyTrue) | \
                 CyU3PDeviceGpioOverride(HARD_VERSION_A1, CyTrue) | \
                 CyU3PDeviceGpioOverride(HARD_VERSION_A2, CyTrue) | \
                 CyU3PDeviceGpioOverride(HARD_VERSION_A3, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO1, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO2, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO3, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO4, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO5, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO6, CyTrue) | \
                 CyU3PDeviceGpioOverride(UNUSED_GPIO7, CyTrue);

  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("GPIO Override failed, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* init output GPIO high level after init */
  gpioConfig.outValue    = CyTrue;
  gpioConfig.driveLowEn  = CyTrue;
  gpioConfig.driveHighEn = CyTrue;
  gpioConfig.inputEn     = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
  apiRetStatus           = CyU3PGpioSetSimpleConfig(CAMERA_SADR_GPIO, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(CAMERA_STANDBY_GPIO, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(SENSOR_LED_GPIO, &gpioConfig);

  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("Camera GPIO Set Config Error, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* init GPIO low level after init */
  gpioConfig.outValue    = CyFalse;
  apiRetStatus           = CyU3PGpioSetSimpleConfig(CAMERA_RST_GPIO, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(CAMERA_EXPOSURE_GPIO, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(IMU_NCS_GPIO, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(IMU_AD0_GPIO, &gpioConfig);
                           CyU3PGpioSetSimpleConfig(CAMPWR_CONTROL_GPIO, &gpioConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("IMU GPIO Set Config Error, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Chip select: 0 is SPI, 1 is I2C */
  CyU3PGpioSetValue(IMU_NCS_GPIO, CyTrue);
  /* AD0 is LSB of IMU chip address, when ADO is 0, the IMU I2C address is 0xD0 */
  CyU3PGpioSetValue(IMU_AD0_GPIO, CyFalse);

  /* init input GPIO */
  gpioConfig.outValue    = CyTrue;
  gpioConfig.inputEn     = CyTrue;
  gpioConfig.driveLowEn  = CyFalse;
  gpioConfig.driveHighEn = CyFalse;
  apiRetStatus           = CyU3PGpioSetSimpleConfig(HARD_VERSION_A0, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(HARD_VERSION_A1, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(HARD_VERSION_A2, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(HARD_VERSION_A3, &gpioConfig);

  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("hardware version detect GPIO Set Error, Error = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  sensor_dbg("FX3 GPIO init finish\r\n");
  hardware_version_num = hadrware_version_detect();
  sensor_type = (enum SensorType)hardware_version_num;
  sensor_info("hardware_version_num: 0x%x \r\n", hardware_version_num);

  gpioConfig.inputEn     = CyTrue;
  apiRetStatus           = CyU3PGpioSetSimpleConfig(UNUSED_GPIO1, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(UNUSED_GPIO2, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(UNUSED_GPIO3, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(UNUSED_GPIO4, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(UNUSED_GPIO5, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(UNUSED_GPIO6, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(UNUSED_GPIO7, &gpioConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("UNUSED GPIO  Set Error, Error = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  gpioConfig.driveLowEn  = CyTrue;
  gpioConfig.driveHighEn = CyTrue;
  gpioConfig.inputEn     = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
  if (sensor_type == XPIRL2 || sensor_type == XPIRL3) {
    /* AR0141 CMOS_OE ENABLE(acitve LOW) */
    gpioConfig.outValue  = CyFalse;
  } else {
    /* MT9V024/034 CMOS_OE ENABLE(active HIGH)*/
    gpioConfig.outValue  = CyTrue;
  }
  apiRetStatus = CyU3PGpioSetSimpleConfig(CAMERA_OE_GPIO, &gpioConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("IMU GPIO Set Config Error, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  if (sensor_type == XPIRL2 || sensor_type == XPIRL3)
    fx3_LIMA_GPIO_init();
}
/**
 *  @brief      FX3 LIMA GPIO Init For XPIRL2/3.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void fx3_LIMA_GPIO_init(void) {
  CyU3PGpioSimpleConfig_t      gpioConfig;
  CyU3PReturnStatus_t          apiRetStatus;

  apiRetStatus = CyU3PDeviceGpioOverride(LIMA_CATL1_GPIO, CyTrue) | \
                 CyU3PDeviceGpioOverride(FRAME_VALID, CyTrue) | \
                 CyU3PDeviceGpioOverride(LEDOUT1_IN, CyTrue) | \
                 CyU3PDeviceGpioOverride(IR_CTL_GPIO, CyTrue);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("GPIO Override failed, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  /* init output GPIO Low level after init */
  gpioConfig.outValue    = CyFalse;
  gpioConfig.driveLowEn  = CyTrue;
  gpioConfig.driveHighEn = CyTrue;
  gpioConfig.inputEn     = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
  apiRetStatus           = CyU3PGpioSetSimpleConfig(LIMA_CATL1_GPIO, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(IR_CTL_GPIO, &gpioConfig);

  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("Infrared GPIO Set Config Error, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  sensor_dbg("LEDOUT1_IN and  Frame Valid init\r\n");
    /* init input GPIO */
  gpioConfig.outValue    = CyFalse;
  gpioConfig.inputEn     = CyTrue;
  gpioConfig.driveLowEn  = CyFalse;
  gpioConfig.driveHighEn = CyFalse;
  // gpioConfig.intrMode    = CY_U3P_GPIO_INTR_POS_EDGE;
  gpioConfig.intrMode    = CY_U3P_GPIO_INTR_BOTH_EDGE;
  apiRetStatus           = CyU3PGpioSetSimpleConfig(LEDOUT1_IN, &gpioConfig) | \
                           CyU3PGpioSetSimpleConfig(FRAME_VALID, &gpioConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("Camera GPIO Set Config Error, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
}

/* Callback for GPIO related interrupts */
void CyFx_GpioIntrCb(uint8_t gpioId) {
  CyBool_t gpioValue = CyFalse;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint32_t flag;
  static uint64_t flash_Rising_count = 0;
  static uint64_t VD_Failing_count = 0;
  static uint64_t VD_Rising_count = 0;

  if (gpioId == LEDOUT1_IN) {
    apiRetStatus = CyU3PGpioGetValue(gpioId, &gpioValue);
    if (apiRetStatus == CY_U3P_SUCCESS) {
      /* Check status of the pin is rising edge*/
      if (gpioValue == CyTrue) {
        flash_Rising_count++;
        IR_LED_OFF();
        if (XPIRL2_LIMA_open)
          CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_GPIO1_INTR_CB_EVENT_FLAG, CYU3P_EVENT_OR);
      }
    }
  } else if (gpioId == FRAME_VALID) {
    apiRetStatus = CyU3PGpioGetValue(gpioId, &gpioValue);
    if (apiRetStatus == CY_U3P_SUCCESS) {
      /* Check status of the pin is rising edge*/
      if (gpioValue == CyTrue) {
        VD_Rising_count++;
      } else {
        VD_Failing_count++;
        if ((XPIRLx_IR_ctrl.Set_infrared_mode == 1 || XPIRLx_IR_ctrl.Set_structured_mode == 1)
            && VD_Failing_count % XPIRLx_IR_ctrl.RGB_IR_period == 0) {
          if (CyU3PEventGet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, \
            &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
            IR_image_trigger = CyTrue;
            IR_LED_ON();
            // For debug
            // CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_DEBUG_INTR_CB_EVENT_FLAG, CYU3P_EVENT_OR);
            if (XPIRL2_LIMA_open) {
              CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_GPIO0_INTR_CB_EVENT_FLAG, CYU3P_EVENT_OR);
            }
          }
        }
      }
    }
  } else {
    // Maybe can't output log message success as running in interrupt context.
    sensor_err("unkown gpio interrupt!\r\n");
  }
}
/**
 *  @brief      TLC59116/TLC59108 power ON for XPIRLx.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void tlc_power_ON(void) {
  CyU3PGpioSetValue(IR_CTL_GPIO, CyTrue);
}

/**
 *  @brief      TLC59116/TLC59108 power OFF for XPIRLx.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void tlc_power_OFF(void) {
  CyU3PGpioSetValue(IR_CTL_GPIO, CyFalse);
}

/**
 *  @brief      IR_LED_ON for XPIRLx.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void IR_LED_ON(void) {
  CyU3PGpioSetValue(LIMA_CATL1_GPIO, CyTrue);
}

/**
 *  @brief      IR_LED_OFF for XPIRLx.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void IR_LED_OFF(void) {
  CyU3PGpioSetValue(LIMA_CATL1_GPIO, CyFalse);
}

/**
 *  @brief      v034/v024 Chip set Unified I2C Address.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void v034_set_unified_addr(void) {
  CyU3PGpioSetValue(CAMERA_SADR_GPIO, CyFalse);
}

/**
 *  @brief      v034/v024 power turn off.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void v034_power_off(void) {
  CyU3PGpioSetValue(CAMPWR_CONTROL_GPIO, CyFalse);
}

/**
 *  @brief      v034/v024 power turn on.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void v034_power_on(void) {
  CyU3PGpioSetValue(CAMPWR_CONTROL_GPIO, CyTrue);
}

/**
 *  @brief      Sensor Chip Gpio Init.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void sensor_gpio_init(void) {
  CyU3PReturnStatus_t apiRetStatus;

  /* Reset the Gpio to low */
  apiRetStatus = CyU3PGpioSetValue(CAMERA_RST_GPIO, CyFalse);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("camera reset GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }
  if (sensor_type == XPIRL2 || sensor_type == XPIRL3) {
    /* AR0141 CMOS_OE ENABLE(acitve LOW) */
    apiRetStatus = CyU3PGpioSetValue(CAMERA_OE_GPIO, CyFalse);
  } else {
    /* MT9V024/034 CMOS_OE ENABLE(active HIGH)*/
    apiRetStatus = CyU3PGpioSetValue(CAMERA_OE_GPIO, CyTrue);
  }
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("SENSOR_OE_GPIO_GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }

  /* SADR,HIGH,diff I2C address for these two sensors */
#ifdef SINGLE_V034_WITH_DIFF_ADDR
  sensor_info("Set two image sensor at diff address\r\n");
  apiRetStatus = CyU3PGpioSetValue(CAMERA_SADR_GPIO, CyTrue);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("Camera sadr GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }
#else
  apiRetStatus = CyU3PGpioSetValue(CAMERA_SADR_GPIO, CyFalse);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("Camera sadr GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }
#endif

  /* STANDBY,keep high, into active mode */
  apiRetStatus = CyU3PGpioSetValue(CAMERA_STANDBY_GPIO, CyFalse);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("Camera standby GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }

  v034_power_on();
  // must let tlc59116 power on as reset pin of tlc59116 is same with camera sensor.
  tlc_power_ON();
  CyU3PThreadSleep(100);
  /* camera chip hard reset */
  camera_hard_reset();

  /* LED off */
  sensor_led_blink(LED_OFF);

  sensor_dbg("sensor GPIO init done! \r\n");
  return;
}

/**
 *  @brief      sensor power mode.
 *  @param[in]  SENSOR_POWER_MODE: active or standby.
 *  @return     NULL.
 */
void sensor_set_power_mode(enum SENSOR_POWER_MODE state) {
  CyU3PReturnStatus_t apiRetStatus;

  apiRetStatus = CyU3PGpioSetValue(CAMERA_STANDBY_GPIO, state);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("Camera standby GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }
}
/**
 *  @brief      sensor led blink.
 *  @param[in]  LED_TYPE: ON or OFF.
 *  @return     NULL.
 */
void sensor_led_blink(enum LED_TYPE led_value) {
  CyU3PReturnStatus_t apiRetStatus;

  if (led_value == LED_OFF)
    apiRetStatus = CyU3PGpioSetValue(SENSOR_LED_GPIO, CyFalse);
  else
    apiRetStatus = CyU3PGpioSetValue(SENSOR_LED_GPIO, CyTrue);

  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("LED_GPIO Set ON/OFF Error, Error Code = 0x%x\r\n", apiRetStatus);
  }
}

/**
 *  @brief      Camera hard reset 10ms.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void camera_hard_reset(void) {
  CyU3PReturnStatus_t apiRetStatus;

  /* Reset the Gpio to low */
  apiRetStatus = CyU3PGpioSetValue(CAMERA_RST_GPIO, CyFalse);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("camera reset GPIO Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }
  /* Wait for some time: 20 camera chip sysclk cycle */
  CyU3PThreadSleep(10);

  /* Set Reset Gpio to high */
  apiRetStatus = CyU3PGpioSetValue(CAMERA_RST_GPIO, CyTrue);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("camera reset gpio Set Value Error, Error Code = 0x%x\r\n", apiRetStatus);
    return;
  }
}

/**
 *  @brief      FX3 GPIO matrix config.
 *  @param[]    NULL.
 *  @return     NULL.
 */
void fx3_IO_matrix_config(void) {
  CyU3PReturnStatus_t apiRetStatus;
  CyU3PIoMatrixConfig_t io_cfg;

  /* Configure the IO matrix for the device. */
  io_cfg.useUart          = CyTrue;   /* Uart is enabled for logging. */
  io_cfg.useI2C           = CyTrue;   /* I2C is used for the sensor interface. */
  io_cfg.useI2S           = CyFalse;
  io_cfg.useSpi           = CyTrue;

  io_cfg.isDQ32Bit = CyFalse;
  io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
  /* GPIOs 50[I2S_CLK] and 52[I2S_WS] are enabled. */
  io_cfg.gpioSimpleEn[0]  = 0x00000000;
  io_cfg.gpioSimpleEn[1]  = 0x00140000;
  io_cfg.gpioComplexEn[0] = 0;
  io_cfg.gpioComplexEn[1] = 0;

  apiRetStatus = CyU3PDeviceConfigureIOMatrix(&io_cfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("FX3 configure IO Matrix Error, Error Code = 0x%x\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
}

int hadrware_version_detect(void) {
  CyBool_t gpioValue = CyFalse;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  char version_gpio[4] = {HARD_VERSION_A0, HARD_VERSION_A1, HARD_VERSION_A2, HARD_VERSION_A3};
  int i = 0;
  int hardware_version_num = 0;

  for (i = 0; i < 4; i++) {
    /* Get the status of the pin */
    apiRetStatus = CyU3PGpioGetValue(version_gpio[i], &gpioValue);
    if (apiRetStatus == CY_U3P_SUCCESS) {
      /* Check status of the pin */
      if (gpioValue == CyTrue) {
        hardware_version_num |= 1 << i;
      } else {
        hardware_version_num &= ~(1 << i);
      }
    }
  }
  return hardware_version_num;
}
