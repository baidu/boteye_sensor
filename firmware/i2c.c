/*
## Cypress USB 3.0 Platform source file
## ===========================
##
##  Copyright Cypress Semiconductor Corporation, 2010-2011,
##  All Rights Reserved
##  UNPUBLISHED, LICENSED SOFTWARE.
##
##  CONFIDENTIAL AND PROPRIETARY INFORMATION
##  WHICH IS THE PROPERTY OF CYPRESS.
##
##  Use of this file is governed
##  by the license agreement included in the file
##
##     <install>/license/license.txt
##
##  where <install> is the Cypress software
##  installation root directory path.
##
## ===========================
*/

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>
#include "include/i2c.h"
#include "include/debug.h"

CyU3PReturnStatus_t CyFx_I2cInit(void) {
  CyU3PI2cConfig_t i2cConfig;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Initialize and configure the I2C master module. */
  status = CyU3PI2cInit();
  if (status != CY_U3P_SUCCESS) {
    return status;
  }

  /* Start the I2C master block. The bit rate is set at 400KHz.
   * The data transfer is done via DMA. */
  CyU3PMemSet((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
  i2cConfig.bitRate    = I2C_SPEED;
  i2cConfig.busTimeout = 0xFFFFFFFF;
  i2cConfig.dmaTimeout = 0xFFFF;
  i2cConfig.isDma      = CyFalse;

  status = CyU3PI2cSetConfig(&i2cConfig, NULL);
  if (status != CY_U3P_SUCCESS) {
    sensor_err("I2C bus init failed\r\n");
    return status;
  }
  sensor_dbg("I2C bus init succeed\r\n");
  return status;
}

/* I2C read / write: single byte addressing mode */
CyU3PReturnStatus_t CyFxUsbI2cTransfer1(uint16_t byteAddress, uint8_t devAddr,
                                        uint16_t byteCount, uint8_t *buffer, CyBool_t isRead) {
  CyU3PI2cPreamble_t preamble;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  if (isRead) {
    /* Update the preamble information. */
    preamble.length = 3;
    preamble.buffer[0] = devAddr;
    preamble.buffer[1] = (uint8_t)(byteAddress);
    preamble.buffer[2] = (devAddr | 0x01);
    preamble.ctrlMask = 0x0002;

#if I2C_RW_SINGLE
    status = CyU3PI2cReceiveBytes(&preamble, buffer, 1, 0);
#else
    status = CyU3PI2cReceiveBytes(&preamble, buffer, byteCount, 0);
#endif
    if (status != CY_U3P_SUCCESS) {
      return status;
    }
  } else {
    /* Write : 1 byte addressing mode */
    /* Update the preamble information. */
    preamble.length = 2;
    preamble.buffer[0] = devAddr;
    preamble.buffer[1] = (uint8_t)(byteAddress);
    preamble.ctrlMask = 0x0000;

    status = CyU3PI2cTransmitBytes(&preamble, buffer, byteCount, 0);
    if (status != CY_U3P_SUCCESS) {
      return status;
    }

    /* Wait for the write to complete. */
    preamble.length = 1;
    status = CyU3PI2cWaitForAck(&preamble, 200);
    if (status != CY_U3P_SUCCESS) {
      return status;
    }
  }

  /* An additional delay seems to be required after receiving an ACK. */
  CyU3PThreadSleep(1);
  return CY_U3P_SUCCESS;
}


/* I2C read / write: 2 bytes addressing mode */
CyU3PReturnStatus_t CyFxUsbI2cTransfer2(uint16_t byteAddress, uint8_t devAddr, uint16_t byteCount,
                                        uint8_t *buffer, CyBool_t isRead) {
  CyU3PI2cPreamble_t preamble;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  if (isRead) {
    /* Update the preamble information. */
    preamble.length    = 4;
    preamble.buffer[0] = devAddr;
    preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
    preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
    preamble.buffer[3] = (devAddr | 0x01);
    preamble.ctrlMask  = 0x0004;

    status = CyU3PI2cReceiveBytes(&preamble, buffer, 1, 0);
    if (status != CY_U3P_SUCCESS) {
      return status;
    }
  } else {
    /* Write : 2 bytes addressing mode */
    /* Update the preamble information */
    preamble.length    = 3;
    preamble.buffer[0] = devAddr;
    preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
    preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
    preamble.ctrlMask  = 0x0000;

    status = CyU3PI2cTransmitBytes(&preamble, buffer, 1, 0);
    if (status != CY_U3P_SUCCESS) {
      return status;
    }

    /* Wait for the write to complete. */
    preamble.length = 1;
    status = CyU3PI2cWaitForAck(&preamble, 200);
    if (status != CY_U3P_SUCCESS) {
      return status;
    }
  }

  /* An additional delay seems to be required after receiving an ACK. */
  CyU3PThreadSleep(1);

  return CY_U3P_SUCCESS;
}


void mdelay(unsigned long num_ms) {
  CyU3PBusyWait(1000 * num_ms);
}

void get_tick_count(unsigned long *count) {
  return;
}

/* I2C read / write: single byte addressing version */
int Sensors_I2C_ReadReg(unsigned char Address, unsigned char RegisterAddr,
                        unsigned short RegisterLen, uint8_t *RegisterValue) {
#if I2C_RW_SINGLE
  int index = 0;

  for (index = 0; index < RegisterLen; index++) {
    CyFxUsbI2cTransfer1(RegisterAddr + index, Address, 1, RegisterValue + index, 1);
  }
#else
  CyFxUsbI2cTransfer1(RegisterAddr, Address, RegisterLen, RegisterValue, 1);
#endif
  return CY_U3P_SUCCESS;
}

int Sensors_I2C_WriteReg(unsigned char Address, unsigned char RegisterAddr,
                         unsigned short RegisterLen, uint8_t *RegisterValue) {
#if I2C_RW_SINGLE
  int index = 0;
  int time = 800;
  for (index = 0; index < RegisterLen; index++) {
    CyFxUsbI2cTransfer1(RegisterAddr + index, Address, RegisterLen, RegisterValue + index, 0);
    while (time--);
  }
#else
  CyFxUsbI2cTransfer1(RegisterAddr, Address, RegisterLen, RegisterValue, 0);
#endif
  return CY_U3P_SUCCESS;
}
