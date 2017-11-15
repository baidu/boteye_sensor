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

#ifndef FIRMWARE_INCLUDE_I2C_H_
#define FIRMWARE_INCLUDE_I2C_H_
/****************************** Includes *****************************/
#include <cyu3types.h>
/****************************** Defines *******************************/
#define SENSORS_I2C               I2C2

#define I2C_SPEED                 (400000)
#define I2C_OWN_ADDRESS           (0x00)

/* I2C READ/WRITE MODE: 1: single 0: burst */
#define I2C_RW_SINGLE  0
/* I2C Data rate */
#define CY_FX_USBI2C_I2C_BITRATE  (100000)

#define I2C_Config() I2cMaster_Init();

void I2cMaster_Init(void);
void Set_I2C_Retry(unsigned short ml_sec);
unsigned short Get_I2C_Retry();
CyU3PReturnStatus_t CyFx_I2cInit(void);
void get_tick_count(unsigned long *count);
void mdelay(unsigned long num_ms);
int Sensors_I2C_ReadReg(unsigned char Address, unsigned char RegisterAddr,
                        unsigned short RegisterLen, uint8_t *RegisterValue);

int Sensors_I2C_WriteReg(unsigned char Address, unsigned char RegisterAddr,
                         unsigned short RegisterLen, uint8_t *RegisterValue);
#endif  // FIRMWARE_INCLUDE_I2C_H_
