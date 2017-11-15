/*
 ## Cypress FX3 Camera Kit header file (camera_ptzcontrol.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
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
/* This header file defines cyfxuvcdscr declaration */

#ifndef FIRMWARE_INCLUDE_CYFXUVCDSCR_H_
#define FIRMWARE_INCLUDE_CYFXUVCDSCR_H_

/* variable declaration */
extern const uint8_t CyFxUSBDeviceDscr[];               /* USB 2.0 Device descriptor. */
extern const uint8_t CyFxUSBDeviceDscrSS[];             /* USB 3.0 device descriptor. */

extern const uint8_t CyFxUSBDeviceQualDscr[];           /* USB 2.0 Device Qual descriptor. */
extern const uint8_t CyFxUSBBOSDscr[];                  /* USB 3.0 BOS descriptor. */

extern const uint8_t CyFxUSBFSConfigDscr[];             /* Full Speed Config descriptor. */
extern const uint8_t CyFxUSBHSConfigDscr[];             /* High Speed Config descriptor. */
extern const uint8_t CyFxUSBSSConfigDscr[];             /* USB 3.0 config descriptor. */

extern const uint8_t CyFxUSBStringLangIDDscr[];         /* String 0 descriptor. */
extern const uint8_t CyFxUSBManufactureDscr[];          /* Manufacturer string descriptor. */
extern uint8_t CyFxUSBProductDscr[];                    /* Product string descriptor. */
extern uint8_t CyFxUSBSerialNumberDscr[];               /* Product string descriptor. */

/* function declaration */
extern void update_soft_version_dscr(void);
extern void update_hard_version_dscr(void);
#endif  // FIRMWARE_INCLUDE_CYFXUVCDSCR_H_
