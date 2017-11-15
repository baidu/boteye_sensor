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

#ifndef FIRMWARE_INCLUDE_EXTENSION_UNIT_H_
#define FIRMWARE_INCLUDE_EXTENSION_UNIT_H_

/* function declaration */
void EU_Rqts_imu_rw(uint8_t bRequest);
void EU_Rqts_imu_burst(uint8_t bRequest);
void EU_Rqts_cam_reg(uint8_t bRequest);
void EU_Rqts_spi_flash(uint8_t bRequest);
extern void EU_Rqts_soft_version(uint8_t bRequest);
extern void EU_Rqts_hard_version(uint8_t bRequest);
extern void EU_Rqts_firmware_flag(uint8_t bRequest);
extern CyU3PReturnStatus_t CyFxFlashProgEraseSector(CyBool_t isErase, uint8_t sector, uint8_t *wip);
extern CyU3PReturnStatus_t CyFxFlashProgSpiInit(uint16_t pageLen);
CyU3PReturnStatus_t CyFxFlashProgSpiTransfer(uint16_t  pageAddress, uint16_t  byteCount,
                                             uint8_t  *buffer, CyBool_t  isRead);
/* volatile declaration */
#define IMU_BURST_LEN 17
extern volatile char last_imu[IMU_BURST_LEN];
/*
 * Must open IMU data loop sample now, otherwise, there is a known bug which
 * xp_sensor_log will show "ioctl(fd, VIDIOC_S_FMT, &format) failed", but this
 * problem is also missing after restarted software serveral times. If we close
 * IMU_LOOP_SAMPLE, It is helpful for sensor lowpower.
 * */
#define IMU_LOOP_SAMPLE

#endif  // FIRMWARE_INCLUDE_EXTENSION_UNIT_H_
