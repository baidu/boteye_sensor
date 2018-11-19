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

// Variable Declare
// Flash only store sensor Device ID, limit to 32 byte.
struct flash_struct_t {
  uint8_t Sensor_ID[32];
  uint8_t tmp[223];
};

// CALIB RW len is 255
#define CALIB_RW_LEN  255
#define PAYLOAD_LEN (CALIB_RW_LEN - 7)
typedef struct {
  uint8_t header[2];                 // header[0]=0xAA, header[1]=0x55
  uint8_t packet_total;              // indicates how many packets
  uint8_t packet_len;                // length of packet
  uint8_t id;                        // start from 0 to 255
  uint8_t data[PAYLOAD_LEN];         // payload
  uint8_t check_sum[2];              // check sum from header[0] to data[PAYLOAD_LEN-1],
                                     // LSB first, MSB last
} calib_struct_t;

/* function declaration */
void EU_Rqts_imu_rw(uint8_t bRequest);
void EU_Rqts_imu_burst(uint8_t bRequest);
void EU_Rqts_cam_reg(uint8_t bRequest);
void EU_Rqts_spi_flash(uint8_t bRequest);
extern void EU_Rqts_soft_version(uint8_t bRequest);
extern void EU_Rqts_hard_version(uint8_t bRequest);
extern void EU_Rqts_firmware_flag(uint8_t bRequest);
extern void EU_Rqts_IR_control(uint8_t bRequest);
extern void EU_Rqts_flash_RW(uint8_t bRequest);
extern void EU_Rqts_debug_RW(uint8_t bRequest);
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
#define  DEVICE_MSG_ADDR      (0x40000 / 0x100)
#define DEVICE_CALIB_ADDR     (0x50000 / 0x100)

#endif  // FIRMWARE_INCLUDE_EXTENSION_UNIT_H_
