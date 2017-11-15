/*
 $License:
    Copyright (C) 2015 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_icm20608.h
 *      @brief      An I2C-based driver for Invensense 6-axis ICM20608
 *      @details    This driver currently works for the following devices:
 *                  ICM20608
 */

#ifndef FIRMWARE_INCLUDE_INV_ICM20608_H_
#define FIRMWARE_INCLUDE_INV_ICM20608_H_

#include <cyu3types.h>

#define EMPL_TARGET_CYPRESS 1
#define ICM20608            1

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

/* Gyro Averaging Filters. */
enum gyro_avgf_e {
  INV_GYRO_1X_AVG = 0,
  INV_GYRO_2X_AVG,
  INV_GYRO_4X_AVG,
  INV_GYRO_8X_AVG,
  INV_GYRO_16X_AVG,
  INV_GYRO_32X_AVG,
  INV_GYRO_64X_AVG,
  INV_GYRO_128X_AVG,
  NUM_GYRO_AVG
};

/* Set up APIs */
int icm_init(void);
int icm_init_slave(void);
int icm_set_bypass(unsigned char bypass_on);
int icm_check_whoami(void);
unsigned char icm_check_whoami_debug(void);

/* Configuration APIs */
int icm_lp_accel_mode(unsigned short rate);
int icm_lp_motion_interrupt(unsigned short thresh, unsigned char time, unsigned short lpa_freq);
int icm_lp_6axis_mode(int gyro_avg, unsigned char enable);
int icm_set_int_level(unsigned char active_low);
int icm_set_int_latched(unsigned char enable);
int icm_set_dmp_state(unsigned char enable);
int icm_get_dmp_state(unsigned char *enabled);
int icm_get_gyro_lpf(unsigned short *lpf);
int icm_set_gyro_lpf(unsigned short lpf);
int icm_get_accel_lpf(unsigned short *lpf);
int icm_set_accel_lpf(unsigned short lpf);
int icm_get_gyro_avgf(unsigned short *avgf);
int icm_set_gyro_avgf(unsigned short avgf);
int icm_get_accel_avgf(unsigned short *avgf);
int icm_set_accel_avgf(unsigned short avgf);
int icm_get_gyro_fsr(unsigned short *fsr);
int icm_set_gyro_fsr(unsigned short fsr);
int icm_get_accel_fsr(unsigned char *fsr);
int icm_set_accel_fsr(unsigned char fsr);
int icm_get_watermark(unsigned char *threshold);
int icm_set_watermark(unsigned char threshold);
int icm_get_compass_fsr(unsigned short *fsr);
int icm_get_gyro_sens(float *sens);
int icm_get_accel_sens(unsigned short *sens);
int icm_get_sample_rate(unsigned short *rate);
int icm_set_sample_rate(unsigned short rate);
int icm_get_compass_sample_rate(unsigned short *rate);
int icm_set_compass_sample_rate(unsigned short rate);
int icm_get_fifo_config(unsigned char *sensors);
int icm_configure_fifo(unsigned char sensors);
int icm_get_power_state(unsigned char *power_on);
int icm_set_sensors(unsigned char sensors);
int icm_read_accel_bias(long *accel_bias);
int icm_set_gyro_bias_reg(long * gyro_bias);
int icm_set_accel_bias_reg(const long *accel_bias);

/* Data getter/setter APIs */
int icm_get_gyro_reg(short *data, unsigned long *timestamp);
int icm_get_accel_reg(short *data, unsigned long *timestamp);
int icm_get_compass_reg(short *data, unsigned long *timestamp);
int icm_get_temperature(long *data, unsigned long *timestamp);
int icm_get_sensor_reg(unsigned char *data, unsigned long *timestamp);

int icm_get_int_status(short *status);
int icm_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
                  unsigned char *sensors, unsigned char *more);
int icm_read_fifo_stream(unsigned short length, unsigned char *data, unsigned char *more);
int icm_reset_fifo(void);

int icm_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);
int icm_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);
int icm_load_firmware(unsigned short length, const unsigned char *firmware,
                      unsigned short start_addr, unsigned short sample_rate);
int icm_reg_dump(void);
int icm_read_reg(unsigned char reg, unsigned char *data);
int icm_write_reg(unsigned char reg, unsigned char data);
int icm_run_self_test(long *gyro, long *accel, unsigned char debug);
int icm_register_tap_cb(void (*func)(unsigned char, unsigned char));
int icm_get_sensor_data(long *gyro, long *accel, unsigned char debug);

#endif  // FIRMWARE_INCLUDE_INV_ICM20608_H_
