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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <fcntl.h>
#include <time.h>

// Define the Leopard Imaging USB3.0 camera uvc extension id
#define CY_FX_UVC_XU_REG_BURST  (0x0c00)
#define CY_FX_UVC_XU_REG_RW     (0x0e00)

// Define the mapping for IMU registers
enum ImuReg {
  CONFIG        = 0x011A,
  GYRO_CONFIG   = 0x011B,
  ACCEL_CONFIG  = 0x011C,
  ACCEL_CONFIG2 = 0x011D,
  LP_MODE_CFG   = 0x011E,
  RAW_GYRO      = 0x0043,
  RAW_ACCEL     = 0x003B
};

// set to 1 for a bit of debug output
#if 1
#define dbg printf
#else
#define dbg(fmt, ...)
#endif

static  __u8 value[64] = {0};
struct uvc_xu_control_query xu_query = {
  .unit       = 3,
  .selector   = 1,
  .query      = UVC_SET_CUR,
  .size       = 4,
  .data       = value,
};
/**
 *  @brief      error handle.
 *  @param[out] NULL.
 *  @return     NULL.
 */
void error_handle() {
  int res = errno;
  const char *err;

  switch (res) {
  case ENOENT:
    err = "Extension unit or control not found";
    break;
  case ENOBUFS:
    err = "Buffer size does not match control size";
    break;
  case EINVAL:
    err = "Invalid request code";
    break;
  case EBADRQC:
    err = "Request not supported by control";
    break;
  default:
    err = strerror(res);
    break;
  }

  dbg("failed to read IMU: %s. (System code: %d) \n\r", err, res);

  return;
}
/**
 *  @brief      calc time diff between start and end.
 *  @param[in]  start, end time.
 *  @return     NULL.
 */
static struct timespec diff_time(struct timespec* start, struct timespec* end) {
  struct timespec out;

  if ((end->tv_nsec - start->tv_nsec) < 0) {
    out.tv_sec = end->tv_sec - start->tv_sec - 1;
    out.tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
  } else {
    out.tv_sec = end->tv_sec - start->tv_sec;
    out.tv_nsec = end->tv_nsec - start->tv_nsec;
  }

  return out;
}

/**
 *  @brief      pull IMU data by extension unit
 *  @param[in]  fd: dev name.
 *  @return     NULL.
 */
void IMU_DataAccess(int fd) {
  static int iteration = 0;
  struct timespec start, end, out;

  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
  int ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query);
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);

  out = diff_time(&start, &end);

  if (ret != 0) {
    error_handle();
  }

  // Get raw IMU readings
  int k;
  int16_t raw_readings[6];  // Use int16_t to correctly represent
  for (k = 0; k < 6; k++) {
    raw_readings[k] = (value[k * 2] << 8) + value[k * 2 + 1];
  }
  // printf raw data
#if 0
  printf("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
         value[0], value[1], value[2], value[3], value[4], value[5]);
  printf("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
         value[6], value[7], value[8], value[9], value[10], value[11]);
  printf("0x%x, 0x%x, 0x%x, 0x%x\n",
         value[12], value[13], value[14], value[15]);
#endif
  // Convert IMU readings
  float readings[6];
  const float accel_scale = 2*9.8;  // +-2g
  const float gyro_scale = 2000;  // 2000 dps
  for (k = 3; k < 6; k++) {
    readings[k] = accel_scale * raw_readings[k] / 32768;
  }
  for (k = 0; k < 3; k++) {
    readings[k] = gyro_scale * raw_readings[k] / 32768;
  }

  int timestamp = value[12] << 24 | value[13] << 16 | value[14] << 8 | value[15];
  printf("[%d] (%lu, %lu (%f)) acc+gyro: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f timestamp:%d",
       iteration++, out.tv_sec, out.tv_nsec, (float)out.tv_nsec/1000000.0f,
       readings[3], readings[4], readings[5],
       readings[0], readings[1], readings[2], timestamp);
  printf(" [OK]\n");
}

/**
 *  @brief      write the imu register value
 *  @param[in]  fd: dev name.
 *  @param[in]  regAddr: imu register address.
 *  @param[in]  regVal: new imu register value.
 *  @return     NULL.
 */
void SensorRegWrite(int fd, int regAddr, int regVal) {
  xu_query.selector = CY_FX_UVC_XU_REG_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 5;

  // setting the read configuration
  value[0] = 1;  // indicate for write
  value[1] = (regAddr >> 8) & 0xff;
  value[2] = regAddr & 0xff;
  value[3] = (regVal >> 8) & 0xff;
  value[4] = regVal & 0xff;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  printf("REG[0x%x] write 0x%x\n", regAddr, regVal);
}

/**
 *  @brief      read the imu register value
 *  @param[in]  fd: dev name.
 *  @param[in]  regAddr: imu register address.
 *  @return     NULL.
 */
int SensorRegRead(int fd, int regAddr) {
  int regVal = 0;

  xu_query.selector = CY_FX_UVC_XU_REG_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 5;

  // setting the read configuration
  value[0] = 0;  // indicate for read
  value[1] = (regAddr >> 8) & 0xff;
  value[2] = regAddr & 0xff;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  sleep(1);
  // getting the value
  xu_query.query = UVC_GET_CUR;
  value[0] = 0;
  value[3] = 0;
  value[4] = 0;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }

  regVal = (value[3] << 8) + value[4];
  printf("REG[0x%x] = 0x%x\n", regAddr, regVal);
  return regVal;
}

/**
 *  @brief      main.
 *  @param[in]  argc: cmd num.
 *  @param[in]  argv: cmd info.
 *  @return     NULL.
 */
void main(int argc, char** argv) {
  char* dev_name = "/dev/video1";
  if (argc > 1) {
    dev_name = argv[1];
  }
  int v4l2_dev = open(dev_name, 0);

  if (v4l2_dev < 0) {
    dbg("open camera failed,err code:%d\n\r", v4l2_dev);
    exit(-1);
  }

  xu_query.selector = CY_FX_UVC_XU_REG_BURST >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = 17;

  int counter = 0;
  while (counter < 1000000) {
    IMU_DataAccess(v4l2_dev);
    usleep(2* 1000);
    /* sleep a little bit */
    /* get another */
    counter++;
  }

  // Register read/write example
  SensorRegRead(v4l2_dev, CONFIG);
  SensorRegRead(v4l2_dev, GYRO_CONFIG);
  SensorRegRead(v4l2_dev, ACCEL_CONFIG);
  SensorRegRead(v4l2_dev, ACCEL_CONFIG2);
  SensorRegRead(v4l2_dev, LP_MODE_CFG);

  // SensorRegWrite(v4l2_dev, CONFIG, 0xff);

  close(v4l2_dev);
  return;
}
