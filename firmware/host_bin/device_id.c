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
#define CY_FX_UVC_XU_FLASH_RW 0x1200

// set to 1 for a bit of debug output

#define dbg printf
typedef unsigned char uint8_t;

// Flash only store sensor Device ID, limit to 32 byte.
// Flash RW len is 255
#define FLASH_RW_LEN  255
struct flash_struct_t {
  uint8_t Sensor_ID[32];
  uint8_t tmp[223];
};
static struct flash_struct_t value;
struct uvc_xu_control_query xu_query = {
  .unit       = 3,  // has to be unit 3
  .selector   = 1,
  .query      = UVC_SET_CUR,
  .size       = 4,
  .data       = (char *)(&value),
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
  dbg("failed to read flash: %s. (System code: %d) \n\r", err, res);
  return;
}

/**
 *  @brief      read device ID from sensor flash.
 *  @param[in]  fd: dev name.
 *  @return     NULL.
 */
void read_deviceID(int fd) {
  xu_query.selector = CY_FX_UVC_XU_FLASH_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = FLASH_RW_LEN;

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  printf("read device ID: %s\n", value.Sensor_ID);
}
/**
 *  @brief      write device ID to sensor flash.
 *  @param[in]  fd: dev name.
 *  @param[in]  str: flash control .
 *  @return     NULL.
 */
void write_deviceID(int fd, unsigned char* str) {
  xu_query.selector = CY_FX_UVC_XU_FLASH_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = FLASH_RW_LEN;

  // setting the read configuration
  memcpy(&(value.Sensor_ID), str, 32);
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  printf("write device ID: %s\n", str);
}

/**
 *  @brief      main.
 *  @param[in]  argc: cmd num.
 *  @param[in]  argv: cmd info.
 *  @return     NULL.
 */
void main(int argc, char** argv) {
  char* dev_name = "/dev/video1";
  char* devID_ptr = NULL;
  if (argc > 1) {
    dev_name = argv[1];
  }
  if (argc > 2) {
    devID_ptr = argv[2];
  }
  int v4l2_dev = open(dev_name, 0);

  if (v4l2_dev < 0) {
    dbg("open camera failed,err code:%d\n\r", v4l2_dev);
    exit(-1);
  }
  if (devID_ptr == NULL)
    read_deviceID(v4l2_dev);
  else
    write_deviceID(v4l2_dev, devID_ptr);

  close(v4l2_dev);
  return;
}
