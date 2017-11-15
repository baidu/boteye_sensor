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
#include "../include/xp_sensor_firmware_version.h"

// Define camera uvc extension id
#define CY_FX_UVC_XU_SVER_RW                                 0x0d00
#define CY_FX_UVC_XU_HVER_RW                                 0x0f00

char *Baidu_ProductDscr[16] = {
  "Baidu_Robotics_vision_XP/XP2",
  "Baidu_Robotics_vision_XP2S",
  "Baidu_Robotics_vision_XP3",
  "Baidu_Robotics_vision_XP3S",
  "Baidu_Robotics_vision_undefined_0100",
  "Baidu_Robotics_vision_undefined_0101",
  "Baidu_Robotics_vision_undefined_0110",
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

// set to 1 for a bit of debug output
#if 1
#define __print printf
#define dbg printf
#else
#define dbg(fmt, ...)
#endif

static  __u8 value[64] = {0};

struct uvc_xu_control_query xu_query = {
  .unit       = 3,  // has to be unit 3
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
  dbg("failed to read FIRMWARE_VERSION: %s. (System code: %d) \n\r", err, res);
  return;
}

/**
 *  @brief      update software vesion.
 *  @param[out] NULL.
 *  @return     NULL.
 */
short read_soft_version(int fd) {
  unsigned short regval = 0;

  xu_query.selector = CY_FX_UVC_XU_SVER_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = strlen(FIRMWARE_VERSION);

  regval = ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query);
  if (regval) {
    error_handle();
  }
  printf("software version: %s\r\n", value);

  return regval;
}

/**
 *  @brief      read hardware version info.
 *  @param[out] NULL.
 *  @return     NULL.
 */
short read_hard_version(int fd) {
  unsigned short regval = 0;

  xu_query.selector = CY_FX_UVC_XU_HVER_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = 4;

  regval = ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query);
  if (regval) {
    error_handle();
  }
  int hardware_version_num = value[0] << 24 | value[1] << 16 | value[2] << 8 | value[3];
  printf("hardware version num: %d\r\n", hardware_version_num);
  printf("hardware version: %s\r\n", Baidu_ProductDscr[hardware_version_num]);

  return regval;
}
void main(int argc, char** argv) {
  char* dev_name = "/dev/video1";
  char* version_type = "s";
  if (argc > 1) {
    dev_name = argv[1];
  }
  if (argc > 2) {
    version_type = argv[2];
  }
  int v4l2_dev = open(dev_name, 0);

  if (v4l2_dev < 0) {
    dbg("open camera failed,err code:%d\n\r", v4l2_dev);
    exit(-1);
  }
  if (version_type[0] == 's')
    read_soft_version(v4l2_dev);
  else if (version_type[0] == 'h')
    read_hard_version(v4l2_dev);
  else
    printf("error: unknown version type request!\r\n");
  close(v4l2_dev);
  return;
}
