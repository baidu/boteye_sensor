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
#define CY_FX_UVC_XU_SPI_FLASH 0x0a00

// set to 1 for a bit of debug output
#if 1
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
  dbg("failed to read IMU: %s. (System code: %d) \n\r", err, res);
  return;
}

/**
 *  @brief      calc time diff between start and end.
 *  @param[in]  start, end .
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
 *  @brief      write flash control cmd to sensor with.
 *  @param[in]  fd: dev name.
 *  @param[in]  cmd: flash control cmd.
 *  @return     NULL.
 */
void sendflashcmd(int fd, unsigned char cmd) {
  xu_query.selector = CY_FX_UVC_XU_SPI_FLASH >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 1;

  // setting the read configuration
  value[0] = cmd;
  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  printf("send cmd %c\n", cmd);
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

  sendflashcmd(v4l2_dev, 'E');
  // sleep(1);
  // sendflashcmd(v4l2_dev, 'R');
  // sleep(1);
  // sendflashcmd(v4l2_dev, 'W');

  close(v4l2_dev);
  return;
}
