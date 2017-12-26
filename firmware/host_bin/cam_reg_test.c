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

// Define camera uvc extension id
#define CY_FX_UVC_XU_CAM_REG 0x0b00

#define SENSOR_VGA

// set to 1 for a bit of debug output
#if 1
#define dbg printf
#else
#define dbg(fmt, ...)
#endif

static  __u8 value[64] = {0};
struct uvc_xu_control_query xu_query = {
  .unit       = 3,  // has to be unit 3
  .selector   = 1,  // TD
  .query      = UVC_SET_CUR,
  .size       = 4,  // TD
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
 *  @brief      write the camera sensor register value
 *  @param[in]  fd: dev name.
 *  @param[in]  regAddr: camera sensor register address.
 *  @param[in]  regVal: new camera sensor register value.
 *  @return     NULL.
 */
void write_cam_reg(int fd, short regaddr, short regval) {
  xu_query.selector = CY_FX_UVC_XU_CAM_REG >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 4;

  value[0] = regaddr >> 8;     /* higher address part*/
  value[1] = regaddr & 0xFF;   /* lower address part*/
  value[2] = regval >> 8;      /* higher byte */
  value[3] = regval & 0xFF;    /* lower byte */

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  printf("write cam reg 0x%x, val 0x%x (%d)\n", regaddr, regval, regval);
}

/**
 *  @brief      read the camera register value
 *  @param[in]  fd: dev name.
 *  @param[in]  regAddr: camera register address.
 *  @return     NULL.
 */
short read_cam_reg(int fd, short regaddr) {
  unsigned short regval = 0;

  xu_query.selector = CY_FX_UVC_XU_CAM_REG >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = 4;

  value[0] = (regaddr >> 8) | 0x80;     /* higher address part*/
  value[1] = regaddr & 0xFF;   /* lower address part*/

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }
  /*delay 1ms for I2C transfers time*/
  usleep(1000);
  xu_query.query = UVC_GET_CUR;

  value[0] = regaddr >> 8;     /* higher address part*/
  value[1] = regaddr & 0xFF;   /* lower address part*/
  value[2] = 0;                /* higher byte */
  value[3] = 0;                /* lower byte */

  if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
    error_handle();
  }

  regval = (value[2] << 8) + value[3];
  printf("read cam reg 0x%x, val: 0x%x (%d) [higher lower byte = %x %x]\n",
         regaddr, regval, regval, value[2], value[3]);

  return regval;
}
/**
 *  @brief      read v4l2 format
 *  @param[in]  fd: dev name.
 *  @return     NULL.
 */
int read_v4l2_format(int fd) {
  struct v4l2_format format;
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int r = ioctl(fd, VIDIOC_G_FMT, &format);
  if (r < 0) {
    printf("ioctl(fd, VIDIOC_G_FMT, &format) failed. code %d dev %d\n", r, fd);
  } else {
    printf(" current v4l2 width x height = %d x %d\n", format.fmt.pix.width, format.fmt.pix.height);
  }
  return r;
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

  short regval;

#ifdef SENSOR_VGA
  write_cam_reg(v4l2_dev, 0x01, 0x0038);  // COL_WINDOW_START_CONTEXTA_REG
  write_cam_reg(v4l2_dev, 0x04, 640);     // COL_WINDOW_SIZE_CONTEXTA_REG 640
#else
  write_cam_reg(v4l2_dev, 0x01, 0x0001);  // COL_WINDOW_START_CONTEXTA_REG
  write_cam_reg(v4l2_dev, 0x04, 752);     // COL_WINDOW_SIZE_CONTEXTA_REG 640
#endif  // SENSOR_VGA
  write_cam_reg(v4l2_dev, 0x03, 480);     // ROW_WINDOW_SIZE_CONTEXTA_REG 480

  regval = read_cam_reg(v4l2_dev, 0x03);
  regval = read_cam_reg(v4l2_dev, 0x04);

  // sleep(1);
  // sendflashcmd(v4l2_dev, 'R');
  // sleep(1);
  // sendflashcmd(v4l2_dev, 'W');

  read_v4l2_format(v4l2_dev);

  close(v4l2_dev);
  return;
}
