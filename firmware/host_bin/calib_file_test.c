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

// Define the Cypress FX USB3.0 camera uvc extension id
#define CY_FX_UVC_XU_CALIB_RW 0x1400
#define dbg printf
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

// CALIB RW len is 255
#define CALIB_RW_LEN  255
#define PAYLOAD_LEN (CALIB_RW_LEN - 7)
#define MAX_BUFFER_LEN (1024 * 64)

// TODO(huyuexiang) take care of alignment if using other kind of member!
typedef struct {
  uint8_t header[2];                 // header[0]=0xAA, header[1]=0x55
  uint8_t packet_total;              // indicates how many packets
  uint8_t packet_len;                // length of packet in byte, including check_sum
  uint8_t id;                        // start from 0 to 255
  uint8_t data[PAYLOAD_LEN];         // payload
  uint8_t check_sum[2];              // check sum from header[0] to data[PAYLOAD_LEN-1],
                                     // LSB first, MSB last
} calib_struct_t;

static calib_struct_t value;
struct uvc_xu_control_query xu_query = {
  .unit       = 3,  // has to be unit 3
  .selector   = 1,
  .query      = UVC_SET_CUR,
  .size       = 4,
  .data       = (char *)(&value),
};

static unsigned short const wCRC16Table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

void make_crc16(const uint8_t* pDataIn, int iLenIn, uint16_t* pCRCOut) {
  uint16_t wResult = 0;
  uint16_t wTableNo = 0;
  int i = 0;
  for ( i = 0; i < iLenIn; i++ ) {
    wTableNo = ((wResult & 0xff) ^ (pDataIn[i] & 0xff));
    wResult = ((wResult >> 8) & 0xff) ^ wCRC16Table[wTableNo];
  }
  *pCRCOut = wResult;
}
int verify_crc16(const uint8_t* pDataIn, int iLenIn) {
  uint8_t crc_buf[2] = {0};
  uint16_t check_sum = 0;
  make_crc16(pDataIn, iLenIn - 2, &check_sum);
  crc_buf[0] = check_sum & 0xFF;
  crc_buf[1] = (check_sum >> 8) & 0xFF;
  if (pDataIn[iLenIn - 2] == crc_buf[0]){
    if (pDataIn[iLenIn - 1] == crc_buf[1]) {
      return 0;
    }
  }
  return -1;
}
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
 *  @brief      load calibration file from local.
 *  @param[in]  path: file path.
 *  @param[out] fp_load: file descriptor
 *  @return     0:success or others:fail.
 */
int load_calib_file(const char *path, FILE **fp_load) {
  FILE *fp;

  if (path == NULL) {
    printf("ERR: Invalid calib file path!\n");
    return -1;
  }
  fp = fopen(path, "r");
  if (fp == NULL) {
    printf("ERR: Open file failed: %s!\n", path);
    return -2;
  }
  *fp_load = fp;
  return 0;
}

/**
 *  @brief      read calibration file and copy to buffer.
 *  @param[in]  buffer: .
 *  @param[in] fp_load: file descriptor
 *  @return     read length: positive success and others fail.
 */
int read_calib_from_file(uint8_t buffer[], FILE *fp) {
  int pos = 0;
  while (fgets(buffer + pos, PAYLOAD_LEN, fp) != NULL) {
    while (buffer[pos] != '\0') {
      pos++;
      if (pos > MAX_BUFFER_LEN) {
        printf("ERR: Input calib file too large!\n");
        return -1;
      }
    }
  }
  return pos;
}

/**
 *  @brief      write calibration file.
 *  @param[in]  buffer: .
 *  @param[in]  path: calibration file path
 *  @return     write length: positive success and others fail.
 */
int write_calib_to_file(uint8_t buffer[], const char *path) {
  FILE *fp;
  int i = 0;
  if (path == NULL || buffer == NULL) {
    printf("ERR: Invalid arguments!\n");
    return -1;
  }
  fp = fopen(path, "w");
  if (fp == NULL) {
    printf("ERR: Open file failed: %s!\n", path);
    return -2;
  }
  while (buffer[i] != '\0') {
    fputc(buffer[i++], fp);
  }
  fclose(fp);
  printf("write calib file done, size: %d\n", i);
  return i;
}

void read_calib_from_device(int fd, uint8_t buffer[], int *size) {
  uint8_t packet_num = 1;
  uint8_t packet_id = 0;
  uint16_t check_sum = 0;
  int read_len = 0;

  xu_query.selector = CY_FX_UVC_XU_CALIB_RW >> 8;
  xu_query.query = UVC_GET_CUR;
  xu_query.size = CALIB_RW_LEN;
  memset((uint8_t *)&value, 0, sizeof(calib_struct_t));

   do {
    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
      error_handle();
      *size = 0;
      return;
    }
    if ((value.header[0] != 0xAA) || (value.header[1] != 0x55)) {
      printf("wrong packet header from device: [0x%x] [0x%x]\n", value.header[0], value.header[1]);
      read_len = 0;
      break;
    }
    if (verify_crc16((uint8_t *)&value, value.packet_len)) {
      printf("%dth packet CRC check failed!\n", value.id);
      break;
    }
    packet_num = value.packet_total;
    packet_id = value.id;
    memcpy((uint8_t *)&buffer[packet_id * PAYLOAD_LEN], (uint8_t *)value.data, PAYLOAD_LEN);
    read_len += PAYLOAD_LEN;
    usleep(300000);
  } while (packet_id < packet_num - 1);
  *size = read_len;
}

void write_calib_to_device(int fd, uint8_t buffer[], int size) {
  uint8_t i, j; 
  uint8_t packet_num;
  uint8_t packet_id;
  uint16_t check_sum;

  packet_num = size /PAYLOAD_LEN + (size % PAYLOAD_LEN ? 1 : 0);
  packet_id = 0;
  value.header[0] = 0xAA;
  value.header[1] = 0x55;
  value.packet_len = CALIB_RW_LEN;

  xu_query.selector = CY_FX_UVC_XU_CALIB_RW >> 8;
  xu_query.query = UVC_SET_CUR;
  xu_query.size = CALIB_RW_LEN;

  for (i = 0; i < packet_num; i++) {
    packet_id = i;
    value.packet_total = packet_num;
    value.id = packet_id;

    // the last packet, we should take care that the length maybe less than PAYLOAD_LEN.
    // in that case, value.data[] must be init to '\0' first
    if (i == packet_num -1) {
      memset(value.data, '\0', PAYLOAD_LEN);
      memcpy(value.data, &buffer[i * PAYLOAD_LEN], size - i * PAYLOAD_LEN);
    } else {
      memcpy(value.data, &buffer[i * PAYLOAD_LEN], PAYLOAD_LEN);
    }
    make_crc16((uint8_t *)&value, value.packet_len - 2, &check_sum);
    value.check_sum[0] = check_sum & 0xFF;
    value.check_sum[1] = (check_sum >> 8) & 0xFF;

    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
      error_handle();
    }
    printf("write %dth packet to device\n", value.id);
    if (packet_id == 0) {
      // The first packet write should wait for a long time.
      sleep(3);
    } else {
      usleep(100000);
    }
  }
}

/**
 *  @brief      main.
 *  @param[in]  argc: cmd num.
 *  @param[in]  argv: cmd info.
 *  @return     NULL.
 */
int main(int argc, char** argv) {
  FILE *fp_calib_file;
  FILE *fp_ir_calib_file;
  char *dev_name = "/dev/video1";
  int ret, len, i;
  int v4l2_dev = 0;
  uint8_t buffer[MAX_BUFFER_LEN] = {0};
  uint8_t buffer_dev[MAX_BUFFER_LEN] = {0};
  const char *calib_file_recovery = "calib_file_recovery.yaml";

  // check the arguments
  if (argc < 2) {
    printf("Usage: ./program  device_name  [calib_file_path]\n\n");
    printf(" -- 1) With 'calib_file_path', program will write calib file to device.\n");
    printf(" -- 2) Without 'calib_file_path', program will read calib file from device,\
           and store it as calib_file_recovery.yaml in current dir.\n");
    printf("NOTICE: The sequence of calib file MUST be as same as above!\n\n");
    exit(0);
  }
  // open device
  dev_name = argv[1];
  v4l2_dev = open(dev_name, 0);
  if (v4l2_dev < 0) {
    dbg("open camera failed,err code:%d\n\r", v4l2_dev);
    goto err;
  }

  if (argc == 2) {
    read_calib_from_device(v4l2_dev, buffer, &ret);
    printf("read %d bytes from device\n", ret);
    ret = write_calib_to_file(buffer, calib_file_recovery);
    goto err;
  }

  // load calib file from local
  ret = load_calib_file(argv[2], &fp_calib_file);
  if (ret) {
    printf("can not load file %s\n", argv[2]);
    goto err;
  }

  // read calib file
  // read the 1st calib file
  ret = read_calib_from_file(buffer, fp_calib_file);
  if (ret >= 0) {
    len = ret;
    printf("read %d bytes from file %s\n", ret, argv[2]);
  }
  write_calib_to_device(v4l2_dev, buffer, len);
  sleep(1);
  memset(buffer, '\0', MAX_BUFFER_LEN);
  read_calib_from_device(v4l2_dev, buffer, &ret);
  printf("read %d bytes from device\n", ret);
  ret = write_calib_to_file(buffer, calib_file_recovery);

err:
  // close file
  if (fp_calib_file != NULL) {
    fclose(fp_calib_file);
  }

  if (v4l2_dev > 0) {
    close(v4l2_dev);
  }

  return 0;
}
