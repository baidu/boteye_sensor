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

#ifndef  FIRMWARE_INCLUDE_DEBUG_H_
#define  FIRMWARE_INCLUDE_DEBUG_H_

#include "console_color.h"  //NOLINT
#include <cyu3system.h>     //NOLINT
#include <stdint.h>         //NOLINT

#define __PRINT(color, format, args...) \
do { \
  CyU3PDebugPrint(2, color "%s[%d]: ", __func__, __LINE__);\
  CyU3PDebugPrint(2, format, ##args); \
} while (0)
#define __print(format, args...)    CyU3PDebugPrint(4, format, ##args)

#define unlikely(x)         (x)

extern int debug_level;

#define DEBUG_DBG_LEVEL     0x01
#define DEBUG_INFO_LEVEL    0x02
#define DEBUG_DUMP_BUFF     0x04

#define DBG_COLOR   FC_GREEN"DBG=> "FC_WHITE
#define INFO_COLOR  FC_YELLOW"INFO=> "FC_WHITE
#define ERR_COLOR   FC_RED"ERR=> "FC_WHITE

#ifndef SENSOR_RELEASE
#define sensor_dbg(format, args...) \
do { \
if (unlikely(debug_level & DEBUG_DBG_LEVEL)) \
  __PRINT(DBG_COLOR, format, ##args); \
} while (0)

#define sensor_info(format, args...) \
do { \
if (unlikely(debug_level & DEBUG_INFO_LEVEL)) \
  __PRINT(INFO_COLOR, format, ##args); \
} while (0)
#else
#define sensor_dbg(format, args...)
#define sensor_info(format, args...)
#endif

#define sensor_err(format, args...) __PRINT(ERR_COLOR, format, ##args)

#define sensor_printf(format, args...) \
do { \
  CyU3PDebugPrint(2, "%s[%d]: ", __func__, __LINE__);\
  CyU3PDebugPrint(2, format, ##args); \
} while (0)

static inline void dump_buf(char *info, uint8_t *buf, uint32_t len) {
  int i;

  if (unlikely(!(debug_level & DEBUG_DUMP_BUFF)))
    return;

  __print("%s\n", info);
  for (i = 0; i < len ; i++) {
    __print("%02x ", buf[i]);
    if ((i + 1) % 16 == 0) {
    __print("\n");
  }
  }
  __print("\n");
}

#endif  // FIRMWARE_INCLUDE_DEBUG_H_
