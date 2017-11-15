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

#ifndef FIRMWARE_INCLUDE_KFIFO_H_
#define FIRMWARE_INCLUDE_KFIFO_H_

/*
 * Note about locking : There is no locking required until only * one reader
 * and one writer is using the fifo and no kfifo_reset() will be * called
 *  kfifo_reset_out() can be safely used, until it will be only called
 * in the reader thread.
 *  For multiple writer and one reader there is only a need to lock the writer.
 * And vice versa for only one writer and multiple reader there is only a need
 * to lock the reader.
 */

struct __kfifo {
  unsigned int  in;
  unsigned int  out;
  unsigned int  esize;
  unsigned int  kfifo_flag;
  CyU3PMutex  lock;         /* Mutex used for data write/read safely */
  void    *data;
};

/* macro declaration */
#define NULL        ((void *)0)
#define min(a, b)    ((a < b) ? a : b)
#define size_t      unsigned int
#define KFIFO_IS_READY   0x0001
#define KFIFO_IS_FULL    0x0002
#define KFIFO_IS_START   0x0004
/* function declaration */
extern unsigned int kfifo_unused(struct __kfifo *fifo);
extern unsigned int kfifo_used(struct __kfifo *fifo);
extern void kfifo_free(struct __kfifo *fifo);
extern int kfifo_init(struct __kfifo *fifo, void *buffer, size_t esize);
extern unsigned int kfifo_in(struct __kfifo *fifo, const void *buf, unsigned int len);
extern unsigned int kfifo_out(struct __kfifo *fifo, void *buf, unsigned int len);

#endif  // FIRMWARE_INCLUDE_KFIFO_H_
