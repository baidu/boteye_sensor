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
#include <cyu3system.h>
#include <cyu3types.h>
#include <cyu3error.h>
#include <cyu3externcstart.h>
#include <cyu3externcend.h>
#include <include/kfifo.h>
#include <include/debug.h>

/* static function declaration */
static void kfifo_copy_in(struct __kfifo *fifo, const void *src, unsigned int len,
                          unsigned int off);
static void kfifo_copy_out(struct __kfifo *fifo, void *dst, unsigned int len, unsigned int off);

unsigned int kfifo_unused(struct __kfifo *fifo) {
  return(fifo->esize - kfifo_used(fifo));
}

unsigned int kfifo_used(struct __kfifo *fifo) {
  if (fifo->in >= fifo->out)
    return(fifo->in - fifo->out);
  else
    return(fifo->esize - fifo->out + fifo->in);
}

void kfifo_free(struct __kfifo *fifo) {
  fifo->in = 0;
  fifo->out = 0;
  fifo->esize = 0;
  fifo->kfifo_flag = 0;
  fifo->data = NULL;
  CyU3PMutexPut(&fifo->lock);
  CyU3PMutexDestroy(&fifo->lock);
}

/**
 *  @brief      initialize a fifo using a preallocated buffer.
 *  @fifo: the fifo to assign the buffer.
 *  @buffer: the preallocated buffer to be used
 *  @size: the size of the internal buffer, this have to be a power of 2
 *  @return     0 if successful, otherwise an error code.
 */
int kfifo_init(struct __kfifo *fifo, void *buffer, size_t esize) {
  uint32_t status;

  fifo->in = 0;
  fifo->out = 0;
  fifo->esize = esize;
  fifo->kfifo_flag |= KFIFO_IS_READY;
  fifo->data = buffer;

  status = CyU3PMutexCreate(&fifo->lock, CYU3P_NO_INHERIT);
  if (status != CY_U3P_SUCCESS)
    return -1;
  return 0;
}

static void kfifo_copy_in(struct __kfifo *fifo, const void *src, unsigned int len,
                          unsigned int off) {
  unsigned int size = fifo->esize;
  unsigned int l;

  l = min(len, size - off);

  CyU3PMemCopy((uint8_t *)(fifo->data + off), (uint8_t *)src, l);
  CyU3PMemCopy((uint8_t *)(fifo->data), (uint8_t *)(src + l), len - l);
}

unsigned int kfifo_in(struct __kfifo *fifo, const void *buf, unsigned int len) {
  unsigned int l;

  l = kfifo_unused(fifo);
  if (len > l)
    len = l;

  kfifo_copy_in(fifo, buf, len, fifo->in);
  /* update fifo->in count */
  fifo->in += len;
  if (fifo->in >= fifo->esize)
    fifo->in = fifo->in - fifo->esize;
  return len;
}

static void kfifo_copy_out(struct __kfifo *fifo, void *dst, unsigned int len, unsigned int off) {
  unsigned int size = fifo->esize;
  unsigned int l;

  l = min(len, size - off);

  CyU3PMemCopy(dst, fifo->data + off, l);
  CyU3PMemCopy(dst + l, fifo->data, len - l);
}

unsigned int kfifo_out(struct __kfifo *fifo, void *buf, unsigned int len) {
  unsigned int l;

  l = fifo->in - fifo->out;
  if (len > l)
    len = l;

  kfifo_copy_out(fifo, buf, len, fifo->out);
  /* update fifo->out count */
  fifo->out += len;
  if (fifo->out >= fifo->esize)
    fifo->out = fifo->out - fifo->esize;

  return len;
}
