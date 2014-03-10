/*
 * Copyright (c) 2013, Thingsquare AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "contiki.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define STACK_POISON_BYTE 0xAA
#define STACK_GUARD_SIZE 0x30

extern void* __stack;
extern void* _end;

/*---------------------------------------------------------------------------*/
uint16_t
stack_avail_max_available(void)
{
  return (void*)(&__stack) - (void*)&_end;
}
/*---------------------------------------------------------------------------*/
uint16_t
stack_avail_register(void)
{
  uint16_t sp;
  __asm__("mov.w r1,%0" : "=r" (sp));
  return sp;
}
/*---------------------------------------------------------------------------*/
uint16_t
stack_avail_depth_now(void)
{
  return (void*)&__stack - (void*)stack_avail_register();
}
/*---------------------------------------------------------------------------*/
uint16_t
stack_avail_depth_left(void)
{
  return stack_avail_max_available() - stack_avail_depth_now();
}
/*---------------------------------------------------------------------------*/
uint16_t
stack_avail_estimate_unused(void)
{
  uint16_t sp, end;
  uint16_t unused = 0;

  sp = stack_avail_register();
  end = (void*)&_end;

  sp -= STACK_GUARD_SIZE;

  while(end < sp) {
    uint8_t d = *((char*)end);
    if(d != STACK_POISON_BYTE) {
      break;
    }
    unused++;
    end++;
  }
  return unused;
}
/*---------------------------------------------------------------------------*/
void
stack_avail_init(void)
{
  uint16_t sp, end;
  uint16_t poisoned = 0;

  sp = stack_avail_register();
  end = (void*)&_end;

  sp -= STACK_GUARD_SIZE;

  while(end < sp) {
    *((char*)end) = STACK_POISON_BYTE;
    end++;
    poisoned++;
  }

  PRINTF("stack_avail_init: sp %p\n", (void*)stack_avail_register());
  PRINTF("stack_avail_init: end %p\n", (void*)&_end);
  PRINTF("stack_avail_init: poisoned %u bytes\n", poisoned);
  PRINTF("stack_avail_init: unused %u bytes\n", stack_avail_estimate_unused());
}
/*---------------------------------------------------------------------------*/
