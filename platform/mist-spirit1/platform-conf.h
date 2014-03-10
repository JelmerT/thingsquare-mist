/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *  \file
 *      platform-conf.h
 *   \author
 *      Marcus Lunden <marcus@thingsquare.com>>
 *   \desc
 *      platform dependent configurations
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include <inttypes.h>
#include <string.h>

#define F_CPU                   32000000ul

#define LEDS_CONF_LED1		      1     /* green */
#define LEDS_CONF_LED2		      2     /* orange */
#define LEDS_CONF_LED3		      4     /* red */
#define LEDS_CONF_LED4		      8     /* blue */
#define LEDS_CONF_LED5		      5     /* yellow */

//#define FIXED_NET_ADDRESS             1
//#define NET_ADDR_A                    0x2001
//#define NET_ADDR_B                    0xdb8
//#define NET_ADDR_C                    0xbbbb
//#define NET_ADDR_D                    0xabcd

/* define ticks/second for slow and fast clocks. Notice that these should be a
  power of two, eg 64,128,256,512 etc, for efficiency as POT's can be optimized
  well. */
#define CLOCK_CONF_SECOND               128
// One tick: 85.33 us
#define RTIMER_ARCH_SECOND              32768
#define RTIMER_CLOCK_LT(a,b)     ((signed short)((a)-(b)) < 0)
typedef unsigned long clock_time_t;
typedef unsigned long long rtimer_clock_t;

#define CC_CONF_REGISTER_ARGS          0
#define CC_CONF_FUNCTION_POINTER_ARGS  1
#define CC_CONF_FASTCALL
#define CC_CONF_VA_ARGS                1
#define CC_CONF_INLINE                 inline

#define CCIF
#define CLIF

typedef uint8_t         u8_t;
typedef uint16_t        u16_t;
typedef uint32_t        u32_t;
typedef  int32_t        s32_t;
typedef unsigned short  uip_stats_t;

#define MULTICHAN_CONF_SET_CHANNEL(x)
#define MULTICHAN_CONF_READ_RSSI(x) 0

#endif /* __PLATFORM_CONF_H__ */
