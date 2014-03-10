/**
 * \file
 * Functions for reading and writing flash ROM.
 * \author Adam Dunkels <adam@sics.se>
 */

/* Copyright (c) 2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * $Id: flash.c,v 1.3 2010/11/15 21:52:54 adamdunkels Exp $
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include "contiki.h"
#include <stdlib.h>

#include "dev/flash.h"
#include "dev/watchdog.h"

#define FLASH_TIMEOUT 30
#define FLASH_REQ_TIMEOUT 150

#define INFOMEM_LO (unsigned long)0x1800
#define INFOMEM_HI (unsigned long)0x1a00

static uint16_t sfrie;

static unsigned long flash_addr;
static uint16_t flash_data, flash_ret;

#ifdef __IAR_SYSTEMS_ICC__
/* Read/write using MSP430X assembler implementation: not yet supported for IAR */
#define WITH_ASM 0
#else
/* Read/write using MSP430X assembler implementation */
#define WITH_ASM 1
#define asmv(arg) __asm__ __volatile__(arg)
#endif

/*---------------------------------------------------------------------------*/
#if WITH_ASM
void
flash_read_(void)
{
  asmv("movx %0, r14" : : "r" (flash_addr) : );
  asmv("mov r15, r13");
  asmv("and #15, r13");
  asmv("clrc");
  asmv(".rpt #5");
  asmv("rrcx.a r13");
  asmv("bisx.a r14, r13");
  asmv("movx @r13, %0" : "=r" (flash_ret) : : );
}
#else /* WITH_ASM */
void
flash_read_(void)
{
  flash_ret = *((unsigned short*)flash_addr);
}
#endif /* WITH_ASM */
/*---------------------------------------------------------------------------*/
#if WITH_ASM
void
flash_write_(void)
{
  asmv("movx %0, r14" : : "r" (flash_addr) : );
  asmv("mov r15, r13");
  asmv("and #15, r13");
  asmv("clrc");
  asmv(".rpt #5");
  asmv("rrcx.a r13");
  asmv("bisx.a r14, r13");
  asmv("movx %0, 0(r13)" : : "r" (flash_data) : );
}
#else /* WITH_ASM */
void
flash_write_(void)
{
  *((unsigned short*)flash_addr) = flash_data; /* program Flash word */
}
#endif /* WITH_ASM */
/*---------------------------------------------------------------------------*/
void
flash_setup(void)
{
  /* disable all interrupts to protect CPU
     during programming from system crash */
  dint();

  /* Clear interrupt flag1. */
  SFRIFG1 = 0;
  /* The IFG1 = 0; statement locks up contikimac - not sure if this
     statement needs to be here at all. I've removed it for now, since
     it seems to work, but leave this little note here in case someone
     stumbles over this code at some point. */

  /* Stop watchdog. */
  watchdog_stop();

  /* disable all NMI-Interrupt sources */
  sfrie = SFRIE1;
  SFRIE1 = 0x00;
}
/*---------------------------------------------------------------------------*/
void
flash_done(void)
{
  /* Enable interrupts. */
  SFRIE1 = sfrie;
  eint();
  watchdog_start();
}
/*---------------------------------------------------------------------------*/
static void
unlock_infomem(void)
{
  FCTL4 = 0xa500;
  FCTL3 = 0xa540;
}
/*---------------------------------------------------------------------------*/
static void
lock_infomem(void)
{
  FCTL3 = 0xa540;
  FCTL4 = 0xa580;
}
/*---------------------------------------------------------------------------*/
void
flash_clear(unsigned long addr)
{
  uint8_t r;

  /* If addr is in infomem, we need to unlock it first. */
  if(addr >= INFOMEM_LO && addr <= INFOMEM_HI) {
    unlock_infomem();
  }

  FCTL3 = 0xa500;               /* Lock = 0 */
  while(FCTL3 & 0x0001) {
    r++;  /* Wait for BUSY = 0, not needed
	     unless run from RAM */
  }
  FCTL1 = 0xa502;               /* ERASE = 1 */
  flash_addr = addr;
  flash_data = 0;
  flash_write_();
  FCTL1 = 0xa500;               /* ERASE = 0 automatically done?! */
  FCTL3 = 0xa510;               /* Lock = 1 */

  if(addr >= INFOMEM_LO && addr <= INFOMEM_HI) {
    lock_infomem();
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
flash_read(unsigned long addr)
{
  flash_addr = addr;
  flash_read_();
  return flash_ret;
}
/*---------------------------------------------------------------------------*/
void
flash_write(unsigned long addr, uint16_t data)
{
  uint8_t r;

  /* If addr is in infomem, we need to unlock it first. */
  if(addr >= INFOMEM_LO && addr <= INFOMEM_HI) {
    unlock_infomem();
  }

  FCTL3 = 0xa500;              /* Lock = 0 */
  while(FCTL3 & 0x0001) {
    r++; /* Wait for BUSY = 0, not needed unless
	    run from RAM */
  }
  FCTL1 = 0xa540;              /* WRT = 1 */
  flash_addr = addr;
  flash_data = data;
  flash_write_();
  FCTL1 = 0xa500;              /* WRT = 0 */
  FCTL3 = 0xa510;              /* Lock = 1 */

  if(addr >= INFOMEM_LO && addr <= INFOMEM_HI) {
    lock_infomem();
  }
}
/*---------------------------------------------------------------------------*/
