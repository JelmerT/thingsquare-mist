/*
 * Copyright (c) 2013, Thingsquare, http://www.thingsquare.com/.
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

#include <stdio.h>
#include <stdint.h>
#include "contiki.h"
#include "dev/watchdog.h"
#include "stm32l1xx.h"

#include "flash.h"

#define FLASH_ER_PRG_TIMEOUT         ((uint32_t)0x8000)

/* XXX Workaround: we have problems rewriting words, so we are currently
 * using a VERY dangerous workaround that relies on each 16-bit address being
 * written in order. */
#define USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND 1

/*---------------------------------------------------------------------------*/
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
} FLASH_Status;
/*---------------------------------------------------------------------------*/
void
flash_clear(unsigned long addr)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* sanity check address */
  if(addr >= FLASH_END || addr < FLASH_START) {
    /* invalid address */
    return;
  }

  /* Wait for last operation to be completed */
  FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  /* Set the ERASE bit */
  FLASH->PECR |= FLASH_PECR_ERASE;

  /* Set PROG bit */
  FLASH->PECR |= FLASH_PECR_PROG;

  /* Write 00000000h to the first word of the program page to erase */
  *(__IO uint32_t *)addr = 0x00000000;

  /* Wait for last operation to be completed */
  FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  /* If the erase operation is completed, disable the ERASE and PROG bits */
  FLASH->PECR &= (uint32_t)(~FLASH_PECR_PROG);
  FLASH->PECR &= (uint32_t)(~FLASH_PECR_ERASE);
}
/*---------------------------------------------------------------------------*/
#if USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND
static uint16_t lastval = -1;
#endif /* USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND */
void
flash_write(unsigned long addr16, uint16_t data16)
{
  uint32_t data32;
  uint32_t addr32;

  /* sanity check address */
  if((addr16 + FLASH_MIN_WRITE) >= FLASH_END || addr16 < FLASH_START) {
    /* invalid address */
    return;
  }

  /* Set the 16 bits of the 32 bit value we will write */
  if((addr16 % 4) == 0) {
    data32 = 0xffff & data16;

#if USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND
    lastval = data16;
    return;
#endif /* USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND */
  } else {
    data32 = 0xffff0000 & (data16 << 16);

#if USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND
    data32 += lastval;
    lastval = 0;
#endif /* USE_DANGEROUS_ADDR_IN_ORDER_WORKAROUND */
  }
  addr32 = (addr16 & 0xfffffffc); /* Mask away the low 2 bits. */

  FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  /* If the previous operation is completed, proceed to program the new word */
  *(__IO uint32_t *)addr32 = data32;

  /* Wait for last operation to be completed */
  FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
}
/*---------------------------------------------------------------------------*/
uint16_t
flash_read(unsigned long addr16)
{
  uint32_t data32;
  uint32_t addr32;

  /* sanity check address */
  if((addr16 + FLASH_MIN_WRITE) >= FLASH_END || addr16 < FLASH_START) {
    /* invalid address */
    return -1;
  }

  addr32 = (addr16 & 0xfffffffc); /* Mask away the low 2 bits. */
  data32 = *((unsigned long *)addr32);
  if((addr16 % 4) == 0) {
    return (uint16_t)(0xffff & data32);
  } else {
    return (uint16_t)(0xffff & (data32 >> 16));
  }
}
/*---------------------------------------------------------------------------*/
void
flash_setup(void)
{
  __disable_irq();
  FLASH_Unlock();
}
/*---------------------------------------------------------------------------*/
/* empty legacy API function */
void
flash_done(void)
{
  FLASH_Lock();
  __enable_irq();
}
/*---------------------------------------------------------------------------*/
