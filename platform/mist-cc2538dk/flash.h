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
 *      flash.h
 *   \author
 *      Marcus Lunden <marcus@thingsquare.com>>
 *   \desc
 *      Flash driver for CC2538
 */

#ifndef __FLASH_H__
#define __FLASH_H__
/*---------------------------------------------------------------------------*/
/* erase a page in flash based on an address anywhere in that page */
void flash_clear(unsigned long addr);

/* write a 32-bit word to flash. */
void flash_write(unsigned long addr, uint16_t data);

/* read a 32-bit word from flash */
unsigned long flash_read(unsigned long addr);

/* empty legacy API function */
void flash_setup(void);

/* empty legacy API function */
void flash_done(void);
/*---------------------------------------------------------------------------*/
/* The flash controller registers */
#define FLASH_CTRL_FCTL           0x400D3008
#define FLASH_CTRL_FADDR          0x400D300C
#define FLASH_CTRL_FWDATA         0x400D3010

/* a few of the bitfields in the FCTL register */
#define FLASH_BITFIELD_UPAGE_ACCESS       ((uint32_t) (1<<9))
#define FLASH_BITFIELD_SEL_INFOP          ((uint32_t) (1<<8))
#define FLASH_BITFIELD_BUSY               ((uint32_t) (1<<7))
#define FLASH_BITFIELD_FULL               ((uint32_t) (1<<6))
#define FLASH_BITFIELD_WRITE              ((uint32_t) (1<<1))
#define FLASH_BITFIELD_ERASE              ((uint32_t) (1<<0))

/* the CC2538 in the CC2538dk has flash from 0x200000, length 0x0007FFD4 */
#define FLASH_START           ((uint32_t) 0x00200000)
#define FLASH_SIZE            ((uint32_t) 0x0007FFD4)
#define FLASH_END             (FLASH_START + FLASH_SIZE)
#define FLASH_PAGE_SIZE       ((uint32_t) 2048)           /* Bytes */
#define FLASH_MIN_WRITE       ((uint32_t) 4)              /* Bytes */
#define FLASH_MIN_ERASE       FLASH_PAGE_SIZE

/*
 * macros for getting the page number, start and end addresses a page from an 
 * address located in that page.
 */
#define FLASH_PAGE_NO(addr)       ((uint16_t) ((addr - FLASH_START) / FLASH_PAGE_SIZE))
#define FLASH_PAGE_START(addr)    (FLASH_START + FLASH_PAGE_SIZE * FLASH_PAGE_NO(addr))
#define FLASH_PAGE_END(addr)      (FLASH_PAGE_START(addr) + FLASH_PAGE_SIZE)
/*---------------------------------------------------------------------------*/
#endif /* __FLASH_H__ */
