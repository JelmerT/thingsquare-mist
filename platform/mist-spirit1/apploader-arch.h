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

#ifndef APPLOADER_ARCH_H
#define APPLOADER_ARCH_H

#define APPLOADER_ROM_ADDR ((uint32_t)0x0801B000ul)
#define APPLOADER_ROM_SIZE ((uint32_t)0x00001000ul)
#define APPLOADER_RAM_ADDR ((uint32_t)0x20003C00ul)
#define APPLOADER_RAM_SIZE ((uint32_t)0x00000400ul)

#define FIRMWARE_ROM_ADDR    ((uint32_t)0x08000000ul)
#define FIRMWARE_ROM_END     ((uint32_t)0x0801B000ul)
#define FIRMWARE_FARROM_ADDR ((uint32_t)0x00000000ul)
#define FIRMWARE_FARROM_END  ((uint32_t)0x00000000ul)

#define APPLOADER_ERASED_16BITS 0x0000

#define APPLOADER_ERASE_PAGESIZE 0x100

#endif /* APPLOADER_ARCH_H */
