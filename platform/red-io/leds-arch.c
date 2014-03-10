/*
 * Copyright (c) 2012-2013, Thingsquare, http://www.thingsquare.com/.
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

#include "dev/leds.h"

#include "mc1322x.h"

static unsigned char last_leds;


/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  /* Green */
  GPIO->FUNC_SEL.GPIO_43 = 3;
  GPIO->PAD_DIR_SET.GPIO_43 = 1;

  /* Red */
  GPIO->FUNC_SEL.GPIO_50 = 3;
  GPIO->PAD_DIR_SET.GPIO_50 = 1;
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return last_leds;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  last_leds = leds;
  if(leds & LEDS_GREEN) {
    gpio_set(GPIO_43);
  } else {
    gpio_reset(GPIO_43);
  }
  if(leds & LEDS_RED) {
    gpio_set(GPIO_50);
  } else {
    gpio_reset(GPIO_50);
  }
}
/*---------------------------------------------------------------------------*/
