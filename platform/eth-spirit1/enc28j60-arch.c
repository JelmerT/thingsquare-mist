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

#include "clock.h"

#include "SDK_EVAL_Spirit_Gpio.h"

#define ENC_SCK_GPIO_PIN GPIO_Pin_12
#define ENC_SCK_GPIO_PORT GPIOB

#define ENC_MOSI_GPIO_PIN GPIO_Pin_13
#define ENC_MOSI_GPIO_PORT GPIOB

#define ENC_MISO_GPIO_PIN GPIO_Pin_14
#define ENC_MISO_GPIO_PORT GPIOB

#define ENC_CS_GPIO_PIN GPIO_Pin_15
#define ENC_CS_GPIO_PORT GPIOB

/*---------------------------------------------------------------------------*/
static void
delay(void)
{
  //  clock_delay_usec(10);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Sck */
  GPIO_InitStructure.GPIO_Pin = ENC_SCK_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(ENC_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Mosi */
  GPIO_InitStructure.GPIO_Pin = ENC_MOSI_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(ENC_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* Miso */
  GPIO_InitStructure.GPIO_Pin = ENC_MISO_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(ENC_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* Clk */
  GPIO_InitStructure.GPIO_Pin = ENC_CS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(ENC_CS_GPIO_PORT, &GPIO_InitStructure);

  /* The CS pin is active low, so we set it high when we haven't
       selected the chip. */
  GPIO_WriteBit(ENC_CS_GPIO_PORT, ENC_CS_GPIO_PIN, SET);

  /* The SCK is active low, we set it high when we aren't using it. */
  GPIO_WriteBit(ENC_SCK_GPIO_PORT, ENC_SCK_GPIO_PIN, RESET); /* XXX */
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_select(void)
{
  GPIO_WriteBit(ENC_CS_GPIO_PORT, ENC_CS_GPIO_PIN, RESET);

  /* SPI delay */
  delay();
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_deselect(void)
{
  GPIO_WriteBit(ENC_CS_GPIO_PORT, ENC_CS_GPIO_PIN, SET);
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_write(uint8_t output)
{
  int i;
  uint8_t input;

  input = 0;

  for(i = 0; i < 8; i++) {

    /* Write data on MOSI pin */
    if(output & 0x80) {
      GPIO_WriteBit(ENC_MOSI_GPIO_PORT, ENC_MOSI_GPIO_PIN, SET);
    } else {
      GPIO_WriteBit(ENC_MOSI_GPIO_PORT, ENC_MOSI_GPIO_PIN, RESET);
    }
    output <<= 1;

    /* Set clock high  */
    GPIO_WriteBit(ENC_SCK_GPIO_PORT, ENC_SCK_GPIO_PIN, SET);

    /* SPI delay */
    delay();

    /* Read data from MISO pin */
    input <<= 1;
    if(GPIO_ReadInputDataBit(ENC_MISO_GPIO_PORT, ENC_MISO_GPIO_PIN) != 0) {
      input |= 0x1;
    }

    /* Set clock low */
    GPIO_WriteBit(ENC_SCK_GPIO_PORT, ENC_SCK_GPIO_PIN, RESET);

    /* SPI delay */
    delay();

  }
  return input;
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_read(void)
{
  return enc28j60_arch_spi_write(0);
}
/*---------------------------------------------------------------------------*/

