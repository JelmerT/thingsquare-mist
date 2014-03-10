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

/* CLK = CLocK */
#define SPI_CLK_PORT  P10OUT
#define SPI_CLK_DIRP  P10DIR
#define SPI_CLK_BIT   (1 << 2)

/* MOSI = Master Output, Slave Input */
#define SPI_MOSI_PORT P10OUT
#define SPI_MOSI_DIRP P10DIR
#define SPI_MOSI_BIT  (1 << 1)

/* MISO = Master Input, Slave Output */
#define SPI_MISO_PORT P10IN
#define SPI_MISO_DIRP P10DIR
#define SPI_MISO_BIT  (1 << 0)

/* CS = Chip Select */
#define SPI_CS_PORT   P10OUT
#define SPI_CS_DIRP   P10DIR
#define SPI_CS_BIT    (1 << 3)

#define GPIO_SET(port, bit) (port |= bit)
#define GPIO_RESET(port, bit) (port &= ~bit)
#define GPIO_GET(port, bit) (port & bit)
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
  /* Set pins to output or input mode */
  SPI_CLK_DIRP |= SPI_CLK_BIT;
  SPI_CS_DIRP |= SPI_CS_BIT;

  SPI_MOSI_DIRP |= SPI_MOSI_BIT;
  SPI_MISO_DIRP &= ~SPI_MISO_BIT;

  /* The CS pin is active low, so we set it high when we haven't
     selected the chip. */
  GPIO_SET(SPI_CS_PORT, SPI_CS_BIT);

  /* The CLK is active low, we set it high when we aren't using it. */
  GPIO_RESET(SPI_CLK_PORT, SPI_CLK_BIT);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_select(void)
{
  GPIO_RESET(SPI_CS_PORT, SPI_CS_BIT);
  /* SPI delay */
  delay();
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_deselect(void)
{
  GPIO_SET(SPI_CS_PORT, SPI_CS_BIT);
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
      GPIO_SET(SPI_MOSI_PORT, SPI_MOSI_BIT);
    } else {
      GPIO_RESET(SPI_MOSI_PORT, SPI_MOSI_BIT);
    }
    output <<= 1;

    /* Set clock high  */
    GPIO_SET(SPI_CLK_PORT, SPI_CLK_BIT);

    /* SPI delay */
    delay();

    /* Read data from MISO pin */
    input <<= 1;
    if(GPIO_GET(SPI_MISO_PORT, SPI_MISO_BIT) != 0) {
      input |= 0x1;
    }

    /* Set clock low */
    GPIO_RESET(SPI_CLK_PORT, SPI_CLK_BIT);

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

