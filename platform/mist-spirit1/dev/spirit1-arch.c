/*
 * Copyright (c) 2012, STMicroelectronics.
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
 *
 * This file is part of the Contiki operating system.
 *
 */

/*
 *  \file
 *      spirit-arch.c
 *   \author
 *      Marcus Lunden <marcus@thingsquare.com>>
 *   \desc
 *      Spirit1 arch driver
 *      
 *      
 */

#include "stm32l1xx.h"
#include "spirit1-arch.h"
#include "spirit1.h"
#include "SDK_EVAL_Spirit_Spi_Config.h"
#include "SPIRIT_Spi_Driver.h"

extern void spirit1_interrupt_callback(void);
SpiritBool spiritdk_timer_expired = S_FALSE;
/*---------------------------------------------------------------------------*/
/* The Spirit1 transceiver use a GPIO to signal irqs; this is the first-line
  interrupt handler. It passes down handling to a callback in the radio driver. */
void
M2S_GPIO_3_EXTI_IRQ_HANDLER(void)
{
  if(EXTI_GetITStatus(M2S_GPIO_3_EXTI_LINE)) {
    EXTI_ClearITPendingBit(M2S_GPIO_3_EXTI_LINE);
    spirit1_interrupt_callback();
  }
}
/*---------------------------------------------------------------------------*/
/* use the SPI-port to acquire the status bytes from the radio. */
#define CS_TO_SCLK_DELAY  0x0100

uint16_t
spirit1_arch_refresh_status(void)
{
  volatile uint16_t mcstate = 0x0000;
  uint8_t header[2];
  int i;
  
  header[0]=0x01;
  header[1]=0;
  
  /* CS is active low */
  SPIRIT_SPI_PERIPH_CS_PORT->BSRRH = SPIRIT_SPI_PERIPH_CS_PIN;
  {
    volatile uint32_t iv;
    for(iv = 0; iv < CS_TO_SCLK_DELAY; iv++);
  }  
  
  /* send arbitrary header bytes and read out MC_STATE from SPI rx buffer */
  for(i = 0; i < 2; i++) {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    mcstate += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  
  /* finish up */
  while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
  SPIRIT_SPI_PERIPH_CS_PORT->BSRRL = SPIRIT_SPI_PERIPH_CS_PIN;
  return mcstate;
}
/*---------------------------------------------------------------------------*/
#if 0
//removed use of another timer, instead use rtimer/BUSYWAIT_UNTIL();
SpiritBool IsTimeExpired(void)
{
  return (SpiritBool)(spiritdk_timer_expired==S_TRUE);
}
/*---------------------------------------------------------------------------*/
void TIM4_IRQHandler(void) 
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update)) {
    spiritdk_timer_expired = S_TRUE;
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);    
  }
}
/*---------------------------------------------------------------------------*/
void Delay1ms(uint16_t period)
{
  uint16_t start_counter;
  SdkEvalTimersTimConfig(TIM2,32000-1, 0xFFFF); // 1ms @ 32MHz clk
  SdkEvalTimersResetCounter(TIM2);  
  TIM_Cmd(TIM2, ENABLE);
  do { 
    start_counter = TIM_GetCounter(TIM2);
  } while(start_counter < period);
  TIM_Cmd(TIM2, DISABLE);
}
/*---------------------------------------------------------------------------*/
void StartWaitTimer(uint16_t value_ms)
{
  /* 1 ms */
  SdkEvalTimersTimConfig(TIM4,32000-1, value_ms); // 1ms @ 32MHz clk
  SdkEvalTimersState(TIM4, ENABLE);
  SdkEvalTimersResetCounter(TIM4);
  spiritdk_timer_expired = S_FALSE;
}
/*---------------------------------------------------------------------------*/
void StopWaitTimer(void)
{
  SdkEvalTimersState(TIM4, DISABLE);
}
#endif    /* if 0; code commented out */
/*---------------------------------------------------------------------------*/

