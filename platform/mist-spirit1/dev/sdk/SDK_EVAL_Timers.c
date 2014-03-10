/**
 * @file    SDK_EVAL_Timers.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   SDK EVAL timers configuration.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Timers.h"



/**
 * @addtogroup SDK_EVAL
 * @{
 */

/**
 * @addtogroup SDK_EVAL_Timers
 * @{
 */


/**
 * @defgroup SDK_EVAL_Timers_Private_TypesDefinitions           SDK EVAL Timers Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup SDK_EVAL_Timers_Private_Defines                    SDK EVAL Timers Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup SDK_EVAL_Timers_Private_Macros                     SDK EVAL Timers Private Macros
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup SDK_EVAL_Timers_Private_Variables                  SDK EVAL Timers Private Variables
 * @{
 */

volatile uint32_t lSystickCounter=0;

/**
 *@}
 */

/**
 * @defgroup SDK_EVAL_Timers_Private_FunctionPrototypes         SDK EVAL Timers Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup SDK_EVAL_Timers_Private_Functions                  SDK EVAL Timers Private Functions
 * @{
 */



/**
 * @brief  Computes two integer value prescaler and period such that Cycles = prescaler * period.
 * @param  lCycles the specified cycles for the desired timer value.
 * @param  pnPrescaler prescaler factor.
 * @param  pnCounter period factor.
 * @retval None.
*/
void SdkEvalTimersFindFactors(uint32_t lCycles, uint16_t *pnPrescaler, uint16_t *pnCounter)
{
  uint16_t b0;
  uint16_t a0;
  long err, err_min=lCycles;

  *pnPrescaler = a0 = ((lCycles-1)/0xffff) + 1;
  *pnCounter = b0 = lCycles / *pnPrescaler;

  for (; *pnPrescaler < 0xffff-1; (*pnPrescaler)++)
  {
    *pnCounter = lCycles / *pnPrescaler;
    err = (long)*pnPrescaler * (long)*pnCounter - (long)lCycles;
    if (ABS(err) > (*pnPrescaler / 2))
    {
      (*pnCounter)++;
      err = (long)*pnPrescaler * (long)*pnCounter - (long)lCycles;
    }
    if (ABS(err) < ABS(err_min))
    {
      err_min = err;
      a0 = *pnPrescaler;
      b0 = *pnCounter;
      if (err == 0) break;
    }
  }

  *pnPrescaler = a0;
  *pnCounter = b0;

}


/**
 * @brief  Configures the specified timer to raise an interrupt every time the counter
 *         reaches the nPeriod value counting with a prescaler of nPrescaler.
 * @note   The specified timer is configured but not enabled.
 * @param  xTim timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef .
 * @param  nPrescaler prescaler factor.
 * @param  nPeriod period factor.
 * @retval None.
 */
void SdkEvalTimersTimConfig(TIM_TypeDef* xTim, uint16_t nPrescaler, uint16_t nPeriod)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* disable the timer */
  TIM_Cmd(xTim, DISABLE);

  /* Configure the timer in update mode */
  TIM_ITConfig(xTim, TIM_IT_Update, DISABLE);

  /* put the timer clock on */
  if(xTim == TIM2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  else if(xTim==TIM3)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  else if(xTim==TIM4)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  else if(xTim==TIM6)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  /* Init the time base structure */
  TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = nPeriod;
  TIM_TimeBaseStructure.TIM_Prescaler = nPrescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(xTim, &TIM_TimeBaseStructure);


  /* NVIC configuration */
  if(xTim == TIM2)
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  else if(xTim == TIM3)
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  else if(xTim == TIM4)
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  else if(xTim == TIM6)
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* Clear the timer pending bit */
  TIM_ClearITPendingBit(xTim , TIM_IT_Update);

}

#ifdef USE_SYSTICK_DELAY
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  lSystickCounter++;
}

/**
  * @brief  This function implements return the current
  *         systick with a step of 1 ms.
  * @param  lTimeMs desired delay expressed in ms.
  * @retval None
  */

/**
  * @brief  This function implements return the current
  *         systick with a step of 1 ms.
  * @param  lTimeMs desired delay expressed in ms.
  * @retval None
  */
uint32_t SdkGetCurrentSysTick(void)
{
  return lSystickCounter;
  
}

void SdkStartSysTick(void)
{
  SysTick_Config(32000);
  lSystickCounter = 0;

}

/**
  * @brief  This function implements a delay using the microcontroller
  *         Systick with a step of 1 ms.
  * @param  lTimeMs desired delay expressed in ms.
  * @retval None
  */
void SdkDelayMs(uint32_t lTimeMs)
{
  uint32_t nWaitPeriod = ~lSystickCounter;
  
  if(nWaitPeriod<lTimeMs)
  {
    while( lSystickCounter != 0xFFFFFFFF);
    nWaitPeriod = lTimeMs-nWaitPeriod;
  }
  else
    nWaitPeriod = lTimeMs+ ~nWaitPeriod;
  
  while( lSystickCounter != nWaitPeriod ) ;

}
#endif
/**
 *@}
 */


/**
 *@}
 */


/**
 *@}
 */


/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
