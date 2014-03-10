/**
 * @file    SDK_EVAL_Timers.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V2.0.2
 * @date    Febrary 7, 2012
 * @brief   SDK EVAL timers configuration.
 * @details
 *
 * This module allows the user to easily configure the STM32L timers.
 * The functions that are provided are limited to the generation of an
 * IRQ every time the timer elapses.
 *
 * <b>Example:</b>
 * @code
 *
 *   ...
 *
 *   SdkEvalTimersTimConfig_ms(TIM2,60.0);
 *
 *   ...
 *
 *   SdkEvalTimersState(TIM2, ENABLE);          // the timer starts counting here
 *
 *   ...
 *
 *   SdkEvalTimersState(TIM2, DISABLE);         // timer stopped
 *
 *   ...
 *
 * @endcode
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
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_TIMERS_H
#define __SDK_EVAL_TIMERS_H

/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include "SDK_EVAL_Com.h"


#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup SDK_EVAL
 * @{
 */

/**
 * @defgroup SDK_EVAL_Timers            SDK EVAL Timers
 * @brief Management of STM32L timers.
 * @details See the file <i>@ref SDK_EVAL_Timers.h</i> for more details.
 * @{
 */



/**
 * @defgroup SDK_EVAL_Timers_Exported_Types             SDK EVAL Timers Exported Types
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup SDK_EVAL_Timers_Exported_Constants         SDK EVAL Timers Exported Constants
 * @{
 */

/**
 * @brief MCU XO frequency(in KHz) definition
 */
#define CLOCK_FREQUENCY         32000

/**
 *@}
 */


/**
 * @defgroup SDK_EVAL_Timers_Exported_Macros            SDK EVAL Timers Exported Macros
 * @{
 */

/**
 * @brief  Configures the specified TIMER to raise an interrupt every TIME ms.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @param  TIME: timer duration in ms.
 *          This parameter is a float.
 * @retval None
 */
#define SdkEvalTimersTimConfig_ms(TIMER , TIME)      {\
                                                        uint32_t n = (uint32_t)(TIME*CLOCK_FREQUENCY);\
                                                        uint16_t a,b;\
                                                        SdkEvalTimersFindFactors(n,&a,&b);\
                                                        SdkEvalTimersTimConfig(TIMER,a-1,b-1);\
                                                      }

/**
 * @brief  Enables or Disables a specific Timer with its IRQ.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @param  NEWSTATE: specifies if a timer has to be enabled or disabled.
 *          This parameter is a float.
 * @retval None
 */
#define SdkEvalTimersState(TIMER , NEWSTATE)      {\
                                            TIM_ITConfig(TIMER, TIM_IT_Update, NEWSTATE);\
                                            TIM_Cmd(TIMER, NEWSTATE);\
                                            TIM_ClearITPendingBit(TIMER , TIM_IT_Update);\
                                            }

/**
 * @brief  Set the counter of the specified TIMER.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @param  VALUE: value to set in the counter.
 *          This parameter is an uint32_t
 * @retval None
 */
#define SdkEvalTimersSetCounter(TIMER,VALUE)   {\
    TIM_SetCounter(TIMER, VALUE);\
}


/**
 * @brief  Resets the counter of a specific timer.
 * @param  TIMER: timer to be reset.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @retval None
 */
#define SdkEvalTimersResetCounter(TIMER)      {\
                                            TIM_SetCounter(TIMER,0);\
                                            }

/**
 * @brief  Absolute value macro.
 * @param  x: Value on which apply the abs function.
 * @retval None
 */
#define ABS(x)  (x>0?x:-x)

/**
 *@}
 */


/**
 * @defgroup SDK_EVAL_Timers_Exported_Functions         SDK EVAL Timers Exported Functions
 * @{
 */
   
#ifdef USE_SYSTICK_DELAY
void SysTick_Handler(void);
void SdkDelayMs(uint32_t lTimeMs);
uint32_t SdkGetCurrentSysTick(void);
void SdkStartSysTick(void);
#endif

void SdkEvalTimersFindFactors(uint32_t n, uint16_t *a, uint16_t *b);
void SdkEvalTimersTimConfig(TIM_TypeDef* tim, uint16_t nPrescaler, uint16_t nPeriod);


/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
