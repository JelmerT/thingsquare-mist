/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_systick.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      SysTick firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32W_SYSTICK_H
#define __STM32W_SYSTICK_H

//#include "stm32l1xx_type.h"
//#include "stm32w_conf.h"
#include "stm32l1xx.h"



#ifndef EXT
  #define EXT extern
#endif /* EXT */

typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_TypeDef;

#if 0
/* System Control Space memory map */
#define SCS_BASE              ((uint32_t)0xE000E000)

#define SysTick_BASE          (SCS_BASE + 0x0010)
#define NVIC_BASE             (SCS_BASE + 0x0100)
#define SCB_BASE              (SCS_BASE + 0x0D00)

#define SysTick             ((SysTick_TypeDef *) SysTick_BASE)
#endif    /* if 0; code commented out */


/*****************  Bit definition for SysTick_CTRL register  *****************/
#define  SysTick_CTRL_ENABLE                 ((uint32_t)0x00000001)        /* Counter enable */
#define  SysTick_CTRL_TICKINT                ((uint32_t)0x00000002)        /* Counting down to 0 pends the SysTick handler */
#define  SysTick_CTRL_CLKSOURCE              ((uint32_t)0x00000004)        /* Clock source */
#define  SysTick_CTRL_COUNTFLAG              ((uint32_t)0x00010000)        /* Count Flag */


/*****************  Bit definition for SysTick_LOAD register  *****************/
#define  SysTick_LOAD_RELOAD                 ((uint32_t)0x00FFFFFF)        /* Value to load into the SysTick Current Value Register when the counter reaches 0 */


/*****************  Bit definition for SysTick_VAL register  ******************/
#define  SysTick_VAL_CURRENT                 ((uint32_t)0x00FFFFFF)        /* Current value at the time the register is accessed */


/*****************  Bit definition for SysTick_CALIB register  ****************/
#define  SysTick_CALIB_TENMS                 ((uint32_t)0x00FFFFFF)        /* Reload value to use for 10ms timing */
#define  SysTick_CALIB_SKEW                  ((uint32_t)0x40000000)        /* Calibration value is not exactly 10 ms */
#define  SysTick_CALIB_NOREF                 ((uint32_t)0x80000000)        /* The reference clock is not provided */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* SysTick clock source */
#define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)

#define IS_SYSTICK_CLK_SOURCE(SOURCE) (((SOURCE) == SysTick_CLKSource_HCLK) || \
                                       ((SOURCE) == SysTick_CLKSource_HCLK_Div8))

/* SysTick counter state */
#define SysTick_Counter_Disable        ((uint32_t)0xFFFFFFFE)
#define SysTick_Counter_Enable         ((uint32_t)0x00000001)
#define SysTick_Counter_Clear          ((uint32_t)0x00000000)

#define IS_SYSTICK_COUNTER(COUNTER) (((COUNTER) == SysTick_Counter_Disable) || \
                                     ((COUNTER) == SysTick_Counter_Enable)  || \
                                     ((COUNTER) == SysTick_Counter_Clear))

/* SysTick Flag */
#define SysTick_FLAG_COUNT             ((uint32_t)0x00000010)
#define SysTick_FLAG_SKEW              ((uint32_t)0x0000001E)
#define SysTick_FLAG_NOREF             ((uint32_t)0x0000001F)

#define IS_SYSTICK_FLAG(FLAG) (((FLAG) == SysTick_FLAG_COUNT) || \
                               ((FLAG) == SysTick_FLAG_SKEW)  || \
                               ((FLAG) == SysTick_FLAG_NOREF))

#define IS_SYSTICK_RELOAD(RELOAD) (((RELOAD) > 0) && ((RELOAD) <= 0xFFFFFF))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);
void SysTick_SetReload(uint32_t Reload);
void SysTick_CounterCmd(uint32_t SysTick_Counter);
void SysTick_ITConfig(FunctionalState NewState);
uint32_t SysTick_GetCounter(void);
FlagStatus SysTick_GetFlagStatus(uint8_t SysTick_FLAG);

/* Exported macro ------------------------------------------------------------*/
#ifdef  DEBUG
/*******************************************************************************
* Macro Name     : assert_param
* Description    : The assert_param macro is used for function's parameters check.
*                  It is used only if the library is compiled in DEBUG mode. 
* Input          : - expr: If expr is false, it calls assert_failed function
*                    which reports the name of the source file and the source
*                    line number of the call that failed. 
*                    If expr is true, it returns no value.
* Return         : None
*******************************************************************************/ 
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* DEBUG */

#endif /* __STM32F10x_SYSTICK_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
