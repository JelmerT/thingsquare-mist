/**
 * @file    SDK_EVAL_Com.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file contains definitions for Spirit Development Kit eval board COM ports.
 * @details
 *
 * Configuration of the COM peripheral for the SDK motherboard.
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
#ifndef __SDK_EVAL_COM_H
#define __SDK_EVAL_COM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "STM32L1xx_StdPeriph_Driver/inc/misc.h"

#ifdef __cplusplus
 extern "C" {
#endif


/** @addtogroup SDK_EVAL
 * @{
 */


/** @addtogroup SDK_EVAL_Com            SDK EVAL Com
 * @brief Management of Spirit Development Kit eval board COM ports.
 * @details See the file <i>@ref SDK_EVAL_Com.h</i> for more details.
 * @{
 */


/** @defgroup SDK_EVAL_Com_Exported_Types               SDK EVAL Com Exported Types
 * @{
 */

/**
 * @brief  COM ports for SDK EVAL enumeration
 */
typedef enum
{
  COM1 = 0

} SdkEvalCom;

/**
 * @}
 */

/** @defgroup SDK_EVAL_Com_Exported_Constants           SDK EVAL Com Exported Constants
 * @{
 */

#define COMn                             1

/**
 * @brief Definition for COM port1, connected to USART2
 */
#define EVAL_COM1                        USART2
#define EVAL_COM1_CLK                    RCC_APB1Periph_USART2
#define EVAL_COM1_TX_PIN                 GPIO_Pin_2
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define EVAL_COM1_TX_SOURCE              GPIO_PinSource2
#define EVAL_COM1_TX_AF                  GPIO_AF_USART2
#define EVAL_COM1_RX_PIN                 GPIO_Pin_3
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define EVAL_COM1_RX_SOURCE              GPIO_PinSource3
#define EVAL_COM1_RX_AF                  GPIO_AF_USART2
#define EVAL_COM1_IRQn                   USART2_IRQn

/**
 * @}
 */


/** @defgroup SDK_EVAL_Com_Exported_Macros              SDK EVAL Com Exported Macros
 * @{
 */
/* XXX added to rid of compiler warnings */
#define assert_param(e)   ((void)0)

/**
 * @}
 */


/** @defgroup SDK_EVAL_Com_Exported_Functions           SDK EVAL Com Exported Functions
 * @{
 */

void SdkEvalComInit(SdkEvalCom xCom, USART_InitTypeDef* xUsartInit);

/**
 * @}
 */

/**
 * @}
 */


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
