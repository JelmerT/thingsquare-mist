/**
 * @file    SDK_EVAL_Com.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file provides all the low level API to manage SDK UART.
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
#include "SDK_EVAL_Com.h"


/** @addtogroup SDK_EVAL
 * @{
 */


/** @addtogroup SDK_EVAL_Com
 * @{
 */


/** @defgroup SDK_EVAL_Com_Private_TypesDefinitions             SDK EVAL Com Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Com_Private_Defines                      SDK EVAL Com Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Com_Private_Macros                       SDK EVAL Com Private Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup SDK_EVAL_Com_Private_Variables                    SDK EVAL Com Private Variables
 * @{
 */


USART_TypeDef* vectpxComUsart[COMn] = {EVAL_COM1};

GPIO_TypeDef* vectpxComTxPort[COMn] = {EVAL_COM1_TX_GPIO_PORT};

GPIO_TypeDef* vectpxComRxPort[COMn] = {EVAL_COM1_RX_GPIO_PORT};

static const uint32_t s_vectlComUsartClk[COMn] = {EVAL_COM1_CLK};

static const uint32_t s_vectlComTxPortClk[COMn] = {EVAL_COM1_TX_GPIO_CLK};

static const uint32_t s_vectlComRxPortClk[COMn] = {EVAL_COM1_RX_GPIO_CLK};

static const uint16_t s_vectnComTxPin[COMn] = {EVAL_COM1_TX_PIN};

static const uint16_t s_vectnComRxPin[COMn] = {EVAL_COM1_RX_PIN};

static const uint16_t s_vectnComTxPinSource[COMn] = {EVAL_COM1_TX_SOURCE};

static const uint16_t s_vectnComRxPinSource[COMn] = {EVAL_COM1_RX_SOURCE};

static const uint16_t s_vectnComTxAF[COMn] = {EVAL_COM1_TX_AF};

static const uint16_t s_vectnComRxAF[COMn] = {EVAL_COM1_RX_AF};


/**
 * @}
 */


/** @defgroup SDK_EVAL_Com_Private_FunctionPrototypes                   SDK EVAL Com Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Com_Private_Functions                            SDK EVAL Com Private Functions
 * @{
 */

/**
 * @brief  Configures COM port.
 * @param  xCom Specifies the COM port to be configured.
 *         This parameter can be one of following parameters:
 *         @arg COM1
 *         @arg COM2
 * @param  xUsartInit pointer to a USART_InitTypeDef structure that
 *         contains the configuration information for the specified USART peripheral.
 * @retval None.
 */
void SdkEvalComInit(SdkEvalCom xCom, USART_InitTypeDef* xUsartInit)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(s_vectlComTxPortClk[xCom] | s_vectlComRxPortClk[xCom], ENABLE);

  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(s_vectlComUsartClk[xCom], ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(vectpxComTxPort[xCom], s_vectnComTxPinSource[xCom], s_vectnComTxAF[xCom]);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(vectpxComRxPort[xCom], s_vectnComRxPinSource[xCom], s_vectnComRxAF[xCom]);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = s_vectnComTxPin[xCom];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(vectpxComTxPort[xCom], &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = s_vectnComRxPin[xCom];
  GPIO_Init(vectpxComRxPort[xCom], &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(vectpxComUsart[xCom], xUsartInit);

  /* Enable USART */
  USART_Cmd(vectpxComUsart[xCom], ENABLE);
}

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */


/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
