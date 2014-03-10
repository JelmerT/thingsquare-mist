/**
 * @file    SDK_EVAL_Led.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file provides all the low level API to manage SDK LEDs.
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
#include "SDK_EVAL_Led.h"

/** @addtogroup SDK_EVAL
 * @{
 */


/** @addtogroup SDK_EVAL_Led
 * @{
 */

/** @defgroup SDK_EVAL_Led_Private_TypesDefinitions             SDK EVAL Led Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Defines                      SDK EVAL Led Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Macros                       SDK EVAL Led Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Variables                    SDK EVAL Led Private Variables
 * @{
 */
GPIO_TypeDef* vectpxGpioPort[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT,
                                 LED4_GPIO_PORT, LED5_GPIO_PORT};
static const uint16_t s_vectnGpioPin[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN,
                                 LED4_PIN, LED5_PIN};
static const uint32_t s_vectlGpioClk[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK,
                                 LED4_GPIO_CLK, LED5_GPIO_CLK};

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Led_Private_FunctionPrototypes                    SDK EVAL Led Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Led_Private_Functions                             SDK EVAL Led Private Functions
 * @{
 */


/**
 * @brief  Configures LED GPIO.
 * @param  xLed Specifies the Led to be configured.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 *         @arg LED3
 *         @arg LED4
 *         @arg LED5
 * @retval None.
 */
void SdkEvalLedInit(SdkEvalLed xLed)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(s_vectlGpioClk[xLed], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = s_vectnGpioPin[xLed];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(vectpxGpioPort[xLed], &GPIO_InitStructure);
  vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed];
}

/**
 * @brief  Turns selected LED On.
 * @param  xLed Specifies the Led to be set on.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 *         @arg LED3
 *         @arg LED4
 *         @arg LED5
 * @retval None.
 */
void SdkEvalLedOn(SdkEvalLed xLed)
{
  vectpxGpioPort[xLed]->BSRRH = s_vectnGpioPin[xLed];
}

/**
 * @brief  Turns selected LED Off.
 * @param  xLed Specifies the Led to be set off.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 *         @arg LED3
 *         @arg LED4
 *         @arg LED5
 * @retval None.
 */
void SdkEvalLedOff(SdkEvalLed xLed)
{
  vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed];
}

/**
 * @brief  Toggles the selected LED.
 * @param  xLed Specifies the Led to be toggled.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 *         @arg LED3
 *         @arg LED4
 *         @arg LED5
 * @retval None.
 */
void SdkEvalLedToggle(SdkEvalLed xLed)
{
  vectpxGpioPort[xLed]->ODR ^= s_vectnGpioPin[xLed];
}

/**
 * @brief  Returns the status of a specified led.
 * @param  xLed Specifies the Led to be read.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 *         @arg LED3
 *         @arg LED4
 *         @arg LED5
 * @retval FlagStatus return the status of the LED. This parameter can be:
 *         SET or RESET.
 */
FlagStatus SdkEvalLedGetState(SdkEvalLed xLed)
{
  if(vectpxGpioPort[xLed]->IDR & s_vectnGpioPin[xLed])
    return RESET;
  else
    return SET;
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
