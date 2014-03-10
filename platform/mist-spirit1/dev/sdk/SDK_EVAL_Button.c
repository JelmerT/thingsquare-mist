/**
 * @file    SDK_EVAL_Button.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file provides all the low level API to manage SDK buttons.
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
#include "SDK_EVAL_Button.h"


/** @addtogroup SDK_EVAL
 * @{
 */


/** @addtogroup SDK_EVAL_Button
 * @{
 */

/** @defgroup SDK_EVAL_Button_Private_TypesDefinitions          SDK EVAL Button Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Button_Private_Defines                   SDK EVAL Button Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Button_Private_Macros                    SDK EVAL Button Private Macros
 * @{
 */

/**
 * @}
 */



/** @defgroup SDK_EVAL_Button_Private_Variables                         SDK EVAL Button Private Variables
 * @{
 */

GPIO_TypeDef* vectpxButtonPort[BUTTONn] = {KEY_BUTTON_GPIO_PORT, RIGHT_BUTTON_GPIO_PORT,
                                      LEFT_BUTTON_GPIO_PORT, UP_BUTTON_GPIO_PORT,
                                      DOWN_BUTTON_GPIO_PORT, SEL_BUTTON_GPIO_PORT,
                                      SCM_PS_BUTTON_GPIO_PORT};

static const uint16_t s_vectnButtonPin[BUTTONn] = {KEY_BUTTON_PIN, RIGHT_BUTTON_PIN,
                                      LEFT_BUTTON_PIN, UP_BUTTON_PIN,
                                      DOWN_BUTTON_PIN, SEL_BUTTON_PIN,
                                      SCM_PS_BUTTON_PIN};

static const uint32_t s_vectlButtonClk[BUTTONn] = {KEY_BUTTON_GPIO_CLK, RIGHT_BUTTON_GPIO_CLK,
                                      LEFT_BUTTON_GPIO_CLK, UP_BUTTON_GPIO_CLK,
                                      DOWN_BUTTON_GPIO_CLK, SEL_BUTTON_GPIO_CLK,
                                      SCM_PS_BUTTON_GPIO_CLK};

static const uint16_t s_vectnButtonExtiLine[BUTTONn] = {KEY_BUTTON_EXTI_LINE,
                                            RIGHT_BUTTON_EXTI_LINE,
                                            LEFT_BUTTON_EXTI_LINE,
                                            UP_BUTTON_EXTI_LINE,
                                            DOWN_BUTTON_EXTI_LINE,
                                            SEL_BUTTON_EXTI_LINE,
                                            SCM_PS_BUTTON_EXTI_LINE};

static const uint16_t s_vectnButtonPortSource[BUTTONn] = {KEY_BUTTON_EXTI_PORT_SOURCE,
                                              RIGHT_BUTTON_EXTI_PORT_SOURCE,
                                              LEFT_BUTTON_EXTI_PORT_SOURCE,
                                              UP_BUTTON_EXTI_PORT_SOURCE,
                                              DOWN_BUTTON_EXTI_PORT_SOURCE,
                                              SEL_BUTTON_EXTI_PORT_SOURCE,
                                              SCM_PS_BUTTON_EXTI_PORT_SOURCE};

static const uint16_t s_vectnButtonPinSource[BUTTONn] = {KEY_BUTTON_EXTI_PIN_SOURCE,
                                             RIGHT_BUTTON_EXTI_PIN_SOURCE,
                                             LEFT_BUTTON_EXTI_PIN_SOURCE,
                                             UP_BUTTON_EXTI_PIN_SOURCE,
                                             DOWN_BUTTON_EXTI_PIN_SOURCE,
                                             SEL_BUTTON_EXTI_PIN_SOURCE,
                                             SCM_PS_BUTTON_EXTI_PIN_SOURCE};

static const uint16_t s_vectnButtonIrqn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn, RIGHT_BUTTON_EXTI_IRQn,
                                       LEFT_BUTTON_EXTI_IRQn, UP_BUTTON_EXTI_IRQn,
                                       DOWN_BUTTON_EXTI_IRQn, SEL_BUTTON_EXTI_IRQn,
                                       SCM_PS_BUTTON_EXTI_IRQn};


/**
 * @}
 */


/** @defgroup SDK_EVAL_Button_Private_FunctionPrototypes                        SDK EVAL Button Private Function Prototypes
 * @{
 */

/**
 * @}
 */

/** @defgroup SDK_EVAL_Button_Private_Functions                                 SDK EVAL Button Private Functions
 * @{
 */


/**
 * @brief  Configures Button GPIO and EXTI Line.
 * @param  xButton Specifies the Button to be configured.
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_WAKEUP: Wakeup Push Button
 *         @arg BUTTON_TAMPER: Tamper Push Button
 *         @arg BUTTON_KEY: Key Push Button
 *         @arg BUTTON_RIGHT: Joystick Right Push Button
 *         @arg BUTTON_LEFT: Joystick Left Push Button
 *         @arg BUTTON_UP: Joystick Up Push Button
 *         @arg BUTTON_DOWN: Joystick Down Push Button
 *         @arg BUTTON_SEL: Joystick Sel Push Button
 * @param  xButtonMode Specifies Button mode.
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_MODE_GPIO: Button will be used as simple IO
 *         @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
 *         generation capability
 * @retval None.
 */
void SdkEvalPushButtonInit(SdkEvalButton xButton, SdkEvalButtonMode xButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enables the BUTTON Clock */
  RCC_AHBPeriphClockCmd(s_vectlButtonClk[xButton], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configures Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = s_vectnButtonPin[xButton];
  GPIO_Init(vectpxButtonPort[xButton], &GPIO_InitStructure);


  if (xButtonMode == BUTTON_MODE_EXTI)
  {
    /* Connects Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(s_vectnButtonPortSource[xButton], s_vectnButtonPinSource[xButton]);

    /* Configures Button EXTI line */
    EXTI_InitStructure.EXTI_Line = s_vectnButtonExtiLine[xButton];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if((xButton != BUTTON_KEY))
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    }
    else
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    }
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enables and sets Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = s_vectnButtonIrqn[xButton];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BUTTON_IRQ_PREEMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = BUTTON_IRQ_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    EXTI_ClearITPendingBit(s_vectnButtonExtiLine[xButton]);

    NVIC_Init(&NVIC_InitStructure);
  }
}

/**
 * @brief  Returns the selected Button state.
 * @param  xButton Specifies the Button to be checked.
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_WAKEUP: Wakeup Push Button
 *         @arg BUTTON_TAMPER: Tamper Push Button
 *         @arg BUTTON_KEY: Key Push Button
 *         @arg BUTTON_RIGHT: Joystick Right Push Button
 *         @arg BUTTON_LEFT: Joystick Left Push Button
 *         @arg BUTTON_UP: Joystick Up Push Button
 *         @arg BUTTON_DOWN: Joystick Down Push Button
 *         @arg BUTTON_SEL: Joystick Sel Push Button
 * @retval FlagStatus The Button GPIO pin value.
 */
FlagStatus SdkEvalPushButtonGetState(SdkEvalButton xButton)
{
  if(GPIO_ReadInputDataBit(vectpxButtonPort[xButton], s_vectnButtonPin[xButton]))
    return SET;
  else
    return RESET;
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
