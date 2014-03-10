/**
 * @file    SDK_EVAL_Config.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V2.0.2
 * @date    Febrary 7, 2012
 * @brief   This file contains SDK EVAL configuration and useful defines.
 * @details
 *
 * This file is used to include all or a part of the SDK Eval
 * libraries into the application program which will be used.
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
#ifndef __SDK_EVAL_CONFIG_H
#define __SDK_EVAL_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Button.h"
#include "SDK_EVAL_Com.h"
#include "SDK_EVAL_Led.h"
#include "SDK_EVAL_PM.h"
#include "SDK_EVAL_Spirit_Gpio.h"
#include "SDK_EVAL_Timers.h"
#include "SDK_EVAL_Com.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SDK_EVAL        SDK EVAL
 * @brief This module is used to configure the SDK Eval board and
 * allows to manage its peripherals in a simple way.
 * @details The board provides:
 * <ul>
 * <li>An STM32L152VB microcontroller clocked by an 8MHz Xtal working at 32MHz</li>
 * <li>An usb connector used to connect the board to the PC</li>
 * <li>An SPI interface to connect SPIRIT</li>
 * <li>Five leds for general purposes</li>
 * <li>Three push buttons and a joystick for general purposes</li>
 * <li>Two digital potentiometers connected via I2C used to regulate the Spirit supply voltage</li>
 * <li>A JTAG/SWD connector</li>
 * <li>An UART 6pins connector</li>
 * </ul>
 * @{
 */

/** @addtogroup SDK_EVAL_Config         SDK EVAL Config
 * @brief SDK EVAL configuration.
 * @details See the file <i>@ref SDK_EVAL_Config.h</i> for more details.
 * @{
 */

/** @addtogroup SDK_EVAL_Config_Exported_Types          SDK EVAL Config Exported Types
 * @{
 */

/**
 * @}
 */

/** @defgroup SDK_EVAL_Config_Exported_Constants        SDK EVAL Config Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Config_Exported_Macros           SDK EVAL Config Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Config_Exported_Functions        SDK EVAL Config Exported Functions
 * @{
 */

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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
