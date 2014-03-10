/**
 * @file    SDK_EVAL_Spirit_Spi_Config.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V2.0.2
 * @date    Febrary 7, 2012
 * @brief   SPI Configuration used in the Spirit Development Kit eval board to drive SPIRIT.
 * @details
 *
 * This header file contains SPI definitions to configure the peripheral
 * on the SDK motherboard.
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
#ifndef __SDK_EVAL_SPIRIT_SPI_CONFIG_H
#define __SDK_EVAL_SPIRIT_SPI_CONFIG_H


  /* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "SDK_EVAL_Com.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SDK_EVAL
 * @{
 */


/** @addtogroup SDK_EVAL_Spirit_Spi             SDK EVAL Spirit SPI
 * @brief SPI Configuration used in the Spirit Development Kit eval board to drive SPIRIT.
 * @details See the file <i>@ref SDK_EVAL_Spirit_Spi_Config.h</i> for more details.
 * @{
 */



/** @defgroup SDK_EVAL_Spirit_Spi_Exported_Types        SDK EVAL Spirit SPI Exported Types
 * @{
 */


/**
 * @}
 */



/** @defgroup SDK_EVAL_Spirit_Spi_Exported_Constants            SDK EVAL Spirit SPI Exported Constants
 * @{
 */

/** @defgroup SDK_EVAL_Spirit_Spi_Peripheral_Gpio
 * @{
 */

#define SPIRIT_SPI_PERIPH_NB                  SPI1
#define SPIRIT_SPI_PERIPH_RCC                 RCC_APB2Periph_SPI1

/* Defines for MOSI pin*/
#define SPIRIT_SPI_PERIPH_MOSI_PORT           GPIOE
#define SPIRIT_SPI_PERIPH_MOSI_PIN            GPIO_Pin_15
#define SPIRIT_SPI_PERIPH_MOSI_AF             GPIO_AF_SPI1
#define SPIRIT_SPI_PERIPH_MOSI_RCC            RCC_AHBPeriph_GPIOE
#define SPIRIT_SPI_PERIPH_MOSI_RCC_SOURCE     GPIO_PinSource15

/* Defines for MISO pin */
#define SPIRIT_SPI_PERIPH_MISO_PORT           GPIOE
#define SPIRIT_SPI_PERIPH_MISO_PIN            GPIO_Pin_14
#define SPIRIT_SPI_PERIPH_MISO_AF             GPIO_AF_SPI1
#define SPIRIT_SPI_PERIPH_MISO_RCC            RCC_AHBPeriph_GPIOE
#define SPIRIT_SPI_PERIPH_MISO_RCC_SOURCE     GPIO_PinSource14

/* Defines for SCLK pin */
#define SPIRIT_SPI_PERIPH_SCLK_PORT           GPIOE
#define SPIRIT_SPI_PERIPH_SCLK_PIN            GPIO_Pin_13
#define SPIRIT_SPI_PERIPH_SCLK_AF             GPIO_AF_SPI1
#define SPIRIT_SPI_PERIPH_SCLK_RCC            RCC_AHBPeriph_GPIOE
#define SPIRIT_SPI_PERIPH_SCLK_RCC_SOURCE     GPIO_PinSource13

/* Defines for chip select pin */
#define SPIRIT_SPI_PERIPH_CS_PORT             GPIOE
#define SPIRIT_SPI_PERIPH_CS_PIN              GPIO_Pin_12
#define SPIRIT_SPI_PERIPH_CS_RCC              RCC_AHBPeriph_GPIOE
#define SPIRIT_SPI_PERIPH_CS_RCC_SOURCE       GPIO_PinSource12

//#define SpiritSPICSLow()        {SPIRIT_SPI_PERIPH_CS_PORT->BSRRH = SPIRIT_SPI_PERIPH_CS_PIN;}
//#define SpiritSPICSHigh()       {SPIRIT_SPI_PERIPH_CS_PORT->BSRRL = SPIRIT_SPI_PERIPH_CS_PIN;}


/**
 * @}
 */


/**
 * @}
 */



/** @defgroup SDK_EVAL_Spirit_Spi_Exported_Macros                       SDK EVAL Spirit SPI Exported Macros
 * @{
 */


/**
 * @}
 */



/** @defgroup SDK_EVAL_Spirit_Spi_Exported_Functions                    SDK EVAL Spirit SPI Exported Functions
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
