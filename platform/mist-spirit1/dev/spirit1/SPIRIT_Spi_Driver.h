/**
 * @file    SPIRIT_SPI_Driver.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V2.0.2
 * @date    Febrary 7, 2012
 * @brief   Header file for low level SPIRIT SPI driver.
 * @details
 *
 * This header file constitutes an interface to the SPI driver used to
 * communicate with Spirit.
 * It exports some function prototypes to write/read registers and FIFOs
 * and to send command strobes.
 *
 * Since the Spirit libraries are totally platform independent, the implementation
 * of these functions are not provided here. The user have to implement these functions
 * taking care to keep the exported prototypes.
 *
 * These functions are:
 *
 * <ul>
 * <li>SpiritSpiInit</i>
 * <li>SpiritSpiWriteRegisters</i>
 * <li>SpiritSpiReadRegisters</i>
 * <li>SpiritSpiCommandStrobes</i>
 * <li>SpiritSpiWriteLinearFifo</i>
 * <li>SpiritSpiReadLinearFifo</i>
 * </ul>
 *
 * @note An example of SPI driver implementation is available in the <i>Sdk_Eval</i> library.
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
#ifndef __SPIRIT_SPI_DRIVER_H
#define __SPIRIT_SPI_DRIVER_H


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Types.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SPIRIT_Libraries
 * @{
 */


/** @defgroup SPIRIT_SPI_Driver         SPI Driver
 * @brief Header file for low level SPIRIT SPI driver.
 * @details See the file <i>@ref SPIRIT_Spi_Driver.h</i> for more details.
 * @{
 */



/** @defgroup SPI_Exported_Types        SPI Exported Types
 * @{
 */

/**
 * @}
 */



/** @defgroup SPI_Exported_Constants    SPI Exported Constants
 * @{
 */

/**
 * @}
 */



/** @defgroup SPI_Exported_Macros       SPI Exported Macros
 * @{
 */

/**
 * @}
 */



/** @defgroup SPI_Exported_Functions    SPI Exported Functions
 * @{
 */

void SpiritSpiInit(void);
SpiritStatus SpiritSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus SpiritSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus SpiritSpiCommandStrobes(uint8_t cCommandCode);
SpiritStatus SpiritSpiWriteLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus SpiritSpiReadLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
void SpiritWorkaroundRx(void);
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
