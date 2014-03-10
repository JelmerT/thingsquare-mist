/**
 * @file    SPIRIT_Management.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V1.0.3
 * @date    August 7, 2012
 * @brief   The service layer of the SPIRIT DK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPIRIT_MANAGEMENT_H_
#define SPIRIT_MANAGEMENT_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "SDK_EVAL_Config.h"
#include "SPIRIT_Config.h"


#ifdef __cplusplus
  "C" {
#endif


/**
 * @addtogroup SPIRIT_DK
 * @{
 */


/**
 * @defgroup SDK_SPIRIT_MANAGEMENT
 * @{
 */
    
typedef struct 
{
  uint16_t nSpiritVersion;
  SpiritVersion xSpiritVersion;
}SpiritVersionMap;
   
#define CUT_MAX_NO 3
#define CUT_2_1v3 0x0103
#define CUT_2_1v4 0x0104
#define CUT_3_0   0x0130


/**
 * @brief  Range extender type
 */
typedef enum
{
  RANGE_EXT_NONE = 0x00,
  RANGE_EXT_SKYWORKS,
} RangeExtType;
    

typedef struct 
{
  SpiritVersion xSpiritVersion;
  RangeExtType xRangeExtType;
  uint32_t nXtalFrequency;
  BandSelect xBandSelect;
}RfBoardIdentification;


/**
 * @addgroup SDK_SPIRIT_MANAGEMENT_FUNCTIONS
 * @{
 */

void SpiritManagementSetXtalFrequency(uint32_t xtalFrequency);        
void SpiritManagementSetVersion(SpiritVersion version);                  
void SpiritManagementIdentificationRFBoard(void);

BandSelect SpiritManagementComputeBand(void);

RangeExtType SpiritManagementGetRangeExtender(void);
uint32_t SpiritManagementXtalFrequency(void);

void SpiritManagementWaVcoCalibration(void);
void SpiritManagementWaRcoCalibration(void);
void SpiritManagementWaRxStartupInit(void);
void SpiritManagementWaRxStartup(void);
void SpiritManagementWaCmdStrobeTx(void);
void SpiritManagementWaCmdStrobeRx(void);
void SpiritManagementWaTRxFcMem(uint32_t nDesiredFreq);

void SpiritManagementRangeExtInit(void);

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

