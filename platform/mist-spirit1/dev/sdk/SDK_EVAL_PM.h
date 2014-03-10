/**
 * @file    SDK_EVAL_PM.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file contains definitions for Spirit Development Kit eval board Digipots.
 * @details
 *
 * This module can be used to regulate the supply voltage of Spirit.
 * The basic idea is to read the actual voltage using the micro ADC
 * and then modify the voltage regulated resistors (digipots) in order
 * to perform a kind of control of the supply voltage.
 *
 * <b>Example:</b>
 * @code
 *
 *  ...
 *
 *  SdkEvalPmADCInit();
 *  SdkEvalPmI2CInit();
 *
 *  ...
 *
 *  SdkEvalPmRegulateVRfI(2.5);
 *
 *  ...
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
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_PM_H
#define __SDK_EVAL_PM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "SDK_EVAL_Com.h"

#ifdef __cplusplus
 extern "C" {
#endif


/** @addtogroup SDK_EVAL
 * @{
 */

/** @addtogroup SDK_EVAL_PM             SDK EVAL PM
 * @brief Management of Spirit Development Kit eval board Digipots.
 * @details See the file <i>@ref SDK_EVAL_PM.h</i> for more details.
 * @{
 */

/** @defgroup SDK_EVAL_PM_Exported_Types        SDK EVAL PM Exported Types
 * @{
 */

/**
 * @brief  I2C_Digipot for SDK EVAL enumeration
 */
typedef enum
{
  DIGIPOT1 = 0,
  DIGIPOT2 = 1

} SdkEvalDigipot;

typedef enum
{
  DIGIPOT1_ADDRESS = 0x5A,
  DIGIPOT2_ADDRESS = 0x58

} SdkEvalDigipotAddress;

typedef enum{
 ADC_CH_V_RF = ADC_Channel_13,
 ADC_CH_V_MCU = ADC_Channel_12,
 ADC_CH_V_5V = ADC_Channel_15,
 ADC_CH_V_3V = ADC_Channel_14,
 ADC_CH_I_R1 = ADC_Channel_10,
 ADC_CH_I_R2 = ADC_Channel_11
} SdkEvalAdcChannel;

/**
 * @}
 */


/** @defgroup SDK_EVAL_PM_Exported_Constants                    SDK EVAL PM Exported Constants
 * @{
 */

/**
 * @brief defines for digipot I2C peripheral
 */
#define DIGIPOT_I2C         I2C2
#define DIGIPOT_I2C_SPEED   400000

/**
 * @brief defines for the return messages of I2C transactions routines
 */
#define I2C_DIGIPOT_ERROR   0
#define I2C_DIGIPOT_OK      1

/**
 * @brief defines for DIGIPOT SCL pin
 */
#define DIGIPOT_I2C_SCL_Port                GPIOB
#define DIGIPOT_I2C_SCL_Pin                 GPIO_Pin_10
#define DIGIPOT_I2C_SCL_AF                  GPIO_AF_I2C2
#define DIGIPOT_I2C_SCL_RCC                 RCC_AHBPeriph_GPIOB
#define DIGIPOT_I2C_SCL_RCC_SOURCE          GPIO_PinSource10

/**
 * @brief defines for DIGIPOT SDA pin
 */
#define DIGIPOT_I2C_SDA_Port                GPIOB
#define DIGIPOT_I2C_SDA_Pin                 GPIO_Pin_11
#define DIGIPOT_I2C_SDA_AF                  GPIO_AF_I2C2
#define DIGIPOT_I2C_SDA_RCC                 RCC_AHBPeriph_GPIOB
#define DIGIPOT_I2C_SDA_RCC_SOURCE          GPIO_PinSource11

/**
 * @brief defines for ADC pins
 */
#define ADC_V_RF_Pin                        GPIO_Pin_3
#define ADC_V_5V_Pin                        GPIO_Pin_5
#define ADC_V_3V_Pin                        GPIO_Pin_4
#define ADC_I_R1_Pin                        GPIO_Pin_0
#define ADC_I_R2_Pin                        GPIO_Pin_1


/**
 * @brief defines for ADC partition gains
 */
#define G_V_RF                              0.68
#define G_V_5V                              0.68
#define G_V_3V                              0.68
#define G_I_R1                              2127.0
#define G_I_R2                              6.67


/**
 * @brief nominal ADC fullscale voltage
 */
#define V_FULLSCALE_ADC                     3.26


#define RF_SWITCH_Port                GPIOE
#define RF_SWITCH_Pin                 GPIO_Pin_3
#define RF_SWITCH_RCC                 RCC_AHBPeriph_GPIOE

/**
 * @}
 */

/**
 * @defgroup SDK_EVAL_PM_Exported_Macros                                SDK EVAL PM Exported Macros
 * @{
 */

/**
 * @brief Macro used to obtain the 3V voltage measurement.
 */
#define SdkEvalPmGetVDummy3()                  ((float)SdkEvalPmGetV(ADC_CH_V_MCU))/4095.0*V_FULLSCALE_ADC/G_V_3V

/**
 * @brief Macro used to obtain the RF supply voltage in volt.
 */
#define SdkEvalPmGetVRf()                     ((float)SdkEvalPmGetV(ADC_CH_V_RF))/4095.0*V_FULLSCALE_ADC/G_V_RF

/**
 * @brief Macro used to obtain the 5V voltage measurement.
 */
#define SdkEvalPmGetV5V()                     ((float)SdkEvalPmGetV(ADC_CH_V_5V))/4095.0*V_FULLSCALE_ADC/G_V_5V

/**
 * @brief Macro used to obtain the 3V voltage measurement.
 */
#define SdkEvalPmGetV3V()                     ((float)SdkEvalPmGetV(ADC_CH_V_3V))/4095.0*V_FULLSCALE_ADC/G_V_3V

/**
 * @brief Macro used to obtain the I_R1 current measurement.
 */
#define SdkEvalPmGetIR1()                     ((float)SdkEvalPmGetV(ADC_CH_I_R1))/4095.0*V_FULLSCALE_ADC/G_I_R1

/**
 * @brief Macro used to obtain the I_R2 current measurement.
 */
#define SdkEvalPmGetIR2()                     ((float)SdkEvalPmGetV(ADC_CH_I_R2))/4095.0*V_FULLSCALE_ADC/G_I_R2

/**
 * @brief Macro used to obtain the settled RF supply voltage in volt.
 */
#define SdkEvalPmGetSettledVRf(VAL_ERR)       ((float)SdkEvalPmGetSettledV(ADC_CH_V_RF , VAL_ERR))/4095.0*V_FULLSCALE_ADC/G_V_RF

/**
 * @brief Macro used to obtain the settled 5V voltage measurement.
 */
#define SdkEvalPmGetSettledV5V(VAL_ERR)       ((float)SdkEvalPmGetSettledV(ADC_CH_V_5V , VAL_ERR))/4095.0*V_FULLSCALE_ADC/G_V_5V

/**
 * @brief Macro used to obtain the settled 3V voltage measurement.
 */
#define SdkEvalPmGetSettledV3V(VAL_ERR)       ((float)SdkEvalPmGetSettledV(ADC_CH_V_3V , VAL_ERR))/4095.0*V_FULLSCALE_ADC/G_V_3V

/**
 * @brief Macro used to obtain the settled I_R1 settled current measurement.
 */
#define SdkEvalPmGetSettledIR1(VAL_ERR)       ((float)SdkEvalPmGetSettledV(ADC_CH_I_R1 , VAL_ERR))/4095.0*V_FULLSCALE_ADC/G_I_R1

/**
 * @brief Macro used to obtain the settled I_R2 settled current measurement.
 */
#define SdkEvalPmGetSettledIR2(VAL_ERR)       ((float)SdkEvalPmGetSettledV(ADC_CH_I_R2 , VAL_ERR))/4095.0*V_FULLSCALE_ADC/G_I_R2

/**
 * @brief Macro used to regulate the RF supply voltage.
 * @param V_REF reference voltage expressed in volt.
 *        This parameter is a float.
 * @param ERR Desired steady state error expressed in milliVolt.
 *        This parameter is an uint8_t.
 */
#define SdkEvalPmRegulateVRf(V_REF , ERR)     SdkEvalPmRegulateVoltage(ADC_CH_V_RF , V_REF , ERR)

/**
 * @brief Macro used to regulate the RF supply voltage using an integral regulator.
 * @param V_REF reference voltage expressed in volt.
 *        This parameter is a float.
 */
#define SdkEvalPmRegulateVRfI(V_REF)          SdkEvalPmRegulateVoltageI(ADC_CH_V_RF , V_REF)

/**
 * @}
 */

/** @defgroup SDK_EVAL_PM_Exported_Functions            SDK EVAL PM Exported Functions
 * @{
 */

void SdkEvalPmI2CInit(void);
uint8_t SdkEvalPmDigipotWrite(SdkEvalDigipot xDigipot, uint8_t cDValue);
uint8_t SdkEvalPmDigipotRead(SdkEvalDigipot xDigipot, uint8_t* cDValue);
void SdkEvalPmADCInit(void);
uint16_t SdkEvalPmGetV(SdkEvalAdcChannel xAdcCh);
uint16_t SdkEvalPmGetSettledV(SdkEvalAdcChannel xAdcCh , uint16_t nError);
void SdkEvalPmRegulateVoltage(SdkEvalAdcChannel xAdcCh , float fVref , uint8_t cError);
void SdkEvalPmRegulateVoltageI(SdkEvalAdcChannel xAdcCh , float fVref);
void SdkEvalPmRfSwitchInit(void);
void SdkEvalPmRfSwitchToVRf(void);
void SdkEvalPmRfSwitchToRcal(void);

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
