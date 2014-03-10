/**
 * @file    SDK_EVAL_PM.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file provides all the low level API to manage SDK Digipots.
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
#include "SDK_EVAL_PM.h"


/** @addtogroup SDK_EVAL
 * @{
 */


/** @addtogroup SDK_EVAL_PM
 * @{
 */

/** @defgroup SDK_EVAL_PM_Private_TypesDefinitions      SDK EVAL PM Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_PM_Private_Defines               SDK EVAL PM Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_PM_Private_Macros                SDK EVAL PM Private Macros
 * @{
 */

#define ABS(a)      (a>0 ? a:-a)

/**
 * @brief Timeout used to notify errors on I2C operations.
 */
#define I2C_WAIT_TIMER()    {\
                              static uint16_t s_nTimeout=0xFFFF;\
                              if(--s_nTimeout == 0){\
                                 s_nTimeout=0xFFFF;\
                                 return I2C_DIGIPOT_ERROR;\
                              }\
                            }


/**
 * @}
 */


/** @defgroup SDK_EVAL_PM_Private_Variables                     SDK EVAL PM Private Variables
 * @{
 */

static const SdkEvalDigipotAddress s_vectxDigipotAddress[2] = { DIGIPOT1_ADDRESS , DIGIPOT2_ADDRESS};


/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_PM_Private_FunctionPrototypes             SDK EVAL PM Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_PM_Private_Functions                      SDK EVAL PM Private Functions
 * @{
 */

/**
 * @brief  Configures the I2C interface and the correspondent GPIO pins.
 * @param  None.
 * @retval None.
 */
void SdkEvalPmI2CInit(void){

  I2C_InitTypeDef I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enables I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* GPIO AF configuration */
  GPIO_PinAFConfig(DIGIPOT_I2C_SCL_Port, DIGIPOT_I2C_SCL_RCC_SOURCE, DIGIPOT_I2C_SCL_AF);
  GPIO_PinAFConfig(DIGIPOT_I2C_SDA_Port, DIGIPOT_I2C_SDA_RCC_SOURCE, DIGIPOT_I2C_SDA_AF);

  /* Configures I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  DIGIPOT_I2C_SCL_Pin | DIGIPOT_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configures I2C */
  I2C_DeInit(DIGIPOT_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = DIGIPOT_I2C_SPEED;

  /* DIGIPOT_I2C Init */
  I2C_Init(DIGIPOT_I2C, &I2C_InitStructure);

  /* DIGIPOT_I2C Enable */
  I2C_Cmd(DIGIPOT_I2C, ENABLE);

}


/**
 * @brief  Writes an 8-bit value in the RDAC register. The potentiometer value in ohm is
 *         given by the formula (cDValue/256*Rab + Rw), where Rab = xxxx and Rw = 60 ohm.
 * @param  xDigipot specifies what digipot has to be set.
 *         This parameter can be DIGIPOT1 or DIGIPOT2.
 * @param  cDValue D value to be written in the RDAC register.
 *         This parameter is an uint8_t.
 * @retval uint8_t Notifies if an I2C error has occured or if the communication has been correctly done.
 *         This parameter can be I2C_DIGIPOT_OK or I2C_DIGIPOT_ERROR.
 */
uint8_t SdkEvalPmDigipotWrite(SdkEvalDigipot xDigipot, uint8_t cDValue)
{

  /* Test on BUSY flag */
  while (I2C_GetFlagStatus(DIGIPOT_I2C,I2C_FLAG_BUSY))I2C_WAIT_TIMER();

  /* Enables the I2C peripheral */
  I2C_GenerateSTART(DIGIPOT_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(DIGIPOT_I2C, I2C_EVENT_MASTER_MODE_SELECT)) I2C_WAIT_TIMER();

  /* Transmits the slave address and enable writing operation */
  I2C_Send7bitAddress(DIGIPOT_I2C, s_vectxDigipotAddress[xDigipot], I2C_Direction_Transmitter);

  /* Test on EV6 flag */
  while (!I2C_CheckEvent(DIGIPOT_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) I2C_WAIT_TIMER();

  /* Transmits the instruction byte */
  I2C_SendData(DIGIPOT_I2C, 0x00);

  /* Test on TXE flag (data sent) */
  while (!I2C_GetFlagStatus(DIGIPOT_I2C,I2C_FLAG_TXE)) I2C_WAIT_TIMER();

  /* Transmits the first address for r/w operations */
  I2C_SendData(DIGIPOT_I2C, cDValue);

  /* Test on TXE flag (data sent) */
  while (!I2C_GetFlagStatus(DIGIPOT_I2C,I2C_FLAG_TXE)) I2C_WAIT_TIMER();

  /* Test on BTF flag before the STOP command */
  while (!I2C_GetFlagStatus(DIGIPOT_I2C,I2C_FLAG_BTF)) I2C_WAIT_TIMER();

  /* Sends STOP Condition */
  I2C_GenerateSTOP(DIGIPOT_I2C, ENABLE);

  /* Returns I2C operation OK value */
  return I2C_DIGIPOT_OK;
}

/**
 * @brief  Returns the RDAC register. The potentiometer value in ohm is
 *         given by the formula (cDValue/256*Rab + Rw), where Rab = xxxx and Rw = 60 ohm.
 * @param  xDigipot specifies what digipot has to be set.
 *         This parameter can be DIGIPOT1 or DIGIPOT2.
 * @param  pcDValue pointer to the variable in which the D value has to be stored.
 *         This parameter is an uint8_t*.
 * @retval uint8_t Notifies if an I2C error has occured or if the communication has been correctly done.
 *         This parameter can be I2C_DIGIPOT_OK or I2C_DIGIPOT_ERROR.
 */
uint8_t SdkEvalPmDigipotRead(SdkEvalDigipot xDigipot, uint8_t* pcDValue)
{
  /* Test on BUSY flag */
  while (I2C_GetFlagStatus(DIGIPOT_I2C,I2C_FLAG_BUSY)) I2C_WAIT_TIMER();

  /* Sends START condition */
  I2C_GenerateSTART(DIGIPOT_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(DIGIPOT_I2C, I2C_EVENT_MASTER_MODE_SELECT)) I2C_WAIT_TIMER();

  /* Sends address for read */
  I2C_Send7bitAddress(DIGIPOT_I2C, s_vectxDigipotAddress[xDigipot], I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(DIGIPOT_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) I2C_WAIT_TIMER();

  /* Disables acknowledgement */
  I2C_AcknowledgeConfig(DIGIPOT_I2C, DISABLE);

  /* Test on EV7 and clear it */
  while(!I2C_CheckEvent(DIGIPOT_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) I2C_WAIT_TIMER();

  /* Read a byte from DIGIPOT */
  *pcDValue = I2C_ReceiveData(DIGIPOT_I2C);

  /* Sends STOP Condition */
  I2C_GenerateSTOP(DIGIPOT_I2C, ENABLE);

  /* Enables acknowledgement to be ready for a new I2C operation */
  I2C_AcknowledgeConfig(DIGIPOT_I2C,ENABLE);

  /* Returns I2C operation OK value */
  return I2C_DIGIPOT_OK;

}


/**
 * @brief  Configures the Analog-to-Digital Converter and the correspondent GPIO pins.
 * @param  None.
 * @retval None.
 */
void SdkEvalPmADCInit(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enables ADC1 and GPIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Configures ADC pins */
  GPIO_InitStructure.GPIO_Pin = ADC_V_RF_Pin | ADC_V_5V_Pin | ADC_V_3V_Pin | ADC_I_R1_Pin | ADC_I_R2_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configures ADC periferial */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;

  ADC_Init(ADC1, &ADC_InitStructure);

}


/**
 * @brief  Samples, converts and returns the voltage on the specified ADC channel.
 * @param  xAdcCh ADC channel.
 *         This parameter can be any value of @ref SdkEvalAdcChannel.
 * @retval uint16_t Converted voltage in an unsiged integer 16-bit format.
 */
uint16_t SdkEvalPmGetV(SdkEvalAdcChannel xAdcCh)
{
  uint16_t convValue;

  /* ADC1 channel configuration */
  ADC_RegularChannelConfig(ADC1, xAdcCh, 1, ADC_SampleTime_48Cycles);

  /* Enables HSI clock */
  RCC_HSICmd(ENABLE);

  /* Enables ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Waits until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);

  /* Waits until the end of conversion */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  /* Gets the ADC conversion value */
  convValue = ADC_GetConversionValue(ADC1);

  /* Disables ADC */
  ADC_Cmd(ADC1, DISABLE);

  /* Disables HSI clock */
  RCC_HSICmd(DISABLE);

  /* Return the converted value */
  return convValue;

}

/**
 * @brief  Samples, converts and returns the voltage on the specified ADC channel.
 *         It waits for the value to be constant (with a nError tolerance) before the conversion
 * @param  xAdcCh ADC channel.
 *         This parameter can be any value of @ref SdkEvalAdcChannel.
 * @param  nError tolerance to be used to consider settled the signal to be acquired.
 *         This parameter is a uint16_t.
 * @retval uint16_t Converted voltage in an unsiged integer 16-bit format.
 */
uint16_t SdkEvalPmGetSettledV(SdkEvalAdcChannel xAdcCh , uint16_t nError)
{
  uint16_t valC, valP;

  /* Gets a first sample */
  valC=SdkEvalPmGetV(xAdcCh);

  do{
    uint16_t i;
    /* store the previous value */
    valP=valC;

    /* get the value at this time */
    valC=SdkEvalPmGetV(xAdcCh);

    /* waits some cycles */
    for(i=0 ; i<0x0FFF ; i++);

    /* exit when the residual error is less than nError */
  } while(ABS((int)valP-(int)valC)>(int)nError);

  /* Returns the last sample */
  return valC;

}

/**
 * @brief  Implements a control loop to make the specified voltage equal
 *         to a reference one (with a specified tolerance on the steady state).
 *         The control loop ends if the error value is less of cError or if
 *         a number of 7 zero crossing events have been reached by the error.
 * @param  xAdcCh ADC channel.
 *         This parameter can be any value of ADC_CH_V_RF or ADC_CH_V_MCU.
 * @param  fVref Reference voltage to be reached by the specified channel 3
 *         voltage expressed in Volt.
 *         This parameter is a float.
 * @param  cError Desired steady state error expressed in milliVolt.
 *         This parameter is an uint8_t.
 * @retval None.
 */
void SdkEvalPmRegulateVoltage(SdkEvalAdcChannel xAdcCh , float fVref , uint8_t cError)
{
  uint8_t cD = 0x7F , nZeros = 0;
  int nVerrC , nVerrP;
  float fVcon;

  if(xAdcCh == ADC_CH_V_RF)
  {
    uint32_t i;
    /* sets the digipot value */
    SdkEvalPmDigipotWrite(DIGIPOT1, cD);

    /* waits for the steady state */
    for(i=0 ; i<0x3FFF ; i++);

    /* gets the new output value */
    fVcon=SdkEvalPmGetVRf();
  }

  /* Computes the error for the first time */
  nVerrC = (int)((fVref - fVcon)*1000);

  /* stops the control loop if the current error is under cError or if the
    number of zero crossing events is 7 */
  while(!(ABS(nVerrC) < cError || nZeros == 7)){

    /* uses the sign of the error to determine if increment or decrease the potentiometer value */
    if(nVerrC>0)
      cD++;
    else
      cD--;

    if(xAdcCh == ADC_CH_V_RF)
    {
      uint32_t i;
      /* sets the digipot value */
      SdkEvalPmDigipotWrite(DIGIPOT1, cD);

      /* waits for the steady state */
      for(i=0 ; i<0x3FFF ; i++);

      /* gets the new output value */
      fVcon=SdkEvalPmGetVRf();
    }
    

    /* stores the previous error value */
    nVerrP = nVerrC;

    /* computes the new error value */
    nVerrC = (int)((fVref - fVcon)*1000);

    /* counts the number of zero crossing events */
    if((nVerrC<0 && nVerrP>0) || (nVerrC>0 && nVerrP<0))
      nZeros++;

  }

}

/**
 * @brief  Implements a control loop to make the specified voltage equal
 *         to a reference one (with a specified tolerance on the steady state).
 *         The control loop vanishes an integral error and it ends when this error
 *         becomes constant.
 * @param  xAdcCh ADC channel.
 *         This parameter can be any value of ADC_CH_V_RF or ADC_CH_V_MCU.
 * @param  fVref Reference voltage to be reached by the specified channel 3
 *         voltage expressed in Volt.
 *         This parameter is a float.
 * @retval None.
 */
void SdkEvalPmRegulateVoltageI(SdkEvalAdcChannel xAdcCh , float fVref)
{
  float fKi=0.13, fVcon;
  int cD = 0x7F;
  int nVerrC, nVerrP=0;

  if(xAdcCh == ADC_CH_V_RF)
  {
    volatile uint32_t i;
    /* sets the digipot value */
    SdkEvalPmDigipotWrite(DIGIPOT1, cD);

    /* waits for the steady state */
    for(i=0 ; i<0xFFFF ; i++);

    /* gets the new output value */
    fVcon=SdkEvalPmGetVRf();
  }

  /* computes the new error value */
  nVerrC = (int)((fVref - fVcon)*1000);

  while(nVerrC != nVerrP){
    /* computes the new input value */
    cD += (int)(fKi*nVerrC);

    /* Saturate cD to be into the range [0 , 255] */
    if(cD>255) cD=255;
    else if(cD<0) cD=0;

    if(xAdcCh == ADC_CH_V_RF)
    {
      volatile uint32_t i;
      /* sets the digipot value */
      SdkEvalPmDigipotWrite(DIGIPOT1, cD);

      /* waits for the steady state */
      for(i=0 ; i<0x3FFF ; i++);

      /* gets the new output value */
      fVcon=SdkEvalPmGetVRf();
    }

    /* stores the previous error value */
    nVerrP = nVerrC;

    /* computes the new error value */
    nVerrC = (int)((fVref - fVcon)*1000);

  }
}

/**
 * @brief  Configures the RF supply voltage switch GPIO.
 * @param  None.
 * @retval None.
 */
void SdkEvalPmRfSwitchInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enables the switch and GPIO clocks */
  RCC_AHBPeriphClockCmd(RF_SWITCH_RCC, ENABLE);

  /* Configures the switch GPIO pins */
  GPIO_InitStructure.GPIO_Pin =  RF_SWITCH_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(RF_SWITCH_Port, &GPIO_InitStructure);
}

/**
 * @brief  Sets the switch to supply RF voltage.
 * @param  None.
 * @retval None.
 */
void SdkEvalPmRfSwitchToVRf(void)
{
  RF_SWITCH_Port->BSRRH = RF_SWITCH_Pin;
}

/**
 * @brief  Sets the switch to supply voltage for the calibration resistor.
 * @param  None.
 * @retval None.
 */
void SdkEvalPmRfSwitchToRcal(void)
{
  RF_SWITCH_Port->BSRRL = RF_SWITCH_Pin;
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
