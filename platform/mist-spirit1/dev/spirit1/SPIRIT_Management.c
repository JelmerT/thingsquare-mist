/**
* @file    SPIRIT_Management.c
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


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Management.h"


/**
* @addtogroup SPIRIT_DK                   SPIRIT DK
* @{
*/


/**
* @defgroup SDK_SPIRIT_MANAGEMENT              SDK SPIRIT Management
* @{
*/

#include "SPIRIT_Config.h"
/**
* @brief  Factor is: B/2 used in the formula for SYNTH word calculation
*/
static const uint8_t s_vectcBHalfFactor[4]={(HIGH_BAND_FACTOR/2), (MIDDLE_BAND_FACTOR/2), (LOW_BAND_FACTOR/2), (VERY_LOW_BAND_FACTOR/2)};

/**
* @brief  BS value to write in the SYNT0 register according to the selected band
*/
static const uint8_t s_vectcBandRegValue[4]={SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32};

/**
* @brief A map that contains the SPIRIT version
*/
const SpiritVersionMap xSpiritVersionMap[] =
{
  /* The Control Board frame handler functions */
  {CUT_2_1v4, SPIRIT_VERSION_2_1},
  {CUT_2_1v3, SPIRIT_VERSION_2_1},
  {CUT_3_0, SPIRIT_VERSION_3_0},
  
};

/**
* @brief This flag is used to synchronize the TIM3 ISR with the XtalMeasurement routine.
*/
static volatile FlagStatus s_xTIM3Ch4CompareModeRaised = RESET;

/**
* @brief This flag is used to synchronize the TIM3 ISR with the XtalMeasurement routine.
*/
static volatile RfBoardIdentification s_xRfBoardIdentification = { SPIRIT_VERSION_2_0, RANGE_EXT_NONE, 52000000, HIGH_BAND };

#define COMMUNICATION_STATE_TX 0
#define COMMUNICATION_STATE_RX 1

static uint32_t s_nDesiredFrequency;
volatile static uint8_t s_cCommunicationState = COMMUNICATION_STATE_TX;


/**
* @defgroup SDK_SPIRIT_MANAGEMENT_FUNCTIONS    SDK SPIRIT Management Functions
* @{
*/

/**
* @defgroup IDENTIFICATION_FUNCTIONS      SDK SPIRIT Management Identification Functions
* @{
*/

void SpiritManagementSetXtalFrequency(uint32_t xtalFrequency)         
{
  s_xRfBoardIdentification.nXtalFrequency = xtalFrequency;
}

void SpiritManagementSetVersion(SpiritVersion version)                     
{
  s_xRfBoardIdentification.xSpiritVersion = version;
}

/**
* @brief This function handles TIM3 global interrupt
* @param None.
* @retval None.
*/
void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3, TIM_IT_CC4))
  {
    /* Set the TIM3 Compare IRQ flag */
    s_xTIM3Ch4CompareModeRaised = SET;
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    
  }
  else if(TIM_GetITStatus(TIM3, TIM_IT_CC2))
  {
    /* Set the TIM3 Compare IRQ flag */
    s_xTIM3Ch4CompareModeRaised = SET;
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    
  }
}

#define ENABLE_TCXO()           { GPIO_SetBits(GPIOC,GPIO_Pin_2); \
uint8_t tmp = 0xD0; SpiritSpiWriteRegisters(0x01, 1, &tmp);}

/**
* @brief  This function can be used to automatically measure the XTAL frequency making use of the
*         Spirit clock output to pin and an STM32L timer in compare mode.
* @param  None.
* @retval None.
*/
#define N_SAMPLES 20
#define SETTLING_PERIODS 4
#define A 0.4
uint32_t SpiritManagementComputeXtalFrequency(void)
{
  /* Instance the variables used to compute the XTAL frequency */
  uint8_t CaptureNumber=0;
  uint16_t IC3ReadValue1=0,IC3ReadValue2=0,Capture=0;
  volatile uint16_t cWtchdg = 0;
  uint32_t TIM3Freq=0,lXoFreq=0;
  float fXoFreqRounded;

  uint32_t lMeasuredXtalFrequency;
  
  uint32_t i;

  //#warning It is more safe disable all the other interrupt source.
  /* MCU GPIO, NVIC and timer configuration structures */
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  
  
  if(s_xRfBoardIdentification.xRangeExtType==RANGE_EXT_SKYWORKS)
  {
    ENABLE_TCXO();
  }
  /* Instance the structure used to configure the Spirit clock to be exported on the GPIO_0 pin. */
  SGpioInit xGpioClock=
  {
    SPIRIT_GPIO_0,
    SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
    SPIRIT_GPIO_DIG_OUT_MCU_CLOCK
  };
  
  /* Instance the structure used to configure the Spirit clock frequency to be divided by a 192 factor. */
  ClockOutputInit xClockOut = 
  {
    XO_RATIO_1_192,
    RCO_RATIO_1,
    EXTRA_CLOCK_CYCLES_0
  };
  
  
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* GPIOC clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* TIM3 channel 2 pin (PC.07) configuration */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  
  /* Configure the timer compare channel 2 */
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  /* Configure the timer IRQ to be raised on the rising fronts */
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  /* Input capture selection setting */
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  /* Input capture prescaler setting. Setting it to TIM_ICPSC_DIV8 makes the IRQ are raised every 8 rising fronts detected by hardware.  */
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  /* Disable every kind of capture filter */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  
  /* Timer 3 initialization */
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
  
  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_2_0)
  {
    /* Disable the clock divider to measure the max frequency of the clock. */
    uint8_t tmp= 0x29;
    SpiritSpiWriteRegisters(0xB4, 1, &tmp);
  }
  
  /* Spirit1 side clock configuration */
  SpiritGpioClockOutputInit(&xClockOut);
  SpiritGpioInit(&xGpioClock);
  SpiritGpioClockOutput(S_ENABLE);
  
  /* measure the frequency and average it on N_SAMPLES. Moreover cycle to wait for same SETTLING_PERIODS */
  for(i=0;i<2*(N_SAMPLES+SETTLING_PERIODS);i++)
  {
    /* block the routine until the TIM3 CCP2 IRQ is raised */
    while(!s_xTIM3Ch4CompareModeRaised && (cWtchdg!=0xFFFF))
      cWtchdg++;    
    
    if(cWtchdg==0xFFFF)
      break;
    else
      cWtchdg=0;
    
    /* reset the IRQ raised flag */
    s_xTIM3Ch4CompareModeRaised = RESET;
    
    /* if the SETTLING PERIODS expired */
    if(i>=SETTLING_PERIODS*2)
    {
      /* First TIMER3 capture */
      if(CaptureNumber == 0)
      {
        /* Get the Input Capture value */
        IC3ReadValue1 = TIM_GetCapture2(TIM3);
        CaptureNumber = 1;
      }
      /* Second TIMER3 capture */
      else if(CaptureNumber == 1)
      {
        /* Get the Input Capture value */
        IC3ReadValue2 = TIM_GetCapture2(TIM3);
        
        /* Capture computation */
        if (IC3ReadValue2 > IC3ReadValue1)
        {
          /* If the TIMER3 didn't overflow between the first and the second capture. Compute it as the difference between the second and the first capture values. */
          Capture = (IC3ReadValue2 - IC3ReadValue1) - 1;
        }
        else
        {
          /* .. else, if overflowed 'roll' the first measure to be complementar of 0xFFFF */
          Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2) - 1;
        }
        
        /* Punctual frequency computation */
        TIM3Freq = (uint32_t) SystemCoreClock / Capture;
        
        /* Averaged frequency computation */
        lXoFreq =(uint32_t)(A*(float)lXoFreq+(1.0-A)*(float)TIM3Freq);
        
        CaptureNumber = 0;
      }
    }
  }
  
  /* Compute the real frequency in Hertz tanking in account the MCU and Spirit divisions */
  lXoFreq *=(192*8);
  
  /* Disable the output clock */
  SpiritGpioClockOutput(S_DISABLE);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, DISABLE);
  
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
  
  /* SPIRIT GPIO 0 to the default configuration */
  SpiritGpioSetLevel(SPIRIT_GPIO_0, LOW);
  
  if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_2_0)
  {
    uint8_t tmp= 0x21;
    SpiritSpiWriteRegisters(0xB4, 1, &tmp);
  }
  
  /* Round the measured frequency to be measured as an integer MHz value */
  fXoFreqRounded = (float)lXoFreq/1e6;
  
  if( fXoFreqRounded-(float)((uint32_t)fXoFreqRounded)>0.5)    
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded+1)*1000000);
  else
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded)*1000000);
  
  SdkEvalM2SGpioInit(M2S_GPIO_0, M2S_MODE_GPIO_IN);
  
  SpiritRadioSetXtalFrequency(lMeasuredXtalFrequency);
  
  return lMeasuredXtalFrequency;
  
  
}

/* This function is used to detect the pa ext board, due to the unworking measurement algorithm */
uint32_t SpiritManagementComputeXtalFrequencyGpio2(void)
{
  
  /* MCU GPIO, NVIC and timer configuration structures */
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  
  uint32_t lMeasuredXtalFrequency;
  
  if(s_xRfBoardIdentification.xRangeExtType==RANGE_EXT_SKYWORKS)
  {
    ENABLE_TCXO();
  }
  /* Instance the structure used to configure the Spirit clock to be exported on the GPIO_2 pin. */
  SGpioInit xGpioClock=
  {
    SPIRIT_GPIO_2,
    SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
    SPIRIT_GPIO_DIG_OUT_MCU_CLOCK
  };
  
  /* Instance the structure used to configure the Spirit clock frequency to be divided by a 192 factor. */
  ClockOutputInit xClockOut = 
  {
    XO_RATIO_1_192,
    RCO_RATIO_1,
    EXTRA_CLOCK_CYCLES_0
  };
  
  /* Instance the variables used to compute the XTAL frequency */
  uint8_t CaptureNumber=0;
  uint16_t IC3ReadValue1=0,IC3ReadValue2=0,Capture=0;
  volatile uint16_t cWtchdg = 0;
  uint32_t TIM3Freq=0,lXoFreq=0;
  float fXoFreqRounded;

  uint32_t i;
  
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* GPIOC clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* TIM3 channel 2 pin (PC.07) configuration */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
  
  /* Configure the timer compare channel 2 */
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_4;
  /* Configure the timer IRQ to be raised on the rising fronts */
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  /* Input capture selection setting */
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  /* Input capture prescaler setting. Setting it to TIM_ICPSC_DIV8 makes the IRQ are raised every 8 rising fronts detected by hardware.  */
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  /* Disable every kind of capture filter */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  
  /* Timer 3 initialization */
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
  /* Enable the CC4 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
  
  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_2_0)
  {
    /* Disable the clock divider to measure the max frequency of the clock. */
    uint8_t tmp= 0x29;
    SpiritSpiWriteRegisters(0xB4, 1, &tmp);
  }
  
  /* Spirit1 side clock configuration */
  SpiritGpioClockOutputInit(&xClockOut);
  SpiritGpioInit(&xGpioClock);
  SpiritGpioClockOutput(S_ENABLE);
  
  /* measure the frequency and average it on N_SAMPLES. Moreover cycle to wait for same SETTLING_PERIODS */
  for(i=0;i<2*(N_SAMPLES+SETTLING_PERIODS);i++)
  {
    /* block the routine until the TIM3 CCP2 IRQ is raised */
    while(!s_xTIM3Ch4CompareModeRaised && (cWtchdg!=0xFFFF))
      cWtchdg++;    
    
    if(cWtchdg==0xFFFF)
      break;
    else
      cWtchdg=0;
    
    /* reset the IRQ raised flag */
    s_xTIM3Ch4CompareModeRaised = RESET;
    
    /* if the SETTLING PERIODS expired */
    if(i>=SETTLING_PERIODS*2)
    {
      /* First TIMER3 capture */
      if(CaptureNumber == 0)
      {
        /* Get the Input Capture value */
        IC3ReadValue1 = TIM_GetCapture4(TIM3);
        CaptureNumber = 1;
      }
      /* Second TIMER3 capture */
      else if(CaptureNumber == 1)
      {
        /* Get the Input Capture value */
        IC3ReadValue2 = TIM_GetCapture4(TIM3);
        
        /* Capture computation */
        if (IC3ReadValue2 > IC3ReadValue1)
        {
          /* If the TIMER3 didn't overflow between the first and the second capture. Compute it as the difference between the second and the first capture values. */
          Capture = (IC3ReadValue2 - IC3ReadValue1) - 1;
        }
        else
        {
          /* .. else, if overflowed 'roll' the first measure to be complementar of 0xFFFF */
          Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2) - 1;
        }
        
        /* Punctual frequency computation */
        TIM3Freq = (uint32_t) SystemCoreClock / Capture;
        
        /* Averaged frequency computation */
        lXoFreq =(uint32_t)(A*(float)lXoFreq+(1.0-A)*(float)TIM3Freq);
        
        CaptureNumber = 0;
      }
    }
  }
  
  /* Compute the real frequency in Hertz tanking in account the MCU and Spirit divisions */
  lXoFreq *=(192*8);
  
  /* Disable the output clock */
  SpiritGpioClockOutput(S_DISABLE);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, DISABLE);
  
  /* Enable the CC4 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);
  
  /* SPIRIT GPIO 2 to the default configuration */
  SpiritGpioSetLevel(SPIRIT_GPIO_2, LOW);
  
  if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_2_0)
  {
    uint8_t tmp= 0x21;
    SpiritSpiWriteRegisters(0xB4, 1, &tmp);
  }
  
  /* Round the measured frequency to be measured as an integer MHz value */
  fXoFreqRounded = (float)lXoFreq/1e6;
  
  if( fXoFreqRounded-(float)((uint32_t)fXoFreqRounded)>0.5)    
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded+1)*1000000);
  else
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded)*1000000);
  
  SdkEvalM2SGpioInit(M2S_GPIO_2, M2S_MODE_GPIO_IN);
  
  SpiritRadioSetXtalFrequency(lMeasuredXtalFrequency);
  
  return lMeasuredXtalFrequency;
}


/**
* @brief  Compute the SPIRIT version.
* @param  None.
* @retval SpiritVersion The dentified version of the SPIRIT.
*/
void SpiritManagementComputeSpiritVersion(void)
{
  uint16_t nSpiritVersion = (uint16_t)(SpiritGeneralGetDevicePartNumber()<<8) | (uint16_t)SpiritGeneralGetDeviceVersionNumber();
  SpiritVersion xSpiritVersion;
  int i;
  
  for(i=0; i<CUT_MAX_NO; i++)
  {
    if(xSpiritVersionMap[i].nSpiritVersion == nSpiritVersion)
    {
      xSpiritVersion = xSpiritVersionMap[i].xSpiritVersion;
      break;
    }
  }
  
  /* Check if the digital divider works (cut 2.1) or not (cut 2.0) */
  if(xSpiritVersion == SPIRIT_VERSION_2_1)  
  {
    uint8_t cXtalFreq = SpiritManagementComputeXtalFrequencyGpio2()/1000000;
    
    if(cXtalFreq>=24 && cXtalFreq<=26)
    {
      /* Disable the clock divider to measure the max frequency of the clock. */
      uint8_t tmp= 0x29;
      SpiritSpiWriteRegisters(0xB4, 1, &tmp);
      
      cXtalFreq = SpiritManagementComputeXtalFrequencyGpio2()/1000000;
      
      if(cXtalFreq>=24 && cXtalFreq<=26)
        xSpiritVersion = SPIRIT_VERSION_2_0;
      else
        xSpiritVersion = SPIRIT_VERSION_2_1;
      
      tmp= 0x21;
      SpiritSpiWriteRegisters(0xB4, 1, &tmp);
    }
    else
      if(cXtalFreq>=12 && cXtalFreq<=13)
      {
        xSpiritVersion = SPIRIT_VERSION_2_1;
      }
      else
      {
        s_xRfBoardIdentification.xRangeExtType = RANGE_EXT_SKYWORKS;
      }
    
  }
  
  SpiritGeneralSetSpiritVersion(xSpiritVersion);
  
  s_xRfBoardIdentification.xSpiritVersion = xSpiritVersion;
  
}

#define VERY_HIGH_BAND 4
#define RFU_BAND 5
BandSelect SpiritManagementComputeBand(void)
{
  SdkEvalPmRegulateVRfI(3.0);
  float fVoltageLevel = SdkEvalPmGetVDummy3();
  SdkEvalPmRegulateVRfI(3.3);
  
  if( fVoltageLevel<0.5)
    return VERY_LOW_BAND;
  else if(fVoltageLevel>=0.5 && fVoltageLevel<1.0)
    return LOW_BAND;
  else if(fVoltageLevel>=1.0 && fVoltageLevel<1.5)
    return MIDDLE_BAND;
  else if(fVoltageLevel>=1.5 && fVoltageLevel<2.0)
    return HIGH_BAND;
  else if(fVoltageLevel>=2.0 && fVoltageLevel<2.5)
    return (BandSelect)VERY_HIGH_BAND;
  else
    return (BandSelect)RFU_BAND;
}

void SpiritManagementIdentificationRFBoard(void)
{
  SpiritManagementComputeSpiritVersion();
  s_xRfBoardIdentification.nXtalFrequency = SpiritManagementComputeXtalFrequency();
  //  s_xRfBoardIdentification.xBandSelect = SpiritManagementComputeBand(); // To fix.
  
}


RangeExtType SpiritManagementGetRangeExtender(void)
{
  return s_xRfBoardIdentification.xRangeExtType;
  
}

uint32_t SpiritManagementXtalFrequency(void)
{
  return s_xRfBoardIdentification.nXtalFrequency;
  
}

/**
* @}
*/



/**
* @defgroup WORKAROUND_FUNCTIONS              SDK SPIRIT Management Workaround Functions
* @{
*/

/**
* @brief  Private SpiritRadioSetFrequencyBase function only used in SpiritManagementWaVcoCalibration.
* @param  lFBase the base carrier frequency expressed in Hz as unsigned word.
* @retval None.
*/
void SpiritManagementSetFrequencyBase(uint32_t lFBase)
{
  uint32_t synthWord, Fc;
  uint8_t band, anaRadioRegArray[4], wcp;
  
  /* Check the parameter */
  s_assert_param(IS_FREQUENCY_BAND(lFBase));
  
  /* Search the operating band */
  if(IS_FREQUENCY_BAND_HIGH(lFBase))
    band = (uint8_t) HIGH_BAND;
  else if(IS_FREQUENCY_BAND_MIDDLE(lFBase))
    band = (uint8_t) MIDDLE_BAND;
  else if(IS_FREQUENCY_BAND_LOW(lFBase))
    band = (uint8_t) LOW_BAND;
  else if(IS_FREQUENCY_BAND_VERY_LOW(lFBase))
    band = (uint8_t) VERY_LOW_BAND;
  
  int32_t FOffset  = SpiritRadioGetFrequencyOffset();
  float fChannelSpace  = SpiritRadioGetChannelSpace();
  uint8_t cChannelNum = SpiritRadioGetChannel();
  
  /* Calculates the channel center frequency */
  Fc = (uint32_t)((float)(lFBase) + (float)FOffset +fChannelSpace*cChannelNum);
  
  if(SpiritGeneralGetSpiritVersion() == SPIRIT_VERSION_2_1 || SpiritGeneralGetSpiritVersion() == SPIRIT_VERSION_3_0)
  {
    uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv()+1;
    
    switch(band)
    {
    case VERY_LOW_BAND:
      if(Fc<161281250)
        SpiritCalibrationSelectVco(VCO_L);
      else
        SpiritCalibrationSelectVco(VCO_H);
      break;
      
    case LOW_BAND:
      if(Fc<322562500)
        SpiritCalibrationSelectVco(VCO_L);
      else
        SpiritCalibrationSelectVco(VCO_H);
      break;
      
    case MIDDLE_BAND:
      if(Fc<430083334)
        SpiritCalibrationSelectVco(VCO_L);
      else
        SpiritCalibrationSelectVco(VCO_H);
      break;
      
    case HIGH_BAND:
      if(Fc<860166667)
        SpiritCalibrationSelectVco(VCO_L);
      else
        SpiritCalibrationSelectVco(VCO_H);
    }
    
    /* Calculates the synth word */
    synthWord = (uint32_t)((float)lFBase*(float)s_vectcBHalfFactor[band]*((float)FBASE_DIVIDER*cRefDiv/s_xRfBoardIdentification.nXtalFrequency));
  }
  else
  {
    synthWord = (uint32_t)((float)lFBase*(float)s_vectcBHalfFactor[band]*((float)FBASE_DIVIDER/s_xRfBoardIdentification.nXtalFrequency));
  }
  /* Search the VCO charge pump word and set the corresponding register */
  wcp = SpiritRadioSearchWCP(Fc);
  
  /* Build the array of registers values for the analog part */
  anaRadioRegArray[0] = (uint8_t)(((synthWord>>21)&(0x0000001F))|(wcp<<5));
  anaRadioRegArray[1] = (uint8_t)((synthWord>>13)&(0x000000FF));
  anaRadioRegArray[2] = (uint8_t)((synthWord>>5)&(0x000000FF));
  anaRadioRegArray[3] = (uint8_t)(((synthWord&0x0000001F)<<3)| s_vectcBandRegValue[band]);
  
  
  /* Configures the needed Analog Radio registers */
  g_xStatus = SpiritSpiWriteRegisters(SYNT3_BASE, 4, anaRadioRegArray);
  
}

void SpiritManagementWaVcoCalibration(void)
{
  uint8_t s_cVcoWordRx;
  uint8_t s_cVcoWordTx;
  uint32_t nFreq = SpiritRadioGetFrequencyBase();
  uint8_t cRestore = 0;
  uint8_t cStandby = 0;
  
  /* Enable the reference divider if the XTAL is between 48 and 52 MHz */
  if(s_xRfBoardIdentification.nXtalFrequency>26000000)
  {
    if(!SpiritRadioGetRefDiv())
    {
      cRestore = 1;
      SpiritRadioSetRefDiv(S_ENABLE);
      SpiritManagementSetFrequencyBase(nFreq);
      nFreq = SpiritRadioGetFrequencyBase();
    }
  }
  
  //  VcoSel vco = SpiritCalibrationGetVcoSelecttion();
  //  if (vco == VCO_H)
  {
    /* Increase the VCO current */
    uint8_t tmp = 0x19;
    SpiritSpiWriteRegisters(0xA1,1,&tmp);
  }
  
  SpiritCalibrationVco(S_ENABLE);
  
  SpiritRefreshStatus();
  if(g_xStatus.MC_STATE == MC_STATE_STANDBY)
  {
    cStandby = 1;
    SpiritCmdStrobeReady();
	    do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_READY); 
  }
  
  SpiritCmdStrobeLockTx();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_LOCK);
  
  s_cVcoWordTx = SpiritCalibrationGetVcoCalData();
  
  SpiritCmdStrobeReady();
   
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_READY); 
  
  /* Enable the reference divider if the XTAL is between 48 and 52 MHz */
  if(cRestore && s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_3_0)
  {  
    SpiritManagementSetFrequencyBase(nFreq+480300);
  }

  SpiritCmdStrobeLockRx();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_LOCK);
  
  s_cVcoWordRx = SpiritCalibrationGetVcoCalData();
  
  SpiritCmdStrobeReady();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_READY);
  
  if(cStandby == 1)
  {
    SpiritCmdStrobeStandby();
    
  }
  SpiritCalibrationVco(S_DISABLE);
  
  /* Disable the reference divider if the XTAL is between 48 and 52 MHz */
  if(cRestore)
  {
    SpiritRadioSetRefDiv(S_DISABLE);
    SpiritManagementSetFrequencyBase(nFreq);
  }
  
  // if (vco == VCO_H)
  {
    /* Restore the VCO current */
    uint8_t tmp = 0x11;
    SpiritSpiWriteRegisters(0xA1,1,&tmp);
  }
  
  SpiritCalibrationSetVcoCalDataTx(s_cVcoWordTx);
  SpiritCalibrationSetVcoCalDataRx(s_cVcoWordRx);
}

void SpiritManagementWaRcoCalibration(void)
{
  if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_3_0)
  {
    if(s_xRfBoardIdentification.nXtalFrequency<48000000)
    {
      uint8_t tmp= 0x02;SpiritSpiWriteRegisters(0x50, 1, &tmp); 
      SpiritCmdStrobeStandby(); 
      tmp= 0x29;SpiritSpiWriteRegisters(0xB4, 1, &tmp); 
      tmp= 0x06;SpiritSpiWriteRegisters(0x50, 1, &tmp); 
      if(s_xRfBoardIdentification.xRangeExtType == RANGE_EXT_SKYWORKS)
      { 
        tmp= 0xD0; SpiritSpiWriteRegisters(0x01, 1, &tmp);
      } 
      SpiritCmdStrobeReady();
    }
  }
}


void SpiritManagementWaRxStartup(void)
{
  if(s_xRfBoardIdentification.xRangeExtType==RANGE_EXT_NONE)
  {
    if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_3_0)
    {
      volatile uint32_t i;
      uint8_t tmp = 0x10;

      for(i=0;i<0xFF;i++);

      SpiritSpiWriteRegisters(0xa8, 1, &tmp);
      tmp = 0x00; SpiritSpiWriteRegisters(0xa8, 1, &tmp);
    }
  }
  
  //  {
  //    uint8_t tmp1,tmp2 = 0x5F; SpiritSpiReadRegisters(0x9E, 1, &tmp1);
  //    SpiritCmdStrobeCommand(CMD_RX); 
  //    for(volatile uint32_t i=0;i<0xFF;i++);
  //    SpiritSpiWriteRegisters(0x9E, 1, &tmp2);
  //    for(volatile uint32_t i=0;i<0xFF;i++);
  //    SpiritSpiWriteRegisters(0x9E, 1, &tmp1);
  //  }
  
  //  { 
  //    SpiritCmdStrobeCommand(CMD_RX); 
  //    if(s_xRfBoardIdentification.xRangeExtType==RANGE_EXT_NONE)
  //    { 
  //      for(volatile uint32_t i=0;i<0x7F;i++);
  //      SpiritWorkaroundRx();
  //    }
  //  }
}


void SpiritManagementWaRxStartupInit(void)
{
  if(s_xRfBoardIdentification.xRangeExtType==RANGE_EXT_NONE)
  {
    if(s_xRfBoardIdentification.xSpiritVersion != SPIRIT_VERSION_3_0)
    {    
      uint8_t tmp;
      //    SpiritCmdStrobeLockRx();
      //    for(volatile uint32_t i=0;i<0xFF;i++);
      //    SpiritSpiReadRegisters(0xe5,1,&tmp);
      //    SpiritCmdStrobeReady();
      //    for(volatile uint32_t i=0;i<0xFF;i++);
      //    SpiritSpiWriteRegisters(0x6f, 1, &tmp);
      //    tmp = 0xc0; 
      //    SpiritSpiWriteRegisters(0x50, 1, &tmp);
      
      tmp = 0xA8;
      SpiritSpiWriteRegisters(0xa9, 1, &tmp);
      tmp = 0x4B; 
      SpiritSpiWriteRegisters(0xaa, 1, &tmp);
      tmp = 0xFC; 
      SpiritSpiWriteRegisters(0xab, 1, &tmp);
    }
  }
  
}


void SpiritManagementWaCmdStrobeTx(void)
{
  if(s_cCommunicationState != COMMUNICATION_STATE_TX)
  {
    if(s_xRfBoardIdentification.xSpiritVersion == SPIRIT_VERSION_2_1  && s_xRfBoardIdentification.nXtalFrequency>26000000)
    {
      SpiritManagementSetFrequencyBase(s_nDesiredFrequency);      
    }
    /* To achive the max output power */
    if(s_nDesiredFrequency>=300000000 && s_nDesiredFrequency<=470000000)
    {
      /* Optimal setting for Tx mode only */
      SpiritRadioSetPACwc(LOAD_3_6_PF);
    }
    else
    {
      /* Optimal setting for Tx mode only */
      SpiritRadioSetPACwc(LOAD_0_PF);
    }
    
    uint8_t tmp = 0x11; SpiritSpiWriteRegisters(0xa9, 1, &tmp); /* Enable VCO_L buffer */
    tmp = 0x20; SpiritSpiWriteRegisters(PM_CONFIG1_BASE, 1, &tmp); /* Set SMPS switching frequency */
    
    s_cCommunicationState = COMMUNICATION_STATE_TX;
  }
}


void SpiritManagementWaCmdStrobeRx(void)
{
  if(s_cCommunicationState != COMMUNICATION_STATE_RX)
  {
    if(s_xRfBoardIdentification.xSpiritVersion == SPIRIT_VERSION_2_1 && s_xRfBoardIdentification.nXtalFrequency>26000000)
    {
      SpiritManagementSetFrequencyBase(s_nDesiredFrequency+480300);
      SpiritManagementWaRxStartupInit();
    }
    
    uint8_t tmp = 0x90; SpiritSpiWriteRegisters(PM_CONFIG1_BASE, 1, &tmp); /* Set SMPS switching frequency */    
    SpiritRadioSetPACwc(LOAD_0_PF); /* Set the correct CWC parameter */
    
    s_cCommunicationState = COMMUNICATION_STATE_RX;
  }
}

void SpiritManagementWaTRxFcMem(uint32_t nDesiredFreq)
{
  s_cCommunicationState = COMMUNICATION_STATE_TX;
  s_nDesiredFrequency = nDesiredFreq;
}



/**
* @}
*/



/**
* @defgroup RANGE_EXT_MANAGEMENT_FUNCTIONS              SDK SPIRIT Management Range Extender Functions
* @{
*/

void SpiritManagementRangeExtInit(void)
{
  if(s_xRfBoardIdentification.xRangeExtType==RANGE_EXT_SKYWORKS)
  {
    ENABLE_TCXO();
    
    SGpioInit xGpio;
    
    /* CSD control */
    xGpio.xSpiritGpioIO = SPIRIT_GPIO_DIG_OUT_TX_RX_MODE;
    xGpio.xSpiritGpioMode = SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP;
    xGpio.xSpiritGpioPin = SPIRIT_GPIO_0;
    SpiritGpioInit(&xGpio);
    
    /* CTX/BYP control */
    xGpio.xSpiritGpioIO = SPIRIT_GPIO_DIG_OUT_TX_STATE;
    xGpio.xSpiritGpioPin = SPIRIT_GPIO_1;
    SpiritGpioInit(&xGpio);
    
    /* Vcont control */
    xGpio.xSpiritGpioIO = SPIRIT_GPIO_DIG_OUT_RX_STATE;
    xGpio.xSpiritGpioPin = SPIRIT_GPIO_2;
    SpiritGpioInit(&xGpio);
  }
  
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

/**
* @}
*/


/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
