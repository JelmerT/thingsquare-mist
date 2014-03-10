/**
* @file    SDK_EVAL_Spirit_Spi_Driver.c
* @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
* @brief    This file provides all the low level SPI API to access to SPIRIT using a software watchdog timer to avoid stuck situation.
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
*
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "SDK_EVAL_Spirit_Spi_Config.h"
#include "SPIRIT_Spi_Driver.h"
#include "spirit1-arch.h"


/** @addtogroup SDK_EVAL
* @{
*/


/** @addtogroup SDK_EVAL_Spirit_Spi
* @{
*/


/** @defgroup SPI_Private_TypesDefinitions
* @{
*/


/**
* @}
*/


/** @defgroup SPI_Private_Defines
* @{
*/


#define CS_TO_SCLK_DELAY  0x0100

/** @defgroup SPI_Headers
* @{
*/

#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/


/**
* @}
*/

/**
* @}
*/


/** @defgroup SPI_Private_Macros
* @{
*/
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/


/**
* @}
*/



/** @defgroup SPI_Private_Variables
* @{
*/


/**
* @}
*/


/** @defgroup SPI_Private_FunctionPrototypes
* @{
*/

/* these commands were previously intended for FreeRTOS with a different define,
  these were commented out; it should be safe to remove them from here. */
#define SPI_ENTER_CRITICAL()           //SdkEvalM2SGpioInterruptCmd(M2S_GPIO_3, 0x0F, 0x0F, DISABLE);
#define SPI_EXIT_CRITICAL()            //SdkEvalM2SGpioInterruptCmd(M2S_GPIO_3, 0x0F, 0x0F, ENABLE);

/**
* @}
*/



/** @defgroup SPI_Private_Functions
* @{
*/

/**
* @brief  Initializes the SPI for SPIRIT
* @param  None
* @retval None
*/
void SpiritSpiInit(void)
{
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable SPI periph clock */
  RCC_APB2PeriphClockCmd(SPIRIT_SPI_PERIPH_RCC, ENABLE);
  
  /* Enable SCLK, MOSI, MISO and CS GPIO clocks */
  RCC_AHBPeriphClockCmd(SPIRIT_SPI_PERIPH_MOSI_RCC | SPIRIT_SPI_PERIPH_MISO_RCC | SPIRIT_SPI_PERIPH_SCLK_RCC | SPIRIT_SPI_PERIPH_CS_RCC, ENABLE);
  
  /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
  GPIO_PinAFConfig(SPIRIT_SPI_PERIPH_MOSI_PORT, SPIRIT_SPI_PERIPH_MOSI_RCC_SOURCE, SPIRIT_SPI_PERIPH_MOSI_AF);
  GPIO_PinAFConfig(SPIRIT_SPI_PERIPH_MISO_PORT, SPIRIT_SPI_PERIPH_MISO_RCC_SOURCE, SPIRIT_SPI_PERIPH_MISO_AF);
  GPIO_PinAFConfig(SPIRIT_SPI_PERIPH_SCLK_PORT, SPIRIT_SPI_PERIPH_SCLK_RCC_SOURCE, SPIRIT_SPI_PERIPH_SCLK_AF);
  
  /* Configure SPI pins:SCLK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  
  GPIO_InitStructure.GPIO_Pin = SPIRIT_SPI_PERIPH_SCLK_PIN;
  GPIO_Init(SPIRIT_SPI_PERIPH_SCLK_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = SPIRIT_SPI_PERIPH_MISO_PIN;
  GPIO_Init(SPIRIT_SPI_PERIPH_MISO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = SPIRIT_SPI_PERIPH_MOSI_PIN;
  GPIO_Init(SPIRIT_SPI_PERIPH_MOSI_PORT, &GPIO_InitStructure);
  
  /* Configure SPI pin: CS */
  GPIO_InitStructure.GPIO_Pin = SPIRIT_SPI_PERIPH_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(SPIRIT_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
  
  /* Configure SPI peripheral */
  SPI_DeInit(SPIRIT_SPI_PERIPH_NB);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //SPI_BaudRatePrescaler_8; /*Baud_Rate= fpclk/4=32MHz/4=8MHZ*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPIRIT_SPI_PERIPH_NB, &SPI_InitStructure);
  
  SPI_Cmd(SPIRIT_SPI_PERIPH_NB, ENABLE); /* SPIRIT_SPI enable */
}

/**
* @brief  Write single or multiple SPIRIT register
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval SPIRIT status
*/
SpiritStatus SpiritSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[2];
  uint16_t tmpstatus = 0x0000;
  int i, index;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;
  
  /* Built the header bytes */
  header[0]=WRITE_HEADER;
  header[1]=cRegAddress;
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  {
    volatile uint32_t iv;
    for(iv = 0; iv < CS_TO_SCLK_DELAY; iv++);
  }  
  
  /* Writes the header bytes and read the SPIRIT status bytes */
  for(i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  
  /* Writes the registers according to the number of bytes */
  for(index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, pcBuffer[index]);   
  }
    
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
  
  if(SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == SET)
  {
    SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
  }

  /* Puts the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}

/**
* @brief  Read single or multiple SPIRIT register
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval SPIRIT status
*/
SpiritStatus
SpiritSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  uint8_t header[2];
  uint8_t dummy=0xFF;
  int i, index;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;
  
  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=cRegAddress;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  
  {
    volatile uint16_t iv;
    for(iv = 0; iv < CS_TO_SCLK_DELAY; iv++);
  }

  /* Write the header bytes and read the SPIRIT status bytes */
  for(i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  
  /* Read the registers according to the number of bytes */
  for(index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, dummy);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    *pcBuffer = SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
    pcBuffer++;
  }
  
  while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval SPIRIT status
*/
SpiritStatus SpiritSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t header[2];
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;
  int i;
  
  /* Built the header bytes */
  header[0]=COMMAND_HEADER;
  header[1]=cCommandCode;
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  {
    volatile uint16_t iv;
    for(iv = 0; iv < CS_TO_SCLK_DELAY; iv++);
  }
  
  /* Writes the header bytes and read the SPIRIT status bytes */
  for(i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  
  while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
  
  /* Puts the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}


/**
* @brief  Write data into TX FIFO
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval SPIRIT status
*/
SpiritStatus SpiritSpiWriteLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  uint8_t header[2];
  int i, index;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;
  
  
  /* Built the header bytes */
  header[0]=WRITE_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  
  {
    volatile uint16_t iv;
    for(iv=0; iv < CS_TO_SCLK_DELAY; iv++);
  }
  
  /* Write the header bytes and read the SPIRIT status bytes */
  for(i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  
  /* Write the data into the FIFO according to the number of bytes */
  for(index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, pcBuffer[index]);
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
  
  if(SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == SET)
  {
    SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
  }
  
  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}


/**
* @brief  Read data from RX FIFO
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval SPIRIT status
*/
SpiritStatus SpiritSpiReadLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;
  int i, index;
  
  SPI_ENTER_CRITICAL();
  
  uint8_t header[2];
  uint8_t dummy=0xFF;
  
  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;
  
  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  
  {
    volatile uint16_t iv;
    for(iv=0 ; iv < CS_TO_SCLK_DELAY; iv++);
  }
  
  /* Write the header bytes and read the SPIRIT status bytes */
  for(i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  
  /* Read the data from the FIFO according to the number of bytes */
  for(index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, dummy);
    while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET);
    *pcBuffer = SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
    pcBuffer++;
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}
/*---------------------------------------------------------------------------*/
/* XXX mlu: what does this work around? legacy function. */
void SpiritWorkaroundRx(void)
{
  uint8_t header;
  volatile uint8_t i;
  
  /* Built the header bytes */
  header=WRITE_HEADER;
  
  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  for(i = 0; i < 0x1; i++);
  SPI_SendData(SPIRIT_SPI_PERIPH_NB, header);
  header=0xA8;
  //  for(volatile uint8_t i=0;i<0x1;i++);
  SPI_SendData(SPIRIT_SPI_PERIPH_NB, header);
  for(i = 0; i < 0x1; i++);
  header = 0x10;
  SPI_SendData(SPIRIT_SPI_PERIPH_NB, header);
  for(i = 0; i < 0x1; i++);

  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
  for(i = 0; i < 0x1; i++);
  
  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();
  for(i = 0; i < 0x1; i++);
  SPI_SendData(SPIRIT_SPI_PERIPH_NB, header);
  header=0xA8;
  //  for(volatile uint8_t i=0;i<0x1;i++);
  SPI_SendData(SPIRIT_SPI_PERIPH_NB, header);
  for(i = 0; i < 0x1; i++);
  header = 0x00;
  SPI_SendData(SPIRIT_SPI_PERIPH_NB, header);
  for(i = 0; i < 0x1; i++);

  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();
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
