/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    21-March-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "usb_lib.h"
#include "SDK_EVAL_VC_Istr.h"
#include "SDK_EVAL_VC_General.h"

/** @addtogroup Template_Project
  * @{
  */

extern uint16_t counter;
extern uint8_t firstinterrupt;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
  SdkEvalLedOn(2);
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
#if 0
void HardFault_Handler(void)
{
  SdkEvalLedOn(2);
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
#endif    /* if 0; code commented out */

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
#if 0
void MemManage_Handler(void)
{
  SdkEvalLedOn(2);
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}
#endif    /* if 0; code commented out */

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
#if 0
void BusFault_Handler(void)
{
  SdkEvalLedOn(2);
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
#endif    /* if 0; code commented out */

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
#if 0
void UsageFault_Handler(void)
{
  SdkEvalLedOn(2);
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
#endif    /* if 0; code commented out */

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
  SdkEvalLedOn(2);
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
#if 0
void DebugMon_Handler(void)
{
  SdkEvalLedOn(2);
}
#endif    /* if 0; code commented out */

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
  SdkEvalLedOn(2);
}



/*******************************************************************************
* Function Name  : USB_LP_IRQHandler
* Description    : This function handles USB Low Priority interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_IRQHandler(void)
{
  SdkEvalVCIntServRoutine();
}

/*******************************************************************************
* Function Name  : USB_HP_IRQHandler
* Description    : This function handles USB High Priority interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_IRQHandler(void)
{
  SdkEvalVCIntServRoutine();
}

/******************************************************************************/
/*                 STM32L15x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_lp.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
#if 0
void PPP_IRQHandler(void)
{
  SdkEvalLedOn(2);
}
#endif    /* if 0; code commented out */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
