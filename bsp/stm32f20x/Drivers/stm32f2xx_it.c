/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include <rtthread.h>
#include "board.h"


/** @addtogroup Template_Project
  * @{
  */

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
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

#if defined(RT_USING_DFS) && STM32_USE_SDIO
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
    extern int SD_ProcessIRQSrc(void);

    /* enter interrupt */
    rt_interrupt_enter();

    /* Process All SDIO Interrupt Sources */
    if( SD_ProcessIRQSrc() == 2)
		rt_kprintf("SD Error\n");

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

extern void EXTI0_Enable(rt_uint32_t enable);
extern rt_sem_t power_key_sem;
/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* enter interrupt */
    rt_interrupt_enter();

	EXTI0_Enable(0);	/* disable power key irq */

 	rt_sem_release(power_key_sem); /* let bh go */

   	/* Clear the EXTI line 0 pending bit */
   	EXTI_ClearITPendingBit(EXTI_Line0);

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
extern void EXTI_Keyscan_Enable(rt_uint32_t enable);
extern rt_sem_t keypad_sem;
void EXTI9_5_IRQHandler(void)
{
	/* enter interrupt */
    rt_interrupt_enter();

	if( (EXTI_GetITStatus(EXTI_Line5) == SET) ||
		(EXTI_GetITStatus(EXTI_Line6) == SET) ||
		(EXTI_GetITStatus(EXTI_Line7) == SET) ){
		EXTI_Keyscan_Enable(0);
		rt_sem_release(keypad_sem); /* let bh go */
	}

	if(EXTI_GetITStatus(EXTI_Line5) == SET)
	{
	  /* Clear the EXTI line 5 pending bit */
	  EXTI_ClearITPendingBit(EXTI_Line5);
	}
	if(EXTI_GetITStatus(EXTI_Line6) == SET)
	{
	  /* Clear the EXTI line 5 pending bit */
	  EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if(EXTI_GetITStatus(EXTI_Line7) == SET)
	{
	  /* Clear the EXTI line 7 pending bit */
	  EXTI_ClearITPendingBit(EXTI_Line7);
	}

	/* leave interrupt */
	rt_interrupt_leave();
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
