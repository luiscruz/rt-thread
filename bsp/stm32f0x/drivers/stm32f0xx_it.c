/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f0xx.h>
#include <rtthread.h>
#include "board.h"
#include "bt_app.h"
#include "led_pwm.h"

/** @addtogroup STM32F0-Discovery_Demo
  * @{
  */
extern rt_uint8_t pm_sleep_cnt;

/** @addtogroup STM32F0XX_IT
  * @brief Interrupts driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
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
  * @brief  This function handles External line 0 to 1 interrupt request.
  * @param  None
  * @retval None
  */

extern void EXTI0_Enable(rt_uint32_t enable);
extern rt_sem_t power_key_sem;

void EXTI0_1_IRQHandler(void)
{
	/* enter interrupt */
    rt_interrupt_enter();
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI0_Enable(0);	/* disable power key irq */

 		rt_sem_release(power_key_sem); /* let bh go */
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	/* leave interrupt */
    rt_interrupt_leave();
}

/**
  * @brief  This function handles External lines 4 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
	extern void bt_status_update(void);
	/* enter interrupt */
    rt_interrupt_enter();
    pm_sleep_cnt = 0;
	if(EXTI_GetITStatus(BT_IND1_EXTI_LINE) != RESET)
	{
		bt_status_update();

		/* Clear the EXTI line 8 pending bit */
		EXTI_ClearITPendingBit(BT_IND1_EXTI_LINE);
	}

	if(EXTI_GetITStatus(BT_IND2_EXTI_LINE) != RESET)
	{

		bt_status_update();
		/* Clear the EXTI line 9 pending bit */
		EXTI_ClearITPendingBit(BT_IND2_EXTI_LINE);
	}

	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{

		/* Clear the EXTI line 13 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
    /* leave interrupt */
    rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
#ifdef RT_USING_UART1
    extern struct rt_device uart1_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

    /* enter interrupt */
    rt_interrupt_enter();
	pm_sleep_cnt = 0;
    rt_hw_serial_isr(&uart1_device);

    /* leave interrupt */
    rt_interrupt_leave();
#endif
}

#ifdef RT_USING_UART2
/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
    extern struct rt_device uart2_device;
	extern void rt_hw_serial_isr(struct rt_device *device);

    /* enter interrupt */
    rt_interrupt_enter();
	//pm_sleep_cnt = 0;
    rt_hw_serial_isr(&uart2_device);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

extern struct led_dev *leds;
/* config pwm timer again after the first blink
 * PA1	"face_led LED0"		TIM2_CH2
 * PA6	"phone_led LED2"	TIM16_CH1
 * PA7	"mail_led LED3"		TIM17_CH1
 * PB0	"social_led LED4"	TIM3_CH3
 * PB1	"crown_led LED1"	TIM14_CH1
 */

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
	isr_update_pwm(LED_FACE);

  /* TIM3_CH2 toggling with frequency = 1171.8 Hz */
  /*
  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    capture = TIM_GetCapture2(TIM2);
    TIM_SetCompare2(TIM2, capture + CCR2_Val);
  }*/

}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	isr_update_pwm(LED_SOCIAL);
  /* TIM3_CH3 toggling with frequency = 2343.75 Hz */
  /*if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture + CCR3_Val);
  }*/

}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM14_IRQHandler(void)
{
	isr_update_pwm(LED_CROWN);
  /* TIM14_CH1 toggling with frequency = 585.9 Hz */
  /*if (TIM_GetITStatus(TIM14, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1 );
    capture = TIM_GetCapture1(TIM14);
    TIM_SetCompare1(TIM14, capture + CCR1_Val );
  }*/

}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM16_IRQHandler(void)
{
	isr_update_pwm(LED_PHONE);
  /* TIM16_CH1 toggling with frequency = 585.9 Hz */
  /*if (TIM_GetITStatus(TIM16, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM16, TIM_IT_CC1 );
    capture = TIM_GetCapture1(TIM16);
    TIM_SetCompare1(TIM3, capture + CCR1_Val );
  }*/
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM17_IRQHandler(void)
{
	isr_update_pwm(LED_MAIL);
  /* TIM17_CH1 toggling with frequency = 585.9 Hz */
  /*if (TIM_GetITStatus(TIM17, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM17, TIM_IT_CC1 );
    capture = TIM_GetCapture1(TIM17);
    TIM_SetCompare1(TIM17, capture + CCR1_Val );
  }*/

}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
