/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include <rtthread.h>
#include <stm32f0xx.h>
#include "board.h"
#include "bt_app.h"
#include "led.h"

#define	PB_THREAD_PRIORITY
static struct rt_thread tid;
ALIGN(RT_ALIGN_SIZE)
static char bh_thread_stack[256];

/* active high, 10K pull low, 10k pull high */
#define	POWER_KEY_PRESSED		GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)

void EXTI0_Enable(rt_uint32_t enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//EXTI_Trigger_Rising_Falling;

    if (enable)
    {
        /* enable */
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    }
    else
    {
        /* disable */
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    }

    EXTI_Init(&EXTI_InitStructure);

//    EXTI_ClearITPendingBit(EXTI_Line0);
}

/**
  * @brief  This function configures the system to enter Standby mode for
  *         current consumption measurement purpose.
  *         STANDBY Mode
  *         ============
  *           - RTC OFF
  *           - IWDG and LSI OFF
  *           - Wakeup using WakeUp Pin2 (PC.13)
  * @param  None
  * @retval None
  */
void StandbyMode_Measure(void)
{
  	/* Enable the PWR clock */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  	/* Allow access to Backup Domain */
 	PWR_BackupAccessCmd(ENABLE);

  	/* Enable WKUP pin 1 */
  	PWR_WakeUpPinCmd(PWR_WakeUpPin_1,ENABLE);

	printk("go to standby \n\n");
  	/* Clear Wakeup flag */
  	PWR_ClearFlag(PWR_FLAG_WU);

  	/* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
  	PWR_EnterSTANDBYMode();

  	/* Infinite loop */
  	while (1)
  	{
  	}
}

/**
  * @brief  This function configures the system to enter Stop mode
  *           - Regulator in LP mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in deep power down mode
  *           - Automatic Wakeup using RTC clocked by LSI (~5s)
  * @param  None
  * @retval None
  */
void StopMode_Measure(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef  NVIC_InitStructure;
//	EXTI_InitTypeDef  EXTI_InitStructure;

	/* Configure all GPIO as analog to reduce current consumption on non used IOs */
	/* Enable GPIOs clock */
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
	                       RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF , ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Disable GPIOs clock, except RCC_AHBPeriph_GPIOA GPA0 power button */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
	                       RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, DISABLE);


	printk("going to stop mode....\n\n");
	EXTI0_Enable(1);/* power button ext0 to wake up system */
	/* Enter Stop Mode */
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	/*back from stop mode, configure all gpio again */
	printk("\n\nreturn from stop mode\n");
}

rt_sem_t power_key_sem = RT_NULL;
static void power_key_bh(void *param)
{
//	int cnt=0;
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* power button : active low by external pull up 10k  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; /* GPA0 EXT0-1*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	/* Enable SYSCFG clock */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  	/* Connect EXTI0 Line to PA0 pin */
  	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  	/* Configure EXTI0 line */
  	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	/* Enable and set EXTI4_15 Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	power_key_sem = rt_sem_create("power_key sem", 0, RT_IPC_FLAG_FIFO); /* bh waits for irq */
	while(1)
	{
		rt_sem_take(power_key_sem, RT_WAITING_FOREVER);/* waits for ISR */
		printk("power button is pressed\n");

		/* power key IRQ is disabled */
		if(POWER_KEY_PRESSED){
			rt_thread_delay(500);	/* 5000ms to debounce */
			if(POWER_KEY_PRESSED){	/* power button is down now */
				//StopMode_Measure();	/*goto stop mode */
				StandbyMode_Measure();	/* the lowest power state */
				//printf("post power key down\n");

				//while(POWER_KEY_PRESSED) rt_thread_delay(1);	/* waiting for key up */

				/* post up event */

				//printf("post power key up\n");
			}
		}
		//enable power key falling edge trigger again
		EXTI0_Enable(1);
	}
}

/* cpu idle in idle thread */
void cpu_sleep(void)
{
	/* check if usart is working, if so, don't sleep to prevent lose data */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == RESET)
		PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
}

int rt_application_init()
{
	rt_err_t result;
	result = rt_thread_init(&tid,
		"power_key_bh",
		power_key_bh, RT_NULL,
		&bh_thread_stack[0], sizeof(bh_thread_stack),
		RT_THREAD_PRIORITY_MAX>>2, 10);

	if (result == RT_EOK)
		rt_thread_startup(&tid);

	led_init();

	btapp_init();

	rt_thread_idle_sethook(cpu_sleep);

    return 0;
}

/*@}*/
