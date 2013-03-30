/*
 * File      : btapp.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-03-02	  Thomas Tsai  a btapp shell downgraded from finsh
 * 2011-06-02     Bernard      Add btapp_get_prompt function declaration
 */

#ifndef __BTAPP_H__
#define __BTAPP_H__

#include <rtthread.h>
#include "board.h"

#ifndef BTAPP_THREAD_PRIORITY
#define BTAPP_THREAD_PRIORITY (RT_THREAD_PRIORITY_MAX/2)	//(RT_THREAD_PRIORITY_MAX>>3)
#endif

#ifndef BTAPP_THREAD_STACK_SIZE
#define BTAPP_THREAD_STACK_SIZE 512	//1024
#endif

#define BTAPP_CMD_SIZE		80

#define BTSPP_DEVICE "uart1"

/* gpio pins
 *
 * PA9		USART1_TX
 * PA10	USART1_RX
 * output pins */
//void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
/* BM57SPP P22 */
#define BT_RST_N_PIN    		GPIO_Pin_4
#define BT_RST_N_GPIO_PORT  	GPIOA
#define BT_RST_N_GPIO_CLK   	RCC_AHBPeriph_GPIOA
#define	BT_RESET_EN				GPIO_ResetBits(BT_RST_N_GPIO_PORT, BT_RST_N_PIN)
#define	BT_RESET_DIS			GPIO_SetBits(BT_RST_N_GPIO_PORT, BT_RST_N_PIN)

/* BM57SPP P09 */
#define BT_WAKEUP_PIN       	GPIO_Pin_5
#define BT_WAKEUP_GPIO_PORT     GPIOA
#define BT_WAKEUP_GPIO_CLK      RCC_AHBPeriph_GPIOA
#define	BT_WAKEUP_DIS			GPIO_ResetBits(BT_WAKEUP_GPIO_PORT, BT_WAKEUP_PIN)
#define	BT_WAKEUP_EN			GPIO_SetBits(BT_WAKEUP_GPIO_PORT, BT_WAKEUP_PIN)

/* BM57SPP P25 */
#define BT_PAIRING_PIN 			GPIO_Pin_8
#define BT_PAIRING_GPIO_PORT    GPIOA
#define BT_PAIRING_GPIO_CLK     RCC_AHBPeriph_GPIOA
#define	BT_PAIRING_EN			GPIO_ResetBits(BT_PAIRING_GPIO_PORT, BT_PAIRING_PIN)
#define	BT_PAIRING_DIS			GPIO_SetBits(BT_PAIRING_GPIO_PORT, BT_PAIRING_PIN)

/*BM57 P07
L: MCU/host Informs BM57 that UART data will be transmitted out after 1 ms
*/
#define BT_RX_IND_PIN 			GPIO_Pin_15
#define BT_RX_IND_GPIO_PORT    	GPIOA
#define BT_RX_IND_GPIO_CLK     	RCC_AHBPeriph_GPIOA
#define	BT_RX_IND_DIS			GPIO_SetBits(BT_PAIRING_GPIO_PORT, BT_RX_IND_PIN)
#define	BT_RX_IND_EN			GPIO_ResetBits(BT_PAIRING_GPIO_PORT, BT_RX_IND_PIN)

/* input pins */
/* BTP04 */
#define BT_IND1_PIN       		GPIO_Pin_6
#define BT_IND1_GPIO_PORT     	GPIOB
#define BT_IND1_GPIO_CLK      	RCC_AHBPeriph_GPIOB
#define	BT_IND1_EXTI_PORT		EXTI_PortSourceGPIOB
#define	BT_IND1_EXTI_SOURCE		EXTI_PinSource6
#define	BT_IND1_EXTI_LINE		EXTI_Line6
#define	BT_IND1_PIN_LEVEL		GPIO_ReadInputDataBit(BT_IND1_GPIO_PORT, BT_IND1_PIN)

/* BTP22*/
#define BT_IND2_PIN       		GPIO_Pin_7
#define BT_IND2_GPIO_PORT     	GPIOB
#define BT_IND2_GPIO_CLK      	RCC_AHBPeriph_GPIOB
#define	BT_IND2_EXTI_PORT		EXTI_PortSourceGPIOB
#define	BT_IND2_EXTI_SOURCE		EXTI_PinSource7
#define	BT_IND2_EXTI_LINE		EXTI_Line7
#define	BT_IND2_PIN_LEVEL		GPIO_ReadInputDataBit(BT_IND2_GPIO_PORT, BT_IND2_PIN)

#define	BT_IND_EXTI_IRQn        EXTI4_15_IRQn

/* 							BTP22  BTP04
 * LINK	w/o USART TX		0      0
 * LINK	w/ USART  TX		0      1
 * ACCESS					1      0
 * POWER ON					1      1
 */
typedef enum _bt_status {BT_LINK=0x00,
		BT_LINK_TX=0x01,
		BT_ACCESS=0x02,
		BT_POWER_ON =0x03,
		BT_SHUTDOWN=0x04 } BT_STATUS;

struct btapp_dev
{
	struct rt_semaphore rx_sem;

//	enum input_stat stat;

//	struct btapp_parser parser;

//	char line[BTAPP_CMD_SIZE];
//	rt_uint8_t line_position;

	//struct rt_device device;
	rt_device_t device;
};


void btapp_init(void);
void btapp_set_device(const char* device_name);
const char* btapp_get_device(void);
rt_int8_t bm57_rx_prep(rt_int8_t on);
rt_int8_t bm57_tx_prep(void);

#endif
