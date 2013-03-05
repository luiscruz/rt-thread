/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 */

#include "usart.h"
#include <rtdevice.h>
#include <serial.h>
#include <stm32f0xx_dma.h>

/*
 * Use UART1 as console output and finsh input
 * interrupt Rx and poll Tx (stream mode)
 *
 * Use UART2 with interrupt Rx and poll Tx
 * Use UART3 with DMA Tx and interrupt Rx -- DMA channel 2
 *
 * USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 2
 * USART1 Rx --> DMA Channel 3
 * USART2 Tx --> DMA Channel 4
 * USART2 Rx --> DMA Channel 5
 */

#ifdef RT_USING_UART1
struct stm32_serial_int_rx uart1_int_rx;
struct stm32_serial_device uart1 =
{
	USART1,
	&uart1_int_rx,
	RT_NULL
};
struct rt_device uart1_device;
#endif

#ifdef RT_USING_UART2
struct stm32_serial_int_rx uart2_int_rx;
struct stm32_serial_device uart2 =
{
	USART2,
	&uart2_int_rx,
	RT_NULL
};
struct rt_device uart2_device;
#endif

#ifdef	STM32F051
USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2};

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK, EVAL_COM2_TX_GPIO_CLK};

const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK, EVAL_COM2_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};

const uint16_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE, EVAL_COM2_TX_SOURCE};

const uint16_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE, EVAL_COM2_RX_SOURCE};

const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF, EVAL_COM2_TX_AF};

const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF, EVAL_COM2_RX_AF};

#else

USART_TypeDef* COM_USART[COMn] = {EVAL_COM1};

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK};

const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN};

const uint16_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE};

const uint16_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE};

const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF};

const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF};
#endif

static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(COM_TX_PORT_CLK[0] | COM_RX_PORT_CLK[0], ENABLE);

  /* Enable USART clock */
  RCC_APB2PeriphClockCmd(COM_USART_CLK[0], ENABLE);
#endif

#ifdef RT_USING_UART2
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(COM_TX_PORT_CLK[1] | COM_RX_PORT_CLK[1], ENABLE);

  /* Enable USART clock */
  RCC_APB1PeriphClockCmd(COM_USART_CLK[1], ENABLE);
#endif

}

static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef RT_USING_UART1
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(COM_TX_PORT[0], COM_TX_PIN_SOURCE[0], COM_TX_AF[0]);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(COM_RX_PORT[0], COM_RX_PIN_SOURCE[0], COM_RX_AF[0]);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[0];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(COM_TX_PORT[0], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[0];
  GPIO_Init(COM_RX_PORT[0], &GPIO_InitStructure);
#endif

#ifdef RT_USING_UART2
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(COM_TX_PORT[1], COM_TX_PIN_SOURCE[1], COM_TX_AF[1]);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(COM_RX_PORT[1], COM_RX_PIN_SOURCE[1], COM_RX_AF[1]);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[1];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(COM_TX_PORT[1], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[1];
  GPIO_Init(COM_RX_PORT[1], &GPIO_InitStructure);
#endif

}

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef RT_USING_UART1
  	NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef RT_USING_UART2
	/* Enable the USART2 Interrupt */
 	NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

}

static void DMA_Configuration(void)
{
/* no dma supported yet */
}

#if 0
/**
  * @brief  Start Bit Method to Wake Up USART from Stop mode Test.
  * @param  None
  * @retval None
  */
static void WakeUp_StartBitMethod(void)
{
  /* Configure the wake up Method = Start bit */
  USART_StopModeWakeUpSourceConfig(USART1, USART_WakeUpSource_StartBit);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);

  /* Before entering the USART in STOP mode the REACK flag must be checked to ensure the USART RX is ready */
  while(USART_GetFlagStatus(USART1, USART_FLAG_REACK) == RESET)
  {}

  /* Enable USART STOP mode by setting the UESM bit in the CR1 register.*/
  USART_STOPModeCmd(USART1, ENABLE);

  /* Enable the wake up from stop Interrupt */
  USART_ITConfig(USART1, USART_IT_WU, ENABLE);

  /* Enable PWR APB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Enter USART in STOP mode with regulator in low power mode */
  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

  /* Waiting Wake Up interrupt */
  while(InterruptCounter == 0x00)
  {}

  /* Disable USART peripheral in STOP mode */
  USART_STOPModeCmd(USART1, DISABLE);

  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
  {}
  DataReceived = USART_ReceiveData(USART1);

  /* Clear the TE bit (if a transmission is on going or a data is in the TDR, it will be sent before
  efectivelly disabling the transmission) */
  USART_DirectionModeCmd(USART1, USART_Mode_Tx, DISABLE);

  /* Check the Transfer Complete Flag */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  /* USART Disable */
  USART_Cmd(USART1, DISABLE);
}
#endif

/*
 * Init all related hardware in here
 * rt_hw_serial_init() will register all supported USART device
 */
void rt_hw_usart_init()
{
	USART_InitTypeDef USART_InitStructure;

	RCC_Configuration();

	GPIO_Configuration();

	NVIC_Configuration();

	DMA_Configuration();

	/* uart init */
#ifdef RT_USING_UART1
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(COM_USART[0], &USART_InitStructure);

	/* Enable USART1 */
	USART_Cmd(COM_USART[0], ENABLE);
	//USART_ClearFlag(COM_USART[0],USART_FLAG_TXE);
	/* register uart1 */
	rt_hw_serial_register(&uart1_device, "uart1",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
		&uart1);

	/* enable interrupt */
	USART_ITConfig(COM_USART[0], USART_IT_RXNE, ENABLE);

  	/* Wake up from USART STOP mode by Start bit Method */
  	//WakeUp_StartBitMethod();

#endif

#ifdef RT_USING_UART2
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(COM_USART[1], &USART_InitStructure);
	/* Enable USART2 */
	//USART_Cmd(COM_USART[1], ENABLE);
	//USART_ClearFlag(COM_USART[1],USART_FLAG_TXE);

	/* register uart2 */
	rt_hw_serial_register(&uart2_device, "uart2",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
		&uart2);

	/* Enable USART2 Rx request */
	USART_ITConfig(COM_USART[1], USART_IT_RXNE, ENABLE);

  	/* Wake up from USART STOP mode by Start bit Method */
  	//WakeUp_StartBitMethod();
#endif

}
