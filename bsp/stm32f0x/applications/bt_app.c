/*
 * File      : btapp_dev.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 03-02-2013	  Thomas Tsai	cloned from finsh to use usart
 */

#include <rtthread.h>
#include <rthw.h>
#include <stm32f0xx.h>
#include "board.h"
#include "bt_app.h"
#include "protocol_bt.h"

#define	printk	rt_kprintf

struct btapp_dev bt_dev;
BT_STATUS bt_stat = BT_SHUTDOWN;

/* btapp thread */
static struct rt_thread btapp_thread;
ALIGN(RT_ALIGN_SIZE)
static char btapp_thread_stack[BTAPP_THREAD_STACK_SIZE];

BT_STATUS bt_status_update(void)
{
	BT_STATUS old = bt_stat;
	printk("old stat=0x%x\n", old);
	bt_stat = (BT_STATUS) (BT_IND2_PIN_LEVEL <<1 | BT_IND1_PIN_LEVEL);
	printk("new stat=0x%x\n", bt_stat);
	if( (old == BT_LINK) && (bt_stat ==BT_POWER_ON)){
		bt_stat = BT_SHUTDOWN;
		printk("actually it's 0x%x\n", bt_stat);
	}
	return bt_stat;
}

static void bt_reset(void)
{
	BT_RESET_EN;
	rt_thread_delay(1);
	BT_RESET_DIS;
	rt_thread_delay(49);
}

static void bt_standby(void)
{
	BT_PAIRING_EN;
	rt_thread_delay(25);
	BT_PAIRING_DIS;
}

/* Resume BM57 from Shutdown State by WAKEUP low active control.*/
static void bt_resume(void)
{
	if(bt_stat != BT_SHUTDOWN) return;
	BT_WAKEUP_EN;
	rt_thread_delay(10);
	BT_WAKEUP_DIS;
}

static rt_err_t btapp_rx_ind(rt_device_t dev, rt_size_t size)
{
	/* release semaphore to let btapp thread rx data */
	rt_sem_release(&bt_dev.rx_sem);

	return RT_EOK;
}

/**
 * @ingroup btapp
 *
 * This function sets the input device of bt_dev.
 *
 * @param device_name the name of new input device. "usart1", "usart2"...
 */
void btapp_set_device(const char* device_name)
{
	rt_device_t dev = RT_NULL;

	dev = rt_device_find(device_name);
	if (dev != RT_NULL && rt_device_open( dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
	{
		bt_dev.device = dev;
		rt_device_set_rx_indicate(dev, btapp_rx_ind);
	}
	else
	{
		printk("btapp: can not find device:%s\n", device_name);
	}
}

/* running the command from bt channel */
void btapp_run_line(const char *line)
{

}

/* scan a byte from bt or a finsh command line */
static rt_uint8_t sh_buf[80];
rt_size_t scan_buffer(rt_device_t dev,
                         void     *buffer,
                         rt_size_t   size)
{
#ifdef RT_USING_FINSH
/*	if(check finsh commnd line first){
		return size bytes from finsh buffer;
		{
	else*/
#endif
	{
		return rt_device_read(dev, 0, buffer, size);
	}
}

parsing_led_command(rt_uint8_t *buf, rt_uint8_t len)
{
	rt_uint8_t i;
	for(i = 0; i < len; i++){
		printk("%x ", buf[i]);
	}
	printk("  ");
	for(i = 0; i < len; i++){
		printk("%c ", buf[i]);
	}
	printk("\n");
}

void btapp_thread_entry(void* parameter)
{
    char ch;

	bt_reset(); /* BM57 is first init */

	while (1){
		/* wait receive */
		if (rt_sem_take(&bt_dev.rx_sem, RT_WAITING_FOREVER) != RT_EOK) continue;

		/* read header from device */
		if( (scan_buffer(bt_dev.device, &ch, 1) == 1) &&
			(btp_h_valid(ch))) { /* valid commands */
			BTP_HEADER btp_h;
			rt_uint8_t *ptr = (rt_uint8_t *)&btp_h;
			*ptr++ = ch;
			if(scan_buffer(bt_dev.device, ptr, BTP_HEAD_SIZE -1 )
				== (BTP_HEAD_SIZE-1 ) ) {/* rest of the header*/
				if(checkbtp_header(&btp_h))	{
					/* parsing header and read the content*/
					rt_uint8_t buf[MAX_BTP_LEN];
					if(scan_buffer(bt_dev.device, buf, btp_h.len )
						!= btp_h.len ){
						printk("read content failure\n");
						continue;
					}else{
						rt_uint8_t i, cks;
						//checksum
						cks = 0;
						for(i = 0; i < BTP_HEAD_SIZE; i++) cks+= ptr[i];
						for(i = 0; i < btp_h.len; i ++) cks+= buf[i];
						if(cks == buf[i] ){
							//parsing it
							parsing_led_command(buf, btp_h.len );

							printk("OK\n");
						}else{
							continue;
						}
					}
				}else{
					printk("bad header\n");
				}
			}else{
				printk("read header failure\n");
			}
		}else{
			printk("read start code failure\n");
		}/* end of device read */
	}
}

/*
output pins
#define BT_RST_N_PIN    		GPIO_Pin_0
#define BT_RST_N_GPIO_PORT  	GPIOB
#define BT_RST_N_GPIO_CLK   	RCC_AHBPeriph_GPIOB

#define BT_WAKEUP_PIN       	GPIO_Pin_1
#define BT_WAKEUP_GPIO_PORT     GPIOB
#define BT_WAKEUP_GPIO_CLK      RCC_AHBPeriph_GPIOB

#define BT_PAIRING_PIN 			GPIO_Pin_8
#define BT_PAIRING_GPIO_PORT    GPIOA
#define BT_PAIRING_GPIO_CLK     RCC_AHBPeriph_GPIOA

BM57 P07
L: MCU/host Informs BM57 that UART data will be transmitted out after 1 ms
#define BT_RX_IND_PIN 			GPIO_Pin_15
#define BT_RX_IND_GPIO_PORT    	GPIOA
#define BT_RX_IND_GPIO_CLK     	RCC_AHBPeriph_GPIOA

input pins
#define BT_IND1_PIN       		GPIO_Pin_6
#define BT_IND1_GPIO_PORT     	GPIOB
#define BT_IND1_GPIO_CLK      	RCC_AHBPeriph_GPIOB

#define BT_IND2_PIN       		GPIO_Pin_7
#define BT_IND2_GPIO_PORT     	GPIOB
#define BT_IND2_GPIO_CLK      	RCC_AHBPeriph_GPIOB
*/
void bt_hw_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

  	/* GPIOA Periph clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  	/* Configure  in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = BT_PAIRING_PIN | BT_RX_IND_PIN;

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	/* GPIOA Periph clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  	/* Configure  in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = BT_RST_N_PIN | BT_WAKEUP_PIN|BT_PAIRING_PIN;

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	////////////////
  	GPIO_InitStructure.GPIO_Pin = BT_IND1_PIN | BT_IND2_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

  	/* Enable SYSCFG clock */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  	/* Connect EXTI6 Line to PB6 pin */
  	SYSCFG_EXTILineConfig(BT_IND1_EXTI_PORT, BT_IND1_EXTI_SOURCE);

  	/* Connect EXTI7 Line to PB7 pin */
  	SYSCFG_EXTILineConfig(BT_IND2_EXTI_PORT, BT_IND2_EXTI_SOURCE);

  	/* Configure EXTI6 line */
  	EXTI_InitStructure.EXTI_Line = BT_IND1_EXTI_LINE;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	/* Configure EXTI7 line */
  	EXTI_InitStructure.EXTI_Line = BT_IND2_EXTI_LINE;
  	EXTI_Init(&EXTI_InitStructure);

  	/* Enable and set EXTI4_15 Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = BT_IND_EXTI_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

/*
 * @ingroup btapp
 *
 * This function will initialize bt_dev
 */
void btapp_init(void)
{
	rt_err_t result;

	bt_hw_init();

	rt_sem_init(&bt_dev.rx_sem, "bt_rx", 0, 0);
	result = rt_thread_init(&btapp_thread,
		"btapp",
		btapp_thread_entry, RT_NULL,
		&btapp_thread_stack[0], sizeof(btapp_thread_stack),
		BTAPP_THREAD_PRIORITY, 10);

	if (result == RT_EOK)
		rt_thread_startup(&btapp_thread);

	btapp_set_device(BTSPP_DEVICE); /*release semaphore to make read thread go */
}

#ifdef RT_USING_FINSH
#include <finsh.h>
/* using command line to simulate the input buffer from bluetooth. */
void cmd_btapp(	rt_uint8_t *packet)
{
	int i=0;
	rt_memset(sh_buf,0, 80);
	while( (i < 80) && packet[i]){
		sh_buf[i] = packet[i];
		i ++;
	}
}

FINSH_FUNCTION_EXPORT(cmd_btapp, enter a command to bt parser);
#endif
