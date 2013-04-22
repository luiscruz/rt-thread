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
#include <rtdevice.h>
#include <stm32f0xx.h>
#include "board.h"
#include "bt_app.h"
//#include "protocol_bt.h"
#include "bt_parser.h"

#define	printk	rt_kprintf
#define BT_RX_BUFSIZE          (64)

struct btapp_dev bt_dev;
BT_STATUS bt_stat = BT_SHUTDOWN;

/* btapp thread */
static struct rt_thread btapp_qt, btapp_pt;
ALIGN(RT_ALIGN_SIZE)
static char btapp_qt_stack[BTAPP_THREAD_QUEUE_STACK_SIZE];
static char btapp_pt_stack[BTAPP_THREAD_PARSER_STACK_SIZE];

/* ring buffer */
static struct rt_semaphore free_space, stored_data;
static rt_uint8_t rx_pool[BT_RX_BUFSIZE];
static struct rt_ringbuffer rx_ringbuffer;

static void dump_stat(BT_STATUS st)
{
	switch(st){
	case BT_LINK:
		printk("BT_LINK\n");
		break;
	case BT_LINK_TX:
		printk("BT_LINK_TX\n");
		break;
	case BT_ACCESS:
		printk("BT_ACCESS\n");
		break;
	case BT_POWER_ON:
		printk("BT_POWER_ON\n");
		break;
	case BT_SHUTDOWN:
		printk("BT_SHUTDOWN\n");
		break;
	default:
		break;
	};

}

BT_STATUS bt_status_update(void)
{
	BT_STATUS old = bt_stat;
	printk("old stat:=0x%x ", old);
	dump_stat(old);
	bt_stat = (BT_STATUS) (BT_IND2_PIN_LEVEL <<1 | BT_IND1_PIN_LEVEL);
	printk("new stat=0x%x ", bt_stat);
	dump_stat(bt_stat);
	if( (old == BT_LINK) && (bt_stat ==BT_POWER_ON)){
		bt_stat = BT_SHUTDOWN;
		printk("actually it's 0x%x\n", bt_stat);
		dump_stat(bt_stat);
	}
	return bt_stat;
}

static void bt_reset(void)
{
	BT_RESET_EN;
	//rt_thread_delay(RT_TIMER_TICK_PER_SECOND);
	board_delay_ms(2);
	BT_RESET_DIS;
	rt_thread_delay(RT_TIMER_TICK_PER_SECOND>>1);
}

static void bt_standby(void)
{
	BT_PAIRING_EN;
	rt_thread_delay(RT_TIMER_TICK_PER_SECOND>>2);
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

rt_int8_t bm57_rx_prep(rt_int8_t on)
{
	if(bt_stat > BT_LINK_TX) {
		printk("bm57_rx_prep err %d\n", bt_stat);
		return -1;
	}
	if(on){
		BT_RX_IND_EN;
		//delay 2ms to wait for bm57 ready to receive from mcu
		board_delay_ms(1);
	}else{BT_RX_IND_DIS;}
	return 0;
}

rt_int8_t bm57_tx_prep(void)
{
	rt_int16_t cnt=0;
	rt_int8_t ret=-1;

	while( (cnt++<1000) && (bt_stat != BT_LINK_TX));
	if(bt_stat == BT_LINK_TX) ret = 0;
	else
		printk("bm57_tx_prep failure %d\n", bt_stat);

	return ret;
}

static rt_size_t rx_size;
static rt_err_t btapp_rx_ind(rt_device_t dev, rt_size_t size)
{
	/* release semaphore to let btapp thread rx data */
	rx_size = size;
	rt_sem_release(&bt_dev.rx_sem);
	//printk("\nrx_ind: %d\n", size);
	return RT_EOK;
}

/* ring buffer */
static void bt_producer_t(struct btapp_dev *dev)
{
	rt_uint8_t ch;
	rt_size_t s=0, r;

	while(1){
		/* wait receive from USART */
		if (rt_sem_take(&(dev->rx_sem), RT_WAITING_FOREVER) != RT_EOK) continue;
		s = rx_size; /* received bytes in RX buffer */
		/* ring buffer */
		while(s --){
			if (rt_sem_take(&free_space, RT_WAITING_FOREVER) != RT_EOK) continue;
			r = rt_device_read(dev->device, 0, &ch, 1);
			//if(r != 1) break;
			rt_ringbuffer_putchar(&rx_ringbuffer, ch);
			rt_sem_release(&stored_data);
		}
	}
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

rt_uint8_t read_bt(rt_uint8_t size, rt_uint8_t *buf)
{
	int i=0;
	rt_uint8_t ch;
	/* bluetooth buffer is a circular queue */
	while(size--){
		if (rt_sem_take(&stored_data, RT_WAITING_FOREVER) != RT_EOK) return 0;
    	rt_ringbuffer_getchar(&rx_ringbuffer, &ch);
    	buf[i++] = ch;
		rt_sem_release(&free_space);
	}
	return i;
}

/*
packet :
0xC1 0x27 0x0002 0x0001 01 0xer 0xck
*/


static int bt_resp(rt_uint8_t sig, rt_uint8_t rc)
{
	rt_uint8_t cks=0;
	rt_uint8_t *p;
	struct _resp_pkt resp;
	int i;

	printk("bt_resp %d\n", rc);
	rt_memset(&resp, 0, sizeof(resp));
	resp.sig = HEAD_SIG1 | (sig << 8);
	resp.len = sizeof (resp) - sizeof(resp.sig) - sizeof(resp.len);
	resp.rc = rc;
	p = (rt_uint8_t *)&resp;
	for(i = 0; i < sizeof(resp) - 1; i++)
		cks += p[i];
	resp.checksum = cks;
	for(i = 0; i < sizeof(resp); i++){
		printk("0x%x ",p[i]);
	}
	printk("\n");
	/*Response command: 0x55 0xA? len rr rr checksum */
	rt_device_write(bt_dev.device, 0, p, sizeof(resp));
	return 0;
}

rt_int8_t bt_findme(rt_uint8_t findme)
{
	return bt_resp(HEAD_FINDME, findme);
}

rt_int8_t bt_led_resp(rt_uint8_t err)
{
	return bt_resp(HEAD_RESP, err);
}

/* processing commands from BT module */
void bt_consumer_t(void* parameter)
{
	rt_int8_t ret;
	BT_PAIRING_DIS; /* disable standy mode */
	bt_reset(); 	/* BM57 is reset for the first init */

	while (1){
#ifdef	LED_PWM_SUPPORTED
		ret = btcmd_paser();
		bt_led_resp(ret);
#else
		process_led_command((struct btapp_dev *)parameter);
#endif
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
	GPIO_InitStructure.GPIO_Pin = BT_PAIRING_PIN | BT_RX_IND_PIN | BT_RST_N_PIN | BT_WAKEUP_PIN;

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	BT_RX_IND_DIS; /* pull high to disable RX_IND, because it makes bt module can't enter low power mode*/

  	/* GPIOB Periph clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  	/* Configure  in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = BT_PAIRING_PIN;

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*input pins*/
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

    /* initialize ring buffer */
    rt_sem_init(&free_space, "bt_free", BT_RX_BUFSIZE, 0);
    rt_sem_init(&stored_data, "bt_stored", 0, 0);
    rt_ringbuffer_init(&rx_ringbuffer, rx_pool, BT_RX_BUFSIZE);

	rt_sem_init(&bt_dev.rx_sem, "bt_rx", 0, 0);

	result = rt_thread_init(&btapp_qt,
		"bt_qt",
		bt_producer_t, (void *)&bt_dev,
		&btapp_qt_stack[0], sizeof(btapp_qt_stack),
		BTAPP_THREAD_PRIORITY, 10);
	if (result == RT_EOK)
		rt_thread_startup(&btapp_qt);

	result = rt_thread_init(&btapp_pt,
		"bt_pt",
		bt_consumer_t, (void *)&bt_dev,
		&btapp_pt_stack[0], sizeof(btapp_pt_stack),
		BTAPP_THREAD_PRIORITY, 10);
	if (result == RT_EOK)
		rt_thread_startup(&btapp_pt);

	btapp_set_device(BTSPP_DEVICE); /*release semaphore to make read thread go */
}

#ifdef RT_USING_FINSH
#include <finsh.h>
/* using command line to simulate the input buffer from bluetooth. */
rt_uint8_t sh_buf[80];
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
