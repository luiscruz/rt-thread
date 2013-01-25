/*
 * File      : key.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2010, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-01		Thomas Tsai
 * 2010-10-01     Yi.Qiu      first version
 */

#include <rtthread.h>
#include <rthw.h>
#include "stm32f2xx.h"
#include "board.h"
#include "key.h"

#define printf               rt_kprintf

#define KEY_RX_BUFFER_SIZE		32

struct rt_key_device
{
	struct rt_device parent;

	rt_uint32_t  rx_buffer[KEY_RX_BUFFER_SIZE];
	rt_uint32_t read_index, save_index;
};
static struct rt_key_device *key_device = RT_NULL;

static rt_err_t rt_keypad_init(rt_device_t dev)
{
	#if 0
	if (!(dev->flag & RT_DEVICE_FLAG_ACTIVATED))
	{

		if (dev->flag & RT_DEVICE_FLAG_INT_RX)
		{
			rt_memset(key_device->rx_buffer, 0,
				sizeof(key_device->rx_buffer));
			key_device->read_index = key_device->save_index = 0;
		}

		dev->flag |= RT_DEVICE_FLAG_ACTIVATED;
	}
#endif
	return RT_EOK;
}

static rt_err_t rt_key_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_key_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t rt_key_read (rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
#if 0
	rt_uint8_t* ptr;
	rt_err_t err_code;
	rt_base_t level;

	ptr = buffer;
	err_code = RT_EOK;

	/* interrupt mode Rx */
	while (size)
	{
		if (key_device->read_index != key_device->save_index)
		{
			*ptr++ = key_device->rx_buffer[key_device->read_index];
			size --;

			/* disable interrupt */
			level = rt_hw_interrupt_disable();

			key_device->read_index ++;
			if (key_device->read_index >= KEY_RX_BUFFER_SIZE)
				key_device->read_index = 0;

			/* enable interrupt */
			rt_hw_interrupt_enable(level);
		}
		else
		{
			/* set error code */
			err_code = -RT_EEMPTY;
			break;
		}
	}

	/* set error code */
	rt_set_errno(err_code);
	return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
#endif
	return 0;
}

static rt_err_t rt_key_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	return RT_EOK;
}

#ifdef RT_USING_RTGUI
#include <rtgui/event.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/kbddef.h>

/*
	C0		C1		C2
R0	ENTER	UP		MENU
R1	LEFT	DOWN	RIGHT
R2	HOME
*/
static int s_dir_key[]={RTGUIK_POWER};

static int s_key_map[3][3]={{RTGUIK_KP_ENTER, RTGUIK_UP, RTGUIK_MENU},
		{RTGUIK_LEFT, RTGUIK_DOWN, RTGUIK_RIGHT},
		{RTGUIK_HOME}};


static rt_err_t rtgui_key_rx(rt_device_t dev, rt_size_t size)
{
#if 0
	struct rtgui_event_kbd kbd_event;
	char key_value;

	while(rt_device_read(dev, 0, &key_value, 1) == 1)
	{
		/* init keyboard event */
		RTGUI_EVENT_KBD_INIT(&kbd_event);
		kbd_event.mod  = RTGUI_KMOD_NONE;
		kbd_event.unicode = 0;
		kbd_event.key = RTGUIK_UNKNOWN;

		if(key_value &  0x80)
		{
			kbd_event.type = RTGUI_KEYUP;
		}
		else
		{
			kbd_event.type = RTGUI_KEYDOWN;
		}

		kbd_event.key = s_key_map[key_value & 0x7F];
	}
	if (kbd_event.key != RTGUIK_UNKNOWN)
	{
		/* post down event */
		rtgui_server_post_event(&(kbd_event.parent), sizeof(kbd_event));
	}
#endif
	return RT_EOK;
}

#endif

/* Since the limitation of EXTI, the pin number should not be the same in the different GPIO port
 * directed keys or matrix scan key???
 * pwr key				PA0
 * PC5,PC6,PC7			: col, input , edge trigger
 * PG11, PG13, PG7		: row, output low
 *
 * enter/select
 * menu/exit/back
 * up/pre
 * down/next
 * left/pause
 * right/pause
 */

void EXTI0_Enable(rt_uint32_t enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

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
    EXTI_ClearITPendingBit(EXTI_Line0);
}

static void EXTI_Keyscan_Enable(rt_uint32_t enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

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
    EXTI_ClearITPendingBit(EXTI_Line5);
    EXTI_ClearITPendingBit(EXTI_Line6);
    EXTI_ClearITPendingBit(EXTI_Line7);
}

rt_sem_t power_key_sem = RT_NULL;
static void power_key_bh(void *param)
{
	struct rtgui_event_kbd kbd_event;

	power_key_sem = rt_sem_create("power_key sem", 0, RT_IPC_FLAG_FIFO); /* bh waits for irq */
	while(1)
	{
		printf("power_key_bh is waiting for IRQ\n");
		rt_sem_take(power_key_sem, RT_WAITING_FOREVER);/* waits for ISR */
		/* init keyboard event */
		RTGUI_EVENT_KBD_INIT(&kbd_event);
		kbd_event.mod  = RTGUI_KMOD_NONE;
		kbd_event.unicode = 0;
		kbd_event.key = RTGUIK_UNKNOWN;
		printf("power key is pressed %d\n",POWER_KEY_DOWN);
		/* power key IRQ is disabled */
		if(POWER_KEY_DOWN){
			rt_thread_delay(5);	/* 50ms to debounce */
			if(POWER_KEY_DOWN){	/* power button is down now */
				kbd_event.type = RTGUI_KEYDOWN;
				kbd_event.key = s_dir_key[0];
				/* post down event */
				rtgui_server_post_event(&(kbd_event.parent), sizeof(kbd_event));
				printf("post power key down\n");
				while(POWER_KEY_DOWN) rt_thread_delay(1);	/* waiting for key up */
				kbd_event.type = RTGUI_KEYUP;
				kbd_event.key = s_dir_key[0];
				/* post up event */
				rtgui_server_post_event(&(kbd_event.parent), sizeof(kbd_event));
				printf("post power key up\n");
			}
		}
		//enable power key falling edge trigger again
		EXTI0_Enable(1);
	}
}

rt_sem_t keypad_sem = RT_NULL;
static void keypad_scan_bh(void *param)
{
	//debouce
	//check again  if the col is down
	//	scan row0,1,2 by pull up
	//	col is up, post scanned key press-down
	//	wait until col is up
	//	post key up
	//else  this is key up, bouncing...
	// enable interrupt again
	//struct rtgui_event_kbd kbd_event;
	//char key_value;

	keypad_sem = rt_sem_create("keypad_sem", 0, RT_IPC_FLAG_FIFO); /* bh waits for irq */
	while(1)
	{
		printf("keypad_scan_bh is waiting for isr\n");
		rt_sem_take(keypad_sem, RT_WAITING_FOREVER);/* waits for ISR */
		/* init keyboard event */
		/* power key IRQ is disabled */
		printf("keypad_scan_bh fired\n");
		//enable power key falling edge trigger again
		//EXTI0_Enable(1);
		EXTI_Keyscan_Enable(1);
	}
}

/**
  * @brief  Configures gpio and EXTI Lines in interrupt mode
  * @param  None
  * @retval None
  */
static void  keypad_gpio_init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* keypad matrix row: GPO pull-down, PG11, PG13, PG7 */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_7 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/* Enable GPIOA clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable SYSCFG clock
	SYSCFG APB clock must be enabled to get write access to SYSCFG_EXTICRx
	*         registers using
	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PA0 pin as input floating: PWR */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /* external 10k pull up*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* keypad matrix col : Enable GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Configure PC5, PC6, PC7 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* external 10k pull up*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect EXTI Line5,6,7 to PC5,6,7 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource7);

	/* Configure EXTI Line5,6,7 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line9_5 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * key driver register
 */
void rt_hw_keypad_init(void)
{
	rt_thread_t tid;

	key_device = (struct rt_key_device*)rt_malloc (sizeof(struct rt_key_device));
	if (key_device == RT_NULL) return; /* no memory yet */

	/* hardware init */
	keypad_gpio_init();

	/* clear device structure */
	rt_memset(&(key_device->parent), 0, sizeof(struct rt_device));

	key_device->parent.type 		= RT_Device_Class_Char;
	key_device->parent.tx_complete = RT_NULL;
	key_device->parent.init 		= rt_keypad_init;
	key_device->parent.open		= rt_key_open;
	key_device->parent.close		= rt_key_close;
	key_device->parent.read 		= rt_key_read;
	key_device->parent.write 		= RT_NULL;
	key_device->parent.control 	= rt_key_control;
	key_device->parent.user_data   = RT_NULL;

#ifdef RT_USING_RTGUI
	key_device->parent.rx_indicate = rtgui_key_rx;
#endif

	tid = rt_thread_create("power_key_bh",
							power_key_bh, RT_NULL,
							1024, 20, 5);
	if (tid != RT_NULL)
		rt_thread_startup(tid);

	tid = rt_thread_create("keypad_bh",
							keypad_scan_bh, RT_NULL,
							1024, 20, 5);
	if (tid != RT_NULL)
		rt_thread_startup(tid);

	/* register key device to RT-Thread */
	rt_device_register(&(key_device->parent), "keypad", RT_DEVICE_FLAG_RDWR);
}
