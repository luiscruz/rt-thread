/*
 * File      : led.c
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
 */

#include <rtthread.h>
#include <stm32f0xx.h>
#include "board.h"

#ifndef	LED_PWM_SUPPORTED
#include "led.h"
#define	printk	rt_kprintf

rt_timer_t led_timer;
//static rt_mutex_t led_mux;

struct led_dev leds[] ={
	{LED0_FACE_GPIO_PORT,LED0_FACE_PIN},
	{LED_CROWN_GPIO_PORT,LED_CROWN_PIN},
	{LED_PHONE_GPIO_PORT,LED_PHONE_PIN},
	{LED_MAIL_GPIO_PORT,LED_MAIL_PIN},
	{LED_SOCIAL_GPIO_PORT,LED_SOCIAL_PIN}
	};

/* set up led behavior */
void rt_hw_led(rt_uint8_t led, rt_uint16_t ticks, rt_uint8_t on, rt_uint32_t cnt)
{
	if(led >= LED_TOTAL){
		printk("led number %d is not supported\n", led);
		return;
	}
	/* mutex */
	//rt_mutex_take(led_mux, RT_WAITING_FOREVER);

	leds[led].mode.on_ratio = on;
	leds[led].mode.ticks = ticks>=(LED_TIMER_PERIOD<<1)?ticks:(LED_TIMER_PERIOD<<1);
	leds[led].mode.cnt = cnt;
	leds[led].on_ticks = ((rt_uint32_t)(leds[led].mode.ticks) * leds[led].mode.on_ratio) / 100;
	leds[led].now=0;
	//rt_mutex_release(led_mux);
}

void led_on(struct led_dev *pled)
{
	if(!pled->set){
		GPIO_ResetBits(pled->GPIOx, pled->GPIO_Pin);
		pled->set = 1;
	}
}

void led_off(struct led_dev *pled)
{
	if(pled->set)	{
		GPIO_SetBits(pled->GPIOx, pled->GPIO_Pin);
		pled->set = 0;
	}
}

/* called back every 50ms to update the led on duty */
static void led_timer_func(void* parameter)
{
	int i;
	struct led_dev *pled;

	for(i = 0 ; i< LED_TOTAL ; i++){/* all leds */
		/* mutex lock */
		pled = leds + i;
		if(pled->mode.on_ratio == 0){
			//led_off(pled);
			continue;
		}
		/* check which led to start*/
		if( !pled->mode.cnt ) {
			//led_off(pled);
			continue;
		}

		//rt_mutex_take(led_mux, RT_WAITING_FOREVER);

		if(pled->mode.on_ratio > 100) {
			/* breathing effect */
		}else{
			if(pled->now < pled->on_ticks)
				led_on(pled);
			else
				led_off(pled);

			pled->now += LED_TIMER_PERIOD;
		}
		/* mutex unlock*/
		if(pled->now >= pled->mode.ticks) {
				pled->mode.cnt --;
				//if(!pled->mode.cnt) led_off(pled);
				pled->now =0;
				//led_off(pled);
		}
		//rt_mutex_release(led_mux);
	}
}

void rt_hw_led_init(void)
{
	int i;
    GPIO_InitTypeDef GPIO_InitStructure;

  	/* GPIOA Periph clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  	/* Configure  in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = LED_PHONE_PIN | LED_MAIL_PIN |
								LED_SOCIAL_PIN ;

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOB Periph clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

 	GPIO_InitStructure.GPIO_Pin = LED0_FACE_PIN | LED_CROWN_PIN ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

  	for(i = LED_FACE ; i <= LED_SOCIAL ; i++){
		GPIO_SetBits(leds[i].GPIOx, leds[i].GPIO_Pin); /* turn off LEDs */
		leds[i].set=0;
	}
}

void led_init(void)
{
    /*rt_err_t result;

    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            &led_stack[0],
                            sizeof(led_stack),
                            4,
                            2);
    if(result == RT_EOK) rt_thread_startup(&led_thread);*/
	//led_mux = rt_mutex_create("led", RT_IPC_FLAG_FIFO);

    rt_hw_led_init();

	/* create 1/20 second timer, period is 5 ticks, 50ms  */
    led_timer = rt_timer_create("led",
    							led_timer_func,
    							leds,
                                LED_TIMER_PERIOD,
                                RT_TIMER_FLAG_PERIODIC);
    /* start timer */
    rt_timer_start(led_timer);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void setled(rt_uint8_t led, rt_uint16_t ticks, rt_uint8_t on, rt_uint32_t cnt)
{
    /* init led configuration if it's not inited. */

	rt_hw_led(led, ticks, on, cnt);
}
FINSH_FUNCTION_EXPORT(setled, "setled(led[0-7], ticks[10-], on[0-101], cnt[0-]")
#endif
#endif
