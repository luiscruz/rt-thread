/*
 * File      : led.h
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

#ifndef __LED_H__
#define __LED_H__

#include <rtthread.h>
#include "board.h"

#define	COLOR_RED		0x01
#define	COLOR_GREEN		0x02
#define	COLOR_BLUE		0x04

#define MASK_LED_PHONE		(0x01)
#define	MASK_LED_MAIL		(0x02)
#define	MASK_LED_SOCIAL 	(0x04)
#define	MASK_LED_MISC		(0x08)
#define	MASK_LED_FACE1		(0x10)
#define	MASK_LED_FACE2		(0x20)
#define	MASK_LED_FACE3		(0x40)
#define	MASK_LED_CROWN		(0x80)

/* working mode by: working freq , on duty %, off duty % = 100 - on */
#define	LED_OFF		0
#define	LED_BLINK	50
#define	LED_ON		(100)
#define	LED_BREATH	(101)

#define	LED_TIMER_PERIOD	(5)	/* 5 TICKS = 50ms */

#define LED1_FACE_PIN           GPIO_Pin_1
#define LED1_FACE_GPIO_PORT     GPIOA
#define LED1_FACE_GPIO_CLK      RCC_AHBPeriph_GPIOA

#ifdef STM32F051

#define LED2_FACE_PIN           GPIO_Pin_2
#define LED2_FACE_GPIO_PORT     GPIOC
#define LED2_FACE_GPIO_CLK      RCC_AHBPeriph_GPIOC

#define LED3_FACE_PIN           GPIO_Pin_3
#define LED3_FACE_GPIO_PORT     GPIOC
#define LED3_FACE_GPIO_CLK      RCC_AHBPeriph_GPIOC

#define LED_PHONE_PIN           GPIO_Pin_13
#define LED_PHONE_GPIO_PORT     GPIOC
#define LED_PHONE_GPIO_CLK      RCC_AHBPeriph_GPIOC

#else

#define LED2_FACE_PIN           GPIO_Pin_2
#define LED2_FACE_GPIO_PORT     GPIOA
#define LED2_FACE_GPIO_CLK      RCC_AHBPeriph_GPIOA

#define LED3_FACE_PIN           GPIO_Pin_3
#define LED3_FACE_GPIO_PORT     GPIOA
#define LED3_FACE_GPIO_CLK      RCC_AHBPeriph_GPIOA

#define LED_PHONE_PIN           GPIO_Pin_5
#define LED_PHONE_GPIO_PORT     GPIOA
#define LED_PHONE_GPIO_CLK      RCC_AHBPeriph_GPIOA

#endif

#define LED_CROWN_PIN           GPIO_Pin_4
#define LED_CROWN_GPIO_PORT     GPIOA
#define LED_CROWN_GPIO_CLK      RCC_AHBPeriph_GPIOA

#define LED_MAIL_PIN            GPIO_Pin_6
#define LED_MAIL_GPIO_PORT      GPIOA
#define LED_MAIL_GPIO_CLK       RCC_AHBPeriph_GPIOA

#define LED_SOCIAL_PIN          GPIO_Pin_7
#define LED_SOCIAL_GPIO_PORT	GPIOA
#define LED_SOCIAL_GPIO_CLK     RCC_AHBPeriph_GPIOA

#define LED_MISC_PIN          	GPIO_Pin_5
#define LED_MISC_GPIO_PORT		GPIOB
#define LED_MISC_GPIO_CLK     	RCC_AHBPeriph_GPIOB

#define	MASK_LED_MISC		(0x08)
#define	MASK_LED_FACE1		(0x10)
#define	MASK_LED_FACE2		(0x20)
#define	MASK_LED_FACE3		(0x40)
#define	MASK_LED_CROWN		(0x80)

#pragma pack(1)
struct led_mode{
	rt_uint16_t ticks; 	/*  period of the on-off in ticks, 1tick=10ms */
	rt_uint8_t on_ratio;
	rt_uint32_t cnt;	/* repeat times, -1 : infinite */
};
#pragma pack()

struct led_dev{
	rt_uint8_t led;
	rt_uint8_t color;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	rt_uint8_t set;
	struct led_mode mode;
	rt_uint16_t now;
	rt_uint16_t on_ticks;
};

#define LED_PHONE		(0x00)
#define	LED_MAIL		(0x01)
#define	LED_SOCIAL 		(0x02)
#define	LED_MISC		(0x03)
#define	LED_FACE1		(0x04)
#define	LED_FACE2		(0x05)
#define	LED_FACE3		(0x06)
#define	LED_CROWN		(0x07)
#define	LED_MAX			LED_CROWN

void led_init(void);
void rt_hw_led(rt_uint8_t led, rt_uint16_t ticks, rt_uint8_t on, rt_uint32_t cnt);

#endif
