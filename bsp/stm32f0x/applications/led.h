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

#define LED0_FACE_PIN           GPIO_Pin_0
#define LED0_FACE_GPIO_PORT     GPIOB
#define LED0_FACE_GPIO_CLK      RCC_AHBPeriph_GPIOB
#define	LED0_FACE_OFF			GPIO_SetBits(LED0_FACE_GPIO_PORT, LED0_FACE_PIN)
#define	LED0_FACE_ON			GPIO_ResetBits(LED0_FACE_GPIO_PORT, LED0_FACE_PIN)

#define LED_CROWN_PIN           GPIO_Pin_1
#define LED_CROWN_GPIO_PORT     GPIOB
#define LED_CROWN_GPIO_CLK      RCC_AHBPeriph_GPIOB
#define	LED_CROWN_OFF			GPIO_SetBits(LED_CROWN_GPIO_PORT, LED_CROWN_PIN)
#define	LED_CROWN_ON			GPIO_ResetBits(LED_CROWN_GPIO_PORT, LED_CROWN_PIN)

#define LED_PHONE_PIN           GPIO_Pin_1
#define LED_PHONE_GPIO_PORT     GPIOA
#define LED_PHONE_GPIO_CLK      RCC_AHBPeriph_GPIOA
#define	LED_PHONE_OFF			GPIO_SetBits(LED_PHONE_GPIO_PORT, LED_PHONE_PIN)
#define	LED_PHONE_ON			GPIO_ResetBits(LED_PHONE_GPIO_PORT, LED_PHONE_PIN)

#define LED_MAIL_PIN            GPIO_Pin_6
#define LED_MAIL_GPIO_PORT      GPIOA
#define LED_MAIL_GPIO_CLK       RCC_AHBPeriph_GPIOA

#define LED_SOCIAL_PIN          GPIO_Pin_7
#define LED_SOCIAL_GPIO_PORT	GPIOA
#define LED_SOCIAL_GPIO_CLK     RCC_AHBPeriph_GPIOA

#define	LED_MAIL_OFF			GPIO_SetBits(LED_MAIL_GPIO_PORT, LED_MAIL_PIN)
#define	LED_MAIL_ON				GPIO_ResetBits(LED_MAIL_GPIO_PORT, LED_MAIL_PIN)

#define	LED_SOCIAL_OFF			GPIO_SetBits(LED_SOCIAL_GPIO_PORT, LED_SOCIAL_PIN)
#define	LED_SOCIAL_ON			GPIO_ResetBits(LED_SOCIAL_GPIO_PORT, LED_SOCIAL_PIN)

#pragma pack(1)
struct led_mode{
	rt_uint16_t ticks; 	/*  period of the on-off in ticks, 1tick=10ms */
	rt_uint8_t on_ratio;
	rt_uint32_t cnt;	/* repeat times, -1 : infinite */
};
#pragma pack()

struct led_dev{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	rt_uint8_t set;
	rt_uint16_t now;
	rt_uint16_t on_ticks;
	struct led_mode mode;
};

enum _led_idx {LED_FACE=0,LED_CROWN,LED_PHONE,LED_MAIL,LED_SOCIAL,LED_TOTAL};

void led_init(void);
void rt_hw_led(rt_uint8_t led, rt_uint16_t ticks, rt_uint8_t on, rt_uint32_t cnt);

#endif