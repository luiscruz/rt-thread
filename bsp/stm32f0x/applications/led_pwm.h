/*
 * File      : led_pwm.h
 * COPYRIGHT (C) 2013, biotrump international technology
 * www.biotrump.cc
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-03-25     thomas tsai      the first version
 */

#ifndef __LED_PWM_H__
#define __LED_PWM_H__

#include <rtthread.h>
#include "board.h"

#define	COLOR_RED		0x01
#define	COLOR_GREEN		0x02
#define	COLOR_BLUE		0x04

/* working mode by: working freq , on duty %, off duty % = 100 - on */
#define	LED_OFF		0
#define	LED_BLINK	50
#define	LED_ON		(100)
#define	LED_BREATH	(101)

#define	TIM_ACTIVE				0x80
#define	TIM_CH1_ACTIVE			0x01
#define	TIM_CH2_ACTIVE			0x02
#define	TIM_CH3_ACTIVE			0x04
#define	TIM_CH4_ACTIVE			0x08

/* TIM2_CH2 */
#define LED_FACE_PIN           	GPIO_Pin_1
#define LED_FACE_GPIO_PORT     	GPIOA
#define LED_FACE_GPIO_CLK      	RCC_AHBPeriph_GPIOA
#define	LED_FACE_OFF			GPIO_SetBits(LED_FACE_GPIO_PORT, LED_FACE_PIN)
#define	LED_FACE_ON				GPIO_ResetBits(LED_FACE_GPIO_PORT, LED_FACE_PIN)

/* optional, TIM3_CH4 */
#define LED_CROWN_PIN           GPIO_Pin_1
#define LED_CROWN_GPIO_PORT     GPIOB
#define LED_CROWN_GPIO_CLK      RCC_AHBPeriph_GPIOB
#define	LED_CROWN_OFF			GPIO_SetBits(LED_CROWN_GPIO_PORT, LED_CROWN_PIN)
#define	LED_CROWN_ON			GPIO_ResetBits(LED_CROWN_GPIO_PORT, LED_CROWN_PIN)

/* TIM3_CH1 */
#define LED_PHONE_PIN           GPIO_Pin_6
#define LED_PHONE_GPIO_PORT     GPIOA
#define LED_PHONE_GPIO_CLK      RCC_AHBPeriph_GPIOA
#define	LED_PHONE_OFF			GPIO_SetBits(LED_PHONE_GPIO_PORT, LED_PHONE_PIN)
#define	LED_PHONE_ON			GPIO_ResetBits(LED_PHONE_GPIO_PORT, LED_PHONE_PIN)

/* TIM3_CH2 */
#define LED_MAIL_PIN            GPIO_Pin_7
#define LED_MAIL_GPIO_PORT      GPIOA
#define LED_MAIL_GPIO_CLK       RCC_AHBPeriph_GPIOA

/* TIM3_CH3 */
#define LED_SOCIAL_PIN          GPIO_Pin_0
#define LED_SOCIAL_GPIO_PORT	GPIOB
#define LED_SOCIAL_GPIO_CLK     RCC_AHBPeriph_GPIOB

#define	LED_MAIL_OFF			GPIO_SetBits(LED_MAIL_GPIO_PORT, LED_MAIL_PIN)
#define	LED_MAIL_ON				GPIO_ResetBits(LED_MAIL_GPIO_PORT, LED_MAIL_PIN)

#define	LED_SOCIAL_OFF			GPIO_SetBits(LED_SOCIAL_GPIO_PORT, LED_SOCIAL_PIN)
#define	LED_SOCIAL_ON			GPIO_ResetBits(LED_SOCIAL_GPIO_PORT, LED_SOCIAL_PIN)

#pragma pack(1)
struct led_mode{
	rt_int8_t period; 	/*  period< 0: |period| in s, period >=0 : period hz */
	rt_uint8_t duty;	/* pwm on ratio */
	rt_uint32_t cnt;	/* repeat times, -1 : infinite */
};
#pragma pack()

struct led_dev{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	rt_uint8_t timer;
	rt_uint8_t channel;
	rt_uint16_t on_ticks;
	struct led_mode mode;
};

enum _led_idx {LED_FACE=0,/*LED_CROWN,*/LED_PHONE,LED_MAIL,LED_SOCIAL,LED_TOTAL};

void led_init(void);
void set_hw_led(rt_uint8_t led, rt_int8_t period, rt_uint8_t on, rt_uint32_t cnt);

#endif
