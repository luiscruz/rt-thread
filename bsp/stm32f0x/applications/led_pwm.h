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
#include "bt_parser.h"

/*
 * PA1	"face_led LED0"		TIM2_CH2
 * PA6	"phone_led LED2"	TIM16_CH1
 * PA7	"mail_led LED3"		TIM17_CH1
 * PB0	"social_led LED4"	TIM3_CH3
 * PB1	"crown_led LED1"	TIM14_CH1
 */

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

/* optional, TIM14_CH1 */
#define LED_CROWN_PIN           GPIO_Pin_1
#define LED_CROWN_GPIO_PORT     GPIOB
#define LED_CROWN_GPIO_CLK      RCC_AHBPeriph_GPIOB
#define	LED_CROWN_OFF			GPIO_SetBits(LED_CROWN_GPIO_PORT, LED_CROWN_PIN)
#define	LED_CROWN_ON			GPIO_ResetBits(LED_CROWN_GPIO_PORT, LED_CROWN_PIN)

/* TIM16_CH1 */
#define LED_PHONE_PIN           GPIO_Pin_6
#define LED_PHONE_GPIO_PORT     GPIOA
#define LED_PHONE_GPIO_CLK      RCC_AHBPeriph_GPIOA
#define	LED_PHONE_OFF			GPIO_SetBits(LED_PHONE_GPIO_PORT, LED_PHONE_PIN)
#define	LED_PHONE_ON			GPIO_ResetBits(LED_PHONE_GPIO_PORT, LED_PHONE_PIN)

/* TIM17_CH1 */
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

/* making clock counter 48M/48K = 1khz as a working clock */
#define	TIM2_PreScale	((rt_uint16_t)48000)
#define	TIM3_PreScale	((rt_uint16_t)48000)
#define	TIM14_PreScale	((rt_uint16_t)48000)
#define	TIM16_PreScale	((rt_uint16_t)48000)
#define	TIM17_PreScale	((rt_uint16_t)48000)

/* 1khz as a PWM working clock */
#define	TIM2_WClock			((rt_uint16_t)1000)
#define	TIM3_WClock			((rt_uint16_t)1000)
#define	TIM14_WClock		((rt_uint16_t)1000)
#define	TIM16_WClock		((rt_uint16_t)1000)
#define	TIM17_WClock		((rt_uint16_t)1000)

struct _led_pwm{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	TIM_TypeDef* tim;
	rt_uint16_t	preScale;	/* making clock counter 48M/48K = 1khz as a working clock */
	rt_uint16_t	wclock;		/* PWM working clock */
	rt_int8_t ch;			/* channel# to control LED */
	rt_uint16_t period_clk;	/* clock count in a PWM period */
	rt_int8_t period;		/* hz(1-127) : 1-128, period(-2 ~ -128 ) : 2-128s */
	rt_uint16_t on_clk;	/* on duty clock counts of pwm in a period */
};

struct _led_dev{
	struct _led_pwm pwm;
	rt_int8_t tok;		/* current active token */
	union _led_cmdq cmdq;
};

enum _led_idx {LED_FACE=0, LED_PHONE, LED_MAIL, LED_SOCIAL, LED_CROWN, LED_TOTAL};

void led_init(void);
void set_hw_led(rt_uint8_t led);
void isr_update_pwm(rt_uint8_t n);

#endif
