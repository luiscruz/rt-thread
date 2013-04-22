/*
 * File      : led_pwm.c
 * COPYRIGHT (C) 2013, biotrump international technology
 * www.biotrump.cc
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * In order to have individual period of each LED,
 * each LED has its own timer, so the period setting won't affect each other.
 * Whenever the counter reach "0" in each period, a interrupt is generated,
 * so the repetition can be counted. When repetition reaches "0", the timer is disabled to save power.
 * PA1	"face_led LED0"		TIM2_CH2
 * PA6	"phone_led LED2"	TIM16_CH1
 * PA7	"mail_led LED3"		TIM17_CH1
 * PB0	"social_led LED4"	TIM3_CH3
 * PB1	"crown_led LED1"	TIM14_CH1

 <<<<<< Change Logs:
 * Date           	Author      	Notes
 * 2013-04-11		thomas tsai		each led has its own timer
 * 2013-03-25     	thomas tsai     the first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <stm32f0xx.h>
#include "board.h"

#ifdef	LED_PWM_SUPPORTED
#include "bt_parser.h"
#include "led_pwm.h"
#define	printk	rt_kprintf

struct rt_semaphore led_sem;

struct _led_dev leds[] = { {LED_FACE_GPIO_PORT, LED_FACE_PIN, TIM2, TIM2_PreScale, TIM2_WClock, 2},
						{LED_PHONE_GPIO_PORT, LED_PHONE_PIN, TIM16, TIM16_PreScale, TIM16_WClock, 1},
						{LED_MAIL_GPIO_PORT, LED_MAIL_PIN, TIM17, TIM17_PreScale, TIM17_WClock, 1},
						{LED_SOCIAL_GPIO_PORT, LED_SOCIAL_PIN, TIM3, TIM3_PreScale, TIM3_WClock, 3},
						{LED_CROWN_GPIO_PORT, LED_CROWN_PIN, TIM14, TIM14_PreScale, TIM14_WClock, 1} };

/* ---------------------------------------------------------------------------
  TIM3 Configuration: Output Compare Active Mode:
  In this example TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK1)
    TIM3CLK = PCLK1
    PCLK1 = HCLK
    => TIM3CLK = HCLK = SystemCoreClock

  To get TIM3 counter clock at 1 KHz, the prescaler is computed as follows:
   Prescaler = (TIM3CLK / TIM3 counter clock) - 1
   Prescaler = (SystemCoreClock /1 KHz) - 1

  CC1 update rate = TIM3 counter clock / CCR1_Val = 146.48 Hz
  ==> Toggling frequency = 73.24 Hz

  C2 update rate = TIM3 counter clock / CCR2_Val = 219.7 Hz
  ==> Toggling frequency = 109.8 Hz

  CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
  ==> Toggling frequency = 219.7 Hz

  CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
  ==> Toggling frequency = 439.4 Hz

  Note:
   SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
   Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
   function to update SystemCoreClock variable value. Otherwise, any configuration
   based on this variable will be incorrect.
----------------------------------------------------------------------- */

/*	LED_FACE_PIN	PA1		TIM2_CH2
 *	LED_PHONE_PIN	PA6		TIM16_CH1
 *	LED_MAIL_PIN	PA7		TIM17_CH2
 *	LED_SOCIAL_PIN	PB0		TIM3_CH3
 */
static void gpio_config(rt_uint8_t on_off)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	if(on_off){/* on to TIMx setting */
	  	/* GPIOA,B Periph clock enable */
	  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE);

	   	GPIO_InitStructure.GPIO_Pin = LED_FACE_PIN | LED_PHONE_PIN | LED_MAIL_PIN;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	  	GPIO_InitStructure.GPIO_Pin =  LED_SOCIAL_PIN | LED_CROWN_PIN;
	  	GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Connect TIM Channels to AF2, TIM2 CH2 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);

	  	/* Connect TIM Channels to AF5, TIM16_CH1 CH1 , TIM17_CH1 */
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);

		/* high power TIM3 CH3, TIM14_CH1*/
	  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);
	 	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_0);

		/* Enable the TIMx global Interrupt */
  		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  		NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  		NVIC_Init(&NVIC_InitStructure);
  		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_Init(&NVIC_InitStructure);
	 	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
		NVIC_Init(&NVIC_InitStructure);
 		NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
		NVIC_Init(&NVIC_InitStructure);
 		NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
		NVIC_Init(&NVIC_InitStructure);
	}else{
		int i;
  		/* GPIOA Periph clock enable */
  		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  		/* Configure  in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = LED_FACE_PIN | LED_PHONE_PIN | LED_MAIL_PIN ;

  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* GPIOB Periph clock enable */
  		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

 		GPIO_InitStructure.GPIO_Pin = LED_SOCIAL_PIN/* | LED_CROWN_PIN*/ ;
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  		GPIO_Init(GPIOB, &GPIO_InitStructure);

  		for(i = LED_FACE ; i <= LED_SOCIAL ; i++){
			GPIO_SetBits(leds[i].pwm.GPIOx, leds[i].pwm.GPIO_Pin); /* turn off LEDs, active low */
		}
	}
}

/*
 * period : 1-127 				=> 1/1s - 1/127s
 *			(-2) - (-128)		=> 2s	- 128s
 */
void tim_config(rt_int8_t led, rt_int8_t period, rt_uint8_t duty)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	struct _led_pwm *pwm = &(leds[led].pwm);
	rt_base_t level;

	if(!period) return;

	/* disable interrupt */
	level = rt_hw_interrupt_disable();

	if( pwm->period != period){ /* changing period, all channels are affected! */
		if(period < 0 ){/* pwm in period > 1.0 s */
			pwm->period_clk = (pwm->wclock * (-period) ) - 1;
		}else{/* pwm in Hz */
			pwm->period_clk = (pwm->wclock / period ) - 1;
		}
		pwm->period = period;
		/* Time Base configuration */
		TIM_TimeBaseStructure.TIM_Prescaler = pwm->preScale;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = pwm->period_clk;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(pwm->tim, &TIM_TimeBaseStructure);
	}

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	/* TIM clock enable */
	if( pwm->tim == TIM16 )
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , ENABLE);
	else if( pwm->tim == TIM17)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17 , ENABLE);
	else if(pwm->tim == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	else if(pwm->tim == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	else if(pwm->tim == TIM14)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Compute CCRx value to generate a duty cycle at ?% for channel x */
	pwm->on_clk = (uint16_t) (((uint32_t) duty * (pwm->period_clk - 1)) / 100);
	TIM_OCInitStructure.TIM_Pulse = pwm->on_clk;
	if(pwm->ch == 1)
		TIM_OC1Init(pwm->tim, &TIM_OCInitStructure);
	else if(pwm->ch == 2)
		TIM_OC2Init(pwm->tim, &TIM_OCInitStructure);
	else if(pwm->ch == 3)
		TIM_OC3Init(pwm->tim, &TIM_OCInitStructure);
	else if(pwm->ch == 4)
		TIM_OC4Init(pwm->tim, &TIM_OCInitStructure);

	/* TIM enable counter */
	TIM_Cmd(pwm->tim, ENABLE);

	/* TIM Main Output enable */
	TIM_CtrlPWMOutputs(pwm->tim, ENABLE);

	/* enable interrupt */
	rt_hw_interrupt_enable(level);
}

void isr_update_pwm(rt_uint8_t n)
{
	struct _led_dev *led = leds + n;
	struct _led_blink *pblinks;
	pblinks = &(led->cmdq.blinks);

	if(led->cmdq.blinks.rpts == 1){ /* blinking is done */
		pblinks->rpts --;
  		/* TIM disable counter */
		TIM_Cmd(led->pwm.tim, DISABLE);

		/* TIM Main Output disable */
		TIM_CtrlPWMOutputs(led->pwm.tim, DISABLE);

		/* TIM clock disable */
		if( led->pwm.tim == TIM16 )
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , DISABLE);
		else if( led->pwm.tim == TIM17)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17 , DISABLE);
		else if(led->pwm.tim == TIM2)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);
		else if(led->pwm.tim == TIM3)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
		else if(led->pwm.tim == TIM14)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, DISABLE);
	}else{
		++led->tok;
		led->tok = led->tok % pblinks->toks;/* get current active token */

		tim_config(n, pblinks->seq[led->tok].period, pblinks->seq[led->tok].on);
		if( led->tok == (pblinks->toks - 1) ) /* the last token in the sequence */
			pblinks->rpts --;
	}

}

/* set up led behavior */
void set_hw_led(rt_uint8_t led)
{
	struct _led_blink *pblinks;

	if(led >= LED_TOTAL){
		printk("led number %d is not supported\n", led);
		return;
	}

	if (rt_sem_take(&led_sem, RT_WAITING_FOREVER) != RT_EOK) return;
	pblinks = &(leds[led].cmdq.blinks);
	tim_config(led, pblinks->seq[0].period, pblinks->seq[0].on);

	rt_sem_release(&led_sem);
}

void led_init(void)
{
	rt_sem_init(&led_sem, "pwm_led", 1, 0);

    gpio_config(0);
}

#ifdef RT_USING_FINSH
/*
 * 0xC1 0x27 len led# toks repeat (token0 token1 token2 ...) checksum
 */
#include <finsh.h>
void setled(rt_uint8_t led,...)
{
	rt_uint8_t toks;
	rt_uint32_t rpt;
	va_list num_list;

	va_start(num_list, led);
    /* init led configuration if it's not inited. */
	toks = va_arg(num_list, rt_uint8_t);
	rpt = va_arg(num_list, rt_uint32_t);
	while(toks){
		rt_int8_t period = va_arg(num_list, rt_int8_t);
		rt_int8_t on  = va_arg(num_list, rt_int8_t);
//		tok[i++] = period,on
	}
//	set_hw_led(led, toks, rpt, cmd);

	va_end(num_list);
}
FINSH_FUNCTION_EXPORT(setled, "setled(led[0-7], ticks[10-], on[0-101], cnt[0-]")
#endif
#endif//LED_PWM_SUPPORTED

