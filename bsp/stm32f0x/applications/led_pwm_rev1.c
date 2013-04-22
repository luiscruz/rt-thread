/*
 * File      : led_pwm.c
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

#include <rtthread.h>
#include <stm32f0xx.h>
#include "board.h"
#ifdef	LED_PWM_SUPPORTED
#include "led_pwm.h"
#define	printk	rt_kprintf

static rt_timer_t led_timer;
struct rt_semaphore led_sem;

static struct led_dev leds[] ={
	{LED_FACE_GPIO_PORT,LED_FACE_PIN, 2, 2},
	/*{LED_CROWN_GPIO_PORT,LED_CROWN_PIN, 3, 4s},*/
	{LED_PHONE_GPIO_PORT,LED_PHONE_PIN, 3, 1},
	{LED_MAIL_GPIO_PORT,LED_MAIL_PIN, 3, 2},
	{LED_SOCIAL_GPIO_PORT,LED_SOCIAL_PIN, 3, 3}
	};

static rt_uint16_t Timer3PCnt = 0, Timer2PCnt = 0, Timer3Clock=1000, Timer2Clock=1000;
static rt_uint16_t TIM3Ch1 = 0, TIM3Ch2 = 0, TIM3Ch3 = 0, TIM3Ch4 = 0;
static rt_uint16_t TIM2Ch1 = 0, TIM2Ch2 = 0, TIM2Ch3 = 0, TIM2Ch4 = 0;
static rt_uint16_t TIM3PreScale=48000, TIM2PreScale=48000; /* making counter 48M/48K = 1khz */
static rt_uint8_t Tim3_flg, Tim2_flg;
static rt_int8_t Tim3Period, Tim2Period;

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
 *	LED_PHONE_PIN	PA6		TIM3_CH1
 *	LED_MAIL_PIN	PA7		TIM3_CH2
 *	LED_SOCIAL_PIN	PB0		TIM3_CH3
 */
static void gpio_config(rt_uint8_t on_off)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	if(on_off){/* on to TIMx setting */
	  	/* GPIOA,B Periph clock enable */
	  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE);
	  	/* TIM3 clock enable */
	 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2, ENABLE);

	   	GPIO_InitStructure.GPIO_Pin = LED_FACE_PIN | LED_PHONE_PIN | LED_MAIL_PIN;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	  	GPIO_InitStructure.GPIO_Pin =  LED_SOCIAL_PIN;/* | LED_CROWN_PIN;*/
	  	GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Connect TIM Channels to AF2, TIM2 CH2 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);

	  	/* Connect TIM Channels to AF1, TIM3 CH1,CH2 */
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);

		/* high power TIM3 CH3,4*/
	  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);
	 	/*GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);*/
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
			GPIO_SetBits(leds[i].GPIOx, leds[i].GPIO_Pin); /* turn off LEDs */
		}
	}
}

#if 0
static void TIM_Config(rt_uint8_t tim, rt_int8_t period)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	gpio_config(1);

	if( (tim == 3) && !(Tim3_flg & TIM_ACTIVE) ){
		Tim3_flg = TIM_ACTIVE;	/* tim3 on now*/
		Tim3Period = period;
		Timer3Clock = (SystemCoreClock / TIM3PreScale ); /* 1khz working clocks */
		TIM3Ch1 = TIM3Ch2 = TIM3Ch3 = TIM3Ch4 = 0;
		if(Tim3Period < 0 ){/* period > 1.0 s */
			Timer3PCnt = (Timer3Clock * (-Tim3Period) ) - 1;
		}else{
			Timer3PCnt = (Timer3Clock / Tim3Period ) - 1;
		}
  		/* Time Base configuration */
  		TIM_TimeBaseStructure.TIM_Prescaler = TIM3PreScale;
  		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  		TIM_TimeBaseStructure.TIM_Period = Timer3PCnt;
  		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/* test
		TIM3Ch1 = (uint16_t) (((uint32_t) 50 * (Timer3PCnt - 1)) / 100);
		TIM3Ch2 = (uint16_t) (((uint32_t) 10 * (Timer3PCnt - 1)) / 100);
		TIM3Ch3 = (uint16_t) (((uint32_t) 80 * (Timer3PCnt - 1)) / 100);
		TIM3Ch4 = (uint16_t) (((uint32_t) 90 * (Timer3PCnt - 1)) / 100);
		*/

  		/* Channel 1, 2,3 and 4 Configuration in PWM mode */
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  		TIM_OCInitStructure.TIM_Pulse = TIM3Ch1;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;
  		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//TIM_OCNPolarity_High
  		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  		TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  		TIM_OCInitStructure.TIM_Pulse = TIM3Ch2;
  		TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  		TIM_OCInitStructure.TIM_Pulse = TIM3Ch3;
  		TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  		TIM_OCInitStructure.TIM_Pulse = TIM3Ch4;
  		TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  		/* TIM enable counter */
  		TIM_Cmd(TIM3, ENABLE);

  		/* TIM3 Main Output Enable */
  		TIM_CtrlPWMOutputs(TIM3, ENABLE);
	}else if( (tim ==2) && !(Tim2_flg & TIM_ACTIVE) ){
		Tim2_flg = TIM_ACTIVE;	/* tim2 on now*/
		Tim2Period = period;
		Timer2Clock = (SystemCoreClock / TIM2PreScale ); /* 1khz working clocks */
		TIM2Ch1 = TIM2Ch2 = TIM2Ch3 = TIM2Ch4 = 0;
		if(Tim2Period < 0 ){/* period > 1.0 s */
			Timer2PCnt = (Timer2Clock * (-Tim2Period) ) - 1;
		}else{
			Timer2PCnt = (Timer2Clock / Tim2Period ) - 1; /* 1/p hz*/
		}
  		/* Time Base configuration */
  		TIM_TimeBaseStructure.TIM_Prescaler = TIM2PreScale;
  		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  		TIM_TimeBaseStructure.TIM_Period = Timer2PCnt;
  		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		/* test
		TIM2Ch1 = (uint16_t) (((uint32_t) 50 * (Timer2PCnt - 1)) / 100);
		TIM2Ch2 = (uint16_t) (((uint32_t) 10 * (Timer2PCnt - 1)) / 100);
		TIM2Ch3 = (uint16_t) (((uint32_t) 80 * (Timer2PCnt - 1)) / 100);
		TIM2Ch4 = (uint16_t) (((uint32_t) 90 * (Timer2PCnt - 1)) / 100);*/
  		/* Channel 1, 2,3 and 4 Configuration in PWM mode */
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  		TIM_OCInitStructure.TIM_Pulse = TIM2Ch1;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;
  		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//TIM_OCNPolarity_High
  		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  		TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  		TIM_OCInitStructure.TIM_Pulse = TIM2Ch2;
  		TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  		TIM_OCInitStructure.TIM_Pulse = TIM2Ch3;
  		TIM_OC3Init(TIM2, &TIM_OCInitStructure);

  		TIM_OCInitStructure.TIM_Pulse = TIM2Ch4;
  		TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  		/* TIM enable counter */
  		TIM_Cmd(TIM2, ENABLE);

  		/* TIM3 Main Output Enable */
  		TIM_CtrlPWMOutputs(TIM2, ENABLE);
	}
}
#endif

/*
 * period : 1-127 				=> 1/1s - 1/127s
 *			(-2) - (-128)		=> 2s	- 128s
 */
static void TIMx_CHx(rt_int8_t tim, rt_int8_t ch, rt_int8_t period, rt_uint8_t duty)
{
	rt_uint16_t *pTimerPCnt=RT_NULL,*pTimerClock=RT_NULL;
	rt_uint16_t *pTIMCh1 = RT_NULL, *pTIMCh2 =RT_NULL, *pTIMCh3 =RT_NULL, *pTIMCh4 =RT_NULL;
	rt_uint8_t *pTim_flg=RT_NULL;
	rt_int8_t *pTimPeriod=RT_NULL;
	rt_int8_t op;
	rt_uint16_t prescale=0;
	TIM_TypeDef* timx=RT_NULL;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	if(!period) return;
	if(tim == 3){
		timx = TIM3;
		prescale = TIM3PreScale;
		pTimerPCnt=&Timer3PCnt;
		pTimerClock=&Timer3Clock;
		pTIMCh1 = &TIM3Ch1;
		pTIMCh2 = &TIM3Ch2;
		pTIMCh3 = &TIM3Ch3;
		pTIMCh4 = &TIM3Ch4;
		pTim_flg= &Tim3_flg;
		pTimPeriod=&Tim3Period;
	}else if(tim == 2){
		timx = TIM2;
		prescale = TIM2PreScale;
		pTimerPCnt=&Timer2PCnt;
		pTimerClock=&Timer2Clock;
		pTIMCh1 = &TIM2Ch1;
		pTIMCh2 = &TIM2Ch2;
		pTIMCh3 = &TIM2Ch3;
		pTIMCh4 = &TIM2Ch4;
		pTim_flg= &Tim2_flg;
		pTimPeriod=&Tim2Period;
	} else return;
gpio_config(1);
	if( (ch<=4) && (ch >=1) ){
		if(duty)
			*pTim_flg |= TIM_CH1_ACTIVE << (ch-1);
		else if(*pTim_flg & (TIM_CH1_ACTIVE << (ch-1)) ){
			*pTim_flg &= !TIM_CH1_ACTIVE << (ch-1); /* clear channel bit while off */
		}else{
			return; /* it has been off! */
		}
	}else return;

	op = *pTimPeriod;

	#if 0
	if(!(*pTim_flg & TIM_ACTIVE) ){ /* TIM3 is not on yet, config TIM3 */
		TIM_Config(tim, period);
		op = period;
	}
	#endif

	if(op != period){ /* changing period, all channels are affected! */
		*pTimPeriod = period;
		if(period < 0 ){/* period > 1.0 s */
			*pTimerPCnt = (*pTimerClock * (-period) ) - 1;
		}else{
			*pTimerPCnt = (*pTimerClock / period ) - 1;
		}
		/* Time Base configuration */
		TIM_TimeBaseStructure.TIM_Prescaler = prescale;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = *pTimerPCnt;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(timx, &TIM_TimeBaseStructure);
	}
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	if(ch == 1){
		/* Compute CCR1 value to generate a duty cycle at ?% for channel 1 and 1N */
		*pTIMCh1 = (uint16_t) (((uint32_t) duty * (*pTimerPCnt - 1)) / 100);
		TIM_OCInitStructure.TIM_Pulse = *pTIMCh1;
		TIM_OC1Init(timx, &TIM_OCInitStructure);
	}else if(ch==2){
		/* Compute CCR2 value to generate a duty cycle at ?% for channel 1 and 1N */
		*pTIMCh2 = (uint16_t) (((uint32_t) duty * (*pTimerPCnt - 1)) / 100);
		TIM_OCInitStructure.TIM_Pulse = *pTIMCh2;
		TIM_OC2Init(timx, &TIM_OCInitStructure);
	}else if(ch==3){
	/* Compute CCR3 value to generate a duty cycle at ?% for channel 1 and 1N */
		*pTIMCh3 = (uint16_t) (((uint32_t) duty * (*pTimerPCnt - 1)) / 100);
		TIM_OCInitStructure.TIM_Pulse = *pTIMCh3;
		TIM_OC3Init(timx, &TIM_OCInitStructure);
	}else if(ch==4){
	/* Compute CCR3 value to generate a duty cycle at ?% for channel 1 and 1N */
		*pTIMCh4 = (uint16_t) (((uint32_t) duty * (*pTimerPCnt - 1)) / 100);
		TIM_OCInitStructure.TIM_Pulse = *pTIMCh4;
		TIM_OC4Init(timx, &TIM_OCInitStructure);
	}

	if( (*pTim_flg & TIM_ACTIVE) &&
		!(*pTim_flg & (TIM_CH1_ACTIVE|TIM_CH2_ACTIVE|TIM_CH3_ACTIVE|TIM_CH4_ACTIVE))){
  		gpio_config(0);
  		/* TIM disable counter */
		TIM_Cmd(timx, DISABLE);

		/* TIM3 Main Output disable */
		TIM_CtrlPWMOutputs(timx, DISABLE);
		/* TIM3 clock disable */
		RCC_APB1PeriphClockCmd((tim == 3)?RCC_APB1Periph_TIM3:RCC_APB1Periph_TIM2 , DISABLE);

		*pTim_flg &= !TIM_ACTIVE;
	}else if( !(*pTim_flg & TIM_ACTIVE) &&
		(*pTim_flg & (TIM_CH1_ACTIVE|TIM_CH2_ACTIVE|TIM_CH3_ACTIVE|TIM_CH4_ACTIVE))){
		rt_uint8_t flag;
  		gpio_config(1);
  		/* TIM enable counter */
		TIM_Cmd(timx, ENABLE);

		/* TIM3 Main Output enable */
		TIM_CtrlPWMOutputs(timx, ENABLE);
		/* TIM3 clock enable */
		RCC_APB1PeriphClockCmd((tim == 3)?RCC_APB1Periph_TIM3:RCC_APB1Periph_TIM2 , ENABLE);
		*pTim_flg |= TIM_ACTIVE;

		/* start timer */
		rt_timer_control(led_timer, RT_TIMER_CTRL_GET_FLAG, &flag);
		if( !(flag & RT_TIMER_FLAG_ACTIVATED) )
    		rt_timer_start(led_timer);
	}
}

#if 0
void TIM_Chx(rt_uint8_t tim, rt_int8_t ch, rt_int8_t period, rt_uint8_t duty )
{
	rt_int8_t op;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	if(tim == 3){
		op = Tim3Period;
		if(!(Tim3_flg & TIM_ACTIVE) ){ /* TIM3 is not on yet, config TIM3 */
			TIM_Config(3, period );
			op = Tim3Period;
		}
		if(op != period){ /* changing period, all channels are affected! */
			if(period < 0 ){/* period > 1.0 s */
				Timer3PCnt = (Timer3Clock * (-period) ) - 1;
			}else{
				Timer3PCnt = (Timer3Clock / period ) - 1;
			}
  			/* Time Base configuration */
  			TIM_TimeBaseStructure.TIM_Prescaler = TIM3PreScale;
  			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  			TIM_TimeBaseStructure.TIM_Period = Timer3PCnt;
  			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		}
  		/* Channel 1, 2,3 and 4 Configuration in PWM mode */
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

		if(ch == 1){
  			/* Compute CCR1 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM3Ch1 = (uint16_t) (((uint32_t) duty * (Timer3PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM3Ch1;
  			TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		}else if(ch==2){
  			/* Compute CCR2 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM3Ch2 = (uint16_t) (((uint32_t) duty * (Timer3PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM3Ch2;
  			TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		}else if(ch==3){
		/* Compute CCR3 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM3Ch3 = (uint16_t) (((uint32_t) duty * (Timer3PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM3Ch3;
  			TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		}else if(ch==4){
		/* Compute CCR3 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM3Ch4 = (uint16_t) (((uint32_t) duty * (Timer3PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM3Ch4;
  			TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    	}

    	if(ch<=4 && ch >=1){
    		if(duty)
				Tim3_flg |= TIM_CH1_ACTIVE << (ch-1);
			else
				Tim3_flg &= !TIM_CH1_ACTIVE << (ch-1);
    	}

    	if( (Tim3_flg & TIM_ACTIVE) &&
    		!(Tim3_flg & (TIM_CH1_ACTIVE|TIM_CH2_ACTIVE|TIM_CH3_ACTIVE|TIM_CH4_ACTIVE))){
    	  		/* TIM disable counter */
  			TIM_Cmd(TIM3, DISABLE);

  			/* TIM3 Main Output disable */
  			TIM_CtrlPWMOutputs(TIM3, DISABLE);
  			/* TIM3 clock disable */
 			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , DISABLE);
  			Tim3_flg &= !TIM_ACTIVE;
    	}
	}else if(tim == 2){
		op = Tim2Period;
		if(!(Tim2_flg & TIM_ACTIVE) ) /* TIM2 is not on yet, config TIM3 */
			TIM_Config(2, period );

		if(op != period){ /* changing period, all channels are affected! */
			if(period < 0 ){/* period > 1.0 s */
				Timer2PCnt = (Timer2Clock * (-period) ) - 1;
			}else{
				Timer2PCnt = (Timer2Clock / period ) - 1;
			}
  			/* Time Base configuration */
  			TIM_TimeBaseStructure.TIM_Prescaler = TIM2PreScale;
  			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  			TIM_TimeBaseStructure.TIM_Period = Timer2PCnt;
  			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  			TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		}
  		/* Channel 1, 2,3 and 4 Configuration in PWM mode */
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

		if(ch == 1){
  			/* Compute CCR1 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM2Ch1 = (uint16_t) (((uint32_t) duty * (Timer2PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM2Ch1;
  			TIM_OC1Init(TIM2, &TIM_OCInitStructure);
		}else if(ch==2){
  			/* Compute CCR2 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM2Ch2 = (uint16_t) (((uint32_t) duty * (Timer2PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM2Ch2;
  			TIM_OC2Init(TIM2, &TIM_OCInitStructure);
		}else if(ch==3){
		/* Compute CCR3 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM2Ch3 = (uint16_t) (((uint32_t) duty * (Timer2PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM2Ch3;
  			TIM_OC3Init(TIM2, &TIM_OCInitStructure);
		}else if(ch==4){
		/* Compute CCR3 value to generate a duty cycle at ?% for channel 1 and 1N */
  			TIM2Ch4 = (uint16_t) (((uint32_t) duty * (Timer2PCnt - 1)) / 100);
  			TIM_OCInitStructure.TIM_Pulse = TIM2Ch4;
   			TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    	}

    	if(ch<=4 && ch >=1){
    		if(duty)
				Tim2_flg |= TIM_CH1_ACTIVE << (ch-1);
			else
				Tim2_flg &= !TIM_CH1_ACTIVE << (ch-1);
    	}

    	if( (Tim2_flg & TIM_ACTIVE) &&
    		!(Tim2_flg & (TIM_CH1_ACTIVE|TIM_CH2_ACTIVE|TIM_CH3_ACTIVE|TIM_CH4_ACTIVE))){
    	  		/* TIM disable counter */
  			TIM_Cmd(TIM2, DISABLE);

  			/* TIM3 Main Output disable */
  			TIM_CtrlPWMOutputs(TIM2, DISABLE);
  			/* TIM2 clock disable */
 			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);
  			Tim2_flg &= !TIM_ACTIVE;
    	}
	}

}
#endif

/* set up led behavior */
void set_hw_led(rt_uint8_t led, rt_int8_t period, rt_uint8_t on, rt_uint32_t cnt)
{
	if(led >= LED_TOTAL){
		printk("led number %d is not supported\n", led);
		return;
	}

	if (rt_sem_take(&led_sem, RT_WAITING_FOREVER) != RT_EOK) return;
	leds[led].mode.duty = on;
	leds[led].mode.period = period;
	leds[led].mode.cnt = cnt;
	TIMx_CHx(leds[led].timer, leds[led].channel, period, on);
	rt_sem_release(&led_sem);
}


/* called back every 50ms to update the led on duty */
static void led_timer_func(void* parameter)
{
	int i, lon=0;
	struct led_dev *pled = (struct led_dev *)parameter;
	if(pled == RT_NULL) return;
	for(i = 0 ; i< LED_TOTAL ; i++){/* all leds */
		/* mutex lock */
		pled += i;
		if( pled->mode.cnt-- && pled->mode.duty) {
			if(pled->mode.cnt == 0)
				set_hw_led(i, 1, 0, 0);
			else lon++;
		}
	}
	if(!lon)/* no led is on, so stop timer thread! */
		rt_timer_stop(led_timer);

}

static void rt_hw_led_init(void)
{
#if 1
	gpio_config(0);
#else
	int i;

    GPIO_InitTypeDef GPIO_InitStructure;

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
		GPIO_SetBits(leds[i].GPIOx, leds[i].GPIO_Pin); /* turn off LEDs */
	}
#endif

}

void led_init(void)
{
	//led_mux = rt_mutex_create("led", RT_IPC_FLAG_FIFO);
	rt_sem_init(&led_sem, "pwm_led", 1, 0);

    rt_hw_led_init();

	/* create 1 second timer */
    led_timer = rt_timer_create("led",
    							led_timer_func,
    							leds,
                                RT_TICK_PER_SECOND,
                                RT_TIMER_FLAG_PERIODIC);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void setled(rt_uint8_t led, rt_uint16_t ticks, rt_uint8_t on, rt_uint32_t cnt)
{
    /* init led configuration if it's not inited. */

	set_hw_led(led, ticks, on, cnt);
}
FINSH_FUNCTION_EXPORT(setled, "setled(led[0-7], ticks[10-], on[0-101], cnt[0-]")
#endif
#endif//LED_PWM_SUPPORTED

