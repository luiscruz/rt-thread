#ifndef _H_KEYPAD_H
#define	_H_KEYPAD_H
/*
 * pwr key				PA0
 * PC5,PC6,PC7			: col, input , falling edge trigger
 * PG11, PG13, PG7		: row, output low
 */
#define	POWER_KEY_GPIO		GPIOA
#define	POWER_KEY_PIN		GPIO_Pin_0

/* all keys are active low and default is not active and pull-up*/
#define	POWER_KEY_DOWN		!GPIO_ReadInputDataBit(POWER_KEY_GPIO, POWER_KEY_PIN)

/* */
#define	KEY_COL_IST(l)		(EXTI_GetITStatus(l)==SET)
#define	KEY_COL0_IST		EXTI_GetITStatus(EXTI_Line5)
#define	KEY_COL1_IST		EXTI_GetITStatus(EXTI_Line6)
#define	KEY_COL2_IST		EXTI_GetITStatus(EXTI_Line7)

#define	KEY_COL0_GPIO		GPIOC
#define	KEY_COL1_GPIO		GPIOC
#define	KEY_COL2_GPIO		GPIOC

#define	KEY_COL0_PIN		GPIO_Pin_5
#define	KEY_COL1_PIN		GPIO_Pin_6
#define	KEY_COL2_PIN		GPIO_Pin_7


typedef struct {
GPIO_TypeDef * port;
uint16_t	pin;
} KEY_CONFG;

#define	IS_COL_LOW(k)	!GPIO_ReadInputDataBit(k.port, k.pin)
#define	KEY_COL0_DOWN		!GPIO_ReadInputDataBit(KEY_COL0_GPIO, KEY_COL0_PIN)
#define	KEY_COL1_DOWN		!GPIO_ReadInputDataBit(KEY_COL1_GPIO, KEY_COL1_PIN)
#define	KEY_COL2_DOWN		!GPIO_ReadInputDataBit(KEY_COL2_GPIO, KEY_COL2_PIN)


#define	KEY_ROW0_GPIO		GPIOG
#define	KEY_ROW1_GPIO		GPIOG
#define	KEY_ROW2_GPIO		GPIOG

#define	KEY_ROW0_PIN		GPIO_Pin_11
#define	KEY_ROW1_PIN		GPIO_Pin_13
#define	KEY_ROW2_PIN		GPIO_Pin_7


#define KEYPAD_PULL_UP(k)		GPIO_SetBits(k.port, k.pin)
#define KEYPAD_PULL_DOWN(k)		GPIO_ResetBits(k.port, k.pin)
#define KEY_ROW0_PULL_UP		GPIO_SetBits(KEY_ROW0_GPIO, KEY_ROW0_PIN)
#define KEY_ROW0_PULL_DOWN  	GPIO_ResetBits(KEY_ROW0_GPIO, KEY_ROW0_PIN)
#define KEY_ROW1_PULL_UP		GPIO_SetBits(KEY_ROW1_GPIO, KEY_ROW1_PIN)
#define KEY_ROW1_PULL_DOWN  	GPIO_ResetBits(KEY_ROW1_GPIO, KEY_ROW1_PIN)
#define KEY_ROW2_PULL_UP		GPIO_SetBits(KEY_ROW2_GPIO, KEY_ROW2_PIN)
#define KEY_ROW2_PULL_DOWN  	GPIO_ResetBits(KEY_ROW2_GPIO, KEY_ROW2_PIN)

#endif
