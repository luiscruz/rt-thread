#ifndef _H_KEYPAD_H
#define	_H_KEYPAD_H

#define	POWER_KEY_GPIO		GPIOA
#define	POWER_KEY_PIN		GPIO_Pin_0

/* active low */
#define	POWER_KEY_DOWN		!GPIO_ReadInputDataBit(POWER_KEY_GPIO, POWER_KEY_PIN)

#endif
