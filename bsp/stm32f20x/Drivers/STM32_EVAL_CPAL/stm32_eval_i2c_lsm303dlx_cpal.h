/**
  ******************************************************************************
  * @file    STM32_EVAL_CPAL/Common/stm32_eval_i2c_tsensor_cpal.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file contains all the functions prototypes for the
  *          stm32_eval_i2c_tsensor_cpal.c firmware driver.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_EVAL_I2C_TSENSOR_CPAL_H
#define __STM32_EVAL_I2C_TSENSOR_CPAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  TSENSOR Status
  */
typedef enum
{
  LSM330DLX_OK = 0,
  LSM330DLX_FAIL
}LSM330DLX_Status_TypDef;

/* Exported constants --------------------------------------------------------*/

/* CPAL Structure configuration : Select I2C device (uncomment relative define) */

#define LSM330DLX_DevStructure                I2C1_DevStructure
//#define LSM330DLX_DevStructure                I2C2_DevStructure
//#define LSM330DLX_DevStructure                I2C3_DevStructure

/* Select clock Speed */
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */

#define I2C_SPEED                        100000

/* Select interrupt programming model : By default DMA programming model is selected.
 To select interrupt programming model uncomment this define */
//#define LSM330DLX_IT

/* Maximum Timeout values for waiting until device is ready for communication.*/

#define LSM330DLX_TIMEOUT        ((uint32_t)0x3FFFF)

/**
  * @brief  Internal register Memory
  */

#define	LSM330DLH_A_ADDR	(0x30)	/*0x32 SA0=1*/
#define	LSM330DLH_M_ADDR	(0x3C)

#define	LSM330DLHC_A_ADDR	(0x32)	/*0x32 SA0=1*/
#define	LSM330DLHC_M_ADDR	(0x3C)

/* table for LSM330DLH (obsolete) 
 */
/* accelerometer */
#define CTRL_REG1_A  	(0x20)
#define CTRL_REG2_A  	(0x21)
#define CTRL_REG3_A  	(0x22)
#define CTRL_REG4_A  	(0x23)
#define CTRL_REG5_A  	(0x24)
#define CTRL_REG6_A  	(0x25)
#define REFERENCE_A  	(0x26)
#define STATUS_REG_A 	(0x27)
#define OUT_X_L_A  		(0x28)
#define OUT_X_H_A  		(0x29)
#define OUT_Y_L_A  		(0x2A)
#define OUT_Y_H_A  		(0x2B)
#define OUT_Z_L_A  		(0x2C)
#define OUT_Z_H_A  		(0x2D)
#define FIFO_CTRL_REG_A (0x2E)
#define FIFO_SRC_REG_A  (0x2F)
#define INT1_CFG_A  	(0x30)
#define INT1_SOURCE_A  	(0x31)
#define INT1_THS_A  	(0x32)
#define INT1_DURATION_A (0x33)
#define INT2_CFG_A  	(0x34)
#define INT2_SOURCE_A  	(0x35)
#define INT2_THS_A  	(0x36)
#define INT2_DURATION_A	(0x37)
#define CLICK_CFG_A  	(0x38)
#define CLICK_SRC_A  	(0x39)
#define CLICK_THS_A  	(0x3A)
#define TIME_LIMIT_A  	(0x3B)
#define TIME_LATENCY_A  (0x3C)
#define TIME_WINDOW_A   (0x3D)

/*magnetic */
#define CRA_REG_M  	(0x00)
#define CRB_REG_M  	(0x01)
#define MR_REG_M   	(0x02)
#define OUT_X_H_M  	(0x03)
#define OUT_X_L_M  	(0x04)
#define OUT_Y_H_M  	(0x05)
#define OUT_Y_L_M  	(0x06)
#define OUT_Z_H_M  	(0x07)
#define OUT_Z_L_M  	(0x08)
#define SR_REG_M  	(0x09)
#define IRA_REG_M  	(0x0A)
#define IRB_REG_M  	(0x0B)
#define IRC_REG_M  	(0x0C)

#define	TEMP_OUT_H_M 	(0x31)
#define	TEMP_OUT_L_M 	(0x32)

/**
  * @brief  GPIO config
  * INT1 : PB8
  * INT2 : PB9
  */
#define LSM330DLX_IT1_PIN                       GPIO_Pin_8
#define LSM330DLX_IT1_GPIO_PORT                 GPIOB
#define LSM330DLX_IT1_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define LSM330DLX_IT1_EXTI_PORT_SOURCE          EXTI_PortSourceGPIOB
#define LSM330DLX_IT1_EXTI_PIN_SOURCE           EXTI_PinSource8
#define LSM330DLX_IT1_EXTI_LINE                 EXTI_Line8
#define LSM330DLX_IT1_EXTI_IRQn                 EXTI9_5_IRQn

#define LSM330DLX_IT2_PIN                       GPIO_Pin_9
#define LSM330DLX_IT2_GPIO_PORT                 GPIOB
#define LSM330DLX_IT2_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define LSM330DLX_IT2_EXTI_PORT_SOURCE          EXTI_PortSourceGPIOB
#define LSM330DLX_IT2_EXTI_PIN_SOURCE           EXTI_PinSource9
#define LSM330DLX_IT2_EXTI_LINE                 EXTI_Line9
#define LSM330DLX_IT2_EXTI_IRQn                 EXTI9_5_IRQn

/* GPH6 : SENSOR1_RESET, active low */
#define	LSM330DLX_RESET_PORT		GPIOH
#define	LSM330DLX_RESET_PIN			GPIO_Pin_6
#define	LSM330DLX_RESET_GPIO_CLK	RCC_AHB1Periph_GPIOH
#define	LSM330DLX_RESET_ON			GPIO_ResetBits(LSM330DLX_RESET_PORT, LSM330DLX_RESET_PIN)
#define	LSM330DLX_RESET_OFF			GPIO_SetBits(LSM330DLX_RESET_PORT, LSM330DLX_RESET_PIN)

/* GPG8 SENSOR1_POWER: active High */
#define	LSM330DLX_POWER_PORT		GPIOG
#define	LSM330DLX_POWER_PIN			GPIO_Pin_8
#define	LSM330DLX_POWER_GPIO_CLK	RCC_AHB1Periph_GPIOG
#define	LSM330DLX_POWER_ON			GPIO_SetBits(LSM330DLX_POWER_PORT, LSM330DLX_POWER_PIN)
#define	LSM330DLX_POWER_OFF			GPIO_ResetBits(LSM330DLX_POWER_PORT, LSM330DLX_POWER_PIN)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void LSM330DLX_DeInit(void);
void LSM330DLX_Config(void);
ErrorStatus LSM330DLX_GetStatus(void);
uint16_t LSM330DLX_ReadTemp(void);
uint16_t LSM330DLX_ReadReg(uint8_t RegName);
uint8_t LSM330DLX_WriteReg(uint8_t RegName, uint16_t RegValue);
uint8_t LSM330DLX_ReadConfReg(void);
uint8_t LSM330DLX_WriteConfReg(uint8_t RegValue);
uint8_t LSM330DLX_ShutDown(FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_EVAL_I2C_TSENSOR_CPAL_H */
/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
