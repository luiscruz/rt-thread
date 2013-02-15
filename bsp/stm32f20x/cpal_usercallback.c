/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Examples/STM32_CPAL_I2C/Two_Boards/cpal_usercallback.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file provides all the CPAL UserCallback functions .
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

/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include <stdio.h>
#include "stm32f2xx.h"
#include "cpal_i2c.h"
#define I2C_DevStructure        I2C1_DevStructure

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CPAL_TransferTypeDef  sRxStructure, sTxStructure;

#define MAX_BUFF_SIZE           200
#define BUFFER_SIZE             (countof(tStateSignal)-1)

uint8_t tRxBuffer[MAX_BUFF_SIZE];
uint32_t BufferSize = 32;

__IO uint32_t RecieverMode;

extern I2C_TypeDef* CPAL_I2C_DEVICE[];

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/*------------------------------------------------------------------------------
                     CPAL User Callbacks implementations
------------------------------------------------------------------------------*/


/*=========== Timeout UserCallback ===========*/


/**
  * @brief  User callback that manages the Timeout error.
  * @param  pDevInitStruct .
  * @retval None.
  */
uint32_t CPAL_TIMEOUT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
  /* Generate STOP */
  __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);

  CPAL_LOG("  Timeout Err\r\n");

#if 0
  I2C_DevStructure.CPAL_State = CPAL_STATE_READY;
  I2C_DevStructure.wCPAL_DevError = CPAL_I2C_ERR_NONE ;        /* No Device Error */
  I2C_DevStructure.wCPAL_Timeout  = CPAL_I2C_TIMEOUT_DEFAULT;  /* Stop timeout countdown */
#endif

  /* DeInitialize CPAL device */
  CPAL_I2C_DeInit(&I2C_DevStructure);

  /* Initialize CPAL device with the selected parameters */
  CPAL_I2C_Init(pDevInitStruct);

  CPAL_LOG("  Timeout Recovered\r\n");

  return CPAL_PASS;
}



/*=========== Transfer UserCallback ===========*/


/**
  * @brief  Manages the End of Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
  CPAL_LOG("CPAL_I2C_TXTC_UserCallback\r\n");

}


/**
  * @brief  Manages the End of Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
  uint8_t  i = 0;

  /* Activate the mode receiver only */
  RecieverMode = 1;

  CPAL_LOG("CPAL_I2C_RXTC_UserCallback\r\n");

  /* Initialize local Reception structures */
  sRxStructure.wNumData = BufferSize;       /* Maximum Number of data to be received */
  sRxStructure.pbBuffer = tRxBuffer;        /* Common Rx buffer for all received data */

#if 0
  /* Reinitialize RXBuffer */
  for(i = 0; i < MAX_BUFF_SIZE; i++)
  {
    tRxBuffer[i]=0;
  }
#endif
}

/**
  * @brief  Manages Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_TX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_RX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages the End of DMA Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages the Half of DMA Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMATXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages Error of DMA Tx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMATXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages the End of DMA Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMARXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages the Half of DMA Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMARXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  Manages Error of DMA Rx transfer event.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DMARXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/



/*=========== Error UserCallback ===========*/


/**
  * @brief  User callback that manages the I2C device errors.
  * @note   Make sure that the define USE_SINGLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInitStruct.
  * @param  DeviceError.
  * @retval None
  */
void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)
{
  /* if an Acknowledge failure error occurred */
  if (I2C_DevStructure.wCPAL_DevError == CPAL_I2C_ERR_AF )
  {
    CPAL_LOG(" Slave Not yet Ready\r\n");
  }
  else
  {
    CPAL_LOG(" Device Err occurred\r\n");
  }
  /* Generate STOP */
  __CPAL_I2C_HAL_STOP(pDevInstance);

  I2C_DevStructure.CPAL_State = CPAL_STATE_READY;
  I2C_DevStructure.wCPAL_DevError = CPAL_I2C_ERR_NONE ;        /* No Device Error */

  /* Deinitialize CPAL device */
  CPAL_I2C_DeInit(&I2C_DevStructure);

  /* Initialize CPAL device with the selected parameters */
  CPAL_I2C_Init(&I2C_DevStructure);

}

/**
  * @brief  User callback that manages BERR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_BERR_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/**
  * @brief  User callback that manages ARLO I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_ARLO_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/**
  * @brief  User callback that manages OVR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_OVR_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/**
  * @brief  User callback that manages AF I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_AF_UserCallback(CPAL_DevTypeDef pDevInstance)
{

}*/


/*=========== Addressing Mode UserCallback ===========*/


/**
  * @brief  User callback that manage General Call Addressing mode.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_GENCALL_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/**
  * @brief  User callback that manage Dual Address Addressing mode.
  * @param  pDevInitStruct
  * @retval None
  */
/*void CPAL_I2C_DUALF_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}*/


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
