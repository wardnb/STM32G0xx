/**
  ******************************************************************************
  * @file    stm32g0xx_hal_uart_ex.c
  * @author  MCD Application Team
  * @brief   Extended UART HAL module driver.
  *          This file provides firmware functions to manage the following extended
  *          functionalities of the Universal Asynchronous Receiver Transmitter (UART) peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/** @addtogroup STM32G0xx_HAL_Driver
  * @{
  */

/** @defgroup UARTEx UARTEx
  * @brief UART Extended HAL module driver
  * @{
  */

#ifdef HAL_UART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup UARTEx_Exported_Functions UARTEx Exported Functions
  * @{
  */

/** @defgroup UARTEx_Exported_Functions_Group1 Extended Peripheral Control functions
  * @brief    Extended Peripheral Control functions
  *
@verbatim
  ==============================================================================
                        ##### Peripheral Control functions #####
  ==============================================================================
    [..] This section provides functions allowing to:
      (+) Initialize the UART wake up from stop mode parameters
      (+) Configure the UART FIFO mode
      (+) Enable/Disable the UART FIFO mode

@endverbatim
  * @{
  */

/**
  * @brief  Set the TXFIFO threshold.
  * @param  huart         UART handle.
  * @param  Threshold     TX FIFO threshold value
  *          This parameter can be one of the following values:
  *            @arg @ref UART_TXFIFO_THRESHOLD_1_8
  *            @arg @ref UART_TXFIFO_THRESHOLD_1_4
  *            @arg @ref UART_TXFIFO_THRESHOLD_1_2
  *            @arg @ref UART_TXFIFO_THRESHOLD_3_4
  *            @arg @ref UART_TXFIFO_THRESHOLD_7_8
  *            @arg @ref UART_TXFIFO_THRESHOLD_8_8
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold)
{
  uint32_t tmpcr1;

  /* Check parameters */
  assert_param(IS_UART_FIFO_INSTANCE(huart->Instance));
  assert_param(IS_UART_TXFIFO_THRESHOLD(Threshold));

  /* Process Locked */
  __HAL_LOCK(huart);

  huart->gState = HAL_UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Update TX threshold configuration */
  MODIFY_REG(huart->Instance->CR3, USART_CR3_TXFTCFG, Threshold);

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  huart->gState = HAL_UART_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

/**
  * @brief  Set the RXFIFO threshold.
  * @param  huart         UART handle.
  * @param  Threshold     RX FIFO threshold value
  *          This parameter can be one of the following values:
  *            @arg @ref UART_RXFIFO_THRESHOLD_1_8
  *            @arg @ref UART_RXFIFO_THRESHOLD_1_4
  *            @arg @ref UART_RXFIFO_THRESHOLD_1_2
  *            @arg @ref UART_RXFIFO_THRESHOLD_3_4
  *            @arg @ref UART_RXFIFO_THRESHOLD_7_8
  *            @arg @ref UART_RXFIFO_THRESHOLD_8_8
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold)
{
  uint32_t tmpcr1;

  /* Check parameters */
  assert_param(IS_UART_FIFO_INSTANCE(huart->Instance));
  assert_param(IS_UART_RXFIFO_THRESHOLD(Threshold));

  /* Process Locked */
  __HAL_LOCK(huart);

  huart->gState = HAL_UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Update RX threshold configuration */
  MODIFY_REG(huart->Instance->CR3, USART_CR3_RXFTCFG, Threshold);

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  huart->gState = HAL_UART_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

/**
  * @brief  Disable the FIFO mode.
  * @param  huart UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart)
{
  uint32_t tmpcr1;

  /* Check parameters */
  assert_param(IS_UART_FIFO_INSTANCE(huart->Instance));

  /* Process Locked */
  __HAL_LOCK(huart);

  huart->gState = HAL_UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Disable FIFO mode */
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_FIFOEN);
  huart->FifoMode = UART_FIFOMODE_DISABLE;

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  huart->gState = HAL_UART_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

/**
  * @brief  Enable the FIFO mode.
  * @param  huart UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *huart)
{
  uint32_t tmpcr1;

  /* Check parameters */
  assert_param(IS_UART_FIFO_INSTANCE(huart->Instance));

  /* Process Locked */
  __HAL_LOCK(huart);

  huart->gState = HAL_UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Enable FIFO mode */
  SET_BIT(tmpcr1, USART_CR1_FIFOEN);
  huart->FifoMode = UART_FIFOMODE_ENABLE;

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  /* Determine the number of data to process during RX/TX ISR execution */
  UARTEx_SetNbDataToProcess(huart);

  huart->gState = HAL_UART_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup UARTEx_Private_Functions
  * @{
  */

/**
  * @brief  Calculate the number of data to process in RX/TX ISR.
  * @note   The RX FIFO depth and the TX FIFO depth is extracted from
  *         the UART configuration registers.
  * @param  huart UART handle.
  * @retval None
  */
void UARTEx_SetNbDataToProcess(UART_HandleTypeDef *huart)
{
  /* Simplified implementation for STM32G0xx */
  huart->NbRxDataToProcess = 1U;
  huart->NbTxDataToTransmit = 1U;
}

/**
  * @}
  */

#endif /* HAL_UART_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */