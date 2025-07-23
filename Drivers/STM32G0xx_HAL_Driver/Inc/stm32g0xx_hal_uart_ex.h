/**
  ******************************************************************************
  * @file    stm32g0xx_hal_uart_ex.h
  * @author  MCD Application Team
  * @brief   Header file of UART HAL Extended module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32G0xx_HAL_UART_EX_H
#define STM32G0xx_HAL_UART_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/** @addtogroup STM32G0xx_HAL_Driver
  * @{
  */

/** @addtogroup UARTEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup UARTEx_Exported_Constants UARTEx Exported Constants
  * @{
  */

/** @defgroup UARTEx_FIFO_mode UARTEx FIFO mode
  * @{
  */
#define UART_FIFOMODE_DISABLE        0x00000000U       /*!< FIFO mode disable */
#define UART_FIFOMODE_ENABLE         USART_CR1_FIFOEN  /*!< FIFO mode enable  */
/**
  * @}
  */

/** @defgroup UARTEx_TXFIFO_threshold_level UARTEx TXFIFO threshold level
  * @{
  */
#define UART_TXFIFO_THRESHOLD_1_8   0x00000000U                               /*!< TX FIFO reaches 1/8 of its depth */
#define UART_TXFIFO_THRESHOLD_1_4   USART_CR3_TXFTCFG_0                       /*!< TX FIFO reaches 1/4 of its depth */
#define UART_TXFIFO_THRESHOLD_1_2   USART_CR3_TXFTCFG_1                       /*!< TX FIFO reaches 1/2 of its depth */
#define UART_TXFIFO_THRESHOLD_3_4   (USART_CR3_TXFTCFG_0|USART_CR3_TXFTCFG_1) /*!< TX FIFO reaches 3/4 of its depth */
#define UART_TXFIFO_THRESHOLD_7_8   USART_CR3_TXFTCFG_2                       /*!< TX FIFO reaches 7/8 of its depth */
#define UART_TXFIFO_THRESHOLD_8_8   (USART_CR3_TXFTCFG_2|USART_CR3_TXFTCFG_0) /*!< TX FIFO becomes empty            */
/**
  * @}
  */

/** @defgroup UARTEx_RXFIFO_threshold_level UARTEx RXFIFO threshold level
  * @{
  */
#define UART_RXFIFO_THRESHOLD_1_8   0x00000000U                               /*!< RX FIFO reaches 1/8 of its depth */
#define UART_RXFIFO_THRESHOLD_1_4   USART_CR3_RXFTCFG_0                       /*!< RX FIFO reaches 1/4 of its depth */
#define UART_RXFIFO_THRESHOLD_1_2   USART_CR3_RXFTCFG_1                       /*!< RX FIFO reaches 1/2 of its depth */
#define UART_RXFIFO_THRESHOLD_3_4   (USART_CR3_RXFTCFG_0|USART_CR3_RXFTCFG_1) /*!< RX FIFO reaches 3/4 of its depth */
#define UART_RXFIFO_THRESHOLD_7_8   USART_CR3_RXFTCFG_2                       /*!< RX FIFO reaches 7/8 of its depth */
#define UART_RXFIFO_THRESHOLD_8_8   (USART_CR3_RXFTCFG_2|USART_CR3_RXFTCFG_0) /*!< RX FIFO becomes full             */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup UARTEx_Exported_Functions
  * @{
  */

/** @addtogroup UARTEx_Exported_Functions_Group1
  * @{
  */

/* Peripheral Control functions ***********************************************/
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *huart);

/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup UARTEx_Private_Constants UARTEx Private Constants
  * @{
  */

/* Defines for UARTEx FIFO */
#define IS_UART_FIFO_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2))

/* Defines for UARTEx_TXFIFO_threshold_level */
#define IS_UART_TXFIFO_THRESHOLD(THRESHOLD) (((THRESHOLD) == UART_TXFIFO_THRESHOLD_1_8) || \
                                             ((THRESHOLD) == UART_TXFIFO_THRESHOLD_1_4) || \
                                             ((THRESHOLD) == UART_TXFIFO_THRESHOLD_1_2) || \
                                             ((THRESHOLD) == UART_TXFIFO_THRESHOLD_3_4) || \
                                             ((THRESHOLD) == UART_TXFIFO_THRESHOLD_7_8) || \
                                             ((THRESHOLD) == UART_TXFIFO_THRESHOLD_8_8))

/* Defines for UARTEx_RXFIFO_threshold_level */
#define IS_UART_RXFIFO_THRESHOLD(THRESHOLD) (((THRESHOLD) == UART_RXFIFO_THRESHOLD_1_8) || \
                                             ((THRESHOLD) == UART_RXFIFO_THRESHOLD_1_4) || \
                                             ((THRESHOLD) == UART_RXFIFO_THRESHOLD_1_2) || \
                                             ((THRESHOLD) == UART_RXFIFO_THRESHOLD_3_4) || \
                                             ((THRESHOLD) == UART_RXFIFO_THRESHOLD_7_8) || \
                                             ((THRESHOLD) == UART_RXFIFO_THRESHOLD_8_8))

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup UARTEx_Private_Functions
  * @{
  */
void UARTEx_SetNbDataToProcess(UART_HandleTypeDef *huart);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32G0xx_HAL_UART_EX_H */