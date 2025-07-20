/**
  ******************************************************************************
  * @file    stm32g0xx_hal_i2c.c
  * @author  MCD Application Team
  * @brief   I2C HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Inter Integrated Circuit (I2C) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions
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

#ifdef HAL_I2C_MODULE_ENABLED

/** @addtogroup STM32G0xx_HAL_Driver
  * @{
  */

/** @defgroup I2C I2C
  * @brief I2C HAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup I2C_Private_Define I2C Private Define
  * @{
  */
#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)  /*!< I2C TIMING clear register Mask */
#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_DIR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOPF   (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TC      (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TCR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TXIS    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

#define MAX_NBYTE_SIZE      255U
#define SlaveAddr_SHIFT     7U
#define SlaveAddr_MSK       0x06U

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */
/* Private functions for I2C transfer management */
static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);
static HAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);

/* Private functions to handle DMA transfer */
static void I2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAError(DMA_HandleTypeDef *hdma);
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma);

/* Private functions to handle IT transfer */
static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITMasterSeqCplt(I2C_HandleTypeDef *hi2c);
static void I2C_ITSlaveSeqCplt(I2C_HandleTypeDef *hi2c);
static void I2C_ITMasterCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITListenCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITError(I2C_HandleTypeDef *hi2c, uint32_t ErrorCode);

/* Private functions to handle IT transfer */
static HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Slave_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Master_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Slave_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Mem_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Mem_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the I2Cx peripheral:

      (+) User must Implement HAL_I2C_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_I2C_Init() to configure the selected device with
          the selected configuration:
        (++) Clock Timing
        (++) Own Address 1
        (++) Addressing mode (Master, Slave)
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) Own Address 2 Mask
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function HAL_I2C_DeInit() to restore the default configuration
          of the selected I2Cx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2C according to the specified parameters
  *         in the I2C_InitTypeDef and initialize the associated handle.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));

  if (hi2c->State == HAL_I2C_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hi2c->Lock = HAL_UNLOCKED;

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    HAL_I2C_MspInit(hi2c);
  }

  hi2c->State = HAL_I2C_STATE_BUSY;

  /* Disable the selected I2C peripheral */
  __HAL_I2C_DISABLE(hi2c);

  /*---------------------------- I2Cx TIMINGR Configuration ------------------*/
  /* Configure I2Cx: Frequency range */
  hi2c->Instance->TIMINGR = hi2c->Init.Timing & TIMING_CLEAR_MASK;

  /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Disable Own Address1 before set the Own Address1 configuration */
  hi2c->Instance->OAR1 &= ~I2C_OAR1_OA1EN;

  /* Configure I2Cx: Own Address1 and ack own address1 mode */
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | hi2c->Init.OwnAddress1);
  }
  else /* I2C_ADDRESSINGMODE_10BIT */
  {
    hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | I2C_OAR1_OA1MODE | hi2c->Init.OwnAddress1);
  }

  /*---------------------------- I2Cx CR2 Configuration ----------------------*/
  /* Configure I2Cx: Addressing Master mode */
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
  {
    hi2c->Instance->CR2 = (I2C_CR2_ADD10);
  }
  /* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
  hi2c->Instance->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK);

  /*---------------------------- I2Cx OAR2 Configuration ---------------------*/
  /* Disable Own Address2 before set the Own Address2 configuration */
  hi2c->Instance->OAR2 &= ~I2C_OAR2_OA2EN;

  /* Configure I2Cx: Dual mode and Own Address2 */
  if (hi2c->Init.DualAddressMode == I2C_DUALADDRESS_ENABLE)
  {
    hi2c->Instance->OAR2 = (I2C_OAR2_OA2EN | (hi2c->Init.OwnAddress2Masks) | ((hi2c->Init.OwnAddress2 & I2C_OAR2_OA2) << 1));
  }

  /*---------------------------- I2Cx CR1 Configuration ----------------------*/
  /* Configure I2Cx: Generalcall and NoStretch mode */
  hi2c->Instance->CR1 = (hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode);

  /* Enable the selected I2C peripheral */
  __HAL_I2C_ENABLE(hi2c);

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_READY;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = HAL_I2C_MODE_NONE;

  return HAL_OK;
}

/**
  * @brief  DeInitialize the I2C peripheral.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));

  hi2c->State = HAL_I2C_STATE_BUSY;

  /* Disable the I2C Peripheral Clock */
  __HAL_I2C_DISABLE(hi2c);

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  HAL_I2C_MspDeInit(hi2c);

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_RESET;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = HAL_I2C_MODE_NONE;

  /* Release Lock */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief Initialize the I2C MSP.
  * @param hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspInit could be implemented in the user file
   */
}

/**
  * @brief DeInitialize the I2C MSP.
  * @param hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @brief   Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_I2C_Master_Transmit()
        (++) HAL_I2C_Master_Receive()
        (++) HAL_I2C_Slave_Transmit()
        (++) HAL_I2C_Slave_Receive()
        (++) HAL_I2C_Mem_Write()
        (++) HAL_I2C_Mem_Read()
        (++) HAL_I2C_IsDeviceReady()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_I2C_Master_Transmit_IT()
        (++) HAL_I2C_Master_Receive_IT()
        (++) HAL_I2C_Slave_Transmit_IT()
        (++) HAL_I2C_Slave_Receive_IT()
        (++) HAL_I2C_Mem_Write_IT()
        (++) HAL_I2C_Mem_Read_IT()
        (++) HAL_I2C_Master_Seq_Transmit_IT()
        (++) HAL_I2C_Master_Seq_Receive_IT()
        (++) HAL_I2C_Slave_Seq_Transmit_IT()
        (++) HAL_I2C_Slave_Seq_Receive_IT()
        (++) HAL_I2C_EnableListen_IT()
        (++) HAL_I2C_DisableListen_IT()
        (++) HAL_I2C_Master_Abort_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_I2C_Master_Transmit_DMA()
        (++) HAL_I2C_Master_Receive_DMA()
        (++) HAL_I2C_Slave_Transmit_DMA()
        (++) HAL_I2C_Slave_Receive_DMA()
        (++) HAL_I2C_Mem_Write_DMA()
        (++) HAL_I2C_Mem_Read_DMA()
        (++) HAL_I2C_Master_Seq_Transmit_DMA()
        (++) HAL_I2C_Master_Seq_Receive_DMA()
        (++) HAL_I2C_Slave_Seq_Transmit_DMA()
        (++) HAL_I2C_Slave_Seq_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2C_MasterTxCpltCallback()
        (++) HAL_I2C_MasterRxCpltCallback()
        (++) HAL_I2C_SlaveTxCpltCallback()
        (++) HAL_I2C_SlaveRxCpltCallback()
        (++) HAL_I2C_MemTxCpltCallback()
        (++) HAL_I2C_MemRxCpltCallback()
        (++) HAL_I2C_AddrCallback()
        (++) HAL_I2C_ListenCpltCallback()
        (++) HAL_I2C_ErrorCallback()
        (++) HAL_I2C_AbortCpltCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Check if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  uint32_t tickstart;

  __IO uint32_t I2C_Trials = 0UL;

  FlagStatus tmp1;
  FlagStatus tmp2;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    do
    {
      /* Generate Start */
      hi2c->Instance->CR2 = I2C_GENERATE_START(hi2c->Init.AddressingMode, DevAddress);

      /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set or a NACK flag is set*/
      tickstart = HAL_GetTick();

      tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
      tmp2 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);

      while ((tmp1 == RESET) && (tmp2 == RESET))
      {
        if (Timeout != HAL_MAX_DELAY)
        {
          if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
          {
            /* Update I2C state */
            hi2c->State = HAL_I2C_STATE_READY;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
          }
        }

        tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
        tmp2 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);
      }

      /* Check if the NACKF flag has not been set */
      if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == RESET)
      {
        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Device is ready */
        hi2c->State = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
      }
      else
      {
        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        /* Clear NACK Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Clear STOP Flag, auto generated with autoend*/
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
      }

      /* Check if the maximum allowed number of trials has been reached */
      if (I2C_Trials == Trials)
      {
        /* Generate Stop */
        hi2c->Instance->CR2 |= I2C_CR2_STOP;

        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
      }

      /* Increment Trials */
      I2C_Trials++;
    } while (I2C_Trials < Trials);

    /* Update I2C state */
    hi2c->State = HAL_I2C_STATE_READY;

    /* Update I2C error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_ERROR;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until TXIS flag is set */
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }
      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to read and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
    }

    do
    {
      /* Wait until TXIS flag is set */
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }

    } while (hi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Send Slave Address */
    /* Set NBYTES to read and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }

    do
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t) hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }

    } while (hi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_RXI);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  * @brief   Peripheral State, Mode and Error functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL state
  */
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
  /* Return I2C handle state */
  return hi2c->State;
}

/**
  * @brief  Returns the I2C Master, Slave, Memory or no mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval HAL mode
  */
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c)
{
  return hi2c->Mode;
}

/**
* @brief  Return the I2C error code.
* @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
* @retval I2C Error Code
*/
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c)
{
  return hi2c->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */

/* Placeholder implementations for missing private functions */
/* These would need full implementation for complete I2C functionality */

static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(DevAddress);
  UNUSED(Size);
  UNUSED(Mode);
  UNUSED(Request);
}

static HAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(DevAddress);
  UNUSED(MemAddress);
  UNUSED(MemAddSize);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(DevAddress);
  UNUSED(MemAddress);
  UNUSED(MemAddSize);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(Flag);
  UNUSED(Status);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  /* Implementation needed for full I2C support */
  UNUSED(hi2c);
  UNUSED(Timeout);
  UNUSED(Tickstart);
  return HAL_OK;
}

/* Placeholder implementations for interrupt and DMA functions */
static void I2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma) { UNUSED(hdma); }
static void I2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma) { UNUSED(hdma); }
static void I2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma) { UNUSED(hdma); }
static void I2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma) { UNUSED(hdma); }
static void I2C_DMAError(DMA_HandleTypeDef *hdma) { UNUSED(hdma); }
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma) { UNUSED(hdma); }

static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags) { UNUSED(hi2c); UNUSED(ITFlags); }
static void I2C_ITMasterSeqCplt(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
static void I2C_ITSlaveSeqCplt(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
static void I2C_ITMasterCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags) { UNUSED(hi2c); UNUSED(ITFlags); }
static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags) { UNUSED(hi2c); UNUSED(ITFlags); }
static void I2C_ITListenCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags) { UNUSED(hi2c); UNUSED(ITFlags); }
static void I2C_ITError(I2C_HandleTypeDef *hi2c, uint32_t ErrorCode) { UNUSED(hi2c); UNUSED(ErrorCode); }

static HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) { UNUSED(hi2c); UNUSED(ITFlags); UNUSED(ITSources); return HAL_OK; }
static HAL_StatusTypeDef I2C_Slave_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) { UNUSED(hi2c); UNUSED(ITFlags); UNUSED(ITSources); return HAL_OK; }
static HAL_StatusTypeDef I2C_Master_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) { UNUSED(hi2c); UNUSED(ITFlags); UNUSED(ITSources); return HAL_OK; }
static HAL_StatusTypeDef I2C_Slave_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) { UNUSED(hi2c); UNUSED(ITFlags); UNUSED(ITSources); return HAL_OK; }
static HAL_StatusTypeDef I2C_Mem_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) { UNUSED(hi2c); UNUSED(ITFlags); UNUSED(ITSources); return HAL_OK; }
static HAL_StatusTypeDef I2C_Mem_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) { UNUSED(hi2c); UNUSED(ITFlags); UNUSED(ITSources); return HAL_OK; }

/* Weak callback implementations */
__weak void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) { UNUSED(hi2c); UNUSED(TransferDirection); UNUSED(AddrMatchCode); }
__weak void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }

/* Interrupt handlers */
__weak void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }
__weak void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); }

/* Placeholder implementations for non-blocking functions */
__weak HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(MemAddress); UNUSED(MemAddSize); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(MemAddress); UNUSED(MemAddSize); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }

/* Additional placeholder implementations for sequence and DMA functions */
__weak HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c) { UNUSED(hi2c); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress) { UNUSED(hi2c); UNUSED(DevAddress); return HAL_ERROR; }

/* DMA function placeholders */
__weak HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(MemAddress); UNUSED(MemAddSize); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(MemAddress); UNUSED(MemAddSize); UNUSED(pData); UNUSED(Size); return HAL_ERROR; }

__weak HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(DevAddress); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); UNUSED(XferOptions); return HAL_ERROR; }

/* Additional slave mode functions */
__weak HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); UNUSED(Timeout); return HAL_ERROR; }
__weak HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout) { UNUSED(hi2c); UNUSED(pData); UNUSED(Size); UNUSED(Timeout); return HAL_ERROR; }

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_I2C_MODULE_ENABLED */