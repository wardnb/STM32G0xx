/**
  ******************************************************************************
  * @file    stm32g0xx_hal_i2c.h
  * @author  MCD Application Team
  * @brief   Header file of I2C HAL module.
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
#ifndef STM32G0xx_HAL_I2C_H
#define STM32G0xx_HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/** @addtogroup STM32G0xx_HAL_Driver
  * @{
  */

/** @addtogroup I2C
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2C_Exported_Types I2C Exported Types
  * @{
  */

/**
  * @brief  I2C Configuration Structure definition
  */
typedef struct
{
  uint32_t Timing;              /*!< Specifies the I2C_TIMINGR_register value.
                                  This parameter calculated by referring to I2C initialization 
                                         section in Reference manual */

  uint32_t OwnAddress1;         /*!< Specifies the first device own address.
                                  This parameter can be a 7-bit or 10-bit address. */

  uint32_t AddressingMode;      /*!< Specifies if 7-bit or 10-bit addressing mode is selected.
                                  This parameter can be a value of @ref I2C_addressing_mode */

  uint32_t DualAddressMode;     /*!< Specifies if dual addressing mode is selected.
                                  This parameter can be a value of @ref I2C_dual_addressing_mode */

  uint32_t OwnAddress2;         /*!< Specifies the second device own address if dual addressing mode is selected
                                  This parameter can be a 7-bit address. */

  uint32_t OwnAddress2Masks;    /*!< Specifies the acknowledge mask address second device own address if dual addressing mode is selected
                                  This parameter can be a value of @ref I2C_own_address2_masks */

  uint32_t GeneralCallMode;     /*!< Specifies if general call mode is selected.
                                  This parameter can be a value of @ref I2C_general_call_addressing_mode */

  uint32_t NoStretchMode;       /*!< Specifies if nostretch mode is selected.
                                  This parameter can be a value of @ref I2C_nostretch_mode */

} I2C_InitTypeDef;

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,   /*!< Peripheral is not yet Initialized         */
  HAL_I2C_STATE_READY             = 0x20U,   /*!< Peripheral Initialized and ready for use  */
  HAL_I2C_STATE_BUSY              = 0x24U,   /*!< An internal process is ongoing             */
  HAL_I2C_STATE_BUSY_TX           = 0x21U,   /*!< Data Transmission process is ongoing       */
  HAL_I2C_STATE_BUSY_RX           = 0x22U,   /*!< Data Reception process is ongoing          */
  HAL_I2C_STATE_LISTEN            = 0x28U,   /*!< Address Listen Mode is ongoing             */
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   /*!< Address Listen Mode and Data Transmission
                                                   process is ongoing                         */
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   /*!< Address Listen Mode and Data Reception
                                                   process is ongoing                         */
  HAL_I2C_STATE_ABORT             = 0x60U,   /*!< Abort user request ongoing                 */
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,   /*!< Timeout state                              */
  HAL_I2C_STATE_ERROR             = 0xE0U    /*!< Error                                      */

} HAL_I2C_StateTypeDef;

/**
  * @brief  HAL Mode structures definition
  */
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,   /*!< No I2C communication on going             */
  HAL_I2C_MODE_MASTER             = 0x10U,   /*!< I2C communication is in Master Mode       */
  HAL_I2C_MODE_SLAVE              = 0x20U,   /*!< I2C communication is in Slave Mode        */
  HAL_I2C_MODE_MEM                = 0x40U    /*!< I2C communication is in Memory Mode       */

} HAL_I2C_ModeTypeDef;

/**
  * @brief  I2C handle Structure definition
  */
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;      /*!< I2C registers base address               */

  I2C_InitTypeDef            Init;           /*!< I2C communication parameters             */

  uint8_t                    *pBuffPtr;      /*!< Pointer to I2C transfer buffer          */

  uint16_t                   XferSize;       /*!< I2C transfer size                        */

  __IO uint16_t              XferCount;      /*!< I2C transfer counter                     */

  __IO uint32_t              XferOptions;    /*!< I2C transfer options                     */

  __IO uint32_t              PreviousState;  /*!< I2C communication Previous state and mode
                                                  context for internal usage               */

  HAL_StatusTypeDef(*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
                                             /*!< I2C transfer IRQ handler function pointer */

  DMA_HandleTypeDef          *hdmatx;        /*!< I2C Tx DMA handle parameters             */

  DMA_HandleTypeDef          *hdmarx;        /*!< I2C Rx DMA handle parameters             */

  HAL_LockTypeDef            Lock;           /*!< I2C locking object                       */

  __IO HAL_I2C_StateTypeDef  State;          /*!< I2C communication state                  */

  __IO HAL_I2C_ModeTypeDef   Mode;           /*!< I2C communication mode                   */

  __IO uint32_t              ErrorCode;      /*!< I2C Error code                           */

  __IO uint32_t              AddrEventCount; /*!< I2C Address Event counter                */
} I2C_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup I2C_Exported_Constants I2C Exported Constants
  * @{
  */

/** @defgroup I2C_addressing_mode I2C addressing mode
  * @{
  */
#define I2C_ADDRESSINGMODE_7BIT         0x00000001U
#define I2C_ADDRESSINGMODE_10BIT        0x00000002U
/**
  * @}
  */

/** @defgroup I2C_dual_addressing_mode I2C dual addressing mode
  * @{
  */
#define I2C_DUALADDRESS_DISABLE         0x00000000U
#define I2C_DUALADDRESS_ENABLE          I2C_OAR2_OA2EN
/**
  * @}
  */

/** @defgroup I2C_own_address2_masks I2C own address2 masks
  * @{
  */
#define I2C_OA2_NOMASK                  ((uint8_t)0x00U)
#define I2C_OA2_MASK01                  ((uint8_t)0x01U)
#define I2C_OA2_MASK02                  ((uint8_t)0x02U)
#define I2C_OA2_MASK03                  ((uint8_t)0x03U)
#define I2C_OA2_MASK04                  ((uint8_t)0x04U)
#define I2C_OA2_MASK05                  ((uint8_t)0x05U)
#define I2C_OA2_MASK06                  ((uint8_t)0x06U)
#define I2C_OA2_MASK07                  ((uint8_t)0x07U)
/**
  * @}
  */

/** @defgroup I2C_general_call_addressing_mode I2C general call addressing mode
  * @{
  */
#define I2C_GENERALCALL_DISABLE         0x00000000U
#define I2C_GENERALCALL_ENABLE          I2C_CR1_GCEN
/**
  * @}
  */

/** @defgroup I2C_nostretch_mode I2C nostretch mode
  * @{
  */
#define I2C_NOSTRETCH_DISABLE           0x00000000U
#define I2C_NOSTRETCH_ENABLE            I2C_CR1_NOSTRETCH
/**
  * @}
  */

/** @defgroup I2C_Memory_Address_Size I2C Memory Address Size
  * @{
  */
#define I2C_MEMADD_SIZE_8BIT            0x00000001U
#define I2C_MEMADD_SIZE_16BIT           0x00000002U
/**
  * @}
  */

/** @defgroup I2C_XferDirection I2C Transfer Direction Master Point of View
  * @{
  */
#define I2C_DIRECTION_TRANSMIT          0x00000000U
#define I2C_DIRECTION_RECEIVE           0x00000001U
/**
  * @}
  */

/** @defgroup I2C_XferOptions_definition I2C Transfer Options
  * @{
  */
#define I2C_FIRST_FRAME                 0x00000001U
#define I2C_FIRST_AND_NEXT_FRAME        0x00000002U
#define I2C_NEXT_FRAME                  0x00000004U
#define I2C_FIRST_AND_LAST_FRAME        0x00000008U
#define I2C_LAST_FRAME                  0x00000010U
#define I2C_LAST_FRAME_NO_STOP          0x00000020U
#define I2C_OTHER_FRAME                 0x00000040U
#define I2C_OTHER_AND_LAST_FRAME        0x00000080U
/**
  * @}
  */

/** @defgroup I2C_Error_Code_definition I2C Error Code definition
  * @{
  */
#define HAL_I2C_ERROR_NONE              0x00000000U    /*!< No error              */
#define HAL_I2C_ERROR_BERR              0x00000001U    /*!< BERR error            */
#define HAL_I2C_ERROR_ARLO              0x00000002U    /*!< ARLO error            */
#define HAL_I2C_ERROR_AF                0x00000004U    /*!< ACKF error            */
#define HAL_I2C_ERROR_OVR               0x00000008U    /*!< OVR error             */
#define HAL_I2C_ERROR_DMA               0x00000010U    /*!< DMA transfer error    */
#define HAL_I2C_ERROR_TIMEOUT           0x00000020U    /*!< Timeout error         */
#define HAL_I2C_ERROR_SIZE              0x00000040U    /*!< Size Management error */
#define HAL_I2C_ERROR_DMA_PARAM         0x00000080U    /*!< DMA Parameter Error   */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/

/** @defgroup I2C_Exported_Macros I2C Exported Macros
  * @{
  */

/** @brief Reset I2C handle state.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
#define __HAL_I2C_RESET_HANDLE_STATE(__HANDLE__)           ((__HANDLE__)->State = HAL_I2C_STATE_RESET)

/** @brief  Enable the specified I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  * @retval None
  */
#define __HAL_I2C_ENABLE_IT(__HANDLE__, __INTERRUPT__)     ((__HANDLE__)->Instance->CR1 |= (__INTERRUPT__))

/** @brief  Disable the specified I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  * @retval None
  */
#define __HAL_I2C_DISABLE_IT(__HANDLE__, __INTERRUPT__)    ((__HANDLE__)->Instance->CR1 &= (~(__INTERRUPT__)))

/** @brief  Check whether the specified I2C interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the I2C interrupt source to check.
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_I2C_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->CR1 & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Check whether the specified I2C flag is set or not.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to check.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_I2C_GET_FLAG(__HANDLE__, __FLAG__) (((((__HANDLE__)->Instance->ISR) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)

/** @brief  Clear the I2C pending flags which are cleared by writing 1 in a specific bit.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to clear.
  * @retval None
  */
#define __HAL_I2C_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->ICR) = (__FLAG__))

/** @brief  Enable the specified I2C peripheral.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
#define __HAL_I2C_ENABLE(__HANDLE__)                            ((__HANDLE__)->Instance->CR1 |= I2C_CR1_PE)

/** @brief  Disable the specified I2C peripheral.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
#define __HAL_I2C_DISABLE(__HANDLE__)                           ((__HANDLE__)->Instance->CR1 &= ~I2C_CR1_PE)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_Exported_Functions
  * @{
  */

/** @addtogroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
/* Initialization and de-initialization functions ****************************/
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* IO operation functions  ***************************************************/
/******* Blocking mode: Polling */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

/******* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

/******* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);

/******* I2C IRQHandler and Callbacks used in non blocking modes (Interrupt and DMA) */
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  * @{
  */
/* Peripheral State, Mode and Error functions  *******************************/
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef  HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t             HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);

/**
  * @}
  */

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

#endif /* STM32G0xx_HAL_I2C_H */