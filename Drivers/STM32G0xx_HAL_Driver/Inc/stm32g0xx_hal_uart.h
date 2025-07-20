/**
  ******************************************************************************
  * @file    stm32g0xx_hal_uart.h
  * @brief   Header file of UART HAL module.
  ******************************************************************************
  */

#ifndef __STM32G0xx_HAL_UART_H
#define __STM32G0xx_HAL_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/* Ensure device types are available by including device header */
#ifndef UART_TypeDef
#if defined(STM32G0B1xx)
#include "stm32g0b1xx.h"
#elif defined(STM32G0xx)
#include "stm32g0xx.h"
#endif
#endif

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief UART Init Structure definition  
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
                                           The baud rate register is calculated using the following formula:
                                           - If OVER8 = 1, BRR = ((2 * fclk) + baudrate) / (2 * baudrate)
                                           - If OVER8 = 0, BRR = (fclk + (baudrate/2)) / baudrate */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode */

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control */

  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                           This parameter can be a value of @ref UART_Over_Sampling */

  uint32_t OneBitSampling;            /*!< Specifies whether a single sample or three samples' majority vote is selected.
                                           This parameter can be a value of @ref UART_OneBit_Sampling */

  uint32_t ClockPrescaler;            /*!< Specifies the prescaler value used to divide the UART clock source.
                                           This parameter can be a value of @ref UART_ClockPrescaler */
} UART_InitTypeDef;

/** 
  * @brief  UART handle Structure definition  
  */  
typedef struct __UART_HandleTypeDef
{
  UART_TypeDef                 *Instance;        /*!< UART registers base address        */

  UART_InitTypeDef             Init;             /*!< UART communication parameters      */

  uint8_t                      *pTxBuffPtr;      /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                     TxXferSize;       /*!< UART Tx Transfer size              */

  __IO uint16_t                TxXferCount;      /*!< UART Tx Transfer Counter           */

  uint8_t                      *pRxBuffPtr;      /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                     RxXferSize;       /*!< UART Rx Transfer size              */

  __IO uint16_t                RxXferCount;      /*!< UART Rx Transfer Counter           */

  HAL_LockTypeDef              Lock;             /*!< Locking object                     */

  __IO HAL_StatusTypeDef       gState;           /*!< UART state information related to global Handle management 
                                                      and also related to Tx operations.
                                                      This parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO HAL_StatusTypeDef       RxState;          /*!< UART state information related to Rx operations.
                                                      This parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO uint32_t                ErrorCode;        /*!< UART Error code                    */

  struct {
    uint32_t AdvFeatureInit;                    /*!< Advanced feature initialization */
  } AdvancedInit;                              /*!< Advanced features initialization structure */

} UART_HandleTypeDef;

/* UART flags */
#define UART_FLAG_TXE                       0x00000080U   /*!< Transmit data register empty flag */
#define UART_FLAG_TC                        0x00000040U   /*!< Transmission complete flag */
#define UART_FLAG_RXNE                      0x00000020U   /*!< Read data register not empty flag */

/* UART interrupts */
#define UART_IT_TXE                         0x00000080U   /*!< Transmit data register empty interrupt */
#define UART_IT_TC                          0x00000040U   /*!< Transmission complete interrupt */
#define UART_IT_RXNE                        0x00000020U   /*!< Read data register not empty interrupt */

/* UART FIFO thresholds */
#define UART_TXFIFO_THRESHOLD_1_8           0x00000000U   /*!< TXFIFO reaches 1/8 of its depth */
#define UART_RXFIFO_THRESHOLD_1_8           0x00000000U   /*!< RXFIFO reaches 1/8 of its depth */

/* UART prescaler */
#define UART_PRESCALER_DIV1                 0x00000000U   /*!< fclk_pres = fclk */

/* UART hardware flow control */
#define UART_HWCONTROL_NONE                 0x00000000U   /*!< No hardware control */
#define UART_HWCONTROL_RTS                  0x00000100U   /*!< Request To Send */
#define UART_HWCONTROL_CTS                  0x00000200U   /*!< Clear To Send */
#define UART_HWCONTROL_RTS_CTS              0x00000300U   /*!< Request and Clear To Send */

/* UART oversampling */
#define UART_OVERSAMPLING_16                0x00000000U   /*!< Oversampling by 16 */
#define UART_OVERSAMPLING_8                 0x00008000U   /*!< Oversampling by 8 */

/* UART one bit sampling */
#define UART_ONE_BIT_SAMPLE_DISABLE         0x00000000U   /*!< One-bit sampling disable */
#define UART_ONE_BIT_SAMPLE_ENABLE          0x00000800U   /*!< One-bit sampling enable */

/* UART word length */
#define UART_WORDLENGTH_8B                  0x00000000U   /*!< 8-bit long UART frame */
#define UART_WORDLENGTH_9B                  0x00001000U   /*!< 9-bit long UART frame */

/* UART stop bits */
#define UART_STOPBITS_1                     0x00000000U   /*!< UART 1 stop bit */
#define UART_STOPBITS_2                     0x00002000U   /*!< UART 2 stop bits */

/* UART parity */
#define UART_PARITY_NONE                    0x00000000U   /*!< No parity */
#define UART_PARITY_EVEN                    0x00000400U   /*!< Even parity */
#define UART_PARITY_ODD                     0x00000600U   /*!< Odd parity */

/* UART mode */
#define UART_MODE_RX                        0x00000004U   /*!< RX mode */
#define UART_MODE_TX                        0x00000008U   /*!< TX mode */
#define UART_MODE_TX_RX                     0x0000000CU   /*!< RX and TX mode */

/* UART advanced features */
#define UART_ADVFEATURE_NO_INIT             0x00000000U   /*!< No advanced feature initialization */

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->ISR & (__FLAG__)) == (__FLAG__))
#define __HAL_UART_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->ICR = (__FLAG__))
#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->CR1 |= (__INTERRUPT__))
#define __HAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CR1 &= ~(__INTERRUPT__))

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* UART Extended functions */
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G0xx_HAL_UART_H */