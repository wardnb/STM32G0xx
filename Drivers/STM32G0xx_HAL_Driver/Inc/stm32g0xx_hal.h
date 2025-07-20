/**
  ******************************************************************************
  * @file    stm32g0xx_hal.h
  * @brief   This file contains all the functions prototypes for the HAL 
  *          module driver.
  ******************************************************************************
  */

#ifndef __STM32G0xx_HAL_H
#define __STM32G0xx_HAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_conf.h"

/* Ensure device header is included for TypeDef definitions */
#include "stm32g0xx.h"

/* Standard includes */
#include <stddef.h>
#include <stdint.h>

/* HAL version definition */
#define __STM32G0_HAL_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32G0_HAL_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __STM32G0_HAL_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __STM32G0_HAL_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32G0_HAL_VERSION         ((__STM32G0_HAL_VERSION_MAIN << 24U)\
                                      |(__STM32G0_HAL_VERSION_SUB1 << 16U)\
                                      |(__STM32G0_HAL_VERSION_SUB2 << 8U )\
                                      |(__STM32G0_HAL_VERSION_RC))

/* HAL module includes */
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32g0xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32g0xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32g0xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32g0xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

/* Exported types ------------------------------------------------------------*/
/* HAL_StatusTypeDef and HAL_LockTypeDef are defined in stm32g0xx_hal_def.h */

typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

/* Exported constants --------------------------------------------------------*/
#define HAL_MAX_DELAY      0xFFFFFFFFU
#define HAL_TIMEOUT_VALUE  5000U
#define HSI_TIMEOUT_VALUE  2U       /* 2 ms (minimum Tick + 1) */
#define HSE_TIMEOUT_VALUE  100U     /* 100 ms */
#define PLL_TIMEOUT_VALUE  2U       /* 2 ms (minimum Tick + 1) */
#define CLOCKSWITCH_TIMEOUT_VALUE  5000U  /* 5 s    */

/* Tick frequency */
extern HAL_TickFreqTypeDef uwTickFreq;
extern uint32_t uwTickPrio;

/* SysTick counter */
extern __IO uint32_t uwTick;

/* Core clock frequency */
extern uint32_t SystemCoreClock;

/* Exported macros -----------------------------------------------------------*/
/* UNUSED is defined in stm32g0xx_hal_def.h */
#define __IO volatile
#define __IM volatile const
#define __OM volatile

/* Lock and unlock macros */
#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;     \
                                    }                                      \
                                  }while (0U)

#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0U)

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G0xx_HAL_H */
