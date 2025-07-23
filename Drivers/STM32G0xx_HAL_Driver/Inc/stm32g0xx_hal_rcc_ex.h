/**
  ******************************************************************************
  * @file    stm32g0xx_hal_rcc_ex.h
  * @author  MCD Application Team
  * @brief   Header file of RCC HAL Extended module.
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
#ifndef STM32G0xx_HAL_RCC_EX_H
#define STM32G0xx_HAL_RCC_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/** @addtogroup STM32G0xx_HAL_Driver
  * @{
  */

/** @addtogroup RCCEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Constants RCCEx Exported Constants
  * @{
  */

/** @defgroup RCCEx_Periph_Clock_Selection Periph Clock Selection
  * @{
  */
#define RCC_PERIPHCLK_USART1           0x00000001U
#define RCC_PERIPHCLK_USART2           0x00000002U
#define RCC_PERIPHCLK_I2C1             0x00000020U
/**
  * @}
  */

/** @defgroup RCCEx_USART1_Clock_Source USART1 Clock Source
  * @{
  */
#define RCC_USART1CLKSOURCE_PCLK1      0x00000000U
#define RCC_USART1CLKSOURCE_SYSCLK     RCC_CCIPR_USART1SEL_0
#define RCC_USART1CLKSOURCE_HSI        RCC_CCIPR_USART1SEL_1
#define RCC_USART1CLKSOURCE_LSE        RCC_CCIPR_USART1SEL
/**
  * @}
  */

/** @defgroup RCCEx_USART2_Clock_Source USART2 Clock Source
  * @{
  */
#define RCC_USART2CLKSOURCE_PCLK1      0x00000000U
#define RCC_USART2CLKSOURCE_SYSCLK     RCC_CCIPR_USART2SEL_0
#define RCC_USART2CLKSOURCE_HSI        RCC_CCIPR_USART2SEL_1
#define RCC_USART2CLKSOURCE_LSE        RCC_CCIPR_USART2SEL
/**
  * @}
  */

/** @defgroup RCCEx_I2C1_Clock_Source I2C1 Clock Source
  * @{
  */
#define RCC_I2C1CLKSOURCE_PCLK1        0x00000000U
#define RCC_I2C1CLKSOURCE_SYSCLK       RCC_CCIPR_I2C1SEL_0
#define RCC_I2C1CLKSOURCE_HSI          RCC_CCIPR_I2C1SEL_1
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Macros RCCEx Exported Macros
  * @{
  */

/** @brief  Macro to configure the USART1 clock (USART1CLK).
  * @param  __USART1_CLKSOURCE__ specifies the USART1 clock source.
  */
#define __HAL_RCC_USART1_CONFIG(__USART1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART1SEL, (uint32_t)(__USART1_CLKSOURCE__))

/** @brief  Macro to get the USART1 clock source.
  * @retval The clock source can be one of the following values:
  */
#define __HAL_RCC_GET_USART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL)))

/** @brief  Macro to configure the USART2 clock (USART2CLK).
  * @param  __USART2_CLKSOURCE__ specifies the USART2 clock source.
  */
#define __HAL_RCC_USART2_CONFIG(__USART2_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART2SEL, (uint32_t)(__USART2_CLKSOURCE__))

/** @brief  Macro to get the USART2 clock source.
  * @retval The clock source can be one of the following values:
  */
#define __HAL_RCC_GET_USART2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART2SEL)))

/** @brief  Macro to configure the I2C1 clock (I2C1CLK).
  * @param  __I2C1_CLKSOURCE__ specifies the I2C1 clock source.
  */
#define __HAL_RCC_I2C1_CONFIG(__I2C1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C1SEL, (uint32_t)(__I2C1_CLKSOURCE__))

/** @brief  Macro to get the I2C1 clock source.
  * @retval The clock source can be one of the following values:
  */
#define __HAL_RCC_GET_I2C1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_I2C1SEL)))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCCEx_Exported_Functions
  * @{
  */

/** @addtogroup RCCEx_Exported_Functions_Group1
  * @{
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RCCEx_Private_Macros RCCEx Private Macros
  * @{
  */
#define IS_RCC_PERIPHCLOCK(SELECTION) \
  (((SELECTION) & RCC_PERIPHCLK_USART1)  || \
   ((SELECTION) & RCC_PERIPHCLK_USART2)  || \
   ((SELECTION) & RCC_PERIPHCLK_I2C1))

#define IS_RCC_USART1CLKSOURCE(SOURCE) \
  (((SOURCE) == RCC_USART1CLKSOURCE_PCLK1)  || \
   ((SOURCE) == RCC_USART1CLKSOURCE_SYSCLK) || \
   ((SOURCE) == RCC_USART1CLKSOURCE_LSE)    || \
   ((SOURCE) == RCC_USART1CLKSOURCE_HSI))

#define IS_RCC_USART2CLKSOURCE(SOURCE) \
  (((SOURCE) == RCC_USART2CLKSOURCE_PCLK1)  || \
   ((SOURCE) == RCC_USART2CLKSOURCE_SYSCLK) || \
   ((SOURCE) == RCC_USART2CLKSOURCE_LSE)    || \
   ((SOURCE) == RCC_USART2CLKSOURCE_HSI))

#define IS_RCC_I2C1CLKSOURCE(SOURCE) \
  (((SOURCE) == RCC_I2C1CLKSOURCE_PCLK1)   || \
   ((SOURCE) == RCC_I2C1CLKSOURCE_SYSCLK)  || \
   ((SOURCE) == RCC_I2C1CLKSOURCE_HSI))

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

#endif /* STM32G0xx_HAL_RCC_EX_H */