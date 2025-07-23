/**
  ******************************************************************************
  * @file    stm32g0xx_hal_rcc_ex.c
  * @author  MCD Application Team
  * @brief   Extended RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCC extension peripheral:
  *           + Extended Peripheral Control functions
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

/** @defgroup RCCEx RCCEx
  * @brief RCC Extended HAL module driver
  * @{
  */

#ifdef HAL_RCC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  * @{
  */

/** @defgroup RCCEx_Exported_Functions_Group1 Extended Peripheral Control functions
  * @brief  Extended Peripheral Control functions
  *
@verbatim
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks
    frequencies.

@endverbatim
  * @{
  */

/**
  * @brief  Return the peripheral clock frequency
  * @param  PeriphClk  Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_PERIPHCLK_USART1  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART2  USART2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C1    I2C1 peripheral clock
  * @retval Frequency in Hz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  uint32_t frequency = 0U;
  uint32_t pllvco = 0U;
  uint32_t plln = 0U;
  uint32_t pllp = 0U;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClk));

  if (PeriphClk == RCC_PERIPHCLK_USART1)
  {
    /* Get USART1 clock source config */
    switch (__HAL_RCC_GET_USART1_SOURCE())
    {
      case RCC_USART1CLKSOURCE_PCLK1:
        frequency = HAL_RCC_GetPCLK1Freq();
        break;

      case RCC_USART1CLKSOURCE_SYSCLK:
        frequency = HAL_RCC_GetSysClockFreq();
        break;

      case RCC_USART1CLKSOURCE_HSI:
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
        {
          frequency = HSI_VALUE;
        }
        break;

      case RCC_USART1CLKSOURCE_LSE:
        if (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
        {
          frequency = LSE_VALUE;
        }
        break;

      default:
        break;
    }
  }
  else if (PeriphClk == RCC_PERIPHCLK_USART2)
  {
    /* Get USART2 clock source config */
    switch (__HAL_RCC_GET_USART2_SOURCE())
    {
      case RCC_USART2CLKSOURCE_PCLK1:
        frequency = HAL_RCC_GetPCLK1Freq();
        break;

      case RCC_USART2CLKSOURCE_SYSCLK:
        frequency = HAL_RCC_GetSysClockFreq();
        break;

      case RCC_USART2CLKSOURCE_HSI:
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
        {
          frequency = HSI_VALUE;
        }
        break;

      case RCC_USART2CLKSOURCE_LSE:
        if (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY))
        {
          frequency = LSE_VALUE;
        }
        break;

      default:
        break;
    }
  }
  else if (PeriphClk == RCC_PERIPHCLK_I2C1)
  {
    /* Get I2C1 clock source config */
    switch (__HAL_RCC_GET_I2C1_SOURCE())
    {
      case RCC_I2C1CLKSOURCE_PCLK1:
        frequency = HAL_RCC_GetPCLK1Freq();
        break;

      case RCC_I2C1CLKSOURCE_SYSCLK:
        frequency = HAL_RCC_GetSysClockFreq();
        break;

      case RCC_I2C1CLKSOURCE_HSI:
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY))
        {
          frequency = HSI_VALUE;
        }
        break;

      default:
        break;
    }
  }
  else
  {
    /* Other peripherals not yet supported */
  }

  return frequency;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_RCC_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */