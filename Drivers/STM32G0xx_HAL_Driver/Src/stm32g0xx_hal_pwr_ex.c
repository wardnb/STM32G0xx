/**
  ******************************************************************************
  * @file    stm32g0xx_hal_pwr_ex.c
  * @author  MCD Application Team
  * @brief   Extended PWR HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Power Controller (PWR) peripheral:
  *           + Extended Initialization and de-initialization functions
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

/** @defgroup PWREx PWREx
  * @brief PWR Extended HAL module driver
  * @{
  */

#ifdef HAL_PWR_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup PWREx_Exported_Functions PWREx Exported Functions
  * @{
  */

/** @defgroup PWREx_Exported_Functions_Group1 Extended Peripheral Control functions
  * @brief  Extended Peripheral Control functions
  *
@verbatim
 ===============================================================================
              ##### Extended Peripheral Control functions #####
 ===============================================================================
    [..]

@endverbatim
  * @{
  */

/**
  * @brief Configure the main internal regulator output voltage.
  * @param  VoltageScaling specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption.
  *          This parameter can be one of the following values:
  *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE1 Regulator voltage output range 1 mode
  *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE2 Regulator voltage output range 2 mode
  * @note  When moving from Range 2 to Range 1, the software must ensure that the
  *        system frequency is below 16 MHz before calling HAL_PWREx_ControlVoltageScaling().
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling)
{
  /* Check the parameters */
  assert_param(IS_PWR_VOLTAGE_SCALING_RANGE(VoltageScaling));

  /* STM32G0 simplified implementation - voltage scaling is automatic */
  /* This function is provided for compatibility but doesn't need to do anything */
  
  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_PWR_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */