/**
  ******************************************************************************
  * @file    stm32g0xx_hal_pwr.h
  * @author  MCD Application Team
  * @brief   Header file of PWR HAL module.
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
#ifndef STM32G0xx_HAL_PWR_H
#define STM32G0xx_HAL_PWR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/** @addtogroup STM32G0xx_HAL_Driver
  * @{
  */

/** @addtogroup PWR
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_Regulator_Voltage_Scale PWR Regulator Voltage Scale
  * @{
  */
#define PWR_REGULATOR_VOLTAGE_SCALE1        0x00000200U      /*!< Voltage scaling range 1 */
#define PWR_REGULATOR_VOLTAGE_SCALE2        0x00000400U      /*!< Voltage scaling range 2 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWR_Exported_Macros PWR Exported Macros
  * @{
  */

/** @brief  Check PWR flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_PWR_GET_FLAG(__FLAG__)       ((PWR->SR1 & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the PWR's pending flags.
  * @param  __FLAG__ specifies the flag to clear.
  */
#define __HAL_PWR_CLEAR_FLAG(__FLAG__)      (PWR->SCR = (__FLAG__))

/**
  * @}
  */

/* Include PWR HAL Extended module */
#include "stm32g0xx_hal_pwr_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */

/** @addtogroup PWR_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
void HAL_PWR_DeInit(void);

/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/** @defgroup PWR_Private_Constants PWR Private Constants
  * @{
  */
/* Defines used for PWR features parameter check */
#define IS_PWR_VOLTAGE_SCALING_RANGE(RANGE) (((RANGE) == PWR_REGULATOR_VOLTAGE_SCALE1) || \
                                             ((RANGE) == PWR_REGULATOR_VOLTAGE_SCALE2))

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PWR_Private_Macros PWR Private Macros
  * @{
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

#endif /* STM32G0xx_HAL_PWR_H */