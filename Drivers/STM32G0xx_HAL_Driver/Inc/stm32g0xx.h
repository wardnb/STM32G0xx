/**
  ******************************************************************************
  * @file    stm32g0xx.h
  * @brief   CMSIS STM32G0xx Device Peripheral Access Layer Header File.
  ******************************************************************************
  */

#ifndef __STM32G0xx_H
#define __STM32G0xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32g0xx
  * @{
  */

#if defined(STM32G0B1xx)
  #include "stm32g0b1xx.h"
#elif defined(STM32G0B0xx)
  #include "stm32g0b0xx.h"
#elif defined(STM32G0C1xx)
  #include "stm32g0c1xx.h"
#elif defined(STM32G061xx)
  #include "stm32g061xx.h"
#elif defined(STM32G071xx)
  #include "stm32g071xx.h"
#elif defined(STM32G081xx)
  #include "stm32g081xx.h"
#else
 #error "Please select first the target STM32G0xx device used in your application (in stm32g0xx.h file)"
#endif

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32G0xx_H */
