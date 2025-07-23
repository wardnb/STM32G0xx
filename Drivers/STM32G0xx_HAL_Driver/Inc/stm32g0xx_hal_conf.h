/**
  ******************************************************************************
  * @file    stm32g0xx_hal_conf.h
  * @brief   HAL configuration file.
  ******************************************************************************
  */

#ifndef __STM32G0xx_HAL_CONF_H
#define __STM32G0xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx.h"

/* Ensure device-specific header is processed before HAL modules */
#ifdef STM32G0B1xx
#include "stm32g0b1xx.h"
#endif

/* Module Selection */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* Oscillator Values adaptation */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

/* Debug configuration */
#ifdef USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

/* Parameter validation macros */
#define IS_TICKFREQ(FREQ) (((FREQ) == HAL_TICK_FREQ_10HZ)  || \
                           ((FREQ) == HAL_TICK_FREQ_100HZ) || \
                           ((FREQ) == HAL_TICK_FREQ_1KHZ))

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/* SysTick IRQ priority */
#define  TICK_INT_PRIORITY            3U /*!< tick interrupt priority */

/* Include module's header file */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32g0xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32g0xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32g0xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32g0xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32g0xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32g0xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* __STM32G0xx_HAL_CONF_H */
