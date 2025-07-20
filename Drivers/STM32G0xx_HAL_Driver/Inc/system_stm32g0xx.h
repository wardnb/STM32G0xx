/**
  ******************************************************************************
  * @file    system_stm32g0xx.h
  * @brief   CMSIS Cortex-M0+ Device System Source File for STM32G0xx devices.
  ******************************************************************************
  */

#ifndef __SYSTEM_STM32G0XX_H
#define __SYSTEM_STM32G0XX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/** @addtogroup stm32g0xx_system
  * @{
  */

/** @addtogroup STM32G0xx_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup STM32G0xx_System_Exported_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
extern uint32_t SystemCoreClock;          /*\!< System Clock Frequency (Core Clock) */

extern const uint8_t  AHBPrescTable[16];  /*\!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[8];   /*\!< APB prescalers table values */

/**
  * @}
  */

/** @addtogroup STM32G0xx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G0xx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G0xx_System_Exported_Functions
  * @{
  */

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32G0XX_H */
