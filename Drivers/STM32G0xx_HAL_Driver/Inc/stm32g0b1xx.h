/**
  ******************************************************************************
  * @file    stm32g0b1xx.h
  * @brief   CMSIS STM32G0B1xx Device Peripheral Access Layer Header File.
  ******************************************************************************
  */

#ifndef __STM32G0B1xx_H
#define __STM32G0B1xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "core_cm0plus.h"
#include "system_stm32g0xx.h"
#include <stdint.h>

/* Configuration of the Cortex-M0+ Processor and Core Peripherals */
#define __CM0PLUS_REV             0x0001U  /*\!< Core revision r0p1                             */
#define __MPU_PRESENT             1U       /*\!< STM32G0xx provides an MPU                     */
#define __VTOR_PRESENT            1U       /*\!< Vector Table Office Register present           */
#define __NVIC_PRIO_BITS          2U       /*\!< Number of Bits used for Priority Levels       */
#define __Vendor_SysTickConfig    0U       /*\!< Set to 1 if different SysTick Config is used  */

/* Interrupt Number Definition */
typedef enum
{
/******  Cortex-M0+ Processor Exceptions Numbers *******************************/
  NonMaskableInt_IRQn         = -14,    /*\!< 2 Cortex-M0+ Non Maskable Interrupt            */
  HardFault_IRQn              = -13,    /*\!< 3 Cortex-M0+ Hard Fault Interrupt              */
  SVCall_IRQn                 = -5,     /*\!< 11 Cortex-M0+ SV Call Interrupt                */
  PendSV_IRQn                 = -2,     /*\!< 14 Cortex-M0+ Pend SV Interrupt                */
  SysTick_IRQn                = -1,     /*\!< 15 Cortex-M0+ System Tick Interrupt            */

/******  STM32G0B1xx specific Interrupt Numbers ********************************/
  WWDG_IRQn                   = 0,      /*\!< Window WatchDog Interrupt                       */
  PVD_IRQn                    = 1,      /*\!< PVD through EXTI Line detection Interrupt      */
  RTC_TAMP_IRQn               = 2,      /*\!< RTC tamper and TimeStamp Interrupts            */
  FLASH_IRQn                  = 3,      /*\!< FLASH global Interrupt                          */
  RCC_IRQn                    = 4,      /*\!< RCC global Interrupt                            */
  EXTI0_1_IRQn                = 5,      /*\!< EXTI Line0 and Line1 Interrupts                */
  EXTI2_3_IRQn                = 6,      /*\!< EXTI Line2 and Line3 Interrupts                */
  EXTI4_15_IRQn               = 7,      /*\!< EXTI Line4 to Line15 Interrupts                */
  USB_IRQn                    = 8,      /*\!< USB Interrupt                                   */
  DMA1_Channel1_IRQn          = 9,      /*\!< DMA1 Channel 1 Interrupt                        */
  DMA1_Channel2_3_IRQn        = 10,     /*\!< DMA1 Channel 2 and Channel 3 Interrupts        */
  DMA1_Ch4_7_DMA2_Ch1_5_IRQn  = 11,     /*\!< DMA1 Channel 4 to 7, DMA2 Channel 1 to 5 Int.  */
  ADC1_IRQn                   = 12,     /*\!< ADC1 Interrupt                                  */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*\!< TIM1 Break, Update, Trigger and Commutation    */
  TIM1_CC_IRQn                = 14,     /*\!< TIM1 Capture Compare Interrupt                  */
  TIM2_IRQn                   = 15,     /*\!< TIM2 Interrupt                                  */
  TIM3_IRQn                   = 16,     /*\!< TIM3 Interrupt                                  */
  TIM6_DAC_LPTIM1_IRQn        = 17,     /*\!< TIM6, DAC and LPTIM1 Interrupts                 */
  TIM7_LPTIM2_IRQn            = 18,     /*\!< TIM7 and LPTIM2 Interrupts                      */
  TIM14_IRQn                  = 19,     /*\!< TIM14 Interrupt                                 */
  TIM15_IRQn                  = 20,     /*\!< TIM15 Interrupt                                 */
  TIM16_IRQn                  = 21,     /*\!< TIM16 Interrupt                                 */
  TIM17_IRQn                  = 22,     /*\!< TIM17 Interrupt                                 */
  I2C1_IRQn                   = 23,     /*\!< I2C1 Interrupt                                  */
  I2C2_IRQn                   = 24,     /*\!< I2C2 Interrupt                                  */
  SPI1_IRQn                   = 25,     /*\!< SPI1 Interrupt                                  */
  SPI2_IRQn                   = 26,     /*\!< SPI2 Interrupt                                  */
  USART1_IRQn                 = 27,     /*\!< USART1 Interrupt                                */
  USART2_IRQn                 = 28,     /*\!< USART2 Interrupt                                */
  USART3_4_5_6_LPUART1_IRQn   = 29,     /*\!< USART3, USART4, USART5, USART6, LPUART1 glb    */
  CEC_IRQn                    = 30,     /*\!< CEC Interrupt                                   */
} IRQn_Type;

/* Peripheral memory map */
#define PERIPH_BASE             0x40000000UL /*\!< Peripheral base address in the alias region */

/* Peripheral_declaration */
#define GPIOA_BASE              (PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE              (PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE              (PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE              (PERIPH_BASE + 0x00000C00UL)
#define GPIOE_BASE              (PERIPH_BASE + 0x00001000UL)
#define GPIOF_BASE              (PERIPH_BASE + 0x00001400UL)

#define RCC_BASE                (PERIPH_BASE + 0x00021000UL)

/* GPIO_TypeDef */
typedef struct
{
  __IO uint32_t MODER;    /*\!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*\!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*\!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*\!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*\!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*\!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*\!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*\!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*\!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;      /*\!< GPIO Bit Reset register,               Address offset: 0x28      */
} GPIO_TypeDef;

/* RCC_TypeDef */
typedef struct
{
  __IO uint32_t CR;         /*\!< RCC clock control register,                                          Address offset: 0x00 */
  __IO uint32_t ICSCR;      /*\!< RCC internal clock sources calibration register,                    Address offset: 0x04 */
  __IO uint32_t CFGR;       /*\!< RCC clock configuration register,                                    Address offset: 0x08 */
  __IO uint32_t PLLCFGR;    /*\!< RCC system PLL configuration register,                              Address offset: 0x0C */
  __IO uint32_t RESERVED0;  /*\!< Reserved,                                                            Address offset: 0x10 */
  __IO uint32_t RESERVED1;  /*\!< Reserved,                                                            Address offset: 0x14 */
  __IO uint32_t CIER;       /*\!< RCC clock interrupt enable register,                                Address offset: 0x18 */
  __IO uint32_t CIFR;       /*\!< RCC clock interrupt flag register,                                  Address offset: 0x1C */
  __IO uint32_t CICR;       /*\!< RCC clock interrupt clear register,                                 Address offset: 0x20 */
  __IO uint32_t IOPRSTR;    /*\!< RCC I/O port reset register,                                        Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*\!< RCC AHB peripherals reset register,                                 Address offset: 0x28 */
  __IO uint32_t APBRSTR1;   /*\!< RCC APB peripherals reset register 1,                               Address offset: 0x2C */
  __IO uint32_t APBRSTR2;   /*\!< RCC APB peripherals reset register 2,                               Address offset: 0x30 */
  __IO uint32_t IOPENR;     /*\!< RCC I/O port clock enable register,                                 Address offset: 0x34 */
  __IO uint32_t AHBENR;     /*\!< RCC AHB peripherals clock enable register,                          Address offset: 0x38 */
  __IO uint32_t APBENR1;    /*\!< RCC APB peripherals clock enable register 1,                        Address offset: 0x3C */
  __IO uint32_t APBENR2;    /*\!< RCC APB peripherals clock enable register 2,                        Address offset: 0x40 */
  __IO uint32_t IOPSMENR;   /*\!< RCC I/O port clocks enable in sleep mode register,                  Address offset: 0x44 */
  __IO uint32_t AHBSMENR;   /*\!< RCC AHB peripheral clocks enable in sleep mode register,            Address offset: 0x48 */
  __IO uint32_t APBSMENR1;  /*\!< RCC APB peripheral clocks enable in sleep mode register 1,          Address offset: 0x4C */
  __IO uint32_t APBSMENR2;  /*\!< RCC APB peripheral clocks enable in sleep mode register 2,          Address offset: 0x50 */
  __IO uint32_t CCIPR;      /*\!< RCC peripherals independent clock configuration register,           Address offset: 0x54 */
  __IO uint32_t CCIPR2;     /*\!< RCC peripherals independent clock configuration register 2,         Address offset: 0x58 */
  __IO uint32_t BDCR;       /*\!< RCC backup domain control register,                                 Address offset: 0x5C */
  __IO uint32_t CSR;        /*\!< RCC clock control & status register,                                Address offset: 0x60 */
} RCC_TypeDef;

/* Peripheral_registers_structures */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

/* Bit definition for GPIO_MODER register */
#define GPIO_MODER_MODE0_Pos           (0U)
#define GPIO_MODER_MODE0_Msk           (0x3UL << GPIO_MODER_MODE0_Pos)          /*\!< 0x00000003 */
#define GPIO_MODER_MODE0               GPIO_MODER_MODE0_Msk

/* Bit definition for GPIO_OTYPER register */
#define GPIO_OTYPER_OT0_Pos            (0U)
#define GPIO_OTYPER_OT0_Msk            (0x1UL << GPIO_OTYPER_OT0_Pos)           /*\!< 0x00000001 */
#define GPIO_OTYPER_OT0                GPIO_OTYPER_OT0_Msk

/* Bit definition for GPIO_OSPEEDR register */
#define GPIO_OSPEEDR_OSPEED0_Pos       (0U)
#define GPIO_OSPEEDR_OSPEED0_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*\!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0           GPIO_OSPEEDR_OSPEED0_Msk

/* Bit definition for GPIO_PUPDR register */
#define GPIO_PUPDR_PUPD0_Pos           (0U)
#define GPIO_PUPDR_PUPD0_Msk           (0x3UL << GPIO_PUPDR_PUPD0_Pos)          /*\!< 0x00000003 */
#define GPIO_PUPDR_PUPD0               GPIO_PUPDR_PUPD0_Msk

/* Bit definition for GPIO_IDR register */
#define GPIO_IDR_ID0_Pos               (0U)
#define GPIO_IDR_ID0_Msk               (0x1UL << GPIO_IDR_ID0_Pos)              /*\!< 0x00000001 */
#define GPIO_IDR_ID0                   GPIO_IDR_ID0_Msk

/* Bit definition for GPIO_ODR register */
#define GPIO_ODR_OD0_Pos               (0U)
#define GPIO_ODR_OD0_Msk               (0x1UL << GPIO_ODR_OD0_Pos)              /*\!< 0x00000001 */
#define GPIO_ODR_OD0                   GPIO_ODR_OD0_Msk

/* Bit definition for GPIO_BSRR register */
#define GPIO_BSRR_BS0_Pos              (0U)
#define GPIO_BSRR_BS0_Msk              (0x1UL << GPIO_BSRR_BS0_Pos)             /*\!< 0x00000001 */
#define GPIO_BSRR_BS0                  GPIO_BSRR_BS0_Msk

/* Bit definition for GPIO_BRR register */
#define GPIO_BRR_BR0_Pos               (0U)
#define GPIO_BRR_BR0_Msk               (0x1UL << GPIO_BRR_BR0_Pos)              /*\!< 0x00000001 */
#define GPIO_BRR_BR0                   GPIO_BRR_BR0_Msk

/* Bit definition for RCC_IOPENR register */
#define RCC_IOPENR_GPIOAEN_Pos         (0U)
#define RCC_IOPENR_GPIOAEN_Msk         (0x1UL << RCC_IOPENR_GPIOAEN_Pos)        /*\!< 0x00000001 */
#define RCC_IOPENR_GPIOAEN             RCC_IOPENR_GPIOAEN_Msk
#define RCC_IOPENR_GPIOBEN_Pos         (1U)
#define RCC_IOPENR_GPIOBEN_Msk         (0x1UL << RCC_IOPENR_GPIOBEN_Pos)        /*\!< 0x00000002 */
#define RCC_IOPENR_GPIOBEN             RCC_IOPENR_GPIOBEN_Msk
#define RCC_IOPENR_GPIOCEN_Pos         (2U)
#define RCC_IOPENR_GPIOCEN_Msk         (0x1UL << RCC_IOPENR_GPIOCEN_Pos)        /*\!< 0x00000004 */
#define RCC_IOPENR_GPIOCEN             RCC_IOPENR_GPIOCEN_Msk

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32G0B1xx_H */
EOF < /dev/null
