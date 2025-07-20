/**
  ******************************************************************************
  * @file    stm32g0b1xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32G0B1xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - peripherals registers declarations and bits definition
  *           - Macros to access peripheral's registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32g0b1xx
  * @{
  */

#ifndef __STM32G0B1xx_H
#define __STM32G0B1xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M0+ Processor and Core Peripherals
   */
#define __CM0PLUS_REV             0U       /*!< Core Revision r0p1                            */
#define __MPU_PRESENT             1U       /*!< STM32G0xx provides MPU                        */
#define __VTOR_PRESENT            1U       /*!< Vector Table Register supported               */
#define __NVIC_PRIO_BITS          2U       /*!< STM32G0xx uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32G0B1xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M0+ Processor Exceptions Numbers ***************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Cortex-M0+ Non Maskable Interrupt                 */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0+ Hard Fault Interrupt                   */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0+ SV Call Interrupt                     */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0+ Pend SV Interrupt                     */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0+ System Tick Interrupt                 */

/******  STM32G0B1xx specific Interrupt Numbers ***************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI line detection Interrupt           */
  RTC_TAMP_IRQn               = 2,      /*!< RTC interrupt through the EXTI line 19 & 21         */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                               */
  RCC_IRQn                    = 4,      /*!< RCC global Interrupt                                 */
  EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupts                        */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupts                        */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupts                        */
  USB_IRQn                    = 8,      /*!< USB global Interrupt                                 */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                             */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupts             */
  DMA1_Ch4_7_DMA2_Ch1_5_IRQn  = 11,     /*!< DMA1 Ch4 to Ch7, DMA2 Ch1 to Ch5 Interrupts         */
  ADC1_COMP_IRQn              = 12,     /*!< ADC1, COMP1 and COMP2 Interrupts                    */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupts */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 15,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                */
  TIM6_DAC_LPTIM1_IRQn        = 17,     /*!< TIM6, DAC and LPTIM1 global Interrupts              */
  TIM7_LPTIM2_IRQn            = 18,     /*!< TIM7 and LPTIM2 global Interrupt                    */
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                               */
  TIM15_IRQn                  = 20,     /*!< TIM15 global Interrupt                               */
  TIM16_IRQn                  = 21,     /*!< TIM16 global Interrupt                               */
  TIM17_IRQn                  = 22,     /*!< TIM17 global Interrupt                               */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup) */
  I2C2_IRQn                   = 24,     /*!< I2C2 Event Interrupt                                 */
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 26,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 27,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 28,     /*!< USART2 global Interrupt                              */
  USART3_4_LPUART1_IRQn       = 29,     /*!< USART3, USART4 and LPUART1 Interrupts               */
  CEC_IRQn                    = 30,     /*!< CEC global Interrupt                                 */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm0plus.h"             /* Cortex-M0+ processor and core peripherals */
#include "system_stm32g0xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/** 
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;      /*!< GPIO Bit Reset register,               Address offset: 0x28      */
} GPIO_TypeDef;

/** 
  * @brief Reset and Clock Control
  */
typedef struct
{
  __IO uint32_t CR;         /*!< RCC clock control register,                                              Address offset: 0x00 */
  __IO uint32_t ICSCR;      /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                                        Address offset: 0x08 */
  __IO uint32_t PLLCFGR;    /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
  __IO uint32_t RESERVED0;  /*!< Reserved,                                                                 Address offset: 0x10 */
  __IO uint32_t RESERVED1;  /*!< Reserved,                                                                 Address offset: 0x14 */
  __IO uint32_t CIER;       /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
  __IO uint32_t CIFR;       /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
  __IO uint32_t CICR;       /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
  __IO uint32_t IOPRSTR;    /*!< RCC I/O port reset register,                                             Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripherals reset register,                                      Address offset: 0x28 */
  __IO uint32_t APBRSTR1;   /*!< RCC APB peripherals reset register 1,                                    Address offset: 0x2C */
  __IO uint32_t APBRSTR2;   /*!< RCC APB peripherals reset register 2,                                    Address offset: 0x30 */
  __IO uint32_t IOPENR;     /*!< RCC I/O port clock enable register,                                      Address offset: 0x34 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripherals clock enable register,                               Address offset: 0x38 */
  __IO uint32_t APBENR1;    /*!< RCC APB peripherals clock enable register 1,                             Address offset: 0x3C */
  __IO uint32_t APBENR2;    /*!< RCC APB peripherals clock enable register 2,                             Address offset: 0x40 */
  __IO uint32_t IOPSMENR;   /*!< RCC I/O port clocks enable in sleep mode register,                       Address offset: 0x44 */
  __IO uint32_t AHBSMENR;   /*!< RCC AHB peripheral clocks enable in sleep mode register,                 Address offset: 0x48 */
  __IO uint32_t APBSMENR1;  /*!< RCC APB peripheral clocks enable in sleep mode register 1,               Address offset: 0x4C */
  __IO uint32_t APBSMENR2;  /*!< RCC APB peripheral clocks enable in sleep mode register 2,               Address offset: 0x50 */
  __IO uint32_t CCIPR;      /*!< RCC peripherals independent clock configuration register,                Address offset: 0x54 */
  __IO uint32_t CCIPR2;     /*!< RCC peripherals independent clock configuration register 2,              Address offset: 0x58 */
  __IO uint32_t BDCR;       /*!< RCC backup domain control register,                                      Address offset: 0x5C */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                                     Address offset: 0x60 */
} RCC_TypeDef;

/** 
  * @brief Debug MCU
  */
typedef struct
{
  __IO uint32_t IDCODE;     /*!< MCU device ID code,              Address offset: 0x00 */
  __IO uint32_t CR;         /*!< Debug MCU configuration register, Address offset: 0x04 */
} DBGMCU_TypeDef;

/** 
  * @brief TIM
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
} TIM_TypeDef;

/** 
  * @brief UART
  */
typedef struct
{
  __IO uint32_t CR1;    /*!< UART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;    /*!< UART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;    /*!< UART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;    /*!< UART Baud rate register,                 Address offset: 0x0C */
  __IO uint32_t GTPR;   /*!< UART Guard time and prescaler register,  Address offset: 0x10 */
  __IO uint32_t RTOR;   /*!< UART Receiver timeout register,          Address offset: 0x14 */
  __IO uint32_t RQR;    /*!< UART Request register,                   Address offset: 0x18 */
  __IO uint32_t ISR;    /*!< UART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< UART Interrupt clear register,           Address offset: 0x20 */
  __IO uint32_t RDR;    /*!< UART Receive data register,              Address offset: 0x24 */
  __IO uint32_t TDR;    /*!< UART Transmit data register,             Address offset: 0x28 */
  __IO uint32_t PRESC;  /*!< UART Prescaler register,                 Address offset: 0x2C */
} UART_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 512 KB) base address */
#define SRAM_BASE             0x20000000UL /*!< SRAM(up to 144 KB) base address */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address */
#define IOPORT_BASE           0x50000000UL /*!< IOPORT base address */

#define SRAM_SIZE_MAX         0x00024000UL /*!< maximum SRAM size (up to 144 KBytes) */

/*!< Peripheral memory map */
#define APBPERIPH_BASE        PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

/*!< APB peripherals */
#define TIM2_BASE             (APBPERIPH_BASE + 0x00000000UL)
#define TIM3_BASE             (APBPERIPH_BASE + 0x00000400UL)
#define TIM6_BASE             (APBPERIPH_BASE + 0x00001000UL)
#define TIM7_BASE             (APBPERIPH_BASE + 0x00001400UL)
#define TIM14_BASE            (APBPERIPH_BASE + 0x00002000UL)
#define RTC_BASE              (APBPERIPH_BASE + 0x00002800UL)
#define WWDG_BASE             (APBPERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE             (APBPERIPH_BASE + 0x00003000UL)
#define SPI2_BASE             (APBPERIPH_BASE + 0x00003800UL)
#define USART2_BASE           (APBPERIPH_BASE + 0x00004400UL)
#define USART3_BASE           (APBPERIPH_BASE + 0x00004800UL)
#define USART4_BASE           (APBPERIPH_BASE + 0x00004C00UL)
#define I2C1_BASE             (APBPERIPH_BASE + 0x00005400UL)
#define I2C2_BASE             (APBPERIPH_BASE + 0x00005800UL)
#define USB_BASE              (APBPERIPH_BASE + 0x00005C00UL) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APBPERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */
#define FDCAN1_BASE           (APBPERIPH_BASE + 0x00006400UL)
#define FDCAN2_BASE           (APBPERIPH_BASE + 0x00006800UL)
#define FDCAN_CONFIG_BASE     (APBPERIPH_BASE + 0x00006C00UL) /*!< FDCAN configuration registers base address */
#define CEC_BASE              (APBPERIPH_BASE + 0x00007800UL)
#define PWR_BASE              (APBPERIPH_BASE + 0x00007000UL)
#define DAC1_BASE             (APBPERIPH_BASE + 0x00007400UL)
#define DAC_BASE              (APBPERIPH_BASE + 0x00007400UL) /*!< DAC compatible base address */
#define LPTIM1_BASE           (APBPERIPH_BASE + 0x00007C00UL)
#define LPUART1_BASE          (APBPERIPH_BASE + 0x00008000UL)
#define LPTIM2_BASE           (APBPERIPH_BASE + 0x00009400UL)
#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000UL)
#define VREFBUF_BASE          (APBPERIPH_BASE + 0x00010030UL)
#define COMP1_BASE            (APBPERIPH_BASE + 0x00010200UL)
#define COMP2_BASE            (APBPERIPH_BASE + 0x00010210UL)
#define ADC1_BASE             (APBPERIPH_BASE + 0x00012400UL)
#define ADC1_COMMON_BASE      (APBPERIPH_BASE + 0x00012708UL)
#define ADC_BASE              (APBPERIPH_BASE + 0x00012708UL) /*!< ADC compatible base address */
#define TIM1_BASE             (APBPERIPH_BASE + 0x00012C00UL)
#define SPI1_BASE             (APBPERIPH_BASE + 0x00013000UL)
#define USART1_BASE           (APBPERIPH_BASE + 0x00013800UL)
#define TIM15_BASE            (APBPERIPH_BASE + 0x00014000UL)
#define TIM16_BASE            (APBPERIPH_BASE + 0x00014400UL)
#define TIM17_BASE            (APBPERIPH_BASE + 0x00014800UL)

/*!< AHB peripherals */
#define DMA1_BASE             (AHBPERIPH_BASE + 0x00000000UL)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x00000400UL)
#define DMAMUX1_BASE          (AHBPERIPH_BASE + 0x00000800UL)
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)
#define EXTI_BASE             (AHBPERIPH_BASE + 0x00001800UL)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000UL) /*!< FLASH registers base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000UL)
#define RNG_BASE              (AHBPERIPH_BASE + 0x00005000UL)
#define AES_BASE              (AHBPERIPH_BASE + 0x00006000UL)

#define DMA1_Channel1_BASE    (DMA1_BASE + 0x00000008UL)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x0000001CUL)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x00000030UL)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x00000044UL)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x00000058UL)
#define DMA1_Channel6_BASE    (DMA1_BASE + 0x0000006CUL)
#define DMA1_Channel7_BASE    (DMA1_BASE + 0x00000080UL)
#define DMA2_Channel1_BASE    (DMA2_BASE + 0x00000008UL)
#define DMA2_Channel2_BASE    (DMA2_BASE + 0x0000001CUL)
#define DMA2_Channel3_BASE    (DMA2_BASE + 0x00000030UL)
#define DMA2_Channel4_BASE    (DMA2_BASE + 0x00000044UL)
#define DMA2_Channel5_BASE    (DMA2_BASE + 0x00000058UL)

#define DMAMUX1_Channel0_BASE      (DMAMUX1_BASE)
#define DMAMUX1_Channel1_BASE      (DMAMUX1_BASE + 0x00000004UL)
#define DMAMUX1_Channel2_BASE      (DMAMUX1_BASE + 0x00000008UL)
#define DMAMUX1_Channel3_BASE      (DMAMUX1_BASE + 0x0000000CUL)
#define DMAMUX1_Channel4_BASE      (DMAMUX1_BASE + 0x00000010UL)
#define DMAMUX1_Channel5_BASE      (DMAMUX1_BASE + 0x00000014UL)
#define DMAMUX1_Channel6_BASE      (DMAMUX1_BASE + 0x00000018UL)
#define DMAMUX1_Channel7_BASE      (DMAMUX1_BASE + 0x0000001CUL)
#define DMAMUX1_Channel8_BASE      (DMAMUX1_BASE + 0x00000020UL)
#define DMAMUX1_Channel9_BASE      (DMAMUX1_BASE + 0x00000024UL)
#define DMAMUX1_Channel10_BASE     (DMAMUX1_BASE + 0x00000028UL)
#define DMAMUX1_Channel11_BASE     (DMAMUX1_BASE + 0x0000002CUL)

#define DMAMUX1_RequestGenerator0_BASE  (DMAMUX1_BASE + 0x00000100UL)
#define DMAMUX1_RequestGenerator1_BASE  (DMAMUX1_BASE + 0x00000104UL)
#define DMAMUX1_RequestGenerator2_BASE  (DMAMUX1_BASE + 0x00000108UL)
#define DMAMUX1_RequestGenerator3_BASE  (DMAMUX1_BASE + 0x0000010CUL)

#define DMAMUX1_ChannelStatus_BASE      (DMAMUX1_BASE + 0x00000080UL)
#define DMAMUX1_RequestGenStatus_BASE   (DMAMUX1_BASE + 0x00000140UL)

/*!< IOPORT */
#define GPIOA_BASE            (IOPORT_BASE + 0x00000000UL)
#define GPIOB_BASE            (IOPORT_BASE + 0x00000400UL)
#define GPIOC_BASE            (IOPORT_BASE + 0x00000800UL)
#define GPIOD_BASE            (IOPORT_BASE + 0x00000C00UL)
#define GPIOE_BASE            (IOPORT_BASE + 0x00001000UL)
#define GPIOF_BASE            (IOPORT_BASE + 0x00001400UL)

/*!< System memory and OTP area */
#define SYSTEM_MEMORY_BASE    0x1FFF0000UL /*!< System Memory base address */
#define OTP_AREA_BASE         0x1FFF7000UL /*!< OTP area base address */
#define UID_BASE              0x1FFF7590UL /*!< Unique device ID register base address */
#define FLASHSIZE_BASE        0x1FFF75E0UL /*!< Flash size data register base address */

/*!< Debug MCU */
#define DBGMCU_BASE           0x40015800UL /*!< Debug MCU registers base address */

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TIM17               ((TIM_TypeDef *) TIM17_BASE)

#define USART1              ((UART_TypeDef *) USART1_BASE)
#define USART2              ((UART_TypeDef *) USART2_BASE)
#define USART3              ((UART_TypeDef *) USART3_BASE)
#define USART4              ((UART_TypeDef *) USART4_BASE)
#define LPUART1             ((UART_TypeDef *) LPUART1_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Hardware_Constant_Definition
  * @{
  */
#define LSI_STARTUP_TIME 85U /*!< LSI Maximum startup time in us */

/**
  * @}
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                            Debug MCU (DBGMCU)                             */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for DBGMCU_IDCODE register  **************/
#define DBGMCU_IDCODE_DEV_ID_Pos       (0U)
#define DBGMCU_IDCODE_DEV_ID_Msk       (0xFFFUL << DBGMCU_IDCODE_DEV_ID_Pos)    /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID           DBGMCU_IDCODE_DEV_ID_Msk                 /*!< Device ID */
#define DBGMCU_IDCODE_REV_ID_Pos       (16U)
#define DBGMCU_IDCODE_REV_ID_Msk       (0xFFFFUL << DBGMCU_IDCODE_REV_ID_Pos)   /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID           DBGMCU_IDCODE_REV_ID_Msk                 /*!< Revision ID */

/******************************************************************************/
/*                                                                            */
/*                        General Purpose I/O (GPIO)                         */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO pins  **************************/
#define GPIO_PIN_0                     ((uint16_t)0x0001)  /*!< Pin 0 selected    */
#define GPIO_PIN_1                     ((uint16_t)0x0002)  /*!< Pin 1 selected    */
#define GPIO_PIN_2                     ((uint16_t)0x0004)  /*!< Pin 2 selected    */
#define GPIO_PIN_3                     ((uint16_t)0x0008)  /*!< Pin 3 selected    */
#define GPIO_PIN_4                     ((uint16_t)0x0010)  /*!< Pin 4 selected    */
#define GPIO_PIN_5                     ((uint16_t)0x0020)  /*!< Pin 5 selected    */
#define GPIO_PIN_6                     ((uint16_t)0x0040)  /*!< Pin 6 selected    */
#define GPIO_PIN_7                     ((uint16_t)0x0080)  /*!< Pin 7 selected    */
#define GPIO_PIN_8                     ((uint16_t)0x0100)  /*!< Pin 8 selected    */
#define GPIO_PIN_9                     ((uint16_t)0x0200)  /*!< Pin 9 selected    */
#define GPIO_PIN_10                    ((uint16_t)0x0400)  /*!< Pin 10 selected   */
#define GPIO_PIN_11                    ((uint16_t)0x0800)  /*!< Pin 11 selected   */
#define GPIO_PIN_12                    ((uint16_t)0x1000)  /*!< Pin 12 selected   */
#define GPIO_PIN_13                    ((uint16_t)0x2000)  /*!< Pin 13 selected   */
#define GPIO_PIN_14                    ((uint16_t)0x4000)  /*!< Pin 14 selected   */
#define GPIO_PIN_15                    ((uint16_t)0x8000)  /*!< Pin 15 selected   */
#define GPIO_PIN_All                   ((uint16_t)0xFFFF)  /*!< All pins selected */

/* GPIO mode definitions */
#define GPIO_MODE_INPUT                        0x00000000U   /*!< Input Floating Mode                   */
#define GPIO_MODE_OUTPUT_PP                    0x00000001U   /*!< Output Push Pull Mode                 */
#define GPIO_MODE_OUTPUT_OD                    0x00000011U   /*!< Output Open Drain Mode                */
#define GPIO_MODE_AF_PP                        0x00000002U   /*!< Alternate Function Push Pull Mode     */
#define GPIO_MODE_AF_OD                        0x00000012U   /*!< Alternate Function Open Drain Mode    */
#define GPIO_MODE_ANALOG                       0x00000003U   /*!< Analog Mode  */

/* GPIO pull-up/pull-down definitions */
#define GPIO_NOPULL                            0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULLUP                            0x00000001U   /*!< Pull-up activation                  */
#define GPIO_PULLDOWN                          0x00000002U   /*!< Pull-down activation                */

/* GPIO speed definitions */
#define GPIO_SPEED_FREQ_LOW                    0x00000000U  /*!< Low speed     */
#define GPIO_SPEED_FREQ_MEDIUM                 0x00000001U  /*!< Medium speed  */
#define GPIO_SPEED_FREQ_HIGH                   0x00000002U  /*!< High speed    */
#define GPIO_SPEED_FREQ_VERY_HIGH              0x00000003U  /*!< Very high speed */

/* GPIO alternate function definitions */
#define GPIO_AF0_EVENTOUT      ((uint8_t)0x00)  /* AF0: EVENTOUT Alternate Function mapping  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)  /* AF1: TIM1 Alternate Function mapping      */
#define GPIO_AF1_TIM3          ((uint8_t)0x01)  /* AF1: TIM3 Alternate Function mapping      */
#define GPIO_AF1_USART1        ((uint8_t)0x01)  /* AF1: USART1 Alternate Function mapping   */
#define GPIO_AF1_USART2        ((uint8_t)0x01)  /* AF1: USART2 Alternate Function mapping   */

#define GPIO_MODE_INPUT                0x00000000U   /*!< Input Floating Mode                   */
#define GPIO_MODE_OUTPUT_PP            0x00000001U   /*!< Output Push Pull Mode                 */
#define GPIO_MODE_OUTPUT_OD            0x00000011U   /*!< Output Open Drain Mode                */
#define GPIO_MODE_AF_PP                0x00000002U   /*!< Alternate Function Push Pull Mode     */
#define GPIO_MODE_AF_OD                0x00000012U   /*!< Alternate Function Open Drain Mode    */
#define GPIO_MODE_ANALOG               0x00000003U   /*!< Analog Mode                           */

#define GPIO_NOPULL                    0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULLUP                    0x00000001U   /*!< Pull-up activation                  */
#define GPIO_PULLDOWN                  0x00000002U   /*!< Pull-down activation                */

#define GPIO_SPEED_FREQ_LOW            0x00000000U  /*!< Low speed     */
#define GPIO_SPEED_FREQ_MEDIUM         0x00000001U  /*!< Medium speed  */
#define GPIO_SPEED_FREQ_HIGH           0x00000002U  /*!< High speed    */
#define GPIO_SPEED_FREQ_VERY_HIGH      0x00000003U  /*!< Very high speed */

/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODE0_Pos           (0U)
#define GPIO_MODER_MODE0_Msk           (0x3UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000003 */
#define GPIO_MODER_MODE0               GPIO_MODER_MODE0_Msk
#define GPIO_MODER_MODE0_0             (0x1UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000001 */
#define GPIO_MODER_MODE0_1             (0x2UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000002 */

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos            (0U)
#define GPIO_OTYPER_OT0_Msk            (0x1UL << GPIO_OTYPER_OT0_Pos)           /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                GPIO_OTYPER_OT0_Msk

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos       (0U)
#define GPIO_OSPEEDR_OSPEED0_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0           GPIO_OSPEEDR_OSPEED0_Msk
#define GPIO_OSPEEDR_OSPEED0_0         (0x1UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1         (0x2UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000002 */

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos           (0U)
#define GPIO_PUPDR_PUPD0_Msk           (0x3UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0               GPIO_PUPDR_PUPD0_Msk
#define GPIO_PUPDR_PUPD0_0             (0x1UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1             (0x2UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000002 */

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos               (0U)
#define GPIO_IDR_ID0_Msk               (0x1UL << GPIO_IDR_ID0_Pos)              /*!< 0x00000001 */
#define GPIO_IDR_ID0                   GPIO_IDR_ID0_Msk

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos               (0U)
#define GPIO_ODR_OD0_Msk               (0x1UL << GPIO_ODR_OD0_Pos)              /*!< 0x00000001 */
#define GPIO_ODR_OD0                   GPIO_ODR_OD0_Msk

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos              (0U)
#define GPIO_BSRR_BS0_Msk              (0x1UL << GPIO_BSRR_BS0_Pos)             /*!< 0x00000001 */
#define GPIO_BSRR_BS0                  GPIO_BSRR_BS0_Msk

/******************************************************************************/
/*                                                                            */
/*                             Reset and Clock Control                        */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for TIM_CR1 register  ******************/
#define TIM_CR1_CEN_Pos           (0U)
#define TIM_CR1_CEN_Msk           (0x1UL << TIM_CR1_CEN_Pos)                   /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!< Counter enable */

/* TIM_CCER register definitions */
#define TIM_CCER_CC1E_Pos         (0U)
#define TIM_CCER_CC1E_Msk         (0x1UL << TIM_CCER_CC1E_Pos)                /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                           /*!< Capture/Compare 1 output enable */
#define TIM_CCER_CC2E_Pos         (4U)
#define TIM_CCER_CC2E_Msk         (0x1UL << TIM_CCER_CC2E_Pos)                /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                           /*!< Capture/Compare 2 output enable */
#define TIM_CCER_CC3E_Pos         (8U)
#define TIM_CCER_CC3E_Msk         (0x1UL << TIM_CCER_CC3E_Pos)                /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                           /*!< Capture/Compare 3 output enable */
#define TIM_CCER_CC4E_Pos         (12U)
#define TIM_CCER_CC4E_Msk         (0x1UL << TIM_CCER_CC4E_Pos)                /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                           /*!< Capture/Compare 4 output enable */
#define TIM_CR1_DIR_Pos           (4U)
#define TIM_CR1_DIR_Msk           (0x1UL << TIM_CR1_DIR_Pos)                   /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!< Direction */
#define TIM_CR1_CMS_Pos           (5U)
#define TIM_CR1_CMS_Msk           (0x3UL << TIM_CR1_CMS_Pos)                   /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!< CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1UL << TIM_CR1_CMS_Pos)                   /*!< 0x00000020 */
#define TIM_CR1_CMS_1             (0x2UL << TIM_CR1_CMS_Pos)                   /*!< 0x00000040 */

/********************  Bit definition for TIM_CCMR1 register  ****************/
#define TIM_CCMR1_OC1M_Pos        (4U)
#define TIM_CCMR1_OC1M_Msk        (0x7UL << TIM_CCMR1_OC1M_Pos)                /*!< 0x00000070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0          (0x1UL << TIM_CCMR1_OC1M_Pos)                /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)                /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2          (0x4UL << TIM_CCMR1_OC1M_Pos)                /*!< 0x00000040 */

/******************************************************************************/
/********************  Bit definition for RCC_CR register  *******************/
#define RCC_CR_HSION_Pos                     (8U)
#define RCC_CR_HSION_Msk                     (0x1UL << RCC_CR_HSION_Pos)      /*!< 0x00000100 */
#define RCC_CR_HSION                         RCC_CR_HSION_Msk                 /*!< Internal High Speed clock enable */

/********************  Bit definition for RCC_CFGR register  ******************/
#define RCC_CFGR_SW_Pos                      (0U)
#define RCC_CFGR_SW_Msk                      (0x7UL << RCC_CFGR_SW_Pos)       /*!< 0x00000007 */
#define RCC_CFGR_SW                          RCC_CFGR_SW_Msk                  /*!< SW[2:0] bits (System clock Switch) */
#define RCC_CFGR_SW_HSI                      (0x01UL << RCC_CFGR_SW_Pos)      /*!< 0x00000001 */
#define RCC_CFGR_SW_HSE                      (0x02UL << RCC_CFGR_SW_Pos)      /*!< 0x00000002 */
#define RCC_CFGR_SW_PLL                      (0x03UL << RCC_CFGR_SW_Pos)      /*!< 0x00000003 */

#define RCC_CFGR_SWS_Pos                     (3U)
#define RCC_CFGR_SWS_Msk                     (0x7UL << RCC_CFGR_SWS_Pos)      /*!< 0x00000038 */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                 /*!< SWS[2:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_HSI                     (0x01UL << RCC_CFGR_SWS_Pos)     /*!< 0x00000008 */
#define RCC_CFGR_SWS_HSE                     (0x02UL << RCC_CFGR_SWS_Pos)     /*!< 0x00000010 */
#define RCC_CFGR_SWS_PLL                     (0x03UL << RCC_CFGR_SWS_Pos)     /*!< 0x00000018 */

#define RCC_CFGR_HPRE_Pos                    (8U)
#define RCC_CFGR_HPRE_Msk                    (0xFUL << RCC_CFGR_HPRE_Pos)     /*!< 0x00000F00 */
#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                /*!< HPRE[3:0] bits (AHB prescaler) */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLSRC_Pos               (0U)
#define RCC_PLLCFGR_PLLSRC_Msk               (0x3UL << RCC_PLLCFGR_PLLSRC_Pos) /*!< 0x00000003 */
#define RCC_PLLCFGR_PLLSRC                   RCC_PLLCFGR_PLLSRC_Msk            /*!< PLL input source */

#define RCC_PLLCFGR_PLLM_Pos                 (4U)
#define RCC_PLLCFGR_PLLM_Msk                 (0x7UL << RCC_PLLCFGR_PLLM_Pos)  /*!< 0x00000070 */
#define RCC_PLLCFGR_PLLM                     RCC_PLLCFGR_PLLM_Msk              /*!< PLLM[2:0] bits */

#define RCC_PLLCFGR_PLLN_Pos                 (8U)
#define RCC_PLLCFGR_PLLN_Msk                 (0x7FUL << RCC_PLLCFGR_PLLN_Pos) /*!< 0x00007F00 */
#define RCC_PLLCFGR_PLLN                     RCC_PLLCFGR_PLLN_Msk              /*!< PLLN[6:0] bits */

#define RCC_PLLCFGR_PLLPEN_Pos               (16U)
#define RCC_PLLCFGR_PLLPEN_Msk               (0x1UL << RCC_PLLCFGR_PLLPEN_Pos) /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLPEN                   RCC_PLLCFGR_PLLPEN_Msk            /*!< PLLPCLK output enable */

#define RCC_PLLCFGR_PLLP_Pos                 (17U)
#define RCC_PLLCFGR_PLLP_Msk                 (0x1FUL << RCC_PLLCFGR_PLLP_Pos) /*!< 0x003E0000 */
#define RCC_PLLCFGR_PLLP                     RCC_PLLCFGR_PLLP_Msk              /*!< PLLP[4:0] bits */

#define RCC_PLLCFGR_PLLQEN_Pos               (24U)
#define RCC_PLLCFGR_PLLQEN_Msk               (0x1UL << RCC_PLLCFGR_PLLQEN_Pos) /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQEN                   RCC_PLLCFGR_PLLQEN_Msk            /*!< PLLQCLK output enable */

#define RCC_PLLCFGR_PLLQ_Pos                 (25U)
#define RCC_PLLCFGR_PLLQ_Msk                 (0x7UL << RCC_PLLCFGR_PLLQ_Pos)  /*!< 0x0E000000 */
#define RCC_PLLCFGR_PLLQ                     RCC_PLLCFGR_PLLQ_Msk              /*!< PLLQ[2:0] bits */

#define RCC_PLLCFGR_PLLREN_Pos               (28U)
#define RCC_PLLCFGR_PLLREN_Msk               (0x1UL << RCC_PLLCFGR_PLLREN_Pos) /*!< 0x10000000 */
#define RCC_PLLCFGR_PLLREN                   RCC_PLLCFGR_PLLREN_Msk            /*!< PLLRCLK output enable */

#define RCC_PLLCFGR_PLLR_Pos                 (29U)
#define RCC_PLLCFGR_PLLR_Msk                 (0x7UL << RCC_PLLCFGR_PLLR_Pos)  /*!< 0xE0000000 */
#define RCC_PLLCFGR_PLLR                     RCC_PLLCFGR_PLLR_Msk              /*!< PLLR[2:0] bits */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32G0B1xx_H */

/**
  * @}
  */

/**
  * @}
  */