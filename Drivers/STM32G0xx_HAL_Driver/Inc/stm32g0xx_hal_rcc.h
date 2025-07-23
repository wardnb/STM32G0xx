/**
  ******************************************************************************
  * @file    stm32g0xx_hal_rcc.h
  * @brief   Header file of RCC HAL module.
  ******************************************************************************
  */

#ifndef __STM32G0xx_HAL_RCC_H
#define __STM32G0xx_HAL_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/

/** @defgroup RCC_AHB_Peripherals_Enable_Disable AHB Peripherals Enable Disable
  * @brief  Enable or disable the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOD_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIODEN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIODEN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOF_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOFEN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOFEN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

/** @defgroup RCC_APB1_Peripherals_Enable_Disable APB1 Peripherals Enable Disable
  * @{
  */
#define __HAL_RCC_USART1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_USART2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->APBENR1, RCC_APBENR1_USART2EN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APBENR1, RCC_APBENR1_USART2EN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_USART3_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->APBENR1, RCC_APBENR1_USART3EN); \
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APBENR1, RCC_APBENR1_USART3EN); \
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_APB1_FORCE_RESET()   (RCC->APBRSTR1 = 0xFFFFFFFFU)
#define __HAL_RCC_APB1_RELEASE_RESET() (RCC->APBRSTR1 = 0x00000000U)

/* RCC register bit definitions for the macros above */
#define RCC_IOPENR_GPIOAEN_Pos          (0U)
#define RCC_IOPENR_GPIOAEN_Msk          (0x1UL << RCC_IOPENR_GPIOAEN_Pos)
#define RCC_IOPENR_GPIOAEN              RCC_IOPENR_GPIOAEN_Msk
#define RCC_IOPENR_GPIOBEN_Pos          (1U)
#define RCC_IOPENR_GPIOBEN_Msk          (0x1UL << RCC_IOPENR_GPIOBEN_Pos)
#define RCC_IOPENR_GPIOBEN              RCC_IOPENR_GPIOBEN_Msk
#define RCC_IOPENR_GPIOCEN_Pos          (2U)
#define RCC_IOPENR_GPIOCEN_Msk          (0x1UL << RCC_IOPENR_GPIOCEN_Pos)
#define RCC_IOPENR_GPIOCEN              RCC_IOPENR_GPIOCEN_Msk
#define RCC_IOPENR_GPIODEN_Pos          (3U)
#define RCC_IOPENR_GPIODEN_Msk          (0x1UL << RCC_IOPENR_GPIODEN_Pos)
#define RCC_IOPENR_GPIODEN              RCC_IOPENR_GPIODEN_Msk
#define RCC_IOPENR_GPIOFEN_Pos          (5U)
#define RCC_IOPENR_GPIOFEN_Msk          (0x1UL << RCC_IOPENR_GPIOFEN_Pos)
#define RCC_IOPENR_GPIOFEN              RCC_IOPENR_GPIOFEN_Msk

#define RCC_APBENR1_USART2EN_Pos        (17U)
#define RCC_APBENR1_USART2EN_Msk        (0x1UL << RCC_APBENR1_USART2EN_Pos)
#define RCC_APBENR1_USART2EN            RCC_APBENR1_USART2EN_Msk
#define RCC_APBENR1_USART3EN_Pos        (18U)
#define RCC_APBENR1_USART3EN_Msk        (0x1UL << RCC_APBENR1_USART3EN_Pos)
#define RCC_APBENR1_USART3EN            RCC_APBENR1_USART3EN_Msk

#define RCC_APBENR2_USART1EN_Pos        (14U)
#define RCC_APBENR2_USART1EN_Msk        (0x1UL << RCC_APBENR2_USART1EN_Pos)
#define RCC_APBENR2_USART1EN            RCC_APBENR2_USART1EN_Msk

/* RCC Clock Source definitions */
#define RCC_OSCILLATORTYPE_HSI          0x00000001U
#define RCC_OSCILLATORTYPE_HSE          0x00000002U
#define RCC_OSCILLATORTYPE_LSI          0x00000004U
#define RCC_OSCILLATORTYPE_LSE          0x00000008U

#define RCC_HSI_OFF                     0x00000000U
#define RCC_HSI_ON                      RCC_CR_HSION

#define RCC_SYSCLKSOURCE_HSI            RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE            RCC_CFGR_SW_HSE  
#define RCC_SYSCLKSOURCE_PLLCLK         RCC_CFGR_SW_PLL

#define RCC_SYSCLK_DIV1                 0x00000000U
#define RCC_HCLK_DIV1                   0x00000000U
#define RCC_HCLK_DIV2                   0x00000400U

/* Additional RCC structure definitions */
typedef struct {
  uint32_t OscillatorType;      /*!< The oscillators to be configured */
  uint32_t HSEState;            /*!< The new state of the HSE */
  uint32_t LSEState;            /*!< The new state of the LSE */
  uint32_t HSIState;            /*!< The new state of the HSI */
  uint32_t HSIDiv;              /*!< The HSI division factor */
  uint32_t HSICalibrationValue; /*!< The HSI calibration trimming value */
  uint32_t LSIState;            /*!< The new state of the LSI */
  struct {
    uint32_t PLLState;          /*!< PLL state */
    uint32_t PLLSource;         /*!< PLL entry clock source */
    uint32_t PLLM;              /*!< PLL division factor for PLL VCO input clock */
    uint32_t PLLN;              /*!< PLL multiplication factor for VCO */
    uint32_t PLLP;              /*!< PLL division factor for PLLPCLK */
    uint32_t PLLQ;              /*!< PLL division factor for PLLQCLK */
    uint32_t PLLR;              /*!< PLL division factor for PLLRCLK */
  } PLL;
} RCC_OscInitTypeDef;

typedef struct {
  uint32_t ClockType;           /*!< The clock to be configured */
  uint32_t SYSCLKSource;        /*!< The clock source used as system clock */
  uint32_t AHBCLKDivider;       /*!< The AHB clock (HCLK) divider */
  uint32_t APB1CLKDivider;      /*!< The APB1 clock (PCLK1) divider */
} RCC_ClkInitTypeDef;

/* Flash latency definitions */
#define FLASH_LATENCY_0                 0x00000000U   /*!< FLASH Zero wait state */
#define FLASH_LATENCY_1                 0x00000001U   /*!< FLASH One wait state */
#define FLASH_LATENCY_2                 0x00000002U   /*!< FLASH Two wait states */

/* Additional RCC definitions */
#define RCC_HSI_DIV1                    0x00000000U
#define RCC_HSICALIBRATION_DEFAULT      0x40U
#define RCC_PLL_ON                      0x01000000U
#define RCC_PLLSOURCE_HSI               0x00000002U
#define RCC_PLLM_DIV1                   0x00000000U
#define RCC_PLLP_DIV2                   0x00000011U
#define RCC_PLLQ_DIV2                   0x00000011U
#define RCC_PLLR_DIV2                   0x00000001U
#define RCC_CLOCKTYPE_HCLK              0x00000002U
#define RCC_CLOCKTYPE_SYSCLK            0x00000001U
#define RCC_CLOCKTYPE_PCLK1             0x00000004U

/* AHB and IOP Reset macros */
#define __HAL_RCC_AHB_FORCE_RESET()      (RCC->AHBRSTR = 0xFFFFFFFFU)
#define __HAL_RCC_AHB_RELEASE_RESET()    (RCC->AHBRSTR = 0x00000000U)
#define __HAL_RCC_IOP_FORCE_RESET()      (RCC->IOPRSTR = 0xFFFFFFFFU)
#define __HAL_RCC_IOP_RELEASE_RESET()    (RCC->IOPRSTR = 0x00000000U)

/* APB2 Force/Release Reset */
#define __HAL_RCC_APB2_FORCE_RESET()     (RCC->APBRSTR2 = 0xFFFFFFFFU)
#define __HAL_RCC_APB2_RELEASE_RESET()   (RCC->APBRSTR2 = 0x00000000U)

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);


/* Include RCC HAL Extended module */
#include "stm32g0xx_hal_rcc_ex.h"

#ifdef __cplusplus
}
#endif

#endif /* __STM32G0xx_HAL_RCC_H */