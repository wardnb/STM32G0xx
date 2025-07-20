/*
  btt_skr_mini_e3_3.0.c - driver code for BTT SKR Mini E3 v3.0 board

  Part of grblHAL

  Copyright (c) 2025

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if BOARD_BTT_SKR_MINI_E3_V30

#include <string.h>
#include "btt_skr_mini_e3_3.0_map.h"
#include "serial.h"

#if TRINAMIC_ENABLE
static io_stream_t tmc_uart;

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *dgr)
{
    static TMC_uart_write_datagram_t wdgr = {0};
    volatile uint32_t dly = 50, ms = hal.get_elapsed_ticks();

    tmc_uart.write_n((char *)dgr->data, sizeof(TMC_uart_read_datagram_t));

    while(tmc_uart.get_tx_buffer_count());

    while(--dly);

    tmc_uart.reset_read_buffer();

    // Wait for response with 3ms timeout
    while(tmc_uart.get_rx_buffer_count() < 8) {
        if(hal.get_elapsed_ticks() - ms >= 3)
            break;
    }

    if(tmc_uart.get_rx_buffer_count() >= 8) {
        wdgr.data[0] = tmc_uart.read();
        wdgr.data[1] = tmc_uart.read();
        wdgr.data[2] = tmc_uart.read();
        wdgr.data[3] = tmc_uart.read();
        wdgr.data[4] = tmc_uart.read();
        wdgr.data[5] = tmc_uart.read();
        wdgr.data[6] = tmc_uart.read();
        wdgr.data[7] = tmc_uart.read();
    } else
        wdgr.msg.addr.value = 0xFF;

    dly = 5000;
    while(--dly);

    return &wdgr;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    tmc_uart.write_n((char *)dgr->data, sizeof(TMC_uart_write_datagram_t));

    while(tmc_uart.get_tx_buffer_count());  // Wait while the datagram is delivered
}
#endif // TRINAMIC_ENABLE

void board_init (void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    // Enable GPIO clocks for all ports used by the board
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;

#ifdef SPINDLE_ENABLE_PIN
    // Configure spindle enable pin
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = SPINDLE_ENABLE_PIN;
    HAL_GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_Init);
#endif

#ifdef SPINDLE_DIRECTION_PIN
    // Configure spindle direction pin
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = SPINDLE_DIRECTION_PIN;
    HAL_GPIO_Init(SPINDLE_DIRECTION_PORT, &GPIO_Init);
#endif

#ifdef COOLANT_FLOOD_PIN
    // Configure coolant flood pin
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = COOLANT_FLOOD_PIN;
    HAL_GPIO_Init(COOLANT_FLOOD_PORT, &GPIO_Init);
#endif

#ifdef COOLANT_MIST_PIN
    // Configure coolant mist pin
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = COOLANT_MIST_PIN;
    HAL_GPIO_Init(COOLANT_MIST_PORT, &GPIO_Init);
#endif

#if USB_SERIAL_CDC
    /* Configure USB D+ enable pin if needed */
    // NOTE: Check if STM32G0 requires specific USB pin configuration
    // The STM32F1 version uses PA14, but STM32G0 might differ
#endif

#if TRINAMIC_ENABLE
    // Initialize TMC2209 UART communication
    io_stream_t const *stream;
    
    if((stream = stream_open_instance(TRINAMIC_STREAM, 115200, NULL, "Trinamic UART")) == NULL)
        stream = stream_null_init(115200);
    
    memcpy(&tmc_uart, stream, sizeof(io_stream_t));
    tmc_uart.disable_rx(true);
    tmc_uart.set_enqueue_rt_handler(stream_buffer_all);
#endif

    // Initialize spindle PWM timer for VFD control
#if defined(SPINDLE_PWM_PIN) && defined(DRIVER_SPINDLE_ENABLE)
    // Configure TIM2 for PWM output on PA1
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // Configure PA1 as TIM2_CH2 alternate function
    GPIO_Init.Pin = SPINDLE_PWM_PIN;
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Alternate = GPIO_AF1_TIM2;  // TIM2_CH2 on PA1
    HAL_GPIO_Init(SPINDLE_PWM_PORT, &GPIO_Init);
    
    // Configure timer for PWM (will be fully configured by spindle initialization)
    // This is just the basic setup - frequency and duty cycle set by grblHAL
    TIM2->CR1 = 0;  // Stop timer during configuration
    TIM2->PSC = 63;  // Prescaler for ~1kHz PWM (64MHz/64 = 1MHz timer clock)
    TIM2->ARR = 999; // Period for 1kHz PWM (1MHz/1000 = 1kHz)
    
    // Configure CH2 for PWM mode 1
    TIM2->CCMR1 |= (0x68 << 8);  // PWM mode 1 on CH2
    TIM2->CCER |= TIM_CCER_CC2E;  // Enable CH2 output
    TIM2->CCR2 = 0;  // Start with 0% duty cycle
    
    // Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
#endif
}

#endif