/*

  driver.h - driver code for STM32G0xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/nuts_bolts.h"
#include "grbl/driver_opts.h"

// STM32G0 does not have bitband support, use regular GPIO operations
#define DIGITAL_IN(port, pin) (((port)->IDR & (1U << (pin))) != 0)
#define DIGITAL_OUT(port, pin, on) { if(on) (port)->BSRR = (1U << (pin)); else (port)->BRR = (1U << (pin)); }

#define timer(t) timerN(t)
#define timerN(t) TIM ## t
#define timerINT(t) timerint(t)
#define timerint(t) TIM ## t ## _IRQn
#define timerHANDLER(t) timerhandler(t)
#define timerhandler(t) TIM ## t ## _IRQHandler
#define timerCCEN(c, n) timerccen(c, n)
#define timerccen(c, n) TIM_CCER_CC ## c ## n ## E
#define timerCCMR(p, c) timerccmr(p, c)
#define timerccmr(p, c) TIM ## p->CCMR ## c
#define timerOCM(p, c) timerocm(p, c)
#define timerocm(p, c) TIM_CCMR ## p ##_OC ## c ## M_1|TIM_CCMR ## p ##_OC ## c ## M_2
#define timerOCMC(p, c) timerocmc(p, c)
#define timerocmc(p, c) (TIM_CCMR ## p ##_OC ## c ## M|TIM_CCMR ## p ##_CC ## c ## S)
#define timerCCR(t, c) timerccr(t, c)
#define timerccr(t, c) TIM ## t->CCR ## c
#define timerCCP(c, n) timerccp(c, n)
#define timerccp(c, n) TIM_CCER_CC ## c ## n ## P
#define timerCR2OIS(c, n) timercr2ois(c, n)
#define timercr2ois(c, n) TIM_CR2_OIS ## c ## n
#define timerAF(t, f) timeraf(t, f)
#define timeraf(t, f) GPIO_AF ## f ## _TIM ## t
#define timerCLKENA(t) timercken(t)
#define timercken(t) __HAL_RCC_TIM ## t ## _CLK_ENABLE

#define usart(t) usartN(t)
#define usartN(t) USART ## t
#define usartINT(t) usartint(t)
#define usartint(t) USART ## t ## _IRQn
#define usartHANDLER(t) usarthandler(t)
#define usarthandler(t) USART ## t ## _IRQHandler
#define usartCLKEN(t) usartclken(t)
#define usartclken(t) __HAL_RCC_USART ## t ## _CLK_ENABLE

// Define GPIO output mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14
#define GPIO_BITBAND 15

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
} output_signal_t;

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
} input_signal_t;

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
    uint8_t offset;
} periph_signal_t;

extern input_signal_t inputpin[];
extern output_signal_t outputpin[];
extern periph_signal_t *periph_pins;

// timer definitions

typedef struct {
    TIM_TypeDef *timer;
    uint32_t channel;
    uint32_t ccmr_reg;
    volatile uint32_t *ccr_reg;
    uint32_t oc_mode;
    IRQn_Type irq;
} stepper_timer_t;

typedef struct {
    TIM_TypeDef *timer;
    uint32_t channel;
    volatile uint32_t *ccr_reg;
    uint32_t ccmr_reg;
    uint32_t oc_mode;
    IRQn_Type irq;
} spindle_pwm_t;

#ifdef HAS_IOPORTS
typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
    ioport_interrupt_callback_ptr interrupt_callback;
    bool invert;
    volatile bool active;
    volatile bool debounce;
    ioport_t id;
} aux_ctrl_t;
#endif

extern stepper_timer_t stepper_timer;

bool driver_init (void);
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);

#endif // __DRIVER_H__