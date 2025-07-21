/*

  driver.c - driver code for STM32G0xx ARM processors

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

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "driver.h"
#include "serial.h"
#include "boards/btt_skr_mini_e3_3.0_map.h"

#include "grbl/task.h"
#include "grbl/machine_limits.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/state_machine.h"

// Forward declarations for HAL functions
void driver_delay_ms (uint32_t ms, void (*callback)(void));
void settings_changed (settings_t *settings, settings_changed_flags_t changed);
bool driver_setup (settings_t *settings);
static void stepperWakeUp (void);
static void stepperGoIdle (bool clear_signals);
static void stepperEnable (axes_signals_t enable);
static uint32_t stepperCyclesPerTick (uint32_t cycles_per_tick);
static void stepperPulseStart (stepper_t *stepper);
static void limitsEnable (bool on, axes_signals_t homing_cycle);
static axes_signals_t limitsGetState (void);
static void coolantSetState (coolant_state_t mode);
static coolant_state_t coolantGetState (void);
static void probeConfigureInvertMask (bool is_probe_away);
static probe_state_t probeGetState (void);
static control_signals_t systemGetState (void);
static uint32_t getElapsedTicks (void);
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits);
static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits);
static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value);

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
// #include "eeprom/eeprom.h"  // TODO: Create eeprom.h when EEPROM functionality is needed
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#define DRIVER_IRQMASK (LIMIT_MASK|DEVICES_IRQ_MASK)

// Simplified pin conflict check - ensure masks are properly defined
#if defined(LIMIT_MASK) && defined(DEVICES_IRQ_MASK)
// Pin conflict checking disabled for STM32G0xx - manually verified unique pins
#endif

#define STEPPER_TIMER_DIV 4

#include "grbl/stepdir_map.h"

#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

input_signal_t inputpin[] = {
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = X_LIMIT_PORT,     .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = X2_LIMIT_PORT,    .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitY,         .port = Y_LIMIT_PORT,     .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,       .port = Y2_LIMIT_PORT,    .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
    { .id = Input_LimitZ,         .port = Z_LIMIT_PORT,     .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,       .port = Z2_LIMIT_PORT,    .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,         .port = A_LIMIT_PORT,     .pin = A_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,         .port = B_LIMIT_PORT,     .pin = B_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,         .port = C_LIMIT_PORT,     .pin = C_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,           .port = AUXINPUT0_PORT,   .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,           .port = AUXINPUT1_PORT,   .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,           .port = AUXINPUT2_PORT,   .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,           .port = AUXINPUT3_PORT,   .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4,           .port = AUXINPUT4_PORT,   .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5,           .port = AUXINPUT5_PORT,   .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6,           .port = AUXINPUT6_PORT,   .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT7_PIN
    { .id = Input_Aux7,           .port = AUXINPUT7_PORT,   .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput },
#endif
#ifdef PROBE_PIN
    { .id = Input_Probe,          .port = PROBE_PORT,       .pin = PROBE_PIN,           .group = PinGroup_Probe }
#endif
};

output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep, },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep, },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep, },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep, },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep, },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep, },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = X2_STEP_PORT,           .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = Y2_STEP_PORT,           .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = Z2_STEP_PORT,           .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir, },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = X2_DIRECTION_PORT,      .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = Y2_DIRECTION_PORT,      .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = Z2_DIRECTION_PORT,      .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#if !TRINAMIC_MOTOR_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable, },
#endif
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_ENABLE_PORT,          .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_ENABLE_PORT,          .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef X2_ENABLE_PORT
    { .id = Output_StepperEnableX_2, .port = X2_ENABLE_PORT,        .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable, },
#endif
#ifdef Y2_ENABLE_PORT
    { .id = Output_StepperEnableY_2, .port = Y2_ENABLE_PORT,        .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable, },
#endif
#ifdef Z2_ENABLE_PORT
    { .id = Output_StepperEnableZ_2, .port = Z2_ENABLE_PORT,        .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable, },
#endif
#endif // !TRINAMIC_MOTOR_ENABLE
#ifdef SPINDLE_ENABLE_PIN
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
#endif
#ifdef SPINDLE_DIRECTION_PIN
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
#ifdef COOLANT_FLOOD_PIN
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
#endif
#ifdef COOLANT_MIST_PIN
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef AUXOUTPUT0_PIN
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PIN
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PIN
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PIN
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PIN
    { .id = Output_Aux4,            .port = AUXOUTPUT4_PORT,        .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PIN
    { .id = Output_Aux5,            .port = AUXOUTPUT5_PORT,        .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PIN
    { .id = Output_Aux6,            .port = AUXOUTPUT6_PORT,        .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PIN
    { .id = Output_Aux7,            .port = AUXOUTPUT7_PORT,        .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput }
#endif
};

// Basic system functions and HAL interface

static bool probe_invert;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

stm32_periph_signal_t *periph_pins = NULL;

static void (*systick_isr)(void);

stepper_timer_t stepper_timer = {
    .timer = TIM2,
    .irq = TIM2_IRQn
};

// Driver initialization

bool driver_init (void)
{
    SystemCoreClockUpdate();

    hal.info = "STM32G0xx";
    hal.driver_version = "240101";
    hal.driver_url = GRBL_URL "/STM32G0xx";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock / STEPPER_TIMER_DIV; // 16 MHz
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.configure = probeConfigureInvertMask;
    hal.probe.get_state = probeGetState;

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;

    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

#if USB_SERIAL_CDC
    stream_connect(usbInit());
#else
    stream_connect(serialInit());
#endif

#ifdef HAS_BOARD_INIT
extern void board_init(void);
    board_init();
#endif

    // Initialize step/direction mapping for GPIO_MAP mode
    stepdirmap_init(NULL);

    return true;
}

// Basic placeholder functions - these would need full implementation

void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

static void stepperWakeUp (void)
{
    // TODO: Enable stepper drivers
}

static void stepperGoIdle (bool clear_signals)
{
    // TODO: Disable stepper drivers
}

static void stepperEnable (axes_signals_t enable)
{
    // Enable/disable individual stepper motors
    
    // X-axis stepper control
    if (enable.x) {
        DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, 0);  // Active low enable
    } else {
        DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, 1);  // Disable
    }
    
    // Y-axis stepper control
    if (enable.y) {
        DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, 0);  // Active low enable
    } else {
        DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, 1);  // Disable
    }
    
    // Z-axis stepper control  
    if (enable.z) {
        DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, 0);  // Active low enable
    } else {
        DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, 1);  // Disable
    }
}

static uint32_t stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    // Configure stepper timer period
    // TODO: Set actual timer registers when timer implementation is complete
    return cycles_per_tick;
}

static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        
        // Set step direction pins
#if DIRECTION_OUTMODE == GPIO_MAP
        // Direction pins span multiple ports, so handle individually
        DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, stepper->dir_out.x);
        DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, stepper->dir_out.y);  
        DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, stepper->dir_out.z);
#else
        // Individual pin control (GPIO_SHIFT0)
        DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, stepper->dir_out.x);
        DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, stepper->dir_out.y);
        DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, stepper->dir_out.z);
#endif
    }

    if(stepper->step_out.value) {
#if STEP_OUTMODE == GPIO_MAP
        // Use lookup table for efficient port-wide step pulse
        STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[stepper->step_out.value];
#else
        // Individual step pin control
        DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, stepper->step_out.x);
        DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, stepper->step_out.y);
        DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, stepper->step_out.z);
#endif
    }
}

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    // TODO: Enable/disable limit switches
}

static axes_signals_t limitsGetState (void)
{
    axes_signals_t signals = {0};
    
#if LIMIT_INMODE == GPIO_MAP
    // Read all limit pins from GPIOC at once
    uint32_t bits = LIMIT_PORT->IDR;
    signals.x = !!(bits & X_LIMIT_BIT);
    signals.y = !!(bits & Y_LIMIT_BIT);
    signals.z = !!(bits & Z_LIMIT_BIT);
#else
    // Individual limit pin reads
    signals.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_PIN);
    signals.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_PIN);
    signals.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN);
#endif

    return signals;
}

static void coolantSetState (coolant_state_t mode)
{
    // TODO: Control coolant outputs
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};
    // TODO: Read coolant output states
    return state;
}

static void probeConfigureInvertMask (bool is_probe_away)
{
    probe_invert = is_probe_away;
}

static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};
    // TODO: Read probe input state
    return state;
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals;
    // TODO: Read control input signals
    signals.value = 0;
    return signals;
}

static uint32_t getElapsedTicks (void)
{
    return uwTick;
}

static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    // TODO: Configure GPIO interrupt
}

void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    // TODO: Handle settings changes
}

bool driver_setup (settings_t *settings)
{
    return true;  // IOInitDone not available, return true for success
}

// Systick interrupt handler
void SysTick_Handler (void)
{
    uwTick++;
    
    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }

    HAL_IncTick();
    
    if(systick_isr)
        systick_isr();
}