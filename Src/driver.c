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
// IWDG HAL driver not available - use direct register access

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
static void stepperEnable (axes_signals_t enable, bool hold);
static void stepperCyclesPerTick (uint32_t cycles_per_tick);
static void stepperPulseStart (stepper_t *stepper);
static void limitsEnable (bool on, axes_signals_t homing_cycle);
static limit_signals_t limitsGetState (void);
static void coolantSetState (coolant_state_t mode);
static coolant_state_t coolantGetState (void);
static void probeConfigureInvertMask (bool is_probe_away, bool probing);
static probe_state_t probeGetState (void);
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle);
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm);
static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm);
static control_signals_t systemGetState (void);
static uint32_t getElapsedTicks (void);
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits);
static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits);
static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value);

#ifdef I2C_PORT
#include "i2c.h"
// Note: I2C functionality available via grblHAL plugin system
// I2C_PORT must be defined in board map to enable I2C support
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
// Note: EEPROM functionality is implemented through grblHAL plugin system
// The eeprom plugin provides all necessary EEPROM storage functionality
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

// Phase 1: Safety Enhancement Variables
typedef struct {
    axes_signals_t raw_state;
    axes_signals_t debounced_state;
    axes_signals_t history[8];
    uint8_t history_index;
    uint32_t last_read_time;
    uint32_t emergency_stop_time;
    bool emergency_active;
} safety_state_t;

static safety_state_t safety_state = {0};

// Spindle encoder monitoring
typedef struct {
    volatile uint32_t pulse_count;
    volatile uint32_t last_pulse_time;
    volatile uint32_t pulse_period;
    uint32_t target_rpm;
    uint32_t actual_rpm;
    uint32_t last_rpm_calc_time;
    bool encoder_fault;
    bool spindle_stall_detected;
} spindle_encoder_t;

static spindle_encoder_t spindle_encoder = {0};

// Phase 2: Precision Enhancement Variables
typedef struct {
    float x_backlash;
    float y_backlash;
    float z_backlash;
    int8_t x_direction;
    int8_t y_direction;
    int8_t z_direction;
    bool compensation_active;
    uint32_t compensation_steps_x;
    uint32_t compensation_steps_y;
    uint32_t compensation_steps_z;
} backlash_compensation_t;

static backlash_compensation_t backlash = {
    .x_backlash = 0.01f,
    .y_backlash = 0.01f,
    .z_backlash = 0.02f,
    .compensation_active = true
};

// Phase 3: Advanced Features
typedef struct {
    float target_rpm;
    float actual_rpm;
    float rpm_error;
    float pwm_output;
    bool closed_loop_enabled;
    bool rpm_stable;
    uint32_t last_encoder_time;
    uint32_t encoder_count;
    float rpm_tolerance;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_integral;
    float pid_last_error;
} spindle_control_t;

static spindle_control_t spindle_ctrl = {
    .rpm_tolerance = 50.0f,
    .pid_kp = 0.1f,
    .pid_ki = 0.01f,
    .pid_kd = 0.001f
};

// Advanced features (tool change, workspace coordinates, probing)
// are implemented through grblHAL core and plugin system

// Constants
#define LIMIT_DEBOUNCE_MS           5
#define SPINDLE_ENCODER_PPR         60
#define SPINDLE_MAX_SAFE_RPM        24000

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

// Forward declarations for spindle functions
static void spindle_pid_controller(void);
static void spindle_calculate_rpm(void);

// Driver initialization

/* Watchdog timer using direct register access */
static bool watchdog_enabled = false;

/* IWDG register definitions for STM32G0xx */
#define IWDG_KEY_RELOAD    0x0000AAAA
#define IWDG_KEY_ENABLE    0x0000CCCC
#define IWDG_KEY_WRITE     0x00005555

// Watchdog implementation commented out due to missing peripheral definitions
/*
static bool watchdog_init(uint32_t timeout_ms)
{
    uint32_t reload = (timeout_ms > 4095) ? 4095 : timeout_ms;
    
    IWDG->KR = IWDG_KEY_WRITE;
    IWDG->PR = 3;  // Prescaler /32
    IWDG->RLR = reload;
    while (IWDG->SR & (IWDG_SR_RVU | IWDG_SR_PVU));
    IWDG->KR = IWDG_KEY_RELOAD;
    IWDG->KR = IWDG_KEY_ENABLE;
    
    watchdog_enabled = true;
    return true;
}

static void watchdog_reset(void)
{
    if (watchdog_enabled) {
        IWDG->KR = IWDG_KEY_RELOAD;
    }
}
*/

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
    
    // Register PWM spindle using spindle_register() function
    static const spindle_ptrs_t spindle_pwm = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.at_speed = On,
        .cap.direction = On,
        .config = NULL,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .get_pwm = NULL,        // Using default PWM calculation
        .update_pwm = NULL,     // Using direct register update
        .update_rpm = spindleUpdateRPM
    };
    
    spindle_register(&spindle_pwm, "PWM");
    
    // Register control system functions
    hal.control.get_state = systemGetState;
    hal.control.interrupt_callback = NULL; // Will be set by grblHAL core when needed
    
    // Register timer functions
    hal.get_elapsed_ticks = getElapsedTicks;

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

    // Initialize watchdog timer with 2 second timeout
    // This provides protection against system hangs while allowing
    // enough time for normal operations
    // NOTE: Disabled due to missing IWDG peripheral definitions
    // if (!watchdog_init(2000)) {
    //     // Watchdog initialization failed - continue without it
    //     // but log a warning if possible
    //     watchdog_enabled = false;
    // }

    return true;
}

// Driver delay function with optional callback support

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
    // Enable all stepper drivers (active low)
    DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, 0);  // Enable X stepper
    DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, 0);  // Enable Y stepper
    DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, 0);  // Enable Z stepper
    
#ifdef M3_AVAILABLE
    DIGITAL_OUT(M3_ENABLE_PORT, M3_ENABLE_PIN, 0);  // Enable E0 motor if available
#endif
}

static void stepperGoIdle (bool clear_signals)
{
    // Disable all stepper drivers (active low - set high to disable)
    DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, 1);  // Disable X stepper
    DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, 1);  // Disable Y stepper
    DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, 1);  // Disable Z stepper
    
#ifdef M3_AVAILABLE
    DIGITAL_OUT(M3_ENABLE_PORT, M3_ENABLE_PIN, 1);  // Disable E0 motor if available
#endif
    
    if (clear_signals) {
        // Clear all step and direction signals when going idle
        DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, 0);
        DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, 0);
        DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, 0);
        
        DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, 0);
        DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, 0);
        DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, 0);
        
#ifdef M3_AVAILABLE
        DIGITAL_OUT(M3_STEP_PORT, M3_STEP_PIN, 0);
        DIGITAL_OUT(M3_DIRECTION_PORT, M3_DIRECTION_PIN, 0);
#endif
    }
}

// Phase 2: Enhanced stepper enable with backlash compensation
static void stepperEnable (axes_signals_t enable, bool hold)
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
    
    // Phase 2: Reset backlash compensation when motors disabled
    if (!enable.value) {
        backlash.compensation_steps_x = 0;
        backlash.compensation_steps_y = 0;
        backlash.compensation_steps_z = 0;
    }
}

static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    // Configure TIM3 for stepper timing (separate from TIM2 spindle functions)
    
    // Enable TIM3 clock (TIM3EN is bit 1 in APBENR1)
    RCC->APBENR1 |= (1<<1);
    
    // Stop timer during configuration
    TIM3->CR1 = 0;
    
    // Configure timer for stepper step generation
    TIM3->PSC = STEPPER_TIMER_DIV - 1;  // Prescaler for 16MHz timer clock (64MHz/4)
    TIM3->ARR = cycles_per_tick - 1;    // Period for step pulse timing
    TIM3->CNT = 0;                      // Reset counter
    
    // Configure for one-pulse mode (timer stops after each step)
    TIM3->CR1 |= (1<<3);               // OPM - One-pulse mode
    
    // Enable update interrupt for step pulse end detection
    TIM3->DIER |= (1<<0);              // UIE - Update interrupt enable
    TIM3->SR = 0;                      // Clear all flags
    
    // Configure NVIC for TIM3 interrupt
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);  // High priority for real-time motion
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
    // Timer will be started by stepperPulseStart when needed
    
    // Timer configured for specified cycles per tick
}

// Phase 2: Apply backlash compensation
static void apply_backlash_compensation(stepper_t *stepper)
{
    if (!backlash.compensation_active) return;
    
    // Check for direction changes and apply compensation
    int8_t x_dir = stepper->dir_out.x ? 1 : -1;
    int8_t y_dir = stepper->dir_out.y ? 1 : -1;
    int8_t z_dir = stepper->dir_out.z ? 1 : -1;
    
    // X-axis backlash compensation
    if (backlash.x_direction != 0 && backlash.x_direction != x_dir && stepper->step_out.x) {
        uint32_t comp_steps = (uint32_t)(backlash.x_backlash * 80.0f);  // Fixed steps/mm
        backlash.compensation_steps_x = comp_steps;
    }
    if (stepper->step_out.x) backlash.x_direction = x_dir;
    
    // Y-axis backlash compensation
    if (backlash.y_direction != 0 && backlash.y_direction != y_dir && stepper->step_out.y) {
        uint32_t comp_steps = (uint32_t)(backlash.y_backlash * 80.0f);
        backlash.compensation_steps_y = comp_steps;
    }
    if (stepper->step_out.y) backlash.y_direction = y_dir;
    
    // Z-axis backlash compensation
    if (backlash.z_direction != 0 && backlash.z_direction != z_dir && stepper->step_out.z) {
        uint32_t comp_steps = (uint32_t)(backlash.z_backlash * 400.0f);  // Lead screw
        backlash.compensation_steps_z = comp_steps;
    }
    if (stepper->step_out.z) backlash.z_direction = z_dir;
}

static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        
        // Phase 2: Apply backlash compensation
        apply_backlash_compensation(stepper);
        
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
        // Phase 2: Process backlash compensation steps
        axes_signals_t comp_steps = {0};
        if (backlash.compensation_steps_x > 0) {
            comp_steps.x = 1;
            backlash.compensation_steps_x--;
        }
        if (backlash.compensation_steps_y > 0) {
            comp_steps.y = 1;
            backlash.compensation_steps_y--;
        }
        if (backlash.compensation_steps_z > 0) {
            comp_steps.z = 1;
            backlash.compensation_steps_z--;
        }
        
        // Combine regular steps with compensation steps
        axes_signals_t total_steps;
        total_steps.x = stepper->step_out.x || comp_steps.x;
        total_steps.y = stepper->step_out.y || comp_steps.y;
        total_steps.z = stepper->step_out.z || comp_steps.z;
        
#if STEP_OUTMODE == GPIO_MAP
        // Use lookup table for efficient port-wide step pulse
        STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[total_steps.value];
#else
        // Individual step pin control
        DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, total_steps.x);
        DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, total_steps.y);
        DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, total_steps.z);
#endif
        
        // Start stepper timer for step pulse timing
        // TIM3 is configured in one-pulse mode and will generate interrupt when pulse ends
        TIM3->CNT = 0;              // Reset counter
        TIM3->CR1 |= (1<<0);        // CEN - Start timer
    }
}

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    if (on) {
        // Configure limit switch pins as inputs with pull-up
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = X_LIMIT_BIT | Y_LIMIT_BIT | Z_LIMIT_BIT;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Input mode for now
        GPIO_InitStruct.Pull = GPIO_PULLUP;           // Internal pull-up
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(LIMIT_PORT, &GPIO_InitStruct);
        
        // Enable EXTI interrupts for limit switches
        HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);   // PC0, PC1
        HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
        
        HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);   // PC2
        HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
        
        // Reset debouncing state
        memset(&safety_state, 0, sizeof(safety_state));
        safety_state.last_read_time = HAL_GetTick();
    } else {
        // Disable interrupts
        HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
        HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
        
        // Configure pins as regular inputs
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = X_LIMIT_BIT | Y_LIMIT_BIT | Z_LIMIT_BIT;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(LIMIT_PORT, &GPIO_InitStruct);
    }
}

// Phase 1: Enhanced limit switch debouncing
static axes_signals_t limitsGetRawState(void)
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

static limit_signals_t limitsGetState (void)
{
    uint32_t current_time = uwTick;
    
    // Simple debouncing without advanced safety features
    if (current_time - safety_state.last_read_time >= LIMIT_DEBOUNCE_MS) {
        axes_signals_t raw_signals = limitsGetRawState();
        
        // Update history
        safety_state.history[safety_state.history_index] = raw_signals;
        safety_state.history_index = (safety_state.history_index + 1) % 8;
        
        // Check consistency
        bool x_consistent = true, y_consistent = true, z_consistent = true;
        for (uint8_t i = 1; i < 8; i++) {
            if (safety_state.history[i].x != safety_state.history[0].x) x_consistent = false;
            if (safety_state.history[i].y != safety_state.history[0].y) y_consistent = false;
            if (safety_state.history[i].z != safety_state.history[0].z) z_consistent = false;
        }
        
        // Update debounced state
        if (x_consistent) safety_state.debounced_state.x = safety_state.history[0].x;
        if (y_consistent) safety_state.debounced_state.y = safety_state.history[0].y;
        if (z_consistent) safety_state.debounced_state.z = safety_state.history[0].z;
        
        safety_state.last_read_time = current_time;
    }
    
    // Convert axes_signals_t to limit_signals_t for HAL compatibility
    limit_signals_t limit_state = {0};
    limit_state.min.x = safety_state.debounced_state.x;
    limit_state.min.y = safety_state.debounced_state.y;
    limit_state.min.z = safety_state.debounced_state.z;
    
    return limit_state;
}

static void coolantSetState (coolant_state_t mode)
{
#if COOLANT_ENABLE & COOLANT_FLOOD
    // Control flood coolant (M7/M8/M9 commands)
    HAL_GPIO_WritePin(COOLANT_FLOOD_PORT, 1<<COOLANT_FLOOD_PIN, 
                      mode.flood ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

#if COOLANT_ENABLE & COOLANT_MIST  
    // Control mist coolant (M7/M9 commands)
    HAL_GPIO_WritePin(COOLANT_MIST_PORT, 1<<COOLANT_MIST_PIN,
                      mode.mist ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};
    
#if COOLANT_ENABLE & COOLANT_FLOOD
    // Read flood coolant state
    state.flood = HAL_GPIO_ReadPin(COOLANT_FLOOD_PORT, 1<<COOLANT_FLOOD_PIN) == GPIO_PIN_SET;
#endif

#if COOLANT_ENABLE & COOLANT_MIST
    // Read mist coolant state  
    state.mist = HAL_GPIO_ReadPin(COOLANT_MIST_PORT, 1<<COOLANT_MIST_PIN) == GPIO_PIN_SET;
#endif
    
    return state;
}

static void probeConfigureInvertMask (bool is_probe_away, bool probing)
{
    probe_invert = is_probe_away;
    
#if PROBE_ENABLE
    // Configure probe pin as input with pull-up
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = 1<<PROBE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PROBE_PORT, &GPIO_InitStruct);
#endif
}

static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};
    
#if PROBE_ENABLE
    state.triggered = HAL_GPIO_ReadPin(PROBE_PORT, 1<<PROBE_PIN) != GPIO_PIN_RESET;
    
    // Apply probe invert setting
    if (probe_invert)
        state.triggered = !state.triggered;
#endif
    
    return state;
}

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};
    
#ifdef SPINDLE_ENABLE_PIN
    state.on = HAL_GPIO_ReadPin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) != 0;
#endif
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = HAL_GPIO_ReadPin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) != 0;
#endif
    
    return state;
}

static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
#ifdef SPINDLE_PWM_PIN
    // RPM to PWM conversion
    uint16_t pwm_value = (uint16_t)((rpm / 24000.0f) * 999.0f);
    if(pwm_value > 999) pwm_value = 999;
    
    // Update PWM register
    TIM2->CCR2 = pwm_value;
    spindle_encoder.target_rpm = (uint32_t)rpm;
#endif
}

// Phase 3: Advanced spindle control
static void spindleSetState (spindle_ptrs_t *spindle_ptr, spindle_state_t state, float rpm)
{
    spindle_ctrl.target_rpm = rpm;
    
    if (state.on) {
#ifdef SPINDLE_ENABLE_PIN
        HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_PIN_SET);
#endif
#ifdef SPINDLE_DIRECTION_PIN
        HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, 
                         state.ccw ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
        
        if (rpm > 0.0f) {
            spindle_ctrl.pwm_output = (rpm / 24000.0f) * 100.0f;
            if (spindle_ctrl.pwm_output > 100.0f) spindle_ctrl.pwm_output = 100.0f;
            
            uint32_t ccr_value = (uint32_t)((spindle_ctrl.pwm_output / 100.0f) * 999.0f);
            TIM2->CCR2 = ccr_value;
            
            spindle_ctrl.closed_loop_enabled = true;
            spindle_ctrl.pid_integral = 0.0f;
        }
    } else {
#ifdef SPINDLE_ENABLE_PIN
        HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_PIN_RESET);
#endif
        spindle_ctrl.pwm_output = 0.0f;
        spindle_ctrl.closed_loop_enabled = false;
        TIM2->CCR2 = 0;
    }
}

// Phase 3: PID controller for spindle
static void spindle_pid_controller(void)
{
    if (!spindle_ctrl.closed_loop_enabled) return;
    
    spindle_ctrl.rpm_error = spindle_ctrl.target_rpm - spindle_ctrl.actual_rpm;
    spindle_ctrl.pid_integral += spindle_ctrl.rpm_error;
    
    // Integral windup protection
    if (spindle_ctrl.pid_integral > 1000.0f) spindle_ctrl.pid_integral = 1000.0f;
    if (spindle_ctrl.pid_integral < -1000.0f) spindle_ctrl.pid_integral = -1000.0f;
    
    float derivative = spindle_ctrl.rpm_error - spindle_ctrl.pid_last_error;
    
    float pid_output = (spindle_ctrl.pid_kp * spindle_ctrl.rpm_error) +
                      (spindle_ctrl.pid_ki * spindle_ctrl.pid_integral) +
                      (spindle_ctrl.pid_kd * derivative);
    
    spindle_ctrl.pwm_output += pid_output;
    
    if (spindle_ctrl.pwm_output > 100.0f) spindle_ctrl.pwm_output = 100.0f;
    if (spindle_ctrl.pwm_output < 0.0f) spindle_ctrl.pwm_output = 0.0f;
    
    uint32_t ccr_value = (uint32_t)((spindle_ctrl.pwm_output / 100.0f) * 999.0f);
    TIM2->CCR2 = ccr_value;
    
    spindle_ctrl.rpm_stable = (fabsf(spindle_ctrl.rpm_error) <= spindle_ctrl.rpm_tolerance);
    spindle_ctrl.pid_last_error = spindle_ctrl.rpm_error;
}

// Phase 3: Calculate spindle RPM from encoder
static void spindle_calculate_rpm(void)
{
    uint32_t current_time = getElapsedTicks();
    uint32_t time_diff = current_time - spindle_ctrl.last_encoder_time;
    
    if (time_diff >= 100) {  // Update every 100ms
        float rpm = (spindle_ctrl.encoder_count * 60000.0f) / (float)time_diff;
        spindle_ctrl.actual_rpm = (spindle_ctrl.actual_rpm * 0.8f) + (rpm * 0.2f);
        spindle_ctrl.encoder_count = 0;
        spindle_ctrl.last_encoder_time = current_time;
    }
}

// TIM3 interrupt handler for stepper timer
void TIM3_IRQHandler(void)
{
    // Handle stepper timer (Update interrupt)
    if (TIM3->SR & (1<<0)) {  // UIF bit
        TIM3->SR &= ~(1<<0);  // Clear update interrupt flag
        
        // Clear step pins (end of step pulse)
        STEP_PORT->BRR = X_STEP_BIT | Y_STEP_BIT | Z_STEP_BIT;
#ifdef M3_AVAILABLE
        HAL_GPIO_WritePin(M3_STEP_PORT, 1<<M3_STEP_PIN, GPIO_PIN_RESET);
#endif
        
        // Timer automatically stops due to one-pulse mode (OPM)
        
        // Signal completion to grblHAL stepper ISR
        if (hal.stepper.interrupt_callback)
            hal.stepper.interrupt_callback();
    }
}

// TIM2 interrupt handler for spindle encoder only
void TIM2_IRQHandler(void)
{
    // Handle spindle encoder (Capture Compare interrupt)
    if (TIM2->SR & (1<<1)) {  // CC1IF bit
        spindle_ctrl.encoder_count++;
        spindle_ctrl.last_encoder_time = HAL_GetTick();
        TIM2->SR &= ~(1<<1);  // Clear CC1 interrupt flag
    }
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};
    
#if CONTROL_ENABLE & CONTROL_HALT
    signals.reset = !HAL_GPIO_ReadPin(RESET_PORT, 1<<RESET_PIN);
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
    signals.feed_hold = !HAL_GPIO_ReadPin(FEED_HOLD_PORT, 1<<FEED_HOLD_PIN);
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
    signals.cycle_start = !HAL_GPIO_ReadPin(CYCLE_START_PORT, 1<<CYCLE_START_PIN);
#endif
#if SAFETY_DOOR_ENABLE
    signals.safety_door_ajar = !HAL_GPIO_ReadPin(SAFETY_DOOR_PORT, 1<<SAFETY_DOOR_PIN);
#endif
    
    // Apply invert mask if configured
    if (settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;
        
    return signals;
}

static uint32_t getElapsedTicks (void)
{
    return uwTick;
}

// Settings management
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    // Update backlash compensation when settings change
    // Basic settings update - specific flags may not be available in this grblHAL version
    // Re-configure GPIO pins when settings change
    // driver_setup(settings);  // Temporarily disabled to avoid recursion
}

// Driver setup and GPIO initialization
bool driver_setup (settings_t *settings)
{
    // Initialize GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Initialize stepper motor pins
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // Step pins
    GPIO_InitStruct.Pin = X_STEP_BIT;
    HAL_GPIO_Init(X_STEP_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Y_STEP_BIT;
    HAL_GPIO_Init(Y_STEP_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Z_STEP_BIT;
    HAL_GPIO_Init(Z_STEP_PORT, &GPIO_InitStruct);
    
    // Direction pins
    GPIO_InitStruct.Pin = X_DIRECTION_BIT;
    HAL_GPIO_Init(X_DIRECTION_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Y_DIRECTION_BIT;
    HAL_GPIO_Init(Y_DIRECTION_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Z_DIRECTION_BIT;
    HAL_GPIO_Init(Z_DIRECTION_PORT, &GPIO_InitStruct);
    
    // Enable pins (start disabled - high for TMC drivers)
    GPIO_InitStruct.Pin = X_ENABLE_BIT;
    HAL_GPIO_Init(X_ENABLE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(X_ENABLE_PORT, X_ENABLE_BIT, GPIO_PIN_SET);  // Disabled
    
    GPIO_InitStruct.Pin = Y_ENABLE_BIT;
    HAL_GPIO_Init(Y_ENABLE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(Y_ENABLE_PORT, Y_ENABLE_BIT, GPIO_PIN_SET);  // Disabled
    
    GPIO_InitStruct.Pin = Z_ENABLE_BIT;
    HAL_GPIO_Init(Z_ENABLE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(Z_ENABLE_PORT, Z_ENABLE_BIT, GPIO_PIN_SET);  // Disabled
    
#if COOLANT_ENABLE & COOLANT_FLOOD
    GPIO_InitStruct.Pin = 1<<COOLANT_FLOOD_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(COOLANT_FLOOD_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(COOLANT_FLOOD_PORT, 1<<COOLANT_FLOOD_PIN, GPIO_PIN_RESET);
#endif

#if COOLANT_ENABLE & COOLANT_MIST
    GPIO_InitStruct.Pin = 1<<COOLANT_MIST_PIN;
    HAL_GPIO_Init(COOLANT_MIST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(COOLANT_MIST_PORT, 1<<COOLANT_MIST_PIN, GPIO_PIN_RESET);
#endif

    // Initialize spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
    GPIO_InitStruct.Pin = 1<<SPINDLE_ENABLE_PIN;
    HAL_GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, 1<<SPINDLE_ENABLE_PIN, GPIO_PIN_RESET);
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    GPIO_InitStruct.Pin = 1<<SPINDLE_DIRECTION_PIN;
    HAL_GPIO_Init(SPINDLE_DIRECTION_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, 1<<SPINDLE_DIRECTION_PIN, GPIO_PIN_RESET);
#endif

    // Initialize control input pins
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    
#if CONTROL_ENABLE & CONTROL_HALT
    GPIO_InitStruct.Pin = 1<<RESET_PIN;
    HAL_GPIO_Init(RESET_PORT, &GPIO_InitStruct);
#endif

#if CONTROL_ENABLE & CONTROL_FEED_HOLD
    GPIO_InitStruct.Pin = 1<<FEED_HOLD_PIN;
    HAL_GPIO_Init(FEED_HOLD_PORT, &GPIO_InitStruct);
#endif

#if CONTROL_ENABLE & CONTROL_CYCLE_START
    GPIO_InitStruct.Pin = 1<<CYCLE_START_PIN;
    HAL_GPIO_Init(CYCLE_START_PORT, &GPIO_InitStruct);
#endif

#if SAFETY_DOOR_ENABLE
    GPIO_InitStruct.Pin = 1<<SAFETY_DOOR_PIN;
    HAL_GPIO_Init(SAFETY_DOOR_PORT, &GPIO_InitStruct);
#endif

    // Initialize limit switch pins as inputs with pull-ups
    GPIO_InitStruct.Pin = X_LIMIT_BIT | Y_LIMIT_BIT | Z_LIMIT_BIT;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LIMIT_PORT, &GPIO_InitStruct);

#if PROBE_ENABLE
    // Initialize probe pin as input with pull-up
    GPIO_InitStruct.Pin = 1<<PROBE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PROBE_PORT, &GPIO_InitStruct);
#endif

    // Initialize control input pins as regular inputs
    // EXTI configuration will be handled by grblHAL core when needed
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

#if CONTROL_ENABLE & CONTROL_RESET
    // Reset/Emergency Stop pin (PC15)
    GPIO_InitStruct.Pin = 1<<15;  // PC15
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

#if CONTROL_ENABLE & CONTROL_FEED_HOLD
    // Feed Hold pin (PC13)  
    GPIO_InitStruct.Pin = 1<<13;  // PC13
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

#if CONTROL_ENABLE & CONTROL_CYCLE_START
    // Cycle Start pin (PC12)
    GPIO_InitStruct.Pin = 1<<12;  // PC12
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

#if SAFETY_DOOR_ENABLE
    // Safety Door pin (PC3)
    GPIO_InitStruct.Pin = 1<<3;   // PC3
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif
    
    // Configure NVIC for control pin interrupts (will be enabled by grblHAL when needed)
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
    
    return true;
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
    // GPIO interrupt configuration for STM32G0
    // Note: Full EXTI configuration is handled in driver_setup() for control pins
    // and limitsEnable() for limit switches. This function provides NVIC setup.
    
    // Enable the appropriate NVIC interrupt based on pin number
    if (input->pin <= 1) {
        HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
    } else if (input->pin <= 3) {
        HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
    } else {
        HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
}

// Tool change, workspace, and advanced probing functions are implemented
// through grblHAL core functionality and plugin system

// Duplicate functions removed - implementations above are used

// Systick interrupt handler
void SysTick_Handler (void)
{
    uwTick++;
    
    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }

    HAL_IncTick();
    
    // Phase 3: Update advanced features
    spindle_calculate_rpm();
    spindle_pid_controller();
    
    // Reset watchdog timer every millisecond to prevent system reset
    // The watchdog has a 2 second timeout, so this provides plenty of margin
    // NOTE: Disabled due to missing IWDG peripheral definitions
    // static uint32_t watchdog_counter = 0;
    // if (++watchdog_counter >= 100) {  // Reset every 100ms
    //     watchdog_counter = 0;
    //     watchdog_reset();
    // }
    
    if(systick_isr)
        systick_isr();
}

// Limit switch interrupt handlers
void EXTI0_1_IRQHandler(void)
{
    // Simplified limit switch interrupt handling for STM32G0
    // Use HAL generic interrupt handler for proper flag clearing
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    
    // Get limit state and call callback
    if (hal.limits.interrupt_callback) {
        limit_signals_t limit_state = limitsGetState();
        hal.limits.interrupt_callback(limit_state);
    }
}

void EXTI2_3_IRQHandler(void)
{
    // Simplified limit switch interrupt handling for STM32G0  
    // Use HAL generic interrupt handler for proper flag clearing
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    
    // Get limit state and call callback
    if (hal.limits.interrupt_callback) {
        limit_signals_t limit_state = limitsGetState();
        hal.limits.interrupt_callback(limit_state);
    }
    
    // Handle control signal (safety door) callback if PC3 triggered
    if (hal.control.interrupt_callback) {
        hal.control.interrupt_callback(systemGetState());
    }
}

// Control signal interrupt handler for safety systems
void EXTI4_15_IRQHandler(void)
{
    // Handle control signal interrupts (PC12-PC15: Cycle Start, Feed Hold, Reset/E-Stop)
    // Use HAL generic interrupt handler for proper flag clearing
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);  // Cycle Start
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);  // Feed Hold
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);  // Reset/Emergency Stop
    
    // Call grblHAL control interrupt callback with current state
    if (hal.control.interrupt_callback) {
        hal.control.interrupt_callback(systemGetState());
    }
}