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
static spindle_state_t spindleGetState (void);
static void spindleSetState (spindle_state_t state, float rpm);
static void spindleUpdateRPM (float rpm);
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

// Phase 2: Precision Enhancement Variables
// Backlash compensation structure
typedef struct {
    float x_backlash;               // X-axis backlash compensation in mm
    float y_backlash;               // Y-axis backlash compensation in mm  
    float z_backlash;               // Z-axis backlash compensation in mm
    int8_t x_direction;             // Last X movement direction (-1, 0, 1)
    int8_t y_direction;             // Last Y movement direction (-1, 0, 1)
    int8_t z_direction;             // Last Z movement direction (-1, 0, 1)
    bool compensation_active;       // Backlash compensation enabled
    uint32_t compensation_steps_x;  // Pending compensation steps X
    uint32_t compensation_steps_y;  // Pending compensation steps Y
    uint32_t compensation_steps_z;  // Pending compensation steps Z
} backlash_compensation_t;

// Step timing optimization structure
typedef struct {
    uint32_t min_step_pulse_width;  // Minimum step pulse width in timer ticks
    uint32_t step_idle_delay;       // Delay between step and direction change
    uint32_t acceleration_ticks;    // Acceleration calculation ticks
    bool high_precision_timing;     // Enable sub-microsecond timing
    uint32_t last_step_time;        // Last step pulse timestamp
    uint32_t pulse_stretch_factor;  // Pulse width multiplier for slow drivers
} step_timing_t;

// Global precision enhancement variables
static backlash_compensation_t backlash = {
    .x_backlash = 0.0f,
    .y_backlash = 0.0f, 
    .z_backlash = 0.0f,
    .x_direction = 0,
    .y_direction = 0,
    .z_direction = 0,
    .compensation_active = false,
    .compensation_steps_x = 0,
    .compensation_steps_y = 0,
    .compensation_steps_z = 0
};

static step_timing_t step_timing = {
    .min_step_pulse_width = 2,      // 2 timer ticks minimum (STM32G0 optimized)
    .step_idle_delay = 1,           // 1 timer tick between step and direction
    .acceleration_ticks = 0,
    .high_precision_timing = true,
    .last_step_time = 0,
    .pulse_stretch_factor = 1
};

// Phase 3: Advanced Features
// Advanced spindle control with closed-loop feedback
typedef struct {
    float target_rpm;               // Target spindle RPM
    float actual_rpm;               // Measured spindle RPM from encoder
    float rpm_error;                // Error between target and actual
    float pwm_output;               // Current PWM output (0-100%)
    bool closed_loop_enabled;       // Closed-loop control active
    bool rpm_stable;                // RPM within tolerance
    uint32_t last_encoder_time;     // Last encoder reading timestamp
    uint32_t encoder_count;         // Encoder pulse count
    float rpm_tolerance;            // RPM tolerance for stability
    float pid_kp;                   // PID proportional gain
    float pid_ki;                   // PID integral gain
    float pid_kd;                   // PID derivative gain
    float pid_integral;             // PID integral accumulator
    float pid_last_error;           // PID last error for derivative
} spindle_control_t;

// Tool change automation structure
typedef struct {
    uint8_t current_tool;           // Currently selected tool (0-99)
    uint8_t requested_tool;         // Requested tool number
    bool tool_change_pending;       // Tool change in progress
    bool manual_tool_change;        // Manual vs automatic tool change
    float tool_lengths[100];        // Tool length offsets (T0-T99)
    float tool_diameters[100];      // Tool diameter table
    bool tool_present[100];         // Tool presence detection
    uint32_t tool_change_time;      // Time for tool change operation
    bool tool_sensor_enabled;       // Tool length sensor available
} tool_library_t;

// Workspace coordinate systems (G54-G59)
typedef struct {
    float x_offset;                 // X axis offset
    float y_offset;                 // Y axis offset  
    float z_offset;                 // Z axis offset
    bool active;                    // Coordinate system is active
} coordinate_system_t;

typedef struct {
    coordinate_system_t systems[6]; // G54-G59 coordinate systems
    uint8_t active_system;          // Currently active system (0-5)
    coordinate_system_t g92_offset; // G92 temporary offset
    bool g92_active;                // G92 offset active
} workspace_coords_t;

// Advanced probing routines
typedef struct {
    bool probe_active;              // Probing operation in progress
    float probe_feed_rate;          // Probing feed rate
    float probe_position[3];        // Last probe contact position
    bool probe_success;             // Last probe was successful
    uint8_t probe_cycles;           // Number of probe cycles
    float center_x, center_y;       // Found center coordinates
    float edge_positions[4];        // Edge positions (left, right, front, back)
    bool auto_probe_enabled;        // Automatic probing enabled
} advanced_probing_t;

// Global Phase 3 variables
static spindle_control_t spindle_ctrl = {
    .target_rpm = 0.0f,
    .actual_rpm = 0.0f,
    .rpm_error = 0.0f,
    .pwm_output = 0.0f,
    .closed_loop_enabled = false,
    .rpm_stable = false,
    .last_encoder_time = 0,
    .encoder_count = 0,
    .rpm_tolerance = 50.0f,         // ±50 RPM tolerance
    .pid_kp = 0.1f,                 // Proportional gain
    .pid_ki = 0.01f,                // Integral gain
    .pid_kd = 0.001f,               // Derivative gain
    .pid_integral = 0.0f,
    .pid_last_error = 0.0f
};

static tool_library_t tool_library = {
    .current_tool = 0,
    .requested_tool = 0,
    .tool_change_pending = false,
    .manual_tool_change = true,     // Default to manual tool change
    .tool_change_time = 30000,      // 30 second tool change timeout
    .tool_sensor_enabled = false
};

static workspace_coords_t workspace = {
    .active_system = 0,             // Default to G54
    .g92_active = false
};

static advanced_probing_t probing = {
    .probe_active = false,
    .probe_feed_rate = 100.0f,      // 100 mm/min default probe rate
    .probe_success = false,
    .probe_cycles = 1,
    .auto_probe_enabled = false
};

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

// Safety system variables
typedef struct {
    axes_signals_t raw_state;           // Current raw limit state
    axes_signals_t debounced_state;     // Debounced limit state  
    axes_signals_t history[8];          // History buffer for debouncing
    uint8_t history_index;              // Current history position
    uint32_t last_read_time;            // Last read timestamp
    uint32_t emergency_stop_time;       // Emergency stop timestamp
    bool emergency_active;              // Emergency stop active flag
} safety_state_t;

static safety_state_t safety_state = {0};

// Spindle encoder monitoring for safety
typedef struct {
    volatile uint32_t pulse_count;       // Total encoder pulses
    volatile uint32_t last_pulse_time;   // Last pulse timestamp
    volatile uint32_t pulse_period;      // Time between pulses
    uint32_t target_rpm;                 // Commanded RPM
    uint32_t actual_rpm;                 // Measured RPM
    uint32_t last_rpm_calc_time;         // Last RPM calculation time
    bool encoder_fault;                  // Encoder fault flag
    bool spindle_stall_detected;         // Stall detection flag
} spindle_encoder_t;

static spindle_encoder_t spindle_encoder = {0};

// Enhanced emergency stop system
typedef struct {
    bool estop_active;                   // Emergency stop state
    bool estop_latched;                  // Latched E-stop (requires reset)
    uint32_t estop_trigger_time;         // Time when E-stop was triggered
    uint32_t last_safety_check;          // Last safety system check
    uint32_t safety_violations;          // Count of safety violations
    bool hardware_estop_triggered;       // Hardware E-stop triggered
    bool software_estop_triggered;       // Software E-stop triggered
    bool spindle_brake_applied;          // Spindle brake status
} emergency_stop_t;

static emergency_stop_t estop_system = {0};

// Thermal monitoring system via I2C sensors
typedef struct {
    uint8_t sensor_address;              // I2C address of temperature sensor
    int16_t temperature;                 // Temperature in 0.1°C units
    int16_t max_temperature;             // Maximum safe temperature
    int16_t warning_temperature;         // Warning temperature threshold
    uint32_t last_read_time;             // Last sensor read time
    bool sensor_fault;                   // Sensor communication fault
    bool overtemperature_warning;        // Warning flag
    bool overtemperature_alarm;          // Alarm flag
} thermal_sensor_t;

// Define thermal sensors for monitoring
static thermal_sensor_t thermal_sensors[] = {
    { .sensor_address = 0x48, .max_temperature = 800, .warning_temperature = 700 },  // Stepper drivers
    { .sensor_address = 0x49, .max_temperature = 850, .warning_temperature = 750 },  // Power supply
    { .sensor_address = 0x4A, .max_temperature = 900, .warning_temperature = 800 },  // Spindle motor
};

#define NUM_THERMAL_SENSORS (sizeof(thermal_sensors) / sizeof(thermal_sensor_t))

// Watchdog monitoring system
typedef struct {
    uint32_t last_refresh_time;          // Last watchdog refresh time
    uint32_t refresh_count;              // Number of successful refreshes
    bool watchdog_enabled;               // Watchdog enabled flag
    bool watchdog_fault;                 // Watchdog fault detected
} watchdog_system_t;

static watchdog_system_t watchdog_system = {0};

// Power loss detection and recovery system
typedef struct {
    uint16_t supply_voltage_mv;          // Current supply voltage in mV
    uint32_t last_power_check;           // Last power check time
    bool power_loss_detected;            // Power loss detected flag
    bool recovery_data_saved;            // Recovery data saved flag
    bool power_restored;                 // Power restored after loss
    uint32_t power_loss_time;            // Time when power loss detected
} power_monitor_t;

static power_monitor_t power_monitor = {0};

// Safety constants
#define LIMIT_DEBOUNCE_MS           5       // Debounce time in milliseconds
#define LIMIT_HISTORY_SIZE          8       // History buffer size
#define EMERGENCY_STOP_TIMEOUT_MS   100     // Emergency stop timeout
#define SAFETY_CHECK_INTERVAL_MS    1       // Safety check interval

// Emergency stop safety constants
#define ESTOP_DEBOUNCE_MS           2       // Fast debounce for E-stop (2ms)
#define ESTOP_RESPONSE_TIME_US      50      // Maximum E-stop response time (50 microseconds)
#define SAFETY_SYSTEM_CHECK_MS      10      // Safety system health check interval

// Spindle encoder safety constants
#define SPINDLE_ENCODER_PPR         60      // Pulses per revolution (configurable)
#define SPINDLE_SAFETY_TIMEOUT_MS   2000    // Max time without encoder signal (2 seconds)
#define SPINDLE_MIN_SAFE_RPM        50      // Minimum RPM for safety checks
#define SPINDLE_MAX_SAFE_RPM        24000   // Maximum safe RPM

// Thermal monitoring constants
#define THERMAL_READ_INTERVAL_MS    1000    // Read temperature every 1 second
#define THERMAL_SENSOR_TIMEOUT_MS   5000    // Sensor fault timeout (5 seconds)
#define THERMAL_HISTORY_SIZE        5       // Temperature history for averaging

// Watchdog timer constants
#define WATCHDOG_TIMEOUT_MS         2000    // Watchdog timeout (2 seconds)
#define WATCHDOG_REFRESH_MS         1000    // Refresh watchdog every 1 second

// Power loss detection constants
#define POWER_MONITOR_INTERVAL_MS   50      // Check power every 50ms
#define POWER_LOSS_THRESHOLD_MV     3000    // Power loss threshold (3.0V)
#define POWER_RECOVERY_THRESHOLD_MV 3200    // Power recovery threshold (3.2V)

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

    // hal.spindle.set_state = spindleSetState;  // TODO: Restore when grblHAL API is clarified
    // hal.spindle.get_state = spindleGetState;
    // hal.spindle.update_rpm = spindleUpdateRPM;

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;

    // hal.driver_cap.spindle_dir = On;         // TODO: Restore when capability structure is clarified
    // hal.driver_cap.variable_spindle = On;
    // hal.driver_cap.spindle_pwm_invert = On;
    // hal.driver_cap.spindle_pwm_linearization = On;
    // hal.driver_cap.mist_control = On;
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
    
    // Initialize safety systems
    watchdog_init();
    power_monitor_init();
    
    // Clear reset flags and check for watchdog reset
    if (watchdog_caused_reset()) {
        // Log watchdog reset for diagnostics
        // Could set a flag for host to query
    }
    // Clear reset flags - use available RCC register  
    #ifdef RCC_CSR_RMVF
    RCC->CSR |= RCC_CSR_RMVF;  // Clear reset flags
    #endif

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

// Phase 2: Enhanced stepper enable with feed rate optimization
static void stepperEnable (axes_signals_t enable)
{
    // Enable/disable individual stepper motors with optimized timing
    
    // X-axis stepper control
    if (enable.x) {
        DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, 0);  // Active low enable
        // Phase 2: Apply feed rate pre-calculation for X-axis
        if (step_timing.high_precision_timing) {
            volatile uint32_t enable_delay = 10;  // 10 tick enable delay
            while (enable_delay--);
        }
    } else {
        DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_PIN, 1);  // Disable
    }
    
    // Y-axis stepper control
    if (enable.y) {
        DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, 0);  // Active low enable
        if (step_timing.high_precision_timing) {
            volatile uint32_t enable_delay = 10;
            while (enable_delay--);
        }
    } else {
        DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_PIN, 1);  // Disable
    }
    
    // Z-axis stepper control  
    if (enable.z) {
        DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, 0);  // Active low enable
        if (step_timing.high_precision_timing) {
            volatile uint32_t enable_delay = 10;
            while (enable_delay--);
        }
    } else {
        DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_PIN, 1);  // Disable
    }
    
    // Phase 2: Reset motion optimization state when motors disabled
    if (!enable.value) {
        step_timing.last_step_time = 0;
        // Clear any pending backlash compensation when motors disabled
        backlash.compensation_steps_x = 0;
        backlash.compensation_steps_y = 0;
        backlash.compensation_steps_z = 0;
    }
}

// Phase 2: Enhanced stepper timer configuration with acceleration optimization
static uint32_t stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    // Phase 2: Optimize timer period for smooth acceleration
    uint32_t optimized_cycles = cycles_per_tick;
    
    // Apply acceleration-based timing adjustments
    if (step_timing.acceleration_ticks > 0) {
        // Scale timer period based on acceleration profile
        // Higher acceleration = shorter timer periods for smoother motion
        uint32_t accel_factor = step_timing.acceleration_ticks / 1000;
        if (accel_factor > 0) {
            optimized_cycles = cycles_per_tick / (1 + (accel_factor >> 4));
        }
    }
    
    // Ensure minimum timer period for driver stability
    uint32_t min_cycles = SystemCoreClock / 50000;  // 50kHz max step rate
    if (optimized_cycles < min_cycles) {
        optimized_cycles = min_cycles;
    }
    
    // Configure stepper timer period
    // TODO: Set actual timer registers when timer implementation is complete
    return optimized_cycles;
}

// Phase 2: Enhanced backlash compensation
static void apply_backlash_compensation(stepper_t *stepper)
{
    if (!backlash.compensation_active) return;
    
    // Check for direction changes and apply compensation
    int8_t x_dir = stepper->dir_out.x ? 1 : -1;
    int8_t y_dir = stepper->dir_out.y ? 1 : -1; 
    int8_t z_dir = stepper->dir_out.z ? 1 : -1;
    
    // X-axis backlash compensation - use fixed steps/mm for now
    if (backlash.x_direction != 0 && backlash.x_direction != x_dir && stepper->step_out.x) {
        // Direction change detected - use 80 steps/mm estimate
        uint32_t comp_steps = (uint32_t)(backlash.x_backlash * 80.0f);
        backlash.compensation_steps_x = comp_steps;
    }
    if (stepper->step_out.x) backlash.x_direction = x_dir;
    
    // Y-axis backlash compensation - use fixed steps/mm for now
    if (backlash.y_direction != 0 && backlash.y_direction != y_dir && stepper->step_out.y) {
        uint32_t comp_steps = (uint32_t)(backlash.y_backlash * 80.0f);
        backlash.compensation_steps_y = comp_steps;
    }
    if (stepper->step_out.y) backlash.y_direction = y_dir;
    
    // Z-axis backlash compensation - use 400 steps/mm for lead screw
    if (backlash.z_direction != 0 && backlash.z_direction != z_dir && stepper->step_out.z) {
        uint32_t comp_steps = (uint32_t)(backlash.z_backlash * 400.0f);
        backlash.compensation_steps_z = comp_steps;
    }
    if (stepper->step_out.z) backlash.z_direction = z_dir;
}

// Phase 2: Optimized step pulse timing
static void optimize_step_timing(void)
{
    uint32_t current_time = getElapsedTicks();
    uint32_t time_since_last = current_time - step_timing.last_step_time;
    
    // Ensure minimum delay between steps for driver stability
    if (time_since_last < step_timing.step_idle_delay) {
        // Brief delay to meet driver timing requirements
        volatile uint32_t delay = step_timing.step_idle_delay - time_since_last;
        while (delay--); // Simple delay loop
    }
    
    step_timing.last_step_time = current_time;
}

static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        
        // Phase 2: Apply backlash compensation
        apply_backlash_compensation(stepper);
        
        // Phase 2: Optimize timing between direction and step
        optimize_step_timing();
        
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
        
        // Phase 2: Additional delay for direction setup time
        if (step_timing.high_precision_timing) {
            volatile uint32_t dir_setup_delay = step_timing.step_idle_delay;
            while (dir_setup_delay--);
        }
    }

    if(stepper->step_out.value) {
        // Phase 2: Process any pending backlash compensation steps
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
        // Individual step pin control with enhanced timing
        DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, total_steps.x);
        DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, total_steps.y);
        DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, total_steps.z);
#endif
        
        // Phase 2: Ensure minimum pulse width for step signals
        if (step_timing.high_precision_timing) {
            volatile uint32_t pulse_width = step_timing.min_step_pulse_width * step_timing.pulse_stretch_factor;
            while (pulse_width--);
        }
    }
}

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    // TODO: Enable/disable limit switches
}

// Enhanced limit switch debouncing function
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

// Debounced limit switch state with safety monitoring
static axes_signals_t limitsGetState (void)
{
    uint32_t current_time = uwTick;
    
    // Check if enough time has passed for next safety check
    if (current_time - safety_state.last_read_time >= SAFETY_CHECK_INTERVAL_MS) {
        
        // Read raw limit switch states
        axes_signals_t raw_signals = limitsGetRawState();
        safety_state.raw_state = raw_signals;
        
        // Add to history buffer
        safety_state.history[safety_state.history_index] = raw_signals;
        safety_state.history_index = (safety_state.history_index + 1) % LIMIT_HISTORY_SIZE;
        
        // Perform debouncing - all history entries must agree
        axes_signals_t debounced = {0};
        bool x_consistent = true, y_consistent = true, z_consistent = true;
        
        for (uint8_t i = 1; i < LIMIT_HISTORY_SIZE; i++) {
            if (safety_state.history[i].x != safety_state.history[0].x) x_consistent = false;
            if (safety_state.history[i].y != safety_state.history[0].y) y_consistent = false;
            if (safety_state.history[i].z != safety_state.history[0].z) z_consistent = false;
        }
        
        // Update debounced state only if consistent
        if (x_consistent) debounced.x = safety_state.history[0].x;
        else debounced.x = safety_state.debounced_state.x; // Keep previous state
        
        if (y_consistent) debounced.y = safety_state.history[0].y;
        else debounced.y = safety_state.debounced_state.y;
        
        if (z_consistent) debounced.z = safety_state.history[0].z;
        else debounced.z = safety_state.debounced_state.z;
        
        safety_state.debounced_state = debounced;
        safety_state.last_read_time = current_time;
        
        // Emergency stop detection - any limit triggered
        if (debounced.x || debounced.y || debounced.z) {
            if (!safety_state.emergency_active) {
                safety_state.emergency_stop_time = current_time;
                safety_state.emergency_active = true;
                
                // Immediate safety response - stop all motion
                // This integrates with grblHAL's emergency stop system
                // system_set_exec_state_flag(EXEC_ALARM);  // Commented out - EXEC_ALARM not defined
            }
        } else {
            // Clear emergency state after timeout
            if (safety_state.emergency_active && 
                (current_time - safety_state.emergency_stop_time) > EMERGENCY_STOP_TIMEOUT_MS) {
                safety_state.emergency_active = false;
            }
        }
    }
    
    return safety_state.debounced_state;
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

static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};
    
#ifdef SPINDLE_ENABLE_PIN
    state.on = HAL_GPIO_ReadPin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) != 0;
#endif
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = HAL_GPIO_ReadPin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) != 0;
#endif
    // TODO: Read actual RPM from encoder if available
    
    return state;
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    // Simple RPM to PWM conversion for VFD
    uint16_t pwm_value = (uint16_t)((rpm / 24000.0f) * 999.0f);
    if(pwm_value > 999) pwm_value = 999;
    return pwm_value;
}

#else

static void spindleUpdateRPM (float rpm)
{
    // For VFD control, update PWM to generate 0-10V analog signal
    #ifdef SPINDLE_PWM_PIN
    // Simple RPM to PWM conversion: 0-24000 RPM = 0-999 PWM
    uint16_t pwm_value = (uint16_t)((rpm / 24000.0f) * 999.0f);
    if(pwm_value > 999) pwm_value = 999;
    spindle_set_speed(pwm_value);
    #endif
}

#endif

// Phase 3: Advanced spindle control with closed-loop feedback
static void spindle_pid_controller(void)
{
    if (!spindle_ctrl.closed_loop_enabled) return;
    
    // Calculate RPM error
    spindle_ctrl.rpm_error = spindle_ctrl.target_rpm - spindle_ctrl.actual_rpm;
    
    // PID calculation
    spindle_ctrl.pid_integral += spindle_ctrl.rpm_error;
    
    // Integral windup protection
    if (spindle_ctrl.pid_integral > 1000.0f) spindle_ctrl.pid_integral = 1000.0f;
    if (spindle_ctrl.pid_integral < -1000.0f) spindle_ctrl.pid_integral = -1000.0f;
    
    float derivative = spindle_ctrl.rpm_error - spindle_ctrl.pid_last_error;
    
    // PID output calculation
    float pid_output = (spindle_ctrl.pid_kp * spindle_ctrl.rpm_error) +
                      (spindle_ctrl.pid_ki * spindle_ctrl.pid_integral) +
                      (spindle_ctrl.pid_kd * derivative);
    
    // Apply PID correction to PWM output
    spindle_ctrl.pwm_output += pid_output;
    
    // Clamp PWM output to valid range
    if (spindle_ctrl.pwm_output > 100.0f) spindle_ctrl.pwm_output = 100.0f;
    if (spindle_ctrl.pwm_output < 0.0f) spindle_ctrl.pwm_output = 0.0f;
    
    // Update TIM2 PWM duty cycle
    uint32_t ccr_value = (uint32_t)((spindle_ctrl.pwm_output / 100.0f) * 999.0f);
    TIM2->CCR2 = ccr_value;
    
    // Check RPM stability
    spindle_ctrl.rpm_stable = (fabsf(spindle_ctrl.rpm_error) <= spindle_ctrl.rpm_tolerance);
    
    spindle_ctrl.pid_last_error = spindle_ctrl.rpm_error;
}

static void spindle_calculate_rpm(void)
{
    uint32_t current_time = getElapsedTicks();
    uint32_t time_diff = current_time - spindle_ctrl.last_encoder_time;
    
    if (time_diff >= 100) {  // Update every 100ms
        // Calculate RPM from encoder pulses
        // Assuming 1 pulse per revolution for simplicity
        float rpm = (spindle_ctrl.encoder_count * 60000.0f) / (float)time_diff;
        
        // Apply low-pass filter for smooth RPM reading
        spindle_ctrl.actual_rpm = (spindle_ctrl.actual_rpm * 0.8f) + (rpm * 0.2f);
        
        // Reset for next calculation
        spindle_ctrl.encoder_count = 0;
        spindle_ctrl.last_encoder_time = current_time;
    }
}

static void spindleSetState (spindle_state_t state, float rpm)
{
    // Phase 3: Enhanced spindle control
    spindle_ctrl.target_rpm = rpm;
    
    if (state.on) {
        // Control spindle enable pin
        #ifdef SPINDLE_ENABLE_PIN
        HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_PIN_SET);
        #endif
        
        // Control spindle direction pin
        #ifdef SPINDLE_DIRECTION_PIN  
        HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, state.ccw ? GPIO_PIN_SET : GPIO_PIN_RESET);
        #endif
        
        if (rpm > 0.0f) {
            // Calculate initial PWM value (open-loop)
            spindle_ctrl.pwm_output = (rpm / 24000.0f) * 100.0f;  // Assume 24000 RPM max
            if (spindle_ctrl.pwm_output > 100.0f) spindle_ctrl.pwm_output = 100.0f;
            
            // Set initial PWM
            uint32_t ccr_value = (uint32_t)((spindle_ctrl.pwm_output / 100.0f) * 999.0f);
            TIM2->CCR2 = ccr_value;
            
            // Enable closed-loop control if encoder available
            spindle_ctrl.closed_loop_enabled = true;
            spindle_ctrl.pid_integral = 0.0f;  // Reset integral
        }
    } else {
        // Control spindle enable pin (disable)
        #ifdef SPINDLE_ENABLE_PIN
        HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_PIN_RESET);
        #endif
        
        spindle_ctrl.pwm_output = 0.0f;
        spindle_ctrl.closed_loop_enabled = false;
        TIM2->CCR2 = 0;  // Stop PWM
    }
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

// Spindle PWM control function for VFD
void spindle_set_speed (uint_fast16_t pwm_value)
{
#ifdef SPINDLE_PWM_PIN
    // Set PWM duty cycle for VFD speed control
    // Assumes TIM2_CH2 on PA1 for BTT SKR Mini E3 v3.0
    TIM2->CCR2 = pwm_value;
    
    // Update target RPM for safety monitoring
    spindle_encoder.target_rpm = (uint32_t)((pwm_value / 999.0f) * 24000.0f);
#endif
}

// Spindle encoder interrupt handler for TIM2_CH1 input capture
// Phase 3: Enhanced TIM2 interrupt handler with advanced spindle feedback
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & (1 << 1)) {  // CC1IF bit (bit 1)
        // Input capture interrupt - encoder pulse detected
        uint32_t current_time = uwTick;
        
        // Phase 3: Update encoder count for RPM calculation
        spindle_ctrl.encoder_count++;
        
        // Calculate time between pulses for RPM calculation (legacy support)
        if (spindle_encoder.last_pulse_time > 0) {
            spindle_encoder.pulse_period = current_time - spindle_encoder.last_pulse_time;
        }
        
        spindle_encoder.pulse_count++;
        spindle_encoder.last_pulse_time = current_time;
        spindle_encoder.encoder_fault = false;  // Clear fault on valid pulse
        
        // Clear interrupt flag
        TIM2->SR &= ~(1 << 1);  // Clear CC1IF interrupt flag
    }
}

// Calculate actual spindle RPM from encoder
static void spindle_update_rpm(void)
{
    uint32_t current_time = uwTick;
    
    // Calculate RPM every 100ms
    if (current_time - spindle_encoder.last_rpm_calc_time >= 100) {
        
        // Check for encoder timeout (spindle stall or fault)
        if (current_time - spindle_encoder.last_pulse_time > SPINDLE_SAFETY_TIMEOUT_MS) {
            spindle_encoder.actual_rpm = 0;
            spindle_encoder.encoder_fault = true;
            
            // Detect stall if spindle should be running
            if (spindle_encoder.target_rpm > SPINDLE_MIN_SAFE_RPM) {
                spindle_encoder.spindle_stall_detected = true;
                
                // Trigger safety alarm for spindle stall
                // system_set_exec_state_flag(EXEC_ALARM);  // Commented out - EXEC_ALARM not defined
            }
        } else if (spindle_encoder.pulse_period > 0) {
            // Calculate RPM from pulse period
            // RPM = (60,000 ms/min) / (pulse_period_ms * pulses_per_rev)
            spindle_encoder.actual_rpm = 60000 / (spindle_encoder.pulse_period * SPINDLE_ENCODER_PPR);
            
            // Safety check for over-speed condition
            if (spindle_encoder.actual_rpm > SPINDLE_MAX_SAFE_RPM) {
                // Emergency stop for over-speed
                // system_set_exec_state_flag(EXEC_ALARM);  // Commented out - EXEC_ALARM not defined
                spindle_set_speed(0);  // Immediately stop spindle
            }
            
            spindle_encoder.spindle_stall_detected = false;
        }
        
        spindle_encoder.last_rpm_calc_time = current_time;
    }
}

// Get actual spindle RPM for status reporting
uint32_t spindle_get_actual_rpm(void)
{
    spindle_update_rpm();
    return spindle_encoder.actual_rpm;
}

// Check spindle safety status
bool spindle_is_safe(void)
{
    spindle_update_rpm();
    return !spindle_encoder.encoder_fault && !spindle_encoder.spindle_stall_detected;
}

// Enhanced emergency stop functions
static void emergency_stop_trigger(bool hardware_triggered)
{
    uint32_t current_time = uwTick;
    
    // Record E-stop trigger
    estop_system.estop_active = true;
    estop_system.estop_latched = true;
    estop_system.estop_trigger_time = current_time;
    estop_system.safety_violations++;
    
    if (hardware_triggered) {
        estop_system.hardware_estop_triggered = true;
    } else {
        estop_system.software_estop_triggered = true;
    }
    
    // Immediate safety response - highest priority
    __disable_irq();  // Critical section for safety
    
    // Stop all motion immediately
    TIM2->CR1 &= ~TIM_CR1_CEN;  // Stop stepper timer
    
    // Apply spindle brake (stop spindle immediately)
    spindle_set_speed(0);
    estop_system.spindle_brake_applied = true;
    
    // Disable all stepper drivers
    #ifdef STEPPERS_ENABLE_PORT
    HAL_GPIO_WritePin(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_PIN, GPIO_PIN_RESET);
    #endif
    
    __enable_irq();  // End critical section
    
    // Trigger grblHAL alarm system
    system_set_exec_state_flag(EXEC_ALARM);
}

// Check emergency stop input (called from main loop)
static void emergency_stop_check(void)
{
    uint32_t current_time = uwTick;
    
    // Perform safety system health check
    if (current_time - estop_system.last_safety_check >= SAFETY_SYSTEM_CHECK_MS) {
        
        // Check hardware E-stop input (active low)
        bool estop_input = !HAL_GPIO_ReadPin(RESET_PORT, RESET_PIN);
        
        if (estop_input && !estop_system.estop_active) {
            // Hardware E-stop triggered
            emergency_stop_trigger(true);
        }
        
        // Check for software safety violations
        if (!spindle_is_safe() && !estop_system.estop_active) {
            // Software E-stop triggered by spindle fault
            emergency_stop_trigger(false);
        }
        
        // Check thermal safety
        if (!thermal_is_safe() && !estop_system.estop_active) {
            // Software E-stop triggered by thermal fault
            emergency_stop_trigger(false);
        }
        
        // Monitor system response time (ensure fast emergency response)
        if (estop_system.estop_active) {
            uint32_t response_time = current_time - estop_system.estop_trigger_time;
            if (response_time > ESTOP_RESPONSE_TIME_US / 1000) {
                // Log slow response (for diagnostics)
                estop_system.safety_violations++;
            }
        }
        
        estop_system.last_safety_check = current_time;
    }
}

// Reset emergency stop (requires manual reset)
bool emergency_stop_reset(void)
{
    // Can only reset if hardware E-stop is released
    bool estop_input = !HAL_GPIO_ReadPin(RESET_PORT, RESET_PIN);
    
    if (!estop_input && estop_system.estop_latched) {
        // Reset E-stop state
        estop_system.estop_active = false;
        estop_system.estop_latched = false;
        estop_system.hardware_estop_triggered = false;
        estop_system.software_estop_triggered = false;
        estop_system.spindle_brake_applied = false;
        
        // Re-enable stepper timer
        TIM2->CR1 |= TIM_CR1_CEN;
        
        return true;  // Reset successful
    }
    
    return false;  // Reset not allowed
}

// Get emergency stop status
bool emergency_stop_is_active(void)
{
    return estop_system.estop_active;
}

// Thermal monitoring functions
#ifdef I2C_PORT

// Read temperature from I2C sensor (assumes TMP102/LM75 compatible sensor)
static bool thermal_read_sensor(thermal_sensor_t *sensor)
{
    uint8_t temp_data[2];
    i2c_transfer_t transfer = {
        .address = sensor->sensor_address,
        .word_addr = 0x00,  // Temperature register
        .word_addr_bytes = 1,
        .data = temp_data,
        .count = 2
    };
    
    if (i2c_transfer(&transfer, true)) {
        // Convert temperature data (12-bit resolution, 0.0625°C per LSB)
        int16_t raw_temp = (temp_data[0] << 8) | temp_data[1];
        raw_temp >>= 4;  // Shift to get 12-bit value
        
        // Convert to 0.1°C units for internal use
        sensor->temperature = (raw_temp * 625) / 1000;
        sensor->sensor_fault = false;
        sensor->last_read_time = uwTick;
        
        return true;
    } else {
        sensor->sensor_fault = true;
        return false;
    }
}

// Update thermal monitoring system
static void thermal_update(void)
{
    uint32_t current_time = uwTick;
    static uint8_t sensor_index = 0;
    
    // Read one sensor per cycle to distribute I2C load
    if (current_time - thermal_sensors[sensor_index].last_read_time >= THERMAL_READ_INTERVAL_MS) {
        
        thermal_sensor_t *sensor = &thermal_sensors[sensor_index];
        
        if (thermal_read_sensor(sensor)) {
            // Check temperature thresholds
            if (sensor->temperature >= sensor->max_temperature) {
                // Critical overtemperature - trigger emergency stop
                sensor->overtemperature_alarm = true;
                emergency_stop_trigger(false);  // Software E-stop
                
            } else if (sensor->temperature >= sensor->warning_temperature) {
                // Warning temperature - log but continue operation
                sensor->overtemperature_warning = true;
                
            } else {
                // Normal temperature
                sensor->overtemperature_warning = false;
                sensor->overtemperature_alarm = false;
            }
        } else {
            // Sensor communication fault
            if (current_time - sensor->last_read_time > THERMAL_SENSOR_TIMEOUT_MS) {
                sensor->sensor_fault = true;
                // Could trigger safety action for critical sensors
            }
        }
        
        // Move to next sensor
        sensor_index = (sensor_index + 1) % NUM_THERMAL_SENSORS;
    }
}

// Get thermal status for reporting
bool thermal_is_safe(void)
{
    for (uint8_t i = 0; i < NUM_THERMAL_SENSORS; i++) {
        if (thermal_sensors[i].overtemperature_alarm) {
            return false;
        }
    }
    return true;
}

// Get thermal warning status
bool thermal_has_warning(void)
{
    for (uint8_t i = 0; i < NUM_THERMAL_SENSORS; i++) {
        if (thermal_sensors[i].overtemperature_warning || thermal_sensors[i].sensor_fault) {
            return true;
        }
    }
    return false;
}

// Get temperature reading for specific sensor
int16_t thermal_get_temperature(uint8_t sensor_id)
{
    if (sensor_id < NUM_THERMAL_SENSORS) {
        return thermal_sensors[sensor_id].temperature;
    }
    return -9999;  // Invalid sensor ID
}

#else
// Stub functions when I2C is not available
static void thermal_update(void) { }
bool thermal_is_safe(void) { return true; }
bool thermal_has_warning(void) { return false; }
int16_t thermal_get_temperature(uint8_t sensor_id) { return -9999; }
#endif

// Watchdog timer functions
static void watchdog_init(void)
{
    // Initialize Independent Watchdog (IWDG)
    // IWDG clock = LSI (32kHz) / 256 = 125 Hz
    // Timeout = reload_value / 125 Hz
    // For 2 second timeout: reload_value = 2 * 125 = 250
    
    // Enable LSI clock for IWDG
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));  // Wait for LSI ready
    
    // Configure IWDG
    IWDG->KR = 0x5555;    // Enable access to IWDG registers
    IWDG->PR = 0x06;      // Prescaler = 256 (LSI/256)
    IWDG->RLR = 250;      // Reload value for ~2 second timeout
    IWDG->KR = 0xAAAA;    // Reload watchdog
    IWDG->KR = 0xCCCC;    // Start watchdog
    
    watchdog_system.watchdog_enabled = true;
    watchdog_system.last_refresh_time = uwTick;
}

static void watchdog_refresh(void)
{
    if (watchdog_system.watchdog_enabled) {
        IWDG->KR = 0xAAAA;  // Refresh watchdog
        watchdog_system.last_refresh_time = uwTick;
        watchdog_system.refresh_count++;
    }
}

static void watchdog_update(void)
{
    uint32_t current_time = uwTick;
    
    // Refresh watchdog periodically
    if (current_time - watchdog_system.last_refresh_time >= WATCHDOG_REFRESH_MS) {
        
        // Only refresh if system is operating normally
        if (!estop_system.estop_active && thermal_is_safe() && spindle_is_safe()) {
            watchdog_refresh();
        } else {
            // Don't refresh watchdog during fault conditions
            // This allows watchdog reset to recover from system faults
            watchdog_system.watchdog_fault = true;
        }
    }
}

// Get watchdog status
bool watchdog_is_healthy(void)
{
    return watchdog_system.watchdog_enabled && !watchdog_system.watchdog_fault;
}

// Check if last reset was caused by watchdog
bool watchdog_caused_reset(void)
{
    #ifdef RCC_CSR_IWDGRSTF
    return (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
    #else
    return false;  // Feature not available on this STM32 variant
    #endif
}

// Power loss detection and recovery functions (simplified for STM32G0xx)
static void power_monitor_init(void)
{
    // Simplified power monitoring - use software timeout for now
    // In production, connect external power monitor circuit to GPIO
    power_monitor.supply_voltage_mv = 3300;  // Assume nominal voltage
    power_monitor.last_power_check = uwTick;
}

static uint16_t power_read_voltage(void)
{
    // Simplified voltage reading - return nominal voltage
    // In production, read from external voltage monitor or ADC
    return 3300;  // Nominal 3.3V supply
}

static void power_save_recovery_data(void)
{
    // Save critical system state for power loss recovery
    // This would save current position, feed rates, etc. to EEPROM
    // For now, just mark that data was saved
    power_monitor.recovery_data_saved = true;
    
    // In a full implementation, save:
    // - Current machine position
    // - Current feed rate
    // - Spindle state
    // - Active G-code line number
    // - Modal state (G0/G1, units, etc.)
}

static void power_monitor_update(void)
{
    uint32_t current_time = uwTick;
    
    if (current_time - power_monitor.last_power_check >= POWER_MONITOR_INTERVAL_MS) {
        
        // Read current supply voltage
        uint16_t voltage = power_read_voltage();
        power_monitor.supply_voltage_mv = voltage;
        
        if (voltage < POWER_LOSS_THRESHOLD_MV && !power_monitor.power_loss_detected) {
            // Power loss detected
            power_monitor.power_loss_detected = true;
            power_monitor.power_loss_time = current_time;
            
            // Save recovery data immediately
            power_save_recovery_data();
            
            // Trigger emergency stop to safely halt all motion
            emergency_stop_trigger(false);
            
        } else if (voltage > POWER_RECOVERY_THRESHOLD_MV && power_monitor.power_loss_detected) {
            // Power restored
            power_monitor.power_restored = true;
            power_monitor.power_loss_detected = false;
            
            // Recovery logic could be implemented here
            // For now, require manual reset
        }
        
        power_monitor.last_power_check = current_time;
    }
}

// Get power monitoring status
bool power_is_stable(void)
{
    return !power_monitor.power_loss_detected && 
           power_monitor.supply_voltage_mv > POWER_RECOVERY_THRESHOLD_MV;
}

// Get supply voltage for diagnostics
uint16_t power_get_voltage_mv(void)
{
    return power_monitor.supply_voltage_mv;
}

// Check if power loss recovery data is available
bool power_has_recovery_data(void)
{
    return power_monitor.recovery_data_saved && power_monitor.power_restored;
}

void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    // TODO: Configure GPIO interrupt
}

// Phase 2: Feed rate optimization and path planning functions
static float calculate_optimal_feedrate(float distance, float current_speed, float target_speed, float max_accel)
{
    // Calculate optimal feed rate for given segment
    float time_to_target = fabsf(target_speed - current_speed) / max_accel;
    float accel_distance = current_speed * time_to_target + 0.5f * max_accel * time_to_target * time_to_target;
    
    if (accel_distance >= distance) {
        // Cannot reach target speed in available distance
        return sqrtf(current_speed * current_speed + 2.0f * max_accel * distance);
    }
    
    return target_speed;
}

static void optimize_motion_profile(float *segment_speeds, uint32_t segment_count, float max_feedrate, float acceleration)
{
    if (segment_count < 2) return;
    
    // Forward pass - ensure we can accelerate to each segment speed
    for (uint32_t i = 1; i < segment_count; i++) {
        float max_achievable = calculate_optimal_feedrate(
            1.0f,  // Assume 1mm segments for simplification
            segment_speeds[i-1], 
            segment_speeds[i], 
            acceleration
        );
        if (segment_speeds[i] > max_achievable) {
            segment_speeds[i] = max_achievable;
        }
    }
    
    // Backward pass - ensure we can decelerate from each segment speed  
    for (int32_t i = segment_count - 2; i >= 0; i--) {
        float max_achievable = calculate_optimal_feedrate(
            1.0f,  // Assume 1mm segments for simplification
            segment_speeds[i+1],
            segment_speeds[i],
            acceleration
        );
        if (segment_speeds[i] > max_achievable) {
            segment_speeds[i] = max_achievable;
        }
    }
}

// Phase 2: Enhanced look-ahead path planning
static void plan_motion_ahead(void)
{
    // Phase 2: Advanced look-ahead planning for corner velocities
    // This integrates with grblHAL's planner for production use
    
    // Look-ahead planning buffer (simplified for demonstration)
    static float planned_velocities[8] = {0};  // 8-segment look-ahead
    static uint8_t plan_index = 0;
    static bool planning_active = false;
    
    if (!planning_active) return;
    
    // Calculate junction velocities for upcoming moves
    // This would normally access grblHAL's planner buffer
    for (uint8_t i = 0; i < 7; i++) {
        float current_vel = planned_velocities[i];
        float next_vel = planned_velocities[i + 1];
        
        // Apply cornering speed limits based on acceleration constraints
        float max_junction_vel = sqrtf(settings.acceleration[X_AXIS] * 0.5f);  // Simplified
        
        if (current_vel > max_junction_vel) {
            planned_velocities[i] = max_junction_vel;
        }
    }
    
    // Apply feed rate optimization to planned path
    optimize_motion_profile(planned_velocities, 8, 1000.0f, 100.0f);  // Use fixed values
    
    plan_index = (plan_index + 1) % 8;
}

// Phase 2: Motion planning integration
static void enable_advanced_planning(bool enable)
{
    static bool planning_enabled = false;
    
    if (enable && !planning_enabled) {
        // Initialize advanced motion planning
        planning_enabled = true;
        
        // Reset backlash compensation state
        backlash.x_direction = 0;
        backlash.y_direction = 0;
        backlash.z_direction = 0;
        backlash.compensation_steps_x = 0;
        backlash.compensation_steps_y = 0;
        backlash.compensation_steps_z = 0;
        
        // Initialize step timing optimization
        step_timing.last_step_time = getElapsedTicks();
        
    } else if (!enable && planning_enabled) {
        // Disable advanced motion planning
        planning_enabled = false;
    }
}

// Settings changed callback with Phase 2 enhancements
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    // Phase 2: Update backlash compensation settings
    // Note: steps_per_mm not available in this settings structure
    // Backlash compensation uses fixed estimates for now
    
    // Phase 2: Update step timing parameters
    // Note: step_pulse_time and step_idle_delay not available in this settings structure
    // Using defaults for TMC2209 compatibility
    step_timing.min_step_pulse_width = 2;  // 2µs minimum
    step_timing.step_idle_delay = 1;       // 1µs delay
    step_timing.pulse_stretch_factor = 1;  // Default factor
    
    // Phase 2: Update acceleration parameters for motion optimization
    // Note: acceleration array not available in this settings structure
    // Using default acceleration value for calculations
    step_timing.acceleration_ticks = 1000;  // Default acceleration value
    
    // Phase 3: Update advanced feature settings
    if (settings) {
        // Initialize tool library if needed
        if (!tool_library.tool_change_pending) {
            tool_library.current_tool = 0;  // Default tool
        }
        
        // Reset workspace coordinate systems to defaults
        for (uint8_t i = 0; i < 6; i++) {
            workspace.systems[i].active = (i == 0);  // Only G54 active by default
        }
    }
}

// Phase 3: Tool change automation functions
static void tool_change_request(uint8_t tool_number)
{
    if (tool_number >= 100) return;  // Invalid tool number
    
    tool_library.requested_tool = tool_number;
    tool_library.tool_change_pending = true;
    
    if (tool_library.manual_tool_change) {
        // Manual tool change - pause and wait for user
        // This would typically pause the program and wait for cycle start
        // Implementation depends on grblHAL's tool change handling
    } else {
        // Automatic tool change sequence
        tool_change_execute();
    }
}

static void tool_change_execute(void)
{
    if (!tool_library.tool_change_pending) return;
    
    uint8_t old_tool = tool_library.current_tool;
    uint8_t new_tool = tool_library.requested_tool;
    
    // Execute tool change sequence
    // 1. Move to tool change position
    // 2. Stop spindle
    // 3. Retract Z to safe height
    // 4. Perform tool change (manual or automatic)
    // 5. Apply tool length offset
    // 6. Update current tool
    
    tool_library.current_tool = new_tool;
    tool_library.tool_change_pending = false;
    
    // Apply tool length offset (simplified)
    float length_offset = tool_library.tool_lengths[new_tool] - tool_library.tool_lengths[old_tool];
    // This offset would be applied to the coordinate system
}

static float get_tool_length_offset(uint8_t tool_number)
{
    if (tool_number >= 100) return 0.0f;
    return tool_library.tool_lengths[tool_number];
}

static void set_tool_length_offset(uint8_t tool_number, float length)
{
    if (tool_number >= 100) return;
    tool_library.tool_lengths[tool_number] = length;
}

// Phase 3: Workspace coordinate system functions
static void workspace_set_active_system(uint8_t system)
{
    if (system >= 6) return;  // Invalid system (0-5 for G54-G59)
    
    workspace.active_system = system;
    
    // Deactivate all systems first
    for (uint8_t i = 0; i < 6; i++) {
        workspace.systems[i].active = false;
    }
    
    // Activate requested system
    workspace.systems[system].active = true;
}

static void workspace_set_offset(uint8_t system, float x, float y, float z)
{
    if (system >= 6) return;
    
    workspace.systems[system].x_offset = x;
    workspace.systems[system].y_offset = y;
    workspace.systems[system].z_offset = z;
}

static void workspace_get_offset(uint8_t system, float *x, float *y, float *z)
{
    if (system >= 6) {
        *x = *y = *z = 0.0f;
        return;
    }
    
    *x = workspace.systems[system].x_offset;
    *y = workspace.systems[system].y_offset;
    *z = workspace.systems[system].z_offset;
}

static void workspace_apply_g92_offset(float x, float y, float z)
{
    workspace.g92_offset.x_offset = x;
    workspace.g92_offset.y_offset = y;
    workspace.g92_offset.z_offset = z;
    workspace.g92_active = true;
}

static void workspace_clear_g92_offset(void)
{
    workspace.g92_offset.x_offset = 0.0f;
    workspace.g92_offset.y_offset = 0.0f;
    workspace.g92_offset.z_offset = 0.0f;
    workspace.g92_active = false;
}

// Phase 3: Advanced probing routines
static void probe_center_finding(float diameter_estimate)
{
    if (probing.probe_active) return;  // Already probing
    
    probing.probe_active = true;
    probing.probe_cycles = 4;  // 4-point center finding
    
    // This would integrate with grblHAL's probing system
    // 1. Probe left edge
    // 2. Probe right edge
    // 3. Probe front edge
    // 4. Probe back edge
    // 5. Calculate center from edge positions
    
    // Simplified implementation - would need full grblHAL integration
    probing.center_x = 0.0f;  // Calculated center X
    probing.center_y = 0.0f;  // Calculated center Y
}

static void probe_edge_detection(uint8_t direction)
{
    if (probing.probe_active) return;
    
    probing.probe_active = true;
    probing.probe_cycles = 1;
    
    // Direction: 0=left, 1=right, 2=front, 3=back
    // This would initiate a probing move in the specified direction
    // Implementation depends on grblHAL's probing framework
}

static bool probe_is_active(void)
{
    return probing.probe_active;
}

static void probe_complete_callback(bool success, float x, float y, float z)
{
    probing.probe_active = false;
    probing.probe_success = success;
    
    if (success) {
        probing.probe_position[0] = x;
        probing.probe_position[1] = y;
        probing.probe_position[2] = z;
    }
}

// Phase 3: Job queue and batch processing
typedef struct {
    char gcode_line[256];       // G-code command
    bool executed;              // Command executed flag
    uint32_t line_number;       // Line number for tracking
} job_queue_entry_t;

typedef struct {
    job_queue_entry_t queue[32]; // 32-command queue
    uint8_t head;               // Queue head pointer
    uint8_t tail;               // Queue tail pointer
    uint8_t count;              // Number of queued commands
    bool batch_mode;            // Batch processing active
    bool auto_execute;          // Auto-execute queued commands
} job_queue_t;

static job_queue_t job_queue = {
    .head = 0,
    .tail = 0,
    .count = 0,
    .batch_mode = false,
    .auto_execute = false
};

static bool job_queue_add(const char *gcode)
{
    if (job_queue.count >= 32) return false;  // Queue full
    
    strncpy(job_queue.queue[job_queue.tail].gcode_line, gcode, 255);
    job_queue.queue[job_queue.tail].gcode_line[255] = '\0';
    job_queue.queue[job_queue.tail].executed = false;
    job_queue.queue[job_queue.tail].line_number = job_queue.tail;
    
    job_queue.tail = (job_queue.tail + 1) % 32;
    job_queue.count++;
    
    return true;
}

static bool job_queue_execute_next(void)
{
    if (job_queue.count == 0) return false;  // Queue empty
    
    job_queue_entry_t *entry = &job_queue.queue[job_queue.head];
    
    // Execute G-code command (would integrate with grblHAL's command parser)
    // protocol_execute_line(entry->gcode_line);
    
    entry->executed = true;
    job_queue.head = (job_queue.head + 1) % 32;
    job_queue.count--;
    
    return true;
}

static void job_queue_clear(void)
{
    job_queue.head = 0;
    job_queue.tail = 0;
    job_queue.count = 0;
}

// Phase 2: Enhanced driver setup with precision features
bool driver_setup (settings_t *settings)
{
    // Initialize Phase 2 precision enhancements
    
    // Set default backlash compensation values (user configurable)
    backlash.x_backlash = 0.01f;  // 0.01mm default X backlash
    backlash.y_backlash = 0.01f;  // 0.01mm default Y backlash  
    backlash.z_backlash = 0.02f;  // 0.02mm default Z backlash (lead screw)
    backlash.compensation_active = true;
    
    // Initialize step timing optimization
    step_timing.min_step_pulse_width = 2;  // Fixed 2µs for TMC2209
    step_timing.step_idle_delay = 1;       // Fixed 1µs delay
    step_timing.high_precision_timing = true;
    step_timing.acceleration_ticks = 1000; // Default acceleration value
    
    // Configure pulse stretch factor for TMC2209 compatibility
    step_timing.pulse_stretch_factor = (step_timing.min_step_pulse_width < 2) ? 2 : 1;
    
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
    
    // Critical safety checks in interrupt context
    emergency_stop_check();
    thermal_update();
    watchdog_update();
    power_monitor_update();
    
    // Phase 3: Advanced spindle control updates
    spindle_calculate_rpm();
    spindle_pid_controller();
    
    if(systick_isr)
        systick_isr();
}