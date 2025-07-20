/*
  btt_skr_mini_e3_3.0_map.h - driver code for STM32G0B1 ARM processors

  Part of grblHAL

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

#ifndef STM32G0B1xx
#error "This board has a STM32G0B1 processor, select a corresponding build!"
#endif

#if SAFETY_DOOR_ENABLE && N_AUTO_SQUARED
#error "Axis configuration is not supported when safety door is enabled!"
#elif N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if N_AXIS == 4
#define BOARD_NAME "BTT SKR MINI E3 V3.0 4-axis"
#else
#define BOARD_NAME "BTT SKR MINI E3 V3.0"
#endif
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3"

#define I2C_PORT        1
#define SERIAL_PORT     1 // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT   31 // GPIOC: TX = 10, RX = 11 - to Trinamic drivers
#define HAS_BOARD_INIT

#ifdef TRINAMIC_ENABLE
#undef TRINAMIC_ENABLE
#endif
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_ENABLE 2209
#define TRINAMIC_MIXED_DRIVERS 0
#define TRINAMIC_STREAM 1

#if EEPROM_ENABLE < 2
#undef EEPROM_ENABLE
#define EEPROM_ENABLE 3 // 32Kbit EEPROM, 32byte page size
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIOB
#define X_STEP_PIN              13 //PB13
#define Y_STEP_PIN              10 //PB10
#define Z_STEP_PIN              0  //PB0
#define STEP_OUTMODE            GPIO_MAP

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOB
#define X_DIRECTION_PIN         12 //PB12
#define Y_DIRECTION_PORT        GPIOB
#define Y_DIRECTION_PIN         2  //PB2
#define Z_DIRECTION_PORT        GPIOC
#define Z_DIRECTION_PIN         5  //PC5
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT           GPIOB
#define X_ENABLE_PIN            14 //PB14
#define Y_ENABLE_PORT           GPIOB
#define Y_ENABLE_PIN            11 //PB11
#define Z_ENABLE_PORT           GPIOB
#define Z_ENABLE_PIN            1  //PB1

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOC
#define X_LIMIT_PORT            GPIOC
#define X_LIMIT_PIN             0 //PC0
#define Y_LIMIT_PORT            GPIOC  
#define Y_LIMIT_PIN             1 //PC1
#define Z_LIMIT_PORT            GPIOC
#define Z_LIMIT_PIN             2 //PC2
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             3  //PB3 - E0 stepper
#define M3_DIRECTION_PORT       GPIOB
#define M3_DIRECTION_PIN        4  //PB4
#define M3_ENABLE_PORT          GPIOD
#define M3_ENABLE_PIN           2  //PD2
#if N_AUTO_SQUARED && !SAFETY_DOOR_ENABLE
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            3
#endif
#endif

#define AUXOUTPUT0_PORT         GPIOA // Spindle PWM (PA1)
#define AUXOUTPUT0_PIN          1
#define AUXOUTPUT1_PORT         GPIOC // Spindle direction
#define AUXOUTPUT1_PIN          6
#define AUXOUTPUT2_PORT         GPIOC // Spindle enable
#define AUXOUTPUT2_PIN          7
#define AUXOUTPUT3_PORT         GPIOC // Coolant flood
#define AUXOUTPUT3_PIN          8
#define AUXOUTPUT4_PORT         GPIOC // Coolant mist
#define AUXOUTPUT4_PIN          9

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT          GPIOC // Safety door
#define AUXINPUT0_PIN           3
#define AUXINPUT1_PORT          GPIOA
#define AUXINPUT1_PIN           1
#define AUXINPUT2_PORT          GPIOC // Probe
#define AUXINPUT2_PIN           14
#define AUXINPUT3_PORT          GPIOC // Reset/EStop
#define AUXINPUT3_PIN           15
#define AUXINPUT4_PORT          GPIOC // Feed hold
#define AUXINPUT4_PIN           13
#define AUXINPUT5_PORT          GPIOC // Cycle start
#define AUXINPUT5_PIN           12

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT3_PORT
#define RESET_PIN               AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT4_PORT
#define FEED_HOLD_PIN           AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT5_PORT
#define CYCLE_START_PIN         AUXINPUT5_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT2_PORT
#define PROBE_PIN               AUXINPUT2_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT1_PORT
#define I2C_STROBE_PIN          AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT  GPIOA
#define SD_CS_PIN   4 //PA4
// The following defines are not used but defined for reference
// Port init is done by HAL_SPI_MspInit() in stm32g0xx_hal_msp.c
#define SD_IO_PORT  GPIOA
#define SD_SCK_PIN  5 //PA5
#define SD_MISO_PIN 6 //PA6
#define SD_MOSI_PIN 7 //PA7
#endif