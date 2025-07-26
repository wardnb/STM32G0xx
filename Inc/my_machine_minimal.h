/*
  my_machine_minimal.h - Minimal configuration for testing

  Part of grblHAL
*/

#ifndef _MY_MACHINE_H_
#define _MY_MACHINE_H_

// Minimal configuration - disable most peripherals for testing
#define N_AXIS 3
#define N_SPINDLE 1

// Use UART for now to test basic functionality
#define USB_SERIAL_CDC 0

// Disable optional features that might cause issues
#define SDCARD_ENABLE 0
#define KEYPAD_ENABLE 0
#define ODOMETER_ENABLE 0
#define PPI_ENABLE 0
#define EEPROM_ENABLE 0
#define EEPROM_IS_FRAM 0

// Disable Trinamic drivers
#define TRINAMIC_ENABLE 0
#define TRINAMIC_UART_ENABLE 0

// Basic safety features only
#define ESTOP_ENABLE 0
#define SAFETY_DOOR_ENABLE 0

// No spindle for testing
#define SPINDLE_ENABLE 0
#define VFD_ENABLE 0

// Minimal probe support
#define PROBE_ENABLE 0

// Disable I2C
#define I2C_ENABLE 0

// Minimal driver capabilities
#define DRIVER_SPINDLE_ENABLE 0
#define DRIVER_SPINDLE_PWM_ENABLE 0

#endif