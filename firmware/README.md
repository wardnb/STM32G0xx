# grblHAL Firmware for BTT SKR Mini E3 v3.0

This directory contains pre-compiled firmware binaries for the BTT SKR Mini E3 v3.0 board (STM32G0B1).

## Firmware Files

- `firmware_uart.bin` - UART communication version (default serial console)
- `firmware_usb.bin` - USB CDC communication version (USB virtual COM port)

## Installation

### Method 1: SD Card (Recommended)
1. Copy the desired firmware file to the root of a FAT32-formatted SD card
2. Rename the file to `firmware.bin`
3. Insert the SD card into the BTT SKR Mini E3 v3.0 board
4. Power cycle the board
5. The firmware will be automatically flashed (LED will blink during update)
6. Remove the SD card after successful update

### Method 2: DFU Mode
1. Hold the BOOT0 button while powering on the board
2. Use STM32CubeProgrammer or similar tool to flash the firmware
3. Connect via USB and select the appropriate firmware file

## Communication

### UART Version (firmware_uart.bin)
- Connect via UART1 pins (TX: PA9, RX: PA10)
- Baud rate: 115200
- 8N1 format

### USB Version (firmware_usb.bin)  
- Connect via USB-C port
- Appears as virtual COM port in Windows/Linux/macOS
- No additional drivers required on most systems

## Features

This firmware includes:
- grblHAL CNC controller functionality
- I2C support for accessories
- TMC2209 stepper driver support via UART
- SD card support (if hardware available)
- Real-time G-code processing
- Limit switch support with debouncing
- Spindle control (PWM and direction)
- Coolant control

## Board Configuration

The firmware is configured for the BTT SKR Mini E3 v3.0 with:
- STM32G0B1RET6 microcontroller (64MHz)
- TMC2209 stepper drivers
- Standard pin assignments as per BTT documentation

## USB Enumeration Fix

The USB version includes fixes for proper USB device enumeration on Windows systems. This resolves the issue where the board would not appear as a USB device when connected.

## Version Information

- grblHAL Version: Latest
- STM32G0xx Driver: v240101
- Build Date: $(date)
- USB Fixes: Applied

## Support

For issues or questions:
- grblHAL: https://github.com/grblHAL
- STM32G0xx Driver: https://github.com/grblHAL/STM32G0xx

## License

This firmware is based on grblHAL, which is licensed under the GNU General Public License v3.0.