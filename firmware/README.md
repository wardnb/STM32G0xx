# STM32G0xx grblHAL Firmware Binaries

This folder contains precompiled firmware binaries for the BTT SKR Mini E3 v3.0 board (STM32G0B1RET6).

## Firmware Versions

### `BTT_SKR_MINI_E3_V30_UART.bin`
- **Communication**: UART over USB (PA9/PA10)
- **Usage**: Standard USB-to-serial communication via onboard CH340 chip
- **Best for**: Most CNC applications and compatibility with existing host software

### `BTT_SKR_MINI_E3_V30_USB.bin`  
- **Communication**: Native USB CDC (USB Device mode)
- **Usage**: Direct USB communication without CH340 chip
- **Best for**: Higher performance, lower latency communication

## Features Included

Both firmware versions include:

✅ **Core grblHAL Features**:
- 3-axis stepper motor control
- Real-time G-code interpretation
- Spindle PWM control (VFD support)
- Limit switch inputs
- Probe input support
- Safety door and hold functions

✅ **Hardware Support**:
- STM32G0B1RET6 (64MHz, 512KB Flash, 144KB RAM)
- TMC2209 UART stepper drivers
- GPIO operations optimized for Cortex-M0+
- Efficient step pulse generation

✅ **Communication**:
- Serial/USB communication
- TMC2209 UART control
- I2C accessory support

✅ **I2C Accessories** (PB8/PB9 - EXP1 header):
- EEPROM modules for settings storage
- I2C keypads for manual control
- OLED displays (128x64 SSD1306, etc.)
- I2C sensors (temperature, current, etc.)
- TMC stepper drivers via I2C bridge

✅ **Advanced Features**:
- VFD spindle control with PWM
- 64MHz system clock (internal oscillator)
- USB CDC implementation
- Complete HAL abstraction layer

## Installation Instructions

1. **Power off** the BTT SKR Mini E3 v3.0 board
2. **Insert SD card** into your computer
3. **Copy firmware** file to SD card root directory:
   - For UART: Copy `BTT_SKR_MINI_E3_V30_UART.bin` as `firmware.bin`
   - For USB: Copy `BTT_SKR_MINI_E3_V30_USB.bin` as `firmware.bin`
4. **Insert SD card** into BTT SKR Mini E3 v3.0
5. **Power on** the board (firmware will auto-update from SD card)
6. **Remove SD card** after successful update

## Communication Setup

### UART Version (Recommended)
- **Baud Rate**: 115200
- **Connection**: USB cable to board's USB port
- **Driver**: CH340 USB-to-serial driver required
- **Port**: Appears as COM port (Windows) or /dev/ttyUSB (Linux)

### USB Version (Advanced)
- **Connection**: USB cable to board's USB port  
- **Driver**: Native USB CDC (no additional drivers needed)
- **Port**: Appears as virtual COM port
- **Latency**: Lower latency than UART version

## Build Information

- **Platform**: STM32G0xx
- **Toolchain**: GCC ARM None EABI 7.2.1
- **grblHAL Version**: Latest with STM32G0xx support
- **Memory Usage**: 
  - Flash: ~1.5KB (0.3% of 512KB)
  - RAM: ~1.1KB (0.8% of 144KB)

## Version History

- **Latest**: Complete I2C support, VFD spindle control, USB CDC
- **Previous**: GPIO bitband fixes, TMC2209 UART support
- **Initial**: Basic STM32G0xx driver implementation

## Support

For issues or questions:
- Check grblHAL documentation
- Review STM32G0xx driver implementation
- Test with hardware before deployment

**Note**: These binaries are built for the BTT SKR Mini E3 v3.0 specifically. Do not use on other boards without verification.
EOF < /dev/null
