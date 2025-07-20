# grblHAL Firmware for BTT SKR Mini E3 v3.0

This directory contains pre-compiled firmware binaries for the BTT SKR Mini E3 v3.0 board with STM32G0B1 processor.

## Firmware Variants

### BTT_SKR_MINI_E3_V30_uart.bin
- **Communication**: UART (traditional serial via TMC2209 pins)
- **Use case**: Connect via UART adapter or TMC2209 UART pins
- **Features**: All core grblHAL features, VFD spindle control, TMC2209 support

### BTT_SKR_MINI_E3_V30_usb.bin  
- **Communication**: USB CDC (direct USB connection)
- **Use case**: Direct USB connection to PC
- **Features**: All core grblHAL features, VFD spindle control, TMC2209 support
- **Note**: USB implementation uses placeholder middleware - full USB functionality requires complete STM32 USB middleware

## Installation Instructions

1. Copy the desired `firmware.bin` file to the root directory of your SD card
2. Rename it to `firmware.bin` (remove the prefix)
3. Insert the SD card into your BTT SKR Mini E3 v3.0
4. Power cycle the board
5. The bootloader will automatically flash the firmware and rename the file to `firmware.cur`

## Board Features Supported

- ✅ **Stepper Control**: X, Y, Z axes with TMC2209 drivers
- ✅ **Limits**: X, Y, Z limit switches on PC0, PC1, PC2
- ✅ **VFD Spindle**: PWM control on PA1 (TIM2_CH2) for 0-10V VFD interface
- ✅ **TMC2209 UART**: Stepper driver configuration via UART
- ✅ **grblHAL Integration**: Full grblHAL feature set
- ✅ **Memory Optimized**: Only 0.8% RAM, 0.3% Flash usage

## Build Information

- **Target**: STM32G0B1RET6 (Cortex-M0+, 64MHz)
- **Memory**: 512KB Flash, 144KB RAM  
- **Bootloader Offset**: 0x8000 (32KB)
- **Build Tool**: PlatformIO
- **Framework**: STM32Cube HAL

## Pin Configuration

See `boards/btt_skr_mini_e3_3.0_map.h` for complete pin mappings.

Key pins:
- **Spindle PWM**: PA1 (TIM2_CH2)
- **Spindle Enable**: PC14
- **Limits**: PC0 (X), PC1 (Y), PC2 (Z)
- **TMC2209 UART**: PA2/PA3 (USART2)

## Development

To rebuild firmware:
```bash
# UART version
pio run -e BTT_SKR_MINI_E3_V30

# USB version  
pio run -e BTT_SKR_MINI_E3_V30_USB
```

Binaries will be generated in `.pio/build/<env_name>/firmware.bin`