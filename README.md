# grblHAL driver for STM32G0xx processors

A grblHAL driver for STM32G0 series processors, specifically targeting the **BTT SKR Mini E3 v3.0** board with STM32G0B1RET6 microcontroller.

## Supported Hardware

- **STM32G0B1RET6** - ARM Cortex-M0+ @ 64MHz, 512KB Flash, 144KB RAM
- **BTT SKR Mini E3 v3.0** - 3D printer control board with TMC2209 stepper drivers
- Compatible with grblHAL plugin ecosystem

## Features

- **Real-time CNC control** optimized for STM32G0 architecture
- **TMC2209 UART stepper drivers** - Silent operation with advanced features
- **USB CDC & UART communication** - Multiple interface options
- **Bootloader support** - SD card firmware updates (32KB offset)
- **Plugin architecture** - Modular extensions (EEPROM, trinamic, motors, etc.)
- **Pin-optimized GPIO** - No bitband dependency (G0 limitation handled)

## Quick Start

### Building with PlatformIO (Recommended)

```bash
# Clone the repository
git clone https://github.com/wardnb/STM32G0xx.git
cd STM32G0xx

# Build firmware variants
pio run -e BTT_SKR_MINI_E3_V30          # UART communication
pio run -e BTT_SKR_MINI_E3_V30_USB      # USB CDC communication

# Firmware output
ls .pio/build/BTT_SKR_MINI_E3_V30/firmware.bin
```

### Flashing Firmware

**BTT SKR Mini E3 v3.0 uses SD card bootloader:**

1. Copy `firmware.bin` to SD card root directory
2. Insert SD card into printer board  
3. Power cycle the board (firmware auto-updates)
4. Remove SD card after successful update

### Alternative: STM32CubeIDE

Import the project directory into STM32CubeIDE for debugging and development.

## Configuration

Board-specific settings are in:
- **Pin mappings**: [`boards/btt_skr_mini_e3_3.0_map.h`](boards/btt_skr_mini_e3_3.0_map.h)
- **Build options**: [`platformio.ini`](platformio.ini)
- **Driver config**: [`Inc/my_machine.h`](Inc/my_machine.h)

### Key Pin Assignments (BTT SKR Mini E3 v3.0)

| Function | Pin | Notes |
|----------|-----|-------|
| X Step | PB13 | |
| Y Step | PB10 | |
| Z Step | PB0 | |
| X Dir | PB12 | |
| Y Dir | PB2 | |
| Z Dir | PC5 | |
| X Enable | PB14 | |
| Y Enable | PB11 | |
| Z Enable | PB1 | |
| X Limit | PC0 | |
| Y Limit | PC1 | |
| Z Limit | PC2 | |
| UART TX | PA9 | Console |
| UART RX | PA10 | Console |
| TMC UART | PC10/PC11 | Stepper drivers |

## STM32G0 vs F1 Differences

| Feature | STM32F1xx | STM32G0xx |
|---------|-----------|-----------|
| **CPU** | Cortex-M3 72MHz | Cortex-M0+ 64MHz |
| **Flash** | 128-256KB | 512KB |
| **RAM** | 20-48KB | 144KB |
| **Bitband** | ✅ Supported | ❌ Not available |
| **GPIO** | Bitband ops | Direct register access |
| **Power** | Higher | Lower (optimized) |

## Development Notes

- **No bitband support** - GPIO operations use direct register manipulation
- **Expanded memory** - 512KB flash allows full feature set without limitations
- **Plugin compatible** - Works with existing grblHAL plugins via symlinks
- **Bootloader aware** - 32KB offset for BTT bootloader compatibility

## Plugins

This driver includes symlinks to grblHAL plugins:
- `eeprom/` - Settings storage
- `trinamic/` - TMC2209 stepper driver support
- `keypad/` - Input device support
- `motors/` - Motor control extensions
- `sdcard/` - SD card functionality
- `grbl/` - Core grblHAL functionality

## Contributing

This driver follows grblHAL architecture patterns. For contributions:

1. Fork the repository
2. Create feature branch
3. Test on actual hardware
4. Submit pull request

## License

This grblHAL driver is released under the [GNU General Public License v3.0](COPYING).

## Links

- [grblHAL Core](https://github.com/grblHAL/core)
- [BTT SKR Mini E3 v3.0](https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3)
- [STM32G0 Documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32g0-series.html)
- [PlatformIO STM32 Platform](https://docs.platformio.org/en/latest/platforms/ststm32.html)

---
*Initial implementation: 2025-01-20*