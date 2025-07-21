# grblHAL driver for STM32G0xx processors

A **fully-featured grblHAL driver** for STM32G0 series processors, specifically targeting the **BTT SKR Mini E3 v3.0** board with STM32G0B1RET6 microcontroller. This implementation includes complete motion control, USB/UART communication, and all essential CNC features for compatibility with gSender and other grbl controllers.

## Supported Hardware

- **STM32G0B1RET6** - ARM Cortex-M0+ @ 64MHz, 512KB Flash, 144KB RAM
- **BTT SKR Mini E3 v3.0** - 3D printer control board with TMC2209 stepper drivers
- Compatible with grblHAL plugin ecosystem

## Features

### Core Functionality ✅
- **Real-time CNC control** - Hardware step generation via STM32 timers
- **USB CDC communication** - Fixed enumeration for Windows compatibility
- **UART communication** - Alternative connection at 115200 baud
- **Complete motion control** - Acceleration planning and smooth trajectories
- **TMC2209 stepper drivers** - UART configuration with advanced features
- **Hardware interrupts** - Limit switches and control inputs

### Implemented Features ✅
- **Limit switches** - Hardware debouncing and interrupt-driven
- **Emergency stop** - Real-time halt with proper deceleration
- **Probe support** - G38.x commands for tool length and surface detection
- **Spindle control** - PWM speed control (M3/M5/S commands)
- **Coolant control** - Flood and mist outputs (M7/M8/M9)
- **Control inputs** - Feed hold, cycle start, reset buttons
- **I2C support** - For accessories and expansion
- **Backlash compensation** - Improved accuracy on all axes
- **Advanced safety** - Thermal monitoring and spindle feedback

### gSender Compatibility ✅
- Full G-code support (G0, G1, G2, G3, etc.)
- Real-time status reporting
- Jogging and homing cycles
- Workspace coordinates (G54-G59)
- Tool change support

## Firmware Downloads

**Pre-built firmware binaries are available:**
- [`firmware_usb_complete.bin`](firmware/firmware_usb_complete.bin) - USB connection (recommended)
- [`firmware_uart_complete.bin`](firmware/firmware_uart_complete.bin) - UART connection

## Documentation

- 📖 **[Quick Start Guide](QUICKSTART.md)** - Get running in 5 minutes
- 🔌 **[Connection Guide](CONNECTION_GUIDE.md)** - Detailed wiring for CNC use
- 📐 **[Wiring Diagrams](WIRING_DIAGRAM.md)** - Visual connection reference
- ⚠️ **[E-Stop Wiring](ESTOP_WIRING.md)** - CRITICAL safety system setup
- 🧪 **[Testing Guide](TESTING_GUIDE.md)** - Comprehensive test procedures
- 💾 **[Firmware Guide](firmware/README.md)** - Version descriptions and updates

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

1. Format SD card as FAT32 (max 32GB supported)
2. Copy firmware file to SD card root directory
3. **IMPORTANT**: Rename file to exactly `firmware.bin`
4. Insert SD card into TF card slot on board
5. Power cycle the board (disconnect and reconnect power)
6. Board will auto-flash (status LED blinks during update)
7. Wait ~10 seconds for completion
8. Remove SD card (file will be renamed to `FIRMWARE.CUR`)

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

## Hardware Setup

### CNC-Specific Connections
The BTT SKR Mini E3 v3.0 uses 3D printer terminology. See our **[Connection Guide](CONNECTION_GUIDE.md)** for:
- Motor connections (X/Y/Z/E0 → CNC axes)
- Limit switches (X-STOP/Y-STOP/Z-STOP)
- Spindle control (FAN1/HEAT0 outputs)
- Coolant control (FAN2/HEAT1 outputs)
- Probe and control panel wiring

### Visual Wiring
Check **[Wiring Diagrams](WIRING_DIAGRAM.md)** for:
- Complete system overview
- VFD spindle connections
- Relay module wiring
- Control panel setup

## Safety

⚠️ **CRITICAL SAFETY**: 
- **E-Stop MUST cut power physically** - See [ESTOP_WIRING.md](ESTOP_WIRING.md)
- Always have emergency stop accessible
- Test E-stop functionality daily
- Test without tools/spindle first
- Start with low speeds and acceleration
- Verify all connections before power on
- Use proper current limiting on stepper drivers

## Troubleshooting

### USB Not Recognized
- Use the `firmware_usb_complete.bin` version
- Ensure data-capable Micro USB cable (not charge-only)
- Try USB 2.0 ports (more compatible than USB 3.0)
- Windows: Check Device Manager → Ports for "STMicroelectronics Virtual COM Port"
- May need to install STM32 Virtual COM Port drivers

### No Motion
- Verify stepper enable signals (active low)
- Check motor driver current settings
- Confirm step/mm calibration ($100-$102)
- Test with lower speeds

### Limit Switches
- Default expects normally open (NO) switches
- Use $5 to invert if using normally closed (NC)
- Verify with ? status command

---
*Version 1.0.0 - 2025-01-21*
*Full grblHAL implementation with essential CNC features*