# STM32G0xx grblHAL Firmware Binaries

Pre-compiled firmware binaries for BTT SKR Mini E3 v3.0 board (STM32G0B1RET6).

## Available Firmware Versions

### 🎯 Latest Release (RECOMMENDED)

#### `firmware_usb_complete.bin` - Full USB Implementation
- **Communication**: Native USB CDC via Micro USB port
- **Features**: Complete grblHAL implementation with all features
- **Status**: ✅ Production ready
- **Best for**: Most users - Windows/Linux/Mac compatible

#### `firmware_uart_complete.bin` - Full UART Implementation  
- **Communication**: UART via TFT connector (PA9/PA10)
- **Features**: Complete grblHAL implementation with all features
- **Status**: ✅ Production ready
- **Best for**: Systems without USB support or using external UART adapter

### 🔧 Development Versions

#### `firmware_usb_phase3.bin` / `firmware_uart_phase3.bin`
- **Features**: Phase 3 advanced features only
- **Status**: ⚠️ Experimental - missing core features
- **Purpose**: Testing advanced functionality

### 📦 Legacy Versions

#### `firmware_usb.bin` / `firmware_uart.bin`
- **Features**: Basic framework only
- **Status**: ❌ Not functional for CNC use
- **Purpose**: Historical reference only

## Installation Instructions

### Method 1: SD Card (Recommended)

1. **Prepare SD Card**:
   - Use 8GB-32GB SD card
   - Format as FAT32 (not exFAT or NTFS)

2. **Copy Firmware**:
   - Choose your firmware (usually `firmware_usb_complete.bin`)
   - Copy to SD card root directory
   - **RENAME to exactly**: `firmware.bin`

3. **Flash Board**:
   - Insert SD card into TF slot on board
   - Power on board
   - Power LED blinks during update (~10 seconds)
   - `firmware.bin` becomes `FIRMWARE.CUR` when done

4. **Complete**:
   - Remove SD card
   - Board restarts with new firmware

### Method 2: USB DFU Mode

1. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. Connect Micro USB cable
3. Hold BOOT button + press RESET
4. Device appears as "STM32 BOOTLOADER"
5. Flash at address 0x08000000
6. Press RESET to start

## Connecting After Installation

### For USB Versions
```
1. Connect Micro USB cable
2. Open gSender/UGS/etc
3. Look for "STMicroelectronics Virtual COM Port"
4. Connect at 115200 baud
5. Send ? to verify
```

### For UART Versions
```
1. Connect FTDI/USB-UART to TFT port:
   - FTDI TX → PA10 (board RX)
   - FTDI RX → PA9 (board TX)
   - FTDI GND → GND
2. Open terminal at 115200 baud
3. Send ? to verify
```

## Features Included

✅ **Motion Control**
- Hardware step generation (TIM2)
- Smooth acceleration profiles
- Backlash compensation
- Up to 100kHz step rates

✅ **I/O Support**
- Limit switches with debouncing
- Probe input (G38.x commands)
- Emergency stop & safety door
- Feed hold/cycle start buttons

✅ **Spindle & Coolant**
- PWM spindle speed (M3/M5/S)
- Spindle direction control
- Flood/mist coolant (M7/M8/M9)

✅ **Communication**
- USB CDC (Virtual COM Port)
- UART (115200 baud)
- Real-time status reports
- Full grbl compatibility

✅ **Advanced Features**
- Workspace coordinates (G54-G59)
- Tool change support
- I2C accessories (PB8/PB9)
- EEPROM settings storage

## Quick Test

After flashing, connect and run:
```
?                  (check status)
$$                 (view settings)
$I                 (build info)
G91 G1 X10 F100   (move X 10mm)
```

## Troubleshooting

**SD Card won't flash:**
- File MUST be named `firmware.bin` exactly
- Use FAT32 format, not exFAT
- Try different SD card (some incompatible)
- Power LED should blink during flash

**USB not detected:**
- Use data cable, not charge-only
- Try USB 2.0 port (more compatible)
- Windows: Check Device Manager → Ports
- May need STM32 VCP drivers

**No movement:**
- Ensure using `complete` version
- Check motor driver installation
- Verify enable signals ($4 setting)
- Start with low speeds/acceleration

## Version Comparison

| Feature | Complete | Phase3 | Basic |
|---------|----------|---------|--------|
| Motion Control | ✅ | ⚠️ | ❌ |
| USB Fixed | ✅ | ✅ | ✅ |
| Limit Switches | ✅ | ⚠️ | ❌ |
| Spindle/Coolant | ✅ | ✅ | ❌ |
| Probe Support | ✅ | ⚠️ | ❌ |
| Safety Systems | ✅ | ✅ | ❌ |
| Production Ready | ✅ | ❌ | ❌ |

## Support

- [grblHAL Wiki](https://github.com/grblHAL/core/wiki)
- [BTT SKR Mini E3 v3.0 Manual](https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3/tree/master/hardware/BTT%20SKR%20MINI%20E3%20V3.0)
- [Issue Tracker](https://github.com/wardnb/STM32G0xx/issues)

---
*Built for BTT SKR Mini E3 v3.0 with STM32G0B1RET6 processor*