# STM32G0xx grblHAL Firmware Binaries

Pre-compiled firmware binaries for BTT SKR Mini E3 v3.0 board (STM32G0B1RET6).

## Available Firmware Versions

### 🌟 ULTIMATE RELEASE - DUAL BULLETPROOF (NEWEST)

#### `firmware_usb_ultimate.bin` - Ultimate USB + UART Implementation  
- **Communication**: Bulletproof USB CDC + Enhanced UART layers
- **Features**: Complete grblHAL + Bulletproof USB + Bulletproof UART
- **USB**: Advanced connection state management, transmit buffering, auto-recovery
- **UART**: Error detection, state management, diagnostic monitoring
- **Windows**: Optimized descriptors and enumeration for Windows 10/11
- **Status**: ✅ Production ready - Maximum reliability on both interfaces
- **Best for**: ALL users - Ultimate reliability and compatibility

#### `firmware_uart_ultimate.bin` - Ultimate UART + USB Management
- **Communication**: Enhanced UART with bulletproof USB management layers
- **Features**: Complete grblHAL + Advanced UART reliability + USB state management
- **UART**: 256-byte TX buffer, error recovery, connection monitoring  
- **USB**: Background management for maximum compatibility
- **Status**: ✅ Production ready - Ultimate UART reliability
- **Best for**: UART users wanting maximum reliability and error handling

### 🚀 BULLETPROOF RELEASE (STABLE)

#### `firmware_usb_bulletproof.bin` - Enterprise-Grade USB Implementation
- **Communication**: Bulletproof USB CDC via Micro USB port
- **Features**: Complete grblHAL + Advanced USB reliability system
- **USB Reliability**: Connection state management, transmit buffering, auto-recovery
- **Windows**: Optimized descriptors and enumeration for Windows 10/11
- **Status**: ✅ Production ready - Handles disconnects, power cycles, driver issues
- **Best for**: USB users preferring previous bulletproof version

#### `firmware_uart_bulletproof.bin` - Enterprise-Grade UART Implementation
- **Communication**: UART via TFT connector (PA9/PA10) + USB management layers
- **Features**: Complete grblHAL + Advanced connection reliability
- **Status**: ✅ Production ready
- **Best for**: UART users preferring previous bulletproof version

### 📦 Previous Stable Versions

#### `firmware_usb_complete.bin` - Full USB Implementation (Stable)
- **Communication**: Native USB CDC via Micro USB port
- **Features**: Complete grblHAL implementation with all features
- **Status**: ✅ Production ready
- **Best for**: Users who prefer previous stable version

#### `firmware_uart_complete.bin` - Full UART Implementation (Stable)
- **Communication**: UART via TFT connector (PA9/PA10)  
- **Features**: Complete grblHAL implementation with all features
- **Status**: ✅ Production ready
- **Best for**: UART users who prefer previous stable version

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
   - Choose your firmware (recommended: `firmware_usb_ultimate.bin`)
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

## 🌟 NEW: Ultimate Reliability Features (Both USB & UART)

✅ **USB Connection Management**
- Automatic disconnect detection & recovery
- Enumeration timeout and retry logic (5 second timeout)
- Connection state tracking with diagnostics
- Survives USB cable removal/reconnection

✅ **Advanced USB Transmit Buffering**
- 128-byte transmit buffer with flow control
- Packet-based 64-byte USB CDC transmission
- Transmit completion tracking and callbacks
- No data loss during USB busy periods

✅ **Enhanced UART Reliability** 
- 256-byte UART transmit buffer with flow control
- Advanced error detection and recovery
- Connection state management and monitoring
- Interrupt-driven transmission with completion tracking
- Hardware error counting and diagnostic reporting

✅ **Dual-Interface Error Recovery**
- Cross-interface state management
- Automatic retry on failed operations
- Connection attempt counting for both USB and UART
- State validation prevents invalid operations
- Recovery from overrun, framing, and parity errors

✅ **Windows 10/11 Optimized**
- Enhanced device descriptors for Windows compatibility
- Proper USB 2.0 compliance and device versioning
- Unique serial numbers from STM32 hardware UID
- High-priority interrupts for reliable enumeration

✅ **Professional Diagnostic Features**
- Real-time error counting and classification
- Connection attempt tracking
- Activity timestamps for both interfaces
- Comprehensive state monitoring

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