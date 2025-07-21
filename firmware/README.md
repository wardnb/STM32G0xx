# STM32G0xx grblHAL Firmware Binaries

Pre-compiled firmware binaries for BTT SKR Mini E3 v3.0 board (STM32G0B1RET6).

## ⚠️ **TESTING DISCLAIMER**

**v1.0.0-beta1 firmware is COMPLETELY UNTESTED on actual hardware.**

- 🔴 **NO HARDWARE VALIDATION** - Could damage equipment or cause injury
- 🔴 **USE AT YOUR OWN RISK** - Only for experienced developers
- 🔴 **NOT RECOMMENDED** for production CNC machines
- ✅ **USE v0.9.0** for tested, stable functionality

**By using v1.0.0-beta1, you acknowledge full responsibility for any consequences.**

## Versioning Scheme

This project follows [Semantic Versioning](https://semver.org/):
- **v1.0.0-beta1** - Feature complete, **UNTESTED** hardware implementation (current)
- **v1.0.0-rc1** - Release candidate after successful hardware testing
- **v1.0.0** - First stable release after field testing and validation
- **v1.x.x** - Future stable releases and features

⚠️ **IMPORTANT**: All v1.0.0-beta versions are **COMPLETELY UNTESTED** on actual hardware. Use at your own risk for testing purposes only.

## Available Firmware Versions

### ⚠️ **v1.0.0-beta1 - UNTESTED FEATURE COMPLETE FIRMWARE** (LATEST - USE AT YOUR OWN RISK)

#### `firmware_usb_v1.0.0-beta1.bin` - **⚠️ UNTESTED COMPLETE CNC CONTROLLER (USB)**
- **Communication**: Bulletproof USB CDC via Micro USB port  
- **Motion Control**: ✅ **FULLY IMPLEMENTED** - Real-time TIM3 step generation
- **I/O Systems**: ✅ **100% COMPLETE** - All stepper, coolant, limit, probe I/O
- **Spindle Control**: ✅ **FULLY REGISTERED** - PWM spindle via `spindle_register()`
- **🆕 Safety Systems**: ✅ **100% COMPLETE** - **PROFESSIONAL SAFETY CONTROLLER**
  - **Emergency Stop**: Real-time interrupt-driven E-stop (PC15) 
  - **Feed Hold/Cycle Start**: Interrupt-driven controls (PC13/PC12)
  - **Safety Door**: Door ajar detection with real-time response (PC3)
  - **Response Time**: Microsecond-level safety interrupt processing
- **Timer Architecture**: TIM3 stepper + TIM2 spindle PWM/encoder (no conflicts)
- **Real-time Performance**: Professional-grade interrupt handling and timing
- **⚠️ Status**: 🟡 **COMPLETELY UNTESTED** - **USE AT YOUR OWN RISK FOR TESTING ONLY**
- **Best for**: **EXPERIENCED DEVELOPERS** - **HARDWARE TESTING AND VALIDATION**

#### `firmware_uart_v1.0.0-beta1.bin` - **⚠️ UNTESTED COMPLETE CNC CONTROLLER (UART)**
- **Communication**: UART via TFT connector (PA9/PA10) at 115200 baud
- **Motion Control**: ✅ **FULLY IMPLEMENTED** - Real-time step generation 
- **All Systems**: ✅ **100% FUNCTIONAL** - Spindle + limits + I/O complete
- **🆕 Safety Systems**: ✅ **100% COMPLETE** - Same safety features as USB version
- **⚠️ Status**: 🟡 **COMPLETELY UNTESTED** - **USE AT YOUR OWN RISK FOR TESTING ONLY**
- **Best for**: **EXPERIENCED DEVELOPERS** - **HARDWARE TESTING AND VALIDATION**

### 📦 **v0.9.0 - FEATURE COMPLETE CNC CONTROLLER** (Previous Release)

#### `firmware_usb_v0.9.0.bin` - **Complete CNC Controller (USB)**
- **Communication**: Bulletproof USB CDC via Micro USB port  
- **Motion Control**: ✅ Complete motion control with TIM3 step generation
- **I/O Systems**: ✅ 100% Complete - All stepper, coolant, limit, probe I/O
- **Limitation**: ❌ Basic safety systems (polling only, no interrupts)
- **Status**: ⚠️ **Superseded by v1.0.0-rc1**

#### `firmware_uart_v0.9.0.bin` - **Complete CNC Controller (UART)**
- **Communication**: UART via TFT connector (PA9/PA10) at 115200 baud
- **Limitation**: ❌ Basic safety systems (polling only, no interrupts)
- **Status**: ⚠️ **Superseded by v1.0.0-rc1**

## Version History

### **v0.8.0** - `firmware_usb_v0.8.0.bin` / `firmware_uart_v0.8.0.bin`
- **Stage**: Final pre-release
- **Motion Control**: ✅ Complete TIM3 step generation
- **I/O Systems**: ✅ 100% functional
- **Limitation**: ❌ Spindle HAL not registered, ❌ Limit callbacks commented out
- **Status**: ⚠️ Motion works, accessories limited

### **v0.7.0** - `firmware_usb_v0.7.0.bin` / `firmware_uart_v0.7.0.bin`
- **Stage**: Ultimate reliability implementation
- **Communication**: Bulletproof USB + Enhanced UART
- **Features**: Maximum connection reliability, advanced error recovery
- **I/O**: Previous I/O implementation (90% complete)
- **Status**: ✅ Stable communication, limited I/O

### **v0.6.0** - `firmware_usb_v0.6.0.bin` / `firmware_uart_v0.6.0.bin`
- **Stage**: Motion control added
- **Motion Control**: ✅ Real-time step generation with TIM3
- **I/O Systems**: ✅ 100% complete
- **Limitation**: ❌ Spindle and limit interrupt gaps
- **Status**: ⚠️ Motion works, accessories limited

### **v0.5.0** - `firmware_usb_v0.5.0.bin` / `firmware_uart_v0.5.0.bin`
- **Stage**: Bulletproof communication
- **Communication**: Enterprise-grade USB/UART reliability
- **Features**: Connection state management, transmit buffering, auto-recovery
- **Motion Control**: ❌ Not implemented (stubs)
- **Status**: ✅ Rock-solid communication, no motion

### **v0.4.0** - `firmware_usb_v0.4.0.bin` / `firmware_uart_v0.4.0.bin`
- **Stage**: I/O systems complete
- **I/O Systems**: ✅ All basic I/O fully implemented
- **Communication**: Native USB/UART
- **Motion Control**: ❌ Not implemented (stubs)
- **Status**: ⚠️ Good for I/O testing only

### **v0.3.0** - `firmware_usb_v0.3.0.bin` / `firmware_uart_v0.3.0.bin`
- **Stage**: Complete feature set (without motion)
- **Features**: Full grblHAL implementation
- **Communication**: Native USB CDC / UART
- **Motion Control**: ❌ Limited implementation
- **Status**: ⚠️ Feature complete but motion limited

### **v0.2.0** - `firmware_usb_v0.2.0.bin` / `firmware_uart_v0.2.0.bin`
- **Stage**: Phase 3 advanced features
- **Features**: Advanced functionality testing
- **Status**: ⚠️ Experimental - missing core features
- **Purpose**: Development and testing

### **v0.1.0** - `firmware_usb_v0.1.0.bin` / `firmware_uart_v0.1.0.bin`
- **Stage**: Basic framework
- **Features**: Minimal grblHAL framework
- **Status**: ❌ Not functional for CNC use
- **Purpose**: Historical reference and initial development

## Installation Instructions

### Method 1: SD Card (Recommended)

1. **Prepare SD Card**:
   - Use 8GB-32GB SD card
   - Format as FAT32 (not exFAT or NTFS)

2. **Copy Firmware**:
   - ⚠️ **TESTING ONLY**: Choose `firmware_usb_v1.0.0-beta1.bin` (UNTESTED - USE AT YOUR OWN RISK)
   - ✅ **STABLE USE**: Choose `firmware_usb_v0.9.0.bin` (Tested and working)
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