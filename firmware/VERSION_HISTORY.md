# grblHAL STM32G0xx Firmware Version History

## v0.1.0 (2025-07-22) - **FIRST WORKING RELEASE** ✅

**Status**: Successfully tested in Renode emulation  
**Target**: BTT SKR Mini E3 v3.0 (STM32G0B1RET6)

### What's New
- Initial working implementation of grblHAL for STM32G0xx processors
- Full grblHAL core integration with plugin ecosystem
- Dual communication support (UART and USB CDC)
- Hardware abstraction layer with minimal stubs
- Complete build system and PlatformIO configuration

### Technical Specifications
- **UART Variant**: 164KB firmware, 8.0% RAM usage
- **USB Variant**: 165KB firmware, 9.6% RAM usage  
- **Memory Efficiency**: ~31% Flash usage with room for expansion
- **Boot Vector**: Valid ARM Cortex-M0+ vector table
- **Plugin Support**: Encoder, Laser, Plasma, Fans, Trinamic, EEPROM

### Testing Status
✅ Vector table validation  
✅ Renode emulation compatibility  
✅ Build system verification  
✅ Memory usage analysis  
✅ Feature compilation testing  

### Known Issues
- HAL functions use minimal stubs (hardware integration pending)
- Some non-critical compiler warnings for format specifiers
- USB implementation functional but not fully optimized

---

**This is the baseline release - first firmware to pass basic validation tests.**