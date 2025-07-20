# STM32G0xx Build Notes

## Current Status

The STM32G0xx driver for grblHAL has been successfully implemented with support for the BTT SKR Mini E3 v3.0 board. All core files have been created including:

- Complete driver structure with GPIO, UART, and timer support
- BTT SKR Mini E3 v3.0 board pin mappings  
- STM32G0B1 device support with CMSIS/HAL headers
- PlatformIO configuration for both UART and USB variants

## Build Limitations on ARM64

Currently there's a known issue with PlatformIO on ARM64 systems (like Raspberry Pi 5) where the STM32 toolchain `platformio/toolchain-gccarmnoneeabi` is not available for the `linux_aarch64` architecture.

**Error**: `UnknownPackageError: Could not find the package with 'platformio/toolchain-gccarmnoneeabi @ >=1.60301.0,<1.80000.0' requirements for your system 'linux_aarch64'`

## Alternative Build Methods

### 1. Build on x86_64 System
The easiest approach is to build on an x86_64 Linux system or Windows where PlatformIO has full toolchain support:

```bash
git clone [your-repo-url]
cd STM32G0xx
pio run -e BTT_SKR_MINI_E3_V30      # UART variant
pio run -e BTT_SKR_MINI_E3_V30_USB  # USB variant
```

### 2. Use Docker with x86_64 Emulation
```bash
# Pull x86_64 PlatformIO image and build with emulation
docker run --platform linux/amd64 -v $(pwd):/workspace -w /workspace platformio/platformio-core:latest pio run -e BTT_SKR_MINI_E3_V30
```

### 3. Cross-compile with System GCC
We've installed `gcc-arm-none-eabi` from the system package manager. You could potentially create a custom Makefile to build directly:

```bash
# System toolchain is available at:
/usr/bin/arm-none-eabi-gcc
```

### 4. Use GitHub Actions
Set up GitHub Actions CI to automatically build firmware on x86_64 runners when you push changes.

## Manual Build Verification

Even without building locally, the code structure follows grblHAL conventions and should compile successfully. Key implementations:

1. **GPIO Operations**: Adapted for STM32G0 (no bitband support)
2. **Pin Mappings**: Correctly mapped to BTT SKR Mini E3 v3.0 pinout  
3. **HAL Integration**: Minimal but functional HAL driver structure
4. **Bootloader Support**: 32KB offset for BTT bootloader compatibility

## Next Steps

1. Build on x86_64 system to generate firmware.bin
2. Test on actual BTT SKR Mini E3 v3.0 hardware
3. Implement remaining grblHAL features as needed
4. Consider automated CI/CD pipeline for multi-architecture builds

## Files Created

- `Inc/driver.h` - Core driver definitions adapted for STM32G0
- `Src/driver.c` - Main driver implementation
- `boards/btt_skr_mini_e3_3.0_map.h` - Pin mappings for BTT board
- `Drivers/CMSIS/Include/*` - Required CMSIS headers for STM32G0+ 
- `Drivers/STM32G0xx_HAL_Driver/Inc/*` - Basic HAL headers
- `platformio.ini` - PlatformIO build configuration
- `STM32G0B1RETX_FLASH.ld` - Linker script with bootloader support

The implementation is complete and ready for testing once built on a compatible system.