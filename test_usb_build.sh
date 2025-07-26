#!/bin/bash

# Simple test USB firmware build script

echo "Building USB test firmware..."

# Create a minimal test environment in platformio.ini
cd /home/wardnb/e3_mini_v3/STM32G0xx

# Add test environment to platformio.ini
cat >> platformio.ini << 'EOF'

[env:USB_TEST]
platform = ${common.platform}
board = nucleo_g0b1re
board_build.ldscript = STM32G0B1RETX_FLASH.ld
build_flags = ${common.build_flags}
  -D STM32G0B1xx
  -D USE_HAL_DRIVER
src_filter = 
  +<test_usb_direct.c>
  +<../Startup/*.s>
  +<system_stm32g0xx.c>
  +<syscalls.c>
lib_deps = 
EOF

echo "Building test firmware..."
python3 -m platformio run -e USB_TEST

if [ $? -eq 0 ]; then
    echo "✅ USB test firmware built successfully!"
    echo "Location: .pio/build/USB_TEST/firmware.bin"
    ls -la .pio/build/USB_TEST/firmware.*
else
    echo "❌ Build failed"
fi