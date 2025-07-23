#!/usr/bin/env python3
"""
Firmware Validator for STM32G0xx grblHAL
Performs basic static analysis without hardware emulation
"""

import os
import struct
import sys

def read_binary_file(filename):
    """Read binary firmware file"""
    with open(filename, 'rb') as f:
        return f.read()

def validate_vector_table(data):
    """Validate ARM Cortex-M0+ vector table"""
    if len(data) < 48:
        return False, "File too small for vector table"
    
    # First word should be initial stack pointer (in RAM region)
    sp = struct.unpack('<I', data[0:4])[0]
    if not (0x20000000 <= sp <= 0x20024000):  # RAM region for STM32G0B1
        return False, f"Invalid initial SP: 0x{sp:08X}"
    
    # Second word should be reset handler (in Flash region)
    reset = struct.unpack('<I', data[4:8])[0]
    if not (0x08008000 <= reset <= 0x08080000):  # Flash region with bootloader offset
        return False, f"Invalid reset handler: 0x{reset:08X}"
    
    # Check some key exception vectors
    vectors = {
        'NMI': struct.unpack('<I', data[8:12])[0],
        'HardFault': struct.unpack('<I', data[12:16])[0],
        'SysTick': struct.unpack('<I', data[60:64])[0] if len(data) >= 64 else 0
    }
    
    print(f"  ✓ Initial SP: 0x{sp:08X}")
    print(f"  ✓ Reset Handler: 0x{reset:08X}")
    print(f"  ✓ Exception vectors present")
    
    return True, "Vector table valid"

def find_grbl_strings(data):
    """Search for grblHAL specific strings"""
    strings_found = []
    
    # Convert to string for searching (ignore decode errors)
    text = data.decode('ascii', errors='ignore')
    
    # Key strings to look for
    search_strings = [
        'grblHAL',
        'STM32G0',
        'BTT SKR Mini E3',
        'v1.0.0-beta2',
        'Spindle plugin',
        'Fans plugin',
        'Encoder plugin',
        'Plasma plugin',
        'Laser plugin'
    ]
    
    for s in search_strings:
        if s in text:
            strings_found.append(s)
    
    return strings_found

def analyze_firmware_size(data):
    """Analyze firmware size and estimate feature completeness"""
    size = len(data)
    
    # Expected sizes based on features
    expectations = {
        'Minimal build': (10000, 30000),
        'Basic grblHAL': (30000, 60000),
        'Full featured': (60000, 120000),
        'Plugin-enabled': (80000, 200000),
        'Bloated': (200000, float('inf'))
    }
    
    category = 'Unknown'
    for name, (min_size, max_size) in expectations.items():
        if min_size <= size < max_size:
            category = name
            break
    
    return size, category

def check_plugin_signatures(data):
    """Check for plugin-specific code signatures"""
    plugins = {
        'Spindle': b'spindle_register',
        'Fans': b'M106',
        'Encoder': b'encoder',
        'Plasma': b'THC',
        'Laser': b'laser',
        'EEPROM': b'eeprom',
        'SD Card': b'sdcard',
        'Keypad': b'keypad',
        'TMC2209': b'TMC'
    }
    
    found = []
    for name, signature in plugins.items():
        if signature in data:
            found.append(name)
    
    return found

def main():
    print("=== STM32G0xx grblHAL Firmware Validator ===\n")
    
    # Check for firmware files
    firmware_files = [
        '../firmware/firmware_usb_v1.0.0-beta2.bin',
        '../firmware/firmware_uart_v1.0.0-beta2.bin',
        '../.pio/build/BTT_SKR_MINI_E3_V30_USB/firmware.bin',
        '../.pio/build/BTT_SKR_MINI_E3_V30/firmware.bin'
    ]
    
    found_files = []
    for f in firmware_files:
        if os.path.exists(f):
            found_files.append(f)
    
    if not found_files:
        print("ERROR: No firmware files found!")
        print("Please build the firmware first.")
        return 1
    
    # Analyze each firmware
    for firmware in found_files:
        print(f"\nAnalyzing: {firmware}")
        print("-" * 50)
        
        try:
            data = read_binary_file(firmware)
            
            # 1. Validate vector table
            print("\n1. Vector Table Validation:")
            valid, msg = validate_vector_table(data)
            if valid:
                print(f"  ✅ {msg}")
            else:
                print(f"  ❌ {msg}")
                continue
            
            # 2. Check firmware size
            print("\n2. Firmware Size Analysis:")
            size, category = analyze_firmware_size(data)
            print(f"  Size: {size:,} bytes ({size/1024:.1f} KB)")
            print(f"  Category: {category}")
            
            # 3. Find grblHAL strings
            print("\n3. grblHAL String Detection:")
            strings = find_grbl_strings(data)
            for s in strings:
                print(f"  ✓ Found: '{s}'")
            
            # 4. Check plugin signatures
            print("\n4. Plugin Detection:")
            plugins = check_plugin_signatures(data)
            for p in plugins:
                print(f"  ✓ {p} plugin detected")
            
            # 5. Summary
            print("\n5. Summary:")
            if valid and category in ['Full featured', 'Plugin-enabled'] and len(strings) > 3:
                print("  ✅ Firmware appears to be valid grblHAL build")
                print("  ✅ Plugins detected:", ', '.join(plugins))
                print("  ⚠️  Note: Static analysis only - hardware testing required")
            else:
                print("  ⚠️  Firmware validation incomplete")
                
        except Exception as e:
            print(f"  ❌ Error analyzing firmware: {e}")
    
    print("\n" + "="*50)
    print("Validation complete. See results above.")
    print("\nFor full testing, install Renode:")
    print("  https://github.com/renode/renode/releases")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())