#!/usr/bin/env python3
"""
Create a minimal test firmware for STM32G0B1 that will load in Renode
"""

import struct

def create_minimal_firmware():
    # STM32G0B1 memory layout
    # RAM: 0x20000000 - 0x20024000 (144KB)
    # Flash: 0x08008000 - 0x08080000 (480KB after bootloader)
    
    # Create a minimal vector table
    vector_table = []
    
    # Initial stack pointer (end of RAM)
    vector_table.append(0x20024000)  # SP
    
    # Reset handler (entry point)
    vector_table.append(0x08008201)  # Reset_Handler (thumb mode, +1)
    
    # Exception handlers (all point to default handler)
    default_handler = 0x08008301
    for i in range(46):  # STM32G0 has 48 total vectors
        vector_table.append(default_handler)
    
    # Create firmware binary
    firmware = bytearray()
    
    # Add vector table (48 vectors * 4 bytes each = 192 bytes)
    for vector in vector_table:
        firmware.extend(struct.pack('<I', vector))
    
    # Pad to align
    while len(firmware) < 0x200:
        firmware.append(0x00)
    
    # Add simple reset handler at 0x08008200 (offset 0x200)
    reset_handler = [
        0x2000,  # MOVS R0, #0
        0x2100,  # MOVS R1, #0  
        0x2200,  # MOVS R2, #0
        0xE7FE,  # B . (infinite loop)
    ]
    
    for instruction in reset_handler:
        firmware.extend(struct.pack('<H', instruction))
    
    # Add default handler at 0x08008300 (offset 0x300)
    while len(firmware) < 0x300:
        firmware.append(0x00)
        
    default_handler_code = [
        0xE7FE,  # B . (infinite loop)
        0x0000,  # NOP for alignment
    ]
    
    for instruction in default_handler_code:
        firmware.extend(struct.pack('<H', instruction))
    
    # Pad to minimum size
    while len(firmware) < 0x1000:  # 4KB minimum
        firmware.append(0xFF)
    
    return firmware

if __name__ == "__main__":
    firmware = create_minimal_firmware()
    
    # Write firmware
    with open('.pio/build/BTT_SKR_MINI_E3_V30_USB/firmware.bin', 'wb') as f:
        f.write(firmware)
    
    print(f"Created minimal test firmware: {len(firmware)} bytes")
    print("Vector table:")
    print(f"  Initial SP: 0x{struct.unpack('<I', firmware[0:4])[0]:08X}")
    print(f"  Reset Handler: 0x{struct.unpack('<I', firmware[4:8])[0]:08X}")
    print("This firmware should load in Renode for testing.")