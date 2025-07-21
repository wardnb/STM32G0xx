# Quick Start Guide - BTT SKR Mini E3 v3.0 grblHAL

Get your CNC running in 5 minutes!

## Step 1: Flash Firmware (1 minute)

1. Download [`firmware_usb_complete.bin`](firmware/firmware_usb_complete.bin)
2. Format SD card as FAT32 (max 32GB)
3. Copy firmware to SD card and rename to `firmware.bin`
4. Insert SD card into TF card slot on board
5. Power on - board will auto-flash (LED blinks)
6. Remove SD card when done (file renamed to FIRMWARE.CUR)

## Step 2: Connect (1 minute)

1. Connect Micro USB cable to computer
2. Open gSender (or your preferred grbl sender)
3. Select COM port that appears (STMicroelectronics Virtual COM Port)
4. Connect at 115200 baud
5. You should see "Grbl 1.1" response

## Step 3: Basic Setup (3 minutes)

Send these commands to configure your machine:

```gcode
; Motor direction - adjust if motors run backwards
$3=0  ; Normal direction (change bits if needed)

; Steps per mm - ADJUST FOR YOUR SETUP
$100=80   ; X steps/mm 
$101=80   ; Y steps/mm
$102=400  ; Z steps/mm (usually different)

; Maximum speeds (start conservative)
$110=500  ; X max rate mm/min
$111=500  ; Y max rate mm/min  
$112=500  ; Z max rate mm/min

; Acceleration (start low)
$120=50   ; X accel mm/sec²
$121=50   ; Y accel mm/sec²
$122=50   ; Z accel mm/sec²

; Enable hard limits (if switches connected)
$21=1     ; Hard limits on
$22=0     ; Homing cycle off (until configured)
```

## Step 4: Test Movement

1. **Check motor enable**: Motors should lock when you connect
2. **Test jog**: Use gSender jog buttons (start with 1mm moves)
3. **Verify direction**: +X right, +Y back, +Z up
4. **Run test pattern**:
   ```gcode
   G90      ; Absolute mode
   G21      ; Millimeter mode
   F200     ; Feed rate 200mm/min
   G1 X10   ; Move X to 10mm
   G1 Y10   ; Move Y to 10mm
   G1 X0 Y0 ; Return to origin
   ```

## Wiring Quick Reference

### Essential Connections
- **Power**: 12-24V to DC jack or VIN/GND terminals
- **Motors**: Connect to X, Y, Z, E0 motor connectors
- **TMC2209**: Install in driver sockets (check orientation!)
- **USB**: Micro USB to computer

### Limit Switches (Optional)
- X Limit → X-STOP connector (PC0)
- Y Limit → Y-STOP connector (PC1)  
- Z Limit → Z-STOP connector (PC2)
- Wiring: COM to Signal, NC/NO to Ground

### Control Buttons (Optional)
- Probe → PROBE connector (PC14)
- E-Stop → PC15 (via EXP1/EXP2)
- Feed Hold → PC13 (via EXP1/EXP2)
- Cycle Start → PC12 (via EXP1/EXP2)
- Wiring: Button between Signal and Ground

## Common Issues

**USB not detected?**
- Try different Micro USB cable (must be data cable)
- Check Device Manager for "STMicroelectronics Virtual COM Port"
- Ensure using firmware_usb_complete.bin (not UART version)
- Try pressing RESET button after connecting

**Motors not moving?**
- Check power supply is on
- Verify motor connections
- Try inverting enable with $4=7

**Movement too slow/fast?**
- Adjust $110-112 (max speeds)
- Check $100-102 (steps/mm)

## Next Steps

1. **Fine-tune settings**: Adjust speeds and acceleration for your machine
2. **Setup homing**: Configure limit switches and homing cycle
3. **Calibrate**: Verify movement accuracy with test cuts
4. **Read full guide**: See [TESTING_GUIDE.md](TESTING_GUIDE.md) for comprehensive testing

## Need Help?

- Status check: Send `?` anytime
- View settings: Send `$$`
- Emergency stop: Send `Ctrl+X`
- Full docs: [README.md](README.md)

---
Happy CNCing! 🎉