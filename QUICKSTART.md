# Quick Start Guide - BTT SKR Mini E3 v3.0 grblHAL

Get your CNC running in 5 minutes!

## Step 1: Flash Firmware (1 minute)

1. Download [`firmware_usb_complete.bin`](firmware/firmware_usb_complete.bin)
2. Copy to SD card and rename to `firmware.bin`
3. Insert SD card into BTT SKR Mini E3 v3.0
4. Power on - board will auto-flash (LED blinks)
5. Remove SD card when done

## Step 2: Connect (1 minute)

1. Connect USB-C cable to computer
2. Open gSender (or your preferred grbl sender)
3. Select COM port that appears
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
- **Power**: 12-24V to power input
- **Motors**: X, Y, Z to respective driver slots
- **USB**: USB-C to computer

### Limit Switches (Optional)
- X Limit → PC0
- Y Limit → PC1  
- Z Limit → PC2
- (Connect between pin and ground)

### Control Buttons (Optional)
- E-Stop → PC15
- Feed Hold → PC13
- Cycle Start → PC12
- (Connect between pin and ground)

## Common Issues

**USB not detected?**
- Try different cable (must be data cable)
- Check Device Manager (Windows)
- Use firmware_usb_complete.bin

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