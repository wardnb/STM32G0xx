# STM32G0xx grblHAL Testing Guide

## Overview
This guide provides comprehensive testing procedures for the STM32G0xx grblHAL implementation on the BTT SKR Mini E3 v3.0 board. Follow these steps systematically to verify all functionality.

## Pre-Test Requirements

### Hardware Required
- BTT SKR Mini E3 v3.0 board
- USB-C cable
- 12-24V power supply
- Stepper motors (at least 3 for X, Y, Z axes)
- Limit switches (normally closed recommended)
- Probe (optional but recommended)
- Emergency stop button (optional but recommended)
- Multimeter for voltage verification

### Software Required
- gSender (https://sienci.com/gsender/) or similar grbl sender
- Serial terminal program (for debugging)
- STM32CubeProgrammer (for firmware updates if needed)

### Safety Checklist
- [ ] Power supply voltage verified (12-24V)
- [ ] All connections double-checked
- [ ] Emergency stop accessible
- [ ] No loose wires or shorts
- [ ] Motors mechanically disconnected initially

## Firmware Installation

### Method 1: SD Card (Recommended)
1. Copy `firmware_usb_complete.bin` to SD card root
2. Rename to `firmware.bin`
3. Insert SD card into BTT SKR Mini E3 v3.0
4. Power cycle the board
5. Board will auto-flash (status LED will blink)
6. Remove SD card after flashing completes

### Method 2: DFU Mode
1. Connect USB while holding BOOT0 button
2. Use STM32CubeProgrammer to flash firmware
3. Reset board after flashing

## Test Plan

### Phase 1: Basic Communication Test

#### 1.1 USB Connection Test
```
Test: USB enumeration and basic communication
Expected: Device appears as COM port
Steps:
1. Connect USB cable to computer
2. Check Device Manager (Windows) or ls /dev/tty* (Linux/Mac)
3. Open gSender or serial terminal
4. Connect at 115200 baud
5. Send '?' command
6. Verify response: <Idle|MPos:0.000,0.000,0.000|...>

Pass Criteria:
- [ ] USB device recognized
- [ ] Serial connection established
- [ ] Status query responds correctly
```

#### 1.2 Basic Command Test
```
Commands to test:
$$ - View settings
$I - View build info
$X - Kill alarm lock
$H - Home (will fail without limits)
? - Status
Ctrl-X - Soft reset

Expected responses documented in each test
```

### Phase 2: I/O Testing

#### 2.1 Limit Switch Test
```
Test: Verify limit switch inputs
Hardware: Connect limit switches to X, Y, Z limit pins
Steps:
1. Send '?' to check initial state
2. Manually trigger each limit switch
3. Send '?' after each trigger
4. Verify status shows limit triggered

Pass Criteria:
- [ ] X limit detected when triggered
- [ ] Y limit detected when triggered  
- [ ] Z limit detected when triggered
- [ ] Debouncing prevents false triggers
```

#### 2.2 Control Input Test
```
Test: Verify control inputs (feed hold, cycle start, reset)
Hardware: Connect buttons to control pins (with pull-ups)
Steps:
1. Start a long G1 move (G1 X100 F100)
2. Press feed hold - motion should pause
3. Press cycle start - motion should resume
4. Test reset button - should halt all motion

Pass Criteria:
- [ ] Feed hold pauses motion
- [ ] Cycle start resumes motion
- [ ] Reset stops all activity
```

#### 2.3 Probe Test
```
Test: Verify probe input
Hardware: Connect probe to probe pin (PC14)
Steps:
1. Send G38.2 Z-10 F50 (probe toward workpiece)
2. Manually trigger probe
3. Verify motion stops immediately
4. Check probe position captured

Pass Criteria:
- [ ] Probe triggers stop motion
- [ ] Position captured accurately
- [ ] No overrun past trigger point
```

### Phase 3: Motion Testing

#### 3.1 Stepper Enable Test
```
Test: Verify stepper enable/disable
Steps:
1. Power on with motors connected
2. Motors should be disabled (free spinning)
3. Send any motion command
4. Motors should lock (enabled)
5. After idle timeout, motors should disable

Pass Criteria:
- [ ] Motors enable on command
- [ ] Motors disable when idle
- [ ] All axes enable/disable together
```

#### 3.2 Basic Motion Test
```
Test: Verify basic motion on all axes
Steps:
1. Send $X to clear any alarms
2. Test each axis individually:
   - G91 (relative mode)
   - G1 X10 F500 (move X 10mm)
   - G1 Y10 F500 (move Y 10mm)
   - G1 Z10 F500 (move Z 10mm)
3. Verify smooth motion, correct direction

Pass Criteria:
- [ ] X axis moves correctly
- [ ] Y axis moves correctly
- [ ] Z axis moves correctly
- [ ] No missed steps or stalling
- [ ] Correct step/mm calibration
```

#### 3.3 Acceleration Test
```
Test: Verify acceleration profiles
Steps:
1. Send G90 (absolute mode)
2. Send rapid moves:
   - G0 X50
   - G0 X0
3. Listen for smooth acceleration/deceleration
4. Test different speeds with F parameter

Pass Criteria:
- [ ] Smooth acceleration ramps
- [ ] No sudden jerks
- [ ] Consistent speeds
```

### Phase 4: Advanced Features

#### 4.1 Spindle Control Test
```
Test: Verify spindle PWM output
Hardware: Connect oscilloscope or LED to spindle PWM pin (PA1)
Steps:
1. M3 S0 - Spindle on, 0 RPM (0% PWM)
2. M3 S12000 - 50% speed (50% PWM)
3. M3 S24000 - Max speed (100% PWM)
4. M5 - Spindle off
5. Verify PWM duty cycle matches

Pass Criteria:
- [ ] PWM signal present on PA1
- [ ] Duty cycle proportional to S value
- [ ] M5 stops PWM output
```

#### 4.2 Coolant Control Test  
```
Test: Verify coolant outputs
Hardware: Connect LEDs to coolant pins (PC8, PC9)
Steps:
1. M7 - Mist on (PC9 high)
2. M8 - Flood on (PC8 high)
3. M9 - All coolant off
4. Verify outputs with multimeter

Pass Criteria:
- [ ] M7 activates mist output
- [ ] M8 activates flood output
- [ ] M9 deactivates both
```

#### 4.3 Homing Cycle Test
```
Test: Full homing cycle
Prerequisite: Limit switches installed and tested
Steps:
1. Position axes away from limits
2. Send $H (home all)
3. Verify homing sequence:
   - Z homes first (up)
   - Then X and Y simultaneously
4. Check machine position zeroed

Pass Criteria:
- [ ] Correct homing order (Z, then XY)
- [ ] Stops on limit switch
- [ ] Backs off correct distance
- [ ] Machine position set to 0,0,0
```

### Phase 5: Performance Testing

#### 5.1 High-Speed Motion Test
```
Test: Verify maximum speeds
Steps:
1. Test increasing feed rates:
   - G1 X50 F1000
   - G1 X0 F2000
   - G1 X50 F5000
   - G1 X0 F10000
2. Find maximum reliable speed
3. Verify no missed steps

Pass Criteria:
- [ ] Achieves expected max speed
- [ ] No missed steps at max speed
- [ ] Returns to exact start position
```

#### 5.2 Complex Path Test
```
Test: Run complex G-code
Steps:
1. Load test pattern (circle, square, etc.)
2. Run at various speeds
3. Verify accuracy with calipers
4. Check for smooth motion

Example circle test:
G90 G17
G0 X0 Y25
G2 X0 Y25 I0 J-25
G0 X0 Y0

Pass Criteria:
- [ ] Smooth curves
- [ ] Accurate dimensions
- [ ] Consistent speed
```

### Phase 6: Stress Testing

#### 6.1 Long Duration Test
```
Test: Extended operation stability
Steps:
1. Create long-running G-code (1+ hours)
2. Monitor for:
   - Communication errors
   - Missed steps
   - Temperature issues
   - Memory leaks (status remains responsive)

Pass Criteria:
- [ ] Completes without errors
- [ ] No communication dropouts
- [ ] Temperature remains safe
```

#### 6.2 Rapid Command Test
```
Test: Command buffer handling
Steps:
1. Send many commands rapidly
2. Verify queuing works correctly
3. Test feed hold during queue
4. Verify no commands lost

Pass Criteria:
- [ ] All commands executed
- [ ] Proper queue handling
- [ ] Feed hold works mid-queue
```

## Troubleshooting Guide

### USB Not Recognized
1. Check USB cable (must be data capable)
2. Verify firmware flashed correctly
3. Try different USB port
4. Check Device Manager for errors

### No Motor Movement
1. Verify motor enable signal
2. Check step pulse with oscilloscope
3. Verify motor wiring
4. Check driver current settings

### Erratic Movement
1. Check motor current (may be too low)
2. Verify step/mm settings ($100, $101, $102)
3. Check for mechanical binding
4. Reduce acceleration if needed

### Limit Switches Not Working
1. Verify NO/NC configuration
2. Check wiring continuity
3. Test with multimeter
4. May need pull-up resistors

## Configuration Settings

### Essential Settings to Configure
```
$0 - Step pulse time (µs)
$1 - Step idle delay (ms)
$2 - Step port invert mask
$3 - Direction port invert mask
$4 - Step enable invert
$5 - Limit pins invert
$6 - Probe pin invert
$10 - Status report mask
$11 - Junction deviation (mm)
$12 - Arc tolerance (mm)
$20 - Soft limits enable
$21 - Hard limits enable
$22 - Homing cycle enable
$23 - Homing dir invert mask
$24 - Homing feed (mm/min)
$25 - Homing seek (mm/min)
$26 - Homing debounce (ms)
$27 - Homing pull-off (mm)
$100 - X steps/mm
$101 - Y steps/mm
$102 - Z steps/mm
$110 - X Max rate (mm/min)
$111 - Y Max rate (mm/min)
$112 - Z Max rate (mm/min)
$120 - X Acceleration (mm/sec²)
$121 - Y Acceleration (mm/sec²)
$122 - Z Acceleration (mm/sec²)
```

### Recommended Starting Values
```
$0=10 (step pulse 10µs)
$1=25 (step idle delay)
$4=7 (invert all enables)
$5=0 (limits NO)
$20=0 (soft limits off initially)
$21=1 (hard limits on)
$22=1 (homing enabled)
$24=25.000 (homing feed)
$25=500.000 (homing seek)
$26=250 (debounce)
$27=1.000 (pull-off)
$100=80 (typical for 16 microstep)
$101=80
$102=400 (Z typically different)
$110=500 (conservative max)
$111=500
$112=500
$120=10 (low acceleration initially)
$121=10
$122=10
```

## Test Log Template

```
Date: _____________
Tester: ___________
Board Version: BTT SKR Mini E3 v3.0
Firmware: firmware_usb_complete.bin

Basic Communication:     [ ] Pass  [ ] Fail
Limit Switches:          [ ] Pass  [ ] Fail  
Control Inputs:          [ ] Pass  [ ] Fail
Probe:                   [ ] Pass  [ ] Fail
Motion X:                [ ] Pass  [ ] Fail
Motion Y:                [ ] Pass  [ ] Fail
Motion Z:                [ ] Pass  [ ] Fail
Spindle Control:         [ ] Pass  [ ] Fail
Coolant:                 [ ] Pass  [ ] Fail
Homing:                  [ ] Pass  [ ] Fail
High Speed:              [ ] Pass  [ ] Fail
Complex Paths:           [ ] Pass  [ ] Fail

Notes:
_________________________________
_________________________________
_________________________________

Overall Result:          [ ] Pass  [ ] Fail
```

## Safety Reminders

1. **Always have emergency stop ready**
2. **Start with low speeds and accelerations**
3. **Test without tool/spindle first**
4. **Verify motor current settings**
5. **Check all connections before power**
6. **Never bypass safety features**

## Support Resources

- grblHAL Wiki: https://github.com/grblHAL/core/wiki
- gSender Docs: https://resources.sienci.com/view/gs-installation/
- BTT SKR Mini E3 v3.0 Docs: https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3

## Next Steps

After successful testing:
1. Fine-tune acceleration and speed settings
2. Calibrate steps/mm precisely
3. Set up soft limits
4. Configure spindle speed curve
5. Test with actual cutting operations