# IndiCNC DIY CNC Setup Guide

Complete setup guide for upgrading your IndiCNC DIY CNC machine with BTT SKR Mini E3 v3.0 and grblHAL firmware.

## Why Upgrade to grblHAL?

### Benefits over Stock IndiCNC Controller:
- **Modern firmware** with active development
- **USB connectivity** - no more parallel port issues  
- **Real-time control** with gSender, UGS, CNCjs
- **Advanced features** - probing, workspace coordinates, tool library
- **Better performance** - 32-bit ARM vs 8-bit Arduino
- **Future-proof** - continuous updates and improvements

### IndiCNC Machine Compatibility:
- ✅ **All IndiCNC router models** (600x400, 800x600, 1200x800, etc.)
- ✅ **Standard NEMA 23 steppers** 
- ✅ **Original limit switches**
- ✅ **Spindle control** (with VFD upgrade)
- ✅ **Existing wiring harness** (with adaptations)

## Hardware Requirements

### What You'll Need:
- **BTT SKR Mini E3 v3.0** board (~$35)
- **TMC2209 stepper drivers** x3-4 (~$12 each)
- **PWM to 0-10V converter** for spindle (~$15)
- **24V power supply** 5-10A (~$25)
- **Micro USB cable** (data capable)
- **SD card** (8-32GB, FAT32)
- **Enclosure** for electronics
- **Wire and connectors**

### Optional Upgrades:
- **VFD and spindle** (if not already upgraded)
- **Probe plate** for tool length setting
- **E-stop button** and contactor
- **Pendant/MPG** via I2C

## IndiCNC Machine Analysis

### Typical IndiCNC Specifications:
```
Machine Type: 3-axis router (X, Y, Z)
Work Area: 600x400mm to 1200x800mm (varies by model)
Motors: NEMA 23 steppers (2.8A typical)
Drive: Lead screw (8mm pitch typical)
Limits: Mechanical switches (NO/NC varies)
Spindle: Router or upgraded to spindle+VFD
Frame: Aluminum extrusion with steel plates
```

### Current Wiring Assessment:
Most IndiCNC machines have:
- **Stepper motor cables** (4-wire, shielded)
- **Limit switch wiring** (3-wire to each switch)  
- **Spindle control** (if VFD upgraded)
- **Emergency stop** (if installed)

## Mechanical Preparation

### 1. Electronics Enclosure
**Mount the BTT SKR Mini E3 v3.0 in a protected enclosure:**
- Use existing control box or add new one
- Ensure adequate ventilation for TMC2209 drivers
- Keep away from spindle motor (EMI)
- Easy access for USB connection

### 2. Motor Driver Current Settings
**IndiCNC typically uses NEMA 23 steppers:**
```
Motor Current: 2.0-2.8A (check motor label)
TMC2209 Setting: Set via firmware
Microstepping: 1/16 (default)
```

### 3. Limit Switch Check
**Verify your existing limit switches:**
- **Mechanical type**: Usually NO (Normally Open)
- **Wiring**: COM, NO, NC terminals
- **Position**: End of travel for each axis

## Step-by-Step Installation

### Phase 1: Board Preparation

1. **Install TMC2209 Drivers**
   ```
   - Check orientation (VM pin alignment)
   - Install heatsinks
   - No jumper changes needed (UART mode built-in)
   ```

2. **Flash grblHAL Firmware**
   ```
   - Format SD card as FAT32
   - Download firmware_usb_complete.bin
   - Rename to firmware.bin
   - Insert SD card and power cycle
   - Wait for LED to stop blinking
   ```

3. **Initial USB Test**
   ```
   - Connect Micro USB to computer
   - Open gSender or UGS
   - Look for "STMicroelectronics Virtual COM Port"
   - Connect at 115200 baud
   - Send ? command to verify communication
   ```

### Phase 2: Motor Connection

**Map IndiCNC motors to board connectors:**

| IndiCNC Axis | Board Connector | Notes |
|--------------|----------------|-------|
| X-Axis (Left/Right) | X-MOTOR | Usually longest travel |
| Y-Axis (Front/Back) | Y-MOTOR | Gantry movement |
| Z-Axis (Up/Down) | Z-MOTOR | Spindle vertical |

**Wiring Notes:**
- Use existing 4-wire stepper cables
- If wires don't match board connector, adapt as needed
- Test direction - may need to swap one coil pair

### Phase 3: Limit Switch Connection

**Connect to board limit switch inputs:**

| IndiCNC Switch | Board Connector | Pin |
|----------------|-----------------|-----|
| X Home/Limit | X-STOP | PC0 |
| Y Home/Limit | Y-STOP | PC1 |
| Z Home/Limit | Z-STOP | PC2 |

**Wiring:**
```
Switch COM → Signal pin on board connector
Switch NO → Ground pin on board connector
(Leave +V pin unconnected)
```

### Phase 4: Spindle Control

**Option A: Router Control (Simple)**
```
Board HEAT0 (PC7) → Relay → Router Power
- Use 24V relay module
- Router plugged into relay-controlled outlet
- M3 turns router on, M5 turns off
```

**Option B: VFD Control (Professional)**
```
Board PA1 → PWM-to-0-10V Converter → VFD AI1
Board FAN1 (PC6) → VFD FOR (Forward)
Board HEAT0 (PC7) → VFD RUN
Board GND → VFD ACM

See VFD_SPINDLE_SETUP.md for complete details
```

### Phase 5: Power System

**Power Distribution:**
```
24V Power Supply
├── BTT SKR Mini E3 v3.0 (2A)
├── Stepper Motors (2A each × 3 = 6A)
├── VFD Control Power (0.5A)
└── Relay Modules (0.2A)

Total: ~10A minimum power supply
```

**Recommended Wiring:**
- Main 24V bus with distribution blocks
- Individual fuses for each subsystem
- Emergency stop cuts motor power via contactor

## Software Configuration

### Initial grblHAL Settings for IndiCNC:

```gcode
; Clear any alarms
$X

; Motor configuration (adjust for your motors)
$100=160      ; X steps/mm (200*16/20 for 20mm belt pitch)
$101=160      ; Y steps/mm  
$102=800      ; Z steps/mm (200*16/2.5 for 2.5mm/rev leadscrew)

; Maximum rates (start conservative)
$110=3000     ; X max rate mm/min
$111=3000     ; Y max rate mm/min
$112=1000     ; Z max rate mm/min (slower for Z)

; Acceleration (start low, increase as tuned)
$120=100      ; X acceleration mm/sec²
$121=100      ; Y acceleration mm/sec²
$122=50       ; Z acceleration mm/sec²

; Motor direction (may need adjustment)
$3=0          ; Direction invert mask (test each axis)

; Limit switches
$5=0          ; Limit invert (0 for NO switches)
$21=1         ; Hard limits enable
$22=1         ; Homing cycle enable

; Homing configuration
$23=0         ; Homing direction invert (toward switches)
$24=100       ; Homing feed rate mm/min
$25=1000      ; Homing seek rate mm/min
$26=250       ; Homing debounce ms
$27=2.0       ; Homing pull-off mm

; Spindle settings
$30=24000     ; Max spindle RPM (adjust for your setup)
$31=0         ; Min spindle RPM
$32=0         ; Laser mode off (use spindle mode)

; Work area limits (adjust for your IndiCNC model)
$130=600      ; X max travel (mm) - adjust for your machine
$131=400      ; Y max travel (mm) - adjust for your machine  
$132=100      ; Z max travel (mm) - adjust for your machine

; Enable soft limits
$20=1         ; Soft limits enable (after homing works)
```

### Testing Sequence:

1. **Motor Direction Test**
   ```gcode
   G91           ; Relative mode
   G1 X10 F500   ; X should move RIGHT (positive direction)
   G1 Y10 F500   ; Y should move AWAY from operator (back)
   G1 Z10 F500   ; Z should move UP (away from table)
   
   ; If wrong direction, adjust $3 setting
   ```

2. **Limit Switch Test**
   ```gcode
   ; Manually trigger each limit switch
   ; Send ? to check status
   ; Should show Pn:X, Pn:Y, or Pn:Z when triggered
   ```

3. **Homing Test**
   ```gcode
   $H            ; Home all axes
   ; Should move toward switches, touch, back off
   ; Positions should be set to 0,0,0
   ```

## IndiCNC-Specific Calibration

### Steps per MM Calculation:

**For Belt Drive (X/Y typically):**
```
Steps/mm = (Motor Steps × Microstepping) / Belt Pitch
Example: (200 × 16) / 20mm = 160 steps/mm
```

**For Lead Screw (Z typically):**
```
Steps/mm = (Motor Steps × Microstepping) / Screw Pitch  
Example: (200 × 16) / 2.5mm = 1280 steps/mm
```

**Calibration Process:**
1. Mark current position
2. Command 100mm move
3. Measure actual distance
4. Adjust: New Steps/mm = Old × (Commanded / Actual)

### Work Area Setup:

**Measure your IndiCNC work envelope:**
```
1. Home machine ($H)
2. Manually jog to maximum safe positions
3. Note coordinates from ? status
4. Set $130, $131, $132 accordingly
5. Enable soft limits with $20=1
```

## Spindle Configuration for IndiCNC

### Router Setup (Basic):
```gcode
; Router on/off only
$30=1         ; Max RPM (1 = on/off mode)
$31=0         ; Min RPM
$32=0         ; Laser mode off

; Usage:
M3            ; Router on
M5            ; Router off
```

### VFD Spindle Setup (Advanced):
```gcode
; Full speed control
$30=24000     ; Max spindle RPM (match your spindle)
$31=0         ; Min spindle RPM
$32=0         ; Laser mode off

; Usage:
M3 S12000     ; 12,000 RPM
M3 S24000     ; 24,000 RPM (max)
M5            ; Stop spindle
```

## Safety Integration

### Emergency Stop Implementation:
```
E-Stop Button → Motor Power Contactor → All Motors
              → Spindle Power Contactor → Spindle
              → Board Input (PC15) → Software Alarm
```

### Work Holding and Safety:
- **Secure workpieces** properly
- **Test programs** at low feed rates first  
- **Keep emergency stop** within reach
- **Wear safety equipment** (glasses, hearing protection)

## Performance Tuning

### Acceleration Tuning:
```
1. Start with low values ($120-122 = 50-100)
2. Gradually increase until motors start missing steps
3. Back off 10-20% for safety margin
4. Test with actual cutting loads
```

### Feed Rate Optimization:
```
1. Test maximum speeds with no cutting load
2. Reduce for actual cutting (typically 50-70% of max)
3. Adjust for material (aluminum slower than wood)
4. Monitor for missed steps or poor surface finish
```

## Troubleshooting Common Issues

### Motors Not Moving:
```
1. Check TMC2209 installation (orientation)
2. Verify motor current setting ($140-142)
3. Test motor connections (multimeter continuity)
4. Check enable signals ($4 setting)
```

### Homing Problems:
```
1. Verify limit switch wiring
2. Test switches with ? command
3. Check homing direction ($23 setting)  
4. Adjust homing speeds ($24, $25)
```

### Poor Accuracy:
```
1. Recalibrate steps/mm ($100-102)
2. Check mechanical backlash
3. Verify belt tension (if belt drive)
4. Test with precision measurement tools
```

### USB Connection Issues:
```
1. Use data-capable Micro USB cable
2. Install STM32 VCP drivers if needed
3. Try USB 2.0 ports
4. Check Windows Device Manager
```

## Recommended Workflow

### Daily Operation:
```
1. Power on system
2. Connect gSender/UGS
3. Home machine ($H)
4. Load G-code file
5. Set work coordinates (G54)
6. Run tool probing (if equipped)
7. Start program with feed override ready
```

### Maintenance Schedule:
- **Weekly**: Clean rails and check belt tension
- **Monthly**: Lubricate lead screws
- **Quarterly**: Check electrical connections
- **Annually**: Calibrate steps/mm accuracy

## Upgrade Path

### Phase 1: Basic CNC Control
- BTT SKR Mini E3 v3.0 + grblHAL
- Existing motors and switches
- Router control via relay

### Phase 2: Spindle Upgrade  
- Add VFD and spindle
- PWM speed control
- Improved cutting performance

### Phase 3: Advanced Features
- Tool length probe
- Automatic tool changer (ATC)
- Fourth axis (rotary)
- Flood coolant system

## Cost Analysis

### Initial Upgrade Cost:
```
BTT SKR Mini E3 v3.0:    $35
TMC2209 Drivers (3):     $36  
24V Power Supply:        $25
PWM Converter:           $15
Misc Hardware:           $20
Total:                   ~$130
```

### ROI Benefits:
- **Reliability**: Modern 32-bit control vs aging parallel port
- **Software**: Modern CAM software compatibility
- **Performance**: Faster processing, smoother motion
- **Features**: Probing, work coordinates, tool library

## Support Resources

### IndiCNC Community:
- IndiCNC forums and user groups
- Local maker spaces with CNC experience
- YouTube channels for specific techniques

### grblHAL Resources:
- [grblHAL Wiki](https://github.com/grblHAL/core/wiki)
- [This project's guides](README.md)
- gSender community and documentation

---

**Remember: Take your time with the conversion. Test each system individually before integrating everything. Your IndiCNC will perform better than ever with modern grblHAL control!**

🔧 *Happy machining with your upgraded IndiCNC!* 🔧