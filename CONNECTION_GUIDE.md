# BTT SKR Mini E3 v3.0 CNC Connection Guide

This guide translates the 3D printer-oriented labels on the BTT SKR Mini E3 v3.0 board to CNC machine connections.

## Overview - Board Layout

```
[USB]  [POWER]  [MOTOR DRIVERS: X Y Z E0]
[TF CARD]

[PROBE] [SERVO] [RGB]    [X-STOP] [Y-STOP] [Z-STOP] [E0-STOP] [PT100]

[FAN0] [FAN1] [FAN2]     [HEAT0] [HEAT1] [TB]

[EXP1]                   [EXP2]                    [TFT]
```

## Motor Connections

### Main Axes - Use 3D Printer Motor Connectors

| 3D Printer Label | CNC Function | Pin | Wire Colors (Typical) | Notes |
|------------------|--------------|-----|----------------------|-------|
| **X-MOTOR** | **X-Axis Motor** | PB13/PB12/PB14 | Black/Green/Blue/Red | Left-Right movement |
| **Y-MOTOR** | **Y-Axis Motor** | PB10/PB2/PB11 | Black/Green/Blue/Red | Front-Back movement |
| **Z-MOTOR** | **Z-Axis Motor** | PB0/PC5/PB1 | Black/Green/Blue/Red | Up-Down movement |
| **E0-MOTOR** | **A-Axis (4th)** | PB3/PB4/PD2 | Black/Green/Blue/Red | Optional rotary axis |

**Wiring**: Standard 4-wire stepper connection
- A+/A- = One coil pair (usually Black/Green)
- B+/B- = Other coil pair (usually Red/Blue)

## Limit Switch Connections

### Use 3D Printer Endstop Connectors

| 3D Printer Label | CNC Function | Pin | Wiring | Notes |
|------------------|--------------|-----|--------|-------|
| **X-STOP** | **X Limit Switch** | PC0 | COM→Signal, NC→GND | Home/limit for X |
| **Y-STOP** | **Y Limit Switch** | PC1 | COM→Signal, NC→GND | Home/limit for Y |
| **Z-STOP** | **Z Limit Switch** | PC2 | COM→Signal, NC→GND | Home/limit for Z |
| **E0-STOP** | **A Limit** or **Door** | PA2 | COM→Signal, NC→GND | Optional 4th limit |

**Switch Types**:
- Mechanical switches: Use NC (Normally Closed) for safety
- Optical switches: Check voltage (3.3V logic)
- Proximity sensors: Ensure 3.3V compatible

## Probe Connection

| 3D Printer Label | CNC Function | Pin | Wiring | Notes |
|------------------|--------------|-----|--------|-------|
| **PROBE** | **Tool Probe** | PC14 | Probe→Signal, Work→GND | Auto tool length |

**Probe Setup**:
- Conductive probe (tool touches plate)
- Signal pin goes HIGH when contact made
- Use $6 setting to invert if needed

## Spindle Connections

### Spindle Control Uses Fan/Heater Outputs

| 3D Printer Label | CNC Function | Pin | Wire | Notes |
|------------------|--------------|-----|------|-------|
| **FAN1** | **Spindle Direction** | PC6 | To VFD DIR+ | HIGH = Forward |
| **HEAT0** | **Spindle Enable** | PC7 | To VFD Enable | HIGH = Run |
| **PA1** | **Spindle PWM** | PA1 | To VFD AI1 | 0-3.3V PWM |

**VFD Wiring Example**:
```
SKR Mini E3 → VFD
PC6 (FAN1) → FOR (Forward Run)
PC7 (HEAT0) → Enable/Run
PA1 → AI1 (Analog Input)
GND → ACM (Analog Common)
```

**PWM Spindle (No VFD)**:
```
PA1 → PWM Speed Controller
PC7 → Relay/MOSFET Enable
```

## Coolant Connections

| 3D Printer Label | CNC Function | Pin | Wire | Notes |
|------------------|--------------|-----|------|-------|
| **FAN2** | **Flood Coolant** | PC8 | To relay | M8 command |
| **HEAT1** | **Mist Coolant** | PC9 | To relay | M7 command |

**Relay Wiring**:
- Use relay module (3.3V logic)
- Connect pump/solenoid through relay
- Max 50mA from board pins

## Control Panel Connections

### Use EXP1/EXP2 or TFT Connector

| Function | Suggested Connection | Pin | Notes |
|----------|---------------------|-----|-------|
| **E-Stop** | See dedicated guide | PC15 | ⚠️ CRITICAL - Hardware power cut required |
| **Feed Hold** | EXP1 Pin | PC13 | Pause button (momentary NO) |
| **Cycle Start** | EXP1 Pin | PC12 | Resume button (momentary NO) |
| **Reset** | Reset button on board | NRST | System reset |

### ⚠️ EMERGENCY STOP - CRITICAL SAFETY

**DO NOT rely on software-only E-stop!**

The E-stop must:
1. **Cut motor power physically** (via contactor)
2. **Stop spindle immediately** 
3. **Signal the board** for software awareness

See **[ESTOP_WIRING.md](ESTOP_WIRING.md)** for complete safety implementation.

**Basic Control Wiring**:
- Feed Hold/Cycle Start: Momentary switches (NO type)
- Wire between pin and GND
- Built-in pull-ups enabled

## Power Connections

| Connector | Function | Spec | Notes |
|-----------|----------|------|-------|
| **POWER** | Main Power | 12-24V DC | 2A minimum, 10A typical |
| **TB** | Heated Bed | 12-24V DC | Not used for CNC |
| **USB** | Communication | 5V | Micro USB to PC |

**Power Supply Requirements**:
- 12V: Suitable for small CNC
- 24V: Recommended for better motor performance
- Current: 2A + (motor current × number of motors)

## Communication Options

### USB (Recommended)
- **Connector**: Micro USB
- **Function**: Direct PC connection
- **Baud**: 115200 (automatic)
- **Driver**: STM32 Virtual COM Port

### UART via TFT
| TFT Pin | Function | Connect To |
|---------|----------|------------|
| TX | Board TX (PA9) | FTDI RX |
| RX | Board RX (PA10) | FTDI TX |
| GND | Ground | FTDI GND |
| 5V | Power | FTDI 5V (optional) |

## Expansion Connections

### I2C Devices (EXP1 Connector)
| Pin | Function | Wire | Example Devices |
|-----|----------|------|-----------------|
| PB8 | SCL | Clock | OLED Display |
| PB9 | SDA | Data | I2C Keypad |
| 3.3V | Power | VCC | EEPROM |
| GND | Ground | GND | Sensors |

### SD Card
- **TF Slot**: For firmware updates
- **Not for G-code**: Use USB/UART for G-code streaming

## Recommended CNC Configuration

### 3-Axis Router/Mill
```
X-MOTOR: X-Axis stepper
Y-MOTOR: Y-Axis stepper  
Z-MOTOR: Z-Axis stepper
X-STOP: X home/limit switch
Y-STOP: Y home/limit switch
Z-STOP: Z home/limit switch
PROBE: Tool length probe
FAN1+HEAT0: Spindle control
FAN2: Flood coolant
```

### 4-Axis with Rotary
```
Add to above:
E0-MOTOR: A-Axis (rotary)
E0-STOP: A home switch (optional)
```

### Plasma Cutter
```
Motors: Same as router
PROBE: Floating head switch
FAN1: Torch fire signal
HEAT0: Arc OK input (through optocoupler)
```

## TMC2209 Driver Settings

The board expects TMC2209 drivers in UART mode:

1. **Install drivers** with heatsinks
2. **Check orientation** - match pin labels
3. **UART jumpers** are built-in (no changes needed)
4. **Current setting** via firmware ($140-$143)

## Safety Considerations

⚠️ **CRITICAL SAFETY** - Read [ESTOP_WIRING.md](ESTOP_WIRING.md) first!

1. **E-Stop** MUST cut motor power via hardware contactor
2. **Limit switches** should be NC (fail-safe)
3. **Spindle safety** - E-stop must kill spindle too
4. **Power isolation** - Use contactors, not just relays
5. **Redundancy** - Hardware + software protection
6. **Testing** - Test E-stop daily!
7. **Ground** the machine frame properly
8. **Fuses** on all high-current circuits

## Wiring Best Practices

1. **Motor wires**: Use shielded cable, ground shield at one end
2. **Limit switches**: Keep away from motor wires
3. **Spindle**: Use separate power wiring from logic
4. **USB**: Use ferrite beads if noise issues
5. **Power**: Star ground configuration

## Quick Test After Wiring

```gcode
; Test each connection
$X              ; Clear alarm
?               ; Check limit switches
G91             ; Relative mode
G1 X10 F500     ; Test X movement
G1 Y10 F500     ; Test Y movement
G1 Z10 F500     ; Test Z movement
M3 S12000       ; Test spindle (50%)
M5              ; Spindle off
M8              ; Flood on
M9              ; Coolant off
```

## Troubleshooting Connections

**Motors not moving**:
- Check enable signals (active LOW)
- Verify driver installation
- Test with multimeter for continuity

**Limits always triggered**:
- Check NO/NC configuration
- Use $5 to invert signals
- Verify wiring to GND

**Spindle not working**:
- Measure PWM output (PA1)
- Check enable signal (PC7)
- Verify VFD programming

**USB not detected**:
- Use data cable (not charge-only)
- Try different ports
- Check for bent pins

---
*Remember: This board was designed for 3D printers, so all labels reflect that origin. This guide translates those labels for CNC use.*