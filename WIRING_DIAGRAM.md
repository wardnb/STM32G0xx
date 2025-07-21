# BTT SKR Mini E3 v3.0 CNC Wiring Diagram

Visual wiring reference for common CNC configurations.

## Basic 3-Axis CNC Router Wiring

```
                          BTT SKR Mini E3 v3.0
    ┌─────────────────────────────────────────────────────────┐
    │                                                         │
    │  [POWER IN]  ←── 12-24V DC Power Supply                │
    │   ├─ V+                                                │
    │   └─ V-                                                │
    │                                                         │
    │  [X-MOTOR]   ←── X-Axis Stepper Motor                 │
    │   ├─ 2B  (Red)                                         │
    │   ├─ 2A  (Blue)                                        │
    │   ├─ 1A  (Black)                                       │
    │   └─ 1B  (Green)                                       │
    │                                                         │
    │  [Y-MOTOR]   ←── Y-Axis Stepper Motor                 │
    │   └─ (Same color pattern)                              │
    │                                                         │
    │  [Z-MOTOR]   ←── Z-Axis Stepper Motor                 │
    │   └─ (Same color pattern)                              │
    │                                                         │
    │  [X-STOP]    ←── X Limit Switch                       │
    │   ├─ S  ←── COM (switch common)                       │
    │   ├─ -  ←── NC (normally closed)                      │
    │   └─ +  (not used)                                    │
    │                                                         │
    │  [Y-STOP]    ←── Y Limit Switch                       │
    │   └─ (Same as X)                                      │
    │                                                         │
    │  [Z-STOP]    ←── Z Limit Switch                       │
    │   └─ (Same as X)                                      │
    │                                                         │
    │  [PROBE]     ←── Tool Length Probe                    │
    │   ├─ S  ←── Probe Plate                              │
    │   ├─ -  ←── Ground Clip (to tool)                    │
    │   └─ +  (not used)                                    │
    │                                                         │
    │  [USB]       ←── To Computer (Micro USB)              │
    │                                                         │
    └─────────────────────────────────────────────────────────┘
```

## Spindle Control Wiring

### Option 1: VFD Control (Recommended)
```
    BTT SKR Mini E3 v3.0              VFD/Inverter
    ┌──────────────────┐              ┌─────────────┐
    │                  │              │             │
    │  FAN1 (PC6) ────┼──────────────┼─→ FOR/REV   │
    │                  │              │             │
    │  HEAT0 (PC7) ───┼──────────────┼─→ RUN/STOP  │
    │                  │              │             │
    │  PA1 (PWM) ─────┼──────────────┼─→ AI1 (0-10V)│
    │       ↓          │              │      ↑      │
    │  [PWM→Analog]   │              │  [Voltage]  │
    │   Converter     │              │   Divider   │
    │                  │              │             │
    │  GND ───────────┼──────────────┼─→ ACM       │
    │                  │              │             │
    └──────────────────┘              └─────────────┘

    PWM to 0-10V Converter Circuit:
    PA1 ──[1kΩ]──┬──[10kΩ]──┬── To VFD AI1
                  │          │
                 ═╪═ 10µF   ═╪═ 100nF
                  │          │
                 GND        GND
```

### Option 2: Relay Control (Simple On/Off)
```
    BTT SKR Mini E3 v3.0              Relay Module
    ┌──────────────────┐              ┌─────────────┐
    │                  │              │             │
    │  HEAT0 (PC7) ───┼──────────────┼─→ IN1       │
    │                  │              │             │
    │  3.3V ──────────┼──────────────┼─→ VCC       │
    │                  │              │             │
    │  GND ───────────┼──────────────┼─→ GND       │
    │                  │              │             │
    └──────────────────┘              │   NO ←──┐   │
                                      │   COM ←──┼───┼── AC Mains L
                                      │   NC     │   │
                                      └──────────┼───┘
                                                 │
                                            To Spindle
```

## Coolant Control Wiring

```
    BTT SKR Mini E3 v3.0              Relay Module
    ┌──────────────────┐              ┌─────────────┐
    │                  │              │             │
    │  FAN2 (PC8) ────┼──────────────┼─→ IN1       │ ← M8 Flood
    │                  │              │             │
    │  HEAT1 (PC9) ───┼──────────────┼─→ IN2       │ ← M7 Mist
    │                  │              │             │
    │  3.3V ──────────┼──────────────┼─→ VCC       │
    │                  │              │             │
    │  GND ───────────┼──────────────┼─→ GND       │
    │                  │              └─────────────┘
    └──────────────────┘                    │ │
                                           NO NO
                                            │ │
                                    To Coolant Pumps
```

## Control Panel Wiring (E-Stop, Feed Hold, Start)

```
    BTT SKR Mini E3 v3.0              Control Panel
    ┌──────────────────┐              ┌─────────────┐
    │                  │              │             │
    │  EXP1 Connector: │              │ [EMERGENCY] │
    │  ┌─────────────┐ │              │  ╔═════╗    │
    │  │ 1 2 3 4 5   │ │              │  ║STOP ║←───┼── To Motor Power
    │  │ 6 7 8 9 10  │ │              │  ╚═════╝    │   Contactor
    │  └─────────────┘ │              │             │
    │    │ │ │         │              │ [FEED HOLD] │
    │    │ │ └─ PC13 ──┼──────────────┼──   ○   ────┤
    │    │ │           │              │      │      │
    │    │ └─── PC12 ──┼──────────────┼── ○ │ ○ ───┤ [CYCLE START]
    │    │             │              │    └─┼─┘    │
    │    └───── GND ───┼──────────────┼──────┴──────┤
    │                  │              │             │
    └──────────────────┘              └─────────────┘
```

## TMC2209 Driver Installation

```
    IMPORTANT: Check orientation before inserting!
    
    ┌─────────────┐
    │ TMC2209     │
    │ ┌─────────┐ │
    │ │ CHIP    │ │← Heat sink on top
    │ └─────────┘ │
    │ ○ ○ ○ ○ ○ ○ │← EN MS1 MS2 PDN STEP DIR
    └─────────────┘
    │ ○ ○ ○ ○ ○ ○ │← VM GND SENS REF UART GND
    └─────────────┘
      ↑
      Match VM pin to board VM marking
```

## Complete System Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────┐
│   24V POWER     │────→│ BTT SKR MINI E3  │←───→│  COMPUTER   │
│   SUPPLY        │     │      v3.0        │     │  (gSender)  │
└─────────────────┘     └────┬─────┬───────┘     └─────────────┘
                             │     │                    USB
                             │     │
                    ┌────────┴─┐ ┌─┴────────┐
                    │ STEPPERS │ │ SPINDLE  │
                    │ X,Y,Z,A  │ │   VFD    │
                    └──────────┘ └──────────┘
                             │
                    ┌────────┴────────┐
                    │ LIMIT SWITCHES  │
                    │ PROBE │ E-STOP │
                    └─────────────────┘
```

## Wire Gauge Recommendations

| Connection | Current | Wire Gauge | Notes |
|------------|---------|------------|-------|
| Main Power | 10A | 16 AWG | 12-24V DC |
| Stepper Motors | 2A | 20 AWG | 4-conductor shielded |
| Limit Switches | 0.01A | 24 AWG | Can be small |
| Spindle Control | 0.01A | 22 AWG | Signal only |
| USB Cable | 0.5A | - | Use quality cable |

## Grounding Best Practices

```
                    Star Ground Point
                           ★
                     ┌─────┼─────┐
                     │     │     │
                 ┌───┴─┐ ┌─┴─┐ ┌─┴───┐
                 │Frame│ │PSU│ │Board│
                 │ GND │ │GND│ │ GND │
                 └─────┘ └───┘ └─────┘
                 
- Single ground point (star configuration)
- Frame connected to earth ground
- Shield drain wires to frame ground only
```

## Color Code Reference

### Stepper Motor Wires (Common)
- **Black/Green**: Coil A (1A/1B)
- **Red/Blue**: Coil B (2A/2B)
- Verify with multimeter (coil = ~2-5Ω)

### Limit Switch Wires
- **Red**: +V (usually not used)
- **Black**: Ground/Common
- **White/Green**: Signal

### Power Wires
- **Red**: Positive (+V)
- **Black**: Negative/Ground
- **Green/Yellow**: Earth Ground

---
*Note: Always verify connections with multimeter before powering on!*