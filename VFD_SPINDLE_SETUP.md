# VFD Spindle Control Setup Guide

Complete guide for connecting Variable Frequency Drive (VFD) spindles to the BTT SKR Mini E3 v3.0.

## Overview

Most VFDs expect **0-10V analog control**, but the STM32G0xx outputs **3.3V PWM**. A converter is typically needed.

## VFD Input Requirements

### Common VFD Analog Input Specs:
- **Voltage Range**: 0-10V DC (most common)
- **Alternative**: 0-5V DC (some models)
- **Current**: Usually <10mA (high impedance)
- **Control**: Linear relationship (0V = 0% speed, 10V = 100% speed)

### Board Output Specs:
- **PA1 Output**: 0-3.3V PWM at ~5kHz
- **Current**: 50mA maximum
- **Resolution**: 10-bit (1024 steps)

## PWM to Voltage Conversion Options

### Option 1: RC Filter (Simple, Inaccurate)

**⚠️ NOT RECOMMENDED for precision work**

```
    PA1 ──[1kΩ]──┬──[10kΩ]─── To VFD AI1
                  │
                 ═╪═ 10µF
                  │
                 GND ────────── To VFD ACM
```

**Problems:**
- Only gives 0-3.3V (33% of VFD range)
- Temperature drift
- Loading effects
- Poor linearity

### Option 2: Op-Amp PWM to Voltage Converter (Better)

```
    PA1 ──[1kΩ]──┬──[10µF]──┬─── Op-Amp Input
                  │          │
                 ═╪═ 1µF    [10kΩ]
                  │          │
                 GND        GND

    Op-Amp Circuit (LM358 or similar):
    ┌─────────────────────────────────────┐
    │     +12V                            │
    │      │                              │
    │   [1kΩ] ← Gain setting              │
    │      │                              │
    │  ┌───┴───┐     [10kΩ]               │
    │  │ LM358 ├─────[────]─── 0-10V Out  │
    │  │   +   │              to VFD      │
    │  │   -   ├─── PWM Input from PA1    │
    │  └───┬───┘                          │
    │      │                              │
    │     GND                             │
    └─────────────────────────────────────┘
```

**Advantages:**
- Full 0-10V output
- Good linearity
- Temperature stable

### Option 3: Commercial PWM to 0-10V Module (RECOMMENDED)

**Available modules:**
- PWM to 0-10V converter boards (~$10-15)
- Isolated versions available
- Plug-and-play solution

**Example specifications:**
```
Input: 3.3V/5V PWM (1kHz-20kHz)
Output: 0-10V analog (linear)
Isolation: 1000V (isolated versions)
Accuracy: ±0.1%
Size: ~30x20mm
```

**Popular Models:**
- "PWM to 0-10V Converter Module"
- "4-20mA/0-10V Signal Generator" 
- "DAC PWM to Analog Converter"

### Option 4: Direct Digital Control (If Supported)

Some modern VFDs accept PWM directly:
- Omron MX2 series
- Schneider ATV series
- Delta VFD-M series

Check your VFD manual for "PWM input" capability.

## Complete VFD Wiring

### Standard 0-10V Setup (Recommended)

```
BTT SKR Mini E3 v3.0    PWM→0-10V Module    VFD
┌─────────────────┐    ┌─────────────────┐  ┌───────────┐
│                 │    │                 │  │           │
│ PA1 (PWM) ──────┼────┼─ PWM IN        │  │           │
│ 3.3V ───────────┼────┼─ VCC           │  │           │
│ GND ────────────┼────┼─ GND           │  │           │
│                 │    │                 │  │           │
│ PC6 (FAN1) ─────┼────┼─────────────────┼──┼─ FOR     │
│ PC7 (HEAT0) ────┼────┼─────────────────┼──┼─ RUN     │
│                 │    │ 0-10V OUT ──────┼──┼─ AI1     │
│                 │    │ GND ────────────┼──┼─ ACM     │
└─────────────────┘    └─────────────────┘  └───────────┘
```

### VFD Parameter Configuration

**Essential VFD Parameters to Set:**

```
P00.01 = 1    ; Control source: Terminal (not keypad)
P00.06 = 1    ; Frequency source: AI1 (analog input 1)
P00.14 = 50   ; Maximum frequency (Hz) - match your spindle
P00.15 = 0    ; Minimum frequency (Hz)
P01.00 = 10.0 ; AI1 maximum voltage (10V = max frequency)
P01.01 = 0.0  ; AI1 minimum voltage (0V = min frequency)
P01.05 = 0    ; AI1 filter time (0.0 sec for fast response)
P02.01 = 10.0 ; Acceleration time (seconds)
P02.02 = 10.0 ; Deceleration time (seconds)
P05.01 = 1    ; Enable run/stop via terminal
```

**Parameters vary by VFD brand - consult your manual!**

## Testing the PWM→Voltage Converter

### Required Tools:
- Multimeter (DC voltage)
- Oscilloscope (optional)

### Test Procedure:

1. **Power On System**
   ```gcode
   ; In gSender or terminal
   M3 S0      ; Spindle on, 0% speed
   ```
   - Measure VFD analog input: Should be ~0V

2. **Test 50% Speed**
   ```gcode
   M3 S12000  ; 50% speed (assuming 24000 max RPM)
   ```
   - Measure VFD analog input: Should be ~5V

3. **Test Maximum Speed**
   ```gcode
   M3 S24000  ; 100% speed
   ```
   - Measure VFD analog input: Should be ~10V

4. **Test PWM Signal (PA1)**
   - With oscilloscope: Should see PWM increasing duty cycle
   - 0% = 0% duty cycle, 100% = 100% duty cycle

## Common VFD Brands & Settings

### Huanyang VFD (Popular Chinese brand)
```
PD001 = 1    ; Control from terminals
PD002 = 1    ; Frequency from AI1  
PD003 = 400  ; Main frequency (400Hz for 24,000 RPM spindle)
PD004 = 400  ; Base frequency
PD005 = 50   ; Max voltage
PD011 = 120  ; Acceleration time
PD012 = 120  ; Deceleration time
PD025 = 1    ; Enable RUN/STOP terminal
```

### Delta VFD-M Series
```
02.00 = 2    ; Frequency source = AVI (0-10V)
02.01 = 1    ; Run command source = Digital input
03.00 = 60   ; Max frequency Hz
07.00 = 10.0 ; AVI max voltage
07.01 = 60   ; AVI max frequency
```

### Omron MX2 Series
```
A001 = 3     ; Frequency reference = Analog input
A002 = 1     ; Run source = Digital input
C001 = 400   ; Maximum frequency
C002 = 6     ; Base frequency
H003 = 1000  ; Analog input maximum (10.00V)
```

## Spindle RPM Calculation

**Formula:** `RPM = (Frequency × 60) / Pole Pairs`

**Common spindle specs:**
- **2-pole spindle**: RPM = Frequency × 30
  - 400Hz → 12,000 RPM
- **4-pole spindle**: RPM = Frequency × 15  
  - 400Hz → 6,000 RPM

**Set grblHAL max RPM accordingly:**
```gcode
$30=24000     ; Max spindle RPM (adjust for your spindle)
```

## Troubleshooting

### PWM Signal Issues:
```gcode
; Test PWM output on PA1 with oscilloscope
M3 S6000      ; Should see ~25% duty cycle
M3 S12000     ; Should see ~50% duty cycle  
M3 S18000     ; Should see ~75% duty cycle
M3 S24000     ; Should see ~100% duty cycle
```

### VFD Not Responding:
1. Check wiring: PWM converter → VFD
2. Verify VFD parameters set correctly
3. Measure voltage at VFD AI1 terminal
4. Check RUN/STOP terminal (PC7)

### Speed Not Linear:
1. Check PWM converter calibration
2. Verify VFD AI1 scaling (P01.00, P01.01)
3. Test with multimeter at different S values

### Spindle Direction Issues:
- PC6 (FAN1) controls direction
- HIGH = Forward, LOW = Reverse
- Some VFDs need separate REV terminal

## Safety Considerations

### VFD E-Stop Integration:
```
E-Stop Button ───┬─── Motor Power Contactor
                 ├─── VFD Emergency Stop Input
                 └─── Board E-Stop Input (PC15)
```

### VFD Fault Monitoring:
- Connect VFD fault output to board input
- Can be monitored via software
- Should also trigger E-stop circuit

## Recommended Shopping List

### For PWM→0-10V Conversion:
1. **PWM to 0-10V converter module** (~$10-15)
   - Search: "PWM to 0-10V converter" or "DAC PWM analog"
2. **Alternative: Op-amp circuit**
   - LM358 or TL072 op-amp
   - Resistors: 1kΩ, 10kΩ  
   - Capacitors: 1µF, 10µF
3. **Cables**
   - Shielded cable for analog signals
   - Separate power and signal wiring

### Testing Equipment:
- Digital multimeter (DC voltage measurement)
- Oscilloscope (optional, for PWM verification)

## Example Complete Setup

**Hardware:**
- BTT SKR Mini E3 v3.0 with grblHAL firmware
- 2.2kW Chinese spindle (24,000 RPM, 400Hz)
- Huanyang VFD
- PWM to 0-10V converter module

**Connections:**
1. PA1 → PWM converter input
2. PWM converter 0-10V out → VFD AI1
3. PC6 → VFD FOR (forward)
4. PC7 → VFD RUN
5. E-stop → VFD emergency stop

**Settings:**
```gcode
$30=24000     ; Max RPM
$31=0         ; Min RPM  
$32=0         ; Laser mode off (use spindle mode)
```

**Result:**
- M3 S12000 → 12,000 RPM
- M3 S24000 → 24,000 RPM  
- M5 → Spindle stop
- Linear speed control via S parameter

---

**Remember: Always test spindle control WITHOUT a tool installed first!**