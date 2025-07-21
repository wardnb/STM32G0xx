# Emergency Stop (E-Stop) Wiring Guide

Proper E-stop wiring is **CRITICAL** for CNC safety. This guide covers both software and hardware E-stop implementations.

## ⚠️ SAFETY WARNING

**An E-stop must cut power to motors IMMEDIATELY and INDEPENDENTLY of the control board.**

**Software-only E-stops are NOT sufficient for safety!**

## Two-Level E-Stop System (RECOMMENDED)

### Level 1: Hardware Power Cut (Primary Safety)
**ALWAYS implement this - it's your life insurance**

```
    24V Power Supply    Motor Power Contactor         Motors
    ┌─────────────┐     ┌─────────────────────┐      ┌─────────┐
    │      +24V ──┼─────┼── A1        NO ──┬──┼──────┼── X,Y,Z │
    │             │     │                  │  │      │ Motors  │
    │       GND ──┼──┬──┼── A2        COM ─┴──┼──────┼──       │
    └─────────────┘  │  └─────┬───────────────┘      └─────────┘
                     │        │
    E-Stop Button    │    ┌───┴────┐
    ┌─────────────┐  │    │ E-Stop │
    │     NC ─────┼──┼────┼── NC   │
    │    COM ─────┼──┴────┼── COM  │
    └─────────────┘       └────────┘
                          (Board input)
```

**How it works:**
1. **E-Stop pressed** → Contactor opens → **Motors lose power instantly**
2. **Board detects** E-stop → Software alarm state
3. **Both systems** work independently

### Level 2: Software Detection (Secondary)
**Connect to board for software awareness**

| Board Connection | Pin | Function |
|------------------|-----|----------|
| **EXP1 or spare pin** | PC15 | E-stop input |
| **Ground** | GND | Common |

## Hardware Implementation Options

### Option 1: Single Contactor (Basic)
```
    24V PSU          Contactor            Motors
    ┌────────┐      ┌──────────┐         ┌───────┐
    │ +24V ──┼──────┼─ A1  NO ─┼─────────┼─ +24V │
    │        │      │          │         │       │
    │ GND ───┼──┬───┼─ A2 COM ─┼─────────┼─ GND  │
    └────────┘  │   └──────────┘         └───────┘
                │
           ┌────┴────┐
           │ E-STOP  │ ← NC button
           │  (NC)   │
           └─────────┘
```

### Option 2: Dual Contactor (Industrial)
```
    Power Supply     Main Contactor    Control Contactor   Motors
    ┌──────────┐     ┌─────────────┐   ┌─────────────┐    ┌──────┐
    │   +24V ──┼─────┼─A1   NO1──┬─┼───┼─A1   NO────┼────┼─+24V │
    │          │     │           │ │   │             │    │      │
    │   GND ───┼──┬──┼─A2  COM1──┴─┼───┼─A2  COM────┼────┼─GND  │
    └──────────┘  │  └─────────────┘   └─────┬───────┘    └──────┘
                  │                          │
                  │  ┌─────────────────────┬─┘
                  │  │                     │
             ┌────┴──┴─┐              ┌────┴────┐
             │ E-STOP  │              │ E-STOP  │
             │  (NC)   │              │  (NC)   │
             └─────────┘              └─────────┘
```

## E-Stop Button Types

### Twist-to-Release (Recommended)
```
    ┌─────────────────────┐
    │  ╔═════════════╗    │
    │  ║   E-STOP    ║    │ ← Red mushroom button
    │  ║    PULL     ║    │
    │  ╚═════════════╝    │
    │                     │
    │  NC ○ ○ NO         │ ← Use NC contacts
    │  COM ○              │
    └─────────────────────┘
```

### Key-Reset Type
```
    Similar to twist-to-release but requires key to reset
    More secure for industrial environments
```

## Contactor Selection

### 24V DC Contactor Specifications:
- **Coil Voltage**: 24V DC
- **Contact Rating**: Motor current + 25%
- **Contact Type**: NO (Normally Open)
- **Auxiliary Contacts**: 1 NC for board feedback

### Example Part Numbers:
- **Schneider**: LC1D09BD (9A)
- **ABB**: B7-30-10 (7A)
- **Allen-Bradley**: 100-C09D*10 (9A)

## Wiring Steps

### Step 1: Power Contactor
```
1. Mount contactor in electrical enclosure
2. Wire 24V+ to contactor coil (A1)
3. Wire 24V- through E-stop NC to coil (A2)
4. Wire motor power through contactor NO contacts
5. Test: E-stop should cut motor power
```

### Step 2: Board Connection
```
1. Find auxiliary NC contact on contactor
2. Wire one side to board input (PC15)
3. Wire other side to GND
4. Configure in firmware with $-command
```

### Step 3: Testing
```
1. Power on system normally
2. Verify motors work
3. Press E-stop
4. Verify:
   - Motors stop immediately (hardware)
   - Board shows alarm (software)
   - Cannot move until E-stop released and reset
```

## Software Configuration

### grblHAL Settings:
```gcode
$5=0        ; Control signals not inverted (default)
$21=1       ; Hard limits enable (if using limits)
```

### Test Commands:
```gcode
?           ; Check status - should show "Alarm" if E-stop active
$X          ; Clear alarm (only works if E-stop released)
```

## Multiple E-Stop Locations

For larger machines, wire multiple E-stop buttons in series:

```
    24V+ ──┬── E-Stop 1 ──┬── E-Stop 2 ──┬── E-Stop 3 ──┬── Contactor A1
           │      NC      │      NC      │      NC      │
           │              │              │              │
    GND ───┴──────────────┴──────────────┴──────────────┴── Contactor A2
```

**All E-stops must be NC (Normally Closed) and in series.**

## Spindle E-Stop Integration

### For VFD Spindles:
```
    E-Stop Button ─── VFD "Emergency Stop" Input
                 └─── Motor Power Contactor
                 └─── Board Input
```

### For AC Spindles:
```
    E-Stop ─── Spindle Contactor ─── AC Power ─── Spindle
           └─── Motor Contactor ─── 24V ─── Motors
           └─── Board Input
```

## Common Mistakes to Avoid

❌ **Software-only E-stop**
- Board failure = no emergency stop

❌ **Using NO contacts**
- Wire break = no emergency stop

❌ **Forgetting spindle**
- Motors stop but spindle keeps running

❌ **No visual indication**
- Can't tell if E-stop is active

✅ **Correct Implementation:**
- Hardware power cut
- NC contacts in series
- All motion stops (including spindle)
- Visual/audible indication
- Board awareness

## Testing Procedure

### Daily Test:
```
1. Power on system
2. Press E-stop - all motion must stop
3. Try to jog - should get alarm
4. Release E-stop and reset
5. System should resume normal operation
```

### Installation Test:
```
1. Disconnect one motor wire
2. Press E-stop
3. Measure motor terminals - should be 0V
4. Release E-stop
5. Should have 24V back
6. Reconnect wire
```

## Troubleshooting

**E-stop doesn't stop motors:**
- Check contactor wiring
- Verify NC contacts used
- Test contactor operation

**Board doesn't detect E-stop:**
- Check board input connection
- Verify pin configuration
- Test with multimeter

**Can't clear alarm:**
- E-stop still pressed/stuck
- Wiring issue to board
- Need to power cycle

## Code Integration

The firmware monitors the E-stop input and will:
1. Immediately halt all motion
2. Set alarm state
3. Require reset ($X) after E-stop release
4. Prevent any motion commands while active

## Regulatory Compliance

For commercial/industrial use:
- **Category 0** stop (immediate power removal)
- **EN ISO 13850** compliant E-stop button
- **Redundant** contactors for critical applications
- **Documentation** of safety system

---

**Remember: The best E-stop system is one you never have to use, but when you do, it works instantly and reliably!**

⚠️ **Test your E-stop regularly - your safety depends on it.**