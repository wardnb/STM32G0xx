# CNC Setup Wizard

Interactive testing and tuning tool for BTT SKR Mini E3 v3.0 with grblHAL firmware.

## Overview

The CNC Setup Wizard is a Python application that guides you through systematically testing and configuring your CNC machine. It's specifically designed for the BTT SKR Mini E3 v3.0 board running grblHAL firmware.

## Features

### ✅ What the Wizard Does:
- **Automated Connection** - Scans and connects to your CNC controller
- **Communication Test** - Verifies grblHAL is responding properly
- **Motor Direction Test** - Checks each axis moves in the correct direction
- **Limit Switch Test** - Verifies all limit switches are working
- **Probe Testing** - Tests tool length probe functionality
- **Spindle Control** - Tests VFD, PWM, or relay spindle control
- **Homing Cycles** - Guides through proper homing setup
- **Movement Accuracy** - Calibrates steps/mm for precise movement
- **Configuration Backup** - Saves all settings and test results

### 🎯 Target Users:
- First-time CNC builders using BTT SKR Mini E3 v3.0
- Users upgrading from other controllers (Arduino, parallel port)
- Anyone wanting systematic testing of their grblHAL setup
- Troubleshooting existing installations

## Requirements

### Hardware:
- **BTT SKR Mini E3 v3.0** with grblHAL firmware installed
- **Windows 10/11** computer with USB port
- **Micro USB cable** (data-capable, not charge-only)
- **CNC machine** (any 3-axis router, mill, or similar)

### Software:
- **Python 3.7+** (download from [python.org](https://www.python.org/downloads/))
- **PySerial library** (automatically installed)

## Installation & Usage

### Windows (Recommended Method):

1. **Download Files**:
   - Download the entire repository or just these files:
     - `cnc_setup_wizard.py`
     - `requirements.txt`  
     - `run_wizard.bat`

2. **Install Python**:
   - Download from [python.org](https://www.python.org/downloads/)
   - ✅ Check "Add Python to PATH" during installation
   - ✅ Check "pip" installation option

3. **Run the Wizard**:
   - Double-click `run_wizard.bat`
   - OR open Command Prompt in the folder and run:
     ```cmd
     run_wizard.bat
     ```

### Manual Installation (Windows/Linux):

1. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Run Wizard**:
   ```bash
   # Windows
   python cnc_setup_wizard.py
   
   # Linux
   python3 cnc_setup_wizard.py
   # OR
   ./run_wizard.sh
   ```

## Usage Guide

### Step 1: Connection
The wizard will:
- Scan for available COM ports
- Show device descriptions (look for "STM32" or "Virtual COM Port")
- Auto-connect if only one port found
- Test basic communication

### Step 2: Testing Menu
Choose from these test modules:

#### 🔌 Basic Communication Test
- Verifies grblHAL responds to commands
- Shows controller version and current settings
- Clears any alarms
- **Safe to run anytime**

#### 🎮 Motor Direction Test
- Moves each axis 10mm positive direction
- Verifies X moves RIGHT, Y moves BACK, Z moves UP
- Automatically calculates direction invert settings ($3)
- **SAFETY**: Ensure spindle is clear of work surface

#### 🔒 Limit Switch Test
- Tests each limit switch individually
- Shows real-time pin status
- Temporarily disables hard limits for safety
- Helps identify wiring issues

#### 📐 Probe Test
- Tests tool length probe circuit
- Verifies electrical connection
- Tests G38.3 probe command
- **Requires**: Probe plate and alligator clip setup

#### 🌀 Spindle Control Test
- Supports VFD (0-10V), PWM, and relay control
- Tests at multiple speed levels
- Verifies direction control
- **SAFETY**: Clear spindle area before testing

#### 🏠 Homing Cycle Test
- Tests automatic homing ($H command)
- Verifies limit switches work during homing
- Sets machine zero coordinates
- **Requires**: Working limit switches

#### 📏 Movement Accuracy Test
- Tests actual distance moved vs commanded
- Calculates calibration for steps/mm settings
- Requires ruler or calipers for measurement
- Updates $100, $101, $102 automatically

### Step 3: Configuration Save
- Generates detailed test report
- Saves current grblHAL settings
- Creates timestamp-named file
- Option to save to controller EEPROM

## Safety Features

### ⚠️ Built-in Safety:
- **Clear warnings** before any movement
- **Confirmation prompts** for all tests
- **Skip options** for each test module
- **Emergency stop instructions** displayed
- **Motor current limits** checked
- **Soft limits disabled** during testing

### 🛑 Safety Requirements:
- **E-stop accessible** at all times
- **Spindle area clear** during tests
- **Manual control** of all test steps
- **Low speed movements** (500 mm/min max)

## Example Test Session

```
==========================================================
           BTT SKR Mini E3 v3.0 CNC Setup Wizard         
==========================================================

Available serial ports:
1. COM3 - STM32 Virtual COM Port

Auto-selecting: COM3
✓ Connected successfully!

Controller info:
  [VER:1.1h.20210901]
  [OPT:V,15,128]

==========================================================
                     Setup Menu                        
==========================================================

1. Basic Communication Test
2. Motor Direction Test  
3. Limit Switch Test
4. Probe Test
5. Spindle Control Test
6. Homing Cycle Test
7. Movement Accuracy Test
8. Save Configuration
9. Exit

Enter choice: 2

==========================================================
                Motor Direction Test                    
==========================================================

⚠ SAFETY: Ensure spindle/tool is clear of work surface!
ℹ Motors will move 10mm in each direction

Ready to test motors? (y/n): y

Testing X axis
Expected direction: RIGHT when facing machine
Did X move RIGHT when facing machine? (y/n): n
⚠ X axis direction inverted

Testing Y axis  
Expected direction: AWAY from you (toward back)
Did Y move AWAY from you (toward back)? (y/n): y
✓ Y axis direction correct

Testing Z axis
Expected direction: UP (away from table)
Did Z move UP (away from table)? (y/n): y
✓ Z axis direction correct

Direction fixes needed:
Current $3=0
Set $3=1 to fix directions

Apply direction fix now? (y/n): y
✓ Direction settings updated
```

## Troubleshooting

### Connection Issues:

**"No serial ports found"**:
- Check USB cable (must be data-capable)
- Install STM32 Virtual COM Port drivers
- Try different USB ports
- Verify grblHAL firmware is installed

**"Connection failed"**:
- Close gSender/UGS if running
- Check Windows Device Manager for port conflicts
- Try lower baud rate (115200 is standard)
- Power cycle the controller

### Movement Issues:

**Motors not moving**:
- Check TMC2209 driver installation
- Verify motor current settings ($140-$142)
- Test motor continuity with multimeter
- Check enable signal polarity ($4)

**Wrong directions**:
- Wizard automatically calculates $3 setting
- May need to physically swap motor wires
- Check mechanical coupling direction

### Limit Switch Issues:

**Switches not detected**:
- Verify wiring (signal to pin, ground to GND)
- Check switch type (NO vs NC)
- Use $5 setting to invert if needed
- Test switches with multimeter

## Output Files

The wizard creates these files:

### Configuration Report (`cnc_config_YYYYMMDD_HHMMSS.txt`):
```
CNC Configuration Report
Generated: 2024-01-21 14:30:25
==================================================

Test Results:
  X_direction: correct
  Y_direction: inverted  
  Z_direction: correct
  X_limit: working
  Y_limit: working
  Z_limit: not_working
  probe: working
  spindle: tested

Current Settings:
  $0=10
  $1=25
  $2=0
  $3=2
  ...
  
Recommended Actions:
  - Check Z limit switch wiring
```

## Compatibility

### ✅ Works With:
- **Controllers**: BTT SKR Mini E3 v3.0 with grblHAL
- **Operating Systems**: Windows 10/11, Linux, macOS
- **CNC Software**: gSender, Universal G-Code Sender, CNCjs
- **Python**: 3.7+ (tested on 3.8, 3.9, 3.10, 3.11)

### ❌ Not Compatible With:
- **Marlin firmware** (3D printer firmware)
- **Stock grbl** (Arduino-based controllers)
- **Other STM32 boards** without grblHAL
- **Serial over Bluetooth** (timing issues)

## Advanced Usage

### Command Line Options:
```bash
# Specify port directly
python cnc_setup_wizard.py --port COM3

# Set custom baud rate  
python cnc_setup_wizard.py --baud 250000

# Auto-run specific tests
python cnc_setup_wizard.py --auto-test motors,limits
```

### Configuration Integration:
The wizard reads and updates these grblHAL settings:

| Setting | Function | Wizard Test |
|---------|----------|-------------|
| $3 | Direction invert mask | Motor Direction |
| $5 | Limit invert mask | Limit Switch |
| $21 | Hard limits enable | Limit Switch |
| $22 | Homing cycle enable | Homing |
| $30 | Max spindle RPM | Spindle |
| $100-$102 | Steps per mm | Movement Accuracy |

## Support

### 🐛 Bug Reports:
- GitHub Issues: [Report problems here]
- Include: Error messages, test results file, controller info

### 💡 Feature Requests:
- Additional test modules
- Support for 4-axis machines
- Automated calibration routines

### 📚 Resources:
- [grblHAL Wiki](https://github.com/grblHAL/core/wiki)
- [BTT SKR Mini E3 Manual](CONNECTION_GUIDE.md)
- [VFD Setup Guide](VFD_SPINDLE_SETUP.md)

---

**Happy machining with your BTT SKR Mini E3 v3.0 CNC!** 🔧