# CNC Testing & Configuration Tools

This directory contains testing and configuration tools for the BTT SKR Mini E3 v3.0 grblHAL system.

## 🧙‍♂️ CNC Setup Wizard

**Primary testing and configuration tool for new installations**

### Quick Start (Windows):
1. Install Python from [python.org](https://www.python.org/downloads/)
2. Double-click `run_wizard.bat`
3. Follow the guided setup process

### Features:
- **Interactive testing** of motors, limits, probe, spindle
- **Auto-calibration** of movement accuracy
- **Safety-focused** with confirmation prompts
- **Configuration backup** and EEPROM save
- **Detailed reports** of test results

### Files:
- `cnc_setup_wizard.py` - Main wizard application
- `run_wizard.bat` - Windows launcher (recommended)
- `run_wizard.sh` - Linux/macOS launcher
- `requirements.txt` - Python dependencies
- `CNC_WIZARD_README.md` - Detailed documentation

## 📁 Future Tools

This directory is designed to hold additional testing and utility tools:

### Planned Tools:
- **Step Loss Detector** - Monitor for missed steps during operation
- **Vibration Analyzer** - Optimize acceleration settings
- **G-code Validator** - Pre-flight check for G-code files
- **Performance Benchmarks** - Speed/accuracy testing suite
- **Settings Backup/Restore** - Complete configuration management
- **Firmware Updater** - Automated firmware flashing tool

### Tool Categories:
- **Setup & Calibration** - Initial machine configuration
- **Testing & Diagnostics** - Ongoing system verification
- **Performance Tuning** - Optimization utilities
- **Maintenance** - Preventive care tools
- **Troubleshooting** - Problem diagnosis helpers

## 🔧 Usage Guidelines

### For First-Time Users:
1. **Start with Setup Wizard** - Complete all basic tests
2. **Save configuration** - Keep backup of working settings
3. **Test incrementally** - Don't skip safety checks

### For Advanced Users:
- Tools can be run independently
- Integration with existing workflows
- Scriptable for automation

### Safety Requirements:
- **E-stop accessible** during all testing
- **Clear work area** before movement tests
- **Verify connections** before power-on tests
- **Start with low speeds** and work up

## 🐍 Development

### Adding New Tools:
1. Follow Python PEP 8 style guidelines
2. Include safety warnings and confirmations
3. Support both Windows and Linux
4. Document thoroughly in README
5. Add requirements to requirements.txt

### Dependencies:
- **Python 3.7+** (tested on 3.8-3.11)
- **PySerial** for USB/UART communication
- **Standard library** preferred for additional features

---

**Start with the CNC Setup Wizard for comprehensive testing and configuration!**