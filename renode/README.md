# Renode Testing for STM32G0xx grblHAL

This directory contains [Renode](https://renode.io/) emulation files for testing the STM32G0xx grblHAL firmware without physical hardware.

## What is Renode?

Renode is an open-source software development framework that lets you develop, debug and test multi-node device systems reliably, scalably and effectively. It provides hardware emulation including CPU cores, peripherals, sensors, and even network connections.

## Files in this Directory

- `stm32g0b1.repl` - Platform description for STM32G0B1RET6 microcontroller
- `btt_skr_mini_e3_v30.repl` - Board-specific configuration for BTT SKR Mini E3 v3.0
- `test_grblhal.resc` - Test script with helper commands and scenarios
- `run_tests.sh` - Automated test runner script

## Prerequisites

### Renode Installation

This project uses a local Renode installation at `~/e3_mini_v3/renode/`.

**Current Setup:**
- Renode executable: `~/e3_mini_v3/renode/renode`
- The test scripts automatically use this local installation

**Alternative Installation:**
```bash
# Ubuntu/Debian
sudo apt install renode

# macOS
brew install renode

# Portable Package
# Download from https://github.com/renode/renode/releases
```

## Running Tests

### Quick Start

```bash
# Make sure you're in the STM32G0xx directory
cd STM32G0xx/renode

# Run the test script
./run_tests.sh
```

### Manual Testing

1. Start Renode:
```bash
~/e3_mini_v3/renode/renode
```

2. In the Renode console:
```
(monitor) include @test_grblhal.resc
(monitor) reset
(monitor) start
```

### Available Test Commands

Once emulation is running, you can use these commands:

- `test_basic_communication` - Test grbl status queries
- `test_motion` - Send motion G-code commands
- `test_safety_systems` - Test E-stop and limit switches
- `trigger_limit_x` - Simulate X limit switch trigger
- `trigger_estop` - Simulate emergency stop button
- `send_gcode "G1 X10"` - Send any G-code command

### Monitoring Output

The test script sets up monitors for:
- **UART1** - Debug output
- **USB** - Main grbl communication
- **GPIO** - Step pulses, limit switches, control inputs

## Example Test Session

```
(monitor) include @test_grblhal.resc
(monitor) reset
(monitor) start
(monitor) test_basic_communication
UART1: grblHAL 1.1f ['$' or '$HELP' for help]
USB: ?
USB: <Idle|MPos:0.000,0.000,0.000|Bf:15,127|FS:0,0|WCO:0.000,0.000,0.000>
(monitor) send_gcode "G91 G1 X10 F100"
STEP: GPIO Port B pin 13 = 1
STEP: GPIO Port B pin 13 = 0
...
```

## Debugging

### View CPU State
```
(monitor) cpu PC       # Show program counter
(monitor) cpu Regs     # Show all registers
(monitor) sysbus ReadByte 0x20000000  # Read memory
```

### Set Breakpoints
```
(monitor) cpu AddHook 0x08008000 "echo 'Hit reset vector'"
(monitor) cpu Step     # Single step execution
```

### Monitor Peripherals
```
(monitor) timer3       # Show timer state
(monitor) uart1        # Show UART state
```

## Limitations

1. **TMC2209 Drivers** - Not fully emulated (UART communication only)
2. **USB CDC** - Basic emulation, may not handle all USB states
3. **Timing** - Emulation timing may differ from real hardware
4. **Analog Peripherals** - ADC, DAC not fully implemented

## Benefits

✅ **No Hardware Required** - Test firmware changes instantly
✅ **Automated Testing** - Script complex test scenarios
✅ **Debugging** - Full visibility into CPU and peripheral state
✅ **Regression Testing** - Ensure changes don't break functionality
✅ **Safe Testing** - No risk of damaging hardware

## Contributing

To improve the Renode emulation:

1. Add more test scenarios to `test_grblhal.resc`
2. Enhance peripheral models in platform files
3. Create automated test suites
4. Document any missing peripheral implementations

## Resources

- [Renode Documentation](https://renode.readthedocs.io/)
- [Renode GitHub](https://github.com/renode/renode)
- [STM32G0 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0444-stm32g0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)