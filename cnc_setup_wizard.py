#!/usr/bin/env python3
"""
CNC Setup Wizard for BTT SKR Mini E3 v3.0 with grblHAL
Interactive testing and tuning guide
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
from datetime import datetime
import threading
import queue
import re

class Colors:
    """Terminal colors for better readability"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'

class GrblSerial:
    """Handle serial communication with grblHAL"""
    def __init__(self):
        self.ser = None
        self.response_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.reader_thread = None
        self.running = False
        
    def list_ports(self):
        """List available serial ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append((port.device, port.description))
        return ports
    
    def connect(self, port, baudrate=115200):
        """Connect to grblHAL"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for grblHAL to initialize
            self.running = True
            self.reader_thread = threading.Thread(target=self._read_responses)
            self.reader_thread.daemon = True
            self.reader_thread.start()
            
            # Clear any startup messages
            self.ser.write(b'\r\n')
            time.sleep(0.5)
            self.clear_queues()
            
            return True
        except Exception as e:
            print(f"{Colors.RED}Connection failed: {e}{Colors.END}")
            return False
    
    def disconnect(self):
        """Disconnect from grblHAL"""
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=1)
        if self.ser:
            self.ser.close()
    
    def _read_responses(self):
        """Background thread to read responses"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        if line.startswith('<') and line.endswith('>'):
                            self.status_queue.put(line)
                        else:
                            self.response_queue.put(line)
            except:
                pass
            time.sleep(0.01)
    
    def send_command(self, cmd, wait_ok=True, timeout=5):
        """Send command and optionally wait for response"""
        if not self.ser:
            return None
            
        # Clear response queue
        while not self.response_queue.empty():
            self.response_queue.get()
            
        self.ser.write((cmd + '\n').encode())
        
        if not wait_ok:
            return True
            
        # Wait for response
        start_time = time.time()
        responses = []
        
        while time.time() - start_time < timeout:
            try:
                response = self.response_queue.get(timeout=0.1)
                responses.append(response)
                if response == 'ok' or response.startswith('error:'):
                    break
            except queue.Empty:
                continue
                
        return responses
    
    def get_status(self):
        """Get current machine status"""
        self.send_command('?', wait_ok=False)
        try:
            status = self.status_queue.get(timeout=1)
            return self._parse_status(status)
        except queue.Empty:
            return None
    
    def _parse_status(self, status_str):
        """Parse status response"""
        # Example: <Idle|MPos:0.000,0.000,0.000|FS:0,0|WCO:0.000,0.000,0.000>
        status = {'raw': status_str}
        
        # Extract state
        match = re.search(r'<(\w+)\|', status_str)
        if match:
            status['state'] = match.group(1)
            
        # Extract machine position
        match = re.search(r'MPos:([-\d.]+),([-\d.]+),([-\d.]+)', status_str)
        if match:
            status['mpos'] = {
                'x': float(match.group(1)),
                'y': float(match.group(2)),
                'z': float(match.group(3))
            }
            
        # Extract work position
        match = re.search(r'WPos:([-\d.]+),([-\d.]+),([-\d.]+)', status_str)
        if match:
            status['wpos'] = {
                'x': float(match.group(1)),
                'y': float(match.group(2)),
                'z': float(match.group(3))
            }
            
        # Extract limit pins
        match = re.search(r'Pn:([XYZ]+)', status_str)
        if match:
            status['pins'] = match.group(1)
            
        return status
    
    def clear_queues(self):
        """Clear all queues"""
        while not self.response_queue.empty():
            self.response_queue.get()
        while not self.status_queue.empty():
            self.status_queue.get()

class CNCSetupWizard:
    """Main setup wizard application"""
    def __init__(self):
        self.grbl = GrblSerial()
        self.config = {}
        self.test_results = {}
        
    def clear_screen(self):
        """Clear terminal screen"""
        os.system('cls' if os.name == 'nt' else 'clear')
        
    def print_header(self, title):
        """Print section header"""
        self.clear_screen()
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.HEADER}{Colors.BOLD}{title.center(60)}{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        print()
        
    def print_info(self, text):
        """Print info message"""
        print(f"{Colors.BLUE}ℹ {text}{Colors.END}")
        
    def print_success(self, text):
        """Print success message"""
        print(f"{Colors.GREEN}✓ {text}{Colors.END}")
        
    def print_warning(self, text):
        """Print warning message"""
        print(f"{Colors.YELLOW}⚠ {text}{Colors.END}")
        
    def print_error(self, text):
        """Print error message"""
        print(f"{Colors.RED}✗ {text}{Colors.END}")
        
    def wait_enter(self):
        """Wait for user to press enter"""
        input(f"\n{Colors.YELLOW}Press Enter to continue...{Colors.END}")
        
    def yes_no(self, prompt):
        """Get yes/no response"""
        while True:
            response = input(f"{prompt} (y/n): ").lower()
            if response in ['y', 'yes']:
                return True
            elif response in ['n', 'no']:
                return False
                
    def get_choice(self, options):
        """Get menu choice"""
        for i, option in enumerate(options, 1):
            print(f"{i}. {option}")
        
        while True:
            try:
                choice = int(input("\nEnter choice: "))
                if 1 <= choice <= len(options):
                    return choice - 1
            except ValueError:
                pass
            print(f"{Colors.RED}Invalid choice. Try again.{Colors.END}")
    
    def connect_to_cnc(self):
        """Connection wizard"""
        self.print_header("CNC Connection Setup")
        
        ports = self.grbl.list_ports()
        if not ports:
            self.print_error("No serial ports found!")
            self.print_info("Please connect your BTT SKR Mini E3 v3.0 via USB")
            self.wait_enter()
            return False
            
        print("Available serial ports:")
        for i, (port, desc) in enumerate(ports, 1):
            print(f"{i}. {port} - {desc}")
            
        if len(ports) == 1:
            port_idx = 0
            print(f"\nAuto-selecting only available port: {ports[0][0]}")
        else:
            port_idx = self.get_choice([f"{p[0]} - {p[1]}" for p in ports])
            
        port = ports[port_idx][0]
        
        print(f"\nConnecting to {port}...")
        if self.grbl.connect(port):
            self.print_success("Connected successfully!")
            
            # Test connection
            response = self.grbl.send_command("$I")
            if response:
                print("\nController info:")
                for line in response:
                    if line != 'ok':
                        print(f"  {line}")
                        
            return True
        else:
            return False
    
    def basic_communication_test(self):
        """Test basic communication"""
        self.print_header("Basic Communication Test")
        
        tests = [
            ("Status request (?)", "?", False),
            ("Version info ($I)", "$I", True),
            ("View settings ($$)", "$$", True),
            ("Clear alarms ($X)", "$X", True),
        ]
        
        for test_name, cmd, wait in tests:
            print(f"\nTesting: {test_name}")
            
            if not wait:
                self.grbl.send_command(cmd, wait_ok=False)
                time.sleep(0.5)
                status = self.grbl.get_status()
                if status:
                    self.print_success(f"Status: {status.get('state', 'Unknown')}")
                    if 'mpos' in status:
                        print(f"  Position: X={status['mpos']['x']:.3f} Y={status['mpos']['y']:.3f} Z={status['mpos']['z']:.3f}")
                else:
                    self.print_error("No status received")
            else:
                response = self.grbl.send_command(cmd)
                if response:
                    self.print_success("Command successful")
                    if cmd == "$$":
                        # Store settings
                        for line in response:
                            if line.startswith('$'):
                                self.parse_setting(line)
                else:
                    self.print_error("No response received")
                    
        self.wait_enter()
        
    def parse_setting(self, line):
        """Parse grbl setting line"""
        match = re.match(r'\$(\d+)=([\d.]+)', line)
        if match:
            setting_num = int(match.group(1))
            value = float(match.group(2))
            self.config[setting_num] = value
            
    def motor_direction_test(self):
        """Test motor directions"""
        self.print_header("Motor Direction Test")
        
        self.print_warning("SAFETY: Ensure spindle/tool is clear of work surface!")
        self.print_info("Motors will move 10mm in each direction")
        
        if not self.yes_no("\nReady to test motors?"):
            return
            
        # Set to relative mode
        self.grbl.send_command("G91")
        
        axes = [
            ('X', 'RIGHT when facing machine'),
            ('Y', 'AWAY from you (toward back)'),
            ('Z', 'UP (away from table)')
        ]
        
        for axis, expected_dir in axes:
            print(f"\n{Colors.BOLD}Testing {axis} axis{Colors.END}")
            print(f"Expected direction: {expected_dir}")
            
            # Get initial position
            initial_status = self.grbl.get_status()
            
            # Move positive
            self.grbl.send_command(f"G1 {axis}10 F500")
            time.sleep(2)
            
            # Check if it moved correctly
            if self.yes_no(f"Did {axis} move {expected_dir}?"):
                self.print_success(f"{axis} axis direction correct")
                self.test_results[f'{axis}_direction'] = 'correct'
            else:
                self.print_warning(f"{axis} axis direction inverted")
                self.test_results[f'{axis}_direction'] = 'inverted'
                print(f"You'll need to invert this axis with $3 setting")
                
            # Return to start
            self.grbl.send_command(f"G1 {axis}-10 F500")
            time.sleep(2)
            
        # Return to absolute mode
        self.grbl.send_command("G90")
        
        # Show direction fix if needed
        inverted = []
        if self.test_results.get('X_direction') == 'inverted':
            inverted.append('X')
        if self.test_results.get('Y_direction') == 'inverted':
            inverted.append('Y')
        if self.test_results.get('Z_direction') == 'inverted':
            inverted.append('Z')
            
        if inverted:
            current_mask = int(self.config.get(3, 0))
            new_mask = current_mask
            
            print(f"\n{Colors.YELLOW}Direction fixes needed:{Colors.END}")
            for axis in inverted:
                bit = {'X': 0, 'Y': 1, 'Z': 2}[axis]
                new_mask ^= (1 << bit)
                
            print(f"Current $3={current_mask}")
            print(f"Set $3={new_mask} to fix directions")
            
            if self.yes_no("\nApply direction fix now?"):
                response = self.grbl.send_command(f"$3={new_mask}")
                if response and response[-1] == 'ok':
                    self.print_success("Direction settings updated")
                    self.config[3] = new_mask
                    
        self.wait_enter()
        
    def limit_switch_test(self):
        """Test limit switches"""
        self.print_header("Limit Switch Test")
        
        self.print_info("This test checks each limit switch")
        self.print_info("You'll manually trigger each switch")
        
        print("\nCurrent limit switch configuration:")
        print(f"  Hard limits enabled ($21): {int(self.config.get(21, 0))}")
        print(f"  Limit invert mask ($5): {int(self.config.get(5, 0))}")
        
        # Disable hard limits for testing
        if int(self.config.get(21, 0)) == 1:
            self.print_warning("Disabling hard limits for testing...")
            self.grbl.send_command("$21=0")
            
        switches = ['X', 'Y', 'Z']
        
        for switch in switches:
            print(f"\n{Colors.BOLD}Testing {switch} limit switch{Colors.END}")
            
            # Get baseline
            status = self.grbl.get_status()
            pins_before = status.get('pins', '') if status else ''
            
            print(f"Manually trigger the {switch} limit switch...")
            
            # Monitor for change
            triggered = False
            start_time = time.time()
            
            while time.time() - start_time < 10:
                status = self.grbl.get_status()
                if status:
                    pins = status.get('pins', '')
                    if switch in pins and switch not in pins_before:
                        triggered = True
                        break
                time.sleep(0.1)
                
            if triggered:
                self.print_success(f"{switch} limit switch detected!")
                self.test_results[f'{switch}_limit'] = 'working'
                
                # Wait for release
                print("Release the switch...")
                while switch in self.grbl.get_status().get('pins', ''):
                    time.sleep(0.1)
                self.print_success("Switch released")
            else:
                self.print_error(f"{switch} limit switch not detected")
                self.test_results[f'{switch}_limit'] = 'not_working'
                print("Check wiring and $5 invert setting")
                
        # Re-enable hard limits if they were on
        if int(self.config.get(21, 0)) == 1:
            self.grbl.send_command("$21=1")
            
        self.wait_enter()
        
    def probe_test(self):
        """Test probe functionality"""
        self.print_header("Probe Test")
        
        self.print_info("Testing tool length probe / touch plate")
        
        if not self.yes_no("Do you have a probe connected?"):
            print("Skipping probe test")
            self.wait_enter()
            return
            
        print("\nProbe connection:")
        print("- Probe plate → PROBE connector Signal pin")
        print("- Alligator clip → Tool (ground)")
        
        print("\nTesting probe circuit...")
        
        # Monitor probe pin
        print("Touch the alligator clip to the probe plate...")
        
        triggered = False
        start_time = time.time()
        
        while time.time() - start_time < 10:
            status = self.grbl.get_status()
            if status and 'P' in status.get('pins', ''):
                triggered = True
                break
            time.sleep(0.1)
            
        if triggered:
            self.print_success("Probe circuit working!")
            self.test_results['probe'] = 'working'
            
            if self.yes_no("\nTest probe command (G38.3)?"):
                self.print_warning("Position probe plate below tool")
                self.print_warning("Tool will move down slowly until contact")
                
                if self.yes_no("Ready?"):
                    # Probe down 20mm max at 50mm/min
                    response = self.grbl.send_command("G38.3 Z-20 F50")
                    if response and 'error' not in response[-1]:
                        self.print_success("Probe successful!")
                        # Get probe position
                        status = self.grbl.get_status()
                        if status and 'mpos' in status:
                            z_pos = status['mpos']['z']
                            print(f"Probe triggered at Z={z_pos:.3f}")
                    else:
                        self.print_error("Probe failed")
        else:
            self.print_error("Probe not detected")
            self.test_results['probe'] = 'not_working'
            print("Check probe wiring and $6 setting")
            
        self.wait_enter()
        
    def spindle_test(self):
        """Test spindle control"""
        self.print_header("Spindle Control Test")
        
        self.print_warning("SAFETY: Ensure spindle area is clear!")
        
        spindle_type = self.get_choice([
            "VFD with 0-10V control",
            "PWM speed controller",
            "Simple relay on/off",
            "No spindle connected"
        ])
        
        if spindle_type == 3:
            print("Skipping spindle test")
            self.wait_enter()
            return
            
        print(f"\nMax spindle RPM ($30): {self.config.get(30, 24000)}")
        
        if spindle_type == 0:  # VFD
            self.print_info("Testing VFD control...")
            self.print_info("You should have:")
            self.print_info("- PA1 → PWM to 0-10V converter → VFD AI1")
            self.print_info("- PC7 (HEAT0) → VFD RUN")
            self.print_info("- PC6 (FAN1) → VFD FOR")
            
            if self.yes_no("\nConnections verified?"):
                # Test at different speeds
                speeds = [
                    (6000, "25%"),
                    (12000, "50%"),
                    (18000, "75%"),
                    (24000, "100%")
                ]
                
                for rpm, percent in speeds:
                    if self.yes_no(f"\nTest {percent} speed ({rpm} RPM)?"):
                        print(f"Starting spindle at {rpm} RPM...")
                        self.grbl.send_command(f"M3 S{rpm}")
                        time.sleep(3)
                        
                        if self.yes_no(f"Is spindle running at approximately {rpm} RPM?"):
                            self.print_success(f"{percent} speed test passed")
                        else:
                            self.print_warning("Check VFD parameters and 0-10V converter")
                            
                        print("Stopping spindle...")
                        self.grbl.send_command("M5")
                        time.sleep(2)
                        
        elif spindle_type == 1:  # PWM
            self.print_info("Testing PWM spindle control...")
            if self.yes_no("Test spindle?"):
                self.grbl.send_command("M3 S12000")
                time.sleep(2)
                if self.yes_no("Is spindle running?"):
                    self.print_success("PWM control working")
                self.grbl.send_command("M5")
                
        elif spindle_type == 2:  # Relay
            self.print_info("Testing relay control...")
            if self.yes_no("Test spindle on/off?"):
                self.grbl.send_command("M3")
                if self.yes_no("Is spindle ON?"):
                    self.print_success("Relay control working")
                self.grbl.send_command("M5")
                
        self.test_results['spindle'] = 'tested'
        self.wait_enter()
        
    def homing_test(self):
        """Test homing cycle"""
        self.print_header("Homing Cycle Test")
        
        self.print_info("Homing moves each axis toward limit switches")
        self.print_info("Then backs off and sets machine zero")
        
        # Check if limits are working
        limits_ok = all([
            self.test_results.get(f'{axis}_limit') == 'working'
            for axis in ['X', 'Y', 'Z']
        ])
        
        if not limits_ok:
            self.print_error("Some limit switches not working!")
            if not self.yes_no("Try homing anyway?"):
                self.wait_enter()
                return
                
        print("\nHoming configuration:")
        print(f"  Homing enabled ($22): {int(self.config.get(22, 0))}")
        print(f"  Homing direction ($23): {int(self.config.get(23, 0))}")
        print(f"  Homing feed rate ($24): {self.config.get(24, 100)} mm/min")
        print(f"  Homing seek rate ($25): {self.config.get(25, 1000)} mm/min")
        print(f"  Homing pull-off ($27): {self.config.get(27, 2)} mm")
        
        # Enable homing if needed
        if int(self.config.get(22, 0)) == 0:
            if self.yes_no("\nEnable homing?"):
                self.grbl.send_command("$22=1")
                self.config[22] = 1
                
        if self.yes_no("\nRun homing cycle?"):
            self.print_warning("Machine will move toward limit switches")
            self.print_info("Press E-stop if something goes wrong!")
            
            if self.yes_no("Ready?"):
                print("\nHoming...")
                response = self.grbl.send_command("$H", timeout=30)
                
                if response and response[-1] == 'ok':
                    self.print_success("Homing successful!")
                    
                    # Check position
                    status = self.grbl.get_status()
                    if status and 'mpos' in status:
                        print(f"Machine position: X={status['mpos']['x']:.3f} Y={status['mpos']['y']:.3f} Z={status['mpos']['z']:.3f}")
                else:
                    self.print_error("Homing failed")
                    print("Check limit switches and homing direction")
                    
        self.wait_enter()
        
    def movement_accuracy_test(self):
        """Test movement accuracy"""
        self.print_header("Movement Accuracy Test")
        
        self.print_info("This tests if motors move the correct distance")
        self.print_info("You'll need a ruler or calipers")
        
        axes = ['X', 'Y', 'Z']
        
        for axis in axes:
            if not self.yes_no(f"\nTest {axis} axis accuracy?"):
                continue
                
            print(f"\nCurrent {axis} steps/mm: {self.config.get(100 + 'XYZ'.index(axis), 160)}")
            
            # Move to safe position
            print("Moving to test position...")
            self.grbl.send_command("G90")  # Absolute mode
            self.grbl.send_command(f"G1 {axis}10 F1000")
            time.sleep(2)
            
            print(f"\nMark current {axis} position with tape/pencil")
            self.wait_enter()
            
            # Move 100mm
            test_distance = 100
            print(f"Moving {axis} {test_distance}mm...")
            self.grbl.send_command(f"G91")  # Relative mode
            self.grbl.send_command(f"G1 {axis}{test_distance} F1000")
            time.sleep(5)
            
            print(f"\nMeasure actual distance moved")
            
            while True:
                try:
                    actual = float(input("Enter measured distance (mm): "))
                    if 0 < actual < 200:
                        break
                except ValueError:
                    pass
                print("Please enter a valid measurement")
                
            # Calculate calibration
            error = abs(actual - test_distance) / test_distance * 100
            
            if error < 1:
                self.print_success(f"{axis} axis accurate (error: {error:.1f}%)")
            else:
                self.print_warning(f"{axis} axis error: {error:.1f}%")
                
                # Calculate new steps/mm
                setting_num = 100 + 'XYZ'.index(axis)
                old_steps = self.config.get(setting_num, 160)
                new_steps = old_steps * test_distance / actual
                
                print(f"Recommended: ${setting_num}={new_steps:.3f}")
                
                if self.yes_no("Apply calibration?"):
                    response = self.grbl.send_command(f"${setting_num}={new_steps:.3f}")
                    if response and response[-1] == 'ok':
                        self.print_success("Calibration applied")
                        self.config[setting_num] = new_steps
                        
            # Return to start
            self.grbl.send_command(f"G1 {axis}-{test_distance} F1000")
            self.grbl.send_command("G90")  # Back to absolute
            time.sleep(5)
            
        self.wait_enter()
        
    def save_configuration(self):
        """Save configuration and test results"""
        self.print_header("Save Configuration")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"cnc_config_{timestamp}.txt"
        
        with open(filename, 'w') as f:
            f.write(f"CNC Configuration Report\n")
            f.write(f"Generated: {datetime.now()}\n")
            f.write(f"{'='*50}\n\n")
            
            f.write("Test Results:\n")
            for test, result in self.test_results.items():
                f.write(f"  {test}: {result}\n")
                
            f.write(f"\nCurrent Settings:\n")
            for setting, value in sorted(self.config.items()):
                f.write(f"  ${setting}={value}\n")
                
            f.write(f"\nRecommended Actions:\n")
            
            # Direction fixes
            for axis in ['X', 'Y', 'Z']:
                if self.test_results.get(f'{axis}_direction') == 'inverted':
                    f.write(f"  - Invert {axis} direction in $3 setting\n")
                    
            # Limit switch fixes
            for axis in ['X', 'Y', 'Z']:
                if self.test_results.get(f'{axis}_limit') == 'not_working':
                    f.write(f"  - Check {axis} limit switch wiring\n")
                    
        self.print_success(f"Configuration saved to {filename}")
        print("\nYou can also save these settings to EEPROM:")
        
        if self.yes_no("Save all settings to EEPROM?"):
            response = self.grbl.send_command("$W")
            if response and response[-1] == 'ok':
                self.print_success("Settings saved to EEPROM")
            else:
                self.print_error("Failed to save settings")
                
    def run(self):
        """Main wizard loop"""
        self.print_header("BTT SKR Mini E3 v3.0 CNC Setup Wizard")
        
        print("This wizard will help you:")
        print("- Connect to your CNC controller")
        print("- Test motors, switches, and spindle")
        print("- Calibrate movement accuracy")
        print("- Configure grblHAL settings")
        
        self.wait_enter()
        
        # Connect first
        if not self.connect_to_cnc():
            print("\nCannot continue without connection")
            return
            
        # Main menu
        while True:
            self.print_header("Setup Menu")
            
            options = [
                "Basic Communication Test",
                "Motor Direction Test",
                "Limit Switch Test",
                "Probe Test",
                "Spindle Control Test",
                "Homing Cycle Test",
                "Movement Accuracy Test",
                "Save Configuration",
                "Exit"
            ]
            
            choice = self.get_choice(options)
            
            if choice == 0:
                self.basic_communication_test()
            elif choice == 1:
                self.motor_direction_test()
            elif choice == 2:
                self.limit_switch_test()
            elif choice == 3:
                self.probe_test()
            elif choice == 4:
                self.spindle_test()
            elif choice == 5:
                self.homing_test()
            elif choice == 6:
                self.movement_accuracy_test()
            elif choice == 7:
                self.save_configuration()
            elif choice == 8:
                break
                
        # Cleanup
        self.grbl.disconnect()
        print("\nSetup wizard complete!")
        print("Use gSender or UGS for full CNC control")

if __name__ == "__main__":
    try:
        wizard = CNCSetupWizard()
        wizard.run()
    except KeyboardInterrupt:
        print("\n\nSetup cancelled by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)