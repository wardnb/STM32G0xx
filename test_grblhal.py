#!/usr/bin/env python3
"""
Test script for grblHAL STM32G0xx firmware in Renode simulation
Connects to the UART telnet interface and tests basic functionality
"""

import socket
import time
import threading
import sys

def connect_to_grblhal():
    """Connect to grblHAL via telnet and test basic commands"""
    print("🔗 Connecting to grblHAL UART simulation...")
    
    try:
        # Connect to Renode UART telnet interface
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        sock.connect(('localhost', 3456))
        
        print("✅ Connected to grblHAL UART interface")
        print("📡 Waiting for grblHAL startup message...")
        
        # Read initial data
        response = sock.recv(1024).decode('utf-8', errors='ignore')
        print(f"📥 Received: {response}")
        
        # Test basic commands
        test_commands = [
            "?",      # Status request
            "$$",     # Show settings  
            "G0 X1",  # Move command
            "M3 S100",# Spindle command
            "M5",     # Spindle off
            "?"       # Final status
        ]
        
        for cmd in test_commands:
            print(f"📤 Sending: {cmd}")
            sock.send(f"{cmd}\n".encode())
            time.sleep(1)
            
            try:
                response = sock.recv(1024).decode('utf-8', errors='ignore')
                print(f"📥 Response: {response}")
            except socket.timeout:
                print("⚠️  No response received")
        
        sock.close()
        print("✅ grblHAL test completed successfully")
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        print("💡 Make sure Renode is running with the grblHAL simulation")

if __name__ == "__main__":
    connect_to_grblhal()