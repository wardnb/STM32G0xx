#!/bin/bash
echo "Installing dependencies..."
pip3 install -r requirements.txt

echo "Starting CNC Setup Wizard..."
python3 cnc_setup_wizard.py