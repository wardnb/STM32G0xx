@echo off
echo Installing dependencies...
pip install -r requirements.txt

echo Starting CNC Setup Wizard...
python cnc_setup_wizard.py

pause