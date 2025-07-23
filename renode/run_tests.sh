#!/bin/bash
# Run grblHAL tests in Renode

echo "=== grblHAL Renode Test Runner ==="
echo ""

# Use local Renode installation
RENODE_PATH="$HOME/e3_mini_v3/renode/renode"

# Check if Renode is available
if [ ! -f "$RENODE_PATH" ]; then
    echo "ERROR: Renode not found at $RENODE_PATH"
    echo ""
    echo "Please ensure Renode is properly extracted at ~/e3_mini_v3/renode/"
    echo "Directory should contain the 'renode' executable"
    exit 1
fi

# Build the firmware first
echo "Building firmware..."
cd ..
~/.platformio/penv/bin/platformio run -e BTT_SKR_MINI_E3_V30_USB

if [ $? -ne 0 ]; then
    echo "ERROR: Firmware build failed"
    exit 1
fi

cd renode

# Run Renode with test script
echo ""
echo "Starting Renode emulation..."
echo "Once in Renode console, type:"
echo "  include @test_grblhal.resc"
echo "  reset"
echo "  start"
echo ""

$RENODE_PATH test_grblhal.resc