#!/bin/bash

# Configuration
MSPFLASHER_PATH="/Users/timcarlo/ti/MSPFlasher_1.3.20"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

OUTPUT_HEX="$SCRIPT_DIR/targets/msp430/build/build.hex"
DEVICE="MSP430FR5994"

echo "Starting flash process for $DEVICE..."
echo "$OUTPUT_HEX"

# Check if the file exists
if [ ! -f "$OUTPUT_HEX" ]; then
  echo "Error: File $OUTPUT_HEX not found!"
  exit 1
fi

echo "Flashing with MSP430Flasher..."
DYLD_LIBRARY_PATH="$MSPFLASHER_PATH" \
"$MSPFLASHER_PATH/MSP430Flasher" \
  -n "$DEVICE" \
  -w "$OUTPUT_HEX" \
  -v \
  -g

if [ $? -eq 0 ]; then
  echo "Flashing complete."
  echo "Press Restart to run the code."
else
  echo "Flashing failed."
  exit 1
fi