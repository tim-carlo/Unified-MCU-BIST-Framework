#!/bin/bash

# Configuration
MSPFLASHER_PATH="/Users/timcarlo/ti/MSPFlasher_1.3.20"
OUTPUT_HEX="../build/build.hex"
DEVICE="MSP430FR5994"

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