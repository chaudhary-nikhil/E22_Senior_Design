#!/bin/bash

echo "📊 FormSync ESP32 Monitor Logs"
echo "=============================="
echo ""

cd /Users/administrator/Desktop/projects/E22_Senior_Design

# Source ESP-IDF
source /Users/administrator/esp/esp-idf/export.sh 2>&1 | grep -i "done"

echo ""
echo "🔍 Starting ESP32 monitor..."
echo "   Press Ctrl+] to exit"
echo ""
echo "Looking for BLE messages..."
echo "----------------------------"
echo ""

# Run monitor
idf.py monitor

