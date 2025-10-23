#!/bin/bash
# Installation script for GoldenForm
# This script sets up all dependencies and verifies the installation

echo "🏊 GoldenForm - Installation Script"
echo "=========================================="
echo ""

# Check if Python 3 is installed
echo "🔍 Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.6+ first."
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "✅ Python $PYTHON_VERSION found"

# Check if pip is installed
if ! command -v pip3 &> /dev/null && ! command -v pip &> /dev/null; then
    echo "❌ pip is not installed. Please install pip first."
    exit 1
fi

# Install Python dependencies
echo ""
echo "📦 Installing Python dependencies..."
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
    if [ $? -eq 0 ]; then
        echo "✅ Python dependencies installed successfully"
    else
        echo "❌ Failed to install Python dependencies"
        exit 1
    fi
else
    echo "⚠️  requirements.txt not found, installing requests manually..."
    pip3 install requests
fi

# Verify requests installation
echo ""
echo "🔍 Verifying WiFi library installation..."
python3 -c "import requests; print('✅ Requests library installed successfully')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Requests library verification failed"
    exit 1
fi

# Check ESP-IDF installation
echo ""
echo "🔍 Checking ESP-IDF installation..."
if command -v idf.py &> /dev/null; then
    echo "✅ ESP-IDF found"
    IDF_VERSION=$(idf.py --version | head -n1)
    echo "   Version: $IDF_VERSION"
else
    echo "⚠️  ESP-IDF not found in PATH"
    echo "   Make sure ESP-IDF is installed and sourced"
    echo "   Run: . $HOME/esp/esp-idf/export.sh"
fi

# Check if ESP32 is connected
echo ""
echo "🔍 Checking for ESP32 connection..."
if ls /dev/cu.usbserial-* 2>/dev/null; then
    echo "✅ ESP32 device found"
else
    echo "⚠️  No ESP32 device found"
    echo "   Make sure ESP32 is connected via USB"
fi

echo ""
echo "🎉 Installation complete!"
echo ""
echo "Next steps:"
echo "1. Build and flash ESP32: idf.py build && idf.py flash"
echo "2. Start session logger: python3 dashboard/run_session_logger.py"
echo "3. Open browser: http://localhost:8016"
echo ""
echo "For troubleshooting, see docs/README.md"
