Quick Start:
cd /Users/Abathini/E22_Senior_Design && source $HOME/esp/esp-idf/export.sh && idf.py build && idf.py flash
cd dashboard/server && python session_logger.py
Connect to "GoldenForm" WiFi (password: goldenform123)
Press EN button → Press BOOT button → Open http://localhost:8016 → View 3D forearm rotation!

# GoldenForm - WiFi IMU System

A complete system that reads motion data from a BNO055 9-axis IMU sensor on an ESP32 and streams it wirelessly via WiFi to a web-based session logger and visualizer.

## 🎯 What This System Does

- **ESP32 Firmware**: Reads 9-axis IMU data (accelerometer + gyroscope + magnetometer) at configurable rate
- **WiFi Streaming**: Sends protobuf data wirelessly via WiFi to your computer
- **Session Logger**: Python web server for logging swim sessions and managing data
- **3D Visualization**: Real-time 3D visualization of IMU orientation and motion
- **Data Storage**: Automatic session logging with JSON file storage and playback

## 📋 Prerequisites

- ESP32 development board
- BNO055 9-axis IMU sensor
- ESP-IDF development environment
- Python 3.6+ with required packages (`requests` for WiFi)
- Modern web browser

## 🔧 Hardware Setup

1. **Connect BNO055 to ESP32:**
   ```
   BNO055    ESP32
   VCC    →   3.3V
   GND    →   GND
   SCL    →   GPIO 22
   SDA    →   GPIO 21
   ```

2. **Connect laptop to ESP32 WiFi** - everything is wireless!

## 🚀 Quick Start Guide

### Option 1: Automated Installation (Recommended)

```bash
# Run the installation script
./install.sh

# This will:
# - Check Python installation
# - Install all dependencies
# - Verify ESP-IDF setup
# - Check for ESP32 connection
```

### Option 2: Manual Installation

#### Step 1: Build and Flash ESP32 Firmware

```bash
# Navigate to project root
cd /path/to/E22_Senior_Design

# Build the project
idf.py build

# Flash to ESP32 (replace /dev/cu.usbserial-* with your port)
idf.py -p /dev/cu.usbserial-0001 flash

# Monitor serial output
idf.py -p /dev/cu.usbserial-0001 monitor
```

### Step 2: Install Python Dependencies

```bash
# Install all required packages from requirements file
pip install -r requirements.txt

# Or install manually if needed
pip install requests
```

**Note**: The `requirements.txt` file contains all necessary Python dependencies for the WiFi functionality.

### Step 3: Start the Session Logger

```bash
# Navigate to dashboard folder
cd dashboard

# Start the Bluetooth session logger
python3 run_session_logger.py
```

The server will start on `http://localhost:8016`

### Step 4: Open Web Interface

Open your web browser and go to:
```
http://localhost:8016
```

The interface will automatically connect via Bluetooth and start logging IMU data.

## 📁 Project Structure

```
E22_Senior_Design/
├── main/
│   ├── app_main.c              # ESP32 main application
│   └── CMakeLists.txt          # Main component configuration
├── components/
│   ├── bus_i2c/               # I2C communication driver
│   ├── imu_bno055/           # BNO055 IMU sensor driver
│   ├── wifi_server/          # WiFi Access Point and HTTP server
│   └── serial_stream/        # Serial JSON output
├── dashboard/
│   ├── server/                   # Python backend components
│   │   ├── session_logger.py     # Main application logic
│   │   ├── session_manager.py    # Session data management
│   │   ├── web_server.py         # HTTP server & API
│   │   └── config.py             # Configuration constants
│   └── client/                   # Web interface components
│       ├── simple_session_logger.html # Main dashboard interface
│       └── visualization.html    # 3D forearm rotation visualization
├── docs/
│   └── README.md              # This documentation
├── requirements.txt           # Python dependencies
├── dependencies.lock         # ESP-IDF component dependencies
├── install.sh               # Automated installation script
└── build/                   # Build output (auto-generated)
```

## 🔍 How It Works

### Data Flow
1. **ESP32** reads BNO055 sensor data at configurable rate
2. **WiFi** sends protobuf data: 57-byte binary packets with IMU data
3. **Session Logger** processes data and logs to JSON files
4. **Web Browser** displays 3D visualization and session management

### WiFi Features
- **Network Name**: "GoldenForm"
- **Password**: "goldenform123"
- **ESP32 IP**: 192.168.4.1
- **Connection**: Direct WiFi connection
- **Data**: Fast protobuf transmission for large data

### Session Management
- **Press EN Button**: Start logging IMU data (LED turns ON)
- **Press BOOT Button**: Stop logging and save session (LED turns OFF)
- **Refresh Data**: Retrieve latest session from ESP32
- **View Forearm Rotation**: 3D visualization of forearm movement

## 🛠️ Troubleshooting

### ESP32 Not Creating WiFi Network
```bash
# Check ESP32 logs for WiFi initialization
idf.py monitor

# Look for: "WiFi: Access Point running - ready for connections"
```

### Python Server Issues
```bash
# Kill existing Python processes
pkill -f session_logger.py

# Check if port 8016 is in use
lsof -i :8016

# Install missing dependencies
pip install -r requirements.txt

# Or install manually
pip install requests
```

### Python Dependencies Issues
```bash
# Check if requests is installed
python3 -c "import requests; print('Requests installed successfully')"

# Reinstall dependencies
pip uninstall requests
pip install requests

# Check Python version (requires 3.6+)
python3 --version
```

### WiFi Connection Issues
- Ensure ESP32 is powered on and WiFi is initialized
- Check that "GoldenForm" network appears in WiFi list
- Try restarting the Python session logger
- Check system WiFi permissions
- Connect to "GoldenForm" network (password: goldenform123)

### No Data in Visualization
- Verify ESP32 is sending data (check serial monitor)
- Ensure Python server shows "Connected" status
- Check that web interface shows "Connected" status
- Verify BNO055 calibration status

## 📊 Expected Output

### ESP32 Serial Output
```
I (1234) APP: All systems initialized, starting real-time IMU streaming via Bluetooth
I (1244) BLE_SERVICE: Advertising started successfully
I (1254) BLE_SERVICE: ESP_GATTS_CONNECT_EVT, conn_id 0
I (1264) APP: Bluetooth: Client connected - data streaming active
```

### Python Server Output
```
🏊 GoldenForm Session Logger (Bluetooth)
==========================================
🔍 Scanning for Bluetooth devices...
✅ Found target device: GoldenForm at AA:BB:CC:DD:EE:FF
🔗 Connecting to AA:BB:CC:DD:EE:FF...
✅ Connected successfully!
📊 Notifications enabled - receiving IMU data...
📊 Logging: t=1.0s Roll=1.2° Cal=3/3/3/3
```

### Web Interface
- Session management controls
- Real-time 3D visualization
- Connection status showing "Connected"
- Session list and playback controls

## 🔧 Customization

### Change Sampling Rate
Edit `main/app_main.c`:
```c
const TickType_t period = pdMS_TO_TICKS(100); // 10Hz (100ms)
// Change to pdMS_TO_TICKS(50) for 20Hz
```

### Modify Bluetooth Settings
Edit `components/ble/ble_service.h`:
- Change device name
- Modify service/characteristic UUIDs
- Adjust advertising parameters

### Adjust Data Processing
Edit `dashboard/session_logger.py`:
- Modify data processing algorithms
- Add filtering or smoothing
- Change data format or logging behavior

## 📚 Technical Details

### ESP32 Firmware
- **Framework**: ESP-IDF
- **Communication**: I2C (100kHz) + Bluetooth Low Energy
- **Sampling**: Configurable rate (default 10Hz)
- **Data Format**: JSON over BLE notifications

### Python Server
- **Framework**: HTTP server with Bluetooth integration
- **Port**: 8016
- **Bluetooth**: Bleak library for BLE communication
- **Data Processing**: Real-time IMU data logging

### Web Interface
- **Framework**: Three.js for 3D graphics
- **Communication**: HTTP + Server-Sent Events
- **Features**: Session management, 3D visualization, data playback

## 🚀 Next Steps

1. **Haptic Feedback**: Add BLE notifications for form deviations
2. **Ideal Form Comparison**: Implement baseline form analysis
3. **Advanced Analytics**: Add stroke analysis and metrics
4. **Mobile App**: Create mobile interface for swim tracking
5. **Cloud Integration**: Upload sessions to cloud storage
6. **Real-time Coaching**: AI-powered form feedback

## 📞 Support

If you encounter issues:
1. Check the troubleshooting section above
2. Verify all hardware connections
3. Ensure ESP-IDF and Python environments are properly set up
4. Check Bluetooth permissions and system settings
5. Verify BNO055 calibration status

---

**System Status**: ✅ Working - Wireless IMU data streaming via Bluetooth with session logging and 3D visualization