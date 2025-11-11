# GoldenForm Firmware - ESP32 IMU Systems

Modular ESP-IDF firmware supporting both **MPU6050** (6-axis) and **BNO055** (9-axis) IMU sensors with comprehensive visualization systems.

## 🏊 GoldenForm Session Logger & Visualizer

A complete swim analysis system using BNO055 9-axis IMU with session logging and playback visualization.

### What It Does
- Reads 9-axis IMU data from BNO055 sensor via ESP32
- Logs complete swim sessions to JSON files
- Provides playback visualization of logged sessions
- Shows all sensor data: acceleration, gyroscope, magnetometer, quaternion, temperature, calibration

### Hardware
- ESP32 development board
- BNO055 IMU sensor
- Wiring: SDA=21, SCL=22, VCC=3.3V, GND=GND

### Software
- `main/app_main.c` - Main application (53 lines)
- `components/imu_bno055/` - BNO055 sensor driver with clock stretching fixes
- `components/bus_i2c/` - I2C communication with BNO055 compatibility
- `components/serial_stream/` - Serial JSON output
- `dashboard/session_logger.py` - Web server + serial reader (286 lines)
- `dashboard/simple_session_logger.html` - Clean session logging interface
- `dashboard/sessions/` - Directory for JSON session files (organized storage)

### Usage
```bash
# 1. Flash ESP32
source $HOME/esp/esp-idf/export.sh
idf.py build flash monitor

# 2. Start Session Logger
cd dashboard
cd server
python3 session_logger.py

# 3. Open Web Interface
# Navigate to: http://localhost:8016
# Click "Start Logging" to begin session
# Swim with ESP32 attached
# Click "Stop Logging" to save session to dashboard/sessions/
# Load saved session and click "Play" for visualization
```

### Features
- **Real-time Logging**: Only works with actual ESP32 serial data
- **Session Management**: Start/stop logging with timestamps from t=0
- **Complete Data**: All 9-axis BNO055 data logged and visualized
- **Professional Charts**: Acceleration, gyroscope, magnetometer charts
- **Dynamic Orientation Analysis**: Automatically determines IMU starting orientation from acceleration data
- **Accurate Coordinate Mapping**: Maps BNO055 coordinate system to Three.js visualization correctly
- **Stroke Detection**: Automatic swimming stroke detection and analysis
- **Future GPS Integration**: Position tracking will be added via GPS module later
- **Clean Codebase**: Minimal, easy-to-understand code
- **Fast Processing**: Optimized JavaScript for quick JSON data rendering
- **Modular Structure**: Separated HTML, CSS, and JavaScript for maintainability

---


## 🏗️ Build System

### ESP-IDF Setup
```bash
idf.py set-target esp32
idf.py build flash monitor
```

### Structure
- `components/bus_i2c`: thin I²C helpers
- `components/imu_bno055`: BNO055 register driver (9-axis with fusion)
- `components/wifi_server`: WiFi Access Point and HTTP server for data transmission
- `components/serial_stream`: JSON serial output
- `main`: application tasks and wiring

### Configuration
- Keep `sdkconfig` out of git; use `sdkconfig.defaults` to share sane defaults
- Use feature flags in `Kconfig.projbuild` to toggle WiFi/Storage/Fusion choices
- Prefer `vTaskDelayUntil` for fixed-rate sampling loops

---

## 📊 Data Format

### BNO055 Session Data
Each logged data point contains:
```json
{
  "t": 158229,
  "session_time": 0.0,
  "ax": 0.1, "ay": 0.2, "az": 9.8,
  "gx": 0.01, "gy": 0.02, "gz": 0.03,
  "mx": 20.1, "my": -5.2, "mz": -10.3,
  "roll": 15.2, "pitch": -8.1, "yaw": 23.4,
  "qw": 0.9234, "qx": 0.1234, "qy": 0.2345, "qz": 0.3456,
  "temp": 25.6,
  "cal": {"sys": 3, "gyro": 3, "accel": 3, "mag": 3}
}
```

**Field Explanations:**
- `t`: ESP32 system uptime in milliseconds (raw FreeRTOS tick count)
- `session_time`: Time relative to session start (starts at 0.0 seconds)
- `ax/ay/az`: Acceleration in m/s²
- `gx/gy/gz`: Gyroscope in rad/s  
- `mx/my/mz`: Magnetometer in µT
- `roll/pitch/yaw`: Euler angles in degrees
- `qw/qx/qy/qz`: Quaternion components for 3D rotation
- `temp`: Temperature in °C
- `cal`: Calibration status (0-3 for each sensor)

---

## 🔧 Requirements

- ESP-IDF framework
- Python 3.7+
- BNO055 IMU sensor (for swim analysis)
- MPU6050 IMU sensor (for live visualization)