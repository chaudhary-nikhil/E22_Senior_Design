# BNO055 Swim Session Logger & Visualizer

A clean, minimal ESP32 + BNO055 IMU system for swim session logging and playback visualization.

## What It Does

- Reads 9-axis IMU data from BNO055 sensor via ESP32
- Logs complete swim sessions to JSON files
- Provides playback visualization of logged sessions
- Shows all sensor data: acceleration, gyroscope, magnetometer, quaternion, temperature, calibration

## Hardware

- ESP32 development board
- BNO055 IMU sensor
- Wiring: SDA=21, SCL=22, VCC=3.3V, GND=GND

## Software

### ESP32 Firmware
- `main/app_main.c` - Main application (53 lines)
- `components/imu_bno055/` - BNO055 sensor driver with clock stretching fixes
- `components/bus_i2c/` - I2C communication with BNO055 compatibility
- `components/serial_stream/` - Serial JSON output

### Python Session Logger
- `dashboard/session_logger.py` - Web server + serial reader (284 lines)
- `dashboard/session_visualizer.html` - Clean HTML interface (228 lines)
- `dashboard/visualizer.js` - Optimized JavaScript for fast JSON processing (813 lines)
- `dashboard/sessions/` - Directory for JSON session files (organized storage)

## Usage

### 1. Flash ESP32
```bash
source $HOME/esp/esp-idf/export.sh
idf.py build flash monitor
```

### 2. Start Session Logger
```bash
cd dashboard
python3 session_logger.py
```

### 3. Open Web Interface
- Navigate to: http://localhost:8016
- Click "Start Logging" to begin session
- Swim with ESP32 attached
- Click "Stop Logging" to save session to `dashboard/sessions/`
- Load saved session and click "Play" for visualization

## Features

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

## Data Format

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

## Requirements

- ESP-IDF framework
- Python 3.7+
- pyserial library
- Modern web browser with WebGL support

## Notes

- **Real Data Only**: No test data generation - requires actual ESP32 connection
- **BNO055 Optimized**: Includes clock stretching fixes for stable operation
- **Merge Compatible**: Maintains compatibility with main branch components