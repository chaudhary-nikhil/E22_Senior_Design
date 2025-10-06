# Real-time IMU Position Tracking Component

This component provides real-time 3D position tracking and visualization for IMU sensors during swimming motion analysis.

## Files

- `realtime_position_tracker.html` - Real-time 3D position tracking web interface
- `esp32_websocket_server.ino` - ESP32 code for real-time IMU data streaming
- `CMakeLists.txt` - Build configuration for ESP-IDF integration

## Features

### Real-time Position Tracking
- **3D Position**: Live X, Y, Z coordinates with accurate motion tracking
- **WebSocket Streaming**: Real-time data transmission at 100Hz
- **Motion Trail**: Visual path history with configurable length
- **Drift Correction**: High-pass filtering for velocity drift compensation

### Advanced Sensor Fusion
- **Gravity Compensation**: Proper gravity removal from accelerometer
- **Complementary Filter**: Combines accelerometer and gyroscope data
- **Quaternion Orientation**: Accurate 6DOF orientation tracking
- **Double Integration**: Acceleration → Velocity → Position

### Motion Analysis
- **Stroke Phase Detection**: Pull, recovery, entry, glide phases
- **Motion State**: Stationary, moving, accelerating, transitioning
- **Speed Calculation**: Real-time velocity magnitude
- **Distance Tracking**: Total path length measurement

### Visualization Features
- **3D IMU Model**: Red cube with orientation arrow
- **Coordinate Grid**: Reference system for position analysis
- **Real-time Updates**: Live position and orientation display
- **Interactive Controls**: Connect, disconnect, clear, reset, export

## Usage

### ESP32 Setup
1. Update WiFi credentials in `esp32_websocket_server.ino`:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

2. Upload the code to your ESP32 with MPU6050 sensor

3. Note the ESP32 IP address from serial monitor

### Web Interface
1. Open `realtime_position_tracker.html` in a web browser
2. Update the ESP32 IP address in the interface
3. Click "Connect" or enable "Auto Connect"
4. View real-time 3D position tracking

## Data Format

Real-time WebSocket data stream:
```json
{
  "ax": 0.402, "ay": -1.501, "az": 9.366,
  "gx": -0.149, "gy": -0.696, "gz": -0.517,
  "px": 0.123, "py": 0.456, "pz": 0.789,
  "vx": 0.012, "vy": 0.034, "vz": 0.056,
  "qw": 0.998, "qx": 0.012, "qy": 0.034, "qz": 0.056,
  "timestamp": 1234567890
}
```

## Technical Implementation

### Position Calculation
- **Gravity Removal**: Rotate gravity vector to body frame and subtract
- **Low-pass Filter**: Smooth acceleration data (α = 0.8)
- **Velocity Integration**: Integrate filtered acceleration
- **Drift Correction**: High-pass filter for velocity (β = 0.1)
- **Position Integration**: Integrate corrected velocity

### Orientation Tracking
- **Gyroscope Integration**: Quaternion-based rotation
- **Accelerometer Correction**: Gravity vector alignment
- **Complementary Filter**: Balance gyro and accel data (γ = 0.95)
- **Quaternion Normalization**: Maintain unit quaternion

### Motion Detection
- **Stroke Phases**: Based on velocity and acceleration patterns
- **Motion States**: Classified by acceleration and velocity magnitudes
- **Real-time Analysis**: Continuous phase and state detection

## Integration with ESP-IDF

This component integrates with the main ESP-IDF project:
- Uses existing `imu_mpu6050` component for sensor data
- WebSocket server for real-time data streaming
- Compatible with existing logging and storage components

## Controls

- **🔌 Connect**: Manual connection to ESP32
- **🔌 Disconnect**: Disconnect from ESP32
- **🗑️ Clear Path**: Clear motion trail
- **🔄 Reset Origin**: Reset position to (0,0,0)
- **💾 Export Data**: Save session data as JSON

## Settings

- **Auto Connect**: Automatically connect on startup
- **Show Trail**: Toggle motion trail visibility
- **Show Grid**: Toggle coordinate grid display
