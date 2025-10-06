# IMU Real-Time 3D Visualization System

A complete system that reads motion data from an MPU6050 IMU sensor on an ESP32 and displays it as a moving 3D object in real-time on a web browser.

## 🎯 What This System Does

- **ESP32 Firmware**: Reads 6-axis IMU data (accelerometer + gyroscope) at 100Hz
- **Serial Streaming**: Sends JSON data via USB cable to your computer
- **Python Web Server**: Processes IMU data and serves web interface
- **3D Visualization**: Real-time 3D cube that moves and rotates based on sensor motion
- **Live Data Display**: Shows current acceleration and angular velocity values

## 📋 Prerequisites

- ESP32 development board
- MPU6050 IMU sensor
- ESP-IDF development environment
- Python 3.6+ with required packages
- Modern web browser

## 🔧 Hardware Setup

1. **Connect MPU6050 to ESP32:**
   ```
   MPU6050    ESP32
   VCC    →   3.3V
   GND    →   GND
   SCL    →   GPIO 22
   SDA    →   GPIO 21
   ```

2. **Connect ESP32 to computer via USB cable**

## 🚀 Quick Start Guide

### Step 1: Build and Flash ESP32 Firmware

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

### Step 2: Start the Python Visualizer

```bash
# Navigate to scripts folder
cd scripts

# Start the Python web server
python3 simple_imu_visualizer.py
```

The server will start on `http://localhost:8003`

### Step 3: Open Web Interface

Open your web browser and go to:
```
http://localhost:8003
```

The interface will automatically connect and start showing real-time IMU data.

## 📁 Project Structure

```
E22_Senior_Design/
├── main/
│   ├── app_main.c              # ESP32 main application
│   └── CMakeLists.txt          # Main component configuration
├── components/
│   ├── bus_i2c/               # I2C communication driver
│   ├── imu_mpu6050/          # MPU6050 IMU sensor driver
│   └── serial_stream/         # Serial data streaming
├── scripts/
│   ├── simple_imu_visualizer.py  # Python web server
│   └── simple_imu_3d.html        # Web interface
├── docs/
│   └── README.md                 # This documentation
└── build/                     # Build output (auto-generated)
```

## 🔍 How It Works

### Data Flow
1. **ESP32** reads MPU6050 sensor data at 100Hz
2. **Serial Stream** sends JSON: `{"t":timestamp, "ax":1.08, "ay":-0.014, "az":8.899, "gx":-0.149, "gy":-0.696, "gz":-0.517}`
3. **Python Server** processes data and calculates position/rotation
4. **Web Browser** displays 3D cube moving in real-time

### Visualization Features
- **Position**: Red cube moves based on acceleration data (ax, ay, az)
- **Rotation**: Cube rotates based on angular velocity (gx, gy, gz)
- **Real-time Display**: Shows live acceleration and angular velocity values
- **3D Environment**: Three.js scene with camera controls

## 🛠️ Troubleshooting

### ESP32 Not Connecting
```bash
# Check available serial ports
ls /dev/cu.usbserial-*

# Try different baud rates
idf.py -p /dev/cu.usbserial-0001 -b 115200 monitor
```

### Python Server Issues
```bash
# Kill existing Python processes
pkill -f simple_imu_visualizer.py

# Check if port 8003 is in use
lsof -i :8003

# Start server with verbose output
python3 simple_imu_visualizer.py --verbose
```

### Web Interface Not Loading
- Ensure Python server is running on port 8003
- Check browser console for errors (F12)
- Try refreshing the page
- Clear browser cache

### No Data in Visualization
- Verify ESP32 is sending data (check serial monitor)
- Ensure Python server is receiving data (check terminal output)
- Check that web interface shows "Connected" status

## 📊 Expected Output

### ESP32 Serial Output
```
I (1234) APP: t=1000 ax=1.080 ay=-0.014 az=8.899 m/s^2 | gx=-0.149 gy=-0.696 gz=-0.517 rad/s
I (1244) SERIAL_STREAM: Sent: {"t":1000,"ax":1.080,"ay":-0.014,"az":8.899,"gx":-0.149,"gy":-0.696,"gz":-0.517}
```

### Python Server Output
```
Server running on http://localhost:8003
Client connected. Total clients: 1
Processed JSON IMU: t=1000, ax=1.080, ay=-0.014, az=8.899
```

### Web Interface
- Red cube moving in 3D space
- Live data values updating
- Connection status showing "Connected"

## 🔧 Customization

### Change Sampling Rate
Edit `main/app_main.c`:
```c
const TickType_t period = pdMS_TO_TICKS(10); // 100Hz (10ms)
// Change to pdMS_TO_TICKS(20) for 50Hz
```

### Modify Visualization
Edit `simple_imu_3d.html`:
- Change cube color, size, or shape
- Add new data displays
- Modify 3D scene settings

### Adjust Data Processing
Edit `simple_imu_visualizer.py`:
- Modify position calculation algorithms
- Add filtering or smoothing
- Change data format

## 📚 Technical Details

### ESP32 Firmware
- **Framework**: ESP-IDF
- **Communication**: I2C (400kHz)
- **Sampling**: 100Hz
- **Data Format**: JSON over serial

### Python Server
- **Framework**: HTTP server with Server-Sent Events
- **Port**: 8003
- **Data Processing**: Real-time IMU data conversion
- **Communication**: Serial USB + HTTP

### Web Interface
- **Framework**: Three.js for 3D graphics
- **Communication**: Server-Sent Events
- **Features**: Real-time 3D visualization, live data display

## 🚀 Next Steps

1. **Enhanced Algorithms**: Implement proper sensor fusion (Kalman filter, Madgwick filter)
2. **Multiple Sensors**: Add support for multiple IMU sensors
3. **Wireless Communication**: Replace USB with WiFi or Bluetooth
4. **Advanced Visualization**: Add human body models, motion analysis
5. **Data Logging**: Save and replay motion data
6. **Mobile App**: Create mobile interface for remote monitoring

## 📞 Support

If you encounter issues:
1. Check the troubleshooting section above
2. Verify all hardware connections
3. Ensure ESP-IDF and Python environments are properly set up
4. Check serial port permissions and availability

---

**System Status**: ✅ Working - Real-time IMU data streaming to 3D web visualization