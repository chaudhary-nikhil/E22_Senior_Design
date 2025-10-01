# ESP32-WROOM Hello World Project

A simple ESP32-WROOM project boilerplate using ESP-IDF framework with FreeRTOS, optimized for ESP32-WROOM development boards.

## Project Structure

```
.
├── CMakeLists.txt         # ESP-IDF project configuration
├── main/
│   ├── CMakeLists.txt     # Main component configuration
│   └── main.c             # Main application source
├── Makefile              # ESP-IDF convenience commands
├── sdkconfig.defaults    # Default ESP32 configuration
└── README.md             # This file
```

## Prerequisites

1. **Install ESP-IDF**: Follow the [ESP-IDF Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)

2. **Set up ESP-IDF environment**:
   ```bash
   # Add to your shell profile (.bashrc, .zshrc, etc.)
   . $HOME/esp/esp-idf/export.sh
   ```

## Building and Flashing

### Quick Start

```bash
# Build the project
make build

# Flash to ESP32
make flash

# Monitor serial output
make monitor

# Build, flash and monitor in one command
make all-in-one
```

### Available Commands

```bash
make help                    # Show all available commands
make build                   # Build the project
make clean                   # Clean build artifacts
make flash                   # Flash to ESP32
make monitor                 # Monitor serial output
make flash-build            # Build and flash
make all-in-one             # Build, flash and monitor
make menuconfig             # Open configuration menu
make size                   # Show memory usage
```

### Using idf.py directly

```bash
idf.py build                # Build
idf.py flash monitor        # Flash and monitor
idf.py menuconfig           # Configuration
idf.py size                 # Memory analysis
```

## Features

- **ESP32 Native**: Uses ESP-IDF framework
- **FreeRTOS**: Real-time operating system
- **GPIO Control**: LED blinking example
- **Logging**: ESP-IDF logging system
- **Memory Monitoring**: Heap usage tracking
- **Configurable**: Easy to customize for different ESP32 variants

## Hardware Requirements

- **ESP32-WROOM development board** (ESP32-DevKitC, ESP32-WROOM-32, NodeMCU-32S, etc.)
- USB cable for programming
- Built-in LED (usually on GPIO 2)
- Boot button (GPIO 0) for user interaction

## Example Output

```
I (1234) ESP32_WROOM: Hello, ESP32-WROOM World!
I (1235) ESP32_WROOM: Chip: esp32 with 2 CPU cores, WiFi/BT/BLE, 
I (1236) ESP32_WROOM: Silicon revision: 1
I (1237) ESP32_WROOM: Free heap size: 295000 bytes
I (1238) ESP32_WROOM: CPU frequency: 240 MHz
I (1239) ESP32_WROOM: GPIO configured - LED on GPIO 2, Button on GPIO 0
I (2240) ESP32_WROOM: Counter: 0, LED: ON, Button: RELEASED
I (3241) ESP32_WROOM: Counter: 1, LED: OFF, Button: PRESSED
I (4242) ESP32_WROOM: Counter: 2, LED: ON, Button: RELEASED
...
I (30000) ESP32_WROOM: ESP32-WROOM Features:
I (30001) ESP32_WROOM:   - Dual-core 32-bit processor
I (30002) ESP32_WROOM:   - Built-in WiFi antenna
I (30003) ESP32_WROOM:   - Bluetooth Classic & BLE
I (30004) ESP32_WROOM:   - 4MB Flash, 520KB SRAM
I (30005) ESP32_WROOM:   - 34 GPIO pins available
```

## Configuration

### Set Target Chip

```bash
# For ESP32
idf.py set-target esp32

# For ESP32-S2
idf.py set-target esp32s2

# For ESP32-S3
idf.py set-target esp32s3

# For ESP32-C3
idf.py set-target esp32c3
```

### Customize Configuration

```bash
make menuconfig
```

## Customization

This boilerplate can be extended with:

1. **WiFi Connectivity**: Add WiFi initialization and connection
2. **Bluetooth**: Enable BLE or Classic Bluetooth
3. **Sensors**: Add I2C, SPI, or UART sensor integration
4. **Web Server**: Create HTTP server for remote control
5. **OTA Updates**: Implement over-the-air firmware updates
6. **Power Management**: Add deep sleep and power optimization

## Troubleshooting

### Common Issues

1. **Permission denied**: Add your user to the `dialout` group (Linux)
2. **Port not found**: Check USB connection and drivers
3. **Build errors**: Ensure ESP-IDF is properly installed and sourced

### Useful Commands

```bash
# Check ESP-IDF installation
idf.py --version

# List available serial ports
ls /dev/ttyUSB* /dev/ttyACM* /dev/cu.*

# Monitor with specific baud rate
idf.py monitor -b 115200
```

## Next Steps

- Add WiFi connectivity
- Implement sensor reading
- Create web interface
- Add OTA update capability
- Implement power management
