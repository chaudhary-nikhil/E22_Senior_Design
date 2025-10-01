# ESP32 Makefile for ESP-IDF
# This Makefile provides convenient shortcuts for ESP32 development

# ESP-IDF commands
IDF_PY = idf.py

# Default target
all: build

# Build the project
build:
	$(IDF_PY) build

# Clean the project
clean:
	$(IDF_PY) fullclean

# Flash the ESP32
flash:
	$(IDF_PY) flash

# Monitor serial output
monitor:
	$(IDF_PY) monitor

# Build and flash
flash-build: build flash

# Build, flash and monitor
all-in-one: build flash monitor

# Set target (ESP32, ESP32-S2, ESP32-S3, etc.)
set-target:
	@echo "Available targets:"
	@echo "  esp32     - ESP32"
	@echo "  esp32s2   - ESP32-S2"
	@echo "  esp32s3   - ESP32-S3"
	@echo "  esp32c3   - ESP32-C3"
	@echo ""
	@echo "Usage: make set-target TARGET=esp32"

# Menuconfig (configure project)
menuconfig:
	$(IDF_PY) menuconfig

# Size analysis
size:
	$(IDF_PY) size

# Show help
help:
	@echo "ESP32 Development Commands:"
	@echo "  build        - Build the project"
	@echo "  clean        - Clean build artifacts"
	@echo "  flash        - Flash to ESP32"
	@echo "  monitor      - Monitor serial output"
	@echo "  flash-build  - Build and flash"
	@echo "  all-in-one   - Build, flash and monitor"
	@echo "  menuconfig   - Open configuration menu"
	@echo "  size         - Show memory usage"
	@echo "  set-target   - Set target chip"
	@echo "  help         - Show this help"

# Phony targets
.PHONY: all build clean flash monitor flash-build all-in-one menuconfig size help set-target
