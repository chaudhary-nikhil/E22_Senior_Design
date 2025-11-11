#!/usr/bin/env python3
"""
Configuration constants for GoldenForm session logger with WiFi support
"""

# WiFi Configuration
ESP32_WIFI_IP = "192.168.4.1"  # ESP32 Access Point IP
ESP32_WIFI_SSID = "GoldenForm"  # ESP32 Access Point SSID
ESP32_WIFI_PASSWORD = "goldenform123"  # ESP32 Access Point Password

# Server Configuration
DEFAULT_PORT = 8016
FALLBACK_PORT = 8017

# Session Configuration
SESSIONS_DIR = "sessions"
SESSION_FILE_PREFIX = "session_"
EMERGENCY_FILE_PREFIX = "session_emergency_"

# Data Configuration
MAX_JSON_LENGTH = 512
WIFI_CONNECTION_TIMEOUT = 5.0
WIFI_DATA_TIMEOUT = 30.0

# Logging Configuration
LOG_FORMAT = "📊 Logging: t={:.1f}s Roll={:.1f}° Cal={}/{}/{}/{}"
