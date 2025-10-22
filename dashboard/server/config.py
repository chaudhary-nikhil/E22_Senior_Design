#!/usr/bin/env python3
"""
Configuration constants for GoldenForm session logger
"""

# BLE Configuration
DEVICE_NAME = "GoldenForm"
SERVICE_UUID = "0000180f-0000-1000-8000-00805f9b34fb"  # Battery Service UUID
CHARACTERISTIC_UUID = "00002a19-0000-1000-8000-00805f9b34fb"  # Battery Level Characteristic UUID

# Server Configuration
DEFAULT_PORT = 8016
FALLBACK_PORT = 8017

# Session Configuration
SESSIONS_DIR = "sessions"
SESSION_FILE_PREFIX = "swim_session_"
EMERGENCY_FILE_PREFIX = "swim_session_emergency_"

# Data Configuration
MAX_JSON_LENGTH = 512
BLE_SCAN_TIMEOUT = 10.0
BLE_CONNECTION_TIMEOUT = 5.0

# Logging Configuration
LOG_FORMAT = "📊 Logging: t={:.1f}s Roll={:.1f}° Cal={}/{}/{}/{}"
