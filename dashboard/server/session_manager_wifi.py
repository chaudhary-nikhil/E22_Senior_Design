#!/usr/bin/env python3
"""
Session management for GoldenForm data logging with WiFi communication
"""

import json
import os
import threading
import time
import struct
import requests
import glob
from datetime import datetime
try:
    from .config import SESSIONS_DIR, SESSION_FILE_PREFIX, EMERGENCY_FILE_PREFIX, LOG_FORMAT, ESP32_WIFI_IP
except ImportError:
    from config import SESSIONS_DIR, SESSION_FILE_PREFIX, EMERGENCY_FILE_PREFIX, LOG_FORMAT, ESP32_WIFI_IP

# Protobuf data packet structure (matches ESP32 struct)
class ProtobufIMUData:
    """Protobuf IMU data packet for efficient storage and transmission"""
    def __init__(self, timestamp_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz, sys_cal, gyro_cal, accel_cal, mag_cal):
        self.timestamp_ms = timestamp_ms
        self.ax, self.ay, self.az = ax, ay, az
        self.gx, self.gy, self.gz = gx, gy, gz
        self.qw, self.qx, self.qy, self.qz = qw, qx, qy, qz
        self.sys_cal, self.gyro_cal, self.accel_cal, self.mag_cal = sys_cal, gyro_cal, accel_cal, mag_cal
    
    def to_protobuf(self):
        """Convert to protobuf format for fast storage/transmission"""
        # Pack calibration status into single byte
        cal_status = (self.sys_cal & 0x3) | ((self.gyro_cal & 0x3) << 2) | ((self.accel_cal & 0x3) << 4) | ((self.mag_cal & 0x3) << 6)
        
        # Pack protobuf data: timestamp(4) + accel(12) + gyro(12) + quat(16) + cal(1) + reserved(4)
        return struct.pack('<IffffffffffffBBBBB',
            self.timestamp_ms,
            self.ax, self.ay, self.az,
            self.gx, self.gy, self.gz,
            self.qw, self.qx, self.qy, self.qz,
            cal_status,
            0, 0, 0, 0, 0, 0  # reserved bytes
        )
    
    def to_dict(self):
        """Convert to dictionary format for visualization"""
        return {
            "t": self.timestamp_ms,
            "ax": self.ax, "ay": self.ay, "az": self.az,
            "gx": self.gx, "gy": self.gy, "gz": self.gz,
            "qw": self.qw, "qx": self.qx, "qy": self.qy, "qz": self.qz,
            "cal": {
                "sys": self.sys_cal,
                "gyro": self.gyro_cal,
                "accel": self.accel_cal,
                "mag": self.mag_cal
            }
        }
    
    @classmethod
    def from_dict(cls, data_dict):
        """Create from dictionary format"""
        cal = data_dict.get('cal', {})
        return cls(
            data_dict.get('t', 0),
            data_dict.get('ax', 0), data_dict.get('ay', 0), data_dict.get('az', 0),
            data_dict.get('gx', 0), data_dict.get('gy', 0), data_dict.get('gz', 0),
            data_dict.get('qw', 1), data_dict.get('qx', 0), data_dict.get('qy', 0), data_dict.get('qz', 0),
            cal.get('sys', 0), cal.get('gyro', 0), cal.get('accel', 0), cal.get('mag', 0)
        )
    
    @classmethod
    def from_protobuf(cls, protobuf_data):
        """Create from protobuf data"""
        # Unpack protobuf data: timestamp(4) + accel(12) + gyro(12) + quat(16) + cal(1) + reserved(4)
        unpacked = struct.unpack('<IffffffffffffBBBBB', protobuf_data)
        timestamp_ms = unpacked[0]
        ax, ay, az = unpacked[1:4]
        gx, gy, gz = unpacked[4:7]
        qw, qx, qy, qz = unpacked[7:11]
        cal_status = int(unpacked[11])
        
        # Extract individual calibration values
        sys_cal = cal_status & 0x3
        gyro_cal = (cal_status >> 2) & 0x3
        accel_cal = (cal_status >> 4) & 0x3
        mag_cal = (cal_status >> 6) & 0x3
        
        return cls(timestamp_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz, 
                  sys_cal, gyro_cal, accel_cal, mag_cal)

class WiFiSessionManager:
    """Manages session logging using WiFi communication with ESP32"""
    
    def __init__(self):
        self.logging_active = False
        self.session_data = []  # Store as ProtobufIMUData objects
        self.session_start_time = None
        self.current_session_id = None
        self.data_lock = threading.Lock()
        self.esp32_ip = ESP32_WIFI_IP
        self.sessions_dir = SESSIONS_DIR
        os.makedirs(self.sessions_dir, exist_ok=True)
        
    def receive_session_data(self, protobuf_data):
        """Receive session data from ESP32 via WiFi"""
        try:
            # Parse protobuf data
            parsed_data = self._parse_protobuf_data(protobuf_data)
            
            if parsed_data:
                # Save session data
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                json_filename = os.path.join(self.sessions_dir, f"session_{timestamp}.json")
                
                with open(json_filename, 'w') as f:
                    json.dump(parsed_data, f, indent=2)
                
                print(f"✅ Session received: {json_filename} ({len(parsed_data)} data points)")
                
                # Also save protobuf data
                protobuf_filename = os.path.join(self.sessions_dir, f"session_{timestamp}.pb")
                with open(protobuf_filename, 'wb') as f:
                    f.write(protobuf_data)
                
                return json_filename
            else:
                print("❌ Failed to parse session data")
                return None
                
        except Exception as e:
            print(f"❌ Error processing session data: {e}")
            return None
    
    def _parse_protobuf_data(self, protobuf_data):
        """Parse protobuf data into JSON format"""
        try:
            # Each data point is 49 bytes: 1 uint32 + 10 floats + 1 uint8 + 4 uint8 reserved
            data_point_size = struct.calcsize('<L 10f B 4B')
            data_points = []
            
            for i in range(0, len(protobuf_data), data_point_size):
                chunk = protobuf_data[i:i + data_point_size]
                if len(chunk) == data_point_size:
                    try:
                        # Unpack: 1 uint32, 10 floats, 1 uint8 cal_status, 4 uint8 reserved
                        unpacked = struct.unpack('<L 10f B 4B', chunk)
                        
                        timestamp_ms = unpacked[0]
                        ax, ay, az = unpacked[1:4]
                        gx, gy, gz = unpacked[4:7]
                        qw, qx, qy, qz = unpacked[7:11]
                        cal_status = unpacked[11]
                        
                        # Extract individual calibration values from packed cal_status
                        sys_cal = cal_status & 0x3
                        gyro_cal = (cal_status >> 2) & 0x3
                        accel_cal = (cal_status >> 4) & 0x3
                        mag_cal = (cal_status >> 6) & 0x3
                        
                        data_points.append({
                            "t": timestamp_ms,
                            "ax": ax, "ay": ay, "az": az,
                            "gx": gx, "gy": gy, "gz": gz,
                            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
                            "sys_cal": sys_cal, "gyro_cal": gyro_cal,
                            "accel_cal": accel_cal, "mag_cal": mag_cal
                        })
                    except struct.error as e:
                        print(f"❌ Error unpacking data chunk: {e}")
                        break
                else:
                    print(f"❌ Incomplete data chunk: {len(chunk)}/{data_point_size} bytes")
                    break
            
            return data_points
            
        except Exception as e:
            print(f"❌ Error parsing protobuf data: {e}")
            return None
    
    def get_session_data(self):
        """Get the latest session data from saved files"""
        try:
            # List all session files and get the latest one
            session_files = self.list_sessions()
            if not session_files:
                return {"status": "error", "error": "No session data available"}
            
            # Get the most recent session
            latest_file = session_files[0]  # Already sorted by date
            filepath = os.path.join(self.sessions_dir, latest_file)
            
            with open(filepath, 'r') as f:
                session_data = json.load(f)
            
            return {
                "status": "data_retrieved",
                "session_id": latest_file.replace('.json', ''),
                "data_points": len(session_data),
                "json_filename": filepath,
                "data": session_data
            }
            
        except Exception as e:
            return {"status": "error", "error": f"Failed to get session data: {e}"}
    
    def get_session_status(self):
        """Get current session status"""
        return {
            "logging_active": self.logging_active,
            "session_id": self.current_session_id,
            "samples": len(self.session_data),
            "start_time": self.session_start_time.isoformat() if self.session_start_time else None,
            "esp32_status": "wifi_connected"
        }
    
    def list_sessions(self):
        """List all available session files"""
        json_files = glob.glob(f"{self.sessions_dir}/{SESSION_FILE_PREFIX}*.json")
        return sorted([os.path.basename(f) for f in json_files], reverse=True)
    
    def get_session_data_by_filename(self, filename):
        """Get data for a specific session file"""
        filepath = os.path.join(self.sessions_dir, filename)
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                return json.load(f)
        else:
            return None
    
    def create_visualization(self):
        """Create visualization by getting the latest session data"""
        try:
            # Get the latest session data
            result = self.get_session_data()
            if result["status"] == "data_retrieved":
                return {
                    "status": "visualization_created",
                    "session_id": result["session_id"],
                    "data_points": result["data_points"],
                    "json_filename": result["json_filename"]
                }
            else:
                return {"status": "error", "error": "No session data available for visualization"}
                
        except Exception as e:
            return {"status": "error", "error": f"Failed to create visualization: {e}"}
    
    def emergency_save(self):
        """Emergency save if actively logging"""
        try:
            with self.data_lock:
                if self.logging_active and self.session_data:
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    
                    # Save protobuf file
                    protobuf_filename = f"{self.sessions_dir}/{EMERGENCY_FILE_PREFIX}{timestamp}.pb"
                    with open(protobuf_filename, 'wb') as f:
                        for data_point in self.session_data:
                            f.write(data_point.to_protobuf())
                    
                    # Save JSON file for visualization
                    json_filename = f"{self.sessions_dir}/{EMERGENCY_FILE_PREFIX}{timestamp}.json"
                    json_data = [data_point.to_dict() for data_point in self.session_data]
                    with open(json_filename, 'w') as f:
                        json.dump(json_data, f, indent=2)
                    
                    print(f"💾 Emergency save: {len(self.session_data)} samples to {protobuf_filename}")
                    return len(self.session_data)
                elif self.session_data:
                    print(f"📊 Session data already saved ({len(self.session_data)} samples)")
                    return len(self.session_data)
                else:
                    print("📊 No session data to save")
                    return 0
        except Exception as e:
            print(f"⚠️  Emergency save warning: {e}")
            return 0

# For backward compatibility, create an alias
SessionManager = WiFiSessionManager
