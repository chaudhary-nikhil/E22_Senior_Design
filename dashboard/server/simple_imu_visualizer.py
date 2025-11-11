#!/usr/bin/env python3
"""
Simple IMU Data Visualizer for Translation Tracking
Reads IMU data from WiFi session manager and displays calibrated translation movement
"""

import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import os
import sys
import math
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from dashboard.server.session_manager_wifi import WiFiSessionManager, ProtobufIMUData
except ImportError:
    from session_manager_wifi import WiFiSessionManager, ProtobufIMUData

# Global variables
latest_data = None
data_lock = threading.Lock()
clients = []
session_manager = None

# Translation tracking state
class TranslationTracker:
    """Tracks translation movement from calibrated IMU data"""
    def __init__(self):
        # Position, velocity, and acceleration state
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z in meters
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz in m/s
        self.last_accel = np.array([0.0, 0.0, 0.0])
        self.last_timestamp = None
        
        # Calibration offsets - these should be set based on initial sensor state
        self.accel_offset = np.array([0.0, 0.0, 0.0])  # Gravity and bias offset
        self.gyro_offset = np.array([0.0, 0.0, 0.0])   # Gyro bias offset
        
        # Filter parameters for noise reduction
        self.alpha = 0.1  # Low-pass filter coefficient for acceleration
        self.velocity_decay = 0.95  # Velocity decay to prevent drift
        
        # Axis isolation - which axis to track (0=X, 1=Y, 2=Z)
        self.track_axis = 0  # Default to X-axis (forward/backward)
        
    def calibrate(self, initial_accel, initial_gyro):
        """Set calibration offsets based on initial sensor readings"""
        # Assume initial state is at rest, so acceleration should be gravity only
        # For swimming, we might want to calibrate when arm is in neutral position
        self.accel_offset = np.array(initial_accel)
        self.gyro_offset = np.array(initial_gyro)
        print(f"Calibrated offsets - Accel: {self.accel_offset}, Gyro: {self.gyro_offset}")
    
    def update(self, accel, gyro, timestamp_ms):
        """Update position based on calibrated acceleration data"""
        if self.last_timestamp is None:
            self.last_timestamp = timestamp_ms
            return self.position.copy()
        
        # Calculate time delta in seconds
        dt = (timestamp_ms - self.last_timestamp) / 1000.0
        if dt <= 0 or dt > 1.0:  # Skip invalid or too large time deltas
            self.last_timestamp = timestamp_ms
            return self.position.copy()
        
        # Apply calibration offsets to remove bias
        calibrated_accel = np.array(accel) - self.accel_offset
        calibrated_gyro = np.array(gyro) - self.gyro_offset
        
        # Low-pass filter to reduce noise
        filtered_accel = self.alpha * calibrated_accel + (1 - self.alpha) * self.last_accel
        self.last_accel = filtered_accel
        
        # For translation tracking, we focus on linear acceleration
        # Remove gravity component if sensor is calibrated properly
        # In swimming, we're interested in the movement acceleration, not gravity
        
        # Update velocity using acceleration (single integration)
        # Apply velocity decay to prevent drift accumulation
        self.velocity = self.velocity * self.velocity_decay + filtered_accel * dt
        
        # Update position using velocity (double integration)
        self.position = self.position + self.velocity * dt
        
        # For axis isolation, zero out non-tracked axes
        if self.track_axis == 0:  # Track X only
            self.position[1] = 0.0
            self.position[2] = 0.0
            self.velocity[1] = 0.0
            self.velocity[2] = 0.0
        elif self.track_axis == 1:  # Track Y only
            self.position[0] = 0.0
            self.position[2] = 0.0
            self.velocity[0] = 0.0
            self.velocity[2] = 0.0
        elif self.track_axis == 2:  # Track Z only
            self.position[0] = 0.0
            self.position[1] = 0.0
            self.velocity[0] = 0.0
            self.velocity[1] = 0.0
        
        self.last_timestamp = timestamp_ms
        return self.position.copy()
    
    def reset(self):
        """Reset position and velocity to zero"""
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_accel = np.array([0.0, 0.0, 0.0])
        self.last_timestamp = None

tracker = TranslationTracker()
calibration_done = False

class SSEHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/events':
            self.handle_sse()
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), '..', 'client', 'simple_imu_3d.html')
            with open(html_path, 'r') as f:
                self.wfile.write(f.read().encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def handle_sse(self):
        # Server-Sent Events
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        
        # Add client to list
        clients.append(self)
        print(f"Client connected. Total clients: {len(clients)}")
        
        try:
            while True:
                # Send latest data to client
                with data_lock:
                    if latest_data:
                        # Send SSE formatted data
                        self.wfile.write(f"data: {latest_data}\n\n".encode())
                        self.wfile.flush()
                time.sleep(0.02)  # 50Hz (matching IMU sampling rate)
        except:
            pass
        finally:
            if self in clients:
                clients.remove(self)
            print(f"Client disconnected. Total clients: {len(clients)}")

def broadcast_data(data):
    """Broadcast data to all connected SSE clients"""
    with data_lock:
        global latest_data
        latest_data = data
    
    # Send to all clients
    for client in clients[:]:  # Copy list to avoid modification during iteration
        try:
            client.wfile.write(f"data: {data}\n\n".encode())
            client.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            clients.remove(client)

def process_imu_data_for_translation(imu_data):
    """Process calibrated IMU data to extract translation movement"""
    global calibration_done, tracker
    
    # Extract data from IMU data structure
    timestamp_ms = imu_data.timestamp_ms
    accel = [imu_data.ax, imu_data.ay, imu_data.az]
    gyro = [imu_data.gx, imu_data.gy, imu_data.gz]
    quat = [imu_data.qw, imu_data.qx, imu_data.qy, imu_data.qz]
    
    # Calibrate on first reading if not done
    if not calibration_done:
        tracker.calibrate(accel, gyro)
        calibration_done = True
    
    # Update translation tracker
    position = tracker.update(accel, gyro, timestamp_ms)
    
    # Create visualization data structure
    data = {
        "acceleration": {"ax": accel[0], "ay": accel[1], "az": accel[2]},
        "angular_velocity": {"gx": gyro[0], "gy": gyro[1], "gz": gyro[2]},
        "position": {"px": position[0], "py": position[1], "pz": position[2]},
        "velocity": {"vx": tracker.velocity[0], "vy": tracker.velocity[1], "vz": tracker.velocity[2]},
        "quaternion": {"qw": quat[0], "qx": quat[1], "qy": quat[2], "qz": quat[3]},
        "timestamp": timestamp_ms,
        "calibration": {
            "sys": imu_data.sys_cal,
            "gyro": imu_data.gyro_cal,
            "accel": imu_data.accel_cal,
            "mag": imu_data.mag_cal
        }
    }
    
    return json.dumps(data)

def data_processor():
    """Process session data from WiFi session manager"""
    global session_manager
    
    session_manager = WiFiSessionManager()
    print("WiFi Session Manager initialized")
    
    while True:
        try:
            # Pull latest session data from ESP32
            result = session_manager.pull_from_esp32()
            
            if result and result.get("status") == "data_retrieved":
                session_data = result.get("data", [])
                print(f"Processing {len(session_data)} data points from ESP32")
                
                # Process each data point
                for data_point in session_data:
                    # Convert to ProtobufIMUData format
                    imu_data = ProtobufIMUData.from_dict(data_point)
                    
                    # Process for translation
                    json_data = process_imu_data_for_translation(imu_data)
                    broadcast_data(json_data)
                    
                    # Small delay to match real-time playback
                    time.sleep(0.02)  # 50Hz
                
                print("Finished processing session data")
            
            # Wait before checking for new data
            time.sleep(1.0)
            
        except Exception as e:
            print(f"Error processing data: {e}")
            time.sleep(1.0)

def start_web_server(port):
    """Start the web server"""
    server = HTTPServer(('localhost', port), SSEHandler)
    print(f"Web server started on http://localhost:{port}")
    server.serve_forever()

if __name__ == "__main__":
    print("ESP32 IMU Translation Tracker")
    print("==============================")
    print("Using calibrated IMU data for translation tracking")
    print("Position calculated from double integration of acceleration")
    print("Noise filtered using low-pass filter and axis isolation")
    
    # Determine port from command-line argument or environment variable
    default_port = 8003
    port = int(os.environ.get("PORT", default_port))
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port argument '{sys.argv[1]}', using port {port}")
    
    # Start data processor in background
    processor_thread = threading.Thread(target=data_processor, daemon=True)
    processor_thread.start()
    
    # Start web server
    start_web_server(port)
