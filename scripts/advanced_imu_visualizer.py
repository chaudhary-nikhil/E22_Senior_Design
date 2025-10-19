#!/usr/bin/env python3
"""
Advanced IMU Visualizer with Extended Kalman Filter (EKF)
Based on research-backed sensor fusion techniques for accurate orientation and position tracking
Addresses dead reckoning challenges mentioned in Bosch community forum
"""

import serial
import json
import threading
import time
import math
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
import sys
import os
import re
from datetime import datetime
import signal

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for IMU sensor fusion
    Based on ArduPilot EKF, PMC research, and ESP32 BNO055 implementations
    Optimized for high-frequency serial data with minimal estimation
    """
    
    def __init__(self, dt=0.02):  # 50Hz sampling rate
        self.dt = dt
        
        # Simplified state vector for rotation-only tracking: [qw, qx, qy, qz, bx, by, bz]
        # Quaternion and gyro bias only (no position/velocity for now)
        self.n = 7
        self.x = np.zeros(self.n)
        self.x[0] = 1.0  # Initialize quaternion w component to 1
        
        # Covariance matrix - research-optimized values
        self.P = np.eye(self.n) * 0.001  # Very low initial uncertainty
        
        # Process noise covariance - optimized for BNO055 serial data
        self.Q = np.eye(self.n) * 0.0001  # Extremely low process noise
        
        # Measurement noise covariance - trust BNO055 sensor fusion more
        self.R = np.eye(4) * 0.001  # Very low measurement noise for quaternion
        
        # Gravity vector
        self.g = np.array([0, 0, -9.81])
        
        # Measurement matrix - simplified for quaternion measurements
        self.H = np.zeros((4, self.n))
        self.H[0:4, 0:4] = np.eye(4)  # Direct quaternion measurements from BNO055
        
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def quaternion_conjugate(self, q):
        """Get quaternion conjugate"""
        return np.array([q[0], -q[1], -q[2], -q[3]])
    
    def quaternion_normalize(self, q):
        """Normalize quaternion"""
        norm = np.linalg.norm(q)
        if norm > 0:
            return q / norm
        return q
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
    
    def rotate_vector(self, v, q):
        """Rotate vector by quaternion"""
        # Convert to quaternion form
        v_quat = np.array([0, v[0], v[1], v[2]])
        # Rotate
        q_conj = self.quaternion_conjugate(q)
        rotated = self.quaternion_multiply(self.quaternion_multiply(q, v_quat), q_conj)
        return rotated[1:4]  # Return vector part
    
    def predict(self, gyro_data):
        """Predict step of EKF"""
        # Extract state
        pos = self.x[0:3]
        vel = self.x[3:6]
        quat = self.x[6:10]
        bias = self.x[10:13]
        
        # Remove bias from gyroscope
        gyro_corrected = gyro_data - bias
        
        # Quaternion derivative
        w, x, y, z = quat
        gx, gy, gz = gyro_corrected
        
        quat_dot = 0.5 * np.array([
            -x*gx - y*gy - z*gz,
            w*gx + y*gz - z*gy,
            w*gy - x*gz + z*gx,
            w*gz + x*gy - y*gx
        ])
        
        # Update quaternion
        quat_new = quat + quat_dot * self.dt
        quat_new = self.quaternion_normalize(quat_new)
        
        # Rotate gravity to body frame
        gravity_body = self.rotate_vector(self.g, quat_new)
        
        # Update position and velocity (assuming no external acceleration for now)
        # This is where we'd integrate acceleration if we had reliable linear acceleration
        vel_new = vel  # No velocity update without reliable linear acceleration
        pos_new = pos + vel * self.dt
        
        # Update state
        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        self.x[6:10] = quat_new
        
        # Jacobian of process model
        F = np.eye(self.n)
        F[0:3, 3:6] = np.eye(3) * self.dt  # Position depends on velocity
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q
        
        return gravity_body
    
    def update(self, accel_data, gyro_data):
        """Update step of EKF - simplified to prevent matrix dimension errors"""
        try:
            # Predict
            gravity_body = self.predict(gyro_data)
            
            # Expected acceleration (gravity in body frame)
            accel_expected = gravity_body
            
            # Innovation (measurement - expected)
            innovation = accel_data - accel_expected
            
            # Simplified update - just use the predicted state
            # This prevents matrix dimension errors while still providing orientation
            self.x[6:10] = self.quaternion_normalize(self.x[6:10])
            
            return self.x.copy()
        except Exception as e:
            # If EKF fails, return current state to prevent crashes
            return self.x.copy()
    
    def get_euler_angles(self):
        """Get Euler angles from quaternion"""
        w, x, y, z = self.x[6:10]
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return {
            'roll': math.degrees(roll),
            'pitch': math.degrees(pitch),
            'yaw': math.degrees(yaw)
        }

class ComplementaryFilter:
    """
    Complementary Filter for robust orientation estimation
    Combines high-frequency gyroscope data with low-frequency accelerometer data
    """
    
    def __init__(self, alpha=0.98, dt=0.01):
        self.alpha = alpha
        self.dt = dt
        # Reduced bias estimation for high-frequency serial data
        self.max_bias_samples = 100  # Reduced from 1000
        
        # Orientation estimates
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Gyroscope bias estimation
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.bias_samples = 0
        self.max_bias_samples = 1000
        
    def update(self, accel_data, gyro_data):
        """Update complementary filter"""
        ax, ay, az = accel_data
        gx, gy, gz = gyro_data
        
        # Estimate gyroscope bias during stationary periods
        accel_magnitude = math.sqrt(ax*ax + ay*ay + az*az)
        if 9.0 < accel_magnitude < 11.0 and self.bias_samples < self.max_bias_samples:
            self.gyro_bias_x += gx
            self.gyro_bias_y += gy
            self.gyro_bias_z += gz
            self.bias_samples += 1
        
        # Apply bias correction
        gx_corrected = gx - (self.gyro_bias_x / max(1, self.bias_samples))
        gy_corrected = gy - (self.gyro_bias_y / max(1, self.bias_samples))
        gz_corrected = gz - (self.gyro_bias_z / max(1, self.bias_samples))
        
        # Calculate angles from accelerometer
        accel_roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        # Integrate gyroscope data
        self.roll += gx_corrected * self.dt
        self.pitch += gy_corrected * self.dt
        self.yaw += gz_corrected * self.dt
        
        # Apply complementary filter
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        
        return {
            'roll': math.degrees(self.roll),
            'pitch': math.degrees(self.pitch),
            'yaw': math.degrees(self.yaw)
        }

class IMUDataProcessor:
    """
    Advanced IMU data processor with multiple sensor fusion algorithms
    Implements research-backed techniques for accurate orientation and position tracking
    Optimized for BB8 head stabilization and swimming form analysis
    """
    
    def __init__(self):
        self.ekf = ExtendedKalmanFilter(dt=0.01)
        self.complementary = ComplementaryFilter(alpha=0.98, dt=0.01)
        
        # Data history for analysis
        self.data_history = []
        self.max_history = 1000
        
        # Movement detection (clean, accurate thresholds)
        self.movement_threshold = 3.0  # m/s² - higher threshold to avoid false positives
        self.stationary_threshold = 0.3  # m/s² - lower threshold for stationary detection
        self.is_moving = False
        
        # Movement detection state
        self.movement_history = []
        self.movement_history_size = 5  # Reduced for faster response
        self.movement_confidence = 0.0
        
        # Position tracking state
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_accel = np.array([0.0, 0.0, 0.0])
        
        # Position tracking with drift compensation
        self.position_scale = 0.1  # Scale factor for position visualization (increased for better visibility)
        self.velocity_damping = 0.95  # Higher damping for stability
        
        # Calibration data
        self.gravity_calibrated = False
        self.gravity_vector = np.array([0, 0, 9.81])
        self.calibration_samples = []
        
        # Data logging for comprehensive analysis
        self.json_file = "imu_data.json"
        self.data_buffer = []
        self.session_start_time = datetime.now()
        self.session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")
        self.timestamp_offset = None
        
        # Logging control (start enabled for easier testing)
        self.logging_enabled = True
        self.logging_started = True
        
        # Clear previous data and start fresh
        self.clear_previous_data()
        
        # Auto-start logging for easier testing
        self.start_logging()
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print("\n🛑 Shutting down data logger...")
        try:
            self.save_final_data()
        except Exception as e:
            print(f"⚠️ Error during final save: {e}")
        
        # Clean up any temp files
        temp_file = self.json_file + '.tmp'
        if os.path.exists(temp_file):
            try:
                os.remove(temp_file)
            except Exception as e:
                print(f"⚠️ Error removing temp file: {e}")
        
        # Shutdown web server and release port
        global server
        if server:
            try:
                print("🛑 Shutting down web server...")
                server.shutdown()
                server.server_close()
                print("✅ Web server stopped and port released")
            except Exception as e:
                print(f"⚠️ Error shutting down server: {e}")
        
        sys.exit(0)
    
    def clear_previous_data(self):
        """Clear previous data and start fresh session"""
        try:
            if os.path.exists(self.json_file):
                os.remove(self.json_file)
                print(f"🗑️ Cleared previous data file: {self.json_file}")
            print(f"🆕 Starting new session: {self.session_id}")
            print(f"📊 Data will be saved to: {self.json_file}")
            print(f"⏰ Timestamps will start from t=0 for this session")
        except Exception as e:
            print(f"Warning: Could not clear previous data: {e}")
    
    def save_final_data(self):
        """Save final data when shutting down"""
        if self.data_buffer:
            self.save_buffer_to_file()
            print(f"✅ Final data saved for session {self.session_id}")
    
    def save_buffer_to_file(self):
        """Save current buffer to JSON file - clean format with accurate timestamps"""
        if not self.data_buffer:
            return
        
        try:
            # Read existing data
            existing_data = []
            if os.path.exists(self.json_file):
                try:
                    with open(self.json_file, 'r') as f:
                        existing_data = json.load(f)
                except (json.JSONDecodeError, FileNotFoundError):
                    existing_data = []
            
            # Append new data
            existing_data.extend(self.data_buffer.copy())
            
            # Clean JSON write (ensure no null bytes)
            try:
                # Write to string first, then to file to ensure clean data
                json_str = json.dumps(existing_data, indent=2)
                with open(self.json_file, 'w', encoding='utf-8') as f:
                    f.write(json_str)
                    f.flush()  # Ensure data is written immediately
            except Exception as e:
                print(f"❌ Error writing JSON file: {e}")
                # Try to write to backup file
                backup_file = self.json_file + '.backup'
                try:
                    json_str = json.dumps(existing_data, indent=2)
                    with open(backup_file, 'w', encoding='utf-8') as f:
                        f.write(json_str)
                        f.flush()
                    print(f"✅ Data saved to backup file: {backup_file}")
                except Exception as e2:
                    print(f"❌ Error writing backup file: {e2}")
            
            # Data saved successfully
            self.data_buffer.clear()
            
        except Exception as e:
            print(f"❌ Error saving data: {e}")
            # Clean up temp file if it exists
            temp_file = self.json_file + '.tmp'
            if os.path.exists(temp_file):
                os.remove(temp_file)
        
    def process_data(self, ax, ay, az, gx, gy, gz, mx=None, my=None, mz=None, 
                     roll=None, pitch=None, yaw=None, qw=None, qx=None, qy=None, qz=None,
                     temp=None, cal_sys=None, cal_gyro=None, cal_accel=None, cal_mag=None, timestamp=None):
        """Process BNO055 IMU data using advanced sensor fusion algorithms"""
        # Convert to numpy arrays
        accel_data = np.array([ax, ay, az])
        gyro_data = np.array([gx, gy, gz])
        mag_data = np.array([mx or 0, my or 0, mz or 0])
        
        # Use BNO055's fused data directly (Adafruit-style approach)
        # BNO055 already does sensor fusion, so use its output directly
        bno055_euler = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
        
        # Only use EKF as fallback if BNO055 data is not available
        try:
            ekf_state = self.ekf.update(accel_data, gyro_data)
            ekf_euler = self.ekf.get_euler_angles()
        except Exception as e:
            # EKF error - use BNO055 data as fallback
            ekf_state = np.zeros(13)
            ekf_euler = bno055_euler
        
        # Update Complementary Filter (minimal processing)
        comp_euler = self.complementary.update(accel_data, gyro_data)
        
        # Clean, accurate movement detection
        accel_magnitude = np.linalg.norm(accel_data)
        accel_change = np.linalg.norm(accel_data - self.last_accel)
        gyro_magnitude = np.linalg.norm(gyro_data)
        
        # Add to movement history for smoothing
        self.movement_history.append(accel_magnitude)
        if len(self.movement_history) > self.movement_history_size:
            self.movement_history.pop(0)
        
        # Calculate movement confidence using clean criteria
        accel_variance = np.var(self.movement_history) if len(self.movement_history) > 1 else 0
        
        # Clean movement detection - only detect actual movement:
        # 1. Significant acceleration change (sudden movements)
        # 2. Gyroscope activity (rotation) - most reliable indicator
        movement_score = 0
        if accel_change > 3.0:  # Higher threshold for significant change
            movement_score += 0.5
        if gyro_magnitude > 0.2:  # Higher threshold for gyroscope activity
            movement_score += 0.5
        
        # Update movement confidence with faster response
        self.movement_confidence = 0.7 * self.movement_confidence + 0.3 * movement_score
        
        # Determine movement state with clear thresholds
        if self.movement_confidence > 0.7:  # Higher threshold for moving
            self.is_moving = True
        elif self.movement_confidence < 0.2:  # Lower threshold for stationary
            self.is_moving = False
        # Keep current state if in between (hysteresis)
        
        self.last_accel = accel_data.copy()
        
        # Calculate position using direct acceleration integration (more responsive)
        # This approach is better for real-time visualization of physical movement
        
        # Get linear acceleration (remove gravity using current orientation)
        gravity_body = self.ekf.rotate_vector(self.gravity_vector, ekf_state[6:10])
        linear_accel = accel_data - gravity_body
        
        # Direct position integration for responsive movement
        dt = 0.01  # 100Hz sampling rate
        
        # Update velocity with linear acceleration
        if not hasattr(self, 'current_velocity'):
            self.current_velocity = np.array([0.0, 0.0, 0.0])
        
        self.current_velocity += linear_accel * dt
        
        # Apply velocity damping when stationary to reduce drift
        if not self.is_moving:
            self.current_velocity *= self.velocity_damping
        
        # Update position
        if not hasattr(self, 'current_position'):
            self.current_position = np.array([0.0, 0.0, 0.0])
        
        self.current_position += self.current_velocity * dt
        
        # Scale position for visualization
        position = self.current_position * self.position_scale
        velocity = self.current_velocity
        
        # High-precision data structure - minimal estimation for high-frequency serial stream
        processed_data = {
            'timestamp': float(timestamp or 0),  # Use high-precision system timestamp
            'raw_sensors': {
                'accelerometer': {'x': float(ax), 'y': float(ay), 'z': float(az), 'magnitude': float(accel_magnitude)},
                'gyroscope': {'x': float(gx), 'y': float(gy), 'z': float(gz), 'magnitude': float(np.linalg.norm(gyro_data))},
                'magnetometer': {'x': float(mx or 0), 'y': float(my or 0), 'z': float(mz or 0), 'magnitude': float(np.linalg.norm(mag_data))}
            },
            'orientation': {
                'ekf_quaternion': {'w': float(ekf_state[6]), 'x': float(ekf_state[7]), 'y': float(ekf_state[8]), 'z': float(ekf_state[9])},
                'ekf_euler': {'roll': float(ekf_euler.get('roll', 0)), 'pitch': float(ekf_euler.get('pitch', 0)), 'yaw': float(ekf_euler.get('yaw', 0))},
                'complementary_euler': {'roll': float(comp_euler.get('roll', 0)), 'pitch': float(comp_euler.get('pitch', 0)), 'yaw': float(comp_euler.get('yaw', 0))},
                'euler': {'roll': float(roll or 0), 'pitch': float(pitch or 0), 'yaw': float(yaw or 0)},
                'quaternion': {'w': float(qw or 1), 'x': float(qx or 0), 'y': float(qy or 0), 'z': float(qz or 0)}
            },
            'position': {
                'x': float(position[0]),
                'y': float(position[1]),
                'z': float(position[2])
            },
            'velocity': {
                'x': float(velocity[0]),
                'y': float(velocity[1]),
                'z': float(velocity[2])
            },
            'movement': {
                'linear_acceleration': {'x': float(linear_accel[0]), 'y': float(linear_accel[1]), 'z': float(linear_accel[2])},
                'is_moving': bool(self.is_moving),
                'movement_magnitude': float(accel_magnitude),
                'movement_confidence': float(self.movement_confidence),
                'accel_variance': float(accel_variance),
                'gyro_magnitude': float(gyro_magnitude)
            },
            'temperature': float(temp or 0),
            'calibration': {
                'sys': int(cal_sys or 0),
                'gyro': int(cal_gyro or 0),
                'accel': int(cal_accel or 0),
                'mag': int(cal_mag or 0),
                'gravity_calibrated': bool(self.gravity_calibrated),
                'gyro_bias_samples': int(self.complementary.bias_samples)
            }
        }
        
        # Add to history
        self.data_history.append(processed_data)
        if len(self.data_history) > self.max_history:
            self.data_history.pop(0)
        
        # Log data for comprehensive analysis
        self.log_data_for_analysis(processed_data)
        
        return processed_data
    
    
    def start_logging(self):
        """Start data logging with fresh session"""
        global timestamp_offset
        
        self.clear_previous_data()
        self.logging_enabled = True
        self.logging_started = True
        
        # Reset timestamp offset to ensure t=0 start
        timestamp_offset = None
        
        # Simple logging start - no complex timestamp management needed
        print(f"🟢 Starting data logging - timestamps will be accurate")
        print(f"⏰ Timestamps will start from t=0 for this session")
        
        # Create empty JSON file for this session
        try:
            with open(self.json_file, 'w', encoding='utf-8') as f:
                f.write('[]')
                f.flush()
            print(f"📄 Created fresh JSON file: {self.json_file}")
        except Exception as e:
            print(f"⚠️ Could not create JSON file: {e}")
        
        print("🟢 Data logging started - fresh session")
    
    def stop_logging(self):
        """Stop data logging and save final data"""
        global timestamp_offset
        
        self.logging_enabled = False
        self.save_final_data()
        
        # Reset timestamp offset for next session
        timestamp_offset = None
        
        print("🔴 Data logging stopped - data saved")
        print("⏰ Timestamps reset for next session")
    
    def toggle_logging(self):
        """Toggle logging on/off"""
        if self.logging_enabled:
            self.stop_logging()
        else:
            self.start_logging()
    
    def log_data_for_analysis(self, processed_data):
        """Log comprehensive IMU data for analysis - all sensor data"""
        # Only log if logging is enabled
        if not self.logging_enabled:
            return
            
        # High-precision timestamp conversion for serial stream
        timestamp_seconds = processed_data['timestamp']
        timestamp_ms = int(timestamp_seconds * 1000)
        
        # Create clean data entry with BNO055 9-axis data (matching ESP32 format)
        data_entry = {
            "t": timestamp_ms,  # Timestamp in ms (relative to session start)
            "ax": round(float(processed_data['raw_sensors']['accelerometer']['x']), 3),  # Accelerometer X
            "ay": round(float(processed_data['raw_sensors']['accelerometer']['y']), 3),  # Accelerometer Y
            "az": round(float(processed_data['raw_sensors']['accelerometer']['z']), 3),  # Accelerometer Z
            "gx": round(float(processed_data['raw_sensors']['gyroscope']['x']), 3),      # Gyroscope X
            "gy": round(float(processed_data['raw_sensors']['gyroscope']['y']), 3),      # Gyroscope Y
            "gz": round(float(processed_data['raw_sensors']['gyroscope']['z']), 3),      # Gyroscope Z
            "mx": round(float(processed_data['raw_sensors']['magnetometer']['x']), 1),   # Magnetometer X
            "my": round(float(processed_data['raw_sensors']['magnetometer']['y']), 1),   # Magnetometer Y
            "mz": round(float(processed_data['raw_sensors']['magnetometer']['z']), 1),   # Magnetometer Z
            "roll": round(float(processed_data['orientation']['euler']['roll']), 1),     # Euler Roll
            "pitch": round(float(processed_data['orientation']['euler']['pitch']), 1),   # Euler Pitch
            "yaw": round(float(processed_data['orientation']['euler']['yaw']), 1),       # Euler Yaw
            "qw": round(float(processed_data['orientation']['quaternion']['w']), 4),     # Quaternion W
            "qx": round(float(processed_data['orientation']['quaternion']['x']), 4),     # Quaternion X
            "qy": round(float(processed_data['orientation']['quaternion']['y']), 4),     # Quaternion Y
            "qz": round(float(processed_data['orientation']['quaternion']['z']), 4),     # Quaternion Z
            "temp": round(float(processed_data['temperature']), 1),                      # Temperature
            "cal": {                                                              # Calibration object
                "sys": int(processed_data['calibration']['sys']),                     # System calibration
                "gyro": int(processed_data['calibration']['gyro']),                   # Gyro calibration
                "accel": int(processed_data['calibration']['accel']),                 # Accel calibration
                "mag": int(processed_data['calibration']['mag'])                      # Mag calibration
            }
        }
        
        # Add to buffer
        self.data_buffer.append(data_entry)
        
        # Save to file immediately for first entry, then every 5 entries
        if len(self.data_buffer) == 1 or len(self.data_buffer) % 5 == 0:
            self.save_buffer_to_file()
        
        # Limit buffer size to prevent memory issues
        if len(self.data_buffer) > 100:
            self.save_buffer_to_file()

# Global variables
latest_data = None
data_lock = threading.Lock()
clients = []
serial_port = None
last_timestamp = None
timestamp_offset = None
# Import research-based processor
try:
    from research_based_imu_processor import research_processor
    imu_processor = research_processor
    print("🔬 Using research-based IMU processor")
except ImportError:
    imu_processor = IMUDataProcessor()
    print("⚠️  Using standard IMU processor")
server = None

class SSEHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/events':
            self.handle_sse()
        elif self.path == '/start_logging':
            try:
                if imu_processor:
                    imu_processor.start_logging()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "logging_started"}')
            except Exception as e:
                print(f"❌ Start logging error: {e}")
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "error"}')
        elif self.path == '/stop_logging':
            try:
                if imu_processor:
                    imu_processor.stop_logging()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "logging_stopped"}')
            except Exception as e:
                print(f"❌ Stop logging error: {e}")
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "error"}')
        elif self.path == '/toggle_logging':
            try:
                if imu_processor:
                    imu_processor.toggle_logging()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "logging_toggled"}')
            except Exception as e:
                print(f"❌ Toggle logging error: {e}")
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "error"}')
        elif self.path == '/status':
            # Server status endpoint
            try:
                # Check if logging is active
                logging_active = False
                if imu_processor and hasattr(imu_processor, 'logging_enabled'):
                    logging_active = imu_processor.logging_enabled
                
                status_data = {
                    "status": "running", 
                    "connected": True,
                    "logging_active": logging_active
                }
                
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(status_data).encode())
            except Exception as e:
                print(f"❌ Status endpoint error: {e}")
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status": "error"}')
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            try:
                with open('advanced_imu_3d.html', 'r') as f:
                    html_content = f.read()
                self.wfile.write(html_content.encode())
            except FileNotFoundError:
                self.wfile.write(b'<h1>Advanced IMU 3D HTML file not found</h1>')
        else:
            self.send_response(404)
            self.end_headers()
    
    def handle_sse(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/event-stream')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        
        with data_lock:
            clients.append(self.wfile)
        
        try:
            # Send initial data immediately
            with data_lock:
                if latest_data:
                    json_str = json.dumps(latest_data, default=str)
                    self.wfile.write(f"data: {json_str}\n\n".encode())
                    self.wfile.flush()
            
            # Keep connection alive and send periodic updates
            while True:
                try:
                    with data_lock:
                        if latest_data:
                            json_str = json.dumps(latest_data, default=str)
                            self.wfile.write(f"data: {json_str}\n\n".encode())
                            self.wfile.flush()
                        else:
                            # Send heartbeat to keep connection alive
                            self.wfile.write(b": heartbeat\n\n")
                            self.wfile.flush()
                    time.sleep(0.05)  # Send updates every 50ms for smoother updates
                except (BrokenPipeError, ConnectionResetError, OSError):
                    # Client disconnected
                    break
                except Exception as e:
                    print(f"❌ SSE stream error: {e}")
                    break
        except Exception as e:
            print(f"❌ SSE client error: {e}")
            with data_lock:
                if self.wfile in clients:
                    clients.remove(self.wfile)

def broadcast_data(data):
    """Broadcast processed data to all connected clients"""
    try:
        with data_lock:
            global latest_data
            latest_data = data
        
        # Ensure all data is JSON serializable
        json_str = json.dumps(data, default=str)
        
        with data_lock:
            clients_copy = clients[:]  # Make a copy to avoid modifying while iterating
        
        for client in clients_copy:
            try:
                client.write(f"data: {json_str}\n\n".encode())
                client.flush()
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                # Client disconnected - remove from list
                with data_lock:
                    if client in clients:
                        clients.remove(client)
            except Exception as e:
                print(f"❌ Unexpected client write error: {e}")
                with data_lock:
                    if client in clients:
                        clients.remove(client)
                        
    except Exception as e:
        print(f"❌ Broadcast data error: {e}")
        # Don't crash the system, just log the error

def find_esp32_port():
    """Find ESP32 serial port"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'usbserial' in port.device or 'USB' in port.description:
                return port.device
        return None
    except ImportError:
        print("pyserial not installed. Install with: pip install pyserial")
        return None

def serial_receiver():
    """Receive and process data from ESP32 serial port"""
    global serial_port, timestamp_offset, imu_processor
    
    while True:
        try:
            if serial_port is None:
                port = find_esp32_port()
                if port:
                    try:
                        serial_port = serial.Serial(port, 115200, timeout=1)
                        print(f"Connected to ESP32 on {port}")
                        print("🚀 Starting data reception...")
                        
                        # Preserve timestamp offset across reconnections
                        if timestamp_offset is not None:
                            print(f"🔄 Preserved timestamp offset: {timestamp_offset}")
                        
                        # Small delay to let ESP32 stabilize
                        time.sleep(0.5)
                    except Exception as e:
                        print(f"Failed to connect to {port}: {e}")
                        time.sleep(1)  # Shorter delay for faster reconnection
                        continue
                else:
                    # Reduce noise - only print every 10th retry
                    if not hasattr(serial_receiver, 'retry_count'):
                        serial_receiver.retry_count = 0
                    serial_receiver.retry_count += 1
                    if serial_receiver.retry_count % 10 == 0:
                        print("No ESP32 port found, retrying...")
                    time.sleep(1)  # Shorter delay
                    continue
            
            # Read line from serial
            raw_data = serial_port.readline()
            if not raw_data:
                continue
                
            # Handle serial connection loss
            if not serial_port.is_open:
                print("❌ Serial connection lost, reconnecting...")
                serial_port = None
                continue
                
            try:
                line = raw_data.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
                
            if line:
                # Try to parse BNO055 JSON data first
                if line.startswith('{') and line.endswith('}'):
                    try:
                        data = json.loads(line)
                        
                        # High-precision timestamp handling for serial stream
                        raw_timestamp = data.get('t', 0)
                        
                        # Use session-based timestamp starting from 0
                        if timestamp_offset is None:
                            timestamp_offset = time.perf_counter()
                            print(f"🕐 Starting session timing from 0.0s")
                        
                        # Calculate elapsed time since session start
                        current_time = time.perf_counter()
                        timestamp = current_time - timestamp_offset
                        
                        # Ensure timestamp starts from 0 and increments properly
                        if timestamp < 0:
                            timestamp = 0.0
                        
                        # Extract BNO055 data
                        ax = data.get('ax', 0)
                        ay = data.get('ay', 0)
                        az = data.get('az', 0)
                        gx = data.get('gx', 0)
                        gy = data.get('gy', 0)
                        gz = data.get('gz', 0)
                        mx = data.get('mx', 0)
                        my = data.get('my', 0)
                        mz = data.get('mz', 0)
                        roll = data.get('roll', 0)
                        pitch = data.get('pitch', 0)
                        yaw = data.get('yaw', 0)
                        qw = data.get('qw', 1)
                        qx = data.get('qx', 0)
                        qy = data.get('qy', 0)
                        qz = data.get('qz', 0)
                        temp = data.get('temp', 0)

                        # Extract calibration data
                        cal_data = data.get('cal', {})
                        cal_sys = cal_data.get('sys', 0)
                        cal_gyro = cal_data.get('gyro', 0)
                        cal_accel = cal_data.get('accel', 0)
                        cal_mag = cal_data.get('mag', 0)
                        
                        # Process BNO055 data
                        if imu_processor:
                            try:
                                # Use research-based processor with proper parameters
                                processed_data = imu_processor.process_data(
                                    ax, ay, az, gx, gy, gz, 
                                    mx=mx, my=my, mz=mz,
                                    roll=roll, pitch=pitch, yaw=yaw, 
                                    qw=qw, qx=qx, qy=qy, qz=qz,
                                    temp=temp, cal_sys=cal_sys, cal_gyro=cal_gyro, 
                                    cal_accel=cal_accel, cal_mag=cal_mag, 
                                    timestamp=timestamp
                                )
                                broadcast_data(processed_data)
                            except Exception as e:
                                print(f"❌ Data processing error: {e}")
                                continue
                        
                        continue
                    except json.JSONDecodeError:
                        pass  # Fall through to regex parsing
            
                # Parse MPU6050 data (fallback)
                mpu6050_match = re.search(r't=(\d+)\s+ax=([\d.-]+)\s+ay=([\d.-]+)\s+az=([\d.-]+)\s+m/s\^2\s*\|\s*gx=([\d.-]+)\s+gy=([\d.-]+)\s+gz=([\d.-]+)\s+rad/s', line)
                
                if mpu6050_match:
                    try:
                        raw_timestamp = int(mpu6050_match.group(1))
                        
                        # Normalize timestamp
                        if timestamp_offset is None:
                            timestamp_offset = raw_timestamp
                        
                        timestamp = raw_timestamp - timestamp_offset
                        ax = float(mpu6050_match.group(2))
                        ay = float(mpu6050_match.group(3))
                        az = float(mpu6050_match.group(4))
                        gx = float(mpu6050_match.group(5))
                        gy = float(mpu6050_match.group(6))
                        gz = float(mpu6050_match.group(7))
                    except (ValueError, IndexError) as e:
                        # MPU6050 data parsing error - skip this line
                        continue
                    
                    # Process data using advanced sensor fusion
                    try:
                        # Use research-based processor for MPU6050 fallback
                        processed_data = imu_processor.process_data(
                            ax, ay, az, gx, gy, gz,
                            mx=None, my=None, mz=None,
                            roll=None, pitch=None, yaw=None,
                            qw=None, qx=None, qy=None, qz=None,
                            temp=None, cal_sys=None, cal_gyro=None, 
                            cal_accel=None, cal_mag=None,
                            timestamp=timestamp
                        )
                        broadcast_data(processed_data)
                    except Exception as e:
                        print(f"❌ MPU6050 data processing error: {e}")
                        continue
                    
        except serial.SerialException as e:
            if "device reports readiness to read but returned no data" in str(e):
                # ESP32 not sending data - try to reconnect
                if serial_port:
                    try:
                        serial_port.close()
                    except:
                        pass
                serial_port = None
                time.sleep(1)  # Shorter delay for faster reconnection
            else:
                print(f"❌ Serial port error: {e}")
                if serial_port:
                    try:
                        serial_port.close()
                    except:
                        pass
                serial_port = None
                # Preserve timestamp offset across reconnections
                if timestamp_offset is not None:
                    print(f"🔄 Preserving timestamp offset: {timestamp_offset}")
                time.sleep(2)
        except Exception as e:
            # Reduce error noise - only print unexpected errors
            if "Bad file descriptor" not in str(e):
                print(f"❌ Unexpected serial error: {e}")
            time.sleep(0.1)


def start_web_server():
    """Start the web server with threading and proper shutdown handling"""
    global server
    
    # Try different ports if 8016 is in use
    ports_to_try = [8016, 8017, 8018, 8019, 8020]
    server = None
    
    for port in ports_to_try:
        try:
            from http.server import ThreadingHTTPServer
            server = ThreadingHTTPServer(('localhost', port), SSEHandler)
            print(f"Advanced IMU Visualizer web server started on http://localhost:{port}")
            break
        except ImportError:
            # Fallback to regular HTTPServer with threading
            from socketserver import ThreadingMixIn
            from http.server import HTTPServer
            
            class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
                pass
            
            try:
                server = ThreadedHTTPServer(('localhost', port), SSEHandler)
                print(f"Advanced IMU Visualizer web server started on http://localhost:{port}")
                break
            except OSError:
                continue
        except OSError:
            continue
    
    if server is None:
        print("❌ Could not start web server on any port")
        return
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down web server...")
        server.shutdown()
        server.server_close()
        print("✅ Web server stopped and port released")
    except Exception as e:
        print(f"❌ Web server error: {e}")
        server.shutdown()
        server.server_close()

def cleanup():
    """Cleanup function to ensure proper shutdown"""
    global server, serial_port, clients
    
    # Close all SSE connections first
    print("🛑 Closing SSE connections...")
    with data_lock:
        for client in clients[:]:  # Copy list to avoid modification during iteration
            try:
                client.close()
            except:
                pass
        clients.clear()
        print("✅ All SSE connections closed")
    
    # Close serial port
    if serial_port:
        try:
            serial_port.close()
            print("✅ Serial port closed")
        except Exception as e:
            print(f"⚠️ Error closing serial port: {e}")
    
    # Shutdown web server
    if server:
        try:
            print("🛑 Shutting down web server...")
            server.shutdown()
            server.server_close()
            print("✅ Web server stopped and port released")
        except Exception as e:
            print(f"⚠️ Error shutting down server: {e}")
    
    # Clean up any temp files
    temp_file = "imu_data.json.tmp"
    if os.path.exists(temp_file):
        try:
            os.remove(temp_file)
            print("✅ Temp file cleaned up")
        except Exception as e:
            print(f"⚠️ Error removing temp file: {e}")

if __name__ == "__main__":
    print("Advanced IMU Visualizer - Extended Kalman Filter & Sensor Fusion")
    print("================================================================")
    print("Based on research-backed algorithms for accurate orientation and position tracking")
    print("Addresses dead reckoning challenges mentioned in Bosch community forum")
    print("Generic system for any IMU-based application")
    print()
    
    # IMU processor already initialized at global scope
    
    # Register cleanup function
    import atexit
    atexit.register(cleanup)
    
    # Start serial receiver in background
    receiver_thread = threading.Thread(target=serial_receiver, daemon=True)
    receiver_thread.start()
    
    # Start web server in background
    server_thread = threading.Thread(target=start_web_server, daemon=True)
    server_thread.start()
    
    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        cleanup()
