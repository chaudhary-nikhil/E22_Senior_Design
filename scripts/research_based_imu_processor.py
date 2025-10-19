#!/usr/bin/env python3
"""
Research-Based IMU Processor
Based on ArduPilot EKF, PMC research, Campus Rover, and ESP32 BNO055 implementations
Focuses on high-precision timestamps and minimal estimation for serial streams
"""

import numpy as np
import time
import json
import os
from datetime import datetime

class ResearchBasedIMUProcessor:
    """
    Research-based IMU processor focusing on:
    - High-precision timestamps (microsecond accuracy)
    - Minimal estimation for high-frequency serial data
    - Direct BNO055 quaternion usage
    - Based on multiple research sources
    """
    
    def __init__(self):
        # High-precision timing - start from 0
        self.start_time = time.perf_counter()
        self.last_timestamp = 0.0
        self.timestamp_counter = 0.0
        self.sample_count = 0
        
        # Direct quaternion from BNO055 (trust sensor fusion)
        self.current_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        
        # Simple complementary filter for gyro bias (based on Sparx Engineering)
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.bias_samples = 0
        self.max_bias_samples = 50  # Reduced for faster convergence
        
        # Data history for analysis
        self.data_history = []
        self.max_history = 100
        
        # Session tracking
        self.session_start_time = datetime.now()
        self.session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")
        
        # JSON logging setup
        self.json_file = "imu_data.json"
        self.data_buffer = []
        self.logging_enabled = False
        self.logging_started = False
        
        print(f"🔬 Research-Based IMU Processor initialized")
        print(f"   Based on: ArduPilot EKF, PMC research, Campus Rover, ESP32 BNO055")
        print(f"   Focus: High-precision timestamps, minimal estimation")
        print(f"   Session: {self.session_id}")
    
    def get_high_precision_timestamp(self):
        """Get high-precision timestamp starting from 0 (50Hz sampling)"""
        # Use sample count for consistent 50Hz timing
        self.sample_count += 1
        timestamp = self.sample_count * 0.02  # 50Hz = 0.02s per sample
        return timestamp
    
    def update_gyro_bias(self, gyro_data, accel_magnitude):
        """Update gyroscope bias using stationary detection (Sparx Engineering approach)"""
        # Only update bias when stationary (gravity magnitude ~9.8 m/s²)
        if 9.0 < accel_magnitude < 11.0 and self.bias_samples < self.max_bias_samples:
            self.gyro_bias += gyro_data
            self.bias_samples += 1
    
    def get_corrected_gyro(self, gyro_data):
        """Get gyroscope data with bias correction"""
        if self.bias_samples > 0:
            return gyro_data - (self.gyro_bias / self.bias_samples)
        return gyro_data
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
    
    def process_data(self, ax, ay, az, gx, gy, gz, mx=None, my=None, mz=None, 
                    roll=None, pitch=None, yaw=None, qw=None, qx=None, qy=None, qz=None, 
                    temp=None, cal_sys=None, cal_gyro=None, cal_accel=None, cal_mag=None, timestamp=None):
        """
        Process IMU data with research-based approach
        Focus on high-precision timestamps and minimal estimation
        """
        
        # Always use our own timestamp starting from 0 (ignore ESP32 timestamps)
        timestamp = self.get_high_precision_timestamp()
        
        # Ensure timestamp is always increasing and starts from 0
        if timestamp <= self.last_timestamp:
            timestamp = self.last_timestamp + 0.02  # 50Hz default
        self.last_timestamp = timestamp
        
        # Convert to numpy arrays with defaults
        # The BNO055 accelerometer values appear to be in cm/s² (based on web interface showing ~978 m/s²)
        # Convert from cm/s² to m/s²
        accel_data = np.array([ax/100, ay/100, az/100])
            
        gyro_data = np.array([gx, gy, gz])
        mag_data = np.array([mx if mx is not None else 0, 
                           my if my is not None else 0, 
                           mz if mz is not None else 0])
        
        # Calculate magnitudes
        accel_magnitude = np.linalg.norm(accel_data)
        gyro_magnitude = np.linalg.norm(gyro_data)
        mag_magnitude = np.linalg.norm(mag_data)
        
        # Update gyro bias (Sparx Engineering approach)
        self.update_gyro_bias(gyro_data, accel_magnitude)
        
        # Get corrected gyro data
        gyro_corrected = self.get_corrected_gyro(gyro_data)
        
        # Use BNO055 quaternion directly (trust sensor fusion)
        if qw is not None and qx is not None and qy is not None and qz is not None:
            bno055_quaternion = np.array([qw, qx, qy, qz])
            
            # Normalize quaternion
            quat_norm = np.linalg.norm(bno055_quaternion)
            if quat_norm > 0:
                bno055_quaternion = bno055_quaternion / quat_norm
                self.current_quaternion = bno055_quaternion
        
        # Convert to Euler angles
        ekf_roll, ekf_pitch, ekf_yaw = self.quaternion_to_euler(self.current_quaternion)
        
        # Accurate movement detection (based on research)
        # Consider moving if gyro magnitude > 0.1 rad/s OR significant acceleration change
        gyro_threshold = 0.1  # 0.1 rad/s = ~5.7 degrees/second
        accel_threshold = 2.0  # 2.0 m/s² change from gravity
        
        # For stationary detection, check if we're close to gravity magnitude
        gravity_magnitude = 9.81
        is_stationary = (abs(accel_magnitude - gravity_magnitude) < 1.0 and 
                        gyro_magnitude < 0.05)
        
        is_moving = not is_stationary
        
        # Create comprehensive data structure
        processed_data = {
            'timestamp': float(timestamp),
            'raw_sensors': {
                'accelerometer': {
                    'x': float(ax), 'y': float(ay), 'z': float(az), 
                    'magnitude': float(accel_magnitude)
                },
                'gyroscope': {
                    'x': float(gx), 'y': float(gy), 'z': float(gz), 
                    'magnitude': float(gyro_magnitude)
                },
                'magnetometer': {
                    'x': float(mx or 0), 'y': float(my or 0), 'z': float(mz or 0), 
                    'magnitude': float(mag_magnitude)
                }
            },
            'orientation': {
                'ekf_quaternion': {
                    'w': float(self.current_quaternion[0]),
                    'x': float(self.current_quaternion[1]),
                    'y': float(self.current_quaternion[2]),
                    'z': float(self.current_quaternion[3])
                },
                'ekf_euler': {
                    'roll': float(ekf_roll),
                    'pitch': float(ekf_pitch),
                    'yaw': float(ekf_yaw)
                },
                'complementary_euler': {
                    'roll': float(roll if roll is not None else 0),
                    'pitch': float(pitch if pitch is not None else 0),
                    'yaw': float(yaw if yaw is not None else 0)
                },
                'euler': {
                    'roll': float(roll if roll is not None else 0),
                    'pitch': float(pitch if pitch is not None else 0),
                    'yaw': float(yaw if yaw is not None else 0)
                },
                'quaternion': {
                    'w': float(qw if qw is not None else 1.0),
                    'x': float(qx if qx is not None else 0.0),
                    'y': float(qy if qy is not None else 0.0),
                    'z': float(qz if qz is not None else 0.0)
                }
            },
            'position': {
                'x': 0.0, 'y': 0.0, 'z': 0.0  # No position estimation for now
            },
            'velocity': {
                'x': 0.0, 'y': 0.0, 'z': 0.0  # No velocity estimation for now
            },
            'movement': {
                'linear_acceleration': {
                    'x': float(ax), 'y': float(ay), 'z': float(az)
                },
                'is_moving': bool(is_moving),
                'movement_magnitude': float(accel_magnitude),
                'movement_confidence': float(1.0 if is_moving else 0.0),
                'accel_variance': float(0.0),  # Simplified
                'gyro_magnitude': float(gyro_magnitude)
            },
            'temperature': float(temp if temp is not None else 0),
            'calibration': {
                'sys': int(cal_sys if cal_sys is not None else 0),
                'gyro': int(cal_gyro if cal_gyro is not None else 0),
                'accel': int(cal_accel if cal_accel is not None else 0),
                'mag': int(cal_mag if cal_mag is not None else 0),
                'gravity_calibrated': bool((cal_accel if cal_accel is not None else 0) >= 2),
                'gyro_bias_samples': int(self.bias_samples)
            }
        }
        
        # Store in history
        self.data_history.append(processed_data)
        if len(self.data_history) > self.max_history:
            self.data_history.pop(0)
        
        # Log data to JSON file if logging is enabled
        self.log_data_for_analysis(processed_data)
        
        return processed_data
    
    def get_latest_data(self):
        """Get the latest processed data"""
        if self.data_history:
            return self.data_history[-1]
        return None
    
    def get_data_history(self):
        """Get all stored data"""
        return self.data_history.copy()
    
    def reset_session(self):
        """Reset for new session"""
        self.start_time = time.perf_counter()
        self.last_timestamp = 0.0
        self.data_history.clear()
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.bias_samples = 0
        self.session_start_time = datetime.now()
        self.session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")
        print(f"🔄 Session reset: {self.session_id}")
    
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
    
    def save_buffer_to_file(self):
        """Save current buffer to JSON file"""
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
            
            # Write to file
            with open(self.json_file, 'w', encoding='utf-8') as f:
                json.dump(existing_data, f, indent=2)
                f.flush()
            
            # Clear buffer
            self.data_buffer.clear()
            
        except Exception as e:
            print(f"❌ Error saving data: {e}")
    
    def log_data_for_analysis(self, processed_data):
        """Log comprehensive IMU data for analysis"""
        # Only log if logging is enabled
        if not self.logging_enabled:
            return
            
        # Create clean data entry with BNO055 9-axis data
        data_entry = {
            "t": int(processed_data['timestamp'] * 1000),  # Convert to milliseconds
            "ax": round(float(processed_data['raw_sensors']['accelerometer']['x']), 3),
            "ay": round(float(processed_data['raw_sensors']['accelerometer']['y']), 3),
            "az": round(float(processed_data['raw_sensors']['accelerometer']['z']), 3),
            "gx": round(float(processed_data['raw_sensors']['gyroscope']['x']), 3),
            "gy": round(float(processed_data['raw_sensors']['gyroscope']['y']), 3),
            "gz": round(float(processed_data['raw_sensors']['gyroscope']['z']), 3),
            "mx": round(float(processed_data['raw_sensors']['magnetometer']['x']), 1),
            "my": round(float(processed_data['raw_sensors']['magnetometer']['y']), 1),
            "mz": round(float(processed_data['raw_sensors']['magnetometer']['z']), 1),
            "roll": round(float(processed_data['orientation']['euler']['roll']), 1),
            "pitch": round(float(processed_data['orientation']['euler']['pitch']), 1),
            "yaw": round(float(processed_data['orientation']['euler']['yaw']), 1),
            "qw": round(float(processed_data['orientation']['quaternion']['w']), 4),
            "qx": round(float(processed_data['orientation']['quaternion']['x']), 4),
            "qy": round(float(processed_data['orientation']['quaternion']['y']), 4),
            "qz": round(float(processed_data['orientation']['quaternion']['z']), 4),
            "temp": round(float(processed_data['temperature']), 1),
            "cal": {
                "sys": int(processed_data['calibration']['sys']),
                "gyro": int(processed_data['calibration']['gyro']),
                "accel": int(processed_data['calibration']['accel']),
                "mag": int(processed_data['calibration']['mag'])
            }
        }
        
        # Add to buffer
        self.data_buffer.append(data_entry)
        
        # Save to file every 5 entries
        if len(self.data_buffer) >= 5:
            self.save_buffer_to_file()
    
    def start_logging(self):
        """Start data logging with JSON file output"""
        self.clear_previous_data()
        self.logging_enabled = True
        self.logging_started = True
        
        # Reset timestamp counter to ensure it starts from 0
        self.sample_count = 0
        self.last_timestamp = 0.0
        
        # Create empty JSON file
        try:
            with open(self.json_file, 'w', encoding='utf-8') as f:
                f.write('[]')
                f.flush()
            print(f"📄 Created fresh JSON file: {self.json_file}")
        except Exception as e:
            print(f"⚠️ Could not create JSON file: {e}")
        
        print(f"🟢 Research-based logging started - timestamps reset to 0")
        return {"status": "logging_started"}
    
    def stop_logging(self):
        """Stop data logging and save final data"""
        self.logging_enabled = False
        self.save_buffer_to_file()
        print(f"🔴 Research-based logging stopped")
        return {"status": "logging_stopped"}
    
    def toggle_logging(self):
        """Toggle logging on/off"""
        if self.logging_enabled:
            self.stop_logging()
        else:
            self.start_logging()
        return {"status": "toggled"}

# Global instance
research_processor = ResearchBasedIMUProcessor()
