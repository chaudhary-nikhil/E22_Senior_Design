#!/usr/bin/env python3
"""
Simple IMU Data Visualizer
Reads IMU data from ESP32 serial output and displays it in real-time 3D visualization
"""

import serial
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
import sys
import os
import re
import math

# Global variables
latest_data = None
data_lock = threading.Lock()
clients = []
serial_port = None

class SSEHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/events':
            self.handle_sse()
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('simple_imu_3d.html', 'r') as f:
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
                time.sleep(0.01)  # 100Hz
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
        except:
            clients.remove(client)

def process_imu_data(ax, ay, az, gx, gy, gz, timestamp):
    """Process raw IMU data and create visualization data"""
    
    # Simple position integration (for demo purposes)
    # In a real system, you'd use proper sensor fusion
    dt = 0.01  # 100Hz sampling
    
    # Calculate simple position from acceleration (double integration)
    # This is just for visualization - not accurate for real motion
    static_position = {
        "px": ax * 0.1,  # Scale acceleration to position
        "py": ay * 0.1,
        "pz": az * 0.1
    }
    
    # Calculate velocity from acceleration (single integration)
    static_velocity = {
        "vx": ax * dt,
        "vy": ay * dt,
        "vz": az * dt
    }
    
    # Simple quaternion from angular velocity (for demo)
    # In reality, you'd use proper quaternion integration
    angle_magnitude = math.sqrt(gx*gx + gy*gy + gz*gz)
    if angle_magnitude > 0.001:  # Avoid division by zero
        angle = angle_magnitude * dt
        sin_half = math.sin(angle / 2)
        cos_half = math.cos(angle / 2)
        
        quaternion = {
            "qw": cos_half,
            "qx": (gx / angle_magnitude) * sin_half,
            "qy": (gy / angle_magnitude) * sin_half,
            "qz": (gz / angle_magnitude) * sin_half
        }
    else:
        quaternion = {"qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}
    
    # Create the data structure for visualization
    data = {
        "acceleration": {"ax": ax, "ay": ay, "az": az},
        "angular_velocity": {"gx": gx, "gy": gy, "gz": gz},
        "position": static_position,
        "velocity": static_velocity,
        "quaternion": quaternion,
        "timestamp": timestamp
    }
    
    return json.dumps(data)

def serial_receiver():
    """Read data from ESP32 serial port"""
    global serial_port
    
    # Try to find the ESP32 serial port
    possible_ports = ['/dev/cu.usbserial-0001', '/dev/cu.usbserial-0002', '/dev/cu.usbserial-0003']
    
    for port in possible_ports:
        try:
            if os.path.exists(port):
                serial_port = serial.Serial(port, 115200, timeout=1)
                print(f"Connected to ESP32 on {port}")
                break
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            continue
    
    if serial_port is None:
        print("ERROR: Could not find ESP32 serial port!")
        print("Make sure ESP32 is connected and try again.")
        return
    
    print("Reading IMU data from ESP32...")
    
    while True:
        try:
            # Read line from serial
            line = serial_port.readline().decode('utf-8').strip()
            
            if line:
                # Try to parse as JSON first (new format)
                try:
                    data = json.loads(line)
                    if 't' in data and 'ax' in data and 'gx' in data:
                        # New JSON format
                        timestamp = data['t']
                        ax = data['ax']
                        ay = data['ay']
                        az = data['az']
                        gx = data['gx']
                        gy = data['gy']
                        gz = data['gz']
                        
                        # Process the IMU data
                        json_data = process_imu_data(ax, ay, az, gx, gy, gz, timestamp)
                        broadcast_data(json_data)
                        print(f"Processed JSON IMU: t={timestamp}, ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}")
                        continue
                except json.JSONDecodeError:
                    pass
                
                # Look for IMU data in the old format: t=XXXXX ax=X.XXX ay=X.XXX az=X.XXX | gx=X.XXX gy=X.XXX gz=X.XXX
                imu_match = re.search(r't=(\d+)\s+ax=([\d.-]+)\s+ay=([\d.-]+)\s+az=([\d.-]+)\s+m/s\^2\s+\|\s+gx=([\d.-]+)\s+gy=([\d.-]+)\s+gz=([\d.-]+)\s+rad/s', line)
                
                if imu_match:
                    timestamp = int(imu_match.group(1))
                    ax = float(imu_match.group(2))
                    ay = float(imu_match.group(3))
                    az = float(imu_match.group(4))
                    gx = float(imu_match.group(5))
                    gy = float(imu_match.group(6))
                    gz = float(imu_match.group(7))
                    
                    # Process the IMU data
                    json_data = process_imu_data(ax, ay, az, gx, gy, gz, timestamp)
                    broadcast_data(json_data)
                    print(f"Processed IMU: t={timestamp}, ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}")
                    
        except Exception as e:
            print(f"Serial read error: {e}")
            time.sleep(0.1)

def start_web_server():
    """Start the web server"""
    server = HTTPServer(('localhost', 8003), SSEHandler)
    print("Web server started on http://localhost:8003")
    server.serve_forever()

if __name__ == "__main__":
    print("ESP32 Simple IMU 3D Visualizer")
    print("===============================")
    
    # Start serial receiver in background
    receiver_thread = threading.Thread(target=serial_receiver, daemon=True)
    receiver_thread.start()
    
    # Start web server
    start_web_server()
