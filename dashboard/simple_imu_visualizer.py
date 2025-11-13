#!/usr/bin/env python3
"""
IMU Data Visualizer & Calibration Helper

Reads IMU data from ESP32, and provides clear, simple
feedback in the terminal to help with sensor calibration.
"""

import serial
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import sys
import os
import re
import math

# --- Global variables ---
latest_data_json = None
data_lock = threading.Lock()
clients = []
serial_port = None
last_cal_print_time = 0
last_cal_status = {}

# --- This class contains your new logic ---
class StrokeProcessor:
    """
    Holds the state for velocity and position integration
    and resets it based on stroke detection.
    """
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.last_timestamp_ms = None
        self.in_stroke = False
        
        # --- You must tune these values! ---
        self.STROKE_START_GYRO_THRESHOLD = 2.0  # rad/s
        self.STROKE_END_GYRO_THRESHOLD = 0.5    # rad/s

    def process_data(self, data_dict):
        """
        Processes a full data dictionary from the sensor.
        `data_dict` should be the parsed JSON from the ESP32.
        """
        
        # 1. Get timestamp and calculate delta-time (dt)
        t_ms = data_dict['t']
        if self.last_timestamp_ms is None:
            self.last_timestamp_ms = t_ms
            return None
        
        dt = (t_ms - self.last_timestamp_ms) / 1000.0  # dt in seconds
        self.last_timestamp_ms = t_ms
        
        if dt <= 0 or dt > 0.5:
            self.last_timestamp_ms = t_ms 
            return None

        # 2. Get necessary data from the JSON
        try:
            lia_x = data_dict['lia_x']
            lia_y = data_dict['lia_y']
            lia_z = data_dict['lia_z']
            
            quat_w = data_dict['qw']
            quat_x = data_dict['qx']
            quat_y = data_dict['qy']
            quat_z = data_dict['qz']

            gyro_y = data_dict['gy'] 
            
            # Get calibration status
            calibration = data_dict['cal']
            cal_sys = calibration['sys']

        except KeyError as e:
            # This will fire if you haven't updated your C code!
            print(f"ERROR: Missing key {e}. Did you update your C code to send LIA data?")
            time.sleep(1)
            return None

        # 3. --- Check Calibration BEFORE doing any logic ---
        if cal_sys < 3:
            # If not calibrated, reset everything
            self.in_stroke = False
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
        else:
            # --- System is calibrated, run stroke logic ---
            # 4. Stroke Detection Logic
            if not self.in_stroke:
                if gyro_y > self.STROKE_START_GYRO_THRESHOLD:
                    self.in_stroke = True
                    self.position = [0.0, 0.0, 0.0]
                    self.velocity = [0.0, 0.0, 0.0]
            else:
                if abs(gyro_y) < self.STROKE_END_GYRO_THRESHOLD:
                    self.in_stroke = False
            
            # 5. Integration Logic (Only runs if calibrated)
            self.velocity[0] += lia_x * dt
            self.velocity[1] += lia_y * dt
            self.velocity[2] += lia_z * dt
            
            self.position[0] += self.velocity[0] * dt
            self.position[1] += self.velocity[1] * dt
            self.position[2] += self.velocity[2] * dt

        # 6. Create the final JSON for the visualizer
        vis_data = {
            "timestamp": t_ms,
            "quaternion": {"qw": quat_w, "qx": quat_x, "qy": quat_y, "qz": quat_z},
            "position": {"px": self.position[0], "py": self.position[1], "pz": self.position[2]},
            "calibration": calibration, # Pass full calibration status to UI
            
            "acceleration": {"ax": data_dict['ax'], "ay": data_dict['ay'], "az": data_dict['az']},
            "angular_velocity": {"gx": data_dict['gx'], "gy": data_dict['gy'], "gz": data_dict['gz']}
        }
        
        return json.dumps(vis_data)


# --- HTTP Server (Unchanged) ---
class SSEHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/events':
            self.handle_sse()
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), 'simple_imu_3d.html')
            try:
                with open(html_path, 'r') as f:
                    self.wfile.write(f.read().encode())
            except FileNotFoundError:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"Error: 'simple_imu_3d.html' not found in this directory.")
                print("ERROR: 'simple_imu_3d.html' not found.", flush=True)
        else:
            self.send_response(404)
            self.end_headers()
    
    def handle_sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        
        clients.append(self)
        print(f"Client connected. Total clients: {len(clients)}", flush=True)
        
        try:
            while True:
                with data_lock:
                    if latest_data_json:
                        self.wfile.write(f"data: {latest_data_json}\n\n".encode())
                        self.wfile.flush()
                time.sleep(0.01) # ~100Hz
        except:
            pass # Client disconnected
        finally:
            if self in clients:
                clients.remove(self)
            print(f"Client disconnected. Total clients: {len(clients)}", flush=True)

def broadcast_data(json_data):
    if json_data is None:
        return
    with data_lock:
        global latest_data_json
        latest_data_json = json_data

# --- Serial Receiver (MODIFIED to be quiet) ---
def serial_receiver(processor: StrokeProcessor):
    global serial_port, last_cal_print_time, last_cal_status
    
    possible_ports = ['/dev/cu.usbserial-0001', '/dev/cu.usbserial-0002', '/dev/cu.usbserial-0003'] # macOS
    if sys.platform.startswith('linux'):
        possible_ports = [f'/dev/ttyUSB{i}' for i in range(4)]
    elif sys.platform.startswith('win'):
        possible_ports = [f'COM{i}' for i in range(1, 10)]

    while True: 
        if serial_port is None:
            print("Attempting to connect to ESP32...", flush=True)
            for port in possible_ports:
                try:
                    if sys.platform.startswith('win') or os.path.exists(port):
                        serial_port = serial.Serial(port, 115200, timeout=1)
                        print(f"Connected to ESP32 on {port}", flush=True)
                        break
                except serial.SerialException as e:
                    print(f"Port {port} busy or unavailable: {e}", flush=True)
                except Exception as e:
                    print(f"Failed to connect to {port}: {e}", flush=True)
            
            if serial_port is None:
                print("ERROR: Could not find ESP32. Retrying in 5 seconds.", flush=True)
                time.sleep(5)
                continue
    
        # If we are here, we have a serial_port connection
        print("Reading IMU data from ESP32...")
        print("--- WAITING FOR CALIBRATION ---")
        print("Please move the sensor to calibrate it (see instructions).")
        
        while True:
            try:
                line = serial_port.readline().decode('utf-8').strip()
                
                if line:
                    try:
                        data_dict = json.loads(line)
                        
                        if 't' in data_dict and 'lia_x' in data_dict:
                            # Process the data (integrates, checks cal, etc.)
                            json_to_broadcast = processor.process_data(data_dict)
                            if json_to_broadcast:
                                broadcast_data(json_to_broadcast)
                            
                            # --- THIS IS THE NEW "SLOW" PRINTING LOGIC ---
                            current_time = time.time()
                            cal_status = data_dict.get('cal', {})
                            
                            # Print an update only once per second OR if status changes
                            if (current_time - last_cal_print_time > 1.0) or (cal_status != last_cal_status):
                                cal_sys = cal_status.get('sys', 0)
                                cal_gyro = cal_status.get('gyro', 0)
                                cal_accel = cal_status.get('accel', 0)
                                cal_mag = cal_status.get('mag', 0)
                                
                                # Use print() with carriage return to update in-place
                                print(f"CALIBRATION STATUS: SYS={cal_sys} | GYRO={cal_gyro} | ACCEL={cal_accel} | MAG={cal_mag}    ", end='\r', flush=True)
                                
                                last_cal_print_time = current_time
                                last_cal_status = cal_status
                                
                                if cal_sys == 3:
                                    print("\n--- SYSTEM FULLY CALIBRATED! ---", flush=True)

                    except json.JSONDecodeError:
                        pass # Ignore non-JSON lines
                    
            except serial.SerialException as e:
                print(f"\nSerial read error (device disconnected?): {e}", flush=True)
                serial_port.close()
                serial_port = None
                print("Connection lost. Will attempt to reconnect...", flush=True)
                last_cal_status = {} # Reset cal status
                break 
            except Exception as e:
                print(f"\nUnhandled serial read error: {e}", flush=True)
                time.sleep(0.1)

# --- (start_web_server and __main__ are unchanged from the previous file) ---
def start_web_server(port):
    """Start the web server"""
    try:
        server = HTTPServer(('localhost', port), SSEHandler)
        print(f"Web server started on http://localhost:{port}", flush=True)
        server.serve_forever()
    except OSError as e:
        print(f"\nERROR: Could not start web server on port {port}. Is it already in use?")
        print(e)
        sys.exit(1)

if __name__ == "__main__":
    print("ESP32 IMU 3D Visualizer - Stroke Integration Version")
    print("===================================================", flush=True)
    
    default_port = 8003
    port = int(os.environ.get("PORT", default_port))
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port argument '{sys.argv[1]}', using port {port}", flush=True)
    
    stroke_processor = StrokeProcessor()
    
    receiver_thread = threading.Thread(target=serial_receiver, args=(stroke_processor,), daemon=True)
    receiver_thread.start()
    
    start_web_server(port)