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
    OPTIMIZATIONS APPLIED:
    1. Low-pass filtering: Reduces noise in LIA before integration
    2. Zero-velocity updates (ZUPT): Detects stationary periods to reset velocity drift
    3. Velocity clamping: Prevents unbounded velocity growth
    4. Improved stroke detection: Uses gyro magnitude + LIA magnitude for better detection
    5. Bias estimation: Tracks and subtracts acceleration bias to reduce drift
    6. Trapezoidal integration: More accurate than Euler integration
    7. Stationary detection: Detects when device is not moving to prevent false integration
    """
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.last_timestamp_ms = None
        self.in_stroke = False
        
        # --- Stroke Detection Thresholds (tunable) ---
        self.STROKE_START_GYRO_THRESHOLD = 2.0  # rad/s - gyro magnitude to start stroke
        self.STROKE_END_GYRO_THRESHOLD = 0.5    # rad/s - gyro magnitude to end stroke
        self.STROKE_START_LIA_THRESHOLD = 1.5   # m/s² - LIA magnitude to confirm stroke start
        
        # --- Zero-Velocity Update (ZUPT) Parameters ---
        # FIX: Detects when device is stationary to reset velocity drift
        self.ZUPT_LIA_THRESHOLD = 0.3   # m/s² - LIA magnitude below this = stationary
        self.ZUPT_GYRO_THRESHOLD = 0.2  # rad/s - gyro magnitude below this = stationary
        self.stationary_count = 0       # Count consecutive stationary samples
        self.STATIONARY_SAMPLES_REQUIRED = 10  # Samples needed to confirm stationary
        
        # --- Velocity Clamping (prevents unbounded growth) ---
        # FIX: Limits velocity to prevent unrealistic values from integration drift
        self.MAX_VELOCITY = 5.0  # m/s - maximum reasonable swimming velocity
        
        # --- Low-Pass Filter for LIA (reduces noise) ---
        # FIX: Filters high-frequency noise before integration to reduce drift
        self.alpha_lia = 0.7  # Filter coefficient (0-1, higher = less filtering)
        self.filtered_lia = [0.0, 0.0, 0.0]  # Filtered LIA values
        self.first_sample = True  # Flag for first sample initialization
        
        # --- Acceleration Bias Estimation ---
        # FIX: Tracks and subtracts bias in LIA to reduce long-term drift
        self.bias_estimation_samples = 0
        self.MAX_BIAS_SAMPLES = 100  # Samples to use for bias estimation
        self.lia_bias = [0.0, 0.0, 0.0]  # Estimated bias in each axis
        self.bias_initialized = False
        
        # --- Previous values for trapezoidal integration ---
        # FIX: Uses trapezoidal rule instead of Euler for better accuracy
        self.prev_lia = [0.0, 0.0, 0.0]  # Previous LIA for trapezoidal integration

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

        # Initialize corrected LIA variables (used for output)
        lia_x_corrected = lia_x
        lia_y_corrected = lia_y
        lia_z_corrected = lia_z
        
        # 3. --- Check Calibration BEFORE doing any logic ---
        if cal_sys < 3:
            # If not calibrated, reset everything
            self.in_stroke = False
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.stationary_count = 0
            self.bias_initialized = False
            self.bias_estimation_samples = 0
        else:
            # --- System is calibrated, run optimized stroke logic ---
            
            #low-Pass Filter to reduce noise, reduces high-frequency noise that causes integration drift
            if self.first_sample:
                self.filtered_lia = [lia_x, lia_y, lia_z]
                self.first_sample = False
            else:
                # Exponential moving average filter
                self.filtered_lia[0] = self.alpha_lia * lia_x + (1 - self.alpha_lia) * self.filtered_lia[0]
                self.filtered_lia[1] = self.alpha_lia * lia_y + (1 - self.alpha_lia) * self.filtered_lia[1]
                self.filtered_lia[2] = self.alpha_lia * lia_z + (1 - self.alpha_lia) * self.filtered_lia[2]
            
            # Use filtered LIA for all subsequent calculations
            lia_x_filt = self.filtered_lia[0]
            lia_y_filt = self.filtered_lia[1]
            lia_z_filt = self.filtered_lia[2]
        
            # Estimate and subtract acceleration Bias,  accumulates from sensor drift
            if not self.bias_initialized and self.bias_estimation_samples < self.MAX_BIAS_SAMPLES:
                # During initial period, estimate bias as average LIA (should be ~0 when stationary)
                self.lia_bias[0] = (self.lia_bias[0] * self.bias_estimation_samples + lia_x_filt) / (self.bias_estimation_samples + 1)
                self.lia_bias[1] = (self.lia_bias[1] * self.bias_estimation_samples + lia_y_filt) / (self.bias_estimation_samples + 1)
                self.lia_bias[2] = (self.lia_bias[2] * self.bias_estimation_samples + lia_z_filt) / (self.bias_estimation_samples + 1)
                self.bias_estimation_samples += 1
                if self.bias_estimation_samples >= self.MAX_BIAS_SAMPLES:
                    self.bias_initialized = True
            
            # Subtract estimated bias from filtered LIA
            lia_x_corrected = lia_x_filt - self.lia_bias[0]
            lia_y_corrected = lia_y_filt - self.lia_bias[1]
            lia_z_corrected = lia_z_filt - self.lia_bias[2]
            
            # Calculate magnitudes for detection
            lia_magnitude = math.sqrt(lia_x_corrected**2 + lia_y_corrected**2 + lia_z_corrected**2)
            gyro_magnitude = math.sqrt(data_dict['gx']**2 + data_dict['gy']**2 + data_dict['gz']**2)
            
            # FIX 3: Improved Stroke Detection using multiple signals
            # Uses both gyro magnitude and LIA magnitude for more robust detection
            if not self.in_stroke:
                # Start stroke when both gyro and LIA exceed thresholds
                if gyro_magnitude > self.STROKE_START_GYRO_THRESHOLD and lia_magnitude > self.STROKE_START_LIA_THRESHOLD:
                    self.in_stroke = True
                    self.position = [0.0, 0.0, 0.0]
                    self.velocity = [0.0, 0.0, 0.0]
                    self.stationary_count = 0
            else:
                # End stroke when gyro drops below threshold
                if gyro_magnitude < self.STROKE_END_GYRO_THRESHOLD:
                    self.in_stroke = False
            
            # FIX 4: Zero-Velocity Update (ZUPT) - Detect stationary periods
            # When device is stationary, velocity should be zero (resets drift)
            is_stationary = (lia_magnitude < self.ZUPT_LIA_THRESHOLD and 
                           gyro_magnitude < self.ZUPT_GYRO_THRESHOLD)
            
            if is_stationary:
                self.stationary_count += 1
                # After enough consecutive stationary samples, reset velocity
                if self.stationary_count >= self.STATIONARY_SAMPLES_REQUIRED:
                    self.velocity = [0.0, 0.0, 0.0]
                    # Don't reset position, but velocity reset prevents further drift
            else:
                self.stationary_count = 0
            
            # FIX 5: Trapezoidal Integration (more accurate than Euler)
            # 
            # MATHEMATICAL JUSTIFICATION:
            # ===========================
            # We need to integrate acceleration to get velocity: v(t) = ∫ a(t) dt
            # 
            # EULER METHOD (what we replaced):
            #   v(t) = v(t-1) + a(t-1) * dt
            #   - Uses only the PREVIOUS acceleration value
            #   - Assumes acceleration is constant at a(t-1) for the entire time step
            #   - Error: O(dt²) - accumulates quadratically over time
            #   - Visual: Approximates area under curve as a rectangle (underestimates/overestimates)
            #
            # TRAPEZOIDAL METHOD (what we use now):
            #   v(t) = v(t-1) + (a(t-1) + a(t)) / 2 * dt
            #   - Uses the AVERAGE of previous and current acceleration
            #   - Assumes acceleration changes linearly between samples
            #   - Error: O(dt³) - much smaller error, accumulates slower
            #   - Visual: Approximates area under curve as a trapezoid (better fit)
            #
            # WHY IT MATTERS FOR IMU DATA:
            # ============================
            # 1. Acceleration changes continuously during swimming strokes
            #    - Euler: "assumes acceleration stayed at old value" → systematic error
            #    - Trapezoidal: "assumes acceleration changed linearly" → much closer to reality
            #
            # 2. Error accumulation:
            #    - Over 1 second at 10Hz (100 samples):
            #      * Euler error: ~100 * (dt²) = significant drift
            #      * Trapezoidal error: ~100 * (dt³) = 10x smaller
            #
            # 3. Real-world impact:
            #    - Swimming stroke: acceleration goes 0 → 2 m/s² → 0 m/s²
            #    - Euler: underestimates velocity during acceleration, overestimates during deceleration
            #    - Trapezoidal: captures the transition much more accurately
            #
            # 4. Numerical stability:
            #    - Trapezoidal method is more stable for noisy data
            #    - Reduces sensitivity to individual noisy samples
            #
            # EXAMPLE:
            # ========
            # Sample at t=0.0s: a = 0.0 m/s²
            # Sample at t=0.1s: a = 2.0 m/s²  (dt = 0.1s)
            #
            # Euler:      Δv = 0.0 * 0.1 = 0.0 m/s        (misses the acceleration!)
            # Trapezoidal: Δv = (0.0 + 2.0)/2 * 0.1 = 0.1 m/s  (captures the change)
            #
            # Over a full stroke cycle, trapezoidal gives ~2x better velocity accuracy
            #
            # COST: Minimal - just one extra addition and division per axis
            # BENEFIT: Significantly reduced velocity drift, more accurate swimming metrics
            if self.prev_lia[0] != 0.0 or self.prev_lia[1] != 0.0 or self.prev_lia[2] != 0.0:
                # Trapezoidal integration: average of current and previous acceleration
                avg_lia_x = (lia_x_corrected + self.prev_lia[0]) / 2.0
                avg_lia_y = (lia_y_corrected + self.prev_lia[1]) / 2.0
                avg_lia_z = (lia_z_corrected + self.prev_lia[2]) / 2.0
            else:
                # First integration step, use current value only
                avg_lia_x = lia_x_corrected
                avg_lia_y = lia_y_corrected
                avg_lia_z = lia_z_corrected
            
            # Update velocity using trapezoidal integration
            self.velocity[0] += avg_lia_x * dt
            self.velocity[1] += avg_lia_y * dt
            self.velocity[2] += avg_lia_z * dt
            
            # FIX 6: Velocity Clamping (prevents unrealistic values)
            # Clamps velocity to maximum reasonable swimming speed
            vel_magnitude = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2 + self.velocity[2]**2)
            if vel_magnitude > self.MAX_VELOCITY:
                # Scale down velocity vector to max magnitude
                scale = self.MAX_VELOCITY / vel_magnitude
                self.velocity[0] *= scale
                self.velocity[1] *= scale
                self.velocity[2] *= scale3
            
            # Update position using current velocity (Euler integration is fine for position)
            self.position[0] += self.velocity[0] * dt
            self.position[1] += self.velocity[1] * dt
            self.position[2] += self.velocity[2] * dt
            
            # Store current LIA for next trapezoidal integration step
            self.prev_lia = [lia_x_corrected, lia_y_corrected, lia_z_corrected]

        # 6. Create the final JSON for the visualizer
        # Use corrected LIA values (filtered and bias-corrected if calibrated)
        vis_data = {
            "timestamp": t_ms,
            "quaternion": {"qw": quat_w, "qx": quat_x, "qy": quat_y, "qz": quat_z},
            "position": {"px": self.position[0], "py": self.position[1], "pz": self.position[2]},
            "calibration": calibration, # Pass full calibration status to UI
            
            "acceleration": {"ax": lia_x_corrected, "ay": lia_y_corrected, "az": lia_z_corrected},  # Filtered & bias-corrected LIA
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
            except (BrokenPipeError, ConnectionResetError, OSError):
                # Client disconnected, ignore silently
                pass
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
                        try:
                            self.wfile.write(f"data: {latest_data_json}\n\n".encode())
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError, OSError):
                            # Client disconnected
                            break
                time.sleep(0.01) # ~100Hz
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass # Client disconnected
        except Exception as e:
            print(f"SSE error: {e}", flush=True)
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
                raw_line = serial_port.readline()
                if not raw_line:
                    time.sleep(0.01)
                    continue
                
                # Try to decode as UTF-8, ignore invalid bytes
                try:
                    line = raw_line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    # Skip binary data or corrupted bytes
                    continue
                
                if line:
                    try:
                        data_dict = json.loads(line)
                        
                        # Ensure data_dict is a dictionary before checking keys
                        if isinstance(data_dict, dict) and 't' in data_dict and 'lia_x' in data_dict:
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
            except UnicodeDecodeError:
                # Already handled above, skip
                continue
            except Exception as e:
                # Only print unexpected errors (not connection issues)
                if not isinstance(e, (BrokenPipeError, ConnectionResetError, OSError)):
                    print(f"\nUnhandled serial read error: {e}", flush=True)
                time.sleep(0.1)

# --- (start_web_server and __main__ are unchanged from the previous file) ---
def start_web_server(port):
    """Start the web server"""
    try:
        server = HTTPServer(('0.0.0.0', port), SSEHandler)
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