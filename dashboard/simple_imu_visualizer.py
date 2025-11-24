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
stroke_processor_instance = None  # Will be set in main

class SimpleKalmanFilter:
    """
    A simplified 1D Kalman filter implementation for single-value signals (like acceleration).
    Math:
        x_est = x_pred + K * (meas - x_pred)
    """
    def __init__(self, process_noise, measurement_noise, estimation_error, initial_value):
        self.Q = process_noise # Process noise covariance
        self.R = measurement_noise # Measurement noise covariance
        self.P = estimation_error # Estimation error covariance
        self.X = initial_value # Value state estimate (x)

    def update(self, measurement):
        # Prediction update (assume constant state)
        self.P = self.P + self.Q

        # Measurement update
        K = self.P / (self.P + self.R)
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        
        return self.X

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
        
        # Kalman Filters for Acceleration (X, Y, Z)
        # TUNING for HUMAN MOTION:
        # Q (Process Noise): 0.3 -> High because human motion changes accel rapidly.
        # R (Measurement Noise): 0.1 -> Low because BNO055 is relatively precise.
        # This reduces "lag" so we catch the start of the stroke instantly.
        self.kalman_ax = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_ay = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_az = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        
        # --- You must tune these values! ---
        # SWIMMING TUNING:
        # - Start: Sensitive enough to catch the catch/pull.
        # - End: "Loose" enough to recognize the glide/turnaround as a stop.
        self.STROKE_START_GYRO_THRESHOLD = 0.6  # rad/s (Lowered for smoother swim starts)
        self.STROKE_START_ACCEL_THRESHOLD = 0.25  # m/s^2
        
        # TIGHTER END THRESHOLDS:
        # You must be VERY still to end a stroke.
        # This prevents "slowing down" during the stroke from splitting it into two.
        self.STROKE_END_GYRO_THRESHOLD = 0.2    # rad/s (Slightly relaxed to catch stops)
        self.STROKE_END_ACCEL_THRESHOLD = 0.25  # m/s^2 (Raised: stop tracking sooner to prevent drift)
        
        self.MIN_CAL_LEVEL = 2
        self.ACCEL_DEADZONE = 0.2 # m/s^2 (Raised: ignore more noise)
        self.VELOCITY_DECAY = 0.98 # Base friction
        self.STATIONARY_FRICTION = 0.80 # Strong braking when accel is low

        self.stroke_count = 0 # Track number of strokes
        self.stroke_start_time = 0 # To track duration
        self.MIN_STROKE_DURATION = 0.5 # Seconds. Ignore short accidental triggers.
        
        # Spike-based stroke detection parameters (tuned for swimming)
        self.ACCEL_SPIKE_THRESHOLD = 1.5  # m/s² - minimum acceleration magnitude (lowered for sensitivity)
        self.ACCEL_SPIKE_RATE_THRESHOLD = 2.0  # m/s² per second - rate of change threshold (lowered)
        self.ACCEL_NOISE_FLOOR = 0.3  # m/s² - ignore spikes below this (noise threshold)
        self.SPIKE_DEBOUNCE_TIME = 0.5  # seconds - minimum time between spike detections (increased)
        self.last_spike_time = 0  # timestamp of last detected spike
        
        # Debug mode - set to True to see detection details
        self.DEBUG_SPIKE_DETECTION = True
        
        # Rolling window for acceleration magnitude history (for spike detection)
        self.accel_mag_history = []  # List of (timestamp, magnitude) tuples
        self.ACCEL_HISTORY_SIZE = 10  # Keep last 10 samples for spike detection

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
            cal_accel = calibration.get('accel', 0)
            cal_gyro = calibration.get('gyro', 0)
            cal_mag = calibration.get('mag', 0)

        except KeyError as e:
            # This will fire if you haven't updated your C code!
            print(f"ERROR: Missing key {e}. Did you update your C code to send LIA data?")
            time.sleep(1)
            return None

        # 3. --- Check Calibration BEFORE doing any logic ---
        # BNO055 TIP: "System" status (sys) often drops to 0 during movement.
        # We should NOT stop tracking just because SYS=0, provided Accel/Gyro are good.
        if (
            cal_accel < self.MIN_CAL_LEVEL
            or cal_gyro < self.MIN_CAL_LEVEL
        ):
            # If Accel or Gyro are bad, we cannot integrate safely.
            self.in_stroke = False
            # Reset velocity but keep position static.
            self.velocity = [0.0, 0.0, 0.0]
        else:
            # --- System is sufficiently calibrated, run stroke logic ---
            # Apply Kalman Filter to Raw Linear Acceleration (LIA)
            lia_x = self.kalman_ax.update(lia_x)
            lia_y = self.kalman_ay.update(lia_y)
            lia_z = self.kalman_az.update(lia_z)

            accel_mag = math.sqrt(lia_x * lia_x + lia_y * lia_y + lia_z * lia_z)
            
            # 3.5. Spike-based stroke detection (NEW METHOD)
            # This detects sudden spikes in acceleration magnitude
            # Only run if calibrated enough (same as rest of logic)
            spike_detected = self._detect_acceleration_spike(accel_mag, t_ms)
            if spike_detected:
                self.stroke_count += 1
                print(f"\n[SPIKE DETECTED] Stroke count: {self.stroke_count} (Accel mag: {accel_mag:.2f} m/s²)", flush=True)
            
            # 4. Stroke Detection Logic (Original method - still active)
            if not self.in_stroke:
                if (
                    abs(gyro_y) > self.STROKE_START_GYRO_THRESHOLD
                    or accel_mag > self.STROKE_START_ACCEL_THRESHOLD
                ):
                    self.in_stroke = True
                    self.stroke_start_time = t_ms
                    self.position = [0.0, 0.0, 0.0] # Reset position on new stroke
                    self.velocity = [0.0, 0.0, 0.0] 
            else:
                # We are IN a stroke. Check if we should stop.
                # Only stop if BOTH Accel and Gyro are very low.
                if (
                    abs(gyro_y) < self.STROKE_END_GYRO_THRESHOLD
                    and accel_mag < self.STROKE_END_ACCEL_THRESHOLD
                ):
                    duration_sec = (t_ms - self.stroke_start_time) / 1000.0
                    # Only count as a valid stroke if it lasted long enough (e.g. > 0.5s)
                    if duration_sec > self.MIN_STROKE_DURATION:
                        self.stroke_count += 1
                        self.in_stroke = False
                    elif duration_sec > 20.0:
                         # Safety: Timeout if stroke is suspiciously long (>20s)
                         self.in_stroke = False
                    else:
                        # If too short, we don't count it, but we might stop tracking
                        # if the person really stopped.
                        # Let's say if it's SUPER short (<0.5s), it was just noise.
                        # We stop tracking but don't increment count.
                        self.in_stroke = False
            
            # 5. Integration Logic (Only runs if calibrated AND in stroke)
            if self.in_stroke:
                # --- Rotate Sensor Acceleration to World Frame ---
                # Formula: v_world = q * v_sensor * q_conjugate
                qw, qx, qy, qz = quat_w, quat_x, quat_y, quat_z
                ax, ay, az = lia_x, lia_y, lia_z

                ww = qw * qw
                xx = qx * qx
                yy = qy * qy
                zz = qz * qz
                wx = qw * qx
                wy = qw * qy
                wz = qw * qz
                xy = qx * qy
                xz = qx * qz
                yz = qy * qz

                world_ax = (ww + xx - yy - zz) * ax + 2 * (xy - wz) * ay + 2 * (xz + wy) * az
                world_ay = 2 * (xy + wz) * ax + (ww - xx + yy - zz) * ay + 2 * (yz - wx) * az
                world_az = 2 * (xz - wy) * ax + 2 * (yz + wx) * ay + (ww - xx - yy + zz) * az

                # --- Apply Deadzone to Acceleration ---
                # If acceleration is low, assume it's noise and clamp to 0
                is_accelerating = False
                if abs(world_ax) < self.ACCEL_DEADZONE: 
                    world_ax = 0.0
                else: is_accelerating = True
                
                if abs(world_ay) < self.ACCEL_DEADZONE: 
                    world_ay = 0.0
                else: is_accelerating = True
                
                if abs(world_az) < self.ACCEL_DEADZONE: 
                    world_az = 0.0
                else: is_accelerating = True

                self.velocity[0] += world_ax * dt
                self.velocity[1] += world_ay * dt
                self.velocity[2] += world_az * dt
                
                # --- Smart Friction/Decay ---
                # If we are actively accelerating, use light friction (fluid motion).
                # If we are NOT accelerating (gliding/coasting/stopping), use heavy friction
                # to kill drift before it builds up.
                decay = self.VELOCITY_DECAY if is_accelerating else self.STATIONARY_FRICTION
                
                self.velocity[0] *= decay
                self.velocity[1] *= decay
                self.velocity[2] *= decay
                
                self.position[0] += self.velocity[0] * dt
                # INVERTING position on Y and Z axes to match screen intuition
                # X: Right is Right
                # Y: Forward motion (sensor Y) should be Up/Away on screen? Or just invert sensor Y?
                # Z: Up motion (sensor Z)
                
                # Standard mapping:
                # Position[0] += Vel[0] * dt
                # Position[1] += Vel[1] * dt
                # Position[2] += Vel[2] * dt
                
                # If visualizer is inverted, we flip the signs here:
                self.position[1] += self.velocity[1] * dt
                self.position[2] += self.velocity[2] * dt
            else:
                # Damp velocity when no stroke is active to prevent drift
                self.velocity = [0.0, 0.0, 0.0]

        # 6. Create the final JSON for the visualizer
        # All directions are inverted, so flip all signs
        vis_data = {
            "timestamp": t_ms,
            "quaternion": {"qw": quat_w, "qx": quat_x, "qy": quat_y, "qz": quat_z},
            "position": {"px": self.position[0], "py": self.position[1], "pz": self.position[2]},
            "calibration": calibration, # Pass full calibration status to UI
            
            "acceleration": {"ax": lia_x, "ay": lia_y, "az": lia_z},  # Use linear acceleration (LIA)
            "angular_velocity": {"gx": data_dict['gx'], "gy": data_dict['gy'], "gz": data_dict['gz']},
            "tracking_active": self.in_stroke,
            "stroke_count": self.stroke_count
        }
        
        return json.dumps(vis_data)
    
    def _detect_acceleration_spike(self, accel_mag, timestamp_ms):
        """
        Detects sudden spikes in acceleration magnitude.
        Returns True if a spike is detected (and debounced).
        
        Algorithm:
        1. Maintain a rolling window of recent acceleration magnitudes
        2. Calculate rate of change (derivative) of acceleration
        3. Detect when rate exceeds threshold AND magnitude exceeds threshold
        4. Apply debouncing to prevent multiple detections from same event
        """
        # Add current sample to history
        self.accel_mag_history.append((timestamp_ms, accel_mag))
        
        # Keep only recent history
        if len(self.accel_mag_history) > self.ACCEL_HISTORY_SIZE:
            self.accel_mag_history.pop(0)
        
        # Need at least 3 samples to calculate rate of change
        if len(self.accel_mag_history) < 3:
            return False
        
        # Check debounce: don't detect spikes too close together
        time_since_last_spike = (timestamp_ms - self.last_spike_time) / 1000.0
        if time_since_last_spike < self.SPIKE_DEBOUNCE_TIME:
            return False
        
        # Ignore noise: acceleration must be above noise floor
        if accel_mag < self.ACCEL_NOISE_FLOOR:
            return False
        
        # Calculate rate of change (derivative) using recent samples
        # Use the last 3 samples for a simple derivative
        recent_samples = self.accel_mag_history[-3:]
        
        # Calculate average rate of change (m/s² per second)
        total_rate = 0.0
        for i in range(1, len(recent_samples)):
            dt = (recent_samples[i][0] - recent_samples[i-1][0]) / 1000.0  # Convert to seconds
            if dt > 0:
                dmag = recent_samples[i][1] - recent_samples[i-1][1]
                rate = abs(dmag / dt)  # Absolute rate of change
                total_rate += rate
        
        avg_rate = total_rate / (len(recent_samples) - 1) if len(recent_samples) > 1 else 0.0
        
        # Detect spike: both magnitude and rate of change must exceed thresholds
        is_spike = (
            accel_mag >= self.ACCEL_SPIKE_THRESHOLD and
            avg_rate >= self.ACCEL_SPIKE_RATE_THRESHOLD
        )
        
        # Debug output (only print occasionally to avoid spam)
        if self.DEBUG_SPIKE_DETECTION and len(self.accel_mag_history) % 20 == 0:
            print(f"[DEBUG] Accel: {accel_mag:.2f} m/s², Rate: {avg_rate:.2f} m/s²/s, "
                  f"Thresholds: mag>={self.ACCEL_SPIKE_THRESHOLD}, rate>={self.ACCEL_SPIKE_RATE_THRESHOLD} | "
                  f"Spike: {is_spike}", flush=True)
        
        if is_spike:
            self.last_spike_time = timestamp_ms
            if self.DEBUG_SPIKE_DETECTION:
                print(f"\n[SPIKE DETECTED] Accel: {accel_mag:.2f} m/s², Rate: {avg_rate:.2f} m/s²/s", flush=True)
            return True
        
        return False


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
    
    def do_POST(self):
        if self.path == '/reset':
            global stroke_processor_instance
            if stroke_processor_instance:
                stroke_processor_instance.stroke_count = 0
                stroke_processor_instance.position = [0.0, 0.0, 0.0]
                stroke_processor_instance.velocity = [0.0, 0.0, 0.0]
                stroke_processor_instance.in_stroke = False
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status": "reset"}')
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
                        serial_port = serial.Serial(port, 115200, timeout=2, write_timeout=1)
                        # Clear any stale data in buffers
                        serial_port.reset_input_buffer()
                        serial_port.reset_output_buffer()
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
        
        # Buffer for partial JSON lines
        buffer = ""
        
        while True:
            try:
                # Read available data (may be partial)
                if serial_port.in_waiting > 0:
                    try:
                        raw_data = serial_port.read(serial_port.in_waiting)
                        buffer += raw_data.decode('utf-8', errors='ignore')
                    except (serial.SerialException, OSError) as e:
                        error_msg = str(e).lower()
                        if "readiness to read" in error_msg or "no data" in error_msg:
                            # Port is in bad state, reset it
                            time.sleep(0.1)
                            try:
                                serial_port.reset_input_buffer()
                            except:
                                pass
                            continue
                        else:
                            raise
                
                # Process complete lines (ending with newline)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not line:
                        continue
                    
                    # Try to parse JSON
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
                                cal_accel = cal_status.get('accel', 0)  # Fixed: read from data
                                cal_mag = cal_status.get('mag', 0)
                                
                                # Provide helpful hints based on missing calibration
                                hint = ""
                                if cal_accel < 3:
                                    hint = "[TIP: Place sensor in 6 different stable positions for ACCEL]"
                                elif cal_mag < 3:
                                    hint = "[TIP: Move in figure-8 for MAG]"
                                elif cal_gyro < 3:
                                    hint = "[TIP: Leave sensor completely still for GYRO]"
                                
                                # Stroke status
                                status_str = "ACTIVE" if processor.in_stroke else "IDLE"
                                count_str = f"Strokes: {processor.stroke_count}"
                                
                                # Use print() with carriage return to update in-place
                                print(f"CALIB: S={cal_sys} G={cal_gyro} A={cal_accel} M={cal_mag} | {status_str} | {count_str} | {hint}          ", end='\r', flush=True)
                                
                                last_cal_print_time = current_time
                                last_cal_status = cal_status
                                
                                if cal_accel >= 2 and cal_gyro >= 2:
                                    print(f"\n--- SENSORS READY (A={cal_accel} G={cal_gyro}) [{status_str}] ---", flush=True)
                                elif cal_accel < 2:
                                    print("\n--- Waiting for Accelerometer (Rotate Sensor) ---", flush=True)

                    except json.JSONDecodeError:
                        # Ignore incomplete or corrupted JSON lines
                        pass
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.01)
                    
            except serial.SerialException as e:
                error_msg = str(e).lower()
                if "readiness to read" in error_msg or "no data" in error_msg:
                    # Try to recover by resetting the port
                    try:
                        serial_port.reset_input_buffer()
                        time.sleep(0.1)
                        continue
                    except:
                        pass
                
                print(f"\nSerial read error (device disconnected?): {e}", flush=True)
                try:
                    serial_port.close()
                except:
                    pass
                serial_port = None
                buffer = ""  # Clear buffer on disconnect
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
    stroke_processor_instance = stroke_processor
    
    receiver_thread = threading.Thread(target=serial_receiver, args=(stroke_processor,), daemon=True)
    receiver_thread.start()
    
    start_web_server(port)