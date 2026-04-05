#!/usr/bin/env python3
"""
IMU Data Visualizer & Calibration Helper - FIXED VERSION

Key fix: Full-cycle integration (catch → pull → exit → recovery)
- tracking_active stays True from stroke detection until NEXT stroke
- Phase-adaptive filtering (tight for underwater, loose for recovery)
- No arbitrary timeout cutting off trajectory mid-stroke

Research basis: SwimBIT (PMC6915422), Frontiers fbioe.2021.793302
"""

import serial
import serial.tools.list_ports
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import sys
import os
import math

# --- Global variables ---
latest_data_json = None
data_lock = threading.Lock()
clients = []
serial_port = None
last_cal_print_time = 0
last_cal_status = {}
stroke_processor_instance = None


class SimpleKalmanFilter:
    """1D Kalman filter for acceleration smoothing."""
    def __init__(self, process_noise, measurement_noise, estimation_error, initial_value):
        self.Q = process_noise
        self.R = measurement_noise
        self.P = estimation_error
        self.X = initial_value

    def update(self, measurement):
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        return self.X


class StrokeProcessor:
    """
    Full-cycle stroke position integration.
    
    KEY CHANGE from original:
    - stroke_cycle_active: True from stroke detection until NEXT stroke
    - Frontend collects points continuously during entire cycle
    - Position resets only when new stroke detected
    - Phase-adaptive filtering reduces drift during recovery
    """
    
    def __init__(self, batch_mode=False):
        self.batch_mode = batch_mode
        
        # Position & velocity state
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.last_timestamp_ms = None
        
        # Kalman filters (Q=0.3 high for rapid human motion, R=0.1 low for BNO055 precision)
        self.kalman_ax = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_ay = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_az = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        
        # High-pass filter for gravity residual removal (uncalibrated accel)
        self.hp_alpha = 0.02
        self.hp_avg = [0.0, 0.0, 0.0]
        
        # =====================================================================
        # STROKE DETECTION THRESHOLDS (research-backed)
        # =====================================================================
        self.WATER_ENTRY_ACCEL_THRESHOLD = 5.5 if batch_mode else 6.0
        self.DOWNWARD_ACCEL_THRESHOLD = -0.8
        self.DOWNWARD_SAMPLE_WINDOW = 8
        self.DOWNWARD_REQUIRED_COUNT = 3 if batch_mode else 4
        self.IMPACT_REVERSAL_THRESHOLD = 2.0
        self.IMPACT_JERK_THRESHOLD = 250.0
        self.IMPACT_DELTA_A_THRESHOLD = 4.0
        self.ENTRY_GYRO_THRESHOLD = 0.6 if batch_mode else 0.8
        self.MIN_STROKE_INTERVAL = 1.1  # Prevents counting recovery as stroke
        
        # Wall/turn detection
        self.WALL_IMPACT_THRESHOLD = 34.0
        self.WALL_SUSTAINED_COUNT = 4
        self.STROKE_TURN_COOLDOWN_MS = 2000
        self.TURN_LOCKOUT_DURATION = 2.5
        self.last_wall_impact_time_ms = 0
        self.wall_high_count = 0
        self.turn_count = 0
        
        # History buffers
        self.recent_world_az = []
        self.MAX_RECENT_SAMPLES = 12
        self.recent_accel_mag = []
        self.prev_accel_mag = None
        self.last_stroke_time_ms = 0
        
        # Glide detection
        self.GLIDE_ACCEL_THRESHOLD = 1.5
        self.glide_sample_count = 0
        self.GLIDE_SAMPLE_THRESHOLD = 20
        self.is_gliding = False
        
        # =====================================================================
        # FULL-CYCLE INTEGRATION (THE KEY FIX)
        # =====================================================================
        # stroke_cycle_active: True from stroke detection until NEXT stroke
        # This is what tracking_active sends to frontend
        # Frontend collects points for ENTIRE cycle, not just 0.6s
        self.stroke_cycle_active = False
        self.stroke_start_time = 0
        self.stroke_count = 0
        self.just_reset = False
        
        # Phase detection for adaptive filtering
        self.current_phase = 'idle'
        self.phase_start_time = 0
        self.CATCH_DURATION_MS = 150
        self.RECOVERY_GYRO_THRESHOLD = 1.5
        self.RECOVERY_ACCEL_CEILING = 3.0
        
        # Phase-adaptive integration parameters
        # Pull phase: underwater, strong signal → tight deadzone, light decay
        # Recovery phase: in air, noisy → wide deadzone, heavy decay
        self.PHASE_PARAMS = {
            'catch': {'deadzone': 0.2, 'decay': 0.98},
            'pull': {'deadzone': 0.2, 'decay': 0.98},
            'exit': {'deadzone': 0.25, 'decay': 0.96},
            'recovery': {'deadzone': 0.5, 'decay': 0.92},  # Heavy damping for air
            'glide': {'deadzone': 0.4, 'decay': 0.90},
            'idle': {'deadzone': 0.3, 'decay': 0.95},
        }
        
        # Entry angle tracking
        self.last_entry_angle = 0.0
        self.entry_angles = []
        self.ideal_entry_angle = 30.0
        self.last_entry_gyro_mag = 0.0
        self.last_entry_gx = self.last_entry_gy = self.last_entry_gz = 0.0
        
        # Debug state
        self.debug_world_az = 0.0
        self.debug_accel_mag = 0.0
        self.debug_gyro_mag = 0.0
        self.debug_was_downward = False
        self.debug_is_impact = False
        self.debug_jerk = 0.0
        self.debug_is_impact_by_jerk = False
        self.debug_in_turn_lockout = False

    def process_data(self, data_dict):
        """Process sensor data, detect strokes, integrate position."""
        
        # 1. Timestamp & delta-time
        t_ms = data_dict['t']
        if self.last_timestamp_ms is None:
            self.last_timestamp_ms = t_ms
            return None
        
        dt = (t_ms - self.last_timestamp_ms) / 1000.0
        self.last_timestamp_ms = t_ms
        
        if dt <= 0 or dt > 0.5:
            return None

        # 2. Extract sensor data
        try:
            lia_x, lia_y, lia_z = data_dict['lia_x'], data_dict['lia_y'], data_dict['lia_z']
            qw, qx, qy, qz = data_dict['qw'], data_dict['qx'], data_dict['qy'], data_dict['qz']
            gx, gy, gz = data_dict['gx'], data_dict['gy'], data_dict['gz']
            
            calibration = data_dict.get('cal') or data_dict.get('calibration', {})
            cal_accel = calibration.get('accel', 0)
            cal_gyro = calibration.get('gyro', 0)
        except KeyError as e:
            print(f"ERROR: Missing key {e}")
            return None

        # 3. Compute magnitudes and world-frame acceleration
        raw_accel_mag = math.sqrt(lia_x**2 + lia_y**2 + lia_z**2)
        gyro_mag = math.sqrt(gx**2 + gy**2 + gz**2)
        
        # Quaternion rotation to world frame
        ww, xx, yy, zz = qw*qw, qx*qx, qy*qy, qz*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        
        world_ax = (ww + xx - yy - zz)*lia_x + 2*(xy - wz)*lia_y + 2*(xz + wy)*lia_z
        world_ay = 2*(xy + wz)*lia_x + (ww - xx + yy - zz)*lia_y + 2*(yz - wx)*lia_z
        world_az = 2*(xz - wy)*lia_x + 2*(yz + wx)*lia_y + (ww - xx - yy + zz)*lia_z
        
        # Kalman-smoothed for integration
        smooth_x = self.kalman_ax.update(lia_x)
        smooth_y = self.kalman_ay.update(lia_y)
        smooth_z = self.kalman_az.update(lia_z)
        
        # =====================================================================
        # 4. STROKE DETECTION
        # =====================================================================
        time_since_wall = (t_ms - self.last_wall_impact_time_ms) / 1000.0 if self.last_wall_impact_time_ms else 999.0
        in_turn_lockout = self.last_wall_impact_time_ms > 0 and time_since_wall < self.TURN_LOCKOUT_DURATION
        
        # Glide detection
        if raw_accel_mag < self.GLIDE_ACCEL_THRESHOLD and gyro_mag < 0.5:
            self.glide_sample_count += 1
            if self.glide_sample_count >= self.GLIDE_SAMPLE_THRESHOLD:
                self.is_gliding = True
        else:
            self.glide_sample_count = 0
            self.is_gliding = False
        
        # Update history buffers
        self.recent_world_az.append(world_az)
        if len(self.recent_world_az) > self.MAX_RECENT_SAMPLES:
            self.recent_world_az.pop(0)
        
        if self.prev_accel_mag is None:
            self.prev_accel_mag = raw_accel_mag
        accel_delta = raw_accel_mag - self.prev_accel_mag
        jerk = accel_delta / dt if dt > 0 else 0.0
        self.prev_accel_mag = raw_accel_mag
        
        # Check downward motion history
        was_moving_downward = False
        if len(self.recent_world_az) > self.DOWNWARD_SAMPLE_WINDOW:
            history = self.recent_world_az[-(self.DOWNWARD_SAMPLE_WINDOW + 1):-1]
            downward_count = sum(1 for az in history if az < self.DOWNWARD_ACCEL_THRESHOLD)
            was_moving_downward = downward_count >= self.DOWNWARD_REQUIRED_COUNT
        
        # Impact detection
        is_impact_spike = (
            raw_accel_mag > self.WATER_ENTRY_ACCEL_THRESHOLD
            and world_az > self.IMPACT_REVERSAL_THRESHOLD
        )
        is_impact_by_jerk = (
            raw_accel_mag > self.WATER_ENTRY_ACCEL_THRESHOLD
            and (jerk > self.IMPACT_JERK_THRESHOLD or accel_delta > self.IMPACT_DELTA_A_THRESHOLD)
        )
        
        time_since_last_stroke = (t_ms - self.last_stroke_time_ms) / 1000.0
        interval_ok = time_since_last_stroke >= self.MIN_STROKE_INTERVAL
        has_entry_rotation = gyro_mag > self.ENTRY_GYRO_THRESHOLD
        
        impact_detected = is_impact_spike or is_impact_by_jerk
        stroke_detected = (
            impact_detected
            and interval_ok
            and has_entry_rotation
            and not in_turn_lockout
            and not self.is_gliding
            and (was_moving_downward or is_impact_by_jerk)
        )
        
        # Store debug values
        self.debug_world_az = world_az
        self.debug_accel_mag = raw_accel_mag
        self.debug_gyro_mag = gyro_mag
        self.debug_was_downward = was_moving_downward
        self.debug_is_impact = is_impact_spike
        self.debug_jerk = jerk
        self.debug_is_impact_by_jerk = is_impact_by_jerk
        self.debug_in_turn_lockout = in_turn_lockout
        
        # =====================================================================
        # 5. NEW STROKE DETECTED → RESET & START NEW CYCLE
        # =====================================================================
        if stroke_detected:
            self.stroke_count += 1
            self.last_stroke_time_ms = t_ms
            self.wall_high_count = 0
            self.recent_world_az.clear()
            self.glide_sample_count = 0
            
            # RESET position & velocity for new stroke
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.hp_avg = [0.0, 0.0, 0.0]
            
            # START new cycle (stays True until NEXT stroke)
            self.stroke_cycle_active = True
            self.stroke_start_time = t_ms
            
            # Entry angle from quaternion pitch
            sinp = 2.0 * (qw * qy - qz * qx)
            sinp = max(-1.0, min(1.0, sinp))
            self.last_entry_angle = abs(math.degrees(math.asin(sinp)))
            self.entry_angles.append(self.last_entry_angle)
            self.last_entry_gyro_mag = gyro_mag
            self.last_entry_gx, self.last_entry_gy, self.last_entry_gz = gx, gy, gz
        
        # Wall/turn detection (after cooldown from stroke)
        since_stroke_ms = t_ms - self.last_stroke_time_ms if self.last_stroke_time_ms else self.STROKE_TURN_COOLDOWN_MS + 1
        if since_stroke_ms >= self.STROKE_TURN_COOLDOWN_MS:
            if raw_accel_mag > self.WALL_IMPACT_THRESHOLD:
                self.wall_high_count += 1
                if self.wall_high_count >= self.WALL_SUSTAINED_COUNT:
                    if time_since_wall > self.TURN_LOCKOUT_DURATION:
                        self.turn_count += 1
                    self.last_wall_impact_time_ms = t_ms
                    self.recent_world_az.clear()
                    self.wall_high_count = 0
            else:
                self.wall_high_count = 0
        else:
            self.wall_high_count = 0
        
        # =====================================================================
        # 6. PHASE DETECTION (for adaptive filtering)
        # =====================================================================
        time_since_stroke_ms = t_ms - self.stroke_start_time if self.stroke_start_time > 0 else 999999
        
        if self.is_gliding:
            new_phase = 'glide'
        elif self.stroke_cycle_active and time_since_stroke_ms < self.CATCH_DURATION_MS:
            new_phase = 'catch'
        elif self.stroke_cycle_active and time_since_stroke_ms < 500:  # ~0.5s underwater
            new_phase = 'pull'
        elif self.stroke_cycle_active and time_since_stroke_ms < 700:  # Exit water
            new_phase = 'exit'
        elif self.stroke_cycle_active and gyro_mag > self.RECOVERY_GYRO_THRESHOLD:
            new_phase = 'recovery'
        elif self.stroke_cycle_active and raw_accel_mag < self.RECOVERY_ACCEL_CEILING:
            new_phase = 'recovery'
        elif self.stroke_cycle_active:
            new_phase = 'recovery'  # Default to recovery if in cycle but not underwater
        else:
            new_phase = 'idle'
        
        if new_phase != self.current_phase:
            self.phase_start_time = t_ms
        self.current_phase = new_phase
        
        # =====================================================================
        # 7. POSITION INTEGRATION (full cycle, phase-adaptive)
        # =====================================================================
        if self.stroke_cycle_active:
            # World-frame transform of smoothed acceleration
            int_ax = (ww + xx - yy - zz)*smooth_x + 2*(xy - wz)*smooth_y + 2*(xz + wy)*smooth_z
            int_ay = 2*(xy + wz)*smooth_x + (ww - xx + yy - zz)*smooth_y + 2*(yz - wx)*smooth_z
            int_az = 2*(xz - wy)*smooth_x + 2*(yz + wx)*smooth_y + (ww - xx - yy + zz)*smooth_z
            
            # High-pass filter for uncalibrated accel
            if cal_accel < 2:
                self.hp_avg[0] += self.hp_alpha * (int_ax - self.hp_avg[0])
                self.hp_avg[1] += self.hp_alpha * (int_ay - self.hp_avg[1])
                self.hp_avg[2] += self.hp_alpha * (int_az - self.hp_avg[2])
                int_ax -= self.hp_avg[0]
                int_ay -= self.hp_avg[1]
                int_az -= self.hp_avg[2]
            
            # Phase-adaptive parameters
            params = self.PHASE_PARAMS.get(self.current_phase, self.PHASE_PARAMS['idle'])
            deadzone = params['deadzone']
            decay = params['decay']
            
            # Vector magnitude deadzone (prevents diagonal drift)
            int_mag = math.sqrt(int_ax**2 + int_ay**2 + int_az**2)
            if int_mag < deadzone:
                int_ax = int_ay = int_az = 0.0
            
            # Integrate velocity
            self.velocity[0] += int_ax * dt
            self.velocity[1] += int_ay * dt
            self.velocity[2] += int_az * dt
            
            # Apply decay
            self.velocity[0] *= decay
            self.velocity[1] *= decay
            self.velocity[2] *= decay
            
            # Integrate position
            self.position[0] += self.velocity[0] * dt
            self.position[1] += self.velocity[1] * dt
            self.position[2] += self.velocity[2] * dt
            
            # Soft clamp (gradual braking, preserves shape)
            # Only activates if position gets unreasonably large (>1m)
            pos_mag = math.sqrt(sum(p*p for p in self.position))
            if pos_mag > 1.0:
                brake = 0.95
                self.velocity = [v * brake for v in self.velocity]
        else:
            self.velocity = [0.0, 0.0, 0.0]
        
        # =====================================================================
        # 8. BUILD OUTPUT JSON
        # =====================================================================
        stroke_vel = math.sqrt(sum(v*v for v in self.velocity))
        avg_entry = sum(self.entry_angles) / len(self.entry_angles) if self.entry_angles else 0
        
        # Firmware-reported values (passthrough)
        fw_entry = float(data_dict.get('entry_angle', 0) or 0)
        fw_strokes = int(data_dict.get('strokes', 0) or 0)
        display_entry = round(fw_entry, 1) if fw_entry > 0.05 else round(self.last_entry_angle, 1)
        
        vis_data = {
            "timestamp": t_ms,
            "quaternion": {"qw": qw, "qx": qx, "qy": qy, "qz": qz},
            "position": {"px": self.position[0], "py": self.position[1], "pz": self.position[2]},
            "calibration": calibration,
            
            "acceleration": {"ax": lia_x, "ay": lia_y, "az": lia_z},
            "angular_velocity": {"gx": gx, "gy": gy, "gz": gz},
            "magnetometer": {
                "mx": float(data_dict.get('mx', 0) or 0),
                "my": float(data_dict.get('my', 0) or 0),
                "mz": float(data_dict.get('mz', 0) or 0),
            },
            
            # THE KEY FIX: tracking_active = stroke_cycle_active (full cycle)
            "tracking_active": self.stroke_cycle_active,
            "stroke_count": self.stroke_count,
            "strokes": fw_strokes,  # Firmware-reported stroke count
            "turn_count": self.turn_count,
            "just_reset": self.just_reset,
            
            # Phase info
            "stroke_phase": self.current_phase,
            "stroke_velocity_ms": round(stroke_vel, 3),
            
            # Entry angle
            "entry_angle": display_entry,
            "avg_entry_angle": round(avg_entry, 1),
            "ideal_entry_angle": self.ideal_entry_angle,
            "entry_gyro_mag": round(self.last_entry_gyro_mag, 4),
            "entry_gyro": {
                "gx": round(self.last_entry_gx, 4),
                "gy": round(self.last_entry_gy, 4),
                "gz": round(self.last_entry_gz, 4),
            },
            
            # Haptic feedback passthrough (from firmware)
            "haptic_fired": data_dict.get('haptic_fired', data_dict.get('haptic', False)),
            "deviation_score": data_dict.get('deviation_score', data_dict.get('deviation', 0.0)),
            "haptic_reason": data_dict.get('haptic_reason', 0),
            "pull_duration_ms": data_dict.get('pull_duration_ms', 0.0),
            
            # Device identification
            "device_id": data_dict.get('dev_id', 0),
            "device_role": data_dict.get('dev_role', 0),
            
            # Debug
            "stroke_debug": {
                "world_az": round(self.debug_world_az, 2),
                "accel_mag": round(self.debug_accel_mag, 2),
                "gyro_mag": round(self.debug_gyro_mag, 2),
                "jerk": round(self.debug_jerk, 1),
                "was_downward": self.debug_was_downward,
                "is_impact": self.debug_is_impact,
                "is_impact_by_jerk": self.debug_is_impact_by_jerk,
                "turn_lockout": self.debug_in_turn_lockout,
                "is_gliding": self.is_gliding
            }
        }
        
        if self.just_reset:
            self.just_reset = False
        
        return json.dumps(vis_data)


# =============================================================================
# HTTP SERVER (unchanged)
# =============================================================================
class SSEHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Suppress HTTP logs
    
    def do_GET(self):
        if self.path == '/events':
            self.handle_sse()
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), 'simple_imu_3d.html')
            try:
                with open(html_path, 'rb') as f:
                    self.wfile.write(f.read())
            except FileNotFoundError:
                self.wfile.write(b"Error: simple_imu_3d.html not found")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
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
                stroke_processor_instance.stroke_cycle_active = False
                stroke_processor_instance.recent_world_az.clear()
                stroke_processor_instance.prev_accel_mag = None
                stroke_processor_instance.last_stroke_time_ms = 0
                stroke_processor_instance.last_wall_impact_time_ms = 0
                stroke_processor_instance.glide_sample_count = 0
                stroke_processor_instance.is_gliding = False
                stroke_processor_instance.just_reset = True
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
        try:
            while True:
                with data_lock:
                    if latest_data_json:
                        try:
                            self.wfile.write(f"data: {latest_data_json}\n\n".encode())
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError, OSError):
                            break
                time.sleep(0.01)
        except:
            pass
        finally:
            if self in clients:
                clients.remove(self)


def broadcast_data(json_data):
    if json_data is None:
        return
    with data_lock:
        global latest_data_json
        latest_data_json = json_data


def get_serial_port_list():
    """Find USB-serial ports, excluding Bluetooth."""
    skip = ("Bluetooth", "BLTH", "debug", "modem", "rfcomm")
    ports = [p.device for p in serial.tools.list_ports.comports()
             if not any(s.upper() in p.device.upper() for s in skip)]
    
    def priority(path):
        if any(x in path for x in ("SLAB_USBtoUART", "usbserial", "wchusbserial")):
            return 0
        if sys.platform == "darwin" and path.startswith("/dev/cu."):
            return 1
        return 2
    
    ports.sort(key=lambda x: (priority(x), x))
    return ports


def serial_receiver(processor, preferred_port=None):
    """Read IMU data from ESP32 serial port."""
    global serial_port, last_cal_print_time, last_cal_status
    
    while True:
        if serial_port is None:
            ports = [preferred_port] if preferred_port else get_serial_port_list()
            print(f"Trying ports: {ports}")
            
            for port in ports:
                try:
                    serial_port = serial.Serial(port, 115200, timeout=1)
                    print(f"Connected: {port}")
                    break
                except Exception as e:
                    print(f"  {port}: {e}")
                    serial_port = None
            
            if serial_port is None:
                print("No ESP32 found. Retrying in 5s...")
                time.sleep(5)
                continue
        
        print("Reading IMU data...")
        
        while True:
            try:
                raw_line = serial_port.readline()
                if not raw_line:
                    time.sleep(0.01)
                    continue
                
                try:
                    line = raw_line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    continue
                
                if line:
                    try:
                        data = json.loads(line)
                        if isinstance(data, dict) and 't' in data and 'lia_x' in data:
                            result = processor.process_data(data)
                            if result:
                                broadcast_data(result)
                            
                            # Periodic status print
                            now = time.time()
                            cal = data.get('cal', {})
                            if now - last_cal_print_time > 1.0 or cal != last_cal_status:
                                phase = processor.current_phase
                                active = "ACTIVE" if processor.stroke_cycle_active else "IDLE"
                                print(f"CAL: A={cal.get('accel',0)} G={cal.get('gyro',0)} | "
                                      f"Strokes: {processor.stroke_count} | "
                                      f"Phase: {phase} | {active}", end='\r')
                                last_cal_print_time = now
                                last_cal_status = cal
                    except json.JSONDecodeError:
                        pass
                        
            except serial.SerialException as e:
                print(f"\nSerial error: {e}")
                serial_port.close()
                serial_port = None
                last_cal_status = {}
                break
            except Exception as e:
                if not isinstance(e, (BrokenPipeError, ConnectionResetError, OSError)):
                    print(f"\nError: {e}")
                time.sleep(0.1)


def start_web_server(port):
    """Start HTTP server."""
    try:
        server = HTTPServer(('0.0.0.0', port), SSEHandler)
        print(f"Web server: http://localhost:{port}")
        server.serve_forever()
    except OSError as e:
        print(f"Port {port} in use: {e}")
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("IMU Visualizer - FIXED (Full-Cycle Integration)")
    print("=" * 60)
    
    http_port = int(os.environ.get("PORT", 8003))
    serial_port_arg = None
    
    if len(sys.argv) >= 2:
        arg = sys.argv[1]
        if arg.startswith("/dev/") or arg.upper().startswith("COM"):
            serial_port_arg = arg
            if len(sys.argv) >= 3:
                try:
                    http_port = int(sys.argv[2])
                except ValueError:
                    pass
        else:
            try:
                http_port = int(arg)
            except ValueError:
                print(f"Usage: python3 {sys.argv[0]} [serial_port] [http_port]")
    
    processor = StrokeProcessor()
    stroke_processor_instance = processor
    
    receiver = threading.Thread(target=serial_receiver, args=(processor, serial_port_arg), daemon=True)
    receiver.start()
    
    start_web_server(http_port)