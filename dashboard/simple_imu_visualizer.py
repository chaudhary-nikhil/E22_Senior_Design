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
    Holds the state for velocity and position integration.
    Detects strokes on DOWNWARD hand entry into water (catch phase).
    
    Research basis (SwimBIT & Frontiers IMU swimming papers):
    - Water entry creates distinct deceleration spike (~1.5-2.5 m/s²)
    - Recovery phase (arm up in air) has no water resistance spike
    - Typical stroke rate: 40-80 strokes/min = 0.75-1.5s per stroke
    - Downward motion shows negative vertical acceleration before impact
    
    Hardware: BNO055 IMU in junction box (e.g., B083H9D3P1) mounted on TOP of hand
    - Box adds ~5-10mm offset but doesn't affect orientation significantly
    - Waterproof enclosure protects sensor during submersion
    
    Coordinate System (BNO055 in NDOF fusion mode):
    - world_az > 0: Accelerating UPWARD
    - world_az < 0: Accelerating DOWNWARD
    - At water impact: Upward deceleration force creates POSITIVE world_az spike
    """
    def __init__(self, batch_mode=False):
        """
        batch_mode: When True (post-session playback), use slightly more lenient thresholds
        to improve stroke detection on recorded data which may differ from live stream.
        """
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.last_timestamp_ms = None
        self.in_stroke = False
        self.just_reset = False
        self.batch_mode = batch_mode
        
        # Kalman Filters for Acceleration (X, Y, Z)
        # TUNING for HUMAN MOTION:
        # Q (Process Noise): 0.3 -> High because human motion changes accel rapidly.
        # R (Measurement Noise): 0.1 -> Low because BNO055 is relatively precise.
        self.kalman_ax = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_ay = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_az = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        
        # =====================================================================
        # STROKE DETECTION: Downward Water Entry (Research-Backed Thresholds)
        # =====================================================================
        # Based on: SwimBIT (PMC6915422) and Frontiers macro-micro swimming analysis
        #
        # PHYSICS OF WATER ENTRY:
        #   1. Before impact: Hand accelerating DOWN → world_az NEGATIVE
        #   2. At impact: Water resistance creates UPWARD force → world_az spikes POSITIVE
        #   3. This positive spike only occurs after downward motion (not during recovery)
        #
        # DETECTION STRATEGY:
        #   - Check if PREVIOUS samples show downward motion (negative world_az)
        #   - Detect CURRENT sample as impact (high accel magnitude + direction reversal)
        #   - Verify with gyroscope (hand rotating during entry)
        #
        # IMU placement: Top of hand in waterproof junction box
        # =====================================================================
        
        # WATER ENTRY IMPACT DETECTION
        # Research: SwimBIT uses ~1.6g envelope; Frontiers uses 0.9g for wrist
        # Water entry produces a short, high-magnitude transient (impact/resistance).
        # batch_mode: slightly lower to catch softer entries in post-session data
        self.WATER_ENTRY_ACCEL_THRESHOLD = 5.5 if batch_mode else 6.0  # m/s² - total acceleration magnitude at impact
        
        # DOWNWARD MOTION VERIFICATION (checked BEFORE impact, not at impact)
        # Must confirm hand WAS moving DOWN before counting impact as stroke
        # Research: Prevents false positives from recovery phase (arm going up in air)
        self.DOWNWARD_ACCEL_THRESHOLD = -0.8  # m/s² - world_az threshold for "moving down"
        self.DOWNWARD_SAMPLE_WINDOW = 8  # Check last 8 samples (~80ms at 100Hz)
        self.DOWNWARD_REQUIRED_COUNT = 3 if batch_mode else 4  # At least N of last 8 must show downward motion
        
        # DIRECTION REVERSAL DETECTION (key to detecting water impact)
        # At impact: world_az changes from negative (down) to positive (deceleration)
        # This reversal is the signature of hitting a resistant medium (water)
        self.IMPACT_REVERSAL_THRESHOLD = 2.0  # m/s² - world_az must jump above this at impact

        # SHARP CHANGE DETECTION (impact fallback)
        # Frontiers macro-micro analysis explicitly uses "sharp change detection" as a common processing method.
        # This makes stroke counting robust when the entry is messy and the vertical reversal isn't clean,
        # while still being independent of where the arm ends after the stroke.
        self.IMPACT_JERK_THRESHOLD = 250.0  # m/s^3  (jerk = d|a|/dt)
        self.IMPACT_DELTA_A_THRESHOLD = 4.0  # m/s^2 (fallback if dt jitter)
        
        # ANGULAR VELOCITY FOR ENTRY DETECTION
        # Hand rotating into water shows characteristic gyro pattern
        # Research: SwimBIT uses roll angle; typical entry shows 1-3 rad/s rotation
        # batch_mode: slightly lower for post-session data (may have different characteristics)
        self.ENTRY_GYRO_THRESHOLD = 0.6 if batch_mode else 0.8  # rad/s - hand rotating during entry
        
        # STROKE TIMING CONSTRAINTS
        # Research: Typical stroke rate 40-80 strokes/min = 0.75-1.5s interval
        # Elite swimmers: up to 100 strokes/min = 0.6s interval
        # Setting minimum to 0.5s to catch fast swimmers
        self.MIN_STROKE_INTERVAL = 0.5  # seconds between strokes
        
        # TURN/WALL DETECTION (prevent false positives at wall)
        # Wall impact has very high acceleration (>30 m/s²) and sustained high values
        # Also detected by prolonged high-accel period (push-off)
        self.WALL_IMPACT_THRESHOLD = 25.0  # m/s² - accelerations above this likely wall/turn
        self.TURN_LOCKOUT_DURATION = 2.0  # seconds - ignore strokes after wall impact
        self.last_wall_impact_time_ms = 0  # Track last wall impact
        
        # Track recent world-frame vertical acceleration for downward motion detection
        self.recent_world_az = []  # Ring buffer of recent world_az values
        self.MAX_RECENT_SAMPLES = 12  # Keep last 12 samples (~120ms at 100Hz)
        self.recent_accel_mag = []  # Ring buffer of accel magnitude for jerk/spike detection
        self.prev_accel_mag = None
        self.last_stroke_time_ms = 0  # Timestamp of last detected stroke
        
        # GLIDE PHASE DETECTION (suppress strokes during glide after turn)
        # During glide, acceleration is very low and steady
        self.GLIDE_ACCEL_THRESHOLD = 1.5  # m/s² - below this is likely gliding
        self.glide_sample_count = 0  # Count consecutive low-accel samples
        self.GLIDE_SAMPLE_THRESHOLD = 20  # 20 consecutive low samples = gliding
        self.is_gliding = False
        
        # TRACKING STATE THRESHOLDS (for position integration, separate from stroke detection)
        self.STROKE_START_GYRO_THRESHOLD = 0.6  # rad/s - start position tracking
        self.STROKE_START_ACCEL_THRESHOLD = 0.25  # m/s²
        self.STROKE_END_GYRO_THRESHOLD = 0.2  # rad/s - stop position tracking
        self.STROKE_END_ACCEL_THRESHOLD = 0.25  # m/s²
        
        self.MIN_CAL_LEVEL = 2
        self.ACCEL_DEADZONE = 0.2  # m/s² - ignore noise below this
        self.VELOCITY_DECAY = 0.98  # Base friction
        self.STATIONARY_FRICTION = 0.80  # Strong braking when accel is low

        self.stroke_count = 0  # Track number of strokes
        self.stroke_start_time = 0  # To track duration
        self.MIN_STROKE_DURATION = 0.5  # Seconds for position tracking
        
        # DEBUG: Track stroke detection state for troubleshooting
        self.debug_world_az = 0.0
        self.debug_accel_mag = 0.0
        self.debug_gyro_mag = 0.0
        self.debug_was_downward = False
        self.debug_is_impact = False
        self.debug_jerk = 0.0
        self.debug_is_impact_by_jerk = False
        self.debug_in_turn_lockout = False

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
            
            # =================================================================
            # COMPUTE WORLD-FRAME ACCELERATION (needed for stroke detection)
            # =================================================================
            # Transform sensor-frame linear acceleration to world frame using quaternion
            # This gives us true vertical acceleration regardless of hand orientation
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
            
            # Gyroscope magnitude for rotation detection
            gyro_mag = math.sqrt(data_dict['gx']**2 + data_dict['gy']**2 + data_dict['gz']**2)
            
            # =================================================================
            # 4. STROKE DETECTION: Downward Water Entry
            # =================================================================
            # Research basis: SwimBIT & Frontiers swimming analysis papers
            # 
            # PHYSICS OF HAND WATER ENTRY:
            #   Timeline:
            #   t-80ms: Hand swinging down, world_az ≈ -2 to -5 m/s² (accelerating down)
            #   t-40ms: Hand approaching water, world_az ≈ -1 to -3 m/s²
            #   t=0:    IMPACT! Water resistance → world_az spikes to +5 to +15 m/s²
            #   t+40ms: Hand in water, pulling, world_az settles
            #
            # DETECTION STRATEGY:
            #   1. Check if PREVIOUS samples (before current) show downward motion
            #   2. Detect CURRENT sample as impact (high magnitude + positive world_az)
            #   3. Verify with gyroscope (hand rotating during entry)
            #   4. Filter out wall impacts and gliding phases
            #
            # This PREVENTS false positives from:
            #   - Recovery phase (arm going UP in air - no downward history)
            #   - Wall/turn impacts (filtered by extreme magnitude + lockout)
            #   - Gliding phases (filtered by low sustained acceleration)
            # =================================================================
            
            # --- WALL/TURN DETECTION ---
            # Wall impacts have very high acceleration; lock out stroke detection briefly
            if accel_mag > self.WALL_IMPACT_THRESHOLD:
                self.last_wall_impact_time_ms = t_ms
                self.recent_world_az.clear()  # Reset buffer after wall impact
            
            time_since_wall = (t_ms - self.last_wall_impact_time_ms) / 1000.0
            in_turn_lockout = (time_since_wall < self.TURN_LOCKOUT_DURATION)
            
            # --- GLIDE PHASE DETECTION ---
            # During glide, acceleration is very low and steady
            if accel_mag < self.GLIDE_ACCEL_THRESHOLD and gyro_mag < 0.5:
                self.glide_sample_count += 1
                if self.glide_sample_count >= self.GLIDE_SAMPLE_THRESHOLD:
                    self.is_gliding = True
            else:
                # Any significant motion ends glide detection
                self.glide_sample_count = 0
                self.is_gliding = False
            
            # --- UPDATE HISTORY BUFFER ---
            # Add current world_az to buffer BEFORE checking (we check previous samples)
            self.recent_world_az.append(world_az)
            if len(self.recent_world_az) > self.MAX_RECENT_SAMPLES:
                self.recent_world_az.pop(0)

            # Track acceleration magnitude history for impact jerk/spike detection
            if self.prev_accel_mag is None:
                self.prev_accel_mag = accel_mag
            accel_delta = accel_mag - self.prev_accel_mag
            jerk = accel_delta / dt if dt > 0 else 0.0
            self.prev_accel_mag = accel_mag

            self.recent_accel_mag.append(accel_mag)
            if len(self.recent_accel_mag) > self.MAX_RECENT_SAMPLES:
                self.recent_accel_mag.pop(0)
            
            # --- CHECK DOWNWARD MOTION HISTORY (excluding current sample) ---
            # Look at samples BEFORE the current one to verify hand was moving down
            # Need at least WINDOW+1 samples: WINDOW for history + 1 for current
            was_moving_downward = False
            if len(self.recent_world_az) > self.DOWNWARD_SAMPLE_WINDOW:
                # Check samples [-(WINDOW+1) to -1), excluding the current sample [-1]
                # This looks at the motion BEFORE the potential impact
                # Example: buffer=[...old, -2, -3, -1, +5], we check [-2, -3, -1] not +5
                history_samples = self.recent_world_az[-(self.DOWNWARD_SAMPLE_WINDOW + 1):-1]
                downward_count = sum(1 for az_val in history_samples 
                                    if az_val < self.DOWNWARD_ACCEL_THRESHOLD)
                was_moving_downward = (downward_count >= self.DOWNWARD_REQUIRED_COUNT)
            
            # --- CHECK FOR IMPACT SIGNATURE ---
            # Water impact: high acceleration magnitude + direction reversal (now positive)
            # The reversal from negative (down) to positive (deceleration) is key
            is_impact_spike = (
                accel_mag > self.WATER_ENTRY_ACCEL_THRESHOLD
                and world_az > self.IMPACT_REVERSAL_THRESHOLD  # Now decelerating (positive)
            )

            # --- IMPACT FALLBACK: SHARP CHANGE DETECTION (axis-agnostic) ---
            # If a swimmer has poor form and enters at an odd angle, the vertical reversal may be weaker.
            # However, the *impact* still produces a sharp change in acceleration magnitude.
            # We only allow this path when:
            # - magnitude is already above the entry threshold
            # - jerk (or delta) is large (sharp-change)
            # - hand is rotating (entry motion)
            # - NOT in wall/turn lockout and NOT gliding
            # This stays independent of the hand's end position (sideways drift doesn't matter).
            is_impact_by_jerk = (
                accel_mag > self.WATER_ENTRY_ACCEL_THRESHOLD
                and (jerk > self.IMPACT_JERK_THRESHOLD or accel_delta > self.IMPACT_DELTA_A_THRESHOLD)
            )
            
            # --- CHECK STROKE INTERVAL ---
            time_since_last_stroke = (t_ms - self.last_stroke_time_ms) / 1000.0
            interval_ok = (time_since_last_stroke >= self.MIN_STROKE_INTERVAL)
            
            # --- CHECK GYROSCOPE ROTATION ---
            # Hand rotating into water shows characteristic angular velocity
            has_entry_rotation = (gyro_mag > self.ENTRY_GYRO_THRESHOLD)
            
            # --- FINAL STROKE DETECTION ---
            # ALL conditions must be true:
            #   1. was_moving_downward: Hand WAS going down before impact
            #   2. is_impact_spike: Current sample shows water impact signature
            #   3. interval_ok: Enough time since last stroke
            #   4. has_entry_rotation: Hand is rotating (entry motion)
            #   5. NOT in turn lockout: Not immediately after wall impact
            #   6. NOT gliding: Not in passive glide phase
            impact_detected = is_impact_spike or is_impact_by_jerk

            # Primary path: clean downward history + vertical reversal impact
            # Fallback path: sharp-change impact + rotation (handles messy entries)
            stroke_detected = (
                impact_detected
                and interval_ok
                and has_entry_rotation
                and not in_turn_lockout
                and not self.is_gliding
                and (was_moving_downward or is_impact_by_jerk)
            )
            
            # Store debug values for troubleshooting output
            self.debug_world_az = world_az
            self.debug_accel_mag = accel_mag
            self.debug_gyro_mag = gyro_mag
            self.debug_was_downward = was_moving_downward
            self.debug_is_impact = is_impact_spike
            self.debug_jerk = jerk
            self.debug_is_impact_by_jerk = is_impact_by_jerk
            self.debug_in_turn_lockout = in_turn_lockout
            
            if stroke_detected:
                self.stroke_count += 1
                self.last_stroke_time_ms = t_ms
                # Clear buffer to prevent immediate re-trigger
                self.recent_world_az.clear()
                self.glide_sample_count = 0
                # Reset position at each water entry for accurate per-stroke hand motion
                # (hand motion shown from this stroke's entry to the next)
                self.position = [0.0, 0.0, 0.0]
                self.velocity = [0.0, 0.0, 0.0]
                self.in_stroke = True
                self.stroke_start_time = t_ms
            
            # =================================================================
            # POSITION TRACKING STATE (separate from stroke counting)
            # =================================================================
            if not self.in_stroke:
                if (
                    abs(gyro_y) > self.STROKE_START_GYRO_THRESHOLD
                    or accel_mag > self.STROKE_START_ACCEL_THRESHOLD
                ):
                    self.in_stroke = True
                    self.stroke_start_time = t_ms
                    self.position = [0.0, 0.0, 0.0]  # Reset position on new stroke
                    self.velocity = [0.0, 0.0, 0.0] 
            else:
                # Check if we should stop position tracking
                if (
                    abs(gyro_y) < self.STROKE_END_GYRO_THRESHOLD
                    and accel_mag < self.STROKE_END_ACCEL_THRESHOLD
                ):
                    duration_sec = (t_ms - self.stroke_start_time) / 1000.0
                    if duration_sec > self.MIN_STROKE_DURATION or duration_sec > 20.0:
                        self.in_stroke = False
                    else:
                        self.in_stroke = False
            
            # 5. Integration Logic (Only runs if calibrated AND in stroke)
            if self.in_stroke:
                # World-frame acceleration already computed above for stroke detection
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
        vis_data = {
            "timestamp": t_ms,
            "quaternion": {"qw": quat_w, "qx": quat_x, "qy": quat_y, "qz": quat_z},
            "position": {"px": self.position[0], "py": self.position[1], "pz": self.position[2]},
            "calibration": calibration,  # Pass full calibration status to UI
            
            "acceleration": {"ax": lia_x, "ay": lia_y, "az": lia_z},  # Use linear acceleration (LIA)
            "angular_velocity": {"gx": data_dict['gx'], "gy": data_dict['gy'], "gz": data_dict['gz']},
            "tracking_active": self.in_stroke,
            "stroke_count": self.stroke_count,
            "just_reset": self.just_reset,  # Confirms reset to client
            
            # Stroke detection debug info (helps with threshold tuning)
            "stroke_debug": {
                "world_az": round(self.debug_world_az, 2),      # Vertical accel in world frame
                "accel_mag": round(self.debug_accel_mag, 2),    # Total acceleration magnitude
                "gyro_mag": round(self.debug_gyro_mag, 2),      # Angular velocity magnitude
                "jerk": round(self.debug_jerk, 1),              # d|a|/dt (sharp change)
                "was_downward": self.debug_was_downward,        # Was hand moving down before?
                "is_impact": self.debug_is_impact,              # Impact via vertical reversal
                "is_impact_by_jerk": self.debug_is_impact_by_jerk,  # Impact via sharp-change fallback
                "turn_lockout": self.debug_in_turn_lockout,     # In post-turn lockout?
                "is_gliding": self.is_gliding                   # In glide phase?
            }
        }
        
        # Clear the reset flag after sending once
        if self.just_reset:
            self.just_reset = False
        
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
    
    def do_POST(self):
        if self.path == '/reset':
            global stroke_processor_instance
            if stroke_processor_instance:
                stroke_processor_instance.stroke_count = 0
                stroke_processor_instance.position = [0.0, 0.0, 0.0]
                stroke_processor_instance.velocity = [0.0, 0.0, 0.0]
                stroke_processor_instance.in_stroke = False
                stroke_processor_instance.recent_world_az.clear()
                stroke_processor_instance.recent_accel_mag.clear()
                stroke_processor_instance.prev_accel_mag = None
                stroke_processor_instance.last_stroke_time_ms = 0
                stroke_processor_instance.last_wall_impact_time_ms = 0
                stroke_processor_instance.glide_sample_count = 0
                stroke_processor_instance.is_gliding = False
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
                                
                                # Provide helpful hints based on missing calibration
                                hint = ""
                                if cal_accel < 3:
                                    hint = "[TIP: Place sensor in 6 different stable positions for ACCEL]"
                                elif cal_mag < 3:
                                    hint = "[TIP: Move in figure-8 for MAG]"
                                elif cal_gyro < 3:
                                    hint = "[TIP: Leave sensor completely still for GYRO]"
                                
                                # Stroke status with debug info
                                status_str = "ACTIVE" if processor.in_stroke else "IDLE"
                                count_str = f"Strokes: {processor.stroke_count}"
                                
                                # Show stroke detection state for debugging
                                az_str = f"wAz:{processor.debug_world_az:+.1f}"
                                down_str = "DOWN" if processor.debug_was_downward else "----"
                                impact_str = "IMPACT" if processor.debug_is_impact else "------"
                                lock_str = "LOCK" if processor.debug_in_turn_lockout else "----"
                                
                                # Use print() with carriage return to update in-place
                                print(f"CAL: S={cal_sys} G={cal_gyro} A={cal_accel} | {count_str} | {az_str} {down_str} {impact_str} {lock_str} | {hint}          ", end='\r', flush=True)
                                
                                last_cal_print_time = current_time
                                last_cal_status = cal_status
                                
                                if cal_accel >= 2 and cal_gyro >= 2:
                                    print(f"\n--- SENSORS READY (A={cal_accel} G={cal_gyro}) [{status_str}] ---", flush=True)
                                elif cal_accel < 2:
                                    print("\n--- Waiting for Accelerometer (Rotate Sensor) ---", flush=True)

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
    stroke_processor_instance = stroke_processor
    
    receiver_thread = threading.Thread(target=serial_receiver, args=(stroke_processor,), daemon=True)
    receiver_thread.start()
    
    start_web_server(port)