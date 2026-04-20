#!/usr/bin/env python3
"""
IMU Data Visualizer & Calibration Helper

Reads IMU data from ESP32, and provides clear, simple
feedback in the terminal to help with sensor calibration.
"""

import serial
import serial.tools.list_ports
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import sys
import os
import re
import math

def land_demo_enabled(default=True):
    """
    GF_LAND_DEMO: unset → default; '0'/'false'/'no'/'off' → off.
    Dashboard replay defaults to land-friendly impact/downward (GF_LAND_DEMO unset = on).
    """
    v = os.environ.get('GF_LAND_DEMO')
    if v is None:
        return bool(default)
    return v.strip().lower() not in ('0', 'false', 'no', 'off')


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

        # Measurement update (fixed R — matches 0401fbde StrokeProcessor)
        K = self.P / (self.P + self.R)
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        
        return self.X

# --- This class contains your new logic ---
class StrokeProcessor:
    """
    Holds the state for velocity and position integration.
    Detects strokes on DOWNWARD hand entry into water (catch phase).

    Position path: Kalman-smoothed LIA → quaternion to world frame → integrate when ``tracking_active``.

    - **Live (Wi‑Fi / UART, ``batch_mode=False``):** matches the *general* 0401fbde behavior —
      integration is gated by **motion** (``|gy|`` + total LIA magnitude), not by the impact stroke
      detector. Position/velocity reset when a new motion segment **starts** (same idea as the
      old ``in_stroke`` gate in
      https://github.com/chaudhary-nikhil/E22_Senior_Design/tree/0401fbde9e7a69887208db1df3e5feff27011ba6 ).
    - **Batch replay (``batch_mode=True``):** default ``replay_integration_mode=uart_match`` uses the
      same motion gate as live / 0401fbde. Set ``stroke_pull`` or env ``GF_REPLAY_INTEGRATION`` to
      integrate only during impact→pull; ``motion_segment`` keeps the long-segment batch gate.

    The dashboard reads ``position`` + ``tracking_active`` from each JSON sample.
    
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
    def __init__(
        self,
        batch_mode=False,
        replay_integration_mode="uart_match",
        land_demo=False,
        strict_min_cal=False,
    ):
        """
        batch_mode: When True (post-session playback), use slightly more lenient thresholds
        to improve stroke detection on recorded data which may differ from live stream.

        replay_integration_mode (batch only; default uart_match = 0401fbde / live UART):
            uart_match — same motion gate as live serial (abs(gy)+|LIA|), independent of impact.
            stroke_pull — integrate only during impact→pull window (stroke-anchored trails).
            motion_segment — long motion segments (older batch experiment).

        land_demo: Softer impact/downward/jerk thresholds and disables glide lockout so dry-land
            swings still register as strokes. Position integration still follows ``replay_integration_mode``
            (default ``uart_match`` = same gate as live / 0401fbde).

        strict_min_cal: If True, require accel+gyro cal ≥2 before stroke/integration (0401fbde).
            Live serial defaults False; dashboard batch replay defaults strict via
            ``wifi_session_processor._replay_stroke_processor`` unless land demo or
            ``GF_RELAX_IMU_CAL``.
        """
        self.batch_mode = batch_mode
        self.replay_integration_mode = (replay_integration_mode or "uart_match").strip().lower()
        if self.replay_integration_mode not in ("stroke_pull", "uart_match", "motion_segment"):
            self.replay_integration_mode = "uart_match"
        self.land_demo = bool(land_demo)

        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.last_timestamp_ms = None
        self.in_stroke = False
        self.just_reset = False

        # Kalman Filters for Acceleration (X, Y, Z)
        # TUNING for HUMAN MOTION:
        # Q (Process Noise): 0.3 -> High because human motion changes accel rapidly.
        # R (Measurement Noise): 0.1 -> Low because BNO055 is relatively precise.
        self.kalman_ax = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_ay = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)
        self.kalman_az = SimpleKalmanFilter(0.3, 0.1, 1.0, 0.0)

        # --- DRIFT-KILL: slow baseline (EMA) for world-frame accel -----------
        # Double-integrating even a 0.05 m/s² residual (gravity bleed from
        # quaternion yaw drift in IMU+ mode) generates ~0.1 m of bogus curve
        # per stroke, which shows up as the "smooth elliptical loop" instead
        # of the real S-shaped pull path. Subtracting a slow EMA (τ≈2 s)
        # behaves like a 1st-order high-pass with fc ≈ 0.08 Hz -- well below
        # stroke frequency (0.4-1 Hz) so real motion passes untouched.
        self._ema_int_ax = 0.0
        self._ema_int_ay = 0.0
        self._ema_int_az = 0.0
        self.WORLD_ACCEL_HPF_TAU_S = 2.0
        
        
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
        # One physical stroke = pull + recovery. Recovery can produce a second
        # accel/gyro peak; we must not count it. Minimum interval ensures only
        # one stroke per arm cycle. Typical stroke 0.8-1.5s; fast ~0.7s.
        self.MIN_STROKE_INTERVAL = 1.1  # seconds — avoids counting return/recovery as a stroke
        
        # TURN/WALL DETECTION (prevent false positives at wall)
        # Flip turn: 20-40 m/s² impact. Open turn: 15-25 m/s² (hand contacts wall).
        # Normal swimming strokes: 5-12 m/s². Vigorous dry-land strokes: up to 15 m/s².
        # Must be well above stroke range to avoid false positives.
        self.WALL_IMPACT_THRESHOLD = 34.0  # m/s² — above typical stroke LIA peaks; real wall push is higher
        self.WALL_SUSTAINED_COUNT = 4  # require N consecutive high-accel samples (stricter than strokes)
        self.STROKE_TURN_COOLDOWN_MS = 2000  # ignore wall/turn shortly after a stroke impact
        self.TURN_LOCKOUT_DURATION = 2.5  # seconds
        self.last_wall_impact_time_ms = 0
        self.wall_high_count = 0
        self.turn_count = 0
        
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
        
        # INTEGRATION WINDOW: Only integrate during active stroke pull-through.
        # Typical arm pull: 0.3-0.5s. After that, hand exits water (recovery).
        # We must NOT track recovery, only the underwater pull.
        # Pull-through window: slightly longer when fusion is strong (matches live UART feel
        # from commit 0401fbde — gyro-led tracking had longer apparent motion)
        self.STROKE_INTEGRATION_TIMEOUT = 0.6  # default hard cutoff (seconds)
        self.STROKE_INTEGRATION_TIMEOUT_WELL_CAL = 0.72  # when accel+gyro cal >= 2
        self.STROKE_SETTLE_ACCEL = 0.8  # m/s² - accel below this = motion settling
        self.STROKE_SETTLE_REQUIRED = 3  # consecutive quiet samples to end integration
        self.stroke_settle_count = 0
        self.stroke_integrating = False
        self.stroke_integration_start = 0

        # 0401fbde position tracking gate (gyro/accel motion-gated, primary integration window).
        # Matches the original StrokeProcessor that produced the most accurate stroke visualization:
        # abs(gy) + accel_mag thresholds, position/velocity reset on segment start, instant end.
        self.pos_tracking_active = False
        self.pos_tracking_start_ms = 0
        self.POS_TRACK_START_GYRO = 0.6   # rad/s — abs(gy), 0401fbde STROKE_START_GYRO_THRESHOLD
        self.POS_TRACK_START_ACCEL = 0.25  # m/s², 0401fbde STROKE_START_ACCEL_THRESHOLD
        self.POS_TRACK_END_GYRO = 0.2      # rad/s, 0401fbde STROKE_END_GYRO_THRESHOLD
        self.POS_TRACK_END_ACCEL = 0.25    # m/s², 0401fbde STROKE_END_ACCEL_THRESHOLD
        self.POS_TRACK_MIN_MS = 500        # 0401fbde MIN_STROKE_DURATION = 0.5s
        
        # 0 = always run fusion path (IMUPLUS-friendly); 2 = match strict 0401fbde bench policy.
        self.MIN_CAL_LEVEL = 2 if strict_min_cal else 0
        # 0401fbde integration constants (used directly in process_data)
        self.ACCEL_DEADZONE = 0.2  # m/s² per-axis — 0401fbde ACCEL_DEADZONE
        self.VELOCITY_DECAY = 0.98  # 0401fbde VELOCITY_DECAY (active motion)
        self.STATIONARY_FRICTION = 0.80  # 0401fbde STATIONARY_FRICTION (quiet)

        self.stroke_count = 0  # Track number of strokes
        self.stroke_start_time = 0  # To track duration
        # MIN_STROKE_DURATION removed - integration window uses settle detection instead
        
        # ANGLE OF ATTACK (entry angle from quaternion pitch at stroke start)
        # Research: eolab SwimBETTER tracks hand angle at water entry via gyroscope.
        # We compute it from the quaternion pitch at the exact sample where stroke detected.
        # Ideal freestyle entry angle: ~15-40 degrees (fingertips-first)
        self.last_entry_angle = 0.0
        self.entry_angles = []  # All entry angles this session
        self.ideal_entry_angle = 30.0  # degrees, configurable per user
        # Gyro ‖ω‖ (rad/s) at water-entry sample — PDP: angle of attack + rotation at impact
        self.last_entry_gyro_mag = 0.0
        self.last_entry_gx = self.last_entry_gy = self.last_entry_gz = 0.0
        
        # STROKE PHASE TRACKING (Research: IEEE Swimming Phase Segmentation,
        # Frontiers fbioe.2021.793302 — gyroscope angular velocity is the primary
        # discriminator between underwater (catch/pull) and aerial (recovery) phases)
        #
        # Phases: catch, pull, recovery, glide
        #   catch: hand entering water — high gyro + downward accel (short ~0.1s)
        #   pull: hand pulling through water — moderate gyro, high LIA, integration active
        #   recovery: arm swinging above water — high gyro, low accel (no water resistance)
        #   glide: low accel + low gyro, streamline position
        #
        # Key insight: during recovery the hand moves fast but through air, so accel
        # magnitude is LOW despite HIGH gyro. During pull, water resistance creates
        # HIGH accel with moderate gyro. This gyro/accel ratio distinguishes phases.
        self.current_phase = 'idle'
        self.phase_start_time = 0
        self.phase_durations = {'glide': [], 'catch': [], 'pull': [], 'recovery': []}
        self.last_phase_pcts = {'glide': 0, 'catch': 0, 'pull': 0, 'recovery': 0}
        self.CATCH_DURATION_MS = 150  # catch phase lasts ~100-200ms after stroke detection
        self.RECOVERY_GYRO_THRESHOLD = 1.5  # rad/s — arm swinging above water
        self.RECOVERY_ACCEL_CEILING = 3.0   # m/s² — low accel during aerial recovery
        
        # DEBUG: Track stroke detection state for troubleshooting
        self.debug_world_az = 0.0
        self.debug_accel_mag = 0.0
        self.debug_gyro_mag = 0.0
        self.debug_was_downward = False
        self.debug_is_impact = False
        self.debug_jerk = 0.0
        self.debug_is_impact_by_jerk = False
        self.debug_in_turn_lockout = False

        # --- Live visualization gate (0401fbde-style): separate from impact stroke counting ---
        self.legacy_pos_track = False
        self.legacy_viz_start_ms = 0
        self._legacy_quiet_streak = 0  # end segment only after N consecutive quiet samples (anti-flutter)
        self.LEGACY_VIZ_START_GYRO = 0.6  # rad/s — abs(gy)
        self.LEGACY_VIZ_START_ACCEL = 0.25  # m/s² — |LIA| magnitude
        self.LEGACY_VIZ_END_GYRO = 0.2
        self.LEGACY_VIZ_END_ACCEL = 0.25
        self.LEGACY_VIZ_MIN_DURATION_S = 0.5  # require brief motion before “quiet” can end segment
        self.LEGACY_VIZ_QUIET_STREAK = 3  # ~30ms @100Hz — avoids gate chatter / drift ticks

        if self.land_demo:
            # Dry-land: same impact/downward/jerk logic as pool, but thresholds match air + desk motion.
            self.WATER_ENTRY_ACCEL_THRESHOLD = min(self.WATER_ENTRY_ACCEL_THRESHOLD, 3.0)
            self.IMPACT_REVERSAL_THRESHOLD = min(self.IMPACT_REVERSAL_THRESHOLD, 0.95)
            self.IMPACT_JERK_THRESHOLD = min(self.IMPACT_JERK_THRESHOLD, 90.0)
            self.IMPACT_DELTA_A_THRESHOLD = min(self.IMPACT_DELTA_A_THRESHOLD, 2.2)
            self.ENTRY_GYRO_THRESHOLD = min(self.ENTRY_GYRO_THRESHOLD, 0.32)
            self.DOWNWARD_REQUIRED_COUNT = min(self.DOWNWARD_REQUIRED_COUNT, 2)
            # Easier “was moving down” in fusion world-Z (standing demo, not pool gravity).
            self.DOWNWARD_ACCEL_THRESHOLD = max(self.DOWNWARD_ACCEL_THRESHOLD, -0.42)

    def _update_uart_motion_gate(self, t_ms, gy_abs, accel_mag):
        """0401fbde / serial-live motion gate: start integration on motion, end on quiet + min time."""
        if not self.legacy_pos_track:
            if (
                gy_abs > self.LEGACY_VIZ_START_GYRO
                or accel_mag > self.LEGACY_VIZ_START_ACCEL
            ):
                self.legacy_pos_track = True
                self.legacy_viz_start_ms = t_ms
                self._legacy_quiet_streak = 0
                # 0401fbde: zero position + velocity at each new motion segment (kills IMU bias drift
                # between bursts). Same for live and batch replay — per-stroke viz uses JS origin
                # subtraction + catch/pull anchor, not one long unbroken integral.
                self.position = [0.0, 0.0, 0.0]
                self.velocity = [0.0, 0.0, 0.0]
        else:
            dur_s = (t_ms - self.legacy_viz_start_ms) / 1000.0
            quiet = (
                gy_abs < self.LEGACY_VIZ_END_GYRO
                and accel_mag < self.LEGACY_VIZ_END_ACCEL
            )
            if quiet:
                self._legacy_quiet_streak += 1
            else:
                self._legacy_quiet_streak = 0
            if (
                self._legacy_quiet_streak >= self.LEGACY_VIZ_QUIET_STREAK
                and dur_s >= self.LEGACY_VIZ_MIN_DURATION_S
            ):
                self.legacy_pos_track = False
                self._legacy_quiet_streak = 0
            elif dur_s > 25.0:
                self.legacy_pos_track = False
                self._legacy_quiet_streak = 0

    def _tracking_active_for_output(self):
        if not self.batch_mode:
            return self.legacy_pos_track
        if self.replay_integration_mode == "uart_match":
            return self.legacy_pos_track
        if self.replay_integration_mode == "stroke_pull":
            return self.stroke_integrating
        return self.pos_tracking_active

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
            
            calibration = data_dict.get('cal') or data_dict.get('calibration', {})
            cal_sys = calibration.get('sys', 0)
            cal_accel = calibration.get('accel', 0)
            cal_gyro = calibration.get('gyro', 0)
            cal_mag = calibration.get('mag', 0)

        except KeyError as e:
            print(f"ERROR: Missing key {e}. Check sensor data format.")
            return None

        # 3. --- Check Calibration BEFORE doing any logic ---
        # BNO055 TIP: "System" status (sys) often drops to 0 during movement.
        # We should NOT stop tracking just because SYS=0, provided Accel/Gyro are good.
        if (
            cal_accel < self.MIN_CAL_LEVEL
            or cal_gyro < self.MIN_CAL_LEVEL
        ):
            self.in_stroke = False
            self.stroke_integrating = False
            self.legacy_pos_track = False
            self._legacy_quiet_streak = 0
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
        else:
            # --- System is sufficiently calibrated, run stroke logic ---
            # 0401fbde order: Kalman-smooth LIA first, then use smoothed LIA for detection + integration.
            smooth_x = self.kalman_ax.update(lia_x)
            smooth_y = self.kalman_ay.update(lia_y)
            smooth_z = self.kalman_az.update(lia_z)

            accel_mag = math.sqrt(smooth_x * smooth_x + smooth_y * smooth_y + smooth_z * smooth_z)
            
            # =================================================================
            # COMPUTE WORLD-FRAME ACCELERATION (stroke detection + integration)
            # =================================================================
            # Transform sensor-frame linear acceleration to world frame using quaternion
            qw, qx, qy, qz = quat_w, quat_x, quat_y, quat_z
            ax, ay, az = smooth_x, smooth_y, smooth_z

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
            
            time_since_wall = (t_ms - self.last_wall_impact_time_ms) / 1000.0 if self.last_wall_impact_time_ms else 999.0
            in_turn_lockout = (self.last_wall_impact_time_ms > 0 and time_since_wall < self.TURN_LOCKOUT_DURATION)
            
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
            #   3. interval_ok: Enough time since last stroke (avoids counting return/recovery)
            #   4. has_entry_rotation: Hand is rotating (entry motion)
            #   5. NOT in turn lockout: Not immediately after wall impact
            #   6. NOT gliding: Not in passive glide phase
            #   7. NOT still in current stroke: No double-count during same arm cycle
            impact_detected = is_impact_spike or is_impact_by_jerk

            # Primary path: clean downward history + vertical reversal impact
            # Fallback path: sharp-change impact + rotation (handles messy entries)
            stroke_detected = (
                impact_detected
                and interval_ok
                and has_entry_rotation
                and not in_turn_lockout
                and (not self.is_gliding or self.land_demo)
                and not self.stroke_integrating  # ignore peaks during pull-through/recovery of same stroke
                # Require true downward→impact signature; "jerk-only" peaks overcount strokes on land.
                and (was_moving_downward or is_impact_spike)
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
                self.wall_high_count = 0
                self.recent_world_az.clear()
                self.glide_sample_count = 0
                # Per-stroke reset (all modes): 0401fbde's live UART behaviour relies on the
                # motion gate opening and closing between bursts. In continuous swimming the
                # gate stays open for the entire lap, so drift accumulates for many seconds.
                # Zeroing position+velocity at each impact anchors every stroke to a clean
                # origin, which matches the JS `applyPerStrokeOriginOffset` behaviour and
                # keeps each stroke's integrated shape independent of cumulative bias.
                self.position = [0.0, 0.0, 0.0]
                self.velocity = [0.0, 0.0, 0.0]
                self.in_stroke = True
                self.stroke_start_time = t_ms
                self.stroke_integrating = True
                self.stroke_integration_start = t_ms
                self.stroke_settle_count = 0
                
                # ANGLE OF ATTACK: compute hand pitch from quaternion at entry
                # Euler pitch from quaternion: pitch = asin(2*(qw*qy - qz*qx))
                sinp = 2.0 * (qw * qy - qz * qx)
                sinp = max(-1.0, min(1.0, sinp))  # clamp for asin
                pitch_rad = math.asin(sinp)
                self.last_entry_angle = abs(math.degrees(pitch_rad))
                self.entry_angles.append(self.last_entry_angle)
                self.last_entry_gyro_mag = gyro_mag
                self.last_entry_gx = float(data_dict.get('gx', 0) or 0)
                self.last_entry_gy = float(data_dict.get('gy', 0) or 0)
                self.last_entry_gz = float(data_dict.get('gz', 0) or 0)

            # --- WALL/TURN (after stroke): vigorous strokes must not register as turns ---
            since_stroke_ms = (t_ms - self.last_stroke_time_ms) if self.last_stroke_time_ms else self.STROKE_TURN_COOLDOWN_MS + 1
            if since_stroke_ms >= self.STROKE_TURN_COOLDOWN_MS:
                if accel_mag > self.WALL_IMPACT_THRESHOLD:
                    self.wall_high_count += 1
                    if self.wall_high_count >= self.WALL_SUSTAINED_COUNT:
                        if (t_ms - self.last_wall_impact_time_ms) / 1000.0 > self.TURN_LOCKOUT_DURATION:
                            self.turn_count += 1
                        self.last_wall_impact_time_ms = t_ms
                        self.recent_world_az.clear()
                        self.wall_high_count = 0
                else:
                    self.wall_high_count = 0
            else:
                self.wall_high_count = 0
            
            # --- STROKE PHASE TRACKING (gyroscope-enhanced, runs AFTER stroke
            # detection so stroke_integrating/stroke_start_time are up-to-date) ---
            # Research: IEEE Swimming Phase Segmentation (2020), Frontiers
            # fbioe.2021.793302 — gyroscope angular velocity is the primary
            # discriminator between underwater and aerial phases.
            prev_phase = self.current_phase
            time_since_stroke_ms = (t_ms - self.stroke_start_time) if self.stroke_start_time > 0 else 999999
            RECOVERY_MAX_MS = 600  # recovery lasts at most ~0.6s after pull ends

            if self.is_gliding and not self.stroke_integrating:
                new_phase = 'glide'
            elif self.stroke_integrating and time_since_stroke_ms < self.CATCH_DURATION_MS:
                new_phase = 'catch'
            elif self.stroke_integrating:
                new_phase = 'pull'
            elif (self.stroke_count > 0
                  and not self.stroke_integrating
                  and not self.is_gliding
                  and gyro_mag > self.RECOVERY_GYRO_THRESHOLD
                  and time_since_stroke_ms < (self.STROKE_INTEGRATION_TIMEOUT * 1000 + RECOVERY_MAX_MS)):
                new_phase = 'recovery'
            elif (self.stroke_count > 0
                  and not self.stroke_integrating
                  and not self.is_gliding
                  and time_since_stroke_ms < (self.STROKE_INTEGRATION_TIMEOUT * 1000 + RECOVERY_MAX_MS)
                  and accel_mag < self.RECOVERY_ACCEL_CEILING):
                new_phase = 'recovery'
            elif self.stroke_count > 0 and not self.stroke_integrating:
                new_phase = 'glide'
            else:
                new_phase = 'idle'

            if new_phase != prev_phase and prev_phase in ('glide', 'catch', 'pull', 'recovery'):
                phase_dur = (t_ms - self.phase_start_time) / 1000.0
                if phase_dur > 0.05:
                    self.phase_durations[prev_phase].append(phase_dur)
                self.phase_start_time = t_ms
            elif new_phase != prev_phase:
                self.phase_start_time = t_ms
            self.current_phase = new_phase

            recent_n = 5
            total_dur = 0
            for ph in ('glide', 'catch', 'pull', 'recovery'):
                durs = self.phase_durations[ph][-recent_n:]
                total_dur += sum(durs)
            if total_dur > 0:
                for ph in ('glide', 'catch', 'pull', 'recovery'):
                    durs = self.phase_durations[ph][-recent_n:]
                    self.last_phase_pcts[ph] = round(sum(durs) / total_dur * 100, 1)

            # =================================================================
            # INTEGRATION WINDOW MANAGEMENT
            # Only integrate during the active pull-through phase of a stroke.
            # Starts at stroke detection (impact). Ends when acceleration
            # settles for N consecutive samples, or hard timeout (0.6s).
            # Gyro is NOT used for ending (recovery rotation ≠ position motion).
            # =================================================================
            if self.stroke_integrating:
                time_since_stroke = (t_ms - self.stroke_integration_start) / 1000.0
                well_cal_integrate = cal_accel >= 2 and cal_gyro >= 2
                t_limit = (
                    self.STROKE_INTEGRATION_TIMEOUT_WELL_CAL
                    if well_cal_integrate
                    else self.STROKE_INTEGRATION_TIMEOUT
                )
                if time_since_stroke > t_limit:
                    self.stroke_integrating = False
                    self.in_stroke = False
                    self.stroke_settle_count = 0
                elif time_since_stroke > 0.15:
                    if accel_mag < self.STROKE_SETTLE_ACCEL:
                        self.stroke_settle_count += 1
                        if self.stroke_settle_count >= self.STROKE_SETTLE_REQUIRED:
                            self.stroke_integrating = False
                            self.in_stroke = False
                    else:
                        self.stroke_settle_count = 0
            else:
                self.in_stroke = False

            # =================================================================
            # 0401fbde UART-style motion gate (live always; batch when replay_integration_mode=uart_match).
            # =================================================================
            gy_abs = abs(float(gyro_y))
            if (not self.batch_mode) or self.replay_integration_mode == "uart_match":
                self._update_uart_motion_gate(t_ms, gy_abs, accel_mag)
            elif self.batch_mode:
                self.legacy_pos_track = False

            # =================================================================
            # POSITION TRACKING GATE (batch replay, motion_segment only)
            # Long motion segments — can desync from per-stroke impact windows.
            # =================================================================
            if self.batch_mode and self.replay_integration_mode == "motion_segment":
                if not self.pos_tracking_active:
                    if (gy_abs > self.POS_TRACK_START_GYRO
                            or accel_mag > self.POS_TRACK_START_ACCEL):
                        self.pos_tracking_active = True
                        self.pos_tracking_start_ms = t_ms
                        self.position = [0.0, 0.0, 0.0]
                        self.velocity = [0.0, 0.0, 0.0]
                else:
                    quiet = (gy_abs < self.POS_TRACK_END_GYRO
                             and accel_mag < self.POS_TRACK_END_ACCEL)
                    duration_ms = t_ms - self.pos_tracking_start_ms
                    if quiet and duration_ms >= self.POS_TRACK_MIN_MS:
                        self.pos_tracking_active = False
                    elif duration_ms > 20000:
                        self.pos_tracking_active = False
            elif self.batch_mode:
                self.pos_tracking_active = False
            
            # 5. Integration Logic — 0401fbde StrokeProcessor math
            # Kalman-smoothed LIA → quaternion to world frame → per-axis deadzone → integrate.
            # Constants match the original 0401fbde commit (most accurate stroke visualization).
            int_ax = (ww + xx - yy - zz) * smooth_x + 2 * (xy - wz) * smooth_y + 2 * (xz + wy) * smooth_z
            int_ay = 2 * (xy + wz) * smooth_x + (ww - xx + yy - zz) * smooth_y + 2 * (yz - wx) * smooth_z
            int_az = 2 * (xz - wy) * smooth_x + 2 * (yz + wx) * smooth_y + (ww - xx - yy + zz) * smooth_z

            # HPF on world-frame accel: subtract slow EMA baseline. This removes
            # any residual gravity / quaternion-drift bias before double-integration,
            # which is what produces the "smooth loop" drift artifact instead of the
            # real back-and-forth S-curve of a pull. Adaptive to dt so variable rate
            # is handled. Continuously updated (even when not integrating) so the
            # baseline stays accurate when a segment opens.
            tau = self.WORLD_ACCEL_HPF_TAU_S
            if tau > 0.0 and dt > 0.0:
                alpha = min(1.0, dt / tau)
                self._ema_int_ax += alpha * (int_ax - self._ema_int_ax)
                self._ema_int_ay += alpha * (int_ay - self._ema_int_ay)
                self._ema_int_az += alpha * (int_az - self._ema_int_az)
                int_ax -= self._ema_int_ax
                int_ay -= self._ema_int_ay
                int_az -= self._ema_int_az

            if not self.batch_mode:
                integrate_now = self.legacy_pos_track
            elif self.replay_integration_mode == "uart_match":
                integrate_now = self.legacy_pos_track
            elif self.replay_integration_mode == "stroke_pull":
                integrate_now = self.stroke_integrating
            else:
                integrate_now = self.pos_tracking_active
            if integrate_now:
                DEADZONE = 0.2   # 0401fbde ACCEL_DEADZONE
                DECAY_ACTIVE = 0.98   # 0401fbde VELOCITY_DECAY
                DECAY_STATIONARY = 0.80  # 0401fbde STATIONARY_FRICTION

                is_accelerating = False
                if abs(int_ax) < DEADZONE:
                    int_ax = 0.0
                else:
                    is_accelerating = True
                if abs(int_ay) < DEADZONE:
                    int_ay = 0.0
                else:
                    is_accelerating = True
                if abs(int_az) < DEADZONE:
                    int_az = 0.0
                else:
                    is_accelerating = True

                self.velocity[0] += int_ax * dt
                self.velocity[1] += int_ay * dt
                self.velocity[2] += int_az * dt

                decay = DECAY_ACTIVE if is_accelerating else DECAY_STATIONARY
                self.velocity[0] *= decay
                self.velocity[1] *= decay
                self.velocity[2] *= decay

                self.position[0] += self.velocity[0] * dt
                self.position[1] += self.velocity[1] * dt
                self.position[2] += self.velocity[2] * dt
            else:
                self.velocity = [0.0, 0.0, 0.0]

        stroke_vel_ms = math.sqrt(self.velocity[0]**2 + self.velocity[1]**2 + self.velocity[2]**2)
        
        # Calculate phase progress
        phase_progress = 0.0
        if self.current_phase != 'idle':
            current_duration = (t_ms - self.phase_start_time) / 1000.0
            history = self.phase_durations[self.current_phase][-5:]
            if history:
                avg_dur = sum(history) / len(history)
                phase_progress = min(1.0, current_duration / avg_dur)
            else:
                fallbacks = {'catch': 0.15, 'pull': 0.4, 'recovery': 0.5, 'glide': 1.0}
                phase_progress = min(1.0, current_duration / fallbacks.get(self.current_phase, 1.0))

        # 6. Create the final JSON for the visualizer
        avg_entry = sum(self.entry_angles) / len(self.entry_angles) if self.entry_angles else 0
        fw_entry = float(data_dict.get('entry_angle', 0) or 0)
        display_entry = round(fw_entry, 1) if fw_entry > 0.05 else round(self.last_entry_angle, 1)
        fw_strokes = int(data_dict.get('strokes', 0) or 0)
        vis_data = {
            "timestamp": t_ms,
            "quaternion": {"qw": quat_w, "qx": quat_x, "qy": quat_y, "qz": quat_z},
            "position": {"px": self.position[0], "py": self.position[1], "pz": self.position[2]},
            "calibration": calibration,  # Pass full calibration status to UI
            
            "acceleration": {"ax": lia_x, "ay": lia_y, "az": lia_z},  # Use linear acceleration (LIA)
            "angular_velocity": {"gx": data_dict['gx'], "gy": data_dict['gy'], "gz": data_dict['gz']},
            "magnetometer": {
                "mx": float(data_dict.get('mx', 0) or 0),
                "my": float(data_dict.get('my', 0) or 0),
                "mz": float(data_dict.get('mz', 0) or 0),
            },
            "tracking_active": self._tracking_active_for_output(),
            "stroke_count": self.stroke_count,
            "strokes": fw_strokes,
            "turn_count": self.turn_count,
            "just_reset": self.just_reset,
            
            # Angle of attack (prefer firmware-reported angle at impact)
            "entry_angle": display_entry,
            "avg_entry_angle": round(avg_entry, 1),
            "ideal_entry_angle": self.ideal_entry_angle,
            "entry_gyro_mag": round(self.last_entry_gyro_mag, 4),
            "entry_gyro": {
                "gx": round(self.last_entry_gx, 4),
                "gy": round(self.last_entry_gy, 4),
                "gz": round(self.last_entry_gz, 4),
            },
            
            # Stroke phase & Signal processing
            "stroke_phase": self.current_phase,
            "phase_pcts": dict(self.last_phase_pcts),
            "phase_progress": round(phase_progress, 3),
            "stroke_velocity_ms": round(stroke_vel_ms, 3),
            "position_integration": (
                "0401fbde+" + self.replay_integration_mode if self.batch_mode else "0401fbde+live_uart_gate"
            ),
            "replay_integration_mode": self.replay_integration_mode if self.batch_mode else "live_uart_gate",
            "land_demo": self.land_demo,
            "in_water_pull": bool(self.stroke_integrating),
            
            "haptic_fired": data_dict.get('haptic_fired', data_dict.get('haptic', False)),
            "deviation_score": data_dict.get('deviation_score', data_dict.get('deviation', 0.0)),
            "haptic_reason": data_dict.get('haptic_reason', 0),
            "pull_duration_ms": data_dict.get('pull_duration_ms', 0.0),
            
            # Module/Device ID 
            "device_id": data_dict.get('dev_id', 0),
            "device_role": data_dict.get('dev_role', 0),
            
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
        
        # Clear the reset flag after sending once
        if self.just_reset:
            self.just_reset = False
        
        return json.dumps(vis_data)


def align_sessions_by_hop(sessions):
    """
    Align multiple device sessions by detecting a synchronization hop/jump.
    
    Each device's data should contain a deliberate high acceleration spike
    (hop) in the first 30 seconds. We find the peak LIA magnitude for each
    session, compute timestamp offsets relative to the first session,
    and shift all timestamps accordingly.
    
    Args:
        sessions: list of dicts, each with 'data' (list of sample dicts with 't', 'lia_x', 'lia_y', 'lia_z')
    
    Returns:
        list of sessions with aligned timestamps, plus alignment metadata
    """
    if len(sessions) < 2:
        return sessions, {'aligned': False, 'reason': 'Need at least 2 sessions'}

    hop_times = []
    hop_mags = []
    WINDOW_MS = 30000  # Search first 30 seconds

    for sess in sessions:
        data = sess.get('data', [])
        if not data:
            hop_times.append(0)
            hop_mags.append(0)
            continue

        t0 = data[0].get('t', 0)
        peak_mag = 0
        peak_time = t0

        for s in data:
            t = s.get('t', 0)
            if (t - t0) > WINDOW_MS:
                break
            lx = s.get('lia_x', s.get('ax', 0))
            ly = s.get('lia_y', s.get('ay', 0))
            lz = s.get('lia_z', s.get('az', 0))
            mag = math.sqrt(lx * lx + ly * ly + lz * lz)
            if mag > peak_mag:
                peak_mag = mag
                peak_time = t

        hop_times.append(peak_time)
        hop_mags.append(peak_mag)

    # Compute offsets relative to first session
    reference_time = hop_times[0]
    offsets = [ht - reference_time for ht in hop_times]

    # Apply offsets
    aligned_sessions = []
    for i, sess in enumerate(sessions):
        new_sess = dict(sess)
        new_data = []
        for s in sess.get('data', []):
            ns = dict(s)
            ns['t'] = s.get('t', 0) - offsets[i]
            new_data.append(ns)
        new_sess['data'] = new_data
        aligned_sessions.append(new_sess)

    meta = {
        'aligned': True,
        'offsets_ms': offsets,
        'hop_magnitudes': [round(m, 2) for m in hop_mags],
        'quality': 'good' if all(m > 5.0 for m in hop_mags) else 'weak'
    }
    return aligned_sessions, meta


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

            html_path = os.path.join(os.path.dirname(__file__), 'simple_imu_3d.html')
            try:
                with open(html_path, 'rb') as f:
                    self.wfile.write(f.read())
            except (BrokenPipeError, ConnectionResetError, OSError):
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
                stroke_processor_instance.legacy_pos_track = False
                stroke_processor_instance._legacy_quiet_streak = 0
                stroke_processor_instance.in_stroke = False
                stroke_processor_instance.stroke_integrating = False
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

def get_serial_port_list():
    """Discover serial ports, excluding Bluetooth. Prefer known USB-serial (ESP32)."""
    # Skip Bluetooth and virtual ports that are never the ESP32
    skip_substrings = (
        "Bluetooth", "BLTH", "Bluetooth-Incoming", "BTSerial",
        "debug", "Dial-in", "modem", "rfcomm",
    )
    def is_likely_esp32(port_path):
        p = port_path.upper()
        for skip in skip_substrings:
            if skip.upper() in p:
                return False
        return True
    ports = [p.device for p in serial.tools.list_ports.comports() if is_likely_esp32(p.device)]
    # Prefer known USB-serial adapters (idf.py / ESP32 typical names)
    def priority(path):
        if "SLAB_USBtoUART" in path or "usbserial" in path or "wchusbserial" in path:
            return 0
        if sys.platform == "darwin" and path.startswith("/dev/cu."):
            return 1
        return 2
    ports.sort(key=lambda x: (priority(x), x))
    return ports


# --- Serial Receiver (MODIFIED to be quiet) ---
def serial_receiver(processor: StrokeProcessor, preferred_port=None):
    global serial_port, last_cal_print_time, last_cal_status

    while True:
        if serial_port is None:
            if preferred_port:
                possible_ports = [preferred_port]
                print(f"Using port: {preferred_port}", flush=True)
            else:
                possible_ports = get_serial_port_list()
            print("Attempting to connect to ESP32...", flush=True)
            if not possible_ports:
                print("No serial ports found. Is the ESP32 plugged in (USB data cable)?", flush=True)
            else:
                print(f"Trying ports: {possible_ports}", flush=True)
            for port in possible_ports:
                try:
                    serial_port = serial.Serial(port, 115200, timeout=1)
                    print(f"Connected to ESP32 on {port}", flush=True)
                    break
                except serial.SerialException as e:
                    print(f"  {port}: {e}", flush=True)
                    serial_port = None
                except Exception as e:
                    print(f"  {port}: {e}", flush=True)
                    serial_port = None

            if serial_port is None:
                print("ERROR: Could not open any port. Unplug other serial apps (monitor, Arduino IDE) or specify port: python3 simple_imu_visualizer.py <port>", flush=True)
                print("Retrying in 5 seconds.", flush=True)
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
    
    default_http_port = 8003
    http_port = int(os.environ.get("PORT", default_http_port))
    serial_port_arg = None
    if len(sys.argv) >= 2:
        arg1 = sys.argv[1]
        if arg1.startswith("/dev/") or arg1.upper().startswith("COM"):
            serial_port_arg = arg1
            if len(sys.argv) >= 3:
                try:
                    http_port = int(sys.argv[2])
                except ValueError:
                    pass
        else:
            try:
                http_port = int(arg1)
            except ValueError:
                print(f"Invalid argument '{arg1}'. Use: python3 simple_imu_visualizer.py [serial_port] [http_port]", flush=True)
    
    stroke_processor = StrokeProcessor(land_demo=land_demo_enabled(default=True))
    stroke_processor_instance = stroke_processor
    
    receiver_thread = threading.Thread(target=serial_receiver, args=(stroke_processor, serial_port_arg), daemon=True)
    receiver_thread.start()
    
    start_web_server(http_port)