import json
import math
import time
from datetime import datetime
import os
import sqlite3
import random

ROLE_HEAD = 3
ROLE_WRIST_LEFT = 1
ROLE_WRIST_RIGHT = 2

STROKE_CYCLE_SAMPLES = 50  # samples per full stroke cycle at 50 Hz = 1 second

def _quat_from_euler(roll_deg, pitch_deg, yaw_deg):
    """Euler angles (degrees) -> quaternion (w, x, y, z)."""
    r = math.radians(roll_deg) / 2
    p = math.radians(pitch_deg) / 2
    y = math.radians(yaw_deg) / 2
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )

def generate_mock_session(role, num_strokes=8, sample_hz=50):
    """Generate a realistic freestyle wrist-mounted IMU session.

    Each stroke cycle is divided into biomechanically accurate phases:
      catch   (8%)  – hand enters water, brief impact spike
      pull    (34%) – underwater pull, moderate-high accel, low gyro
      recovery(33%) – arm in air, high gyro, low accel
      glide   (25%) – streamlined coast, low accel & gyro
    Quaternions rotate realistically through the stroke arc.
    """
    data = []
    base_t = int(time.time() * 1000) - 120000
    dt_ms = 1000 // sample_hz

    glide_before = 30  # glide samples before first stroke
    glide_after = 20   # glide samples after last stroke

    stroke_count = 0
    sample_idx = 0

    def _add_sample(t, ax, ay, az, gx, gy, gz, qw, qx, qy, qz, phase,
                    entry_angle=0.0, haptic=False, deviation=0.0):
        nonlocal sample_idx
        data.append({
            't': t, 'timestamp': t,
            'ax': round(ax, 3), 'ay': round(ay, 3), 'az': round(az, 3),
            'gx': round(gx, 3), 'gy': round(gy, 3), 'gz': round(gz, 3),
            'lia_x': round(ax, 3), 'lia_y': round(ay, 3), 'lia_z': round(az - 9.81, 3),
            'qw': round(qw, 4), 'qx': round(qx, 4), 'qy': round(qy, 4), 'qz': round(qz, 4),
            'dev_id': role * 100, 'dev_role': role,
            'haptic_fired': haptic,
            'deviation_score': round(deviation, 3),
            'cal_sys': 3, 'cal_gyro': 3, 'cal_accel': 3, 'cal_mag': 3,
            'stroke_count': stroke_count,
            'stroke_phase': phase,
            'tracking_active': True,
            'entry_angle': round(entry_angle, 1),
            'breath_count': 0,
            'quaternion': {'qw': round(qw, 4), 'qx': round(qx, 4),
                           'qy': round(qy, 4), 'qz': round(qz, 4)},
            'acceleration': {'ax': round(ax, 3), 'ay': round(ay, 3), 'az': round(az, 3)},
            'angular_velocity': {'gx': round(gx, 3), 'gy': round(gy, 3), 'gz': round(gz, 3)},
            'calibration': {'sys': 3, 'accel': 3, 'gyro': 3, 'mag': 3}
        })
        sample_idx += 1

    # --- Pre-stroke glide ---
    for i in range(glide_before):
        t = base_t + sample_idx * dt_ms
        noise = random.gauss(0, 0.1)
        qw, qx, qy, qz = _quat_from_euler(0, 0, 0)
        _add_sample(t, noise, noise, 9.81 + noise, 0.05, 0.05, 0.02,
                    qw, qx, qy, qz, 'glide')

    # --- Stroke cycles ---
    for s in range(num_strokes):
        catch_n = max(2, int(STROKE_CYCLE_SAMPLES * 0.08))
        pull_n = int(STROKE_CYCLE_SAMPLES * 0.34)
        recovery_n = int(STROKE_CYCLE_SAMPLES * 0.33)
        glide_n = STROKE_CYCLE_SAMPLES - catch_n - pull_n - recovery_n

        entry_angle = 28.0 + random.gauss(0, 3)

        # CATCH: brief water-entry impact
        for j in range(catch_n):
            t = base_t + sample_idx * dt_ms
            frac = j / max(1, catch_n - 1)
            impact = 12.0 * (1 - frac) + random.gauss(0, 0.5)
            yaw = -30 + frac * 15
            qw, qx, qy, qz = _quat_from_euler(10 * frac, -20 + frac * 10, yaw)
            _add_sample(t, impact * 0.6, -impact, 9.81 + impact * 0.3,
                        2.5 + random.gauss(0, 0.3), 1.0, 0.5,
                        qw, qx, qy, qz, 'catch',
                        entry_angle=entry_angle,
                        haptic=(j == 0 and s % 3 == 0),
                        deviation=0.3 if j == 0 and s % 3 == 0 else 0.0)
        stroke_count += 1

        # PULL: underwater propulsion
        for j in range(pull_n):
            t = base_t + sample_idx * dt_ms
            frac = j / max(1, pull_n - 1)
            pull_accel = 6.0 * math.sin(frac * math.pi) + random.gauss(0, 0.3)
            roll = 10 + 70 * frac
            pitch = -10 - 30 * frac
            yaw = -15 + 60 * frac
            qw, qx, qy, qz = _quat_from_euler(roll, pitch, yaw)
            _add_sample(t, pull_accel * 0.4, -pull_accel * 0.8, 9.81 + pull_accel * 0.2,
                        0.8 + 0.5 * frac, 0.4, 0.3 + 0.2 * frac,
                        qw, qx, qy, qz, 'pull')

        # RECOVERY: arm out of water, high gyro
        for j in range(recovery_n):
            t = base_t + sample_idx * dt_ms
            frac = j / max(1, recovery_n - 1)
            gyro_mag = 3.5 * math.sin(frac * math.pi) + random.gauss(0, 0.2)
            roll = 80 - 80 * frac
            pitch = -40 + 40 * frac
            yaw = 45 - 45 * frac
            qw, qx, qy, qz = _quat_from_euler(roll, pitch, yaw)
            _add_sample(t, random.gauss(0, 0.3), random.gauss(0, 0.4), 9.81 + random.gauss(0, 0.2),
                        gyro_mag * 0.7, gyro_mag, gyro_mag * 0.3,
                        qw, qx, qy, qz, 'recovery')

        # GLIDE: coast
        for j in range(glide_n):
            t = base_t + sample_idx * dt_ms
            noise = random.gauss(0, 0.15)
            qw, qx, qy, qz = _quat_from_euler(noise * 2, noise, noise)
            _add_sample(t, noise, noise, 9.81 + noise, 0.08, 0.05, 0.03,
                        qw, qx, qy, qz, 'glide')

    # --- Post-stroke glide ---
    for i in range(glide_after):
        t = base_t + sample_idx * dt_ms
        noise = random.gauss(0, 0.1)
        qw, qx, qy, qz = _quat_from_euler(0, 0, 0)
        _add_sample(t, noise, noise, 9.81 + noise, 0.05, 0.05, 0.02,
                    qw, qx, qy, qz, 'glide')

    return data

def main():
    print("Generating mock sessions and writing to cache...")
    roles = [
        (ROLE_HEAD, "Head Mock"),
        (ROLE_WRIST_LEFT, "Left Wrist Mock"),
        (ROLE_WRIST_RIGHT, "Right Wrist Mock")
    ]
    
    sessions = []
    
    # Import database module and init
    import sys
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    import database as db
    db.init_db()
    
    # Create an initial user to satisfy the foreign key constraint
    uid = db.upsert_user(name="Test User", height_cm=180, wingspan_cm=180, skill_level="beginner")
    
    for role, name in roles:
        print(f"Sending mock data for {name}...")
        processed = generate_mock_session(role)
        
        metrics = {
            'stroke_count': 8,
            'turn_count': 0,
            'duration': 16.0,
            'stroke_rate': 30,
            'avg_stroke_time': 1.0,
            'consistency': 82,
            'peak_accel_avg': 6.0,
            'avg_entry_angle': 28.0,
            'ideal_entry_angle': 30.0,
            'phase_pcts': {'glide': 25, 'catch': 8, 'pull': 34, 'recovery': 33},
            'haptic_count': 3,
            'avg_deviation': 0.3,
            'cal_quality': {'accel_pct': 100, 'gyro_pct': 100},
            'stroke_breakdown': []
        }
        
        # Inject into db first so that we have a real sqlite ID!
        sid = db.save_session(user_id=uid, device_ids=[role*100], processed_data=processed, metrics=metrics, duration=30.0, raw_data=processed)
        
        sessions.append({
            'id': sid,
            'name': name + ' - ' + datetime.now().strftime('%b %d, %-I:%M %p'),
            'processed_data': processed,
            'metrics': metrics,
            'duration': 30.0,
            'syncedAt': datetime.now().isoformat(),
            'raw_data': processed # Cheat code for the backend to merge
        })

    payload = {
        'sessions': sessions
    }

    cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.session_cache')
    os.makedirs(cache_dir, exist_ok=True)
    cache_path = os.path.join(cache_dir, 'last_processed.json')
    
    with open(cache_path, 'w') as f:
        json.dump(payload, f)
        
    print(f"Successfully generated 3 mock device sessions! They are waiting in your browser's Sync Cache.")

if __name__ == '__main__':
    main()
