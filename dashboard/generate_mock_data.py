import json
import math
import time
from datetime import datetime
import os
import sqlite3

ROLE_HEAD = 3
ROLE_WRIST_LEFT = 1
ROLE_WRIST_RIGHT = 2

def generate_mock_session(role, num_samples=300):
    data = []
    base_t = int(time.time() * 1000) - 60000 # 1 minute ago
    hop_idx = 20
    
    for i in range(num_samples):
        t = base_t + i * 100 # 10Hz
        phase = (i / 30.0) * math.pi * 2
        
        ax = math.sin(phase) * 5.0
        ay = math.cos(phase * 0.5) * 2.0
        az = math.sin(phase * 2.0) * 3.0
        
        gx = math.cos(phase) * 50.0
        gy = math.sin(phase) * 20.0
        gz = math.cos(phase * 0.5) * 10.0
        
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        
        if i >= hop_idx and i <= hop_idx + 3:
            ax += 20.0
            ay += 20.0
            
        data.append({
            't': t, 'timestamp': t,
            'ax': ax, 'ay': ay, 'az': az,
            'gx': gx, 'gy': gy, 'gz': gz,
            'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
            'dev_id': role * 100,
            'dev_role': role,
            'haptic_fired': i % 100 == 0,
            'deviation_score': 0.5 if i % 100 == 0 else 0.0,
            'cal_sys': 3, 'cal_gyro': 3, 'cal_accel': 3, 'cal_mag': 3,
            'stroke_count': i // 30,
            'stroke_phase': 'pull' if i % 30 < 10 else ('recovery' if i % 30 < 20 else 'glide'),
            'tracking_active': True,
            'entry_angle': 30.0,
            'quaternion': {'qw':qw, 'qx':qx, 'qy':qy, 'qz':qz},
            'acceleration': {'ax':ax, 'ay':ay, 'az':az},
            'angular_velocity': {'gx':gx, 'gy':gy, 'gz':gz},
            'calibration': {'sys':3, 'accel':3, 'gyro':3, 'mag':3}
        })
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
    
    # Generate the processed objects layout
    db_id_ctr = 100
    for role, name in roles:
        print(f"Sending mock data for {name}...")
        processed = generate_mock_session(role)
        
        metrics = {
            'stroke_count': 10,
            'duration': 30.0,
            'stroke_rate': 20,
            'avg_stroke_time': 3.0,
            'consistency': 85,
            'peak_accel_avg': 5.0,
            'avg_entry_angle': 30.0,
            'phase_pcts': {'glide': 30, 'pull': 40, 'recovery': 30},
            'haptic_count': 3,
            'avg_deviation': 0.5
        }
        
        sessions.append({
            'id': db_id_ctr,
            'name': name + ' - ' + datetime.now().strftime('%b %d, %-I:%M %p'),
            'processed_data': processed,
            'metrics': metrics,
            'duration': 30.0,
            'syncedAt': datetime.now().isoformat(),
            'raw_data': processed # Cheat code for the backend to merge
        })
        
        # Inject into db so that /api/sessions/merge can find the raw bodies!
        db.save_session(user_id=uid, device_ids=[role*100], processed_data=processed, metrics=metrics, duration=30.0, raw_data=processed)
        db_id_ctr += 1

    payload = {
        'sessions': sessions
    }

    cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.cache')
    os.makedirs(cache_dir, exist_ok=True)
    cache_path = os.path.join(cache_dir, 'last_processed.json')
    
    with open(cache_path, 'w') as f:
        json.dump(payload, f)
        
    print(f"Successfully generated 3 mock device sessions! They are waiting in your browser's Sync Cache.")

if __name__ == '__main__':
    main()
