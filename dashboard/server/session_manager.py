#!/usr/bin/env python3
"""
Session management for GoldenForm data logging
"""

import json
import os
import threading
import time
from datetime import datetime
import glob
from .config import SESSIONS_DIR, SESSION_FILE_PREFIX, EMERGENCY_FILE_PREFIX, LOG_FORMAT

class SessionManager:
    """Manages session logging and file operations"""
    
    def __init__(self):
        self.logging_active = False
        self.session_data = []
        self.session_start_time = None
        self.session_start_monotonic = None
        self.current_session_id = None
        self.data_lock = threading.Lock()
        
    def start_logging(self):
        """Start a new logging session"""
        if not self.logging_active:
            self.logging_active = True
            self.session_data = []
            self.session_start_time = datetime.now()
            self.session_start_monotonic = time.monotonic()
            self.current_session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")
            
            return {
                "status": "logging_started",
                "session_id": self.current_session_id,
                "start_time": self.session_start_time.isoformat()
            }
        else:
            return {
                "status": "already_logging",
                "session_id": self.current_session_id
            }
    
    def stop_logging(self):
        """Stop logging and save session to file"""
        if self.logging_active:
            self.logging_active = False
            
            # Save session data to JSON file
            os.makedirs(SESSIONS_DIR, exist_ok=True)
            filename = f"{SESSIONS_DIR}/{SESSION_FILE_PREFIX}{self.current_session_id}.json"
            with open(filename, 'w') as f:
                json.dump(self.session_data, f, indent=2)
            
            return {
                "status": "logging_stopped",
                "session_id": self.current_session_id,
                "data_points": len(self.session_data),
                "filename": filename
            }
        else:
            return {"status": "not_logging"}
    
    def get_session_status(self):
        """Get current session status"""
        return {
            "logging_active": self.logging_active,
            "session_id": self.current_session_id,
            "samples": len(self.session_data),
            "start_time": self.session_start_time.isoformat() if self.session_start_time else None
        }
    
    def list_sessions(self):
        """List all available session files"""
        json_files = glob.glob(f"{SESSIONS_DIR}/{SESSION_FILE_PREFIX}*.json")
        return [os.path.basename(f) for f in json_files]
    
    def get_session_data(self, filename):
        """Get data for a specific session file"""
        filepath = f"{SESSIONS_DIR}/{filename}"
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                return json.load(f)
        else:
            return None
    
    def add_data_point(self, data_obj):
        """Add a data point to the current session"""
        if self.logging_active:
            # Add timestamp relative to session start
            session_timestamp = time.monotonic() - self.session_start_monotonic
            data_obj['session_time'] = round(session_timestamp, 3)
            
            with self.data_lock:
                self.session_data.append(data_obj)
            
            # Log status
            cal = data_obj.get('cal', {})
            print(LOG_FORMAT.format(
                session_timestamp,
                data_obj.get('roll', 0),
                cal.get('sys', 0),
                cal.get('gyro', 0),
                cal.get('accel', 0),
                cal.get('mag', 0)
            ))
    
    def emergency_save(self):
        """Emergency save if actively logging"""
        try:
            with self.data_lock:
                if self.logging_active and self.session_data:
                    os.makedirs(SESSIONS_DIR, exist_ok=True)
                    filename = f"{SESSIONS_DIR}/{EMERGENCY_FILE_PREFIX}{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                    with open(filename, 'w') as f:
                        json.dump(self.session_data, f, indent=2)
                    print(f"💾 Emergency save: {len(self.session_data)} samples to {filename}")
                    return len(self.session_data)
                elif self.session_data:
                    print(f"📊 Session data already saved ({len(self.session_data)} samples)")
                    return len(self.session_data)
                else:
                    print("📊 No session data to save")
                    return 0
        except Exception as e:
            print(f"⚠️  Emergency save warning: {e}")
            return 0
