#!/usr/bin/env python3
"""
BNO055 Session Logger & Playback Visualizer
Logs IMU session data to JSON, then plays back visualization
Perfect for swim analysis - log session, analyze afterward
"""

import serial
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import os
from datetime import datetime

# Global variables
serial_port = None
logging_active = False
session_data = []
session_start_time = None
current_session_id = None

class SessionHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), 'session_visualizer.html')
            with open(html_path, 'r') as f:
                self.wfile.write(f.read().encode())
        elif self.path == '/start_logging':
            self.start_logging()
        elif self.path == '/stop_logging':
            self.stop_logging()
        elif self.path == '/sessions':
            self.list_sessions()
        elif self.path.startswith('/data/'):
            self.get_session_data()
        elif self.path == '/visualizer.js':
            self.send_response(200)
            self.send_header('Content-type', 'application/javascript')
            self.end_headers()
            js_path = os.path.join(os.path.dirname(__file__), 'visualizer.js')
            with open(js_path, 'r') as f:
                self.wfile.write(f.read().encode())
        elif self.path == '/session_status':
            self.get_session_status()
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_POST(self):
        if self.path == '/start_logging':
            self.start_logging()
        elif self.path == '/stop_logging':
            self.stop_logging()
        else:
            self.send_response(404)
            self.end_headers()
    
    def start_logging(self):
        global logging_active, session_data, session_start_time, current_session_id
        
        if not logging_active:
            logging_active = True
            session_data = []
            session_start_time = datetime.now()
            current_session_id = session_start_time.strftime("%Y%m%d_%H%M%S")
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "logging_started",
                "session_id": current_session_id,
                "start_time": session_start_time.isoformat()
            }).encode())
        else:
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "already_logging",
                "session_id": current_session_id
            }).encode())

    def stop_logging(self):
        global logging_active, session_data, current_session_id
        
        if logging_active:
            logging_active = False
            
            # Save session data to JSON file
            filename = f"swim_session_{current_session_id}.json"
            with open(filename, 'w') as f:
                json.dump(session_data, f, indent=2)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "logging_stopped",
                "session_id": current_session_id,
                "data_points": len(session_data),
                "filename": filename
            }).encode())
        else:
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "not_logging"
            }).encode())

    def list_sessions(self):
        """List available session files"""
        import glob
        json_files = glob.glob("swim_session_*.json")
        sessions = [os.path.basename(f) for f in json_files]
        
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(sessions).encode())
        
    def get_session_data(self):
        """Get data for a specific session file"""
        filename = self.path[6:]  # Remove '/data/' prefix
        if os.path.exists(filename):
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with open(filename, 'r') as f:
                self.wfile.write(f.read().encode())
        else:
            self.send_response(404)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"error": "Session not found"}).encode())

def find_esp32_port():
    """Find ESP32 serial port"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'usbserial' in port.device or 'USB' in port.description:
                return port.device
        return None
    except ImportError:
        print("Install pyserial: pip install pyserial")
        return None

def serial_logger():
    """Log BNO055 data from ESP32 serial port during active sessions"""
    global serial_port, session_data, logging_active, session_start_time
    connected_once = False  # Track if we've already printed connection message
    
    while True:
        try:
            if serial_port is None:
                port = find_esp32_port()
                if port:
                    serial_port = serial.Serial(port, 115200, timeout=1)
                    if not connected_once:
                        print(f"✅ Connected to ESP32 on {port}")
                        connected_once = True
                    time.sleep(0.5)
                else:
                    if not connected_once:
                        print("⚠️  No ESP32 found - waiting for connection...")
                    time.sleep(2)
                    continue

            raw_data = serial_port.readline()
            if not raw_data:
                continue

            if not serial_port.is_open:
                serial_port = None
                continue

            try:
                line = raw_data.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            # Parse BNO055 JSON data
            if line and line.startswith('{') and line.endswith('}'):
                try:
                    data = json.loads(line)
                    
                    # Only log during active sessions
                    if logging_active:
                        # Add timestamp relative to session start
                        session_timestamp = time.time() - session_start_time.timestamp()
                        data['session_time'] = round(session_timestamp, 3)
                        session_data.append(data)
                        
                        # Simple status print
                        cal = data.get('cal', {})
                        print(f"📊 Logging: t={session_timestamp:.1f}s Roll={data.get('roll', 0):.1f}° Cal={cal.get('sys', 0)}/{cal.get('gyro', 0)}/{cal.get('accel', 0)}/{cal.get('mag', 0)}")

                except json.JSONDecodeError:
                    pass

        except serial.SerialException:
            if serial_port:
                serial_port.close()
            serial_port = None
            connected_once = False  # Reset connection flag for reconnection
            time.sleep(1)
        except Exception as e:
            print(f"Serial error: {e}")
            time.sleep(0.1)

def start_web_server():
    """Start web server for session management"""
    try:
        server = HTTPServer(('localhost', 8016), SessionHandler)
        print("🌐 Session Logger started on http://localhost:8016")
        server.serve_forever()
    except OSError:
        print("Port 8016 in use, trying 8017...")
        try:
            server = HTTPServer(('localhost', 8017), SessionHandler)
            print("🌐 Session Logger started on http://localhost:8017")
            server.serve_forever()
        except OSError:
            print("❌ Could not start web server")

if __name__ == "__main__":
    print("🏊 BNO055 Swim Session Logger")
    print("=============================")
    print("📊 Features: Real-time ESP32 data logging, JSON storage, playback visualization")
    print("🎯 Perfect for swim analysis - log session, analyze afterward")
    print("⚠️  Requires ESP32 with BNO055 connected via serial")
    print("")
    print("🌐 Open http://localhost:8016 to manage sessions")
    print("📝 Click 'Start Logging' to begin logging")
    print("⏹️ Click 'Stop Logging' to save and analyze")

    # Start serial logger
    logger_thread = threading.Thread(target=serial_logger, daemon=True)
    logger_thread.start()

    # Start web server
    server_thread = threading.Thread(target=start_web_server, daemon=True)
    server_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        if serial_port:
            serial_port.close()
        if session_data:
            filename = f"swim_session_emergency_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(session_data, f, indent=2)
            print(f"💾 Emergency save: {len(session_data)} samples to {filename}")
