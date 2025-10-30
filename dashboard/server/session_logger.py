#!/usr/bin/env python3
"""
GoldenForm Session Logger - Main Application
WiFi-based IMU data logging and visualization system
"""

import threading
import time
try:
    from .session_manager_wifi import SessionManager
    from .web_server import start_web_server
    from .config import DEFAULT_PORT
except ImportError:
    from session_manager_wifi import SessionManager
    from web_server import start_web_server
    from config import DEFAULT_PORT

def main():
    """Main application entry point"""
    print("🏊 GoldenForm Session Logger (WiFi)")
    print("=====================================")
    print("📊 Features: Real-time ESP32 data logging via WiFi, protobuf storage, playback visualization")
    print("🎯 Perfect for swim analysis - log session, analyze afterward")
    print("⚠️  Requires ESP32 with BNO055 connected via WiFi")
    print("")
    print(f"🌐 Open http://localhost:{DEFAULT_PORT} to view sessions")
    print("📝 Press EN button on ESP32 to START logging")
    print("⏹️ Press BOOT button on ESP32 to STOP logging")
    print("")
    print("📡 ESP32 WiFi: Connect to 'GoldenForm' network (password: goldenform123)")
    print("🌐 ESP32 Web: http://192.168.4.1")

    # Initialize session manager
    session_manager = SessionManager()

    # Start web server in separate thread
    server_thread = threading.Thread(target=start_web_server, daemon=True, args=(session_manager, DEFAULT_PORT))
    server_thread.start()

    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        
        # Emergency save if actively logging
        saved_samples = session_manager.emergency_save()
        
        print("✅ Shutdown complete")

if __name__ == "__main__":
    main()
