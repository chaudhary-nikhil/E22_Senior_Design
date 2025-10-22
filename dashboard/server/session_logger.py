#!/usr/bin/env python3
"""
GoldenForm Session Logger - Main Application
Modular Bluetooth IMU data logging and visualization system
"""

import threading
import time
import asyncio
from .session_manager import SessionManager
from .bluetooth_client import bluetooth_logger
from .web_server import start_web_server
from .config import DEFAULT_PORT

def main():
    """Main application entry point"""
    print("🏊 GoldenForm Session Logger (Bluetooth)")
    print("==========================================")
    print("📊 Features: Real-time ESP32 data logging via Bluetooth, JSON storage, playback visualization")
    print("🎯 Perfect for swim analysis - log session, analyze afterward")
    print("⚠️  Requires ESP32 with BNO055 connected via Bluetooth")
    print("")
    print(f"🌐 Open http://localhost:{DEFAULT_PORT} to manage sessions")
    print("📝 Click 'Start Logging' to begin logging")
    print("⏹️ Click 'Stop Logging' to save and analyze")

    # Initialize session manager
    session_manager = SessionManager()
    
    # Create data callback for Bluetooth
    def data_callback(data_obj):
        """Callback to handle incoming Bluetooth data"""
        session_manager.add_data_point(data_obj)
    
    # Start Bluetooth logger in separate thread
    def run_bluetooth_logger():
        asyncio.run(bluetooth_logger(data_callback))
    
    logger_thread = threading.Thread(target=run_bluetooth_logger, daemon=True)
    logger_thread.start()

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
