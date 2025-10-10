#!/usr/bin/env python3
"""
Simple HTTP server for BLE dashboard
Web Bluetooth requires HTTPS or localhost
"""

from http.server import HTTPServer, SimpleHTTPRequestHandler
import os

class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

def run_server(port=8080):
    # Change to dashboard directory
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    server_address = ('', port)
    httpd = HTTPServer(server_address, CORSRequestHandler)
    
    print("=" * 60)
    print("BLE Dashboard Server")
    print("=" * 60)
    print(f"Server running on: http://localhost:{port}")
    print(f"BLE Dashboard: http://localhost:{port}/ble_dashboard.html")
    print("")
    print("📡 Web Bluetooth Requirements:")
    print("  ✅ Use Chrome, Edge, or Opera browser")
    print("  ✅ Bluetooth must be enabled on your computer")
    print("  ✅ ESP32 must be running with BLE enabled")
    print("")
    print("🚀 To connect:")
    print("  1. Open http://localhost:8080/ble_dashboard.html in Chrome")
    print("  2. Click 'Connect to ESP32'")
    print("  3. Select 'FormSync' from device list")
    print("  4. Watch live IMU data!")
    print("")
    print("Press Ctrl+C to stop the server")
    print("=" * 60)
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n\nServer stopped.")
        httpd.shutdown()

if __name__ == '__main__':
    run_server()

