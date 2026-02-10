#!/usr/bin/env python3
"""
Simple HTTP server to serve integrated_session_viewer.html
Serves on port 8004 (different from simple_imu_visualizer.py on 8003)
"""
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
import sys

class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 8004))
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    server = HTTPServer(('0.0.0.0', port), CORSRequestHandler)
    print(f'Serving integrated_session_viewer.html on http://localhost:{port}')
    print('Open: http://localhost:8004/integrated_session_viewer.html')
    print('Press Ctrl+C to stop')
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down...')
        server.shutdown()
