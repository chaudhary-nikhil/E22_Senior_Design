#!/usr/bin/env python3
"""
Simple HTTP server to serve integrated_session_viewer.html
Default port 8844 (override with PORT env; simple_imu_visualizer.py uses 8003)
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

    def _api_not_available(self):
        import json
        body = json.dumps({
            'error': 'This static server does not support API routes. '
                     'Run wifi_session_processor.py instead: '
                     'python wifi_session_processor.py [--demo]'
        }).encode()
        self.send_response(501)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith('/api/') or self.path == '/process':
            self._api_not_available()
            return
        super().do_GET()

    def do_POST(self):
        self._api_not_available()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 8844))
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    server = HTTPServer(('0.0.0.0', port), CORSRequestHandler)
    print(f'Serving integrated_session_viewer.html on http://localhost:{port}')
    print(f'Open: http://localhost:{port}/integrated_session_viewer.html')
    print('Press Ctrl+C to stop')
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down...')
        server.shutdown()
