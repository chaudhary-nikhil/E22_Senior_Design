#!/usr/bin/env python3
"""
Web server and API endpoints for GoldenForm session logger
"""

import os
from http.server import HTTPServer, BaseHTTPRequestHandler
from .session_manager import SessionManager

class SessionHandler(BaseHTTPRequestHandler):
    """HTTP request handler for session management API"""
    
    def __init__(self, session_manager, *args, **kwargs):
        self.session_manager = session_manager
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            self.serve_html()
        elif self.path == '/start_logging':
            self.handle_start_logging()
        elif self.path == '/stop_logging':
            self.handle_stop_logging()
        elif self.path == '/sessions':
            self.handle_list_sessions()
        elif self.path.startswith('/data/'):
            self.handle_get_session_data()
        elif self.path == '/visualizer.js':
            self.serve_js()
        elif self.path == '/session_status':
            self.handle_session_status()
        else:
            self.send_error(404)
    
    def do_POST(self):
        """Handle POST requests"""
        if self.path == '/start_logging':
            self.handle_start_logging()
        elif self.path == '/stop_logging':
            self.handle_stop_logging()
        else:
            self.send_error(404)
    
    def serve_html(self):
        """Serve the main HTML interface"""
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        html_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'client', 'session_visualizer.html')
        with open(html_path, 'r') as f:
            self.wfile.write(f.read().encode())
    
    def serve_js(self):
        """Serve the JavaScript visualizer"""
        self.send_response(200)
        self.send_header('Content-type', 'application/javascript')
        self.end_headers()
        js_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'client', 'visualizer.js')
        with open(js_path, 'r') as f:
            self.wfile.write(f.read().encode())
    
    def handle_start_logging(self):
        """Handle start logging request"""
        result = self.session_manager.start_logging()
        self.send_json_response(result)
    
    def handle_stop_logging(self):
        """Handle stop logging request"""
        result = self.session_manager.stop_logging()
        self.send_json_response(result)
    
    def handle_session_status(self):
        """Handle session status request"""
        result = self.session_manager.get_session_status()
        self.send_json_response(result)
    
    def handle_list_sessions(self):
        """Handle list sessions request"""
        sessions = self.session_manager.list_sessions()
        self.send_json_response(sessions)
    
    def handle_get_session_data(self):
        """Handle get session data request"""
        filename = self.path[6:]  # Remove '/data/' prefix
        data = self.session_manager.get_session_data(filename)
        
        if data is not None:
            self.send_json_response(data)
        else:
            self.send_json_response({"error": "Session not found"}, status=404)
    
    def send_json_response(self, data, status=200):
        """Send JSON response"""
        import json
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

def create_handler(session_manager):
    """Create a handler class with session manager"""
    def handler(*args, **kwargs):
        return SessionHandler(session_manager, *args, **kwargs)
    return handler

def start_web_server(session_manager, port=8016):
    """Start the web server"""
    try:
        handler = create_handler(session_manager)
        server = HTTPServer(('localhost', port), handler)
        print(f"🌐 Session Logger started on http://localhost:{port}")
        server.serve_forever()
    except OSError:
        print(f"Port {port} in use, trying {port + 1}...")
        try:
            handler = create_handler(session_manager)
            server = HTTPServer(('localhost', port + 1), handler)
            print(f"🌐 Session Logger started on http://localhost:{port + 1}")
            server.serve_forever()
        except OSError:
            print("❌ Could not start web server")
