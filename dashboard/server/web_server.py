#!/usr/bin/env python3
"""
Web server and API endpoints for GoldenForm session logger
"""

import os
from http.server import HTTPServer, BaseHTTPRequestHandler
try:
    from .session_manager_wifi import SessionManager
except ImportError:
    from session_manager_wifi import SessionManager

class SessionHandler(BaseHTTPRequestHandler):
    """HTTP request handler for session management API"""
    
    def __init__(self, session_manager, *args, **kwargs):
        self.session_manager = session_manager
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            self.serve_html()
        elif self.path == '/get_session_data':
            self.handle_get_session_data()
        elif self.path == '/create_visualization':
            self.handle_create_visualization()
        elif self.path == '/sessions':
            self.handle_list_sessions()
        elif self.path.startswith('/data/'):
            self.handle_get_session_data()
        elif self.path == '/session_status':
            self.handle_session_status()
        elif self.path == '/visualization':
            self.handle_visualization()
        else:
            self.send_error(404)
    
    def do_POST(self):
        """Handle POST requests"""
        if self.path == '/get_session_data':
            self.handle_get_session_data()
        elif self.path == '/create_visualization':
            self.handle_create_visualization()
        else:
            self.send_error(404)
    
    def serve_html(self):
        """Serve the main HTML interface"""
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        html_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'client', 'simple_session_logger.html')
        with open(html_path, 'r') as f:
            self.wfile.write(f.read().encode())
    
    
    def handle_start_logging(self):
        """Handle start logging request"""
        result = self.session_manager.start_logging()
        self.send_json_response(result)
    
    def handle_stop_logging(self):
        """Handle stop logging request"""
        result = self.session_manager.stop_logging()
        self.send_json_response(result)
    
    def handle_create_visualization(self):
        """Handle create visualization request"""
        result = self.session_manager.create_visualization()
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
        if self.path == '/get_session_data':
            # Get latest session data from ESP32 with timeout protection
            try:
                data = self.session_manager.get_session_data()
                self.send_json_response(data)
            except Exception as e:
                self.send_json_response({"status": "error", "error": f"Request failed: {e}"}, status=500)
        else:
            # Get specific session data by filename
            filename = self.path[6:]  # Remove '/data/' prefix
            data = self.session_manager.get_session_data_by_filename(filename)
            
            if data is not None:
                self.send_json_response(data)
            else:
                self.send_json_response({"error": "Session not found"}, status=404)
    
    def handle_visualization(self):
        """Serve the visualization HTML page"""
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        html_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'client', 'visualization.html')
        with open(html_path, 'r') as f:
            self.wfile.write(f.read().encode())
    
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
    ports_to_try = [port, port + 1, port + 2, port + 3, port + 4]
    
    for try_port in ports_to_try:
        try:
            handler = create_handler(session_manager)
            server = HTTPServer(('localhost', try_port), handler)
            print(f"🌐 Session Logger started on http://localhost:{try_port}")
            server.serve_forever()
            return  # Success, exit the function
        except OSError:
            if try_port == ports_to_try[-1]:  # Last port tried
                print(f"❌ Could not start web server on any port {port}-{try_port}")
                print("💡 Try closing other applications or restart your terminal")
            else:
                print(f"Port {try_port} in use, trying {try_port + 1}...")
