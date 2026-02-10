#!/usr/bin/env python3
"""
WiFi Session Processor - Processes data from ESP32 WiFi server using Python StrokeProcessor
Fetches /data.json from ESP32, processes with StrokeProcessor, returns processed results
"""
import json
import urllib.request
import urllib.error
from http.server import HTTPServer, BaseHTTPRequestHandler
import os
import sys

# Import StrokeProcessor from simple_imu_visualizer.py
sys.path.insert(0, os.path.dirname(__file__))
from simple_imu_visualizer import StrokeProcessor, SimpleKalmanFilter

ESP32_URL = 'http://192.168.4.1'

class WiFiSessionHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/integrated_session_viewer.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), 'integrated_session_viewer.html')
            with open(html_path, 'r') as f:
                self.wfile.write(f.read().encode())
        elif self.path == '/process':
            self.process_wifi_session()
        elif self.path.startswith('/viz/'):
            # Serve shared visualization module
            filename = self.path[5:]  # Remove '/viz/'
            if filename == 'simple_imu_3d_viz.js':
                self.send_response(200)
                self.send_header('Content-type', 'application/javascript')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                js_path = os.path.join(os.path.dirname(__file__), 'simple_imu_3d_viz.js')
                if os.path.exists(js_path):
                    with open(js_path, 'r') as f:
                        self.wfile.write(f.read().encode())
                else:
                    self.wfile.write(b'// Visualization module not found')
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def process_wifi_session(self):
        """Fetch data from ESP32 /data.json and process with StrokeProcessor"""
        try:
            # Fetch raw data from ESP32
            url = f'{ESP32_URL}/data.json'
            req = urllib.request.Request(url)
            req.add_header('Accept', 'application/json')
            
            with urllib.request.urlopen(req, timeout=30) as response:
                result = json.loads(response.read().decode('utf-8'))
            
            # Extract data array
            data = result.get('data', [])
            if not data or len(data) == 0:
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "No data in ESP32 response"}).encode())
                return
            
            # Process with StrokeProcessor - same settings as live serial for accuracy
            processor = StrokeProcessor(batch_mode=False)
            processed_data = []
            
            for sample in data:
                # Use LIA (linear accel, gravity-removed) when available to prevent position Z drift
                lia_x = sample.get('lia_x', sample['ax'])
                lia_y = sample.get('lia_y', sample['ay'])
                lia_z = sample.get('lia_z', sample['az'])
                data_dict = {
                    't': sample['t'],
                    'lia_x': lia_x,
                    'lia_y': lia_y,
                    'lia_z': lia_z,
                    'gx': sample['gx'],
                    'gy': sample['gy'],
                    'gz': sample['gz'],
                    'qw': sample['qw'],
                    'qx': sample['qx'],
                    'qy': sample['qy'],
                    'qz': sample['qz'],
                    'cal': {'sys': 3, 'accel': 3, 'gyro': 3, 'mag': 3}  # Assume calibrated for post-session
                }
                
                # Process sample - returns JSON string matching format expected by visualization
                processed_json = processor.process_data(data_dict)
                if processed_json:
                    processed_data.append(json.loads(processed_json))
            
            # Calculate stroke metrics
            stroke_metrics = self.calculate_stroke_metrics(processed_data, processor)
            
            # Return processed data + metrics
            response_data = {
                'processed_data': processed_data,
                'metrics': stroke_metrics,
                'raw_samples': len(data),
                'processed_samples': len(processed_data)
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(response_data).encode())
            
        except urllib.error.URLError as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"error": f"Failed to fetch from ESP32: {str(e)}"}).encode())
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"error": f"Processing error: {str(e)}"}).encode())
    
    def calculate_stroke_metrics(self, processed_data, processor):
        """Calculate stroke metrics from processed data"""
        if len(processed_data) == 0:
            return {
                'stroke_count': 0,
                'duration': 0,
                'stroke_rate': 0,
                'avg_stroke_time': 0,
                'consistency': 0,
                'peak_accel_avg': 0
            }
        
        # Extract stroke times from processed data
        stroke_times = []
        peak_accels = []
        last_stroke_count = 0
        
        for p in processed_data:
            stroke_count = p.get('stroke_count', 0)
            if stroke_count > last_stroke_count:
                stroke_times.append(p.get('timestamp', 0))
                debug = p.get('stroke_debug', {})
                if debug.get('accel_mag', 0) > 0:
                    peak_accels.append(debug['accel_mag'])
            last_stroke_count = stroke_count
        
        duration = (processed_data[-1].get('timestamp', 0) - processed_data[0].get('timestamp', 0)) / 1000.0
        
        metrics = {
            'stroke_count': processor.stroke_count,
            'duration': duration,
            'stroke_rate': (processor.stroke_count / duration * 60) if duration > 0 else 0,
            'avg_stroke_time': 0,
            'consistency': 0,
            'peak_accel_avg': sum(peak_accels) / len(peak_accels) if peak_accels else 0
        }
        
        # Calculate average stroke time and consistency
        if len(stroke_times) >= 2:
            intervals = [stroke_times[i] - stroke_times[i-1] for i in range(1, len(stroke_times))]
            avg_interval = sum(intervals) / len(intervals) / 1000.0  # Convert to seconds
            metrics['avg_stroke_time'] = avg_interval
            
            if len(intervals) >= 2:
                variance = sum((i - avg_interval * 1000)**2 for i in intervals) / len(intervals)
                cv = (variance**0.5 / (avg_interval * 1000)) * 100 if avg_interval > 0 else 0
                metrics['consistency'] = max(0, 100 - min(cv, 100))
        
        return metrics

def start_server(port=8004):
    """Start the WiFi session processor server"""
    server = HTTPServer(('0.0.0.0', port), WiFiSessionHandler)
    print(f'WiFi Session Processor running on http://localhost:{port}')
    print(f'Open: http://localhost:{port}/integrated_session_viewer.html')
    print('Press Ctrl+C to stop')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down...')
        server.shutdown()

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 8004))
    start_server(port)
