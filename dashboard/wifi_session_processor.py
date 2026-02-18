#!/usr/bin/env python3
"""
WiFi Session Processor - Processes data from ESP32 WiFi server using Python StrokeProcessor
Fetches /data.json from ESP32, processes with StrokeProcessor, returns processed results.
Supports multiple sessions per sync.
"""
import json
import urllib.request
import urllib.error
from http.server import HTTPServer, BaseHTTPRequestHandler
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from simple_imu_visualizer import StrokeProcessor, SimpleKalmanFilter

ESP32_URL = 'http://192.168.4.1'
CACHE_DIR = os.path.join(os.path.dirname(__file__), '.session_cache')
RAW_CACHE = os.path.join(CACHE_DIR, 'last_raw.json')
PROCESSED_CACHE = os.path.join(CACHE_DIR, 'last_processed.json')
os.makedirs(CACHE_DIR, exist_ok=True)


class WiFiSessionHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/integrated_session_viewer.html':
            self._serve_html()
        elif self.path == '/process':
            self._process_wifi_session()
        elif self.path == '/cached':
            self._serve_cached()
        elif self.path.startswith('/viz/'):
            self._serve_viz_module(self.path[5:])
        elif self.path == '/docs/screenshot_server_log.html':
            self._serve_doc('docs/screenshot_server_log.html', 'text/html')
        elif self.path == '/docs/screenshot_esp32_terminal.html':
            self._serve_doc('docs/screenshot_esp32_terminal.html', 'text/html')
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_doc(self, rel_path, content_type):
        path = os.path.join(os.path.dirname(__file__), rel_path)
        if not os.path.exists(path):
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-type', content_type)
        self.end_headers()
        with open(path, 'rb') as f:
            self.wfile.write(f.read())

    def do_POST(self):
        if self.path == '/reprocess':
            self._reprocess_session()
        else:
            self.send_response(404)
            self.end_headers()

    def _reprocess_session(self):
        """Re-process raw session data through the current StrokeProcessor."""
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length).decode('utf-8'))
            raw_data = body.get('processedData', [])
            if not raw_data:
                self._send_json({"error": "No data provided"}, 400)
                return

            processor = StrokeProcessor(batch_mode=True)
            processed_data = []
            for sample in raw_data:
                acc = sample.get('acceleration', {})
                ang = sample.get('angular_velocity', {})
                q = sample.get('quaternion', {})
                cal = sample.get('calibration', {})
                data_dict = {
                    't': sample.get('timestamp', 0),
                    'lia_x': acc.get('ax', 0), 'lia_y': acc.get('ay', 0), 'lia_z': acc.get('az', 0),
                    'gx': ang.get('gx', 0), 'gy': ang.get('gy', 0), 'gz': ang.get('gz', 0),
                    'qw': q.get('qw', 1), 'qx': q.get('qx', 0), 'qy': q.get('qy', 0), 'qz': q.get('qz', 0),
                    'cal': {
                        'sys': cal.get('sys', 0), 'accel': cal.get('accel', 0),
                        'gyro': cal.get('gyro', 0), 'mag': cal.get('mag', 0)
                    }
                }
                result = processor.process_data(data_dict)
                if result:
                    processed_data.append(json.loads(result))

            metrics = self._calculate_stroke_metrics(processed_data, processor)
            self._send_json({
                'processedData': processed_data,
                'metrics': metrics
            })
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({"error": str(e)}, 500)

    def _serve_cached(self):
        """Serve last successfully processed data from disk cache."""
        if os.path.exists(PROCESSED_CACHE):
            with open(PROCESSED_CACHE, 'r') as f:
                self._send_json(json.load(f))
        else:
            self._send_json({"error": "No cached data available"}, 404)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        html_path = os.path.join(os.path.dirname(__file__), 'integrated_session_viewer.html')
        with open(html_path, 'r') as f:
            self.wfile.write(f.read().encode())

    _ALLOWED_JS = {
        'simple_imu_3d_viz.js', 'three.min.js', 'OrbitControls.js', 'chart.umd.min.js'
    }

    def _serve_viz_module(self, filename):
        if filename in self._ALLOWED_JS:
            self.send_response(200)
            self.send_header('Content-type', 'application/javascript')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'public, max-age=86400')
            self.end_headers()
            js_path = os.path.join(os.path.dirname(__file__), filename)
            if os.path.exists(js_path):
                with open(js_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.wfile.write(b'// File not found: ' + filename.encode())
        else:
            self.send_response(404)
            self.end_headers()

    def _send_json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _process_wifi_session(self):
        """Fetch data from ESP32 and process each session independently."""
        try:
            url = f'{ESP32_URL}/data.json'
            req = urllib.request.Request(url)
            req.add_header('Accept', 'application/json')

            with urllib.request.urlopen(req, timeout=8) as response:
                result = json.loads(response.read().decode('utf-8'))

            # Handle multi-session format: {"sessions":[...],"totalSamples":N}
            sessions_raw = result.get('sessions', [])
            expected_total = result.get('totalSamples', None)

            # Backward compat: old format {"files":N,"data":[...]}
            if not sessions_raw and result.get('data'):
                sessions_raw = [{'id': 1, 'name': 'Session 1', 'data': result['data']}]

            if not sessions_raw:
                self._send_json({"error": "No sessions in ESP32 response"})
                return

            processed_sessions = []
            actual_total = 0
            from datetime import datetime

            for sess in sessions_raw:
                data = sess.get('data', [])
                if not data:
                    continue

                actual_total += len(data)
                processor = StrokeProcessor(batch_mode=True)
                processed_data = []

                for sample in data:
                    lia_x = sample.get('lia_x', sample.get('ax', 0))
                    lia_y = sample.get('lia_y', sample.get('ay', 0))
                    lia_z = sample.get('lia_z', sample.get('az', 0))
                    data_dict = {
                        't': sample.get('t', 0),
                        'lia_x': lia_x, 'lia_y': lia_y, 'lia_z': lia_z,
                        'gx': sample.get('gx', 0),
                        'gy': sample.get('gy', 0),
                        'gz': sample.get('gz', 0),
                        'qw': sample.get('qw', 1),
                        'qx': sample.get('qx', 0),
                        'qy': sample.get('qy', 0),
                        'qz': sample.get('qz', 0),
                        'cal': {
                            'sys': sample.get('cal_sys', 0),
                            'accel': sample.get('cal_accel', 0),
                            'gyro': sample.get('cal_gyro', 0),
                            'mag': sample.get('cal_mag', 0),
                        }
                    }

                    processed_json = processor.process_data(data_dict)
                    if processed_json:
                        processed_data.append(json.loads(processed_json))

                metrics = self._calculate_stroke_metrics(processed_data, processor)

                duration = 0
                if len(processed_data) >= 2:
                    t0 = processed_data[0].get('timestamp', 0)
                    t1 = processed_data[-1].get('timestamp', 0)
                    duration = (t1 - t0) / 1000.0

                now_str = datetime.now().strftime('%b %d, %-I:%M %p')
                sess_name = f"Session {sess.get('id', '?')} - {now_str}"

                processed_sessions.append({
                    'id': sess.get('id', 0),
                    'name': sess_name,
                    'processed_data': processed_data,
                    'metrics': metrics,
                    'raw_samples': len(data),
                    'processed_samples': len(processed_data),
                    'duration': duration,
                    'syncedAt': datetime.now().isoformat()
                })

            integrity_ok = (expected_total is None or expected_total == actual_total)
            if not integrity_ok:
                print(f'[WARN] Data integrity mismatch: expected {expected_total}, got {actual_total}')

            response = {
                'sessions': processed_sessions,
                'integrity': {
                    'expected': expected_total,
                    'received': actual_total,
                    'verified': integrity_ok
                }
            }
            try:
                with open(RAW_CACHE, 'w') as f:
                    json.dump(result, f)
                with open(PROCESSED_CACHE, 'w') as f:
                    json.dump(response, f)
                print(f'[CACHE] Saved {len(processed_sessions)} session(s) to disk')
            except Exception as ce:
                print(f'[CACHE] Failed to save: {ce}')

            self._send_json(response)

        except urllib.error.URLError as e:
            print(f'[ESP32] Unreachable: {e}. Trying cache...')
            if os.path.exists(PROCESSED_CACHE):
                with open(PROCESSED_CACHE, 'r') as f:
                    self._send_json(json.load(f))
                return
            self._send_json({"error": f"ESP32 unreachable and no cache: {str(e)}"}, 500)
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({"error": f"Processing error: {str(e)}"}, 500)

    def _calculate_stroke_metrics(self, processed_data, processor):
        if len(processed_data) == 0:
            return {
                'stroke_count': 0, 'duration': 0, 'stroke_rate': 0,
                'avg_stroke_time': 0, 'consistency': 0, 'peak_accel_avg': 0
            }

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

        duration = (processed_data[-1].get('timestamp', 0) -
                    processed_data[0].get('timestamp', 0)) / 1000.0

        accel_good = sum(1 for p in processed_data
                        if (p.get('calibration', {}).get('accel', 0) >= 2))
        gyro_good = sum(1 for p in processed_data
                        if (p.get('calibration', {}).get('gyro', 0) >= 2))
        n = len(processed_data)

        metrics = {
            'stroke_count': processor.stroke_count,
            'turn_count': processor.turn_count,
            'duration': duration,
            'stroke_rate': (processor.stroke_count / duration * 60) if duration > 0 else 0,
            'avg_stroke_time': 0,
            'consistency': 0,
            'peak_accel_avg': sum(peak_accels) / len(peak_accels) if peak_accels else 0,
            'cal_quality': {
                'accel_pct': round(accel_good / n * 100) if n > 0 else 0,
                'gyro_pct': round(gyro_good / n * 100) if n > 0 else 0
            }
        }

        if len(stroke_times) >= 2:
            intervals = [stroke_times[i] - stroke_times[i - 1]
                         for i in range(1, len(stroke_times))]
            avg_interval = sum(intervals) / len(intervals) / 1000.0
            metrics['avg_stroke_time'] = avg_interval

            if len(intervals) >= 2:
                variance = sum((iv - avg_interval * 1000) ** 2
                               for iv in intervals) / len(intervals)
                cv = (variance ** 0.5 / (avg_interval * 1000)) * 100 if avg_interval > 0 else 0
                metrics['consistency'] = max(0, 100 - min(cv, 100))

        return metrics


def start_server(port=8004):
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
