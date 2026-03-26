#!/usr/bin/env python3
"""
WiFi Session Processor - Processes data from ESP32 WiFi server using Python StrokeProcessor
Fetches /data.json from ESP32, processes with StrokeProcessor, returns processed results.
Supports multiple sessions per sync, user registration, ideal strokes, and SQLite persistence.
"""
import json
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from simple_imu_visualizer import StrokeProcessor, SimpleKalmanFilter, BreathingDetector, align_sessions_by_hop
import database as db

ESP32_URL = 'http://192.168.4.1'
CACHE_DIR = os.path.join(os.path.dirname(__file__), '.session_cache')
RAW_CACHE = os.path.join(CACHE_DIR, 'last_raw.json')
PROCESSED_CACHE = os.path.join(CACHE_DIR, 'last_processed.json')
os.makedirs(CACHE_DIR, exist_ok=True)


class WiFiSessionHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        """Suppress default request logging to keep terminal clean."""
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/integrated_session_viewer.html':
            self._serve_html()
        elif self.path == '/app.css':
            self._serve_static('app.css', 'text/css')
        elif self.path == '/app.js':
            self._serve_static('app.js', 'application/javascript')
        elif self.path == '/process':
            self._process_wifi_session()
        elif self.path == '/cached':
            self._serve_cached()
        elif self.path.startswith('/viz/'):
            self._serve_viz_module(self.path[5:])
        elif self.path == '/api/user':
            self._get_user_profile()
        elif self.path == '/api/ideal_stroke':
            self._get_ideal_stroke()
        elif self.path == '/api/device_info':
            self._get_device_info()
        elif self.path == '/api/devices':
            self._get_devices()
        elif self.path == '/api/sessions':
            self._get_sessions()
        elif self.path.startswith('/api/sessions/'):
            parts = self.path.split('/')
            if len(parts) >= 4 and parts[3].isdigit():
                self._get_session_detail(int(parts[3]))
            else:
                self._get_sessions()
        elif self.path.startswith('/api/progress'):
            self._get_progress()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/reprocess':
            self._reprocess_session()
        elif self.path == '/api/register':
            self._register_user()
        elif self.path == '/api/ideal_stroke':
            self._upload_ideal_stroke()
        elif self.path == '/api/ideal_stroke/delete':
            self._delete_ideal_stroke()
        elif self.path == '/api/ideal_stroke/push':
            self._push_ideal_to_device()
        elif self.path == '/api/user_config/push':
            self._push_user_config_to_device()
        elif self.path == '/api/test_buzz':
            self._test_buzz()
        elif self.path == '/api/devices/register':
            self._register_device()
        elif self.path.startswith('/api/devices/delete'):
            self._delete_device()
        elif self.path == '/api/sessions/save':
            self._save_session()
        elif self.path == '/api/sessions/merge':
            self._merge_sessions()
        else:
            self.send_response(404)
            self.end_headers()

    # ── User Registration (SQLite) ──

    def _register_user(self):
        try:
            body = self._read_body()
            uid = db.upsert_user(
                name=body.get('name', 'Swimmer'),
                height_cm=body.get('height_cm', 0),
                wingspan_cm=body.get('wingspan_cm', 0),
                skill_level=body.get('skill_level', 'beginner')
            )
            user = db.get_user()
            self._send_json({'status': 'ok', 'user_id': uid, 'profile': user})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _get_user_profile(self):
        user = db.get_user()
        self._send_json(user if user else {})

    # ── Device Registration ──

    def _register_device(self):
        try:
            body = self._read_body()
            did = db.register_device(
                user_id=body.get('user_id', 1),
                device_hw_id=body.get('device_hw_id', 0),
                role=body.get('role', 'wrist_right'),
                name=body.get('name', '')
            )
            self._send_json({'status': 'ok', 'device_id': did})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _delete_device(self):
        try:
            body = self._read_body()
            did = body.get('device_id')
            if not did:
                self._send_json({'error': 'Missing device_id'}, 400)
                return
            db.delete_device(did)
            self._send_json({'status': 'ok'})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _get_devices(self):
        user = db.get_user()
        uid = user['id'] if user else None
        devices = db.get_devices(uid)
        self._send_json({'devices': devices})

    # ── Ideal Strokes ──

    def _upload_ideal_stroke(self):
        try:
            body = self._read_body()
            user = db.get_user()
            uid = user['id'] if user else None
            samples = body.get('samples', [])

            # Save to database
            if uid:
                db.save_ideal_stroke(uid, body.get('name', 'Default'), samples, len(samples))

            # Also save to file cache for backwards compatibility
            ideal_path = os.path.join(CACHE_DIR, 'ideal_stroke.json')
            with open(ideal_path, 'w') as f:
                json.dump(body, f)

            print(f"Ideal stroke saved: {len(samples)} samples")
            self._send_json({'status': 'ok', 'samples': len(samples)})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _get_ideal_stroke(self):
        user = db.get_user()
        uid = user['id'] if user else None
        ideal = db.get_latest_ideal_stroke(uid)
        if ideal:
            self._send_json({'samples': ideal['lia_data'], 'name': ideal['name'], 'num_samples': ideal['num_samples']})
        else:
            # Fall back to file cache
            ideal_path = os.path.join(CACHE_DIR, 'ideal_stroke.json')
            if os.path.exists(ideal_path):
                with open(ideal_path, 'r') as f:
                    self._send_json(json.load(f))
            else:
                self._send_json({'samples': []})

    def _push_ideal_to_device(self):
        """Forward ideal stroke data to ESP32 device."""
        try:
            body = self._read_body()
            samples = body.get('samples', [])
            
            # ESP32 firmware strictly limits ideal strokes to 200 samples to save RAM
            if len(samples) > 200:
                print(f"Downsampling ideal stroke from {len(samples)} to 200 samples for ESP32 constraints")
                # Evenly distribute 200 points across the original array
                step = len(samples) / 200.0
                downsampled = [samples[int(i * step)] for i in range(200)]
                body['samples'] = downsampled
                
            data = json.dumps(body).encode()
            req = urllib.request.Request(
                f'{ESP32_URL}/api/ideal_stroke',
                data=data,
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            resp = urllib.request.urlopen(req, timeout=5)
            result = json.loads(resp.read().decode())
            self._send_json({'status': 'ok', 'device_response': result})
        except Exception as e:
            self._send_json({'error': f'Failed to push to device: {str(e)}'}, 500)

    def _delete_ideal_stroke(self):
        try:
            # Delete from internal cache
            ideal_path = os.path.join(CACHE_DIR, 'ideal_stroke.json')
            if os.path.exists(ideal_path):
                os.remove(ideal_path)
            
            user = db.get_user()
            uid = user['id'] if user else None
            if uid:
                conn = db._connect()
                c = conn.cursor()
                c.execute('DELETE FROM ideal_strokes WHERE user_id = ?', (uid,))
                conn.commit()
                conn.close()
            
            # Forward delete to ESP32
            req = urllib.request.Request(
                f'{ESP32_URL}/api/ideal_stroke',
                method='DELETE'
            )
            try:
                resp = urllib.request.urlopen(req, timeout=5)
                result = json.loads(resp.read().decode())
            except Exception:
                result = {'status': 'device offline or unreachable'}
                
            self._send_json({'status': 'ok', 'device_response': result})
        except Exception as e:
            self._send_json({'error': f'Failed to delete: {str(e)}'}, 500)

    def _push_user_config_to_device(self):
        try:
            body = self._read_body()
            data = json.dumps(body).encode()
            req = urllib.request.Request(
                f'{ESP32_URL}/api/user_config',
                data=data,
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            resp = urllib.request.urlopen(req, timeout=5)
            result = json.loads(resp.read().decode())
            self._send_json({'status': 'ok', 'device_response': result})
        except Exception as e:
            self._send_json({'error': f'Failed to push user config: {str(e)}'}, 500)

    def _test_buzz(self):
        try:
            req = urllib.request.Request(
                f'{ESP32_URL}/api/test_buzz',
                data=b'{}',
                headers={'Content-Type': 'application/json', 'Content-Length': '2'},
                method='POST'
            )
            resp = urllib.request.urlopen(req, timeout=4)
            result = json.loads(resp.read().decode())
            self._send_json({'status': 'ok', 'device_response': result})
        except urllib.error.HTTPError as e:
            self._send_json({'error': f'HTTP Error {e.code}: {e.reason}'}, 500)
        except Exception as e:
            self._send_json({'error': f'Failed to send test buzz: {str(e)}'}, 500)

    # ── Device Info ──

    def _get_device_info(self):
        try:
            resp = urllib.request.urlopen(f'{ESP32_URL}/api/device_info', timeout=3)
            data = json.loads(resp.read().decode())
            self._send_json(data)
        except Exception:
            self._send_json({'status': 'disconnected', 'message': 'Device not reachable'})

    # ── Session Persistence ──

    def _save_session(self):
        try:
            body = self._read_body()
            user = db.get_user()
            uid = user['id'] if user else None
            sid = db.save_session(
                user_id=uid,
                device_ids=body.get('device_ids', []),
                processed_data=body.get('processed_data', []),
                metrics=body.get('metrics', {}),
                duration=body.get('duration', 0),
                raw_data=body.get('raw_data')
            )
            # Save progress snapshot
            metrics = body.get('metrics', {})
            if uid and metrics:
                form_score = self._compute_form_score(metrics)
                db.save_progress(
                    user_id=uid,
                    session_id=sid,
                    stroke_rate=metrics.get('stroke_rate', 0),
                    consistency=metrics.get('consistency', 0),
                    avg_deviation=metrics.get('avg_deviation', 0),
                    avg_entry_angle=metrics.get('avg_entry_angle', 0),
                    form_score=form_score,
                    stroke_count=metrics.get('stroke_count', 0)
                )
            self._send_json({'status': 'ok', 'session_id': sid})
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({'error': str(e)}, 500)

    def _get_sessions(self):
        user = db.get_user()
        uid = user['id'] if user else None
        limit = 20
        sessions = db.get_sessions(uid, limit)
        self._send_json({'sessions': sessions})

    def _get_session_detail(self, session_id):
        s = db.get_session(session_id)
        if not s:
            self._send_json({'error': 'Session not found'}, 404)
            return
        self._send_json({'session': s})

    def _get_progress(self):
        user = db.get_user()
        if not user:
            self._send_json({'progress': []})
            return
        progress = db.get_progress(user['id'], 50)
        self._send_json({'progress': progress})

    # ── Multi-Device Merge ──

    def _merge_sessions(self):
        try:
            body = self._read_body()
            session_ids = body.get('session_ids', [])
            if len(session_ids) < 2:
                self._send_json({"error": "Need at least 2 sessions to merge"}, 400)
                return
            
            # Load raw data for these sessions
            sessions_to_merge = []
            for sid in session_ids:
                s = db.get_session(sid)
                if s and s.get('raw_data'):
                    sessions_to_merge.append({
                        'id': sid,
                        'name': f"Session {sid}",
                        'data': s['raw_data']
                    })
            
            if len(sessions_to_merge) < 2:
                self._send_json({"error": "Could not load raw data for at least 2 sessions"}, 400)
                return

            # Align them
            aligned_sessions, meta = align_sessions_by_hop(sessions_to_merge)
            if not meta.get('aligned'):
                self._send_json({"error": "Failed to align sessions: " + meta.get('reason', '')}, 400)
                return
            
            # Now reprocess the aligned sessions together
            processed_aligned_sessions = []
            for sess in aligned_sessions:
                processor = StrokeProcessor(batch_mode=True)
                p_data = []
                for sample in sess.get('data', []):
                    lia_x = sample.get('lia_x', sample.get('ax', 0))
                    lia_y = sample.get('lia_y', sample.get('ay', 0))
                    lia_z = sample.get('lia_z', sample.get('az', 0))
                    data_dict = {
                        't': sample.get('t', sample.get('timestamp', 0)),
                        'lia_x': lia_x, 'lia_y': lia_y, 'lia_z': lia_z,
                        'gx': sample.get('gx', 0), 'gy': sample.get('gy', 0), 'gz': sample.get('gz', 0),
                        'qw': sample.get('qw', 1), 'qx': sample.get('qx', 0), 'qy': sample.get('qy', 0), 'qz': sample.get('qz', 0),
                        'haptic': sample.get('haptic', sample.get('haptic_fired', False)),
                        'deviation': sample.get('deviation', sample.get('deviation_score', 0.0)),
                        'dev_id': sample.get('dev_id', sample.get('device_id', 0)),
                        'dev_role': sample.get('dev_role', sample.get('device_role', 0)),
                        'cal': sample.get('cal', {
                            'sys': sample.get('cal_sys', 0), 'accel': sample.get('cal_accel', 0),
                            'gyro': sample.get('cal_gyro', 0), 'mag': sample.get('cal_mag', 0)
                        })
                    }
                    result = processor.process_data(data_dict)
                    if result:
                        p_data.append(json.loads(result))
                
                metrics = self._calculate_stroke_metrics(p_data, processor)
                duration = 0
                if len(p_data) >= 2:
                    duration = (p_data[-1].get('timestamp', 0) - p_data[0].get('timestamp', 0)) / 1000.0

                processed_aligned_sessions.append({
                    'id': sess.get('id'),
                    'name': sess.get('name'),
                    'processed_data': p_data,
                    'metrics': metrics,
                    'duration': duration
                })

            self._send_json({
                'status': 'ok',
                'aligned_sessions': processed_aligned_sessions,
                'meta': meta
            })
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({"error": str(e)}, 500)

    # ── Reprocess ──

    def _reprocess_session(self):
        try:
            body = self._read_body()
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
                    'haptic': sample.get('haptic_fired', False),
                    'deviation': sample.get('deviation_score', 0.0),
                    'dev_id': sample.get('device_id', 0),
                    'dev_role': sample.get('device_role', 0),
                    'cal': sample.get('cal', {
                        'sys': cal.get('sys', 0), 'accel': cal.get('accel', 0),
                        'gyro': cal.get('gyro', 0), 'mag': cal.get('mag', 0)
                    })
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
        if os.path.exists(PROCESSED_CACHE):
            with open(PROCESSED_CACHE, 'r') as f:
                self._send_json(json.load(f))
        else:
            self._send_json({"error": "No cached data available"}, 404)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, DELETE')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        html_path = os.path.join(os.path.dirname(__file__), 'integrated_session_viewer.html')
        with open(html_path, 'rb') as f:
            self.wfile.write(f.read())


    _ALLOWED_JS = {
    'app.js',
    'simple_imu_3d_viz.js',
    'three.min.js',
    'OrbitControls.js',
    'chart.umd.min.js'
}

    def _serve_static(self, filename, content_type):
        path = os.path.join(os.path.dirname(__file__), filename)
        if not os.path.exists(path):
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        with open(path, 'rb') as f:
            self.wfile.write(f.read())

    def _serve_viz_module(self, filename):
        if filename in self._ALLOWED_JS:
            self._serve_static(filename, 'application/javascript')
        else:
            self.send_response(404)
            self.end_headers()

    def _read_body(self):
        length = int(self.headers.get('Content-Length', 0))
        return json.loads(self.rfile.read(length)) if length > 0 else {}

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

            sessions_raw = result.get('sessions', [])
            expected_total = result.get('totalSamples', None)

            # Backward compat: old format {"files":N,"data":[...]}
            if not sessions_raw and result.get('data'):
                sessions_raw = [{'id': 1, 'name': 'Session 1', 'data': result['data']}]

            if not sessions_raw:
                self._send_json({"error": "No sessions in ESP32 response"}, status=404)
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
                breath_detector = BreathingDetector()
                processed_data = []
                breath_events = []

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
                        'haptic_fired': sample.get('haptic', sample.get('haptic_fired', False)),
                        'deviation_score': sample.get('deviation', sample.get('deviation_score', 0.0)),
                        'dev_id': sample.get('dev_id', 0),
                        'dev_role': sample.get('dev_role', 0),
                        'cal': sample.get('cal', {
                            'sys': sample.get('cal_sys', 0),
                            'accel': sample.get('cal_accel', 0),
                            'gyro': sample.get('cal_gyro', 0),
                            'mag': sample.get('cal_mag', 0),
                        })
                    }

                    processed_json = processor.process_data(data_dict)
                    if processed_json:
                        p = json.loads(processed_json)
                        q = p.get('quaternion', {})
                        be = breath_detector.feed(
                            data_dict['t'],
                            q.get('qw', 1), q.get('qx', 0),
                            q.get('qy', 0), q.get('qz', 0)
                        )
                        if be:
                            breath_events.append(be)
                            p['breath_event'] = be
                        p['breath_count'] = breath_detector.breath_count
                        processed_data.append(p)

                metrics = self._calculate_stroke_metrics(processed_data, processor)
                breath_stats = breath_detector.get_stats()
                metrics['breathing'] = breath_stats

                duration = 0
                if len(processed_data) >= 2:
                    t0 = processed_data[0].get('timestamp', 0)
                    t1 = processed_data[-1].get('timestamp', 0)
                    duration = (t1 - t0) / 1000.0

                now_str = datetime.now().strftime('%b %d, %I:%M %p').lstrip('0')
                sess_name = f"Session {sess.get('id', '?')} - {now_str}"

                processed_sessions.append({
                    'id': sess.get('id', 0),
                    'name': sess_name,
                    'processed_data': processed_data,
                    'metrics': metrics,
                    'raw_samples': len(data),
                    'processed_samples': len(processed_data),
                    'duration': duration,
                    'syncedAt': datetime.now().isoformat(),
                    'raw_data': data
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
                'stroke_count': 0, 'turn_count': 0, 'duration': 0, 'stroke_rate': 0,
                'avg_stroke_time': 0, 'consistency': 0, 'peak_accel_avg': 0,
                'avg_entry_angle': 0, 'ideal_entry_angle': 30.0,
                'phase_pcts': {'glide': 0, 'catch': 0, 'pull': 0, 'recovery': 0},
                'haptic_count': 0, 'avg_deviation': 0,
                'cal_quality': {'accel_pct': 0, 'gyro_pct': 0},
                'stroke_breakdown': []
            }

        stroke_times = []
        peak_accels = []
        haptic_count = 0
        deviation_scores = []
        last_stroke_count = 0
        stroke_breakdown = []

        for p in processed_data:
            stroke_count = p.get('stroke_count', 0)
            if stroke_count > last_stroke_count:
                t_ms = p.get('timestamp', 0)
                stroke_times.append(t_ms)
                debug = p.get('stroke_debug', {})
                if debug.get('accel_mag', 0) > 0:
                    peak_accels.append(debug['accel_mag'])
                
                dev = p.get('deviation_score', 0)
                stroke_breakdown.append({
                    'number': stroke_count,
                    'timestamp_s': t_ms / 1000.0,
                    'entry_angle': p.get('entry_angle', 0),
                    'haptic_fired': p.get('haptic_fired', False),
                    'deviation': dev
                })
            last_stroke_count = stroke_count

            if p.get('haptic_fired', False):
                haptic_count += 1
            dev = p.get('deviation_score', 0)
            if dev > 0:
                deviation_scores.append(dev)

        duration = (processed_data[-1].get('timestamp', 0) -
                    processed_data[0].get('timestamp', 0)) / 1000.0

        accel_good = sum(1 for p in processed_data
                        if (p.get('calibration', {}).get('accel', 0) >= 2))
        gyro_good = sum(1 for p in processed_data
                        if (p.get('calibration', {}).get('gyro', 0) >= 2))
        n = len(processed_data)

        # Get entry angles from processor
        avg_entry_angle = 0
        if hasattr(processor, 'entry_angles') and processor.entry_angles:
            avg_entry_angle = sum(processor.entry_angles) / len(processor.entry_angles)

        phase_pcts = {'glide': 0, 'catch': 0, 'pull': 0, 'recovery': 0}
        if hasattr(processor, 'last_phase_pcts'):
            phase_pcts = dict(processor.last_phase_pcts)

        avg_deviation = sum(deviation_scores) / len(deviation_scores) if deviation_scores else 0

        metrics = {
            'stroke_count': processor.stroke_count,
            'turn_count': processor.turn_count,
            'duration': duration,
            'stroke_rate': (processor.stroke_count / duration * 60) if duration > 0 else 0,
            'avg_stroke_time': 0,
            'consistency': 0,
            'peak_accel_avg': sum(peak_accels) / len(peak_accels) if peak_accels else 0,
            'avg_entry_angle': round(avg_entry_angle, 1),
            'ideal_entry_angle': getattr(processor, 'ideal_entry_angle', 30.0),
            'phase_pcts': phase_pcts,
            'haptic_count': haptic_count,
            'avg_deviation': round(avg_deviation, 3),
            'cal_quality': {
                'accel_pct': round(accel_good / n * 100) if n > 0 else 0,
                'gyro_pct': round(gyro_good / n * 100) if n > 0 else 0
            },
            'stroke_breakdown': stroke_breakdown
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

    @staticmethod
    def _compute_form_score(metrics):
        """Compute a 0-10 form score from metrics."""
        score = 5.0  # baseline
        # Consistency bonus (0-100% → 0-3 points)
        score += min(metrics.get('consistency', 0) / 100 * 3, 3)
        # Low deviation bonus (0 = +2, 1 = 0, >1 = -1)
        dev = metrics.get('avg_deviation', 0)
        if dev < 0.3:
            score += 2
        elif dev < 0.7:
            score += 1
        elif dev > 1.0:
            score -= 1
        # Entry angle quality (within 15-40 ideal range)
        angle = metrics.get('avg_entry_angle', 0)
        if 15 <= angle <= 40:
            score += 1
        elif angle > 0:
            score -= 0.5
        return round(max(0, min(10, score)), 1)


def start_server(port=8004):
    db.init_db()
    server = ThreadingHTTPServer(('0.0.0.0', port), WiFiSessionHandler)
    print(f'GoldenForm Session Processor running on http://localhost:{port}')
    print(f'Open: http://localhost:{port}/')
    print('Press Ctrl+C to stop')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down...')
        server.shutdown()


if __name__ == '__main__':
    port = int(os.environ.get('PORT', 8004))
    start_server(port)
