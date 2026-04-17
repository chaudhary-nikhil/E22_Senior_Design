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
import uuid
import argparse
import threading
import queue
import urllib.parse
import time

sys.path.insert(0, os.path.dirname(__file__))
from simple_imu_visualizer import StrokeProcessor, align_sessions_by_hop, land_demo_enabled
import database as db

ESP32_URL = 'http://192.168.4.1'
CACHE_DIR = os.path.join(os.path.dirname(__file__), '.session_cache')
RAW_CACHE = os.path.join(CACHE_DIR, 'last_raw.json')
PROCESSED_CACHE = os.path.join(CACHE_DIR, 'last_processed.json')
INSTANCE_ID_PATH = os.path.join(CACHE_DIR, 'gf_instance_id')
IDEAL_CACHE_FILE = os.path.join(CACHE_DIR, 'ideal_stroke.json')
os.makedirs(CACHE_DIR, exist_ok=True)


def _env_flag_true(key):
    return (os.environ.get(key) or '').strip().lower() in ('1', 'true', 'yes', 'on')


def _replay_stroke_processor():
    """Batch StrokeProcessor: GF_REPLAY_INTEGRATION, GF_LAND_DEMO, min-cal policy.

    Min cal (0401fbde = accel+gyro ≥ 2): **on by default** for pool batch replay.
    - ``GF_RELAX_IMU_CAL=true`` — MIN_CAL 0 (IMUPLUS / weak logs).
    - ``GF_STRICT_IMU_CAL=true`` — force strict even on land demo.
    Pool replay: ``GF_LAND_DEMO`` unset → off (0401fbde strict min-cal). Set ``GF_LAND_DEMO=true``
    for dry-land thresholds + relaxed min-cal.
    """
    # uart_match = 0401fbde motion gate (gy+|LIA|), segment resets limit drift.
    # stroke_pull = integrate only during underwater pull window (least drift vs recovery air).
    mode = (os.environ.get('GF_REPLAY_INTEGRATION') or 'uart_match').strip().lower()
    if mode not in ('stroke_pull', 'uart_match', 'motion_segment'):
        mode = 'uart_match'
    land = land_demo_enabled(default=False)
    if _env_flag_true('GF_STRICT_IMU_CAL'):
        strict = True
    elif _env_flag_true('GF_RELAX_IMU_CAL'):
        strict = False
    elif land:
        strict = False
    else:
        strict = True
    return StrokeProcessor(
        batch_mode=True,
        replay_integration_mode=mode,
        land_demo=land,
        strict_min_cal=strict,
    )


def _load_local_env():
    """Load optional dashboard/.env into os.environ (does not override existing vars)."""
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.env')
    if not os.path.isfile(env_path):
        return
    try:
        with open(env_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if '=' not in line:
                    continue
                k, _, v = line.partition('=')
                k, v = k.strip(), v.strip().strip('"').strip("'")
                if k and k not in os.environ:
                    os.environ[k] = v
    except OSError:
        pass


_load_local_env()

# Set in start_server — used by /api/bootstrap
SERVER_INSTANCE_ID = ''
DEMO_MODE = False

# ── Real-time events (SSE) ──
_SSE_LOCK = threading.Lock()
_SSE_CLIENTS = []  # list[queue.SimpleQueue]
_SSE_LAST_DEVICE_INFO = None
_SSE_DEVICE_POLL_THREAD_STARTED = False


def _sse_publish(event: str, data: dict):
    payload = {'event': event, 'data': data, 'ts': int(time.time() * 1000)}
    with _SSE_LOCK:
        clients = list(_SSE_CLIENTS)
    for q in clients:
        try:
            q.put(payload)
        except Exception:
            pass


def _sse_start_device_poll_thread():
    global _SSE_DEVICE_POLL_THREAD_STARTED
    if _SSE_DEVICE_POLL_THREAD_STARTED:
        return
    _SSE_DEVICE_POLL_THREAD_STARTED = True

    def _run():
        global _SSE_LAST_DEVICE_INFO
        while True:
            time.sleep(1.0)
            with _SSE_LOCK:
                has_clients = len(_SSE_CLIENTS) > 0
            if not has_clients:
                continue
            try:
                resp = urllib.request.urlopen(f'{ESP32_URL}/api/device_info', timeout=2.0)
                d = json.loads(resp.read().decode())
            except Exception:
                d = {'status': 'disconnected'}
            # Only publish when it changes (basic string compare is fine for small JSON).
            try:
                s = json.dumps(d, sort_keys=True)
            except Exception:
                s = None
            if s and s == _SSE_LAST_DEVICE_INFO:
                continue
            _SSE_LAST_DEVICE_INFO = s
            _sse_publish('device_info', d if isinstance(d, dict) else {'status': 'disconnected'})

    t = threading.Thread(target=_run, name='gf_sse_device_poll', daemon=True)
    t.start()


def _get_instance_id(demo: bool) -> str:
    """Stable id across restarts (persisted file). Demo: new id each server start (see _prepare_server_state)."""
    if demo:
        if os.path.isfile(INSTANCE_ID_PATH):
            with open(INSTANCE_ID_PATH, 'r', encoding='utf-8') as f:
                s = f.read().strip()
                if s:
                    return s
        iid = str(uuid.uuid4())
        with open(INSTANCE_ID_PATH, 'w', encoding='utf-8') as f:
            f.write(iid)
        return iid
    if os.path.isfile(INSTANCE_ID_PATH):
        with open(INSTANCE_ID_PATH, 'r', encoding='utf-8') as f:
            s = f.read().strip()
            if s:
                return s
    iid = str(uuid.uuid4())
    with open(INSTANCE_ID_PATH, 'w', encoding='utf-8') as f:
        f.write(iid)
    return iid


def _prepare_server_state(demo: bool, fresh: bool = False) -> None:
    """
    Demo: separate demo SQLite file (always empty on each server start) + clear file caches.
    Prod: goldenform.db; pass fresh=True once to delete existing DB for a clean install.
    """
    global SERVER_INSTANCE_ID, DEMO_MODE
    DEMO_MODE = demo
    db.configure_database(demo=demo, fresh=fresh)
    if demo:
        for fn in (RAW_CACHE, PROCESSED_CACHE, IDEAL_CACHE_FILE):
            try:
                if os.path.isfile(fn):
                    os.remove(fn)
            except OSError:
                pass
        # New instance id every demo boot so /api/bootstrap changes; the dashboard clears
        # goldenform_* localStorage when instance_id differs (matches empty demo DB).
        try:
            if os.path.isfile(INSTANCE_ID_PATH):
                os.remove(INSTANCE_ID_PATH)
        except OSError:
            pass
    SERVER_INSTANCE_ID = _get_instance_id(demo=demo)


class WiFiSessionHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        """Suppress default request logging to keep terminal clean."""
        pass

    def _auth_bearer_token(self):
        auth = self.headers.get('Authorization', '')
        if auth.lower().startswith('bearer '):
            return auth[7:].strip()
        return None

    def _auth_token_from_query(self):
        try:
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query or '')
            t = (q.get('token') or [None])[0]
            return t
        except Exception:
            return None

    def _current_user(self):
        t = self._auth_bearer_token() or self._auth_token_from_query()
        if not t:
            return None
        return db.get_user_from_token(t)

    def _require_user(self):
        u = self._current_user()
        if not u:
            self._send_json({'error': 'Unauthorized', 'auth_required': True}, 401)
            return None
        return u

    def do_GET(self):
        if self.path == '/' or self.path == '/integrated_session_viewer.html':
            self._serve_html()
        elif self.path == '/app.css':
            self._serve_static('app.css', 'text/css')
        elif self.path == '/app.js':
            self._serve_static('app.js', 'application/javascript')
        elif self.path.startswith('/js/') and self.path.endswith('.js'):
            base_dir = os.path.dirname(os.path.abspath(__file__))
            rel = self.path.lstrip('/')
            full = os.path.normpath(os.path.join(base_dir, rel))
            try:
                if os.path.commonpath([base_dir, full]) != base_dir:
                    raise ValueError('path escape')
            except ValueError:
                self.send_response(403)
                self.end_headers()
                return
            if os.path.isfile(full):
                self._serve_static(rel, 'application/javascript')
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path == '/process':
            self._process_wifi_session()
        elif self.path == '/api/user':
            self._get_user_profile()
        elif self.path == '/api/ideal_stroke':
            self._get_ideal_stroke()
        elif self.path == '/api/ideal_stroke/set_from_stroke':
            self._set_from_stroke_api()
        elif self.path == '/api/device_info':
            self._get_device_info()
        elif self.path.startswith('/api/events'):
            self._sse_events()
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
        elif self.path == '/api/bootstrap':
            self._get_bootstrap()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        if path == '/api/register':
            self._register_user()
        elif path == '/api/auth/register':
            self._auth_register()
        elif path == '/api/auth/login':
            self._auth_login()
        elif path == '/api/auth/logout':
            self._auth_logout()
        elif path == '/api/profile':
            self._update_profile()
        elif path == '/api/ideal_stroke':
            self._upload_ideal_stroke()
        elif path == '/api/ideal_stroke/set_from_stroke':
            self._set_ideal_from_stroke()
        elif path == '/api/ideal_stroke/delete':
            self._delete_ideal_stroke()
        elif path == '/api/ideal_stroke/push':
            self._push_ideal_to_device()
        elif path == '/api/user_config/push':
            self._push_user_config_to_device()
        elif path == '/api/test_buzz':
            self._test_buzz()
        elif path == '/api/devices/register':
            self._register_device()
        elif path == '/api/registration_done':
            self._notify_registration_done()
        elif path.startswith('/api/devices/delete'):
            self._delete_device()
        elif path == '/api/sessions/save':
            self._save_session()
        elif path == '/api/sessions/merge':
            self._merge_sessions()
        elif path == '/api/coaching/insights':
            self._coaching_insights()
        else:
            self.send_response(404)
            self.end_headers()

    def _sse_events(self):
        u = self._require_user()
        if not u:
            return
        # Register client queue
        q = queue.SimpleQueue()
        with _SSE_LOCK:
            _SSE_CLIENTS.append(q)
        _sse_start_device_poll_thread()

        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            # Initial hello
            hello = {'demo': DEMO_MODE}
            self.wfile.write(b"event: hello\n")
            self.wfile.write(("data: " + json.dumps(hello) + "\n\n").encode())
            self.wfile.flush()

            last_send = time.time()
            while True:
                try:
                    msg = q.get(timeout=15.0)
                except Exception:
                    msg = None
                if msg is None:
                    # keepalive
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                ev = msg.get('event', 'message')
                data = msg.get('data', {})
                self.wfile.write(("event: " + str(ev) + "\n").encode())
                self.wfile.write(("data: " + json.dumps(data) + "\n\n").encode())
                self.wfile.flush()
                last_send = time.time()
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            pass
        finally:
            with _SSE_LOCK:
                try:
                    _SSE_CLIENTS.remove(q)
                except ValueError:
                    pass

    # ── Auth & profile (SQLite) ──

    def _register_user(self):
        """Legacy: single-field registration removed — use /api/auth/register."""
        self._send_json({
            'error': 'Create an account with POST /api/auth/register (email + password).',
            'deprecated': True
        }, 400)

    def _auth_register(self):
        try:
            body = self._read_body()
            email = (body.get('email') or '').strip()
            password = body.get('password') or ''
            name = (body.get('name') or '').strip() or 'Swimmer'
            if not email or not password:
                self._send_json({'error': 'Email and password are required'}, 400)
                return
            try:
                uid = db.create_user(
                    email=email,
                    password_plain=password,
                    name=name,
                    height_cm=float(body.get('height_cm', 0) or 0),
                    wingspan_cm=float(body.get('wingspan_cm', 0) or 0),
                    skill_level=str(body.get('skill_level', 'beginner') or 'beginner'),
                    pool_length=float(body.get('pool_length', 25) or 25),
                )
            except ValueError as e:
                self._send_json({'error': str(e)}, 400)
                return
            token = db.create_session(uid)
            user = db.get_user_by_id(uid)
            self._send_json({'status': 'ok', 'token': token, 'profile': user})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _auth_login(self):
        try:
            body = self._read_body()
            user = db.verify_login(body.get('email'), body.get('password'))
            if not user:
                self._send_json({'error': 'Invalid email or password'}, 401)
                return
            token = db.create_session(user['id'])
            self._send_json({'status': 'ok', 'token': token, 'profile': user})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _auth_logout(self):
        try:
            t = self._auth_bearer_token()
            if t:
                db.revoke_auth_session(t)
            _sse_publish('auth', {'status': 'logout'})
            self._send_json({'status': 'ok'})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _update_profile(self):
        try:
            u = self._require_user()
            if not u:
                return
            body = self._read_body()
            kw = {}
            if 'email' in body:
                kw['email'] = body.get('email')
            if 'name' in body:
                kw['name'] = body.get('name')
            if 'height_cm' in body:
                kw['height_cm'] = float(body.get('height_cm') or 0)
            if 'wingspan_cm' in body:
                kw['wingspan_cm'] = float(body.get('wingspan_cm') or 0)
            if 'skill_level' in body:
                kw['skill_level'] = body.get('skill_level')
            if 'pool_length' in body:
                kw['pool_length'] = float(body.get('pool_length') or 25)
            db.update_user_profile(u['id'], **kw)
            user = db.get_user_by_id(u['id'])
            _sse_publish('profile', {'user_id': u['id']})
            self._send_json({'status': 'ok', 'profile': user})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _get_user_profile(self):
        u = self._current_user()
        if not u:
            self._send_json({'error': 'Unauthorized', 'auth_required': True}, 401)
            return
        self._send_json(u)

    # ── Device Registration ──

    def _register_device(self):
        try:
            u = self._require_user()
            if not u:
                return
            body = self._read_body()
            did = db.register_device(
                user_id=u['id'],
                device_hw_id=body.get('device_hw_id', 0),
                role=body.get('role', 'wrist_right'),
                name=body.get('name', ''),
                wifi_ssid=body.get('wifi_ssid', '') or ''
            )
            _sse_publish('devices', {'user_id': u['id']})
            self._send_json({'status': 'ok', 'device_id': did})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _notify_registration_done(self):
        """Forward to ESP32 so it clears registration linger + stops status LED + closes AP."""
        try:
            req = urllib.request.Request(
                f'{ESP32_URL}/api/registration_done',
                data=b'{}',
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            resp = urllib.request.urlopen(req, timeout=4)
            json.loads(resp.read().decode())
            self._send_json({'status': 'ok'})
        except Exception as e:
            self._send_json({'status': 'error', 'error': str(e)}, 502)

    def _delete_device(self):
        try:
            u = self._require_user()
            if not u:
                return
            body = self._read_body()
            did = body.get('device_id')
            if not did:
                self._send_json({'error': 'Missing device_id'}, 400)
                return
            db.delete_device(did)
            _sse_publish('devices', {'user_id': u['id']})
            self._send_json({'status': 'ok'})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _get_devices(self):
        u = self._require_user()
        if not u:
            return
        devices = db.get_devices(u['id'])
        self._send_json({'devices': devices})

    # ── Ideal Strokes ──

    def _upload_ideal_stroke(self):
        try:
            u = self._require_user()
            if not u:
                return
            body = self._read_body()
            uid = u['id']
            samples = body.get('samples', [])

            db.save_ideal_stroke(uid, body.get('name', 'Default'), samples, len(samples))

            # Also save to file cache for backwards compatibility
            ideal_path = os.path.join(CACHE_DIR, 'ideal_stroke.json')
            with open(ideal_path, 'w') as f:
                json.dump(body, f)

            print(f"Ideal stroke saved: {len(samples)} samples")
            self._send_json({'status': 'ok', 'samples': len(samples)})
        except Exception as e:
            self._send_json({'error': str(e)}, 500)
    def _set_from_stroke_api(self):
        try:
            # For GET request with query params?
            # session_id, stroke_num
            # or use POST
            self.send_response(405) # Allow only POST
            self.end_headers()
        except: pass

    # In WiFiSessionHandler do_POST
    def _set_ideal_from_stroke(self):
        try:
            body = self._read_body()
            session_id = body.get('session_id')
            stroke_num = body.get('stroke_num')
            try:
                session_id = int(session_id)
            except (TypeError, ValueError):
                self._send_json({'error': 'Invalid session_id'}, 400)
                return
            try:
                stroke_num = int(stroke_num)
            except (TypeError, ValueError):
                self._send_json({'error': 'Invalid stroke_num'}, 400)
                return

            u = self._require_user()
            if not u:
                return
            uid = u['id']
            if not db.session_owned_by(session_id, uid):
                self._send_json({'error': 'Forbidden'}, 403)
                return

            session = db.get_session(session_id)
            if not session:
                self._send_json({'error': 'Session not found'}, 404)
                return

            pd = session.get('processed_data', [])

            use_fw = any(int(x.get('strokes') or 0) > 0 for x in pd)

            def stroke_idx(d):
                if use_fw:
                    return int(d.get('strokes') or 0)
                return int(d.get('stroke_count') or 0)

            stroke_samples = [d for d in pd if stroke_idx(d) == stroke_num]

            if not stroke_samples:
                self._send_json({'error': f'Stroke {stroke_num} not found in session (check stroke fields)'}, 404)
                return

            lia_data = []
            for d in stroke_samples:
                lia = d.get('lia') or {}
                acc = d.get('acceleration', {}) or {}
                q = d.get('quaternion', {}) or {}
                lia_data.append({
                    'lia_x': lia.get('x', acc.get('ax', 0)),
                    'lia_y': lia.get('y', acc.get('ay', 0)),
                    'lia_z': lia.get('z', acc.get('az', 0)),
                    'qw': q.get('qw', 1),
                    'qx': q.get('qx', 0),
                    'qy': q.get('qy', 0),
                    'qz': q.get('qz', 0),
                    'entry_angle': d.get('entry_angle', 0),
                })

            db.save_ideal_stroke(uid, f'Stroke {stroke_num} from Session {session_id}', lia_data, len(lia_data))

            ideal_path = os.path.join(CACHE_DIR, 'ideal_stroke.json')
            with open(ideal_path, 'w') as f:
                json.dump({
                    'samples': lia_data,
                    'name': f'Stroke {stroke_num}',
                    'ideal_entry_angle': body.get('ideal_entry_angle', 30),
                }, f)

            self._send_json({'status': 'ok', 'samples': len(lia_data)})
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({'error': str(e)}, 500)

    def _get_ideal_stroke(self):
        u = self._require_user()
        if not u:
            return
        uid = u['id']
        ideal = db.get_latest_ideal_stroke(uid)
        if ideal:
            self._send_json({
                'samples': ideal['lia_data'],
                'name': ideal['name'],
                'num_samples': ideal['num_samples'],
                'created_at': ideal.get('created_at'),
                'id': ideal.get('id'),
            })
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
            u = self._require_user()
            if not u:
                return
            uid = u['id']
            ideal_path = os.path.join(CACHE_DIR, 'ideal_stroke.json')
            if os.path.exists(ideal_path):
                os.remove(ideal_path)
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
        last_err = None
        for attempt in range(2):
            try:
                # ESP32 can be busy right after AP start; retry once with a slightly longer read.
                resp = urllib.request.urlopen(f'{ESP32_URL}/api/device_info', timeout=4.0)
                data = json.loads(resp.read().decode())
                self._send_json(data)
                return
            except Exception as e:
                last_err = e
                if attempt == 0:
                    continue
        self._send_json({
            'status': 'disconnected',
            'message': 'Device not reachable',
            'detail': str(last_err) if last_err else None,
        })

    def _get_bootstrap(self):
        self._send_json({
            'instance_id': SERVER_INSTANCE_ID,
            'demo': DEMO_MODE,
        })

    # ── Session Persistence ──

    def _save_session(self):
        try:
            u = self._require_user()
            if not u:
                return
            body = self._read_body()
            uid = u['id']
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
                _sse_publish('progress', {'user_id': uid})
            _sse_publish('sessions', {'user_id': uid})
            self._send_json({'status': 'ok', 'session_id': sid})
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({'error': str(e)}, 500)

    def _get_sessions(self):
        u = self._require_user()
        if not u:
            return
        limit = 20
        sessions = db.get_sessions(u['id'], limit)
        self._send_json({'sessions': sessions})

    def _get_session_detail(self, session_id):
        u = self._require_user()
        if not u:
            return
        if not db.session_owned_by(session_id, u['id']):
            self._send_json({'error': 'Forbidden'}, 403)
            return
        s = db.get_session(session_id)
        if not s:
            self._send_json({'error': 'Session not found'}, 404)
            return
        self._send_json({'session': s})

    def _get_progress(self):
        u = self._require_user()
        if not u:
            return
        progress = db.get_progress(u['id'], 50)
        self._send_json({'progress': progress})

    # ── Multi-Device Merge ──

    def _merge_sessions(self):
        try:
            u = self._require_user()
            if not u:
                return
            body = self._read_body()
            session_ids = body.get('session_ids', [])
            if len(session_ids) < 2:
                self._send_json({"error": "Need at least 2 sessions to merge"}, 400)
                return
            for sid in session_ids:
                try:
                    sid_int = int(sid)
                except (TypeError, ValueError):
                    self._send_json({"error": "Invalid session id"}, 400)
                    return
                if not db.session_owned_by(sid_int, u['id']):
                    self._send_json({"error": "Forbidden"}, 403)
                    return

            sessions_to_merge = []
            for sid in session_ids:
                s = db.get_session(int(sid))
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
                processor = _replay_stroke_processor()
                p_data = []
                for sample in sess.get('data', []):
                    lia_x = sample.get('lia_x', sample.get('ax', 0))
                    lia_y = sample.get('lia_y', sample.get('ay', 0))
                    lia_z = sample.get('lia_z', sample.get('az', 0))
                    mx = float(sample.get('mx', 0) or 0)
                    my = float(sample.get('my', 0) or 0)
                    mz = float(sample.get('mz', 0) or 0)
                    data_dict = {
                        't': sample.get('t', sample.get('timestamp', 0)),
                        'lia_x': lia_x, 'lia_y': lia_y, 'lia_z': lia_z,
                        'mx': mx, 'my': my, 'mz': mz,
                        'gx': sample.get('gx', 0), 'gy': sample.get('gy', 0), 'gz': sample.get('gz', 0),
                        'qw': sample.get('qw', 1), 'qx': sample.get('qx', 0), 'qy': sample.get('qy', 0), 'qz': sample.get('qz', 0),
                        'haptic': sample.get('haptic', sample.get('haptic_fired', False)),
                        'deviation': sample.get('deviation', sample.get('deviation_score', 0.0)),
                        'strokes': sample.get('strokes', sample.get('stroke_count', 0)),
                        'entry_angle': float(sample.get('entry_angle', 0) or 0),
                        'dev_id': sample.get('dev_id', sample.get('device_id', 0)),
                        'dev_role': sample.get('dev_role', sample.get('device_role', 0)),
                        'cal': sample.get('cal', {
                            'sys': sample.get('cal_sys', 0), 'accel': sample.get('cal_accel', 0),
                            'gyro': sample.get('cal_gyro', 0), 'mag': sample.get('cal_mag', 0)
                        })
                    }
                    result = processor.process_data(data_dict)
                    if result:
                        p = json.loads(result)
                        p['magnetometer'] = {'mx': mx, 'my': my, 'mz': mz}
                        p_data.append(p)
                
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

    def _coaching_insights(self):
        u = self._require_user()
        if not u:
            return
        try:
            body = self._read_body()
        except Exception:
            self._send_json({'error': 'Invalid JSON body'}, 400)
            return
        if not isinstance(body, dict):
            body = {}
        try:
            from gemini_coaching import _refresh_gemini_key_from_dotenv
            _refresh_gemini_key_from_dotenv()
        except Exception:
            pass
        key = (os.environ.get('GEMINI_API_KEY') or '').strip()
        if not key:
            self._send_json({
                'status': 'unconfigured',
                'message': (
                    'AI coaching is not configured. Add GEMINI_API_KEY to dashboard/.env '
                    '(same folder as wifi_session_processor.py) and restart the server.'
                ),
            }, 200)
            return
        try:
            from gemini_coaching import coaching_rate_allow, generate_coaching_insights
        except Exception as e:
            self._send_json({'status': 'error', 'error': str(e)}, 500)
            return
        uid = u['id']
        allowed, rl_msg = coaching_rate_allow(uid)
        if not allowed:
            self._send_json({
                'status': 'error',
                'error': rl_msg or 'Rate limited',
                'rate_limited': True,
            }, 429)
            return
        privacy = self.headers.get('X-GoldenForm-Coaching-NoLog', '').strip().lower() in (
            '1', 'true', 'yes', 'on',
        )
        try:
            result = generate_coaching_insights(
                body,
                user_id=uid,
                privacy_no_log=privacy,
            )
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_json({'status': 'error', 'error': str(e)}, 500)
            return
        code = 200
        if result.get('status') == 'error':
            code = 502
        self._send_json(result, code)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, DELETE, PUT')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization, X-GoldenForm-Coaching-NoLog')
        self.end_headers()
    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        html_path = os.path.join(os.path.dirname(__file__), 'integrated_session_viewer.html')
        with open(html_path, 'rb') as f:
            self.wfile.write(f.read())

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

    def _read_body(self):
        length = int(self.headers.get('Content-Length', 0))
        return json.loads(self.rfile.read(length)) if length > 0 else {}

    def _send_json(self, data, status=200):
        body = json.dumps(data).encode()
        try:
            self.send_response(status)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            pass

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
                processor = _replay_stroke_processor()
                processed_data = []

                for sample in data:
                    lia_x = sample.get('lia_x', sample.get('ax', 0))
                    lia_y = sample.get('lia_y', sample.get('ay', 0))
                    lia_z = sample.get('lia_z', sample.get('az', 0))
                    dev_role = int(sample.get('dev_role', 0) or 0)
                    mx = float(sample.get('mx', 0) or 0)
                    my = float(sample.get('my', 0) or 0)
                    mz = float(sample.get('mz', 0) or 0)
                    data_dict = {
                        't': sample.get('t', 0),
                        'lia_x': lia_x, 'lia_y': lia_y, 'lia_z': lia_z,
                        'mx': mx, 'my': my, 'mz': mz,
                        'gx': sample.get('gx', 0),
                        'gy': sample.get('gy', 0),
                        'gz': sample.get('gz', 0),
                        'qw': sample.get('qw', 1),
                        'qx': sample.get('qx', 0),
                        'qy': sample.get('qy', 0),
                        'qz': sample.get('qz', 0),
                        'haptic_fired': sample.get('haptic', sample.get('haptic_fired', False)),
                        'deviation_score': sample.get('deviation', sample.get('deviation_score', 0.0)),
                        'strokes': sample.get('strokes', sample.get('stroke_count', 0)),
                        'entry_angle': float(sample.get('entry_angle', 0) or 0),
                        'dev_id': sample.get('dev_id', 0),
                        'dev_role': dev_role,
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
                        p.setdefault('breath_count', 0)
                        p['magnetometer'] = {'mx': mx, 'my': my, 'mz': mz}
                        processed_data.append(p)

                metrics = self._calculate_stroke_metrics(processed_data, processor)

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

        except urllib.error.HTTPError as e:
            allow = os.environ.get('GOLDENFORM_ALLOW_STALE_CACHE', '').strip().lower() in ('1', 'true', 'yes')
            hint = (
                f'HTTP {e.code} from {ESP32_URL}/data.json — if you are on the band Wi‑Fi: often there are '
                'no session files yet (record a swim first), or the swim was already synced and cleared from '
                'the SD card. Also check you joined the correct GoldenForm SSID (not a different AP), and '
                'flash current firmware (older builds required “sync transfer” mode for data.json).'
            )
            print(f'[ESP32] {hint} ({e})')
            if allow and os.path.exists(PROCESSED_CACHE):
                print('[ESP32] GOLDENFORM_ALLOW_STALE_CACHE set — serving last_processed.json (dev only).')
                with open(PROCESSED_CACHE, 'r') as f:
                    self._send_json(json.load(f))
                return
            self._send_json({
                'error': f'{hint} Join the correct GoldenForm Wi‑Fi on this computer, then tap Sync again.',
            }, 503)
        except urllib.error.URLError as e:
            allow = os.environ.get('GOLDENFORM_ALLOW_STALE_CACHE', '').strip().lower() in ('1', 'true', 'yes')
            print(f'[ESP32] Unreachable: {e}.')
            if allow and os.path.exists(PROCESSED_CACHE):
                print('[ESP32] GOLDENFORM_ALLOW_STALE_CACHE set — serving last_processed.json (dev only).')
                with open(PROCESSED_CACHE, 'r') as f:
                    self._send_json(json.load(f))
                return
            self._send_json({
                "error": f"ESP32 unreachable: {str(e)}. Join the device Wi‑Fi and try Sync again."
            }, 503)
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
                'stroke_breakdown': [],
                'calibration_snapshot': {},
            }

        stroke_times = []
        peak_accels = []
        stroke_breakdown = []

        use_fw = any((p.get('strokes') or 0) > 0 for p in processed_data)
        last_fw = 0
        last_py = 0
        stroke_boundary_indices = []
        for i, p in enumerate(processed_data):
            fw = int(p.get('strokes') or 0)
            py = int(p.get('stroke_count') or 0)
            t_ms = p.get('timestamp', 0)
            if use_fw:
                if fw > last_fw:
                    stroke_times.append(t_ms)
                    dbg = p.get('stroke_debug', {})
                    if dbg.get('accel_mag', 0) > 0:
                        peak_accels.append(dbg['accel_mag'])
                    stroke_boundary_indices.append((i, fw, t_ms, p))
                    last_fw = fw
            else:
                if py > last_py:
                    stroke_times.append(t_ms)
                    dbg = p.get('stroke_debug', {})
                    if dbg.get('accel_mag', 0) > 0:
                        peak_accels.append(dbg['accel_mag'])
                    stroke_boundary_indices.append((i, py, t_ms, p))
                    last_py = py

        for idx, sc, t_ms, p in stroke_boundary_indices:
            angle = float(p.get('entry_angle', 0) or 0)
            dev = float(p.get('deviation_score', 0) or 0)
            haptic = bool(p.get('haptic_fired', False))
            scan_limit = min(idx + 120, len(processed_data))
            for j in range(idx + 1, scan_limit):
                q = processed_data[j]
                if use_fw:
                    if int(q.get('strokes') or 0) > sc:
                        break
                else:
                    if int(q.get('stroke_count') or 0) > sc:
                        break
                qdev = float(q.get('deviation_score', 0) or 0)
                if qdev > 0 and dev == 0:
                    dev = qdev
                if q.get('haptic_fired', False):
                    haptic = True
                qa = float(q.get('entry_angle', 0) or 0)
                if qa > 0.05 and angle == 0:
                    angle = qa
            stroke_breakdown.append({
                'number': sc,
                'timestamp_s': t_ms / 1000.0,
                'entry_angle': angle,
                'haptic_fired': haptic,
                'deviation': dev
            })

        haptic_count = sum(1 for s in stroke_breakdown if s['haptic_fired'])
        deviation_scores = [s['deviation'] for s in stroke_breakdown if s['deviation'] > 0]

        duration = (processed_data[-1].get('timestamp', 0) -
                    processed_data[0].get('timestamp', 0)) / 1000.0

        accel_good = sum(1 for p in processed_data
                        if (p.get('calibration', {}).get('accel', 0) >= 2))
        gyro_good = sum(1 for p in processed_data
                        if (p.get('calibration', {}).get('gyro', 0) >= 2))
        n = len(processed_data)

        # Get entry angles from processor or from stroke breakdown
        avg_entry_angle = 0
        if hasattr(processor, 'entry_angles') and processor.entry_angles:
            avg_entry_angle = sum(processor.entry_angles) / len(processor.entry_angles)
        elif stroke_breakdown:
            angles = [s['entry_angle'] for s in stroke_breakdown if s['entry_angle'] > 0]
            if angles:
                avg_entry_angle = sum(angles) / len(angles)

        phase_pcts = {'glide': 0, 'catch': 0, 'pull': 0, 'recovery': 0}
        if hasattr(processor, 'last_phase_pcts'):
            phase_pcts = dict(processor.last_phase_pcts)

        avg_deviation = sum(deviation_scores) / len(deviation_scores) if deviation_scores else 0

        eff_strokes = len(stroke_breakdown) if stroke_breakdown else processor.stroke_count
        cal_snap = {}
        for p in reversed(processed_data):
            c = p.get('calibration') or p.get('cal')
            if isinstance(c, dict) and any(
                c.get(k) is not None for k in ('sys', 'gyro', 'accel', 'mag')
            ):
                cal_snap = {
                    'sys': int(c.get('sys', 0) or 0),
                    'gyro': int(c.get('gyro', 0) or 0),
                    'accel': int(c.get('accel', 0) or 0),
                    'mag': int(c.get('mag', 0) or 0),
                }
                break

        metrics = {
            'stroke_count': eff_strokes,
            'turn_count': processor.turn_count,
            'duration': duration,
            'stroke_rate': (eff_strokes / duration * 60) if duration > 0 else 0,
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
            'stroke_breakdown': stroke_breakdown,
            'calibration_snapshot': cal_snap,
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


class _QuietThreadingHTTPServer(ThreadingHTTPServer):
    """Suppress noisy ConnectionResetError tracebacks from client disconnects."""
    def handle_error(self, request, client_address):
        import traceback, sys as _sys
        exc = _sys.exc_info()[1]
        if isinstance(exc, (ConnectionResetError, BrokenPipeError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


def start_server(port=8004, demo=False, fresh=False):
    _prepare_server_state(demo, fresh)
    server = _QuietThreadingHTTPServer(('0.0.0.0', port), WiFiSessionHandler)
    print(f'GoldenForm Session Processor running on http://localhost:{port}')
    print(f'Open: http://localhost:{port}/')
    if demo:
        print('Demo mode: isolated DB (goldenform.demo.db), wiped each server start; file caches cleared.')
    elif fresh:
        print('Fresh production DB: goldenform.db was recreated (--fresh).')
    print('Press Ctrl+C to stop')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down...')
        server.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='GoldenForm dashboard + Wi‑Fi session processor')
    ap.add_argument('--demo', action='store_true', help='Use demo SQLite file (emptied each run) + clear session caches.')
    ap.add_argument('--fresh', action='store_true', help='Delete production goldenform.db before start (ignored with --demo).')
    ap.add_argument('--port', type=int, default=int(os.environ.get('PORT', 8004)), help='HTTP port (default 8004)')
    args = ap.parse_args()
    start_server(port=args.port, demo=args.demo, fresh=args.fresh)
