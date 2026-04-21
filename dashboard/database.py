#!/usr/bin/env python3
"""
GoldenForm Local Database — SQLite persistence with multi-user auth (sessions).
Production uses goldenform.db; demo uses goldenform.demo.db (wiped each demo server start).
"""
import sqlite3
import json
import os
import hashlib
import secrets
import time
from datetime import datetime

_CACHE_ROOT = os.path.join(os.path.dirname(__file__), '.session_cache')
# Production DB (persistent). Use --fresh on the server to delete and recreate.
PROD_DB_FILENAME = 'goldenform.db'
DEMO_DB_FILENAME = 'goldenform.demo.db'

DB_PATH = os.path.join(_CACHE_ROOT, PROD_DB_FILENAME)


def configure_database(demo: bool = False, fresh: bool = False) -> None:
    """
    Call once at server startup before any DB access.
    - Demo: always uses demo DB file and removes it first (registration portal each run).
    - Prod: uses goldenform.db; if fresh=True, deletes it first (clean install).
    """
    global DB_PATH
    os.makedirs(_CACHE_ROOT, exist_ok=True)
    if demo:
        DB_PATH = os.path.join(_CACHE_ROOT, DEMO_DB_FILENAME)
        _unlink_if_exists(DB_PATH)
    else:
        DB_PATH = os.path.join(_CACHE_ROOT, PROD_DB_FILENAME)
        if fresh:
            _unlink_if_exists(DB_PATH)
    init_db()


def _unlink_if_exists(path: str) -> None:
    try:
        if os.path.isfile(path):
            os.remove(path)
    except OSError:
        pass


def _connect():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


def init_db():
    """Create tables and apply lightweight migrations."""
    conn = _connect()
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            email TEXT UNIQUE,
            password_hash TEXT,
            name TEXT NOT NULL,
            height_cm REAL DEFAULT 0,
            wingspan_cm REAL DEFAULT 0,
            skill_level TEXT DEFAULT 'beginner',
            pool_length REAL DEFAULT 25.0,
            created_at TEXT DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS auth_sessions (
            token TEXT PRIMARY KEY,
            user_id INTEGER NOT NULL,
            created_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS devices (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id INTEGER,
            device_hw_id INTEGER NOT NULL,
            role TEXT NOT NULL DEFAULT 'wrist_right',
            name TEXT DEFAULT '',
            wifi_ssid TEXT DEFAULT '',
            registered_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS sessions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id INTEGER,
            device_ids TEXT DEFAULT '',
            raw_data TEXT,
            processed_data TEXT,
            metrics TEXT,
            duration REAL DEFAULT 0,
            synced_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL
        );

        CREATE TABLE IF NOT EXISTS ideal_strokes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id INTEGER,
            name TEXT DEFAULT 'Default',
            lia_data TEXT,
            num_samples INTEGER DEFAULT 0,
            created_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS progress (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id INTEGER,
            session_id INTEGER,
            stroke_rate REAL DEFAULT 0,
            consistency REAL DEFAULT 0,
            avg_deviation REAL DEFAULT 0,
            avg_entry_angle REAL DEFAULT 0,
            form_score REAL DEFAULT 0,
            stroke_count INTEGER DEFAULT 0,
            created_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
            FOREIGN KEY (session_id) REFERENCES sessions(id) ON DELETE CASCADE
        );
    """)
    conn.commit()
    conn.close()
    _migrate_users_profile_columns()
    _migrate_devices_wifi_ssid()


def _migrate_users_profile_columns():
    conn = _connect()
    cur = conn.execute("PRAGMA table_info(users)")
    cols = {row[1] for row in cur.fetchall()}
    if 'email' not in cols:
        try:
            conn.execute("ALTER TABLE users ADD COLUMN email TEXT")
        except sqlite3.OperationalError:
            pass
    if 'password_hash' not in cols:
        try:
            conn.execute("ALTER TABLE users ADD COLUMN password_hash TEXT")
        except sqlite3.OperationalError:
            pass
    conn.commit()
    conn.close()


def _migrate_devices_wifi_ssid():
    conn = _connect()
    try:
        conn.execute("ALTER TABLE devices ADD COLUMN wifi_ssid TEXT DEFAULT ''")
        conn.commit()
    except sqlite3.OperationalError:
        pass
    conn.close()


# ── Password hashing (stdlib only) ──

def hash_password(password: str) -> str:
    salt = secrets.token_hex(16)
    dk = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt.encode('ascii'), 120_000, dklen=32)
    return f"pbkdf2_sha256${salt}${dk.hex()}"


def verify_password(password: str, stored: str) -> bool:
    if not stored or '$' not in stored:
        return False
    try:
        _, salt, hexdigest = stored.split('$', 2)
        dk = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt.encode('ascii'), 120_000, dklen=32)
        return dk.hex() == hexdigest
    except Exception:
        return False


def _public_user(row) -> dict:
    if not row:
        return None
    d = dict(row)
    d.pop('password_hash', None)
    d.pop('pool_length', None)  # legacy column; not used by dashboard (no pool-distance metrics)
    return d


# ── Auth ──

def create_user(
    email: str,
    password_plain: str,
    name: str,
    height_cm: float = 0,
    wingspan_cm: float = 0,
    skill_level: str = 'beginner',
    pool_length: float = 25.0,
) -> int:
    email = (email or '').strip().lower()
    if not email or not password_plain:
        raise ValueError('email and password required')
    ph = hash_password(password_plain)
    conn = _connect()
    try:
        cur = conn.execute(
            """INSERT INTO users (email, password_hash, name, height_cm, wingspan_cm, skill_level, pool_length)
               VALUES (?,?,?,?,?,?,?)""",
            (email, ph, name or 'Swimmer', height_cm, wingspan_cm, skill_level, pool_length),
        )
        uid = cur.lastrowid
        conn.commit()
        return uid
    except sqlite3.IntegrityError:
        conn.close()
        raise ValueError('email already registered')
    finally:
        conn.close()


def verify_login(email: str, password_plain: str):
    email = (email or '').strip().lower()
    conn = _connect()
    row = conn.execute(
        "SELECT * FROM users WHERE email=? COLLATE NOCASE LIMIT 1", (email,)
    ).fetchone()
    conn.close()
    if not row:
        return None
    d = dict(row)
    if not verify_password(password_plain, d.get('password_hash') or ''):
        return None
    return _public_user(row)


def update_user_profile(
    user_id: int,
    email=None,
    name=None,
    height_cm=None,
    wingspan_cm=None,
    skill_level=None,
    pool_length=None,
):
    conn = _connect()
    row = conn.execute("SELECT * FROM users WHERE id=?", (user_id,)).fetchone()
    if not row:
        conn.close()
        raise ValueError('user not found')
    r = dict(row)
    em = (email.strip().lower() if isinstance(email, str) else None)
    if em == '':
        conn.close()
        raise ValueError('email cannot be empty')
    em_final = em if em is not None else r.get('email')
    n = name if name is not None else r['name']
    h = height_cm if height_cm is not None else r['height_cm']
    w = wingspan_cm if wingspan_cm is not None else r['wingspan_cm']
    sk = skill_level if skill_level is not None else r['skill_level']
    pl = pool_length if pool_length is not None else r['pool_length']
    try:
        conn.execute(
            "UPDATE users SET email=?, name=?, height_cm=?, wingspan_cm=?, skill_level=?, pool_length=? WHERE id=?",
            (em_final, n, h, w, sk, pl, user_id),
        )
        conn.commit()
    except sqlite3.IntegrityError:
        raise ValueError('email already registered')
    finally:
        conn.close()


def get_user_by_id(user_id: int):
    conn = _connect()
    row = conn.execute("SELECT * FROM users WHERE id=?", (user_id,)).fetchone()
    conn.close()
    return _public_user(row)


def create_session(user_id: int) -> str:
    token = secrets.token_urlsafe(48)
    conn = _connect()
    conn.execute("INSERT INTO auth_sessions (token, user_id) VALUES (?,?)", (token, user_id))
    conn.commit()
    conn.close()
    return token


def get_user_from_token(token: str):
    if not token:
        return None
    conn = _connect()
    row = conn.execute(
        """SELECT u.* FROM users u
           INNER JOIN auth_sessions s ON s.user_id = u.id
           WHERE s.token=? LIMIT 1""",
        (token,),
    ).fetchone()
    conn.close()
    return _public_user(row)


def revoke_auth_session(token: str):
    if not token:
        return
    conn = _connect()
    conn.execute("DELETE FROM auth_sessions WHERE token=?", (token,))
    conn.commit()
    conn.close()


def delete_all_sessions_for_user(user_id: int):
    conn = _connect()
    conn.execute("DELETE FROM auth_sessions WHERE user_id=?", (user_id,))
    conn.commit()
    conn.close()


# ── Devices ──

def register_device(user_id, device_hw_id, role, name='', wifi_ssid=''):
    conn = _connect()
    existing = conn.execute(
        "SELECT id FROM devices WHERE device_hw_id=?", (device_hw_id,)
    ).fetchone()
    if existing:
        conn.execute(
            "UPDATE devices SET user_id=?, role=?, name=?, wifi_ssid=? WHERE id=?",
            (user_id, role, name, wifi_ssid or '', existing['id'])
        )
        did = existing['id']
    else:
        cur = conn.execute(
            "INSERT INTO devices (user_id, device_hw_id, role, name, wifi_ssid) VALUES (?,?,?,?,?)",
            (user_id, device_hw_id, role, name, wifi_ssid or '')
        )
        did = cur.lastrowid
    conn.commit()
    conn.close()
    return did


def get_devices(user_id=None):
    conn = _connect()
    if user_id:
        rows = conn.execute("SELECT * FROM devices WHERE user_id=?", (user_id,)).fetchall()
    else:
        rows = conn.execute("SELECT * FROM devices").fetchall()
    conn.close()
    return [dict(r) for r in rows]


def delete_device(device_id):
    conn = _connect()
    conn.execute("DELETE FROM devices WHERE id=?", (device_id,))
    conn.commit()
    conn.close()


# ── Sessions (swim data) ──

def save_session(user_id, device_ids, processed_data, metrics, duration, raw_data=None):
    conn = _connect()
    cur = conn.execute(
        """INSERT INTO sessions (user_id, device_ids, raw_data, processed_data, metrics, duration)
           VALUES (?,?,?,?,?,?)""",
        (
            user_id,
            json.dumps(device_ids) if isinstance(device_ids, list) else str(device_ids),
            json.dumps(raw_data) if raw_data else None,
            json.dumps(processed_data),
            json.dumps(metrics),
            duration
        )
    )
    sid = cur.lastrowid
    conn.commit()
    conn.close()
    return sid


def get_sessions(user_id=None, limit=20):
    conn = _connect()
    if user_id:
        rows = conn.execute(
            "SELECT id, user_id, device_ids, metrics, duration, synced_at FROM sessions WHERE user_id=? ORDER BY id DESC LIMIT ?",
            (user_id, limit)
        ).fetchall()
    else:
        rows = conn.execute(
            "SELECT id, user_id, device_ids, metrics, duration, synced_at FROM sessions ORDER BY id DESC LIMIT ?",
            (limit,)
        ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


def get_session(session_id):
    conn = _connect()
    row = conn.execute("SELECT * FROM sessions WHERE id=?", (session_id,)).fetchone()
    conn.close()
    if not row:
        return None
    d = dict(row)
    d['processed_data'] = json.loads(d['processed_data']) if d['processed_data'] else []
    d['metrics'] = json.loads(d['metrics']) if d['metrics'] else {}
    d['raw_data'] = json.loads(d['raw_data']) if d['raw_data'] else None
    return d


def session_owned_by(session_id: int, user_id: int) -> bool:
    conn = _connect()
    row = conn.execute(
        "SELECT id FROM sessions WHERE id=? AND user_id=?", (session_id, user_id)
    ).fetchone()
    conn.close()
    return row is not None


def delete_swim_session(session_id):
    conn = _connect()
    conn.execute("DELETE FROM sessions WHERE id=?", (session_id,))
    conn.commit()
    conn.close()


# ── Ideal strokes ──

def save_ideal_stroke(user_id, name, lia_data, num_samples):
    conn = _connect()
    cur = conn.execute(
        "INSERT INTO ideal_strokes (user_id, name, lia_data, num_samples) VALUES (?,?,?,?)",
        (user_id, name, json.dumps(lia_data), num_samples)
    )
    iid = cur.lastrowid
    conn.commit()
    conn.close()
    return iid


def get_ideal_strokes(user_id=None):
    conn = _connect()
    if user_id:
        rows = conn.execute("SELECT * FROM ideal_strokes WHERE user_id=? ORDER BY id DESC", (user_id,)).fetchall()
    else:
        rows = conn.execute("SELECT * FROM ideal_strokes ORDER BY id DESC").fetchall()
    conn.close()
    results = []
    for r in rows:
        d = dict(r)
        d['lia_data'] = json.loads(d['lia_data']) if d['lia_data'] else []
        results.append(d)
    return results


def get_latest_ideal_stroke(user_id=None):
    conn = _connect()
    if user_id:
        row = conn.execute("SELECT * FROM ideal_strokes WHERE user_id=? ORDER BY id DESC LIMIT 1", (user_id,)).fetchone()
    else:
        row = conn.execute("SELECT * FROM ideal_strokes ORDER BY id DESC LIMIT 1").fetchone()
    conn.close()
    if not row:
        return None
    d = dict(row)
    d['lia_data'] = json.loads(d['lia_data']) if d['lia_data'] else []
    return d


# ── Progress ──

def save_progress(user_id, session_id, stroke_rate, consistency, avg_deviation, avg_entry_angle, form_score, stroke_count):
    conn = _connect()
    conn.execute(
        """INSERT INTO progress (user_id, session_id, stroke_rate, consistency, avg_deviation, avg_entry_angle, form_score, stroke_count)
           VALUES (?,?,?,?,?,?,?,?)""",
        (user_id, session_id, stroke_rate, consistency, avg_deviation, avg_entry_angle, form_score, stroke_count)
    )
    conn.commit()
    conn.close()


def get_progress(user_id, limit=50):
    conn = _connect()
    rows = conn.execute(
        "SELECT * FROM progress WHERE user_id=? ORDER BY id DESC LIMIT ?",
        (user_id, limit)
    ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


def wipe_database_file():
    """Remove current DB_PATH and recreate schema (dev helper)."""
    _unlink_if_exists(DB_PATH)
    init_db()
