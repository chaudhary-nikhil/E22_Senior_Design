#!/usr/bin/env python3
"""
GoldenForm Local Database — SQLite persistence for users, devices, sessions, and progress.
"""
import sqlite3
import json
import os
import time
from datetime import datetime

DB_PATH = os.path.join(os.path.dirname(__file__), '.session_cache', 'goldenform.db')


def _connect():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn
get_db = _connect


def init_db():
    """Create tables if they don't exist."""
    conn = _connect()
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            height_cm REAL DEFAULT 0,
            wingspan_cm REAL DEFAULT 0,
            skill_level TEXT DEFAULT 'beginner',
            pool_length REAL DEFAULT 25.0,
            created_at TEXT DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS devices (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id INTEGER,
            device_hw_id INTEGER NOT NULL,
            role TEXT NOT NULL DEFAULT 'wrist_right',
            name TEXT DEFAULT '',
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


# ── User CRUD ──

def upsert_user(name, height_cm=0, wingspan_cm=0, skill_level='beginner', pool_length=25.0):
    conn = _connect()
    cur = conn.execute("SELECT id FROM users LIMIT 1")
    row = cur.fetchone()
    if row:
        try:
            conn.execute(
                "UPDATE users SET name=?, height_cm=?, wingspan_cm=?, skill_level=?, pool_length=? WHERE id=?",
                (name, height_cm, wingspan_cm, skill_level, pool_length, row['id'])
            )
        except sqlite3.OperationalError:
            conn.execute("ALTER TABLE users ADD COLUMN pool_length REAL DEFAULT 25.0")
            conn.execute(
                "UPDATE users SET name=?, height_cm=?, wingspan_cm=?, skill_level=?, pool_length=? WHERE id=?",
                (name, height_cm, wingspan_cm, skill_level, pool_length, row['id'])
            )
        uid = row['id']
    else:
        try:
            cur = conn.execute(
                "INSERT INTO users (name, height_cm, wingspan_cm, skill_level, pool_length) VALUES (?,?,?,?,?)",
                (name, height_cm, wingspan_cm, skill_level, pool_length)
            )
            uid = cur.lastrowid
        except sqlite3.OperationalError:
            # Table might not have pool_length, attempt to migrate
            conn.execute("ALTER TABLE users ADD COLUMN pool_length REAL DEFAULT 25.0")
            cur = conn.execute(
                "INSERT INTO users (name, height_cm, wingspan_cm, skill_level, pool_length) VALUES (?,?,?,?,?)",
                (name, height_cm, wingspan_cm, skill_level, pool_length)
            )
            uid = cur.lastrowid
    conn.commit()
    conn.close()
    return uid


def get_user():
    conn = _connect()
    row = conn.execute("SELECT * FROM users LIMIT 1").fetchone()
    conn.close()
    return dict(row) if row else None


# ── Device CRUD ──

def register_device(user_id, device_hw_id, role, name=''):
    conn = _connect()
    existing = conn.execute(
        "SELECT id FROM devices WHERE device_hw_id=?", (device_hw_id,)
    ).fetchone()
    if existing:
        conn.execute(
            "UPDATE devices SET user_id=?, role=?, name=? WHERE id=?",
            (user_id, role, name, existing['id'])
        )
        did = existing['id']
    else:
        cur = conn.execute(
            "INSERT INTO devices (user_id, device_hw_id, role, name) VALUES (?,?,?,?)",
            (user_id, device_hw_id, role, name)
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


# ── Session CRUD ──

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
    results = []
    for r in rows:
        d = dict(r)
        d['metrics'] = json.loads(d['metrics']) if d['metrics'] else {}
        return_val = d
        results.append(d)
    return results


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


def delete_session(session_id):
    conn = _connect()
    conn.execute("DELETE FROM sessions WHERE id=?", (session_id,))
    conn.commit()
    conn.close()


# ── Ideal Strokes ──

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


# ── Progress Tracking ──

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


# Initialize on import
init_db()
