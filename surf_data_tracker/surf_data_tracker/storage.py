"""Crash-resilient local experiment storage."""

import json
import math
import re
import socket
import sqlite3
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


SCHEMA_VERSION = 2


def validate_run_id(run_id):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', run_id or ''):
        raise ValueError(
            'run_id must start with a letter or number and contain only '
            'letters, numbers, underscores, periods, and hyphens')
    return run_id


class EventStore:
    def __init__(self, directory, run_id, host_role, metadata=None, raw_events=True,
                 overwrite=True):
        validate_run_id(run_id)
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / 'telemetry.sqlite3'
        if overwrite:
            # A run ID identifies one invocation per host. Remove SQLite's main
            # file and sidecars together so restarting an ID cannot retain old
            # events or replay an old WAL into the new database.
            for name in ('telemetry.sqlite3', 'telemetry.sqlite3-wal',
                         'telemetry.sqlite3-shm', 'telemetry.sqlite3-journal',
                         'metadata.json', 'metadata.json.tmp'):
                try:
                    (self.directory / name).unlink()
                except FileNotFoundError:
                    pass
        self.lock = threading.RLock()
        self.accepting = True
        self.raw_events = raw_events
        self.connection = sqlite3.connect(self.path, check_same_thread=False)
        self.connection.execute('PRAGMA journal_mode=WAL')
        self.connection.execute('PRAGMA synchronous=NORMAL')
        self.connection.executescript('''
          CREATE TABLE IF NOT EXISTS events (
            event_id TEXT PRIMARY KEY, wall_time_ns INTEGER NOT NULL,
            monotonic_ns INTEGER NOT NULL, host TEXT NOT NULL,
            category TEXT NOT NULL, pipeline TEXT NOT NULL,
            stage TEXT NOT NULL, payload_json TEXT NOT NULL);
          CREATE INDEX IF NOT EXISTS event_time ON events(wall_time_ns);
          CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value_json TEXT NOT NULL);
        ''')
        base = {
            'schema_version': SCHEMA_VERSION, 'run_id': run_id,
            'hostname': socket.gethostname(), 'host_role': host_role,
            'started_at_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'running', 'raw_event_persistence': bool(raw_events),
        }
        base.update(metadata or {})
        self.metadata = base
        self._write_metadata()
        self.connection.commit()

    def _write_metadata(self):
        self.connection.executemany(
            'INSERT OR REPLACE INTO metadata VALUES (?,?)',
            [(key, json.dumps(value, allow_nan=False)) for key, value in self.metadata.items()])

    def record(self, category, pipeline, stage, payload, *, wall_time_ns=None,
               monotonic_ns=None, event_id=None):
        if not self.accepting:
            return False
        wall_time_ns = wall_time_ns or time.time_ns()
        monotonic_ns = monotonic_ns or time.monotonic_ns()
        event_id = event_id or f'{self.metadata["hostname"]}:{monotonic_ns}:{category}:{stage}'
        clean = {
            key: value for key, value in payload.items()
            if value is not None and not (
                isinstance(value, float) and not math.isfinite(value))}
        with self.lock:
            self.connection.execute(
                'INSERT OR IGNORE INTO events VALUES (?,?,?,?,?,?,?,?)',
                (event_id, wall_time_ns, monotonic_ns, self.metadata['hostname'],
                 category, pipeline, stage, json.dumps(clean, allow_nan=False)))
        return True

    def flush(self):
        with self.lock:
            self.connection.commit()
            self._write_local_metadata()

    def _write_local_metadata(self):
        temporary = self.directory / 'metadata.json.tmp'
        temporary.write_text(json.dumps(self.metadata, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.directory / 'metadata.json')

    def close(self, status='completed'):
        if not self.connection:
            return
        self.accepting = False
        with self.lock:
            self.metadata['status'] = status
            self.metadata['ended_at_utc'] = datetime.now(timezone.utc).isoformat()
            self._write_metadata()
            self.connection.commit()
            self._write_local_metadata()
            self.connection.close()
            self.connection = None
