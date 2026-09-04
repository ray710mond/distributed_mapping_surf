#!/usr/bin/env python3
"""Conservative Chrony verification for cross-host timestamp metadata."""

import csv
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_STATUS_PATH = Path('/run/surf-clock-sync/status.json')
MAX_STATUS_AGE_SECONDS = 300.0


def _validated_status(document, now_ns=None):
    try:
        if not document.get('verified'):
            return None
        measured_ns = int(document['measured_at_unix_ns'])
        offset_ms = float(document['offset_ms'])
        uncertainty_ms = float(document['uncertainty_ms'])
        method = str(document['method']).strip()
    except (KeyError, TypeError, ValueError):
        return None
    now_ns = time.time_ns() if now_ns is None else now_ns
    age_seconds = (now_ns - measured_ns) / 1e9
    if (not method or method == 'unverified' or not math.isfinite(offset_ms)
            or not math.isfinite(uncertainty_ms) or uncertainty_ms < 0
            or age_seconds < -5.0 or age_seconds > MAX_STATUS_AGE_SECONDS):
        return None
    return {
        'clock_sync_method': method,
        'clock_offset_ms': offset_ms,
        'clock_uncertainty_ms': uncertainty_ms,
        'clock_verification_time_utc': document.get('measured_at_utc', ''),
        'clock_reference': document.get('source', ''),
    }


def _chrony_csv_status(text, now_ns=None):
    try:
        row = next(csv.reader([text.strip()]))
        # chronyc -c tracking: ref, stratum, ref-time, system-time,
        # last-offset, RMS-offset, frequency, residual, skew, root-delay,
        # root-dispersion, update-interval, leap-status.
        if len(row) < 13 or row[-1].strip().lower() != 'normal':
            return None
        system_time = float(row[-10])
        rms_offset = float(row[-8])
        root_delay = float(row[-4])
        root_dispersion = float(row[-3])
    except (StopIteration, TypeError, ValueError):
        return None
    uncertainty_ms = (abs(system_time) + abs(rms_offset)
                      + 0.5 * abs(root_delay)
                      + abs(root_dispersion)) * 1000.0
    document = {
        'verified': True,
        'method': 'chrony-local',
        'offset_ms': system_time * 1000.0,
        'uncertainty_ms': uncertainty_ms,
        'measured_at_unix_ns': time.time_ns() if now_ns is None else now_ns,
        'measured_at_utc': datetime.now(timezone.utc).isoformat(),
        'source': (row[1] if len(row) > 13 else row[0]).strip(),
    }
    return _validated_status(document, now_ns=document['measured_at_unix_ns'])


def detect_clock_sync(status_path=DEFAULT_STATUS_PATH, now_ns=None):
    """Return verified metadata from the host export or local Chrony."""
    try:
        document = json.loads(Path(status_path).read_text())
    except (OSError, json.JSONDecodeError):
        document = None
    if document is not None:
        status = _validated_status(document, now_ns=now_ns)
        if status:
            return status
    try:
        result = subprocess.run(
            ['chronyc', '-c', 'tracking'], capture_output=True, text=True,
            timeout=1.0, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return _chrony_csv_status(result.stdout, now_ns=now_ns)


def main():
    status = detect_clock_sync()
    if not status:
        print(json.dumps({
            'verified': False,
            'reason': 'no fresh exported proof and local Chrony is not verified',
        }))
        raise SystemExit(1)
    print(json.dumps({
        'verified': True,
        'method': status['clock_sync_method'],
        'offset_ms': status['clock_offset_ms'],
        'uncertainty_ms': status['clock_uncertainty_ms'],
        'source': status.get('clock_reference', ''),
        'measured_at_utc': status.get('clock_verification_time_utc', ''),
    }, sort_keys=True))


if __name__ == '__main__':
    main()
