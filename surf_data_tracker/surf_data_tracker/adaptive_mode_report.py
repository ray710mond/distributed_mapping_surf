"""Calculate time spent in each adaptive-control mode from tracker telemetry."""

import argparse
import csv
import json
import sqlite3
from pathlib import Path


MODE_NAMES = {
    0: 'FULL',
    1: 'VOXEL_DELTAS',
    2: 'DYNAMIC_ONLY',
    3: 'METADATA_ONLY',
}


def calculate_mode_residency(connection):
    """Return time-weighted mode residency over the observed sensor-time span.

    Each interval between consecutive realtime pipeline samples is assigned to
    the mode reported by the earlier sample. This uses message timestamps, not
    callback arrival time, so transport delays do not count toward a mode.
    """
    samples = connection.execute(
        '''SELECT time_ns, operating_mode FROM pipeline
           WHERE traffic_class = 1 ORDER BY time_ns, rowid'''
    ).fetchall()
    durations_ns = {mode: 0 for mode in MODE_NAMES}
    sample_counts = {mode: 0 for mode in MODE_NAMES}
    unknown_durations_ns = {}
    unknown_sample_counts = {}

    for _, mode in samples:
        target = sample_counts if mode in MODE_NAMES else unknown_sample_counts
        target[mode] = target.get(mode, 0) + 1

    intervals = 0
    for (start_ns, mode), (end_ns, _) in zip(samples, samples[1:]):
        duration_ns = end_ns - start_ns
        if duration_ns <= 0:
            continue
        target = durations_ns if mode in MODE_NAMES else unknown_durations_ns
        target[mode] = target.get(mode, 0) + duration_ns
        intervals += 1

    total_ns = sum(durations_ns.values()) + sum(unknown_durations_ns.values())

    def row(mode, name, duration_ns, count):
        return {
            'mode': mode,
            'name': name,
            'duration_seconds': duration_ns / 1_000_000_000,
            'percentage': 100.0 * duration_ns / total_ns if total_ns else 0.0,
            'samples': count,
        }

    modes = [
        row(mode, name, durations_ns[mode], sample_counts[mode])
        for mode, name in MODE_NAMES.items()
    ]
    modes.extend(
        row(mode, f'UNKNOWN_{mode}', duration_ns, unknown_sample_counts.get(mode, 0))
        for mode, duration_ns in sorted(unknown_durations_ns.items())
    )
    return {
        'method': 'time-weighted consecutive realtime PipelineMetrics timestamps',
        'traffic_class': 1,
        'observed_duration_seconds': total_ns / 1_000_000_000,
        'intervals': intervals,
        'samples': len(samples),
        'first_sample_time_ns': samples[0][0] if samples else None,
        'last_sample_time_ns': samples[-1][0] if samples else None,
        'modes': modes,
        'notes': [
            'An interval is credited to the mode in effect at its first sample.',
            'The span before the first and after the last metric is unobserved.',
            'Sensor timestamps prevent transport delays from being counted.',
        ],
    }


def write_mode_reports(connection, json_path, csv_path):
    report = calculate_mode_residency(connection)
    Path(json_path).write_text(json.dumps(report, indent=2) + '\n')
    with Path(csv_path).open('w', newline='') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=['mode', 'name', 'duration_seconds', 'percentage', 'samples'],
        )
        writer.writeheader()
        writer.writerows(report['modes'])
    return report


def main():
    parser = argparse.ArgumentParser(
        description='Generate adaptive-mode residency reports from a tracker run.')
    parser.add_argument(
        'input',
        type=Path,
        help='telemetry.sqlite3 or the run directory containing it',
    )
    args = parser.parse_args()
    input_path = args.input.expanduser().resolve()
    database_path = (
        input_path / 'telemetry.sqlite3' if input_path.is_dir() else input_path
    )
    if not database_path.is_file():
        parser.error(f'telemetry database not found: {database_path}')
    run_directory = database_path.parent
    with sqlite3.connect(database_path) as connection:
        report = write_mode_reports(
            connection,
            run_directory / 'adaptive_modes.json',
            run_directory / 'adaptive_modes.csv',
        )
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
