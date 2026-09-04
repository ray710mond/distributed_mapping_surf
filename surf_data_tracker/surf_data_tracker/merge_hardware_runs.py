"""Join independently recorded sender and receiver telemetry after a hardware run."""

import argparse
import csv
import json
import math
import sqlite3
from pathlib import Path


def database_path(value):
    path = Path(value).expanduser().resolve()
    return path / 'telemetry.sqlite3' if path.is_dir() else path


def metadata(connection):
    connection.row_factory = sqlite3.Row
    row = connection.execute('SELECT * FROM run_metadata LIMIT 1').fetchone()
    return dict(row) if row else {}


def finite(value):
    return value is not None and math.isfinite(value)


def main():
    parser = argparse.ArgumentParser(
        description='Offline join of drone and humanoid local telemetry databases.')
    parser.add_argument('sender', help='drone run directory or telemetry.sqlite3')
    parser.add_argument('receiver', help='humanoid run directory or telemetry.sqlite3')
    parser.add_argument(
        '--output', type=Path, default=Path('merged_hardware_telemetry'),
        help='output directory (default: ./merged_hardware_telemetry)')
    args = parser.parse_args()

    sender_path = database_path(args.sender)
    receiver_path = database_path(args.receiver)
    for path in (sender_path, receiver_path):
        if not path.is_file():
            parser.error(f'telemetry database not found: {path}')

    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    with sqlite3.connect(sender_path) as sender, sqlite3.connect(receiver_path) as receiver:
        sender.row_factory = sqlite3.Row
        receiver.row_factory = sqlite3.Row
        sender_meta = metadata(sender)
        receiver_meta = metadata(receiver)
        pipeline = {
            (row['source_id'], row['map_epoch'], row['version'], row['traffic_class']): row
            for row in sender.execute('SELECT * FROM pipeline')
        }
        deliveries = list(receiver.execute('SELECT * FROM delivery ORDER BY time_ns'))

    rows = []
    for delivery in deliveries:
        key = (
            delivery['source_id'], delivery['map_epoch'],
            delivery['version'], delivery['traffic_class'])
        sender_row = pipeline.get(key)
        rows.append({
            'source_id': delivery['source_id'],
            'map_epoch': delivery['map_epoch'],
            'version': delivery['version'],
            'traffic_class': delivery['traffic_class'],
            'matched_sender_record': sender_row is not None,
            'accepted': bool(delivery['accepted']),
            'wire_bytes': delivery['wire_bytes'],
            'voxel_count': delivery['voxel_count'],
            'sender_processing_ms': (
                sender_row['processing_ms'] if sender_row is not None else None),
            'compression_ms': (
                sender_row['compression_ms'] if sender_row is not None else None),
            'network_sender_to_receiver_ms': delivery['sender_to_receiver_ms'],
            'sensor_to_receiver_ms': delivery['sensor_to_receiver_ms'],
            'decode_ms': delivery['decode_ms'],
            'sensor_to_decoded_ms': delivery['end_to_end_ms'],
            'rejection_reason': delivery['rejection_reason'],
        })

    fieldnames = list(rows[0]) if rows else [
        'source_id', 'map_epoch', 'version', 'traffic_class',
        'matched_sender_record', 'accepted', 'wire_bytes', 'voxel_count',
        'sender_processing_ms', 'compression_ms',
        'network_sender_to_receiver_ms', 'sensor_to_receiver_ms',
        'decode_ms', 'sensor_to_decoded_ms', 'rejection_reason']
    with (output / 'joined_packets.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    sender_uncertainty = sender_meta.get('clock_uncertainty_ms')
    receiver_uncertainty = receiver_meta.get('clock_uncertainty_ms')
    combined_uncertainty = (
        sender_uncertainty + receiver_uncertainty
        if finite(sender_uncertainty) and finite(receiver_uncertainty) else None)
    matched = sum(row['matched_sender_record'] for row in rows)
    report = {
        'sender_database': str(sender_path),
        'receiver_database': str(receiver_path),
        'sender_metadata': sender_meta,
        'receiver_metadata': receiver_meta,
        'packets': {
            'delivery_records': len(rows),
            'matched_sender_records': matched,
            'unmatched_delivery_records': len(rows) - matched,
        },
        'clock': {
            'combined_declared_uncertainty_ms': combined_uncertainty,
            'latency_valid': (
                sender_meta.get('clock_sync_method') not in (None, '', 'unverified')
                and receiver_meta.get('clock_sync_method') not in (None, '', 'unverified')
                and combined_uncertainty is not None),
            'note': (
                'Cross-host latency is only defensible when both run metadata records '
                'contain a verified synchronization method and measured uncertainty.'),
        },
    }
    (output / 'merge_summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
