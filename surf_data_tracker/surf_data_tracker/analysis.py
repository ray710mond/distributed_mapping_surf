"""Merge host telemetry into compact experimental results."""

import argparse
import csv
import json
import math
import sqlite3
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from .metrics import describe, distance, nearest_pose, safe_reduction


STAT_FIELDS = ('sample_count', 'mean', 'median', 'min', 'max', 'stddev',
               'p5', 'p50', 'p95', 'p99')

NETWORK_COUNTERS = {
    'rx_bytes', 'tx_bytes', 'rx_packets', 'tx_packets',
    'rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped',
    'tx_retries', 'tx_failed', 'station_tx_packets', 'station_rx_packets',
    'morse_mmrc_total_success', 'morse_mmrc_total_attempts',
    'morse_mmrc_mpdu_success', 'morse_mmrc_mpdu_failures',
    'morse_page_command_tx', 'morse_page_beacon_tx', 'morse_page_management_tx',
    'morse_page_data_tx', 'morse_page_page_write_fail', 'morse_page_no_page',
    'morse_page_queue_stop', 'morse_page_tx_aged_out',
    'morse_page_tx_dropped_due_to_duty_cycle', 'morse_page_tx_status_dropped',
    'morse_page_rx_empty_queue', 'morse_page_rx_packet_split_across_window',
    'morse_page_rx_invalid_byte_count', 'morse_page_invalid_checksum',
    'morse_page_invalid_tx_status_checksum',
}


def load_events(run_directory):
    events, hosts = [], []
    for path in sorted(Path(run_directory).rglob('telemetry.sqlite3')):
        with sqlite3.connect(path) as connection:
            tables = {row[0] for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'")}
            if 'events' not in tables:
                continue
            metadata = {key: json.loads(value) for key, value in connection.execute(
                'SELECT key, value_json FROM metadata')}
            metadata['database'] = str(path)
            hosts.append(metadata)
            for row in connection.execute(
                    'SELECT event_id,wall_time_ns,monotonic_ns,host,category,pipeline,stage,payload_json FROM events'):
                event = dict(zip(('event_id', 'wall_time_ns', 'monotonic_ns', 'host',
                                  'category', 'pipeline', 'stage', 'payload_json'), row))
                event['payload'] = json.loads(event.pop('payload_json'))
                events.append(event)
    # Cross-host latency is defensible only when every participating host has
    # independently recorded clock verification for this invocation.
    clocks_valid = bool(hosts) and all(
        metadata.get('clock_sync_method') not in (None, '', 'unverified')
        and metadata.get('clock_uncertainty_ms') is not None
        for metadata in hosts)
    combined_clock_uncertainty_ms = (
        sum(float(metadata['clock_uncertainty_ms']) for metadata in hosts)
        if clocks_valid else None)
    for event in events:
        event['clock_valid'] = clocks_valid
        event['cross_host_clock_uncertainty_ms'] = combined_clock_uncertainty_ms
    # Collection can be retried and copies can overlap. Event IDs are stable.
    deduplicated = {event['event_id']: event for event in events}
    return sorted(deduplicated.values(), key=lambda event: event['wall_time_ns']), hosts


def _add_metric(groups, category, pipeline, stage, metric, unit, value,
                measurement='measured', note=''):
    if value is None or isinstance(value, bool):
        return
    try:
        value = float(value)
    except (TypeError, ValueError):
        return
    if not math.isfinite(value):
        return
    groups[(category, pipeline, stage, metric, unit, measurement, note)].append(value)


def derived_measurements(event):
    """Yield normalized measurement rows without conflating unlike units."""
    p = event['payload']
    category, pipeline, stage = event['category'], event['pipeline'], event['stage']
    if category == 'delivery' and p.get('applied_temporal_regressions', 0) != 0:
        raise ValueError('Correctness failure: applied temporal regressions must be zero')

    if category in ('allocation', 'capacity'):
        for metric, value in p.items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                continue
            unit = 'count' if isinstance(value, int) else 'dimensionless'
            if metric.startswith(('debt_', 'previous_debt_', 'generated_debt_',
                                  'acknowledged_debt_', 'superseded_debt_', 'reclassified_debt_',
                                  'disturbance_', 'timed_out_debt_', 'excluded_debt_',
                                  'sent_priority_', 'mean_priority_', 'max_priority_')):
                unit = 'priority'
            elif metric.startswith(('requested_rate_', 'allocated_rate_', 'offered_cdr_rate_')) or metric in (
                    'usable_capacity', 'usable_capacity_bytes_per_second'):
                unit = 'bytes/s'
            elif metric.startswith(('target_bytes_', 'wire_bytes_', 'compressed_bytes_', 'uncompressed_bytes_')):
                unit = 'bytes'
            elif metric.endswith('_mbps'):
                unit = 'Mbps'
            elif metric.endswith('_dbm'):
                unit = 'dBm'
            elif metric.endswith(('_dt', '_age', '_s')) or metric == 'matrix_update_time':
                unit = 's'
            elif metric.endswith('_ms'):
                unit = 'ms'
            elif metric.startswith('b_'):
                unit = 'priority/(bytes/s)'
            elif metric.startswith('k_'):
                unit = '(bytes/s)/priority'
            elif metric.startswith('q_'):
                unit = 'cost/priority^2'
            elif metric.startswith('r_'):
                unit = 'cost/(bytes/s)^2'
            yield category, pipeline, stage, metric, unit, value, 'measured', ''
        return
    if category == 'pipeline':
        traffic = p.get('traffic_class_name', stage)
        yield category, pipeline, f'{traffic}:conceptual_data_buffer', 'output_bytes', 'bytes', p.get('raw_data_bytes'), 'measured', 'PointCloud2 data buffer; excludes ROS metadata and padding outside data[]'
        byte_stages = [
            ('raw_ros_message', p.get('raw_serialized_bytes')),
            ('sparse_representation', p.get('uncompressed_bytes')),
            ('compression', p.get('payload_bytes')),
            ('ros_transport_message', p.get('wire_bytes')),
        ]
        raw = byte_stages[0][1]
        previous = None
        for name, output in byte_stages:
            if output is None:
                continue
            yield category, pipeline, f'{traffic}:{name}', 'output_bytes', 'bytes', output, 'measured', ''
            if previous is not None:
                saved, percent, ratio = safe_reduction(previous, output)
                for metric, value, unit in (
                        ('stage_bytes_saved', saved, 'bytes'),
                        ('stage_saved_pct', percent, 'pct'), ('stage_ratio', ratio, 'ratio')):
                    yield category, pipeline, f'{traffic}:{name}', metric, unit, value, 'calculated', 'adjacent measured byte sizes'
            if raw is not None:
                saved, percent, ratio = safe_reduction(raw, output)
                for metric, value, unit in (
                        ('cumulative_bytes_saved', saved, 'bytes'),
                        ('cumulative_saved_pct', percent, 'pct'),
                        ('cumulative_ratio', ratio, 'ratio')):
                    yield category, pipeline, f'{traffic}:{name}', metric, unit, value, 'calculated', 'relative to raw ROS serialization'
            previous = output
            if traffic == 'realtime' and p.get('input_rate_hz') is not None:
                yield category, pipeline, f'{traffic}:{name}', 'counterfactual_demand_mbps', 'Mbps', output * p['input_rate_hz'] * 8.0 / 1e6, 'calculated', 'measured bytes per update multiplied by measured input frequency'
        element_stages = [
            ('raw_points', p.get('raw_points')), ('valid_points', p.get('valid_points')),
            ('unique_voxels', p.get('unique_voxels')), ('selected_voxels', p.get('selected_voxels'))]
        previous = None
        for name, output in element_stages:
            yield category, pipeline, f'{traffic}:{name}', 'output_elements', 'elements', output, 'measured', 'point and voxel units change at voxelization'
            if previous is not None:
                _, percent, ratio = safe_reduction(previous, output)
                yield category, pipeline, f'{traffic}:{name}', 'stage_element_saved_pct', 'pct', percent, 'calculated', 'element-count attribution; not a byte saving'
                yield category, pipeline, f'{traffic}:{name}', 'stage_element_ratio', 'ratio', ratio, 'calculated', 'element-count attribution; not a byte saving'
            previous = output
        timing = {
            'transform_lookup': 'transform_lookup_ms',
            'point_filter_transform_voxelize': 'point_preprocessing_ms',
            'occupancy_selection': 'occupancy_selection_ms', 'clearing': 'clearing_ms',
            'compression': 'compression_ms', 'sender_total': 'processing_ms',
            'queue_wait': 'queue_wait_ms', 'raw_ros_serialization': 'raw_serialization_ms',
            'transport_ros_serialization': 'wire_serialization_ms'}
        for name, field in timing.items():
            yield 'compute', pipeline, f'{traffic}:{name}', 'compute_ms', 'ms', p.get(field), 'measured', 'same-host monotonic duration'
        yield category, pipeline, f'{traffic}:packetization', 'packets_per_update', 'packets', p.get('packet_count'), 'measured', 'number of CDR-serialized transport chunks published for one logical update'
        yield category, pipeline, f'{traffic}:packetization', 'packet_budget_bytes', 'bytes', p.get('packet_budget_bytes'), 'configured', 'maximum CDR-serialized bytes per transport chunk'
        yield category, pipeline, f'{traffic}:packetization', 'update_budget_bytes', 'bytes', p.get('update_budget_bytes'), 'configured', 'maximum aggregate CDR-serialized bytes selected per logical update'
        utilization = (100.0 * p['wire_bytes'] / p['update_budget_bytes'] if
                       p.get('wire_bytes') is not None and p.get('update_budget_bytes') else None)
        yield category, pipeline, f'{traffic}:packetization', 'update_budget_utilization_pct', 'pct', utilization, 'calculated', 'aggregate CDR bytes divided by configured update budget'
    elif category == 'delivery':
        for name in ('wire_bytes', 'delivered_payload_bytes', 'voxel_count',
                     'chunk_count', 'received_chunks', 'missing_chunks', 'chunk_loss_pct',
                     'sequence_gap_updates', 'update_loss_pct', 'update_complete',
                     'jitter_ms', 'decode_ms', 'stale_voxels_rejected',
                     'attempted_temporal_regressions', 'applied_temporal_regressions',
                     'codec_reconstruction_ms', 'receiver_processing_ms'):
            unit = ('bytes' if name.endswith('_bytes') else 'ms' if name.endswith('_ms')
                    else 'pct' if name.endswith('_pct') else 'count')
            yield category, pipeline, stage, name, unit, p.get(name), 'measured', 'receiver-local observation'
        if event.get('clock_valid'):
            for name in ('sender_to_receiver_ms', 'sensor_to_receiver_ms', 'end_to_end_ms'):
                yield category, pipeline, stage, name, 'ms', p.get(name), 'measured_clock_validated', 'requires synchronized sender and receiver clocks; interpret with the separately reported cross-host clock uncertainty'
            yield category, pipeline, stage, 'cross_host_clock_uncertainty_ms', 'ms', event.get('cross_host_clock_uncertainty_ms'), 'calculated', 'conservative sum of sender and receiver clock uncertainty at tracker startup'
    else:
        units = {
            '_bytes': 'bytes', '_mbps': 'Mbps', '_ms': 'ms', '_pct': 'pct',
            '_dbm': 'dBm', '_m': 'm', '_hz': 'Hz', '_packets': 'packets',
            '_errors': 'count', '_dropped': 'count'}
        for name, value in p.items():
            # Linux/iw counters are cumulative since interface initialization.
            # Per-run deltas are added by enrich_network_counters().
            if category == 'network' and name in NETWORK_COUNTERS:
                continue
            unit = next((unit for suffix, unit in units.items() if name.endswith(suffix)), 'count')
            if name.endswith('_bytes_delta'):
                unit = 'bytes'
            elif name.endswith('_packets_delta'):
                unit = 'packets'
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                yield category, pipeline, stage, name, unit, value, 'measured', ''


def build_summary(events):
    groups = defaultdict(list)
    for event in events:
        for row in derived_measurements(event):
            _add_metric(groups, *row)
    rows = []
    for key, values in sorted(groups.items()):
        category, pipeline, stage, metric, unit, measurement, note = key
        rows.append({
            'category': category, 'pipeline': pipeline, 'stage': stage,
            'metric': metric, 'unit': unit, 'measurement': measurement, 'note': note,
            **describe(values), 'total': sum(values),
        })
    return rows


def enrich_network_distance(events, max_age_ms):
    poses = defaultdict(list)
    previous_distance = {}
    for event in events:
        if event['category'] == 'pose':
            poses[event['stage']].append({'time_ns': event['wall_time_ns'], **event['payload']})
    for samples in poses.values():
        samples.sort(key=lambda sample: sample['time_ns'])
    max_age_ns = int(max_age_ms * 1e6)
    for event in events:
        if event['category'] != 'network':
            continue
        a = nearest_pose(poses.get('drone', []), event['wall_time_ns'], max_age_ns)
        b = nearest_pose(poses.get('humanoid', []), event['wall_time_ns'], max_age_ns)
        if not a or not b:
            event['payload']['pose_alignment_status'] = 'unavailable_or_stale'
            continue
        pa, age_a = a; pb, age_b = b
        xyz_a = tuple(pa.get(axis) for axis in ('x_m', 'y_m', 'z_m'))
        xyz_b = tuple(pb.get(axis) for axis in ('x_m', 'y_m', 'z_m'))
        if None in xyz_a or None in xyz_b or pa.get('frame_id') != pb.get('frame_id'):
            event['payload']['pose_alignment_status'] = 'frame_mismatch'
            continue
        separation, planar = distance(xyz_a, xyz_b)
        event['payload'].update({
            'distance_m': separation, 'distance_xy_m': planar,
            'system_a_x_m': xyz_a[0], 'system_a_y_m': xyz_a[1], 'system_a_z_m': xyz_a[2],
            'system_b_x_m': xyz_b[0], 'system_b_y_m': xyz_b[1], 'system_b_z_m': xyz_b[2],
            'pose_a_age_ms': age_a / 1e6, 'pose_b_age_ms': age_b / 1e6,
            'pose_alignment_status': 'nearest_within_limit',
        })
        key = (event['host'], event['stage'])
        prior = previous_distance.get(key)
        delta = None if prior is None else separation - prior
        event['payload']['separation_direction'] = (
            'unknown' if delta is None else 'approximately_stationary' if abs(delta) < .05
            else 'increasing' if delta > 0 else 'decreasing')
        previous_distance[key] = separation


def enrich_delivery(events):
    streams = defaultdict(list)
    for event in events:
        if event['category'] == 'delivery':
            p = event['payload']
            # New packet IDs are global across both logical streams.
            stream_class = None if p.get('protocol') == 'information' else p.get('traffic_class')
            streams[(p.get('source_id'), p.get('map_epoch'), stream_class)].append(event)
    for stream in streams.values():
        stream.sort(key=lambda event: (event['payload'].get('version', 0), event['wall_time_ns']))
        updates = defaultdict(list)
        for event in stream:
            updates[event['payload'].get('version')].append(event)
        previous_version = None
        previous_latency = None
        for version, update_events in sorted(updates.items(), key=lambda item: item[0]):
            expected = max((event['payload'].get('chunk_count') or 1)
                           for event in update_events)
            received_indexes = {event['payload'].get('chunk_index', 0)
                                for event in update_events}
            received = len(received_indexes)
            missing = max(0, expected - received)
            traffic_class = update_events[0]['payload'].get('traffic_class')
            # Information packets share consecutive attempt IDs across classes.
            # Legacy datasets retain their old per-stream interpretation.
            gap = max(0, version - previous_version - 1) if (
                (update_events[0]['payload'].get('protocol') == 'information' or traffic_class == 1) and previous_version is not None and
                version is not None) else 0
            incomplete = 1 if missing else 0
            loss_pct = 100.0 * (gap + incomplete) / (gap + 1)
            for event_index, event in enumerate(update_events):
                p = event['payload']
                # Logical-update metrics belong to one row per version, not
                # every chunk; otherwise totals and percentages are weighted
                # by the number of chunks in each update.
                if event_index == 0:
                    p['received_chunks'] = received
                    p['missing_chunks'] = missing
                    p['chunk_loss_pct'] = 100.0 * missing / expected if expected else 0.0
                    p['sequence_gap_updates'] = gap
                    p['update_loss_pct'] = loss_pct
                    p['update_complete'] = 0 if incomplete else 1
                if p.get('accepted'):
                    p['delivered_payload_bytes'] = p.get('wire_bytes')
                latency = p.get('sender_to_receiver_ms') if event.get('clock_valid') else None
                if latency is not None and previous_latency is not None:
                    p['jitter_ms'] = abs(latency - previous_latency)
                if latency is not None:
                    previous_latency = latency
            if version is not None:
                previous_version = max(version, previous_version or version)


def enrich_network_counters(events):
    """Convert interface-lifetime counters into nonnegative per-run deltas."""
    streams = defaultdict(list)
    for event in events:
        if event['category'] == 'network':
            streams[(event['host'], event['pipeline'], event['stage'])].append(event)
    for stream in streams.values():
        stream.sort(key=lambda event: event['wall_time_ns'])
        previous = None
        for event in stream:
            payload = event['payload']
            if previous is not None:
                for name in NETWORK_COUNTERS:
                    current_value = payload.get(name)
                    previous_value = previous.get(name)
                    if current_value is None or previous_value is None:
                        continue
                    # A reset or wrap is unavailable for that interval.
                    if current_value >= previous_value:
                        payload[f'{name}_delta'] = current_value - previous_value
            previous = payload


def build_timeseries(events, window_s):
    if not events:
        return []
    origin = min(event['wall_time_ns'] for event in events)
    groups = defaultdict(list)
    context = defaultdict(dict)
    host_context = defaultdict(dict)
    window_ns = max(1, int(window_s * 1e9))
    for event in events:
        window = (event['wall_time_ns'] - origin) // window_ns
        for row in derived_measurements(event):
            category, pipeline, stage, metric, unit, value, measurement, _ = row
            if value is not None:
                groups[(window, event['host'], category, pipeline, stage, metric, unit, measurement)].append(value)
        if event['category'] == 'network':
            values = {
                key: value for key, value in event['payload'].items()
                if key.startswith(('distance_', 'system_', 'pose_', 'separation_'))}
            context[(window, event['host'], event['pipeline'], event['stage'])].update(values)
            host_context[(window, event['host'])].update(values)
    rows = []
    for key, values in sorted(groups.items()):
        window, host, category, pipeline, stage, metric, unit, measurement = key
        stats = describe(values)
        row = {
            'window_start_s': window * window_s,
            'window_end_s': (window + 1) * window_s, 'host': host,
            'category': category, 'pipeline': pipeline, 'stage': stage,
            'metric': metric, 'unit': unit, 'measurement': measurement,
            'sample_count': stats['sample_count'], 'mean': stats['mean'],
            'min': stats['min'], 'max': stats['max'], 'total': sum(values),
        }
        row.update(host_context.get((window, host), {}))
        row.update(context.get((window, host, pipeline, stage), {}))
        rows.append(row)
        if metric in ('output_bytes', 'data_bytes', 'wire_bytes', 'delivered_payload_bytes'):
            rate_row = dict(row)
            rate_name = ('application_goodput_mbps' if metric == 'delivered_payload_bytes'
                         else metric.removesuffix('_bytes') + '_rate_mbps')
            rate_row.update(metric=rate_name, unit='Mbps',
                            measurement='calculated', mean=sum(values) * 8.0 / window_s / 1e6,
                            min=None, max=None, total=None)
            rows.append(rate_row)
    return rows


def _write_csv(path, rows, default_fields):
    fields = list(default_fields)
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)


def generate_plots(timeseries, directory):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return {'status': 'unavailable', 'reason': 'python3-matplotlib is not installed'}
    directory.mkdir(parents=True, exist_ok=True)
    created = []
    candidates = ('output_rate_mbps', 'data_rate_mbps', 'tx_interface_mbps',
                  'rx_interface_mbps', 'application_goodput_mbps',
                  'sender_to_receiver_ms', 'jitter_ms', 'packet_loss_pct', 'rssi_dbm',
                  'tx_bitrate_mbps', 'rx_bitrate_mbps', 'distance_m',
                  'compute_ms', 'output_bytes')
    for metric in candidates:
        rows = [row for row in timeseries if row['metric'] == metric and row.get('mean') is not None]
        if not rows:
            continue
        fig, axis = plt.subplots(figsize=(8, 4.5))
        for label in sorted({f"{row['pipeline']}:{row['stage']}" for row in rows}):
            selected = [row for row in rows if f"{row['pipeline']}:{row['stage']}" == label]
            axis.plot([row['window_start_s'] for row in selected],
                      [row['mean'] for row in selected], marker='.', label=label)
        axis.set(xlabel='Elapsed time (s)', ylabel=f"{metric} ({rows[0]['unit']})")
        axis.grid(True, alpha=.3); axis.legend(fontsize=7); fig.tight_layout()
        path = directory / f'{metric}_vs_time.png'; fig.savefig(path, dpi=160); plt.close(fig)
        created.append(path.name)
    # Direct, unfit observations against distance.
    for metric in ('tx_interface_mbps', 'rx_interface_mbps', 'application_goodput_mbps',
                   'sender_to_receiver_ms', 'jitter_ms', 'packet_loss_pct',
                   'rssi_dbm', 'tx_bitrate_mbps', 'rx_bitrate_mbps'):
        rows = [row for row in timeseries if row['metric'] == metric and row.get('distance_m') is not None]
        if not rows:
            continue
        fig, axis = plt.subplots(figsize=(7, 4.5))
        axis.scatter([row['distance_m'] for row in rows], [row['mean'] for row in rows], s=12)
        axis.set(xlabel='Physical separation (m)', ylabel=f"{metric} ({rows[0]['unit']})")
        axis.grid(True, alpha=.3); fig.tight_layout()
        path = directory / f'{metric}_vs_distance.png'; fig.savefig(path, dpi=160); plt.close(fig)
        created.append(path.name)
    return {'status': 'generated', 'files': created, 'models_fitted': False}


def analyze(run_directory, output=None, window_s=1.0, pose_max_age_ms=500.0,
            export_raw=False, plots=True):
    run_directory = Path(run_directory).expanduser().resolve()
    output = Path(output).expanduser().resolve() if output else run_directory
    output.mkdir(parents=True, exist_ok=True)
    events, hosts = load_events(run_directory)
    if not hosts:
        raise ValueError(f'no version-2 telemetry databases found under {run_directory}')
    enrich_network_distance(events, pose_max_age_ms)
    enrich_delivery(events)
    enrich_network_counters(events)
    summary = build_summary(events)
    timeseries = build_timeseries(events, window_s)
    _write_csv(output / 'summary.csv', summary,
               ('category', 'pipeline', 'stage', 'metric', 'unit', 'measurement',
                'sample_count', 'mean', 'median', 'min', 'max', 'stddev',
                'p5', 'p50', 'p95', 'p99', 'total', 'note'))
    _write_csv(output / 'timeseries.csv', timeseries,
               ('window_start_s', 'window_end_s', 'host', 'category', 'pipeline',
                'stage', 'metric', 'unit', 'measurement', 'sample_count',
                'mean', 'min', 'max', 'total'))
    export_raw = export_raw or any(host.get('raw_event_persistence') for host in hosts)
    if export_raw:
        raw = [{**{key: value for key, value in event.items() if key != 'payload'},
                **event['payload']} for event in events]
        _write_csv(output / 'raw_events.csv', raw, ('event_id', 'wall_time_ns',
                   'monotonic_ns', 'host', 'category', 'pipeline', 'stage'))
    run_ids = sorted({host.get('run_id') for host in hosts if host.get('run_id')})
    metadata = {
        'schema_version': 2, 'run_ids': run_ids, 'run_id_consistent': len(run_ids) == 1,
        'analysis_time_utc': datetime.now(timezone.utc).isoformat(),
        'timeseries_window_s': window_s, 'pose_sync_max_age_ms': pose_max_age_ms,
        'hosts': hosts, 'event_count_after_deduplication': len(events),
        'clock_warning': ('Cross-host one-way latency is valid only for hosts whose '
                          'clock_sync_method is verified and uncertainty is recorded.'),
        'unavailable_by_architecture': [
            'network_bridge private framing/compression bytes',
            'receiver Bonxai integration compute time',
            'per-flow TCP retransmissions without eBPF/socket instrumentation',
            'unused link capacity; normal application traffic measures consumption, not saturation capacity'],
    }
    metadata['plots'] = generate_plots(timeseries, output / 'plots') if plots else {'status': 'disabled'}
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description='Merge and analyze a distributed SURF run.')
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--window-seconds', type=float, default=1.0)
    parser.add_argument('--pose-max-age-ms', type=float, default=500.0)
    parser.add_argument('--raw-events', action='store_true')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    result = analyze(args.run_directory, args.output, args.window_seconds,
                     args.pose_max_age_ms, args.raw_events, not args.no_plots)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
