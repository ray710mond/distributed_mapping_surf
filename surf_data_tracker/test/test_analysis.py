import csv
import json
import sqlite3
import tempfile
import unittest
from pathlib import Path

from surf_data_tracker.analysis import (analyze, build_summary, build_timeseries,
                                        derived_measurements, enrich_delivery,
                                        enrich_network_distance, load_events)
from surf_data_tracker.storage import EventStore


class AnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / 'run-1'

    def tearDown(self):
        self.temporary.cleanup()

    def _store(self, host, role, status='completed'):
        store = EventStore(self.root / 'hosts' / host, 'run-1', role,
                           {'clock_sync_method': 'chrony', 'clock_uncertainty_ms': .5})
        return store

    def test_synthetic_pipeline_summary_and_windows(self):
        store = self._store('drone-a', 'drone')
        for index, stamp in enumerate((1_000_000_000, 1_500_000_000)):
            store.record('pipeline', 'pointcloud_communication', 'realtime', {
                'traffic_class_name': 'realtime', 'raw_points': 100,
                'valid_points': 80, 'unique_voxels': 40, 'selected_voxels': 20,
                'raw_serialized_bytes': 1000, 'uncompressed_bytes': 400,
                'payload_bytes': 100, 'wire_bytes': 150,
                'processing_ms': 10 + index, 'compression_ms': 2},
                wall_time_ns=stamp, event_id=f'p{index}')
        store.close()
        metadata = analyze(self.root, plots=False)
        self.assertEqual(metadata['event_count_after_deduplication'], 2)
        with (self.root / 'summary.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        compression = next(row for row in rows if row['stage'] == 'realtime:compression'
                           and row['metric'] == 'cumulative_saved_pct')
        self.assertEqual(float(compression['mean']), 90.0)
        timing = next(row for row in rows if row['stage'] == 'realtime:sender_total'
                      and row['metric'] == 'compute_ms')
        self.assertEqual(float(timing['mean']), 10.5)
        self.assertGreater(float(timing['p95']), 10.9)
        with (self.root / 'timeseries.csv').open() as stream:
            windows = list(csv.DictReader(stream))
        self.assertTrue(any(row['metric'] == 'output_rate_mbps' for row in windows))

    def test_merge_missing_host_duplicate_and_interrupted(self):
        store = self._store('drone-a', 'drone')
        store.record('data', 'map', 'occupied', {'data_bytes': 10}, event_id='same')
        store.record('data', 'map', 'occupied', {'data_bytes': 20}, event_id='same')
        store.close('interrupted')
        events, hosts = load_events(self.root)
        self.assertEqual(len(events), 1)
        self.assertEqual(len(hosts), 1)
        self.assertEqual(hosts[0]['status'], 'interrupted')
        analyze(self.root, plots=False)
        self.assertTrue((self.root / 'summary.csv').is_file())

    def test_missing_wireless_metrics_are_not_fabricated(self):
        store = self._store('humanoid-a', 'humanoid')
        store.record('network', 'wireless', 'halow_realtime',
                     {'interface_state': 'unavailable'}, event_id='network-missing')
        store.close()
        analyze(self.root, plots=False)
        with (self.root / 'summary.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertFalse(any(row['category'] == 'network' for row in rows))

    def test_cross_host_clock_requires_verification_from_every_host(self):
        verified = self._store('humanoid-a', 'humanoid')
        verified.record('delivery', 'pointcloud_communication', 'realtime',
                        {'sender_to_receiver_ms': 3.0}, event_id='delivery')
        verified.close()
        unverified = EventStore(
            self.root / 'hosts' / 'drone-a', 'run-1', 'drone',
            {'clock_sync_method': 'unverified', 'clock_uncertainty_ms': None})
        unverified.close()
        events, _ = load_events(self.root)
        self.assertFalse(events[0]['clock_valid'])

    def test_cross_host_latency_reports_combined_clock_uncertainty(self):
        receiver = self._store('humanoid-a', 'humanoid')
        receiver.record('delivery', 'pointcloud_communication', 'realtime',
                        {'sender_to_receiver_ms': 3.0}, event_id='delivery')
        receiver.close()
        sender = EventStore(
            self.root / 'hosts' / 'drone-a', 'run-1', 'drone',
            {'clock_sync_method': 'chrony', 'clock_uncertainty_ms': 1.5})
        sender.close()
        events, _ = load_events(self.root)
        rows = list(derived_measurements(events[0]))
        uncertainty = next(row for row in rows
                           if row[3] == 'cross_host_clock_uncertainty_ms')
        self.assertEqual(uncertainty[5], 2.0)

    def test_cumulative_network_counters_become_run_deltas(self):
        store = self._store('drone-a', 'drone')
        store.record('network', 'wireless', 'halow_realtime',
                     {'rx_bytes': 1000, 'tx_packets': 40},
                     wall_time_ns=1_000_000_000, event_id='n1')
        store.record('network', 'wireless', 'halow_realtime',
                     {'rx_bytes': 1600, 'tx_packets': 45},
                     wall_time_ns=2_000_000_000, event_id='n2')
        store.close()
        analyze(self.root, plots=False)
        with (self.root / 'summary.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        rx = next(row for row in rows if row['metric'] == 'rx_bytes_delta')
        packets = next(row for row in rows if row['metric'] == 'tx_packets_delta')
        self.assertEqual(float(rx['total']), 600)
        self.assertEqual(float(packets['total']), 5)
        self.assertFalse(any(row['metric'] == 'rx_bytes' for row in rows))

    def test_distance_frame_guard_and_alignment(self):
        events = [
            {'event_id': 'a', 'wall_time_ns': 100, 'host': 'h', 'category': 'pose',
             'pipeline': 'localization', 'stage': 'drone',
             'payload': {'x_m': 0, 'y_m': 0, 'z_m': 0, 'frame_id': 'map'}},
            {'event_id': 'b', 'wall_time_ns': 105, 'host': 'h', 'category': 'pose',
             'pipeline': 'localization', 'stage': 'humanoid',
             'payload': {'x_m': 3, 'y_m': 4, 'z_m': 0, 'frame_id': 'map'}},
            {'event_id': 'n', 'wall_time_ns': 110, 'host': 'h', 'category': 'network',
             'pipeline': 'wireless', 'stage': 'halow', 'payload': {}}]
        enrich_network_distance(events, max_age_ms=.001)
        self.assertEqual(events[-1]['payload']['distance_m'], 5)
        events[1]['payload']['frame_id'] = 'humanoid/map'
        events[-1]['payload'] = {}
        enrich_network_distance(events, max_age_ms=.001)
        self.assertEqual(events[-1]['payload']['pose_alignment_status'], 'frame_mismatch')

    def test_graceful_close_is_idempotent(self):
        store = self._store('drone-a', 'drone')
        store.close(); store.close()
        metadata = json.loads((self.root / 'hosts/drone-a/metadata.json').read_text())
        self.assertEqual(metadata['status'], 'completed')

    def test_reused_run_id_replaces_previous_host_events(self):
        store = self._store('drone-a', 'drone')
        store.record('data', 'map', 'occupied', {'data_bytes': 10}, event_id='old')
        store.close()

        replacement = self._store('drone-a', 'drone')
        replacement.record(
            'data', 'map', 'occupied', {'data_bytes': 20}, event_id='new')
        replacement.close()

        with sqlite3.connect(self.root / 'hosts/drone-a/telemetry.sqlite3') as connection:
            event_ids = [row[0] for row in connection.execute(
                'SELECT event_id FROM events ORDER BY event_id')]
        self.assertEqual(event_ids, ['new'])

    def test_chunk_loss_and_update_gaps_are_distinct(self):
        events = []
        for version, chunk in ((10, 0), (10, 2), (12, 0)):
            events.append({
                'wall_time_ns': version * 100 + chunk, 'category': 'delivery',
                'payload': {'source_id': 'drone', 'map_epoch': 1, 'traffic_class': 1,
                            'version': version, 'chunk_index': chunk,
                            'chunk_count': 3 if version == 10 else 1}})
        enrich_delivery(events)
        first = events[0]['payload']
        self.assertEqual(first['received_chunks'], 2)
        self.assertEqual(first['missing_chunks'], 1)
        self.assertAlmostEqual(first['chunk_loss_pct'], 100.0 / 3.0)
        last = events[-1]['payload']
        self.assertEqual(last['sequence_gap_updates'], 1)
        self.assertEqual(last['update_loss_pct'], 50.0)

    def test_sync_version_spacing_is_not_reported_as_loss(self):
        events = [
            {'wall_time_ns': version, 'category': 'delivery',
             'payload': {'source_id': 'drone', 'map_epoch': 1, 'traffic_class': 2,
                         'version': version, 'chunk_index': 0, 'chunk_count': 1}}
            for version in (10, 50)]
        enrich_delivery(events)
        self.assertEqual(events[-1]['payload']['sequence_gap_updates'], 0)
        self.assertEqual(events[-1]['payload']['update_loss_pct'], 0.0)

    def test_logical_update_metrics_are_not_repeated_per_chunk(self):
        events = [
            {'wall_time_ns': chunk, 'category': 'delivery',
             'payload': {'source_id': 'drone', 'map_epoch': 1, 'traffic_class': 1,
                         'version': 10, 'chunk_index': chunk, 'chunk_count': 3}}
            for chunk in (0, 2)]
        enrich_delivery(events)
        self.assertEqual(events[0]['payload']['missing_chunks'], 1)
        self.assertNotIn('missing_chunks', events[1]['payload'])


if __name__ == '__main__':
    unittest.main()

class AllocationCounterSummaryTest(unittest.TestCase):
    def test_counter_total_differences_each_epoch_and_keeps_signed_disturbance(self):
        events = [dict(wall_time_ns=i, host='drone', category='allocation', pipeline='debt',
                       stage='controller', payload=dict(source_id='drone', map_epoch=epoch,
                       generated_debt_0=generated, disturbance_0=disturbance))
                  for i, (epoch, generated, disturbance) in enumerate(
                      [(1, 100, 50), (1, 120, 40), (2, 500, 100), (2, 530, 105)])]
        rows = {r['metric']: r for r in build_summary(events)}
        self.assertEqual(rows['generated_debt_0']['total'], 50)
        self.assertEqual(rows['disturbance_0']['total'], -5)
        self.assertTrue(all(r['total'] is None for r in build_timeseries(events, 1)
                            if r['metric'] in ('generated_debt_0', 'disturbance_0')))

    def test_observation_ages_and_unresolved_one_way_latency(self):
        event = dict(category='delivery', pipeline='communication', stage='backlog',
                     clock_valid=True, cross_host_clock_uncertainty_ms=68,
                     payload=dict(sender_to_receiver_ms=10, oldest_accepted_observation_age_s=57))
        rows = {r[3]: r for r in derived_measurements(event)}
        self.assertEqual(rows['one_way_below_clock_uncertainty'][5], 1)
        self.assertEqual(rows['oldest_accepted_observation_age_s'][4:6], ('s', 57))
