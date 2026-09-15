import unittest
from types import SimpleNamespace

from surf_data_tracker.tracker import DataTracker


class MapTransmissionTransitionTests(unittest.TestCase):
    def test_pause_reason_changes_and_resume_are_recorded_once(self):
        events = []
        tracker = SimpleNamespace(
            _map_capacity_pause_reasons={},
            _record=lambda *args, **kwargs: events.append((args, kwargs)))
        fields = ('header', 'source_id', 'map_epoch', 'step', 'pending_count',
                  'usable_capacity', 'capacity_method', 'requested_rate',
                  'allocated_rate', 'projection_scale', 'selected_count',
                  'sent_priority', 'wire_bytes', 'actual_dt')
        message = SimpleNamespace(
            get_fields_and_field_types=lambda: dict.fromkeys(fields),
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
            source_id='drone', map_epoch=7, step=1, pending_count=[3, 0],
            usable_capacity=0.0, capacity_method='waiting_for_telemetry',
            requested_rate=[0.0, 0.0], allocated_rate=[0.0, 0.0],
            projection_scale=1.0, selected_count=[0, 0],
            sent_priority=[0.0, 0.0], wire_bytes=[0, 0], actual_dt=.1)

        def transitions():
            return [args[2] for args, _ in events if args[0] == 'transport']

        DataTracker._allocation(tracker, message)
        self.assertEqual(transitions(), ['paused:waiting_for_telemetry'])
        message.step += 1
        DataTracker._allocation(tracker, message)
        self.assertEqual(len(transitions()), 1)
        message.step += 1
        message.capacity_method = 'stale_telemetry'
        DataTracker._allocation(tracker, message)
        self.assertEqual(transitions()[-1], 'paused:stale_telemetry')
        message.step += 1
        message.usable_capacity = 10000.0
        message.capacity_method = 'guarded_mmrc_estimate'
        DataTracker._allocation(tracker, message)
        self.assertEqual(transitions()[-1], 'resumed:stale_telemetry')
        message.step += 1
        DataTracker._allocation(tracker, message)
        self.assertEqual(len(transitions()), 3)


if __name__ == '__main__':
    unittest.main()
