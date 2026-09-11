import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from surf_data_tracker.clock import _chrony_csv_status, clock_document, detect_clock_sync


class ClockVerificationTest(unittest.TestCase):
    def test_parses_normal_chrony_tracking(self):
        text = 'A29FC801,3,1,0.0002,-0.0001,0.0003,0,0,0,0.001,0.0004,2,Normal'
        result = _chrony_csv_status(text, now_ns=1_000_000_000)
        self.assertEqual(result['clock_sync_method'], 'chrony-local')
        self.assertAlmostEqual(result['clock_offset_ms'], .2)
        self.assertAlmostEqual(result['clock_uncertainty_ms'], 1.4)

    def test_parses_chrony_tracking_with_reference_hostname(self):
        text = ('0A2B1E02,raymond-HP-Envy,4,1,0.0002,-0.0001,0.0003,'
                '0,0,0,0.001,0.0004,2,Normal')
        result = _chrony_csv_status(text, now_ns=1_000_000_000)
        self.assertEqual(result['clock_reference'], 'raymond-HP-Envy')
        self.assertAlmostEqual(result['clock_offset_ms'], .2)
        self.assertAlmostEqual(result['clock_uncertainty_ms'], 1.4)

    def test_rejects_local_authority_and_unsynchronized_tracking(self):
        for reference, stratum, leap in [('7F7F0101', 10, 'Normal'),
                                          ('00000000', 0, 'Not synchronised'),
                                          ('A29FC801', 16, 'Normal')]:
            text = f'{reference},{stratum},1,0,0,0,0,0,0,0,0,2,{leap}'
            self.assertIsNone(clock_document(text))

    def test_classifies_both_fallback_directions_and_internet_recovery(self):
        for peer, reference in [('10.43.30.1', '0A2B1E01'),
                                ('10.43.30.2', '0A2B1E02')]:
            template = '{},3,1,0.0002,0,0.0003,0,0,0,0.001,0.0004,2,Normal'
            fallback = clock_document(template.format(reference), peer=peer)
            self.assertEqual(fallback['method'], 'chrony-halow-client')
            recovered = clock_document(template.format('A29FC801'), peer=peer)
            self.assertEqual(recovered['method'], 'chrony-internet')

    def test_explicit_failure_invalidates_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            path.write_text(json.dumps({'verified': False}))
            with patch('surf_data_tracker.clock.subprocess.run') as run:
                self.assertIsNone(detect_clock_sync(path))
                run.assert_not_called()

    def test_rejects_stale_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            path.write_text(json.dumps({
                'verified': True, 'method': 'chrony-halow-client',
                'offset_ms': 0.1, 'uncertainty_ms': 1.0,
                'measured_at_unix_ns': 1,
            }))
            with patch('surf_data_tracker.clock.subprocess.run') as run:
                run.return_value.returncode = 1
                self.assertIsNone(detect_clock_sync(path, now_ns=400_000_000_001))

    def test_reads_fresh_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            path.write_text(json.dumps({
                'verified': True, 'method': 'chrony-halow-client',
                'source': '10.42.0.1', 'offset_ms': -0.2,
                'uncertainty_ms': 1.5, 'measured_at_unix_ns': 1_000_000_000,
            }))
            result = detect_clock_sync(path, now_ns=2_000_000_000)
            self.assertEqual(result['clock_reference'], '10.42.0.1')
            self.assertEqual(result['clock_uncertainty_ms'], 1.5)


if __name__ == '__main__':
    unittest.main()
