import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from surf_data_tracker.collect import collect


class Result:
    returncode = 0
    stderr = ''


class CollectionTest(unittest.TestCase):
    @patch('surf_data_tracker.collect.subprocess.run', return_value=Result())
    def test_repeated_collection_replaces_changed_host_files(self, run):
        with tempfile.TemporaryDirectory() as directory:
            first = collect('run-1', Path(directory), ['drone=user@host:/data/telemetry'])
            second = collect('run-1', Path(directory), ['drone=user@host:/data/telemetry'])
            self.assertTrue(first['all_succeeded'] and second['all_succeeded'])
            command = run.call_args.args[0]
            self.assertIn('--partial', command)
            self.assertNotIn('--ignore-existing', command)

    @patch('surf_data_tracker.collect.subprocess.run')
    def test_failed_machine_is_reported_and_preserved(self, run):
        result = Result(); result.returncode = 255; result.stderr = 'unreachable'
        run.return_value = result
        with tempfile.TemporaryDirectory() as directory:
            report = collect('run-1', Path(directory), ['drone=user@host:/data/telemetry'])
            self.assertFalse(report['all_succeeded'])
            self.assertEqual(report['sources'][0]['error'], 'unreachable')


if __name__ == '__main__':
    unittest.main()
