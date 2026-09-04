import math
import unittest

from surf_data_tracker.metrics import describe, distance, nearest_pose, percentile, safe_reduction
from surf_data_tracker.storage import validate_run_id


class MetricsTest(unittest.TestCase):
    def test_incremental_and_cumulative_reduction(self):
        self.assertEqual(safe_reduction(100, 40), (60, 60.0, 2.5))
        self.assertEqual(safe_reduction(40, 10), (30, 75.0, 4.0))
        self.assertEqual(safe_reduction(100, 10), (90, 90.0, 10.0))

    def test_zero_handling_is_explicit(self):
        self.assertEqual(safe_reduction(0, 0), (0, None, None))
        self.assertEqual(safe_reduction(10, 0), (10, 100.0, None))

    def test_statistics_and_percentiles(self):
        values = [1, 2, 3, 4, 5]
        result = describe(values)
        self.assertEqual(result['mean'], 3)
        self.assertEqual(result['median'], 3)
        self.assertAlmostEqual(percentile(values, .95), 4.8)
        self.assertEqual(result['sample_count'], 5)

    def test_distance_and_temporal_association(self):
        self.assertEqual(distance((0, 0, 0), (3, 4, 12)), (13, 5))
        samples = [{'time_ns': 100, 'x_m': 0}, {'time_ns': 200, 'x_m': 1}]
        self.assertEqual(nearest_pose(samples, 175, 30)[0]['time_ns'], 200)
        self.assertIsNone(nearest_pose(samples, 300, 30))

    def test_run_id_rejects_whitespace_and_paths(self):
        self.assertEqual(validate_run_id('practice-20260825T190254Z'),
                         'practice-20260825T190254Z')
        for invalid in ('practice run', 'practice\nrun', '../practice', '/tmp/run'):
            with self.assertRaises(ValueError):
                validate_run_id(invalid)


if __name__ == '__main__':
    unittest.main()
