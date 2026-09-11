import unittest
from unittest.mock import patch

from surf_data_tracker.network import NetworkSampler


class NetworkSamplerTest(unittest.TestCase):
    def test_rejects_implausible_halow_station_metrics(self):
        values = {'rssi_dbm': 6.0, 'rssi_avg_dbm': 121.0, 'tx_bitrate_mbps': 433.3,
                  'rx_bitrate_mbps': 433.3}
        NetworkSampler._label_radio_metrics(values, 'halow_realtime')
        self.assertNotIn('rssi_dbm', values)
        self.assertNotIn('rssi_avg_dbm', values)
        self.assertNotIn('tx_bitrate_mbps', values)
        self.assertEqual(values['tx_compatibility_rate_mbps'], 433.3)
        self.assertEqual(values['rssi_state'], 'saturated_or_invalid')

    def test_keeps_plausible_halow_station_metrics(self):
        values = {'rssi_dbm': -74.0, 'tx_bitrate_mbps': 32.5}
        NetworkSampler._label_radio_metrics(values, 'halow_realtime')
        self.assertEqual(values['rssi_dbm'], -74.0)
        self.assertEqual(values['tx_compatibility_rate_mbps'], 32.5)
        self.assertNotIn('tx_bitrate_mbps', values)

    def test_rejects_unknown_rssi_sentinel(self):
        values = {'rssi_dbm': -127.0, 'rssi_avg_dbm': -127.0}
        NetworkSampler._label_radio_metrics(values, 'halow_realtime')
        self.assertNotIn('rssi_dbm', values)
        self.assertNotIn('rssi_avg_dbm', values)

    def test_flattens_native_morse_json_metrics(self):
        output = {}
        NetworkSampler._flatten_numeric(
            {'phy': {'mcs': 4, 'success': 98.5}}, output, 'morse', 256)
        self.assertEqual(output['morse_phy_mcs'], 4)
        self.assertEqual(output['morse_phy_success'], 98.5)

    def test_parses_morse_debugfs_physical_rate(self):
        table = ('bandwidth,guard,rate_selection,mcs,max_throughput,'
                 'average_throughput,probability,last_retry,total_success,'
                 'total_attempts,mpdu_success,mpdu_failures\n'
                 '4MHz,SGI,AL,MCS9,20.0,19.5,98,1,29,30,28,2\n')
        values = {
            '/run/surf-halow-telemetry/morse0/mmrc_table_csv': table,
            '/run/surf-halow-telemetry/morse0/page_stats': 'Data Tx: 50\nQueue stop: 2\n'}
        with patch('pathlib.Path.read_text', lambda path: values[str(path)]), \
                patch('pathlib.Path.stat') as stat:
            import time
            stat.return_value.st_mtime = time.time()
            result = NetworkSampler._morse_debugfs_stats('morse0')
        self.assertEqual(result['s1g_mcs'], 9)
        self.assertEqual(result['s1g_bandwidth_mhz'], 4)
        self.assertEqual(result['s1g_short_guard_interval'], 1)
        self.assertEqual(result['morse_mmrc_total_attempts'], 30)
        self.assertEqual(result['morse_page_data_tx'], 50)


if __name__ == '__main__':
    unittest.main()

class MorseCacheTest(unittest.TestCase):
    def test_poll_jitter_reuses_fresh_sample_and_expires_on_read_failure(self):
        sampler = NetworkSampler({'morse0': 'halow'})
        with patch.object(sampler, '_read_morse_stats', side_effect=[
                {'morse_stats_state': 'debugfs_available', 's1g_average_throughput_mbps': 20.0},
                {'morse_stats_state': 'error'}, {'morse_stats_state': 'error'}]) as read:
            self.assertEqual(sampler._morse_stats('morse0', 1_000_000_000)['s1g_average_throughput_mbps'], 20)
            cached = sampler._morse_stats('morse0', 1_999_000_000)
            self.assertEqual(cached['morse_poll_state'], 'poll_suppressed')
            self.assertAlmostEqual(cached['morse_measurement_age_s'], .999)
            self.assertEqual(read.call_count, 1)
            self.assertTrue(sampler._morse_stats('morse0', 2_100_000_000)['morse_sample_valid'])
            expired = sampler._morse_stats('morse0', 4_100_000_000)
            self.assertFalse(expired['morse_sample_valid'])
            self.assertNotIn('s1g_average_throughput_mbps', expired)

    def test_export_age_is_not_reset_by_caching(self):
        sampler = NetworkSampler({'morse0': 'halow'})
        with patch.object(sampler, '_read_morse_stats', return_value={
                'morse_stats_state': 'debugfs_available', 'morse_measurement_age_s': 2.9,
                's1g_average_throughput_mbps': 20.0}):
            self.assertTrue(sampler._morse_stats('morse0', 10_000_000_000)['morse_sample_valid'])
            self.assertFalse(sampler._morse_stats('morse0', 10_200_000_000)['morse_sample_valid'])

    def test_access_point_iw_output_does_not_override_interface_operstate(self):
        from types import SimpleNamespace
        with patch('subprocess.run', side_effect=[
                SimpleNamespace(stdout='Not connected.\n'),
                SimpleNamespace(stdout='Station 00:11:22:33:44:55\n signal: -4 dBm\n')]):
            values = NetworkSampler._iw('morse0')
        self.assertNotIn('interface_state', values)
        self.assertEqual(values['association_state'], 'not_reported')
        self.assertEqual(values['rssi_dbm'], -4)
