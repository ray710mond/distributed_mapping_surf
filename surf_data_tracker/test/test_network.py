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
        with patch('pathlib.Path.read_text', lambda path: values[str(path)]):
            result = NetworkSampler._morse_debugfs_stats('morse0')
        self.assertEqual(result['s1g_mcs'], 9)
        self.assertEqual(result['s1g_bandwidth_mhz'], 4)
        self.assertEqual(result['s1g_short_guard_interval'], 1)
        self.assertEqual(result['morse_mmrc_total_attempts'], 30)
        self.assertEqual(result['morse_page_data_tx'], 50)


if __name__ == '__main__':
    unittest.main()
