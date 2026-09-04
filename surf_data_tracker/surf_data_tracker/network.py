"""Low-rate Linux network sampling with graceful unavailable values."""

import re
import json
import shutil
import csv
import io
import subprocess
import time
from pathlib import Path


COUNTERS = ('rx_bytes', 'tx_bytes', 'rx_packets', 'tx_packets', 'rx_errors',
            'tx_errors', 'rx_dropped', 'tx_dropped')


def _number(path):
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return None


class NetworkSampler:
    def __init__(self, interfaces, morse_stats_rate_hz=1.0, morse_cli=''):
        self.interfaces = dict(interfaces)
        self.previous = {}
        self.morse_interval_ns = int(1e9 / max(0.1, morse_stats_rate_hz))
        self.morse_last_sample_ns = {}
        self.morse_cli = morse_cli or shutil.which('morse_cli') or shutil.which('morsectrl')

    @staticmethod
    def _iw(interface):
        try:
            result = subprocess.run(
                ['iw', 'dev', interface, 'link'], capture_output=True, text=True,
                timeout=1.0, check=False)
        except (OSError, subprocess.TimeoutExpired):
            return {}
        text = result.stdout
        values = {}
        patterns = {
            'rssi_dbm': r'signal:\s*(-?[0-9.]+)\s*dBm',
            'rssi_avg_dbm': r'signal avg:\s*(-?[0-9.]+)\s*dBm',
            'tx_bitrate_mbps': r'tx bitrate:\s*([0-9.]+)\s*MBit/s',
            'rx_bitrate_mbps': r'rx bitrate:\s*([0-9.]+)\s*MBit/s',
            'expected_throughput_mbps': r'expected throughput:\s*([0-9.]+)\s*Mbps',
        }
        for name, pattern in patterns.items():
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                values[name] = float(match.group(1))
        values['interface_state'] = 'connected' if 'Connected to ' in text else 'unavailable'
        try:
            station = subprocess.run(
                ['iw', 'dev', interface, 'station', 'dump'], capture_output=True,
                text=True, timeout=1.0, check=False).stdout
        except (OSError, subprocess.TimeoutExpired):
            station = ''
        station_patterns = {
            'tx_retries': r'tx retries:\s*([0-9]+)',
            'tx_failed': r'tx failed:\s*([0-9]+)',
            'station_tx_packets': r'tx packets:\s*([0-9]+)',
            'station_rx_packets': r'rx packets:\s*([0-9]+)',
            'beacon_loss': r'beacon loss:\s*([0-9]+)',
            'rx_drop_misc': r'rx drop misc:\s*([0-9]+)',
            'inactive_time_ms': r'inactive time:\s*([0-9]+)\s*ms',
            'connected_time_s': r'connected time:\s*([0-9]+)\s*seconds',
        }
        for name, pattern in station_patterns.items():
            match = re.search(pattern, station, re.IGNORECASE)
            if match:
                values[name] = int(match.group(1))
        for name, pattern in patterns.items():
            if name in values:
                continue
            match = re.search(pattern, station, re.IGNORECASE)
            if match:
                values[name] = float(match.group(1))
        return values

    def _morse_stats(self, interface, now_ns):
        previous = self.morse_last_sample_ns.get(interface)
        if previous is not None and now_ns - previous < self.morse_interval_ns:
            return {}
        self.morse_last_sample_ns[interface] = now_ns
        debugfs = self._morse_debugfs_stats(interface)
        if debugfs:
            return debugfs
        if not self.morse_cli:
            return {'morse_stats_state': 'debugfs_unreadable_and_cli_unavailable'}
        try:
            result = subprocess.run(
                [self.morse_cli, '-i', interface, 'stats', '-j'],
                capture_output=True, text=True, timeout=1.0, check=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            return {'morse_stats_state': 'error', 'morse_stats_error': str(error)}
        if result.returncode != 0:
            return {'morse_stats_state': 'error',
                    'morse_stats_error': result.stderr.strip()[:256]}
        try:
            document = json.loads(result.stdout)
        except json.JSONDecodeError as error:
            return {'morse_stats_state': 'invalid_json',
                    'morse_stats_error': str(error)}
        output = {'morse_stats_state': 'available'}
        self._flatten_numeric(document, output, 'morse', 256)
        return output

    @staticmethod
    def _morse_debugfs_stats(interface):
        exported_root = Path('/run/surf-halow-telemetry') / interface
        try:
            table_text = (exported_root / 'mmrc_table_csv').read_text()
            page_text = (exported_root / 'page_stats').read_text()
        except OSError:
            # Direct access remains useful for development and root-run hosts.
            try:
                phy = (Path('/sys/class/net') / interface / 'phy80211/name').read_text().strip()
                root = Path('/sys/kernel/debug/ieee80211') / phy / 'morse'
                table_text = (root / 'mmrc_table_csv').read_text()
                page_text = (root / 'page_stats').read_text()
            except OSError:
                return {}
        rows = list(csv.DictReader(io.StringIO(table_text)))
        output = {'morse_stats_state': 'debugfs_available'}
        numeric_columns = ('total_success', 'total_attempts', 'mpdu_success', 'mpdu_failures')
        for column in numeric_columns:
            output[f'morse_mmrc_{column}'] = sum(
                int(float(row.get(column) or 0)) for row in rows)
        # MMRC marks its best/current throughput choice with A. Keep the
        # physical S1G rate tuple, not nl80211's compatibility representation.
        selected = next((row for row in rows if 'A' in (row.get('rate_selection') or '')), None)
        if selected:
            bandwidth = re.search(r'([0-9]+)', selected.get('bandwidth', ''))
            mcs = re.search(r'([0-9]+)', selected.get('mcs', ''))
            if bandwidth:
                output['s1g_bandwidth_mhz'] = int(bandwidth.group(1))
            if mcs:
                output['s1g_mcs'] = int(mcs.group(1))
            output['s1g_short_guard_interval'] = int(selected.get('guard') == 'SGI')
            for source, target in (
                    ('max_throughput', 's1g_max_throughput_mbps'),
                    ('average_throughput', 's1g_average_throughput_mbps'),
                    ('probability', 's1g_success_probability_pct'),
                    ('last_retry', 's1g_last_retry_count')):
                try:
                    output[target] = float(selected[source])
                except (KeyError, TypeError, ValueError):
                    pass
        for line in page_text.splitlines():
            if ':' not in line:
                continue
            name, value = line.split(':', 1)
            try:
                number = int(value.strip())
            except ValueError:
                continue
            clean = re.sub(r'[^a-z0-9]+', '_', name.lower()).strip('_')
            output[f'morse_page_{clean}'] = number
        return output

    @classmethod
    def _flatten_numeric(cls, value, output, prefix, limit):
        if len(output) >= limit:
            return
        if isinstance(value, dict):
            for key, child in value.items():
                clean = re.sub(r'[^a-z0-9]+', '_', str(key).lower()).strip('_')
                cls._flatten_numeric(child, output, f'{prefix}_{clean}', limit)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                cls._flatten_numeric(child, output, f'{prefix}_{index}', limit)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            output[prefix] = value

    def sample(self, now_ns=None):
        now_ns = now_ns or time.monotonic_ns()
        output = []
        for interface, link_name in self.interfaces.items():
            root = Path('/sys/class/net') / interface
            current = {name: _number(root / 'statistics' / name) for name in COUNTERS}
            current.update(interface=interface, link_identifier=link_name)
            try:
                current['interface_state'] = (root / 'operstate').read_text().strip()
            except OSError:
                current['interface_state'] = 'unavailable'
            prior = self.previous.get(interface)
            if prior:
                seconds = (now_ns - prior['_time_ns']) / 1e9
                if seconds > 0:
                    for direction in ('rx', 'tx'):
                        key = f'{direction}_bytes'
                        if current[key] is not None and prior[key] is not None:
                            delta = max(0, current[key] - prior[key])
                            current[f'{direction}_interface_mbps'] = delta * 8.0 / seconds / 1e6
            current.update(self._iw(interface))
            self._label_radio_metrics(current, link_name)
            if link_name.startswith('halow'):
                current.update(self._morse_stats(interface, now_ns))
            self.previous[interface] = {**current, '_time_ns': now_ns}
            output.append(current)
        return output

    @staticmethod
    def _label_radio_metrics(values, link_name):
        """Separate Morse's nl80211 compatibility values from physical metrics."""
        if not link_name.startswith('halow'):
            return
        invalid_rssi = False
        for field in ('rssi_dbm', 'rssi_avg_dbm'):
            rssi = values.get(field)
            # -127 dBm is a common nl80211/driver "unknown" sentinel. Morse
            # HaLow can operate below ordinary Wi-Fi RSSI, but values below
            # -120 dBm or above 0 dBm are not defensible measurements.
            if rssi is not None and not -120.0 <= rssi <= 0.0:
                values.pop(field, None)
                invalid_rssi = True
        if invalid_rssi:
            values['rssi_state'] = 'saturated_or_invalid'
        for direction in ('tx', 'rx'):
            rate = values.pop(f'{direction}_bitrate_mbps', None)
            if rate is not None:
                values[f'{direction}_compatibility_rate_mbps'] = rate
        values['phy_reporting_mode'] = 'morse_s1g_nl80211_compatibility'
