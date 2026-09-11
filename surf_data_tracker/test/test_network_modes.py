"""Readiness commands must preserve internet Wi-Fi and never step live clocks."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


DEPLOY = Path(__file__).resolve().parents[2] / 'deploy'


class NetworkModeTest(unittest.TestCase):
    def run_mode(self, jetson=False, good_route=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / 'calls'
            for name, body in {
                'ip': 'echo "10.43.30.1 dev halow src 10.43.30.2"',
                'chronyc': 'echo "chronyc $*" >> "$CALL_LOG"',
                'clock-sync-status': 'echo status >> "$CALL_LOG"',
                'nmcli': 'echo forbidden >> "$CALL_LOG"; exit 99',
                'systemctl': 'echo forbidden >> "$CALL_LOG"; exit 99',
            }.items():
                script = root / name
                script.write_text('#!/bin/bash\n' + body + '\n')
                script.chmod(0o755)
            prefix = 'SURF_JETSON_' if jetson else 'SURF_'
            config = root / 'config'
            config.write_text(f'{prefix}HALOW_PEER=10.43.30.1\n'
                              f'{prefix}HALOW_IF={"halow" if good_route else "wrong"}\n')
            env = dict(os.environ, PATH=f'{root}:/usr/bin:/bin', CALL_LOG=str(log))
            env['SURF_JETSON_NETWORK_CONFIG' if jetson else 'SURF_LOCAL_CONFIG'] = str(config)
            result = subprocess.run(
                ['bash', str(DEPLOY / ('jetson/surf-jetson-test-mode' if jetson
                                       else 'local/surf-test-mode'))],
                env=env, text=True, capture_output=True)
            return result, log.read_text() if log.exists() else ''

    def test_both_modes_only_check_route_and_clock(self):
        for jetson in (False, True):
            result, calls = self.run_mode(jetson)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(calls, 'chronyc waitsync 30 0.01 0 1\nstatus\n')

    def test_wrong_interface_fails_before_clock_check(self):
        for jetson in (False, True):
            result, calls = self.run_mode(jetson, False)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(calls, '')
