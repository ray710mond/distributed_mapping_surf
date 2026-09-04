"""Retry-safe post-run collection onto the analysis laptop."""

import argparse
import json
import subprocess
from pathlib import Path

from .storage import validate_run_id


def collect(run_id, destination, sources):
    validate_run_id(run_id)
    destination = Path(destination).expanduser().resolve() / run_id / 'hosts'
    destination.mkdir(parents=True, exist_ok=True)
    results = []
    for source in sources:
        # source is deliberately configuration, not a repository-hardcoded host.
        if '=' not in source:
            results.append({'source': source, 'ok': False, 'error': 'expected name=user@host:/path'})
            continue
        name, remote_root = source.split('=', 1)
        # Replace changed host files when a run ID was deliberately reused;
        # unrelated host directories already collected from another source stay.
        command = ['rsync', '-a', '--partial',
                   f'{remote_root.rstrip("/")}/{run_id}/hosts/', str(destination) + '/']
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        results.append({'source': name, 'remote': remote_root, 'ok': result.returncode == 0,
                        'returncode': result.returncode, 'error': result.stderr.strip()})
    report = {'run_id': run_id, 'destination': str(destination.parent), 'sources': results,
              'all_succeeded': all(item['ok'] for item in results)}
    (destination.parent / 'collection.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description='Collect host-local SURF telemetry with rsync.')
    parser.add_argument('run_id')
    parser.add_argument('--destination', type=Path, default=Path('experiments'))
    parser.add_argument('--source', action='append', required=True,
                        help='Repeatable name=user@host:/remote/experiment/root')
    args = parser.parse_args()
    report = collect(args.run_id, args.destination, args.source)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['all_succeeded'] else 2)


if __name__ == '__main__':
    main()
