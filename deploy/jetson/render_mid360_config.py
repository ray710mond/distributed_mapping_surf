#!/usr/bin/env python3
import ipaddress
import json
import os
import sys
from pathlib import Path


def ipv4_from_environment(name, fallback):
    value = os.environ.get(name, fallback)
    try:
        address = ipaddress.ip_address(value)
    except ValueError as error:
        raise SystemExit(f'{name} is not a valid IP address: {value}') from error
    if address.version != 4:
        raise SystemExit(f'{name} must be an IPv4 address: {value}')
    return str(address)


def main():
    if len(sys.argv) != 3:
        raise SystemExit(
            'usage: render_mid360_config.py INPUT_JSON OUTPUT_JSON')

    input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])
    with input_path.open(encoding='utf-8') as stream:
        config = json.load(stream)

    lidar_configs = config.get('lidar_configs', [])
    if not lidar_configs:
        raise SystemExit('MID360 config must contain at least one lidar_configs entry')

    lidar_ip = ipv4_from_environment(
        'LIVOX_LIDAR_IP', lidar_configs[0].get('ip', '192.168.1.12'))
    host_config = config.get('MID360', {}).get('host_net_info')
    if host_config is None:
        raise SystemExit('MID360 config is missing MID360.host_net_info')

    if isinstance(host_config, dict):
        fallback_host_ip = host_config.get('point_data_ip', '192.168.1.5')
    elif isinstance(host_config, list) and host_config:
        fallback_host_ip = host_config[0].get('host_ip', '192.168.1.5')
    else:
        raise SystemExit('MID360.host_net_info must be an object or non-empty list')
    host_ip = ipv4_from_environment('LIVOX_HOST_IP', fallback_host_ip)

    lidar_configs[0]['ip'] = lidar_ip
    if isinstance(host_config, dict):
        for key in (
                'cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            host_config[key] = host_ip
    else:
        host_config[0]['host_ip'] = host_ip
        host_config[0]['lidar_ip'] = [lidar_ip]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as stream:
        json.dump(config, stream, indent=2)
        stream.write('\n')

    print(f'MID360 network: lidar={lidar_ip}, host={host_ip}')


if __name__ == '__main__':
    main()
