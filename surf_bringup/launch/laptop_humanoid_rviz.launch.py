import ipaddress
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


DEFAULT_MAP_DIRECTORY = os.environ.get(
    'SURF_MAP_DIRECTORY', os.path.expanduser('~/SURF_2026/maps'))
DEFAULT_MAP_NAME = os.environ.get('SURF_MAP_NAME', 'amberlab')


def _ipv4(context, argument):
    value = LaunchConfiguration(argument).perform(context)
    try:
        address = ipaddress.ip_address(value)
    except ValueError as error:
        raise RuntimeError(f'{argument} is not a valid IP address: {value}') from error
    if address.version != 4:
        raise RuntimeError(f'{argument} must be an IPv4 address: {value}')
    return str(address)


def _launch_humanoid(context):
    share = get_package_share_directory('surf_bringup')
    host_ip = _ipv4(context, 'livox_host_ip')
    lidar_ip = _ipv4(context, 'livox_lidar_ip')
    template_path = os.path.join(share, 'config', 'MID360_config.json')
    with open(template_path, encoding='utf-8') as stream:
        config = json.load(stream)

    config['lidar_configs'][0]['ip'] = lidar_ip
    host_config = config['MID360']['host_net_info']
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        host_config[key] = host_ip

    runtime_path = os.path.join(
        tempfile.gettempdir(), 'surf_mid360_humanoid.json')
    with open(runtime_path, 'w', encoding='utf-8') as stream:
        json.dump(config, stream, indent=2)
        stream.write('\n')

    passthrough = (
        'map_directory', 'map_name', 'map_voxel_resolution', 'map_chunk_size',
        'map_2d_resolution', 'prepare_map_on_startup', 'save_map_on_shutdown',
        'caltech_nearest_chunk_count',
        'caltech_initial_z', 'caltech_reload_distance', 'tracker_run_id',
        'tracker_experiment_name', 'tracker_notes', 'tracker_clock_sync_method',
        'tracker_clock_offset_ms', 'tracker_clock_uncertainty_ms',
        'tracker_record_raw_events', 'tracker_timeseries_window_s',
        'tracker_network_stats_rate_hz',
    )
    launch_arguments = {
        name: LaunchConfiguration(name) for name in passthrough
    }
    launch_arguments.update({
        'role': 'humanoid',
        'livox_config': runtime_path,
        'launch_rviz': 'true',
    })
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware.launch.py')),
        launch_arguments=launch_arguments.items(),
    )]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'livox_host_ip',
            default_value=os.environ.get('LIVOX_HOST_IP', '192.168.0.5'),
            description='Laptop Ethernet address on the MID-360 subnet.'),
        DeclareLaunchArgument(
            'livox_lidar_ip',
            default_value=os.environ.get('LIVOX_LIDAR_IP', '192.168.0.120'),
            description='Humanoid MID-360 address.'),
        DeclareLaunchArgument('map_directory', default_value=DEFAULT_MAP_DIRECTORY),
        DeclareLaunchArgument('map_name', default_value=DEFAULT_MAP_NAME),
        DeclareLaunchArgument('map_voxel_resolution', default_value='0.05'),
        DeclareLaunchArgument('map_chunk_size', default_value='10.0'),
        DeclareLaunchArgument('map_2d_resolution', default_value='0.20'),
        DeclareLaunchArgument('prepare_map_on_startup', default_value='true'),
        DeclareLaunchArgument('save_map_on_shutdown', default_value='true'),
        DeclareLaunchArgument('caltech_nearest_chunk_count', default_value='25'),
        DeclareLaunchArgument('caltech_initial_z', default_value='0.0'),
        DeclareLaunchArgument('caltech_reload_distance', default_value='2.5'),
        DeclareLaunchArgument('tracker_run_id', default_value=''),
        DeclareLaunchArgument('tracker_experiment_name', default_value=''),
        DeclareLaunchArgument('tracker_notes', default_value=''),
        DeclareLaunchArgument('tracker_clock_sync_method', default_value='unverified'),
        DeclareLaunchArgument('tracker_clock_offset_ms', default_value='nan'),
        DeclareLaunchArgument('tracker_clock_uncertainty_ms', default_value='nan'),
        DeclareLaunchArgument('tracker_record_raw_events', default_value='false'),
        DeclareLaunchArgument('tracker_timeseries_window_s', default_value='1.0'),
        DeclareLaunchArgument('tracker_network_stats_rate_hz', default_value='2.0'),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_humanoid)])
