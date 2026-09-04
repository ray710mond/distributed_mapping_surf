"""Wire SURF mapping/comms into an existing robot sensing/localization graph."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name} must be true or false')
    return value == 'true'


def _required(mapping, name):
    value = mapping.get(name)
    if not isinstance(value, str) or not value.strip() or 'CHANGE_ME' in value:
        raise RuntimeError(f'integration.{name} must be configured')
    return value


def _node_parameters(document, node_name):
    section = document.get(f'/**/{node_name}', {})
    if not isinstance(section, dict):
        raise RuntimeError(f'/**/{node_name} must be a mapping')
    parameters = section.get('ros__parameters', {})
    if not isinstance(parameters, dict):
        raise RuntimeError(f'/**/{node_name}.ros__parameters must be a mapping')
    return parameters


def _setup(context):
    role = LaunchConfiguration('role').perform(context)
    if role not in ('drone', 'humanoid'):
        raise RuntimeError("role must be 'drone' or 'humanoid'")

    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f'config does not exist: {config_path}')
    document = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    integration = document.get('integration')
    if not isinstance(integration, dict):
        raise RuntimeError('config must contain an integration mapping')

    namespace = _required(integration, 'robot_namespace').strip('/')
    if not namespace or '/' in namespace:
        raise RuntimeError('integration.robot_namespace must be one namespace token')
    cloud_topic = _required(integration, 'pointcloud_topic')
    odometry_topic = _required(integration, 'odometry_topic')
    pose_source = _required(integration, 'pose_source')
    if pose_source not in ('odometry', 'tf'):
        raise RuntimeError('integration.pose_source must be odometry or tf')
    map_frame = _required(integration, 'map_frame')
    odometry_frame = _required(integration, 'odometry_frame')
    body_frame = _required(integration, 'body_frame')
    transport_config = Path(
        _required(integration, 'transport_config')).expanduser().resolve()
    if _as_bool(context, 'enable_transport') and not transport_config.is_file():
        raise RuntimeError(f'transport config does not exist: {transport_config}')

    map_path = Path(_required(integration, 'map_path')).expanduser().resolve()
    map_path.parent.mkdir(parents=True, exist_ok=True)
    telemetry_directory = Path(
        _required(integration, 'telemetry_directory')).expanduser().resolve()
    telemetry_directory.mkdir(parents=True, exist_ok=True)
    actions = []

    # Normalize robot-owned localization onto the existing SURF transport
    # contract without modifying the source odometry or robot-owned TF.
    actions.append(Node(
        package='surf_portable_bringup', executable='global_odometry',
        namespace=namespace, name='global_odometry', output='screen',
        parameters=[{
            'input_topic': odometry_topic,
            'output_topic': f'transport/{role}_odometry',
            'target_frame': map_frame,
            'output_child_frame': body_frame,
            'fallback_to_input_frame': False,
            'max_transform_wait_seconds': 0.5,
        }],
    ))

    if _as_bool(context, 'enable_mapping'):
        actions.append(Node(
            package='bonxai_ros', executable='bonxai_server_node',
            namespace=namespace, name='bonxai_server_node', output='screen',
            parameters=[_node_parameters(document, 'bonxai_server_node'), {
                'frame_id': map_frame,
                'base_frame_id': body_frame,
                'topic_in': cloud_topic,
                'delta_topic_in': (
                    f'/{namespace}/comm/drone_voxel_delta'
                    if role == 'humanoid' else ''),
                'map_storage.path': str(map_path),
                'map_storage.load_on_startup': map_path.is_file(),
            }],
        ))

    if role == 'drone':
        actions.append(Node(
            package='surf_drone', executable='drone_scan_sender',
            namespace=namespace, name='drone_scan_sender', output='screen',
            parameters=[_node_parameters(document, 'drone_scan_sender'), {
                'robot_name': namespace,
                'map_frame': map_frame,
                'input_topic': cloud_topic,
                'pose_source.type': pose_source,
                'pose_source.odometry_topic': odometry_topic,
                'pose_source.odometry_parent_frame': odometry_frame,
                'pose_source.base_frame': body_frame,
                'filters.humanoid_mask.odometry_topic': (
                    f'/{namespace}/transport/humanoid_odometry'),
            }],
        ))
    else:
        actions.append(Node(
            package='surf_humanoid', executable='drone_data_receiver',
            namespace=namespace, name='drone_data_receiver', output='screen',
            parameters=[_node_parameters(document, 'drone_data_receiver'), {
                'robot_name': namespace,
            }],
        ))

    if _as_bool(context, 'enable_transport'):
        for bridge_name in ('halow_bridge', 'wifi_bridge'):
            actions.append(Node(
                package='network_bridge', executable='network_bridge',
                namespace=namespace, name=bridge_name, output='screen',
                parameters=[str(transport_config), {
                    'subscribe_namespace': f'/{namespace}/transport',
                    'publish_namespace': f'/{namespace}/transport',
                }],
            ))
    if _as_bool(context, 'enable_tracker'):
        peer_role = 'humanoid' if role == 'drone' else 'drone'
        actions.append(Node(
            package='surf_data_tracker', executable='data_tracker',
            namespace=namespace, name='data_tracker', output='screen',
            parameters=[_node_parameters(document, 'data_tracker'), {
                'robot_role': role,
                'output_root': str(telemetry_directory),
                'pose_topics': [
                    f'{role}=/{namespace}/transport/{role}_odometry',
                    f'{peer_role}=/{namespace}/transport/{peer_role}_odometry',
                ],
                'cloud_topics': [
                    f'pointcloud=communication_input={cloud_topic}',
                    f'map=occupied=/{namespace}/bonxai/occupied_voxels',
                    f'map=static=/{namespace}/bonxai/static_occupied_voxels',
                ],
                'pipeline_topic': f'/{namespace}/comm/pipeline_metrics',
                'delivery_topic': f'/{namespace}/comm/delivery_metrics',
                'sync_status_topic': f'/{namespace}/comm/sync_status',
                'realtime_ack_metrics_topic': (
                    f'/{namespace}/comm/realtime_ack_metrics'),
            }],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('role'),
        DeclareLaunchArgument('config'),
        DeclareLaunchArgument('enable_mapping', default_value='true'),
        DeclareLaunchArgument('enable_transport', default_value='true'),
        DeclareLaunchArgument('enable_tracker', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
