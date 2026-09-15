import json
import os
import re
import subprocess

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_MAP_DIRECTORY = os.environ.get(
    'SURF_MAP_DIRECTORY', os.path.expanduser('~/SURF_2026/maps'))
DEFAULT_MAP_NAME = os.environ.get('SURF_MAP_NAME', 'amberlab')
TELEMETRY_DIRECTORY = '/data/telemetry'


def _frame(robot_namespace, frame):
    """Return a globally unique TF frame owned by a robot namespace."""
    return f'{robot_namespace}/{frame}'


def _package_executable(package, executable):
    path = os.path.join(
        get_package_prefix(package), 'lib', package, executable)
    if not os.path.isfile(path) or not os.access(path, os.X_OK):
        raise RuntimeError(
            f'Required executable is not installed: {path}. Rebuild the '
            'workspace with the caltech_mapping submodule initialized.')
    return path


def _boolean_launch_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('1', 'true', 'yes', 'on', '0', 'false', 'no', 'off'):
        raise RuntimeError(
            f'{name} must be a boolean value, got {value!r}')
    return value in ('1', 'true', 'yes', 'on')


def _prepare_environment_map(context, map_directory, map_name):
    command = [
        _package_executable('surf_bringup', 'prepare_map'),
        '--map-root', map_directory,
        '--map-name', map_name,
        '--bonxai-converter',
        _package_executable(
            'caltech_bonxai_builder', 'bonxai_pcd_converter'),
        '--chunk-converter',
        _package_executable(
            'caltech_bonxai_builder', 'pcd_to_rolling_bonxai'),
        '--occupancy-converter',
        _package_executable(
            'caltech_bonxai_builder', 'pcd_to_ros_occupancy.py'),
        '--voxel-resolution',
        LaunchConfiguration('map_voxel_resolution').perform(context),
        '--chunk-size',
        LaunchConfiguration('map_chunk_size').perform(context),
        '--map-2d-resolution',
        LaunchConfiguration('map_2d_resolution').perform(context),
    ]
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f'Map preparation failed for {map_name!r}; see converter output '
            'above for the invalid or incomplete artifact.')
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            'Map preparation returned an invalid manifest: '
            f'{result.stdout!r}') from error


def _load_prepared_environment_map(map_directory, map_name):
    environment = os.path.join(map_directory, map_name)
    stem = os.path.join(environment, map_name)
    paths = {
        'pcd_path': stem + '.pcd',
        'bonxai_path': stem + '.bonxai',
        'map_yaml': stem + '.yaml',
        'chunk_directory': stem + '_chunks',
    }
    missing = [
        path for key, path in paths.items()
        if not (
            os.path.isdir(path) if key == 'chunk_directory'
            else os.path.isfile(path) and os.path.getsize(path) > 0)
    ]
    if missing:
        raise RuntimeError(
            'prepare_map_on_startup is false, but the prepared map is missing: '
            + ', '.join(missing))
    return {
        'map_name': map_name,
        'map_directory': environment,
        'localization_enabled': True,
        **paths,
    }


def _nodes(context):
    role = LaunchConfiguration('role').perform(context)
    telemetry_directory = (
        TELEMETRY_DIRECTORY
        if os.path.isdir('/data') and os.access('/data', os.W_OK) else '')
    if role not in ('drone', 'humanoid'):
        raise RuntimeError("role must be 'drone' or 'humanoid'")
    imu_frame = _frame(role, 'body')
    tracking_frame = _frame(role, 'body')

    map_directory = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('map_directory').perform(context)))
    map_name = LaunchConfiguration('map_name').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', map_name):
        raise RuntimeError(
            'map_name must start with an alphanumeric character and contain '
            'only letters, numbers, underscores, and hyphens')
    prepare_map_on_startup = _boolean_launch_argument(
        context, 'prepare_map_on_startup')
    map_manifest = (
        _prepare_environment_map(context, map_directory, map_name)
        if prepare_map_on_startup
        else _load_prepared_environment_map(map_directory, map_name))
    voxel_resolution = float(
        LaunchConfiguration('map_voxel_resolution').perform(context))
    environment_map_directory = map_manifest['map_directory']
    localization_map = map_manifest['pcd_path']
    map_path = map_manifest['bonxai_path']
    localization_enabled = map_manifest['localization_enabled']
    save_map_on_shutdown = _boolean_launch_argument(
        context, 'save_map_on_shutdown')
    odom_frame = _frame(role, 'odom')
    mapping_frame = 'map' if localization_enabled else odom_frame
    initial_pose_topic = (
        f'/{role}/transport/' +
        ('drone_initialpose' if role == 'drone' else 'initialpose'))

    bringup_share = get_package_share_directory('surf_bringup')
    role_share = get_package_share_directory(
        'surf_drone' if role == 'drone' else 'surf_humanoid')
    container_transport = f'/config/{role}_transport.yaml'
    transport_file = (
        container_transport if os.path.isfile(container_transport) else
        os.path.join(bringup_share, 'config', f'{role}_transport.yaml'))
    registered_cloud = 'points'
    body_cloud = 'points_body'
    mapping_base_frame = tracking_frame
    launch_rviz = (
        LaunchConfiguration('launch_rviz').perform(context).lower()
        in ('1', 'true', 'yes', 'on'))

    actions = []
    if localization_enabled:
        actions.extend([
            LogInfo(msg=(
                f'Using reference map {localization_map}; fixed-map '
                'localization is enabled automatically.')),
            Node(
                package='nav2_map_server',
                executable='map_server',
                namespace=role,
                name='map_server',
                output='screen',
                parameters=[{
                    'yaml_filename': map_manifest['map_yaml'],
                    # The live projector owns /map and overlays this trusted
                    # reference with current Bonxai observations.
                    'topic_name': '/map/reference',
                    'frame_id': 'map',
                }],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace=role,
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
            ),
        ])
        if role == 'humanoid':
            actions.append(Node(
                package='caltech_bonxai_builder',
                executable='initial_pose_chunk_loader',
                namespace=role,
                name='reference_chunk_loader',
                output='screen',
                parameters=[{
                    'map_directory': map_manifest['chunk_directory'],
                    'map_frame': 'map',
                    'odom_frame': odom_frame,
                    'base_frame': tracking_frame,
                    'follow_tf_only': True,
                    'nearest_chunk_count': int(
                        LaunchConfiguration(
                            'caltech_nearest_chunk_count').perform(context)),
                    'chunk_size': float(
                        LaunchConfiguration('map_chunk_size').perform(context)),
                    'reload_distance': float(
                        LaunchConfiguration(
                            'caltech_reload_distance').perform(context)),
                    'voxel_topic': 'reference_map/occupied_voxels',
                    'marker_topic': 'reference_map/loaded_chunks',
                    'publish_voxels': True,
                }],
            ))
    else:
        actions.append(LogInfo(msg=(
            f'No usable reference cloud exists in '
            f'{environment_map_directory}. Collecting a {role} map with '
            f'FAST-LIO in {mapping_frame}; ScanLock and GLIM are disabled. Collaborative '
            'transmission and fusion are disabled. A graceful shutdown saves '
            'the mapping artifacts for the next launch.')))
    livox_config = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('livox_config').perform(context)))
    if not os.path.isfile(livox_config):
        raise RuntimeError(
            f'Livox config does not exist or is not a file: {livox_config}')
    actions.append(Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        namespace=role,
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': _frame(role, 'livox_frame'),
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        remappings=[
            ('livox/lidar', 'livox/lidar'),
            ('livox/imu', 'livox/imu'),
        ],
    ))

    lio_parameters = [
        os.path.join(bringup_share, 'config', 'mid360_lio.yaml'),
        os.path.join(bringup_share, 'config', f'{role}_mid360.yaml'),
    ]
    lio_parameters.append({
        'common.lid_topic': 'livox/lidar',
        'common.imu_topic': 'livox/imu',
        'common.odom_frame': odom_frame,
        'common.imu_frame': imu_frame,
    })
    # FAST-LIO is the continuous high-rate odometry source in every mode.
    if True:
        actions.append(Node(
            package='fast_lio',
            executable='fastlio_mapping',
            namespace=role,
            name='fastlio',
            output='screen',
            parameters=[
                *lio_parameters,
                {
                    'use_sim_time': False,
                    'pcd_save.pcd_save_en': False,
                    'publish.scan_bodyframe_pub_en': True,
                },
            ],
            remappings=[
                ('/cloud_registered', 'points'),
                ('/cloud_registered_body', 'points_body'),
                ('/laser_map', 'laser_map'),
                ('/odometry', 'odom'),
                ('/path', 'path'),
            ],
        ))
        actions.append(Node(
            package='surf_bringup',
            executable='live_occupancy_projector',
            namespace=role,
            name='live_occupancy_projector',
            output='screen',
            parameters=[{
                'map_frame': mapping_frame,
                'resolution': float(LaunchConfiguration(
                    'map_2d_resolution').perform(context)),
                'reference_topic': '/map/reference',
                'output_topic': '/map',
                'min_z': float(LaunchConfiguration(
                    'live_map_min_z').perform(context)),
                'max_z': float(LaunchConfiguration(
                    'live_map_max_z').perform(context)),
                'update_period_s': float(LaunchConfiguration(
                    'live_map_update_period_s').perform(context)),
            }],
        ))

    if localization_enabled:
        actions.append(Node(
            package='surf_bringup',
            executable='localization_overlap_monitor',
            namespace=role,
            name='localization_overlap_monitor',
            output='screen',
            parameters=[{
                # In a first-run session the supervisor creates this path;
                # the monitor notices and loads it without being restarted.
                'reference_pcd': localization_map,
                'cloud_topic': registered_cloud,
                'map_frame': mapping_frame,
                'body_frame': tracking_frame,
                'bounds_margin': float(LaunchConfiguration(
                    'scanlock_bounds_margin').perform(context)),
                'status_topic': 'localization/status',
                'reference_path_topic': 'localization/reference_path',
            }],
        ))
    if localization_enabled:
        actions.append(Node(
            package='surf_bringup',
            executable='slam_to_scanlock_supervisor',
            namespace=role,
            name='slam_to_scanlock_supervisor',
            output='screen',
            parameters=[{
                'role': role,
                'pcd_path': localization_map,
                'initial_reference_available': localization_enabled,
                'scan_lock_executable': _package_executable(
                    'scan_lock', 'scan_lock_node'),
                'scan_lock_config': os.path.join(
                    bringup_share, 'config', 'scan_lock.yaml'),
                'odom_frame': odom_frame,
                'body_frame': tracking_frame,
                'imu_frame': imu_frame,
                'map_frame': mapping_frame,
                'initialpose_topic': initial_pose_topic,
                'initial_guess_z': float(LaunchConfiguration(
                    'caltech_initial_z').perform(context)),
                'handoff.exit_overlap': float(LaunchConfiguration(
                    'scanlock_exit_overlap').perform(context)),
                'handoff.consecutive_exit_updates': int(LaunchConfiguration(
                    'scanlock_exit_updates').perform(context)),
            }],
        ))
    if True:
        actions.append(Node(
            package='bonxai_ros',
            executable='bonxai_server_node',
            namespace=role,
            name='bonxai_server_node',
            output='screen',
            parameters=[
                os.path.join(bringup_share, 'config', 'bonxai.yaml'),
                {
                    'frame_id': mapping_frame,
                    'topic_in': body_cloud,
                    'base_frame_id': mapping_base_frame,
                    'delta_topic_in': (
                        'comm/drone_voxel_delta'
                        if role == 'humanoid' and localization_enabled else ''),
                    'fusion.require_both_localized': True,
                    'occupancy.static_resolution': voxel_resolution,
                    'occupancy.dynamic_resolution': voxel_resolution,
                    # A first-run map must accumulate the explored route in
                    # the layer written on shutdown. Localized operation keeps
                    # the existing dynamic-obstacle filtering unchanged.
                    'dynamic_obstacles.enabled': localization_enabled,
                    'map_storage.path': map_path,
                    'map_storage.pcd_path': localization_map,
                    'map_storage.load_on_startup': os.path.isfile(map_path),
                    'map_storage.save_on_shutdown': save_map_on_shutdown,
                },
            ],
        ))

    actions.extend([
        Node(
            package='surf_portable_bringup',
            executable='global_odometry',
            namespace=role,
            name='global_odometry',
            output='screen',
            parameters=[{
                'input_topic': 'odom',
                'output_topic': f'transport/{role}_odometry',
                # With no reference, this remains FAST-LIO's odom frame. A
                # known PCD supplies the shared map frame through ScanLock.
                'target_frame': mapping_frame,
                'fallback_to_input_frame': not localization_enabled,
                'output_child_frame': tracking_frame,
                # ScanLock TF can trail FAST-LIO odometry by roughly one
                # update; global_odometry buffers rather than dropping it.
                'max_transform_wait_seconds': 0.5,
            }],
        ),
        Node(
            package='surf_data_tracker',
            executable='data_tracker',
            namespace=role,
            name='data_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_role': role,
                'run_id': LaunchConfiguration('tracker_run_id'),
                'experiment_name': LaunchConfiguration('tracker_experiment_name'),
                'notes': LaunchConfiguration('tracker_notes'),
                'clock_sync_method': LaunchConfiguration('tracker_clock_sync_method'),
                'clock_offset_ms': ParameterValue(
                    LaunchConfiguration('tracker_clock_offset_ms'), value_type=float),
                'clock_uncertainty_ms': ParameterValue(
                    LaunchConfiguration('tracker_clock_uncertainty_ms'), value_type=float),
                'record_raw_events': ParameterValue(
                    LaunchConfiguration('tracker_record_raw_events'), value_type=bool),
                'timeseries_window_s': ParameterValue(
                    LaunchConfiguration('tracker_timeseries_window_s'), value_type=float),
                'network_stats_rate_hz': ParameterValue(
                    LaunchConfiguration('tracker_network_stats_rate_hz'), value_type=float),
                'output_root': telemetry_directory,
            }],
        ),
        Node(
            package='surf_data_tracker', executable='halow_capacity_provider',
            name='halow_capacity_provider', output='screen',
            parameters=[{'interface': LaunchConfiguration('capacity_interface').perform(context) or
                         ('wlx0cbf7400343c' if role == 'drone' else 'wlx0cbf740035d4'),
                         'experimental_mmrc_factor': ParameterValue(LaunchConfiguration('capacity_mmrc_factor'), value_type=float)}],
        ),
        Node(
            package='network_bridge', executable='network_bridge', namespace=role,
            name='halow_control_bridge', output='screen', parameters=[transport_file],
        ),
        Node(
            package='network_bridge',
            executable='network_bridge',
            namespace=role,
            name='halow_bridge',
            output='screen',
            parameters=[transport_file],
        ),

    ])

    if _boolean_launch_argument(context, 'clock_guard_enabled'):
        peer = 'humanoid' if role == 'drone' else 'drone'
        clock_guard = Node(
            package='surf_bringup',
            executable='peer_clock_guard',
            namespace=role,
            name='peer_clock_guard',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'peer_odometry_topic': f'/{role}/transport/{peer}_odometry',
                'localization_status_topic': 'localization/promotion_status',
                'maximum_skew_seconds': ParameterValue(
                    LaunchConfiguration('clock_guard_maximum_skew_seconds'),
                    value_type=float),
                'startup_timeout_seconds': ParameterValue(
                    LaunchConfiguration('clock_guard_startup_timeout_seconds'),
                    value_type=float),
            }],
        )
        actions.append(clock_guard)

    if launch_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(
                    bringup_share, 'rviz', 'humanoid_map.rviz'),
            ],
        ))

    if role == 'drone' and localization_enabled:
        actions.append(Node(
            package='surf_drone',
            executable='drone_scan_sender',
            namespace=role,
            name='drone_scan_sender',
            output='screen',
            parameters=[
                os.path.join(role_share, 'config', 'drone.yaml'),
                {
                    'input_topic': body_cloud,
                    'resolution': voxel_resolution,
                },
            ],
        ))
    elif role == 'humanoid':
        actions.append(Node(
            package='surf_humanoid',
            executable='drone_data_receiver',
            namespace=role,
            name='drone_data_receiver',
            output='screen',
            parameters=[os.path.join(role_share, 'config', 'humanoid.yaml')],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'role',
            description="This computer's role: drone or humanoid"),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.environ.get(
                'LIVOX_CONFIG_PATH', '/run/surf/MID360_config.json'),
            description='Rendered Livox MID-360 driver JSON for this host.'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz for laptop testing (leave false on robots)'),
        DeclareLaunchArgument(
            'clock_guard_enabled',
            default_value='true',
            description=(
                'Report apparent peer odometry timestamp age and missing samples.')),
        DeclareLaunchArgument(
            'clock_guard_maximum_skew_seconds',
            default_value='1.0',
            description='Apparent peer odometry age above which to warn.'),
        DeclareLaunchArgument(
            'clock_guard_startup_timeout_seconds',
            default_value='10.0',
            description=(
                'Seconds to wait for timely peer odometry after local '
                'localization becomes active. Time before initial localization '
                'does not count.')),
        DeclareLaunchArgument(
            'caltech_nearest_chunk_count',
            default_value='25',
            description='Number of nearest Caltech Bonxai chunks to load.'),
        DeclareLaunchArgument(
            'caltech_initial_z',
            default_value='0.0',
            description=(
                'Robot body height above the PCD ground estimate when a 2D '
                'initial pose is provided.')),
        DeclareLaunchArgument(
            'caltech_reload_distance',
            default_value='2.5',
            description='Travel distance before Caltech chunks are reloaded.'),
        DeclareLaunchArgument(
            'map_directory',
            default_value=DEFAULT_MAP_DIRECTORY,
            description=(
                'Root directory containing named environment map folders.')),
        DeclareLaunchArgument(
            'map_name',
            default_value=DEFAULT_MAP_NAME,
            description=(
                'Environment name; maps are read from and saved to '
                '<map_directory>/<map_name>/<map_name>.*.')),
        DeclareLaunchArgument(
            'map_voxel_resolution',
            default_value='0.05',
            description=(
                'Voxel resolution used when filling Bonxai and chunk '
                'artifacts from a PCD.')),
        DeclareLaunchArgument(
            'map_chunk_size',
            default_value='10.0',
            description='Rolling-Bonxai chunk width in meters.'),
        DeclareLaunchArgument(
            'map_2d_resolution',
            default_value='0.20',
            description='Resolution of the generated /map occupancy grid.'),
        DeclareLaunchArgument(
            'live_map_min_z', default_value='0.25',
            description='Lowest Bonxai voxel projected into the live 2D map.'),
        DeclareLaunchArgument(
            'live_map_max_z', default_value='2.0',
            description='Highest Bonxai voxel projected into the live 2D map.'),
        DeclareLaunchArgument(
            'live_map_update_period_s', default_value='1.0',
            description='Period between live Bonxai-to-2D projections.'),
        DeclareLaunchArgument(
            'prepare_map_on_startup',
            default_value='true',
            description=(
                'Validate and regenerate map derivatives at startup. Set '
                'false to load an already prepared map without conversion.')),
        DeclareLaunchArgument(
            'save_map_on_shutdown',
            default_value='true',
            description=(
                'Save the live Bonxai map and localization PCD on shutdown. '
                'Set false to keep a prepared reference map immutable.')),
        DeclareLaunchArgument(
            'scanlock_exit_overlap', default_value='0.50',
            description=(
                'Reference overlap below which ScanLock is stopped after it '
                'has anchored successfully.')),
        DeclareLaunchArgument(
            'scanlock_exit_updates', default_value='5',
            description=(
                'Consecutive low-overlap updates required before handing off '
                'permanently to FAST-LIO.')),
        DeclareLaunchArgument(
            'scanlock_bounds_margin', default_value='0.0',
            description='XY margin around the original PCD bounds.'),
        DeclareLaunchArgument(
            'tracker_run_id', default_value='',
            description=(
                'Shared experiment run ID; use the same value on the drone '
                'Jetson and humanoid laptop.')),
        DeclareLaunchArgument(
            'tracker_experiment_name', default_value='',
            description='Human-readable experiment name stored with telemetry.'),
        DeclareLaunchArgument(
            'tracker_notes', default_value='',
            description='Optional experiment notes stored verbatim.'),
        DeclareLaunchArgument('capacity_interface', default_value=''),
        DeclareLaunchArgument('capacity_mmrc_factor', default_value='0.1'),
        DeclareLaunchArgument(
            'tracker_clock_sync_method', default_value='unverified',
            description='Verified clock synchronization method, or unverified.'),
        DeclareLaunchArgument(
            'tracker_clock_offset_ms', default_value='nan',
            description='Measured local clock offset in milliseconds.'),
        DeclareLaunchArgument(
            'tracker_clock_uncertainty_ms', default_value='nan',
            description='Measured local clock uncertainty in milliseconds.'),
        DeclareLaunchArgument(
            'tracker_record_raw_events', default_value='false',
            description='Export raw events during post-run analysis when requested.'),
        DeclareLaunchArgument(
            'tracker_timeseries_window_s', default_value='1.0',
            description='Preferred offline aggregation window in seconds.'),
        DeclareLaunchArgument(
            'tracker_network_stats_rate_hz', default_value='2.0',
            description='Low-overhead Linux wireless/interface polling frequency.'),
        OpaqueFunction(function=_nodes),
    ])
