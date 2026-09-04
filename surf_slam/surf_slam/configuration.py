"""Render a role-specific GLIM configuration from the vendored defaults."""

import json
from pathlib import Path
import shutil


_COPIED_CONFIGS = (
    'config_logging.json',
    'config_preprocess.json',
    'config_odometry_cpu.json',
    'config_sub_mapping_passthrough.json',
    'config_global_mapping_pose_graph.json',
)


def render_glim_config(
        source_directory, target_directory, *, role, imu_topic, points_topic):
    """Create a headless CPU pose-graph configuration and return its path."""
    if role not in ('drone', 'humanoid'):
        raise ValueError("role must be 'drone' or 'humanoid'")
    source = Path(source_directory)
    target = Path(target_directory)
    target.mkdir(parents=True, exist_ok=True)
    for filename in _COPIED_CONFIGS:
        shutil.copy2(source / filename, target / filename)

    global_config = {
        'global': {
            'config_path': '',
            'config_ros': 'config_ros.json',
            'config_logging': 'config_logging.json',
            'config_sensors': 'config_sensors.json',
            'config_preprocess': 'config_preprocess.json',
            'config_odometry': 'config_odometry_cpu.json',
            'config_sub_mapping': 'config_sub_mapping_passthrough.json',
            'config_global_mapping': 'config_global_mapping_pose_graph.json',
        },
    }
    ros_config = {
        'glim_ros': {
            'enable_local_mapping': True,
            'enable_global_mapping': True,
            'keep_raw_points': False,
            'imu_time_offset': 0.0,
            'points_time_offset': 0.0,
            'acc_scale': 0.0,
            'ang_scale': 1.0,
            'imu_frame_id': f'{role}/glim_body',
            'lidar_frame_id': f'{role}/glim_livox_frame',
            # GLIM may run beside FAST-LIO during first-run mapping. Keep its
            # complete TF subtree private so FAST-LIO remains the sole owner
            # of the robot's odom -> body transform.
            'base_frame_id': f'{role}/glim_body',
            'odom_frame_id': f'{role}/glim_odom',
            'map_frame_id': f'{role}/map',
            'publish_imu2lidar': True,
            'tf_time_offset': 1e-6,
            # This ROS extension publishes TF, poses, registered clouds, and
            # the transient-local global cloud without opening a GUI.
            'extension_modules': ['librviz_viewer.so'],
            'imu_topic': imu_topic,
            'points_topic': points_topic,
            'image_topic': '',
            'imu_qos': {'profile': 'sensor_data', 'depth': 1000},
            'points_qos': {'profile': 'sensor_data'},
            'image_qos': {'profile': 'sensor_data'},
        },
    }
    sensor_config = {
        'sensors': {
            'imu_acc_noise': 0.05,
            'imu_gyro_noise': 0.02,
            'imu_int_noise': 0.001,
            'imu_bias_noise': 1e-5,
            'global_shutter_lidar': False,
            'T_lidar_imu': [-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0, 1.0],
            'intensity_field': 'intensity',
            'ring_field': '',
            'autoconf_perpoint_times': True,
            'autoconf_prefer_frame_time': False,
            'perpoint_relative_time': True,
            'perpoint_time_scale': 1.0,
        },
    }

    for filename, value in (
            ('config.json', global_config),
            ('config_ros.json', ros_config),
            ('config_sensors.json', sensor_config)):
        (target / filename).write_text(
            json.dumps(value, indent=2) + '\n', encoding='utf-8')
    return str(target)
