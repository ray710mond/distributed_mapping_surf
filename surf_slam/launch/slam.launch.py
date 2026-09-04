import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from surf_slam.configuration import render_glim_config


def _nodes(context):
    role = LaunchConfiguration('role').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    points_topic = 'livox/points'
    runtime_config = render_glim_config(
        os.path.join(get_package_share_directory('glim'), 'config'),
        os.path.join('/tmp', 'surf_slam', role),
        role=role,
        imu_topic=imu_topic,
        points_topic=points_topic,
    )
    dump_path = os.path.join(
        os.path.abspath(os.path.expanduser(
            LaunchConfiguration('map_directory').perform(context))),
        LaunchConfiguration('map_name').perform(context),
        'glim_session',
    )
    os.makedirs(os.path.dirname(dump_path), exist_ok=True)

    actions = [LogInfo(msg=(
        f'Starting GLIM pose-graph SLAM for {role}; session output will be '
        f'saved to {dump_path} on graceful shutdown.'))]
    actions.append(Node(
        package='surf_slam',
        executable='livox_custom_to_pc2',
        namespace=role,
        name='livox_custom_to_pc2',
        output='screen',
        parameters=[{
            'input_topic': 'livox/lidar',
            'output_topic': points_topic,
            # GLIM's TF subtree is deliberately isolated from FAST-LIO's.
            'frame_id': f'{role}/glim_livox_frame',
        }],
    ))
    actions.append(Node(
        package='glim_ros',
        executable='glim_rosnode',
        namespace=role,
        name='glim',
        output='screen',
        parameters=[{
            'config_path': runtime_config,
            'dump_path': dump_path,
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('role'),
        DeclareLaunchArgument('imu_topic'),
        DeclareLaunchArgument('map_directory'),
        DeclareLaunchArgument('map_name'),
        OpaqueFunction(function=_nodes),
    ])
