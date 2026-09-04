from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package='surf_data_tracker', executable='data_tracker', name='data_tracker',
        output='screen', parameters=[{'use_sim_time': False}],
    )])
