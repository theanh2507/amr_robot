import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('plicp_map_matcher'),
        'config', 'plicp_map_matcher_params.yaml')

    return LaunchDescription([
        Node(
            package='plicp_map_matcher',
            executable='plicp_map_matcher',
            name='plicp_map_matcher',
            output='screen',
            parameters=[params_file],
        ),
    ])
