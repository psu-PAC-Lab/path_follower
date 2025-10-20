import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('path_follower'),
        'config',
        'path_follower_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='path_follower',
            executable='path_follower',
            name='path_follower',
            namespace='control',
            output='screen',
            parameters=[config]
        )
    ])