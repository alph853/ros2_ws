from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


param_file = os.path.join(
    get_package_share_directory('distance_warning'),
    'config',
    'thresholds.yaml'
)


def generate_launch_description():

    return LaunchDescription([
        # Node(
        #     package='distance_warning',
        #     executable='set_threshold_service',
        #     name='set_threshold_service',
        #     parameters=[param_file],
        # ),
        Node(
            package='distance_warning',
            executable='distance_listener',
            name='distance_listener',
            parameters=[param_file],
        ),
        Node(
            package='distance_warning',
            executable='distance_publisher',
            name='distance_publisher',
            parameters=[param_file],
        ),
        Node(
            package='distance_warning',
            executable='distance_action_server',
            name='distance_action_server',
            parameters=[param_file],
        ),
        Node(
            package='distance_warning',
            executable='distance_action_client',
            name='distance_action_client',
            parameters=[param_file],
        ),
    ])
