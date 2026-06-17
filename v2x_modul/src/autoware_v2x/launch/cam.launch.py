import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('autoware_v2x'),
        'config',
        'cam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='autoware_v2x',
            executable='cam_read',
            name='cam_read_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='autoware_v2x',
            executable='cam_to_obj',
            name='cam_to_obj_node',
            output='screen',
        ),
    ])
