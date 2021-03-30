import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    dir = get_package_share_directory('rostron_ia_ms')

    ns = LaunchConfiguration('team')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'team',
            default_value='yellow',
            description='Namespace teams'),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(dir, 'config', 'manager.yaml'),
            description='Configuration files for rostron'),
        PushRosNamespace(namespace=ns),
        Node(
            package='rostron_ia_ms',
            executable='exemple',
            parameters=[config],
            output='screen'
        )
    ])
