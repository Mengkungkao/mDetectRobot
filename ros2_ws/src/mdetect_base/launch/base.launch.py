from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('mdetect_base'), 'config', 'base.yaml')

    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='mdetect_base',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='mdetect_base',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='mdetect_base',
            executable='timed_waypoint_executor',
            name='timed_waypoint_executor',
            output='screen',
            parameters=[config],
        ),
    ])
