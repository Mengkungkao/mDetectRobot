import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('cspc_lidar')
    parameter_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share_dir, 'params', 'cspc_lidar.yaml')),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                share_dir, 'rviz', 'cspc_lidar.rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_dir, 'launch', 'lidar_launch.py')),
            launch_arguments={'params_file': parameter_file}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='cspc_lidar_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
