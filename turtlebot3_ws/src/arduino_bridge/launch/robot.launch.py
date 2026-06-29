#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('arduino_bridge')
    lidar_pkg = get_package_share_directory('cspc_lidar')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port',   default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'cspc_lidar.launch.py')
            ),
            launch_arguments={
                'port':     LaunchConfiguration('lidar_port'),
                'frame_id': 'base_scan',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'arduino_bridge.launch.py')
            ),
            launch_arguments={
                'port': LaunchConfiguration('arduino_port'),
            }.items(),
        ),

        # Adjust x/y/z to match where the LiDAR is physically mounted on the robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_scan',
            arguments=['0.0', '0.0', '0.18', '0.0', '0.0', '0.0',
                       'base_footprint', 'base_scan'],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(pkg, 'param', 'slam_params.yaml'),
                {'use_sim_time': False},
            ],
            output='screen',
        ),
    ])
