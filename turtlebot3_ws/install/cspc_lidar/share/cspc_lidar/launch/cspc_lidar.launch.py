#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cspc_lidar')
    params_file = os.path.join(pkg_dir, 'params', 'cspc_lidar.yaml')

    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port connected to CSPC D4 LiDAR'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_scan',
            description='TF frame id for the LaserScan header'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS2 namespace'),

        Node(
            package='cspc_lidar',
            executable='cspc_lidar',
            name='cspc_lidar',
            namespace=namespace,
            parameters=[
                params_file,
                # Override yaml defaults with launch-time arguments so that
                # robot.launch.py can pass port/frame_id without editing the yaml.
                {'port': port, 'frame_id': frame_id},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
