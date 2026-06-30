#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('arduino_bridge')
    params_file = os.path.join(pkg_dir, 'param', 'arduino_bridge.yaml')

    port = LaunchConfiguration('port', default='/dev/ttyUSB1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB1',
                              description='USB serial port connected to Arduino'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),

        Node(
            package='arduino_bridge',
            executable='arduino_bridge_node',
            name='arduino_bridge',
            parameters=[
                params_file,
                {'port': port, 'use_sim_time': use_sim_time},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
