#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg       = get_package_share_directory('arduino_bridge')
    lidar_pkg = get_package_share_directory('cspc_lidar')
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )

    robot_description = Command(['xacro ', urdf_path, ' namespace:='])

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port',   default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB1'),

        # ── Robot description ────────────────────────────────────────────
        # Publishes /robot_description and all fixed-joint TF
        # (base_footprint → base_link → base_scan, imu_link, caster, etc.)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}],
        ),

        # Publishes zero wheel positions so robot_state_publisher
        # can resolve the continuous wheel joints (wheels won't animate
        # until arduino_bridge publishes real /joint_states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # ── LiDAR ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'cspc_lidar.launch.py')
            ),
            launch_arguments={
                'port':     LaunchConfiguration('lidar_port'),
                'frame_id': 'base_scan',
            }.items(),
        ),

        # ── Arduino bridge (odometry + cmd_vel) ──────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'arduino_bridge.launch.py')
            ),
            launch_arguments={
                'port': LaunchConfiguration('arduino_port'),
            }.items(),
        ),
    ])
