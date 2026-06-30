#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg          = get_package_share_directory('arduino_bridge')
    slam_pkg     = get_package_share_directory('slam_toolbox')
    nav2_pkg     = get_package_share_directory('nav2_bringup')
    rviz_config  = get_package_share_directory('turtlebot3_navigation2') + \
                   '/rviz/tb3_navigation2.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # ── SLAM Toolbox (online async) ──────────────────────────────────
        # Subscribes /scan, publishes /map, broadcasts map→odom TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time':     use_sim_time,
                'slam_params_file': os.path.join(pkg, 'param', 'slam_params.yaml'),
            }.items(),
        ),

        # ── Nav2 navigation stack ────────────────────────────────────────
        # Runs controller, planner, BT navigator — NO localization
        # (slam_toolbox above handles map→odom localisation)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  os.path.join(pkg, 'param', 'nav2_params.yaml'),
            }.items(),
        ),

        # ── RViz2 ────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
