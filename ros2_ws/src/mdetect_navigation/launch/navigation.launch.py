import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    share = get_package_share_directory('mdetect_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    map_yaml = LaunchConfiguration('map')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'false',
            'params_file': params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'rviz_launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a saved map YAML file'),
        localization,
        GroupAction([SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'), navigation]),
        rviz,
    ])
