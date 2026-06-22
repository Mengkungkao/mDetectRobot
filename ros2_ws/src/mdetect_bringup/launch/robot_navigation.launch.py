import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('mdetect_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    base_config = LaunchConfiguration('base_config')
    ekf_config = LaunchConfiguration('ekf_config')
    nav2_params = LaunchConfiguration('nav2_params')
    start_lidar = LaunchConfiguration('start_lidar')
    lidar_params = LaunchConfiguration('lidar_params')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument(
            'lidar_params',
            default_value=os.path.join(
                get_package_share_directory('cspc_lidar'),
                'params', 'cspc_lidar.yaml')),
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the saved Nav2 map YAML file'),
        DeclareLaunchArgument(
            'base_config',
            default_value=os.path.join(
                get_package_share_directory('mdetect_base'), 'config', 'base.yaml')),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(bringup_share, 'config', 'ekf.yaml')),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'robot_bringup.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'base_config': base_config,
                'ekf_config': ekf_config,
                'start_lidar': start_lidar,
                'lidar_params': lidar_params,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
    ])
