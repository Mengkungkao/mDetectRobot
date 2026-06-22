import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('mdetect_bringup')
    slam_share = get_package_share_directory('slam_toolbox')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    base_config = LaunchConfiguration('base_config')
    ekf_config = LaunchConfiguration('ekf_config')
    slam_params = LaunchConfiguration('slam_params')
    nav2_params = LaunchConfiguration('nav2_params')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'base_config',
            default_value=os.path.join(
                get_package_share_directory('mdetect_base'), 'config', 'base.yaml')),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(bringup_share, 'config', 'ekf.yaml')),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(bringup_share, 'config', 'slam_toolbox.yaml')),
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
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params,
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
