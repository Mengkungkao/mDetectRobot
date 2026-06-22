import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    share = get_package_share_directory('mdetect_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_share = get_package_share_directory('slam_toolbox')
    nav_params = os.path.join(share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(share, 'config', 'slam_toolbox.yaml')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'slam_params_file': slam_params}.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'rviz_launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        GroupAction([SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'), navigation]),
        slam,
        rviz,
    ])
