import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('mdetect_bringup')
    base_share = get_package_share_directory('mdetect_base')
    description_share = get_package_share_directory('mdetect_description')
    lidar_share = get_package_share_directory('cspc_lidar')

    use_sim_time = LaunchConfiguration('use_sim_time')
    base_config = LaunchConfiguration('base_config')
    ekf_config = LaunchConfiguration('ekf_config')
    start_lidar = LaunchConfiguration('start_lidar')
    lidar_params = LaunchConfiguration('lidar_params')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument(
            'lidar_params',
            default_value=os.path.join(
                lidar_share, 'params', 'cspc_lidar.yaml'),
            description='COIN-D6 LiDAR parameter file.'),
        DeclareLaunchArgument(
            'base_config',
            default_value=os.path.join(base_share, 'config', 'base.yaml')),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(bringup_share, 'config', 'ekf.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_share, 'launch', 'lidar_launch.py')),
            condition=IfCondition(start_lidar),
            launch_arguments={'params_file': lidar_params}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(base_share, 'launch', 'base.launch.py')),
            launch_arguments={'config': base_config}.items(),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])
