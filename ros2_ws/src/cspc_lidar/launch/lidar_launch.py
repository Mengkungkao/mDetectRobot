import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('cspc_lidar')
    default_params = os.path.join(share, 'params', 'cspc_lidar.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        Node(
            package='cspc_lidar',
            executable='cspc_lidar',
            name='cspc_lidar',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {'port': LaunchConfiguration('port')}],
        ),
    ])
