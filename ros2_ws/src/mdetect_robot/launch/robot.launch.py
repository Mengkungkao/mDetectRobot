import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mdetect_robot')
    lidar_share = get_package_share_directory('cspc_lidar')
    base_config = os.path.join(share, 'config', 'base.yaml')
    lidar_config = os.path.join(lidar_share, 'params', 'cspc_lidar.yaml')
    urdf = os.path.join(share, 'urdf', 'mdetect_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    arduino_port = LaunchConfiguration('arduino_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_lidar = LaunchConfiguration('use_lidar')

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument('lidar_angle_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('lidar_reverse_scan', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='mdetect_robot',
            executable='serial_bridge',
            name='mdetect_serial_bridge',
            parameters=[base_config, {'port': arduino_port}],
            output='screen',
        ),
        Node(
            package='cspc_lidar',
            executable='cspc_lidar',
            name='cspc_lidar',
            condition=IfCondition(use_lidar),
            parameters=[
                lidar_config,
                {
                    'port': lidar_port,
                    'frame_id': LaunchConfiguration('lidar_frame'),
                    'angle_offset_deg': ParameterValue(LaunchConfiguration('lidar_angle_offset_deg'), value_type=float),
                    'reverse_scan': ParameterValue(LaunchConfiguration('lidar_reverse_scan'), value_type=bool),
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='mdetect_robot',
            executable='cmd_mux',
            name='mdetect_cmd_mux',
            parameters=[base_config],
            output='screen',
        ),
    ])
