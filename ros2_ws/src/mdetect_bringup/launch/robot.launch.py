import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('mdetect_bringup')
    description_share = get_package_share_directory('mdetect_description')

    base_params = os.path.join(bringup_share, 'config', 'base.yaml')
    lidar_params = os.path.join(bringup_share, 'config', 'lidar.yaml')
    urdf = os.path.join(description_share, 'urdf', 'mdetect_robot.urdf.xacro')

    arduino_port = LaunchConfiguration('arduino_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    initialize = LaunchConfiguration('initialize')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('initialize', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mdetect_base',
            executable='serial_bridge',
            name='mdetect_serial_bridge',
            output='screen',
            parameters=[base_params, {'port': arduino_port}],
        ),
        Node(
            package='cspc_lidar',
            executable='cspc_lidar',
            name='cspc_lidar',
            output='screen',
            parameters=[lidar_params, {'port': lidar_port, 'frame_id': 'laser'}],
        ),
        Node(
            package='mdetect_base',
            executable='cmd_mux',
            name='mdetect_cmd_mux',
            output='screen',
            parameters=[base_params],
        ),
        TimerAction(
            period=2.5,
            actions=[Node(
                package='mdetect_base',
                executable='robot_initializer',
                name='mdetect_robot_initializer',
                output='screen',
                parameters=[base_params, {'initialize_on_start': ParameterValue(initialize, value_type=bool)}],
            )],
        ),
    ])
