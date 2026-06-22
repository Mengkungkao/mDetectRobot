import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_model = os.path.join(
        get_package_share_directory('mdetect_description'),
        'urdf',
        'mdetect_robot.urdf.xacro',
    )
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
    ])
