import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/ttyUSB0')


    return LaunchDescription([
        
        DeclareLaunchArgument(
            'rplidar_port',
            default_value=rplidar_port,
            description='Port for rplidar sensor'
        ),
        # lidar launchfile
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/lidar.launch.py']
            ),
            launch_arguments={'serial_port': rplidar_port}.items()
        ),
        # robot control node
        Node(
            package='rpi_robot_bringup',
            executable='rpi_robot_control.py',
            arguments=[],
            output='screen'
        ),
          # static transform from link to footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.015', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),      
        # static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.015', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        )

    ])
