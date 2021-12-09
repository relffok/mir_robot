import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='mir_restapi',
            executable='mir_restapi_server',
            output='screen'),
        
        Node(
            package='mir_restapi',
            executable='mir_restapi_client',
            output='screen'),

    ])
