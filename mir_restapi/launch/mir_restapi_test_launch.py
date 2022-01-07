from launch import LaunchDescription
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
