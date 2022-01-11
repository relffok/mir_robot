from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # def process_prefix(context):
    #     namespace = ""
    #     try:
    #         prefix = context.launch_configurations['prefix']
    #         try:
    #             namespace = context.launch_configurations['namespace']
    #         except KeyError: # No namespace
    #             namespace = prefix
    #         namespace = namespace + '/' + prefix
    #     except KeyError:
    #         pass
    #     return [SetLaunchConfiguration('namspace_parsed', namespace)]


    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([

        #OpaqueFunction(function=process_prefix),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Robot prefix'),

        Node(
            package='ira_laser_tools',
            name='mir_laser_scan_merger',
            executable='laserscan_multi_merger',
            parameters=[
                {'laserscan_topics': "b_scan f_scan",
                'destination_frame': "virtual_laser_link",
                'scan_destination_topic': "scan",
                'cloud_destination_topic': "scan_cloud",
                'min_height': -0.25,
                'max_completion_time': 0.05,
                'max_merge_time_diff': 0.005,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'best_effort': False}],
            namespace = LaunchConfiguration('namespace'),    # adds namespace to topic names and frames
            output='screen')
    ])
