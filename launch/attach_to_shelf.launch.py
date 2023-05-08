# from launch.actions import DeclareLaunchArgument
# from launch import LaunchDescription
# import launch_ros.actions
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'obstacle',
#             default_value='1.0',
#             description='Distance to obstacle'
#         ),
#         DeclareLaunchArgument(
#             'degrees',
#             default_value='30',
#             description='Angle in degrees'
#         ),
#         DeclareLaunchArgument(
#             'final_approach',
#             default_value='true',
#             description='Whether to perform final approach'
#         ),
#         launch_ros.actions.Node(
#             package='attach_shelf',
#             executable='pre_approach_v2',
#             name='attach_shelf_client',
#             parameters=[
#                 {'obstacle': LaunchConfiguration('obstacle')},
#                 {'degrees': LaunchConfiguration('degrees')},
#                 {'final_approach': LaunchConfiguration('final_approach')}
#             ]
#         ),
#             launch_ros.actions.Node(
#             package='attach_shelf',
#             executable='approach_server',
#             name='attach_shelf_server',
#         )
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

class Obstacle(TextSubstitution):
    def __init__(self):
        super().__init__(text="some_text")

    def describe(self):
        return "Distance to obstacle"

    def perform(self, context):
        return context.launch_configurations["obstacle"]

class Degrees(TextSubstitution):
    def __init__(self):
        super().__init__(text="some_text")

    def describe(self):
        return "Angle in degrees"

    def perform(self, context):
        return context.launch_configurations["degrees"]

class FinalApproach(TextSubstitution):
    def __init__(self):
        super().__init__(text="some_text")

    def describe(self):
        return "Whether to perform final approach"

    def perform(self, context):
        return context.launch_configurations["final_approach"]

rviz_config_dir = os.path.join(get_package_share_directory('attach_shelf'), 'config', 'check.rviz')

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle',
            default_value='0.3',
            description='Distance to obstacle'
        ),
        DeclareLaunchArgument(
            'degrees',
            default_value='90',
            description='Angle in degrees'
        ),
        DeclareLaunchArgument(
            'final_approach',
            default_value='true',
            description='Whether to perform final approach'
        ),
        Node(
            package='attach_shelf',
            executable='pre_approach_v2',
            parameters=[
                {'obstacle': LaunchConfiguration('obstacle')},
                {'degrees': LaunchConfiguration('degrees')},
                {'final_approach': LaunchConfiguration('final_approach')}
            ]
        ),
        Node(
            package='attach_shelf',
            executable='approach_server',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    ])

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch.actions import DeclareLaunchArgument

# def generate_launch_description():
#     return LaunchDescription([
#         # Node(
#         #     package='attach_shelf',
#         #     executable='pre_approach_v2',
#         #     name='attach_shelf_client_1' # cambio de nombre
#         # ),
#         Node(
#             package='attach_shelf',
#             executable='world_to_odom',
#             name='pub_frame_world',
#         ),
#         Node(
#             package='attach_shelf',
#             executable='approach_server',
#             name='time_to',
#         )
#     ])
