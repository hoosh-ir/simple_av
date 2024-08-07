from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_av',
            executable='localization',
            name='localization_node',
            output='screen'
        ),
        Node(
            package='simple_av',
            executable='planning',
            name='planning_node',
            output='screen'
        ),
        Node(
            package='simple_av',
            executable='controller',
            name='control_node',
            output='screen'
        )
    ])