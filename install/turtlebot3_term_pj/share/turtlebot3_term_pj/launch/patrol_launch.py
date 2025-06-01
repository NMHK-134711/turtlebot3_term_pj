from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_term_pj',
            executable='ui_main_node',
            name='ui_main_node',
            output='screen'
        ),
        Node(
            package='turtlebot3_term_pj',
            executable='patrol_node',
            name='patrol_node',
            output='screen'
        )
    ])