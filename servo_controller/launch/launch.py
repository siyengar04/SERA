from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='servo_controller',
            executable='controller_to_servo',
            name='controller_to_servo'
        )
    ])
