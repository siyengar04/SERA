from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command(['xacro ', LaunchConfiguration('model')]),
            description='URDF file for the robot model'),
        DeclareLaunchArgument(
            'model',
            default_value='../urdf/robot.urdf',
            description='Absolute path to robot urdf file'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'robot_description': LaunchConfiguration('robot_description')}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', 'path_to_your_rviz_config_file/config.rviz'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])

