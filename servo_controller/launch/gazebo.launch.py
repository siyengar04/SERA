from setuptools import Command, dist
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('ros_gz')
    world_file_name = 'empty.world'
    world = os.path.join(pkg_gazebo_ros, 'worlds', world_file_name)
    gazebo_launch = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    
    robot_description = Command(['xacro ', LaunchConfiguration('model')])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description,
            description='URDF file for the robot model'),
        DeclareLaunchArgument(
            'model',
            default_value='../urdf/robot.urdf',
            description='Absolute path to robot urdf file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world}.items()
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'robot_description': LaunchConfiguration('robot_description')}]
        ),
        
        Node(
            package='ros_gz',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robot'],
            output='screen'
        ),
    ])

