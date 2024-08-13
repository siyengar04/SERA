import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue  # <-- Import this

import xacro


def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = "servo_controller"
    file_subpath = "urdf/ros2control.urdf.xacro"

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_raw, value_type=str
                ),  # Explicitly set as string
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    # Spawner nodes for controllers
    position_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["position_controllers"],
                output="screen",
            )
        ],
    )

    joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [controller_manager, spawn_entity, position_spawner, joint_broad_spawner]
    )


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue  # <-- Import this

import xacro


def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = "servo_controller"
    file_subpath = "urdf/ros2control.urdf.xacro"

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_raw, value_type=str
                ),  # Explicitly set as string
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    # Spawner nodes for controllers
    position_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["position_controllers"],
                output="screen",
            )
        ],
    )

    joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [controller_manager, spawn_entity, position_spawner, joint_broad_spawner]
    )
