import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    package_name = "servo_controller"
    urdf_file = "ros2control.urdf.xacro"

    urdf_path = os.path.join(
        get_package_share_directory(package_name), "urdf", urdf_file
    )

    urdf_output = os.path.join(
        get_package_share_directory(package_name), "urdf", "robot.urdf"
    )

    # Convert xacro to urdf with error checking
    xacro_command = f"xacro {urdf_path} -o {urdf_output}"
    conversion_result = os.system(xacro_command)

    if conversion_result != 0:
        raise RuntimeError(f"Failed to convert {urdf_file} to URDF.")

    return LaunchDescription(
        [
            # Log the URDF path
            LogInfo(msg=f"Using URDF file at: {urdf_output}"),
            # Launch Gazebo
            ExecuteProcess(
                cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),
            # Spawn robot in Gazebo
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "robot",
                    "-file",
                    urdf_output,
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0.5",
                ],
                output="screen",
            ),
            # Start robot state publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"robot_description": open(urdf_output).read()},
                ],
            ),
            # Start the controller manager
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": open(urdf_output).read()},
                    os.path.join(
                        get_package_share_directory(package_name),
                        "config",
                        "robot_controllers.yaml",
                    ),
                ],
                output="screen",
            ),
            # Spawn the joint_state_broadcaster
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            # Spawn the position_controllers
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["position_controllers"],
                output="screen",
            ),
            # Start the joystick driver
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": "/dev/input/js0"}],
            ),
            # Joystick to joint command converter
            Node(
                package=package_name,
                executable="joystick_to_joint_control",
                name="joystick_to_joint_control",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            # Initial joint position publisher
            Node(
                package=package_name,
                executable="initial_joint_position_publisher",  # This should match your Python node
                name="initial_joint_position_publisher",
                output="screen",
            ),
        ]
    )
