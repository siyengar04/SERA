import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "servo_controller"
    urdf_file = "ros2control.urdf.xacro"

    urdf_path = os.path.join(
        get_package_share_directory(package_name), "urdf", urdf_file
    )

    # Convert xacro to urdf
    urdf_output = os.path.join(
        get_package_share_directory(package_name), "urdf", "robot.urdf"
    )

    os.system(f"xacro {urdf_path} -o {urdf_output}")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "robot", "-file", urdf_output],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": open(urdf_output).read(),
                    }
                ],
            ),
        ]
    )
