from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "servo_controller"  # Replace with your package name
    urdf_file = "ros2control.urdf.xacro"  # Replace with your URDF file name
    controllers_file = (
        "robot_controllers.yaml"  # Replace with your controllers config file name
    )

    urdf_path = get_package_share_directory(package_name) + "/urdf/" + urdf_file
    controllers_path = (
        get_package_share_directory(package_name) + "/config/" + controllers_file
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
    )

    # Start the robot state publisher
    robot_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_path}],
    )

    # Start the controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": urdf_path}, controllers_path],
        output="screen",
    )

    # Spawner nodes for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers"],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_description,
            controller_manager,
            joint_state_broadcaster_spawner,
            position_controller_spawner,
        ]
    )
