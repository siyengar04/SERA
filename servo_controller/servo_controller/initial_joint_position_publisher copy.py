import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class SimpleWalkGait(Node):
    def __init__(self):
        super().__init__("simple_walk_gait")
        self.get_logger().info("SimpleWalkGait node has started.")

        # Publisher for joint positions
        self.joint_publisher = self.create_publisher(
            Float64MultiArray, "/position_controllers/commands", 10
        )

        # Robot's link length in meters for inverse kinematics
        self.a = 0.11  # Set the link length for your robot's legs

        # Initial joint angles for each leg within joint limits
        self.initial_position = [
            0.3,
            0.75,
            -1.5,  # Front left leg (hip, shoulder, knee)
            0.3,
            -0.75,
            1.5,  # Front right leg (hip, shoulder, knee)
            0.3,
            0.75,
            -1.5,  # Back left leg (hip, shoulder, knee)
            0.3,
            -0.75,
            1.5,  # Back right leg (hip, shoulder, knee)
        ]

        # Simple walking gait sequence without the initial position
        self.walk_sequence = [
            {"leg": 0, "X": 0.03, "Y": 0.15, "Z": 0.02},  # Front Right Up
            {"leg": 0, "X": 0.06, "Y": 0.15, "Z": 0.02},  # Front Right Forward
            {
                "leg": 0,
                "positions": self.initial_position[0:3],
            },  # Front Right Return to Initial
            {"leg": 1, "X": -0.03, "Y": 0.15, "Z": 0.02},  # Front Left Up
            {"leg": 1, "X": -0.06, "Y": 0.15, "Z": 0.02},  # Front Left Forward
            {
                "leg": 1,
                "positions": self.initial_position[3:6],
            },  # Front Left Return to Initial
            {"leg": 2, "X": 0.03, "Y": 0.15, "Z": 0.02},  # Back Right Up
            {"leg": 2, "X": 0.06, "Y": 0.15, "Z": 0.02},  # Back Right Forward
            {
                "leg": 2,
                "positions": self.initial_position[6:9],
            },  # Back Right Return to Initial
            {"leg": 1, "X": -0.03, "Y": 0.15, "Z": 0.02},  # Back Left Up
            {"leg": 1, "X": -0.06, "Y": 0.15, "Z": 0.02},  # Back Left Forward
            {
                "leg": 3,
                "positions": self.initial_position[9:12],
            },  # Back Left Return to Initial
        ]

        # Wait for Gazebo to fully initialize
        time.sleep(2.0)  # Adjust delay if needed to ensure Gazebo is ready

        # Publish the initial position once
        self.publish_initial_position()

        # Start the gait sequence at the first movement step
        self.current_step = 0  # Start at the first gait movement
        self.sequence_timer = self.create_timer(0.5, self.publish_gait_step)

    def publish_initial_position(self):
        joint_command = Float64MultiArray()
        joint_command.data = self.initial_position
        self.get_logger().info(f"Publishing initial position: {joint_command.data}")
        self.joint_publisher.publish(joint_command)
        time.sleep(5)  # Pause for 1 second before starting the gait

    def publish_gait_step(self):
        if not self.walk_sequence:
            return  # Ensure sequence is non-empty

        # Start with the initial position as a base
        positions = self.initial_position.copy()

        # Get the current step in the walk sequence
        step = self.walk_sequence[self.current_step]

        # Check if the step specifies positions or gait movement
        if "positions" in step:
            # Replace only the specified leg's joints with the provided positions
            leg_index = step["leg"]
            positions[leg_index * 3 : leg_index * 3 + 3] = step["positions"]
            self.get_logger().info(
                f"Publishing reset position for leg {leg_index}: {positions}"
            )

            # Create and publish the reset position command
            joint_command = Float64MultiArray()
            joint_command.data = positions
            self.joint_publisher.publish(joint_command)

            # Add a delay to wait for this leg to complete its reset before moving the next leg
            time.sleep(0.5)  # Adjust the delay time as needed
        else:
            # Calculate positions based on gait movement and update only the specified leg
            leg_positions = self.calculate_gait_positions(step)
            leg_index = step["leg"]
            positions[leg_index * 3 : leg_index * 3 + 3] = leg_positions
            self.get_logger().info(f"Publishing gait step command: {positions}")

            # Create and publish the gait movement command
            joint_command = Float64MultiArray()
            joint_command.data = positions
            self.joint_publisher.publish(joint_command)

        # Advance to the next step or loop back to the start
        self.current_step += 1
        if self.current_step >= len(self.walk_sequence):
            self.current_step = 0  # Loop back to the start

    def calculate_gait_positions(self, step):
        # Initialize a position array with default values for all legs
        positions = self.initial_position.copy()

        leg_index = step["leg"]
        X, Y, Z = step["X"], step["Y"], step["Z"]
        theta1 = self.get_theta1(X, Y, Z)
        theta2 = self.get_theta2(X, Y, Z)
        theta3 = self.get_theta3(X, Y, Z)

        # Apply joint-specific limits
        theta1 = min(max(theta1, -0.2), 0.4)  # Hip limit
        if leg_index in [0, 2]:  # Left side legs
            theta2 = min(max(theta2, 0), 1)  # Shoulder left limit
            theta3 = min(max(theta3, -2.5), -1)  # Knee left limit
        else:  # Right side legs
            theta2 = min(max(theta2, -1), 0)  # Shoulder right limit
            theta3 = min(max(theta3, 1), 2.5)  # Knee right limit

        # Set angles for the specific leg
        positions[leg_index * 3] = theta1
        positions[leg_index * 3 + 1] = theta2
        positions[leg_index * 3 + 2] = theta3

        return positions[
            leg_index * 3 : leg_index * 3 + 3
        ]  # Only return the leg's angles

    def get_theta1(self, X, Y, Z):
        try:
            angle1 = math.atan2(-X, math.sqrt(Y**2 + Z**2))
            angle2 = math.acos(
                min(max(math.sqrt(X**2 + Y**2 + Z**2) / (2 * self.a), -1), 1)
            )
            theta1 = math.pi / 2 - angle1 - angle2
            return theta1
        except ValueError as e:
            self.get_logger().warn(f"Invalid value in get_theta1 calculation: {e}")
            return 0.0

    def get_theta2(self, X, Y, Z):
        try:
            term = 1 - ((X**2 + Y**2 + Z**2) / (2 * self.a**2))
            term = min(max(term, -1), 1)
            angle = math.acos(term)
            theta1 = self.get_theta1(X, Y, Z)
            theta2 = math.pi / 2 - angle + theta1
            return theta2
        except ValueError as e:
            self.get_logger().warn(f"Invalid value in get_theta2 calculation: {e}")
            return 0.0

    def get_theta3(self, X, Y, Z):
        try:
            term = Z / math.sqrt(Z**2 + Y**2)
            term = min(max(term, -1), 1)
            angle = math.asin(term)
            theta3 = math.pi / 2 + angle
            return theta3
        except ValueError as e:
            self.get_logger().warn(f"Invalid value in get_theta3 calculation: {e}")
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWalkGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
