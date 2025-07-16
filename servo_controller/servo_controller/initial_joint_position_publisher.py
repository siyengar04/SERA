import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
from .maestro import Controller


class SimpleWalkGait(Node):
    def __init__(self):
        super().__init__("simple_walk_gait")
        self.get_logger().info("SimpleWalkGait node has started.")

        # Initialize the Maestro controller
        self.controller = Controller()  # Adjust port if necessary

        # Publisher for joint positions (for debugging or visualization)
        self.joint_publisher = self.create_publisher(
            Float64MultiArray, "/position_controllers/commands", 10
        )

        # Robot's link length in meters for inverse kinematics
        self.a = 0.11  # Set this to the appropriate link length for your robot's legs

        # Define the min and max range for each joint to match the calibration settings
        self.servo_limits = {
            0: (992, 2000),  # FRhip
            1: (496, 1200),  # BRhip
            2: (496, 2000),  # BLhip
            3: (992, 2000),  # FLhip
            4: (992, 2000),  # FRshoulder
            5: (992, 1856),  # BRknee
            6: (1168, 1808),  # BLknee
            7: (784, 2000),  # FLknee
            8: (1296, 2000),  # FRknee
            9: (992, 2000),  # BRshoulder
            10: (992, 2000),  # BLshoulder
            11: (496, 2000),  # FLshoulder
        }

        # Initial joint angles for each leg
        self.initial_position = [
            0.3,
            1.0,
            -2.0,  # Front left leg (hip, shoulder, knee)
            0.3,
            -1.0,
            1.5,  # Front right leg (hip, shoulder, knee)
            0.3,
            1.0,
            -2.0,  # Back left leg (hip, shoulder, knee)
            0.3,
            -1.0,
            1.5,  # Back right leg (hip, shoulder, knee)
        ]

        # Simple gait sequence (diagonally opposite legs moving in sync)
        self.walk_sequence = [
            {
                "legs": [0, 2],
                "X": 0.03,
                "Y": 0.15,
                "Z": 0.02,
            },  # Front Right & Back Left Up
            {
                "legs": [0, 2],
                "X": 0.06,
                "Y": 0.15,
                "Z": 0.02,
            },  # Front Right & Back Left Forward
            {
                "legs": [1, 3],
                "X": -0.03,
                "Y": 0.15,
                "Z": 0.02,
            },  # Front Left & Back Right Up
            {
                "legs": [1, 3],
                "X": -0.06,
                "Y": 0.15,
                "Z": 0.02,
            },  # Front Left & Back Right Forward
        ]

        time.sleep(2.0)  # Allow time for setup

        self.publish_initial_position()
        self.current_step = 0
        self.sequence_timer = self.create_timer(0.5, self.publish_gait_step)

    def publish_initial_position(self):
        # Publish to the servo and also to the topic
        joint_command = Float64MultiArray()
        joint_command.data = self.initial_position
        self.joint_publisher.publish(joint_command)
        self.get_logger().info(f"Publishing initial position: {joint_command.data}")

        # Send initial position to the servos
        for i in range(12):
            angle = self.initial_position[i]
            self.send_servo_command(i, angle)

    import time

    def publish_gait_step(self):
        step = self.walk_sequence[self.current_step]
        positions = (
            self.initial_position.copy()
        )  # Start from initial positions for all joints

        # Activate servos for only one leg at a time
        for leg in step["legs"]:
            angles = self.calculate_gait_positions(step)

            # Update only the specific leg's angles in the positions array
            positions[leg * 3] = angles[0]  # Hip
            positions[leg * 3 + 1] = angles[1]  # Shoulder
            positions[leg * 3 + 2] = angles[2]  # Knee

            # Send command to each servo in the leg with a small delay
            self.send_servo_command(leg * 3, angles[0])
            time.sleep(0.05)  # Delay to avoid simultaneous activation
            self.send_servo_command(leg * 3 + 1, angles[1])
            time.sleep(0.05)
            self.send_servo_command(leg * 3 + 2, angles[2])
            time.sleep(0.05)

        # Publish updated joint positions for visualization or debugging
        joint_command = Float64MultiArray()
        joint_command.data = positions
        self.joint_publisher.publish(joint_command)
        self.get_logger().info(f"Publishing gait step command: {positions}")

        # Move to the next step or loop back to the start
        self.current_step = (self.current_step + 1) % len(self.walk_sequence)

    def calculate_gait_positions(self, step):
        X, Y, Z = step["X"], step["Y"], step["Z"]
        theta1 = self.get_theta1(X, Y, Z)
        theta2 = self.get_theta2(X, Y, Z)
        theta3 = self.get_theta3(X, Y, Z)
        return [theta1, theta2, theta3]

    def send_servo_command(self, channel, angle):
        # Retrieve the minimum and maximum values for the servo in quarter-microseconds
        min_val, max_val = self.servo_limits[channel]

        # Define the expected range of angles for scaling purposes
        # Assuming the range for joint angles (e.g., -2 to +2 radians)
        # Adjust max_angle as necessary for specific joint ranges
        max_angle = 2.0 if channel in [5, 6, 7, 8] else 1.0

        # Map angle to the servo target range
        target = int(
            ((angle + max_angle) / (2 * max_angle)) * (max_val - min_val) + min_val
        )

        # Send the command to the Maestro controller
        self.controller.setTarget(channel, target)
        self.get_logger().info(
            f"Set servo {channel} to target {target} (angle {angle:.2f} rad)"
        )

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
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
