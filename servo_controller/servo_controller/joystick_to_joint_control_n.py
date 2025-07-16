import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import math


class JoystickToJointControl(Node):
    def __init__(self):
        super().__init__("joystick_to_joint_control")
        self.get_logger().info("JoystickToJointControl node has started.")

        # Initialize subscribers and publishers
        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )
        self.joint_publisher = self.create_publisher(
            Float64MultiArray, "/position_controllers/commands", 10
        )

        # Parameters for the robot's kinematics
        self.a = 0.11  # Link length in meters
        self.Yoffset = 0.0
        self.Yoffset2 = 0.0
        self.inc = 0.01
        self.Ylim = 0.06
        self.Ylim2 = 0.03
        self.home_positions = [0.0] * 12  # Default joint positions for home

        # Define gaits and gait parameters
        self.walk_sequence = [
            {"leg": 0, "X": 0, "Y": 0.15, "Z": 0.04},  # Front Right Up
            {"leg": 0, "X": 0.05, "Y": 0.15, "Z": 0.04},  # Front Right Forward
            {"leg": 0, "X": 0.05, "Y": 0.15, "Z": 0.0},  # Front Right Down
            {"leg": 1, "X": 0, "Y": 0.15, "Z": 0.04},  # Front Left Up
            {"leg": 1, "X": 0.05, "Y": 0.15, "Z": 0.04},  # Front Left Forward
            {"leg": 1, "X": 0.05, "Y": 0.15, "Z": 0.0},  # Front Left Down
            # Continue this for each leg in the walk sequence
        ]

        self.trot_sequence = [
            {"legs": [0, 3], "X": 0, "Y": 0.15, "Z": 0.04},  # Diagonal legs up
            {"legs": [0, 3], "X": 0.05, "Y": 0.15, "Z": 0.04},  # Diagonal legs forward
            {"legs": [0, 3], "X": 0.05, "Y": 0.15, "Z": 0.0},  # Diagonal legs down
            {"legs": [1, 2], "X": 0, "Y": 0.15, "Z": 0.04},  # Other diagonal legs up
            {
                "legs": [1, 2],
                "X": 0.05,
                "Y": 0.15,
                "Z": 0.04,
            },  # Other diagonal legs forward
            {
                "legs": [1, 2],
                "X": 0.05,
                "Y": 0.15,
                "Z": 0.0,
            },  # Other diagonal legs down
        ]

        # Default to the "walk" gait mode
        self.gait_mode = "walk"
        self.current_step = 0
        self.gait_active = False  # Flag to control the gait loop
        self.sequence_timer = self.create_timer(0.5, self.publish_gait_step)

    def joy_callback(self, msg):
        # Right stick vertical for hip movement (theta1)
        right_stick_vertical = msg.axes[4]
        # Right stick horizontal for shoulder movement (theta2)
        right_stick_horizontal = msg.axes[3]

        # Adjust Yoffset for hip (theta1) based on up/down movement
        if right_stick_vertical > 0.1:
            self.Yoffset += self.inc
            self.Yoffset = min(self.Yoffset, self.Ylim)
        elif right_stick_vertical < -0.1:
            self.Yoffset -= self.inc
            self.Yoffset = max(self.Yoffset, -self.Ylim)

        # Adjust Yoffset2 for shoulder (theta2) based on left/right movement
        if right_stick_horizontal > 0.1:
            self.Yoffset2 += self.inc
            self.Yoffset2 = min(self.Yoffset2, self.Ylim2)
        elif right_stick_horizontal < -0.1:
            self.Yoffset2 -= self.inc
            self.Yoffset2 = max(self.Yoffset2, -self.Ylim2)

        # Publish updated joint positions
        self.publish_joint_positions()

    def publish_gait_step(self):
        # Exit if gait sequence is not active
        if not self.gait_active:
            return

        # Debug log to trace publish calls
        self.get_logger().debug(
            f"Publishing gait step {self.current_step} in mode {self.gait_mode}"
        )

        # Choose the sequence based on the selected gait mode
        sequence = (
            self.walk_sequence if self.gait_mode == "walk" else self.trot_sequence
        )

        # Perform the current step
        step = sequence[self.current_step]
        positions = self.calculate_gait_positions(step)

        joint_command = Float64MultiArray()
        joint_command.data = self.order_joints(positions)
        self.joint_publisher.publish(joint_command)

        # Advance to the next step or loop back to the start
        self.current_step += 1
        if self.current_step >= len(sequence):
            self.current_step = 0  # Reset to the beginning to create a loop

    def calculate_gait_positions(self, step):
        # Initialize a position array with 12 elements for hip, shoulder, knee for each leg
        positions = [0.0] * 12
        if "leg" in step:
            # For single-leg movement in a walking gait
            leg_index = step["leg"]
            X, Y, Z = step["X"], step["Y"], step["Z"]
            theta1 = self.get_theta1(X, Y, Z)
            theta2 = self.get_theta2(X, Y, Z)
            theta3 = self.get_theta3(X, Y, Z)

            # Apply mirroring for right-side legs
            if leg_index in [1, 3]:  # Right side legs (FR, BR)
                theta1 = -theta1
                theta2 = -theta2
                theta3 = -theta3

            # Set angles for the specific leg
            positions[leg_index * 3] = theta1  # Hip joint
            positions[leg_index * 3 + 1] = theta2  # Shoulder joint
            positions[leg_index * 3 + 2] = theta3  # Knee joint

        elif "legs" in step:
            # For multi-leg movement in a trotting gait
            for leg_index in step["legs"]:
                X, Y, Z = step["X"], step["Y"], step["Z"]
                theta1 = self.get_theta1(X, Y, Z)
                theta2 = self.get_theta2(X, Y, Z)
                theta3 = self.get_theta3(X, Y, Z)

                # Apply mirroring for right-side legs
                if leg_index in [1, 3]:  # Right side legs (FR, BR)
                    theta1 = -theta1
                    theta2 = -theta2
                    theta3 = -theta3

                # Set angles for each leg in the trot
                positions[leg_index * 3] = theta1  # Hip joint
                positions[leg_index * 3 + 1] = theta2  # Shoulder joint
                positions[leg_index * 3 + 2] = theta3  # Knee joint

        return positions

    def publish_joint_positions(self):
        positions = []
        for leg_index in range(4):
            # Calculate X, Y, Z for each leg based on tilt and offsets
            X, Y, Z = self.calculate_xyz_for_leg(leg_index)

            # Get joint angles for hip, shoulder, and knee
            theta1 = self.get_theta1(X, Y, Z)
            theta2 = self.get_theta2(X, Y, Z)
            theta3 = self.get_theta3(X, Y, Z)

            # Apply mirroring for right-side legs
            if leg_index in [1, 3]:  # Right side legs (FR, BR)
                theta1 = -theta1
                theta2 = -theta2
                theta3 = -theta3

            # Append joint angles in order for each leg (hip, shoulder, knee)
            positions.extend([theta1, theta2, theta3])

        # Publish to the /position_controllers/commands topic
        joint_command = Float64MultiArray()
        joint_command.data = self.order_joints(positions)
        self.joint_publisher.publish(joint_command)

    def order_joints(self, positions):
        # Order the joints according to the URDF configuration
        ordered_positions = [
            positions[0],
            positions[1],
            positions[2],  # hip_fl, shoulder_fl, knee_fl
            positions[3],
            positions[4],
            positions[5],  # hip_fr, shoulder_fr, knee_fr
            positions[6],
            positions[7],
            positions[8],  # hip_bl, shoulder_bl, knee_bl
            positions[9],
            positions[10],
            positions[11],  # hip_br, shoulder_br, knee_br
        ]
        return ordered_positions

    def publish_joint_positions(self):
        positions = []
        for leg_index in range(4):
            # Calculate X, Y, Z for each leg based on tilt and offsets
            X, Y, Z = self.calculate_xyz_for_leg(leg_index)

            # Get joint angles for hip, shoulder, and knee
            theta1 = self.get_theta1(X, Y, Z)
            theta2 = self.get_theta2(X, Y, Z)
            theta3 = self.get_theta3(X, Y, Z)

            # Append joint angles in order for each leg (hip, shoulder, knee)
            positions.extend([theta1, theta2, theta3])

        # Publish to the /position_controllers/commands topic
        joint_command = Float64MultiArray()
        joint_command.data = self.order_joints(positions)
        self.joint_publisher.publish(joint_command)

    def calculate_xyz_for_leg(self, leg_index):
        # Base positions
        X_base = 0.0
        Y_base = 0.15
        Z_base = 0.0

        # Apply offsets based on joystick movements
        X = X_base + self.Yoffset2  # Affects theta2 (shoulder)
        Y = Y_base + self.Yoffset  # Affects theta1 (hip)
        Z = Z_base  # No offset for knee in this function

        return X, Y, Z

    def get_theta1(self, X, Y, Z):
        # Calculate theta1 angle for inverse kinematics
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
        # Calculate theta2 angle for inverse kinematics
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
        # Calculate theta3 angle for inverse kinematics
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
    node = JoystickToJointControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
