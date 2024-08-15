import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class JoystickToJointControl(Node):
    def __init__(self):
        super().__init__("joystick_to_joint_control")
        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )
        self.joint_publisher = self.create_publisher(
            Float64MultiArray, "/position_controllers/command", 10
        )
        self.joint_positions = [0.0] * 12  # Assuming 12 joints

    def joy_callback(self, msg):
        # Example: Mapping joystick axes to joint positions
        # This is a simple mapping; you'll need to adjust it for your specific robot
        # Assuming the joystick axes control joint positions directly
        self.joint_positions[0] = msg.axes[0]  # Axis 0 controls first joint
        self.joint_positions[1] = msg.axes[1]  # Axis 1 controls second joint
        # Add more mappings as needed

        joint_command = Float64MultiArray()
        joint_command.data = self.joint_positions
        self.joint_publisher.publish(joint_command)


def main(args=None):
    rclpy.init(args=args)
    joystick_to_joint_control = JoystickToJointControl()
    rclpy.spin(joystick_to_joint_control)
    joystick_to_joint_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
