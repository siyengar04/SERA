import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from .maestro import (
    Controller,
)  # Ensure this points to the path where maestro.py is located


class ControllerToServoNode(Node):
    def __init__(self):
        super().__init__("controller_to_servo")
        self.get_logger().info("ControllerToServoNode has started.")

        # Initialize the Maestro controller
        self.servo_controller = Controller()

        # Set acceleration and speed for each servo
        for chan in range(12):
            self.servo_controller.setAccel(chan, 4)
            self.servo_controller.setSpeed(chan, 10)

        # Map joint indices to Maestro channel numbers
        self.joint_to_channel = {
            0: 0,
            1: 1,
            2: 2,
            3: 3,
            4: 4,
            5: 5,
            6: 6,
            7: 7,
            8: 8,
            9: 9,
            10: 10,
            11: 11,
        }

        # Subscribe to the servo joint positions from the walking gait
        self.subscription = self.create_subscription(
            Float64MultiArray, "/servo_joint_positions", self.command_callback, 10
        )

    def command_callback(self, msg):
        if len(msg.data) != 12:
            self.get_logger().error("Received command does not have 12 values.")
            return

        # Send each position to the corresponding servo channel
        for i, position in enumerate(msg.data):
            if i in self.joint_to_channel:
                channel = self.joint_to_channel[i]
                target = self.convert_position_to_target(position, channel)
                self.servo_controller.setTarget(channel, target)
                self.get_logger().info(f"Set servo {channel} to target {target}")

    def convert_position_to_target(self, position, channel):
        min_target = 992
        max_target = 2000
        target = int((position + 1) * (max_target - min_target) / 2 + min_target)
        return max(min_target, min(target, max_target))

    def on_shutdown(self):
        self.servo_controller.close()
        self.get_logger().info("ControllerToServoNode is shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerToServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
