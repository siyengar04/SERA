import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class InitialJointPositionPublisher(Node):
    def __init__(self):
        super().__init__("initial_joint_position_publisher")
        self.publisher = self.create_publisher(
            Float64MultiArray, "/position_controllers/commands", 10
        )
        self.timer = self.create_timer(1.0, self.publish_initial_positions)

    def publish_initial_positions(self):
        msg = Float64MultiArray()
        msg.data = [
            1.0,
            1.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Set your initial positions here
        self.publisher.publish(msg)
        self.get_logger().info("Published initial joint positions")


def main(args=None):
    rclpy.init(args=args)
    node = InitialJointPositionPublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
