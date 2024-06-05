import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray  # Assuming servo control uses an array of integers
from .maestro import Controller  # Adjust based on your actual package structure
from rclpy.logging import get_logger
from rclpy.time import Time

logger = get_logger('controller_to_servo')


class ControllerToServo(Node):
    def __init__(self):
        super().__init__('controller_to_servo')
        self.servo_controller = Controller()
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.last_command_time = self.get_clock().now()
        self.command_interval = 0.1  # seconds between commands

        # Define min and max positions for each servo
        self.servo_limits = [
            (4544, 7104),  # Servo 0
            (1728, 6848),  # Servo 1
            (256, 6144),  # Servo 2
            (6656, 10304),  # Servo 3
            (5632, 10432),  # Servo 4
            (3200, 7616),  # Servo 5
            (4672, 6720),  # Servo 6
            (4416, 8832),  # Servo 7
            (5760, 8704),  # Servo 8
            (256, 5568),  # Servo 9
            (8128, 10816),  # Servo 10
            (6912, 16320)   # Servo 11
        ]


    def joy_callback(self, msg):
        try:
            if len(msg.axes) < 6:
                self.get_logger().error('Not enough joystick axes available')
                return

            positions = [6000] * 12  # Initialize positions

            # Calculate positions from joystick input within defined limits
            for index in range(12):
                # Assuming we have an axis for each servo directly
                axis_value = msg.axes[index] if index < len(msg.axes) else 0
                min_val, max_val = self.servo_limits[index]
                position = int((max_val - min_val) / 2 * (axis_value + 1) + min_val)
                positions[index] = position

            # Send positions to servos
            for index, position in enumerate(positions):
                self.servo_controller.setTarget(index, position)
                self.get_logger().info(f"Servo {index} set to position {position}")
        except Exception as e:
            self.get_logger().error(f'Failed to set servo positions: {str(e)}')




def main(args=None):
    rclpy.init(args=args)
    node = ControllerToServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
