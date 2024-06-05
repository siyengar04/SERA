import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .maestro import Controller  # Ensure this import matches your file structure

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.servo_controller = Controller()
        self.subscription = self.create_subscription(
            Int32MultiArray, 
            'servo_positions', 
            self.update_servo_positions, 
            10
        )
        self.subscription  # prevent unused variable warning

    def update_servo_positions(self, msg):
        positions = msg.data  # Array of positions
        for i, position in enumerate(positions):
            self.servo_controller.setTarget(i, position)

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
