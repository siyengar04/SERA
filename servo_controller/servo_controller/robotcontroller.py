import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from math import atan2, sqrt, pow, acos, asin, pi

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self._publishers = None
        self.initialize_publishers()
        
        self.servo_limits = [
            (600, 2400),  # Adjust these values as necessary for each servo
            (500, 2500),  # Example values
            (700, 2600),
            # Add the rest for each of the 12 servos
        ]
        
        self.subscription = self.create_subscription(
            Vector3,
            'target_positions',
            self.position_callback,
            10
        )

    def initialize_publishers(self):
        self._publishers = []
        for i in range(12):  # Assuming 12 servos
            topic_name = f'robot/joint{i}_position_controller/command'
            pub = self.create_publisher(Float64, topic_name, 10)
            self._publishers.append(pub)

    def position_callback(self, msg):
        X, Y, Z = msg.x, msg.y, msg.z
        thetas = [self.get_theta1(X, Y, Z), self.get_theta2(X, Y, Z), self.get_theta3(X, Y, Z)]
        self.publish_joint_commands(thetas)

    def publish_joint_commands(self, angles):
        for i, angle in enumerate(angles):
            if i < len(self._publishers):
                msg = Float64()
                msg.data = self.degrees_to_servo_position(angle, i)
                self._publishers[i].publish(msg)

    def degrees_to_servo_position(self, degrees, index):
        min_val, max_val = self.servo_limits[index]
        # Convert degrees into the servo range
        servo_position = min_val + (degrees / 90) * (max_val - min_val)  # Assumes 0-90 degrees input
        return int(servo_position)

    def get_theta1(self, X, Y, Z):
        return 90 - atan2(-X, sqrt(Y**2 + Z**2)) * (180 / pi) - acos(sqrt(X**2 + Y**2 + Z**2) / (2 * 110)) * (180 / pi)

    def get_theta2(self, X, Y, Z):
        return 90 - acos(1 - ((X**2 + Y**2 + Z**2) / (2 * 110**2))) * (180 / pi) + self.get_theta1(X, Y, Z)

    def get_theta3(self, X, Y, Z):
        return 90 + asin(Z / sqrt(Z**2 + Y**2)) * (180 / pi)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
