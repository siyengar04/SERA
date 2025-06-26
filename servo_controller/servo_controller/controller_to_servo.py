import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .maestro import Controller
import math
from rclpy.logging import get_logger
logger = get_logger('controller_to_servo')

class ControllerToServo(Node):
    def __init__(self):
        super().__init__('controller_to_servo')
        self.servo_controller = Controller()
        self.servo_controller = Controller()
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.servo_channels = [i for i in range(12)] 
        self.posnorm = [1500] * 12  # neutral pos
        self.dir = ["-", "+", "+", "+", "-", "-", "+", "+", "+", "-", "-", "-"]  # angle direction
        self.K = 850 / 90  # ms/deg
        self.a = 110  # links, mm

    def to_deg(self, rad):
        return rad * 180 / math.pi

    def get_theta1(self, x, y, z):
        return 90 - math.atan(-x / math.sqrt(y**2 + z**2)) * (180 / math.pi) - math.acos(math.sqrt(x**2 + y**2 + z**2) / (2 * self.a)) * (180 / math.pi)

    def get_theta2(self, x, y, z):
        return 90 - math.acos(1 - ((x**2 + y**2 + z**2) / (2 * self.a**2))) * (180 / math.pi) + self.get_theta1(x, y, z)

    def get_theta3(self, x, y, z):
        return 90 + math.asin(z / math.sqrt(z**2 + y**2)) * (180 / math.pi)

    def move_servos(self, targets):
        for i, target in enumerate(targets):
            if target is not None:
                if self.dir[i] == "+":
                    target_ms = self.posnorm[i] + self.K * (target - 90)
                else:
                    target_ms = self.posnorm[i] - self.K * (target - 90)
                self.servo_controller.setTarget(self.servo_channels[i], int(target_ms))

    def update_positions(self, x, y, z):
        theta1 = self.get_theta1(x, y, z)
        theta2 = self.get_theta2(x, y, z)
        theta3 = self.get_theta3(x, y, z)
        self.move_servos([theta3, theta1, theta2, None, None, None, None, None, None, None, None, None])

    def joy_callback(self, msg):
        self.get_logger().info('Joy callback triggered')  # logger
        
        if len(msg.axes) >= 3:
            self.update_positions(msg.axes[0] * 100, msg.axes[1] * 100, msg.axes[2] * 100)  

def main(args=None):
    rclpy.init(args=args)
    node = ControllerToServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
