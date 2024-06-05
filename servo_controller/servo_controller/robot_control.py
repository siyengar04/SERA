import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .maestro import Controller  # Adjust based on your actual package structure
from rclpy.logging import get_logger
import math

logger = get_logger('robot_control')



class ControllerToServo(Node):
    def __init__(self):
        super().__init__('robot_control')
        # Initialize variables and set up subscribers, etc.

    # IK functions
    def get_theta1(self, x, y, z):
        a = 110  # Length of both links in mm
        try:
            return 90 - math.degrees(math.atan(-x / math.sqrt(y**2 + z**2))) - \
                   math.degrees(math.acos(math.sqrt(x**2 + y**2 + z**2) / (2 * a)))
        except ValueError as e:
            self.logger.error(f'Error computing theta1: {str(e)}')
            return 0

    def get_theta2(self, x, y, z):
        a = 110
        try:
            return 90 - math.degrees(math.acos(1 - ((x**2 + y**2 + z**2) / (2 * a**2)))) + \
                   self.get_theta1(x, y, z)
        except ValueError as e:
            self.logger.error(f'Error computing theta2: {str(e)}')
            return 0

    def get_theta3(self, x, y, z):
        try:
            return 90 + math.degrees(math.asin(z / math.sqrt(z**2 + y**2)))
        except ValueError as e:
            self.logger.error(f'Error computing theta3: {str(e)}')
            return 0

    # Existing methods...
