import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .maestro import Controller  # Adjust based on your actual package structure
from rclpy.logging import get_logger

logger = get_logger('controller_to_servo')

class ControllerToServo(Node):
    def __init__(self):
        self.FRhip = 0
        self.BRhip = 1
        self.BLhip = 2
        self.FLhip = 3
        self.FRshoulder = 4
        self.BRknee = 5
        self.BLknee = 6
        self.FLknee = 7
        self.FRknee = 8
        self.BRshoulder = 9
        self.BLshoulder = 10
        self.FLshoulder = 11
        
                
        super().__init__('controller_to_servo')
        self.servo_controller = Controller()
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.logger = self.get_logger()
        self.is_walk_mode = False  # Initialize walk mode state
        self.last_command_time = self.get_clock().now()
        self.command_interval = 0.1  # seconds between commands

        # Define min and max positions for each servo
        self.servo_limits = [
            (1136*4, 1776*4),  # Servo 0
            (432*4, 1712*4),  # Servo 1
            (64*4, 1632*4),   # Servo 2
            (1664*4, 4080*4), # Servo 3
            (1408*4, 3968*4), # Servo 4
            (800*4, 1904*4),  # Servo 5
            (64*4, 2304*4),  # Servo 6
            (848*4, 1904*4),  # Servo 7
            (64*4, 2192*4),  # Servo 8
            (896*4, 2128*4),   # Servo 9
            (2000*4, 2608*4), # Servo 10
            (1616*4, 2608*4)  # Servo 11
        ]
        
        # Define home positions for each servo (example values)
        self.home_positions = [
            1414*4,  # Servo 0
            432*4,   # Servo 1
            687*4,  # Servo 2
            2270*4,  # Servo 3
            2121*4,  # Servo 4
            800*4,  # Servo 5
            2137*4,  # Servo 6
            1466*4,  # Servo 7
            1642*4,  # Servo 8
            1390*4,  # Servo 9
            2592*4,  # Servo 10
            2128*4   # Servo 11
        ]
        self.degree_to_servo_scale = 90  # Assuming each servo moves 90 degrees

        self.move_to_home()

    def joy_callback(self, msg):
        if any(msg.buttons):  # Check if any button is pressed
            if msg.buttons[0] == 1:
                self.move_to_home()
            elif msg.buttons[1] == 1:
                self.is_walk_mode = not self.is_walk_mode
                self.logger.info(f"Walk mode {'enabled' if self.is_walk_mode else 'disabled'}")
            
            if self.is_walk_mode and msg.axes[1] > 0:
                self.start_walking()
            else:
                self.update_servo_positions(msg)
        else:
            # If no buttons are pressed, move to home positions
            self.move_to_home()
    def degrees_to_servo_position(self, degrees, index):
        min_val, max_val = self.servo_limits[index]
        # Normalize degrees based on the servo's range
        servo_position = min_val + (degrees / self.degree_to_servo_scale) * (max_val - min_val)
        return int(servo_position)

    def update_servo_positions(self, degrees_list):
        positions = []
        for index, degrees in enumerate(degrees_list):
            position = self.degrees_to_servo_position(degrees, index)
            positions.append(position)
            self.servo_controller.setTarget(index, position)
        self.logger.info(f"Updated positions: {positions}")


    def start_walking(self):
        # Add your walking functionality
        self.logger.info("Walking...")

    def move_to_home(self):
        for index, home_pos in enumerate(self.home_positions):
            self.servo_controller.setTarget(index, int(home_pos))
            self.logger.info(f"Moving Servo {index} to home position {home_pos}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerToServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
