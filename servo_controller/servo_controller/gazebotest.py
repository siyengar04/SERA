import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class LegController(Node):
    def __init__(self, a):
        super().__init__("leg_controller")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.a = a
        self.timer = self.create_timer(0.1, self.move_leg)
        self.movements = [
            (100, 100, -100),
            (50, 150, -50),
            (0, 200, 0),
            (-50, 150, 50),
            (-100, 100, 100),
        ]
        self.current_movement_index = 0

    def to_rad(self, deg):
        return deg * np.pi / 180

    def get_theta1(self, X, Y):
        theta = np.arctan2(Y, X)
        return np.clip(theta, np.radians(-60), np.radians(60))

    def get_theta2(self, r, a):
        try:
            theta = np.arccos((2 * a**2 - r**2) / (2 * a**2))
        except ValueError:
            theta = 0
        return np.clip(theta, np.radians(0), np.radians(135))

    def get_theta3(self, Z, r, a, theta1, theta2):
        theta = np.arctan2(Z, r) - theta1 - theta2
        return np.clip(theta, np.radians(-45), np.radians(45))

    def move_leg(self):
        if self.current_movement_index >= len(self.movements):
            return

        X, Y, Z = self.movements[self.current_movement_index]
        self.current_movement_index += 1

        theta1 = self.get_theta1(X, Y)
        r = np.sqrt(X**2 + Y**2)
        if r > 2 * self.a:
            return  # Skip this point if it's out of the maximum reach
        theta2 = self.get_theta2(r, self.a)
        theta3 = self.get_theta3(Z, r, self.a, theta1, theta2)

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["hip_fl", "shoulder_fl", "knee_fl"]
        joint_state.position = [theta1, theta2, theta3]

        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    leg_controller = LegController(a=110)
    rclpy.spin(leg_controller)
    leg_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
