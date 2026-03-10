"""
Subscribes to /cmd_vel and drives the VESC.
"""

import traceback

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_twist_node'
TOPIC_NAME = '/cmd_vel'


class VescTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vesc = VESC_()

        self.create_subscription(Twist, TOPIC_NAME, self.callback, 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_potential_rpm', 20000),
                ('steering_polarity', 1),
                ('throttle_polarity', 1),
                ('max_right_steering', 0.792),
                ('straight_steering', 0.0),
                ('max_left_steering', -0.831),
                ('zero_throttle', -0.032),
                ('max_throttle', 0.382),
                ('min_throttle', 0.363),
            ],
        )

        max_rpm_base = int(self.get_parameter('max_potential_rpm').value)
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)
        self.max_right_steering = float(self.get_parameter('max_right_steering').value)
        self.straight_steering = float(self.get_parameter('straight_steering').value)
        self.max_left_steering = float(self.get_parameter('max_left_steering').value)
        zero_throttle = float(self.get_parameter('zero_throttle').value)
        max_throttle = float(self.get_parameter('max_throttle').value)
        min_throttle = float(self.get_parameter('min_throttle').value)

        self.zero_rpm = int(zero_throttle * max_rpm_base)
        self.max_rpm = int(max_throttle * max_rpm_base)
        self.min_rpm = int(min_throttle * max_rpm_base)

        self.get_logger().info(
            f'\n{NODE_NAME} ready'
            f'\n  max_rpm (scaled): {self.max_rpm}'
            f'\n  min_rpm (scaled): {self.min_rpm}'
            f'\n  zero_rpm (offset): {self.zero_rpm}'
            f'\n  steering_polarity: {self.steering_polarity}'
            f'\n  throttle_polarity: {self.throttle_polarity}'
            f'\n  right/straight/left: '
            f'{self.max_right_steering:.3f}/{self.straight_steering:.3f}/{self.max_left_steering:.3f}'
        )

    def callback(self, msg: Twist):
        steering_angle = self._clamp(
            self._map_steering(self.steering_polarity * msg.angular.z), 1.0, 0.0
        )

        if msg.linear.x <= 0.0:
            rpm = self.zero_rpm
        else:
            rpm = int(self.max_rpm * msg.linear.x)
            rpm = max(rpm, self.min_rpm)

        self.vesc.send_rpm(int(self.throttle_polarity * rpm))
        self.vesc.send_servo_angle(float(steering_angle))

    @staticmethod
    def _lerp(start: float, end: float, alpha: float) -> float:
        return start + (end - start) * alpha

    def _map_steering(self, angular_z: float) -> float:
        """Map angular_z in [-1, 1] to servo position in [0, 1].

        Steering parameters (straight_steering, max_right_steering, max_left_steering)
        are in the [-1, 1] command space. Output is converted to [0, 1] servo space
        via (servo_cmd + 1) / 2, matching the reference calibration convention.
        """
        angular_z = self._clamp(float(angular_z), 1.0)
        if angular_z >= self.straight_steering:
            alpha = (angular_z - self.straight_steering) / max(
                1.0 - self.straight_steering, 1e-6
            )
            servo_cmd = self._lerp(self.straight_steering, self.max_right_steering, alpha)
        else:
            alpha = (self.straight_steering - angular_z) / max(
                self.straight_steering + 1.0, 1e-6
            )
            servo_cmd = self._lerp(self.straight_steering, self.max_left_steering, alpha)
        return (servo_cmd + 1.0) / 2.0

    @staticmethod
    def _clamp(value: float, upper: float, lower: float = None) -> float:
        if lower is None:
            lower = -upper
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VescTwist()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
