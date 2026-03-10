"""
PID controller that steers toward the tracked subject's bounding-box center and
uses bounding-box size as a distance proxy for throttle.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

NODE_NAME = 'tracker_pid_node'
ERROR_TOPIC = '/tracking_error'
CMD_VEL_TOPIC = '/cmd_vel'


class TrackerPidNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 0.8),
                ('Ki_steering', 0.0),
                ('Kd_steering', 0.10),
                ('integral_max', 0.5),
                ('Kp_throttle', 0.5),
                ('Ki_throttle', 0.0),
                ('Kd_throttle', 0.05),
                ('max_speed', 0.50),
                ('min_speed', 0.10),
                ('max_steering', 1.0),
                ('lateral_dead_band', 0.05),
                ('distance_dead_band', 0.05),
                ('Ts', 0.10),
            ],
        )

        self.Kp_s = float(self.get_parameter('Kp_steering').value)
        self.Ki_s = float(self.get_parameter('Ki_steering').value)
        self.Kd_s = float(self.get_parameter('Kd_steering').value)
        self.integral_max = float(self.get_parameter('integral_max').value)
        self.Kp_t = float(self.get_parameter('Kp_throttle').value)
        self.Ki_t = float(self.get_parameter('Ki_throttle').value)
        self.Kd_t = float(self.get_parameter('Kd_throttle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.lat_db = float(self.get_parameter('lateral_dead_band').value)
        self.dist_db = float(self.get_parameter('distance_dead_band').value)
        self.Ts = float(self.get_parameter('Ts').value)

        self.create_subscription(Float32MultiArray, ERROR_TOPIC, self._error_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._e_lat = 0.0
        self._e_dist = 0.0
        self._is_tracking = False
        self._e_lat_prev = 0.0
        self._e_dist_prev = 0.0
        self._int_steer = 0.0
        self._int_throt = 0.0

        self.create_timer(self.Ts, self._control_loop)
        self.get_logger().info(
            f'{NODE_NAME} started\n'
            f'  Steering PID : Kp={self.Kp_s} Ki={self.Ki_s} Kd={self.Kd_s}\n'
            f'  Throttle PID : Kp={self.Kp_t} Ki={self.Ki_t} Kd={self.Kd_t}\n'
            f'  Sample time  : {self.Ts} s'
        )

    def _error_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        self._e_lat = float(msg.data[0])
        self._e_dist = float(msg.data[1])
        self._is_tracking = msg.data[3] > 0.5

    def _control_loop(self):
        cmd = Twist()
        if not self._is_tracking:
            self._int_steer = 0.0
            self._int_throt = 0.0
            self._e_lat_prev = 0.0
            self._e_dist_prev = 0.0
            self.cmd_pub.publish(cmd)
            return

        if abs(self._e_lat) > self.lat_db:
            prop_s = self.Kp_s * self._e_lat
            deriv_s = self.Kd_s * (self._e_lat - self._e_lat_prev) / self.Ts
            self._int_steer += self.Ki_s * self._e_lat * self.Ts
            self._int_steer = self._clamp(self._int_steer, self.integral_max)
            steer_raw = prop_s + deriv_s + self._int_steer
        else:
            steer_raw = 0.0
            self._int_steer = 0.0

        if abs(self._e_dist) > self.dist_db:
            prop_t = self.Kp_t * self._e_dist
            deriv_t = self.Kd_t * (self._e_dist - self._e_dist_prev) / self.Ts
            self._int_throt += self.Ki_t * self._e_dist * self.Ts
            self._int_throt = self._clamp(self._int_throt, self.integral_max)
            throt_raw = prop_t + deriv_t + self._int_throt
        else:
            throt_raw = 0.0
            self._int_throt = 0.0

        self._e_lat_prev = self._e_lat
        self._e_dist_prev = self._e_dist

        steering = self._clamp(steer_raw, self.max_steering)
        throttle = self._clamp(throt_raw, self.max_speed)
        if throttle < 0.0:
            throttle = 0.0
        if 0.0 < throttle < self.min_speed:
            throttle = self.min_speed

        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _clamp(value: float, upper: float, lower: float = None) -> float:
        if lower is None:
            lower = -upper
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = TrackerPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
